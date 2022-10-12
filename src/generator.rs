use crate::{
    parser::{self, ArrayInfo, Expr, TypeName, ValueType},
    DynError, SafeDrive,
};
use convert_case::{Case, Casing};
use nom::{error::convert_error, Finish};
use std::{
    borrow::Cow,
    collections::{BTreeSet, VecDeque},
    fs::File,
    io::{Read, Write},
    path::{Path, PathBuf},
};

#[derive(Debug, Clone)]
pub struct Generator<'a> {
    msgs: BTreeSet<String>,
    srvs: BTreeSet<String>,
    pub(crate) dependencies: BTreeSet<String>,
    safe_drive: SafeDrive<'a>,
}

impl<'a> Generator<'a> {
    pub fn new(safe_drive: SafeDrive<'a>) -> Self {
        Self {
            msgs: Default::default(),
            srvs: Default::default(),
            dependencies: Default::default(),
            safe_drive,
        }
    }

    pub fn generate(&mut self, out_dir: &Path, path: &Path, lib: &str) -> Result<(), DynError> {
        // Read checksum.
        let cksum = checksumdir::checksumdir(path.to_str().unwrap())?;
        let cksum_file = out_dir.join(lib).join("cksum");
        if let Ok(mut f) = File::open(&cksum_file) {
            let mut buf = String::new();
            f.read_to_string(&mut buf)?;

            if buf == cksum {
                return Ok(()); // already generated and it is the latest
            }
        }

        self.generate_recursive(out_dir, path, lib)?;
        self.generate_crate(out_dir, lib)?;

        // Write checksum.
        if let Ok(mut f) = File::create(cksum_file) {
            let _ = f.write(cksum.as_bytes());
        }

        Ok(())
    }

    fn generate_crate(&self, out_dir: &Path, lib: &str) -> Result<(), DynError> {
        if self.msgs.is_empty() && self.srvs.is_empty() {
            return Ok(());
        }

        // lib.rs, msg.rs
        let src_dir = out_dir.join(lib).join("src");
        let mut lib_file = File::create(src_dir.join("lib.rs")).unwrap();

        if !self.msgs.is_empty() {
            lib_file.write_fmt(format_args!("pub mod msg;\n"))?;

            let mut msg_file = File::create(src_dir.join("msg.rs"))?;
            for msg in self.msgs.iter() {
                msg_file.write_fmt(format_args!("pub mod {msg};\n"))?;
                msg_file.write_fmt(format_args!("pub use {msg}::*;\n\n"))?;
            }
        }

        if !self.srvs.is_empty() {
            lib_file.write_fmt(format_args!("pub mod srv;\n"))?;

            let mut srv_file = File::create(src_dir.join("srv.rs"))?;
            for srv in self.srvs.iter() {
                srv_file.write_fmt(format_args!("pub mod {srv};\n"))?;
                srv_file.write_fmt(format_args!("pub use {srv}::*;\n\n"))?;
            }
        }

        // Cargo.toml
        self.generate_cargo_toml(out_dir, lib)?;

        // build.rs
        self.generate_build_rs(out_dir, lib)?;

        Ok(())
    }

    fn generate_build_rs(&self, out_dir: &Path, lib: &str) -> Result<(), DynError> {
        let mut build_rs = File::create(out_dir.join(lib).join("build.rs"))?;

        build_rs.write_fmt(format_args!(
            r#"fn main() {{
    println!("cargo:rustc-link-lib={lib}__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib={lib}__rosidl_generator_c");

    if let Some(e) = std::env::var_os("AMENT_PREFIX_PATH") {{
        let env = e.to_str().unwrap();
        for path in env.split(':') {{
            println!("cargo:rustc-link-search={{path}}/lib");
        }}
    }}
}}
"#
        ))?;

        Ok(())
    }

    fn generate_cargo_toml(&self, out_dir: &Path, lib: &str) -> Result<(), DynError> {
        let ros2ver = std::env::var("ROS_DISTRO")?;

        let safe_drive_dep = match &self.safe_drive {
            SafeDrive::Path(s) => {
                format!(
                    "{{ path = \"{s}\", default-features = false, features = [\"{ros2ver}\"] }}"
                )
            }
            SafeDrive::Version(s) => format!(
                "{{ version = \"{s}\", default-features = false, features = [\"{ros2ver}\"] }}"
            ),
        };

        let mut cargo_toml = File::create(out_dir.join(lib).join("Cargo.toml"))?;
        cargo_toml.write_fmt(format_args!(
            r#"[package]
name = "{lib}"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies]
safe_drive = {safe_drive_dep}
"#,
        ))?;

        for dependency in self.dependencies.iter() {
            let path = out_dir.join(dependency);
            cargo_toml.write_fmt(format_args!(
                r#"{dependency} = {{ path = "{}" }}
"#,
                path.to_str().unwrap()
            ))?;
        }

        Ok(())
    }

    fn generate_recursive(
        &mut self,
        out_dir: &Path,
        path: &Path,
        lib: &str,
    ) -> Result<(), DynError> {
        if let Ok(metadata) = path.metadata() {
            if metadata.is_dir() {
                // directory
                for file in std::fs::read_dir(path)? {
                    if let Ok(file) = file {
                        self.generate_recursive(out_dir, &file.path(), lib)?;
                    }
                }
            } else if metadata.is_file() {
                // file
                if let Some(ext) = path.extension() {
                    match ext.to_str() {
                        Some("msg") => self.generate_msg(out_dir, path, lib)?,
                        Some("srv") => self.generate_srv(out_dir, path, lib)?,
                        _ => (),
                    }
                }
            } else if metadata.is_symlink() {
                let path = std::fs::read_link(path)?;
                self.generate_recursive(out_dir, &path, lib)?;
            }
        }

        Ok(())
    }

    fn generate_msg(&mut self, out_lib_dir: &Path, path: &Path, lib: &str) -> Result<(), DynError> {
        // Read.
        let contents = read_file(path)?;

        // Parse.
        let exprs = match parser::parse_msg(&contents).finish() {
            Ok((_, exprs)) => exprs,
            Err(e) => {
                eprintln!("{}", convert_error(contents.as_str(), e));
                let msg = format!("failed to parse: {}", path.display());
                return Err(msg.into());
            }
        };

        let (rs_module, rs_file, rs_type_name) = module_file_type(path)?;
        self.msgs.insert(rs_module.into());

        let mut lines = VecDeque::<Cow<'static, str>>::new();

        // extern "C"
        lines.push_back(gen_c_extern_msg(lib, &rs_type_name).into());

        self.generate_exprs(&exprs, &mut lines, lib, &rs_type_name, MsgType::Msg);

        lines.push_back(gen_impl_for_msg(lib, &rs_type_name).into());
        lines.push_front("use safe_drive::{msg::TopicMsg, rcl};".into());

        // Create a directory.
        let out_dir = out_lib_dir.join(lib).join("src").join("msg");
        std::fs::create_dir_all(&out_dir)?;

        // Write.
        let out_file = out_dir.join(&rs_file);
        let mut f = File::create(out_file)?;
        for line in lines {
            f.write_fmt(format_args!("{}\n", line))?;
        }

        Ok(())
    }

    fn generate_srv(&mut self, out_lib_dir: &Path, path: &Path, lib: &str) -> Result<(), DynError> {
        // Read.
        let contents = read_file(path)?;

        // Parse.
        let (exprs_req, exprs_resp) = match parser::parse_srv(&contents).finish() {
            Ok((_, exprs)) => exprs,
            Err(e) => {
                eprintln!("{}", convert_error(contents.as_str(), e));
                let msg = format!("failed to parse: {}", path.display());
                return Err(msg.into());
            }
        };

        let (rs_module, rs_file, rs_type_name) = module_file_type(path)?;
        self.srvs.insert(rs_module.into());

        let mut lines = VecDeque::new();

        // extern "C"
        lines.push_back(gen_c_extern_srv(lib, &rs_type_name).into());

        self.generate_exprs(
            &exprs_req,
            &mut lines,
            lib,
            &rs_type_name,
            MsgType::SrvRequest,
        );

        self.generate_exprs(
            &exprs_resp,
            &mut lines,
            lib,
            &rs_type_name,
            MsgType::SrvResponse,
        );

        lines.push_back(gen_impl_for_srv(lib, &rs_type_name).into());
        lines.push_front("use safe_drive::{msg::ServiceMsg, rcl};".into());

        // Create a directory.
        let out_dir = out_lib_dir.join(lib).join("src").join("srv");
        std::fs::create_dir_all(&out_dir)?;

        // Write.
        let out_file = out_dir.join(&rs_file);
        let mut f = File::create(out_file)?;
        for line in lines {
            f.write_fmt(format_args!("{}\n", line))?;
        }

        Ok(())
    }

    fn generate_var(&mut self, type_name: &TypeName, var_name: &str) -> String {
        match type_name {
            TypeName::Type {
                type_name,
                array_info,
            } => match array_info {
                ArrayInfo::NotArray => {
                    format!("{var_name}: {}", gen_type(type_name.as_str()))
                }
                ArrayInfo::Dynamic => {
                    let type_name = gen_seq_type(&type_name, 0);
                    format!("{var_name}: {type_name}")
                }
                ArrayInfo::Limited(size) => {
                    let type_name = gen_seq_type(&type_name, *size);
                    format!("{var_name}: {type_name}")
                }
                ArrayInfo::Static(size) => {
                    format!("{var_name}: [{type_name}; {size}]")
                }
            },
            TypeName::ScopedType {
                scope,
                type_name,
                array_info,
            } => {
                self.dependencies.insert(scope.clone());

                match array_info {
                    ArrayInfo::NotArray => {
                        format!("{var_name}: {scope}::msg::{type_name}")
                    }
                    ArrayInfo::Dynamic => {
                        let type_name = gen_seq_type(&type_name, 0);
                        format!("{var_name}: {scope}::msg::{type_name}")
                    }
                    ArrayInfo::Limited(size) => {
                        let type_name = gen_seq_type(&type_name, *size);
                        format!("{var_name}: {scope}::msg::{type_name}")
                    }
                    ArrayInfo::Static(size) => {
                        format!("{var_name}: [{scope}::msg::{type_name}; {size}]")
                    }
                }
            }
            TypeName::LimitedString {
                size: str_len,
                array_info,
            } => match array_info {
                ArrayInfo::NotArray => {
                    format!("{var_name}: safe_drive::msg::RosString<{str_len}>")
                }
                ArrayInfo::Dynamic => {
                    format!("{var_name}: safe_drive::msg::RosStringSeq<{str_len}, 0>")
                }
                ArrayInfo::Limited(size) => {
                    format!("{var_name}: safe_drive::msg::RosStringSeq<{str_len}, {size}>")
                }
                ArrayInfo::Static(size) => {
                    format!("{var_name}: [safe_drive::msg::RosString<{str_len}>; {size}]")
                }
            },
            TypeName::String(array_info) => match array_info {
                ArrayInfo::NotArray => {
                    format!("{var_name}: safe_drive::msg::RosString<0>")
                }
                ArrayInfo::Dynamic => {
                    format!("{var_name}: safe_drive::msg::RosStringSeq<0, 0>")
                }
                ArrayInfo::Limited(size) => {
                    format!("{var_name}: safe_drive::msg::RosStringSeq<0, {size}>")
                }
                ArrayInfo::Static(size) => {
                    format!("{var_name}: [safe_drive::msg::RosString<0>; {size}]")
                }
            },
        }
    }

    fn generate_struct(
        &mut self,
        exprs: &[Expr],
        struct_name: &str,
        lines: &mut VecDeque<Cow<'static, str>>,
    ) {
        lines.push_back("#[repr(C)]".into());
        lines.push_back("#[derive(Debug)]".into());
        lines.push_back(format!("pub struct {struct_name} {{").into());

        let mut num_member = 0;

        for expr in exprs.iter() {
            match expr {
                Expr::Variable {
                    type_name,
                    var_name,
                    value: Some(ValueType::Const(const_value)),
                    comment: _,
                } => {
                    let var = self.generate_var(type_name, var_name);
                    lines.push_front(format!("const {var} = {const_value};").into());
                }
                Expr::Variable {
                    type_name,
                    var_name,
                    ..
                } => {
                    let var = self.generate_var(type_name, var_name);
                    lines.push_back(format!("    {var},").into());
                    num_member += 1;
                }
                _ => (),
            };
        }

        if num_member == 0 {
            lines.push_back("    _structure_needs_at_least_one_member: u8,".into());
        }

        lines.push_back("}".into());
    }

    fn generate_exprs(
        &mut self,
        exprs: &[Expr],
        lines: &mut VecDeque<Cow<'static, str>>,
        lib: &str,
        rs_type_name: &str,
        msg_type: MsgType,
    ) {
        let struct_name = match &msg_type {
            MsgType::Msg => rs_type_name.to_string(),
            MsgType::SrvRequest => format!("{rs_type_name}Request"),
            MsgType::SrvResponse => format!("{rs_type_name}Response"),
        };

        self.generate_struct(exprs, &struct_name, lines);

        lines.push_back(gen_impl(lib, &rs_type_name, msg_type).into());
    }
}

fn read_file(path: &Path) -> Result<String, DynError> {
    // Read.
    let contents = {
        let mut f = File::open(path)?;
        let mut c = String::new();
        f.read_to_string(&mut c)?;
        c
    };
    Ok(contents)
}

fn module_file_type(path: &Path) -> Result<(String, PathBuf, String), DynError> {
    // "{CamelFileName}.msg" to "{snake_file_name}.rs"
    let names: Vec<_> = path
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .split(".")
        .collect();
    let camel_file_name = names[0];
    let module_name = camel_file_name.to_case(Case::Snake);
    let module_name = mangle(&module_name);

    // "{snake_file_name}.rs"
    let rs_file = format!("{module_name}.rs");
    let rs_file = Path::new(&rs_file);

    // "{CamelFileName}"
    let rs_type_name = camel_file_name.to_string();

    Ok((module_name.to_string(), rs_file.to_owned(), rs_type_name))
}

fn mangle(var_name: &str) -> Cow<'_, str> {
    match var_name {
        "type" | "pub" | "fn" | "match" | "if" | "while" | "break" | "continue" | "unsafe"
        | "async" | "move" | "trait" | "impl" | "for" | "i8" | "u8" | "i16" | "u16" | "i32"
        | "u32" | "i64" | "u64" | "bool" | "char" => format!("{var_name}_").into(),
        _ => var_name.into(),
    }
}

fn gen_seq_type(type_str: &str, size: usize) -> String {
    match type_str.as_ref() {
        "bool" => format!("safe_drive::msg::BoolSeq<{size}>"),
        "byte" | "uint8" => format!("safe_drive::msg::I8Seq<{size}>"),
        "int16" => format!("safe_drive::msg::I16Seq<{size}>"),
        "int32" => format!("safe_drive::msg::I32Seq<{size}>"),
        "int64" => format!("safe_drive::msg::I64Seq<{size}>"),
        "char" | "int8" => format!("safe_drive::msg::U8Seq<{size}>"),
        "uint16" => format!("safe_drive::msg::U16Seq<{size}>"),
        "uint32" => format!("safe_drive::msg::U32Seq<{size}>"),
        "uint64" => format!("safe_drive::msg::U64Seq<{size}>"),
        "float32" => format!("safe_drive::msg::F32Seq<{size}>"),
        "float64" => format!("safe_drive::msg::F64Seq<{size}>"),
        _ => {
            let mod_file = type_str.to_case(Case::Snake);
            let mod_file = mangle(&mod_file);
            format!("crate::msg::{mod_file}::{type_str}Seq<{size}>").into()
        }
    }
}

fn gen_type(type_str: &str) -> Cow<'_, str> {
    match type_str.as_ref() {
        "bool" => "bool".into(),
        "byte" | "uint8" => "u8".into(),
        "char" | "int8" => "i8".into(),
        "int16" => "i16".into(),
        "uint16" => "u16".into(),
        "int32" => "i32".into(),
        "uint32" => "u32".into(),
        "int64" => "i64".into(),
        "uint64" => "u64".into(),
        "float32" => "f32".into(),
        "float64" => "f64".into(),
        _ => {
            let mod_file = type_str.to_case(Case::Snake);
            let mod_file = mangle(&mod_file);
            format!("crate::msg::{mod_file}::{type_str}").into()
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum MsgType {
    Msg,
    SrvRequest,
    SrvResponse,
}

fn gen_impl(module_name: &str, type_name: &str, msg_type: MsgType) -> String {
    let (mid, c_func_mid, type_name_full) = match msg_type {
        MsgType::Msg => ("msg", "", format!("{type_name}")),
        MsgType::SrvRequest => ("srv", "_Request", format!("{type_name}Request")),
        MsgType::SrvResponse => ("srv", "_Response", format!("{type_name}Response")),
    };

    format!(
        "
impl {type_name_full} {{
    pub fn new() -> Option<Self> {{
        let mut msg: Self = unsafe {{ std::mem::MaybeUninit::zeroed().assume_init() }};
        if unsafe {{ {module_name}__{mid}__{type_name}{c_func_mid}__init(&mut msg) }} {{
            Some(msg)
        }} else {{
            None
        }}
    }}
}}

impl Drop for {type_name_full} {{
    fn drop(&mut self) {{
        unsafe {{ {module_name}__{mid}__{type_name}{c_func_mid}__fini(self) }};
    }}
}}

#[repr(C)]
#[derive(Debug)]
struct {type_name_full}SeqRaw {{
    data: *mut {type_name_full},
    size: usize,
    capacity: usize,
}}

/// Sequence of {type_name_full}.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct {type_name_full}Seq<const N: usize> {{
    data: *mut {type_name_full},
    size: usize,
    capacity: usize,
}}

impl<const N: usize> {type_name_full}Seq<N> {{
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {{
        if N != 0 && size >= N {{
            // the size exceeds in the maximum number
            return None;
        }}
        let mut msg: {type_name_full}SeqRaw = unsafe {{ std::mem::MaybeUninit::zeroed().assume_init() }};
        if unsafe {{ {module_name}__{mid}__{type_name}{c_func_mid}__Sequence__init(&mut msg, size) }} {{
            Some(Self {{data: msg.data, size: msg.size, capacity: msg.capacity }})
        }} else {{
            None
        }}
    }}

    pub fn null() -> Self {{
        let msg: {type_name_full}SeqRaw = unsafe {{ std::mem::MaybeUninit::zeroed().assume_init() }};
        Self {{data: msg.data, size: msg.size, capacity: msg.capacity }}
    }}

    pub fn as_slice(&self) -> &[{type_name_full}] {{
        if self.data.is_null() {{
            &[]
        }} else {{
            let s = unsafe {{ std::slice::from_raw_parts(self.data, self.size) }};
            s
        }}
    }}

    pub fn as_slice_mut(&mut self) -> &mut [{type_name_full}] {{
        if self.data.is_null() {{
            &mut []
        }} else {{
            let s = unsafe {{ std::slice::from_raw_parts_mut(self.data, self.size) }};
            s
        }}
    }}

    pub fn iter(&self) -> std::slice::Iter<'_, {type_name_full}> {{
        self.as_slice().iter()
    }}

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, {type_name_full}> {{
        self.as_slice_mut().iter_mut()
    }}

    pub fn len(&self) -> usize {{
        self.as_slice().len()
    }}

    pub fn is_empty(&self) -> bool {{
        self.len() == 0
    }}
}}

impl<const N: usize> Drop for {type_name_full}Seq<N> {{
    fn drop(&mut self) {{
        let mut msg = {type_name_full}SeqRaw{{data: self.data, size: self.size, capacity: self.capacity}};
        unsafe {{ {module_name}__{mid}__{type_name}{c_func_mid}__Sequence__fini(&mut msg) }};
    }}
}}

unsafe impl<const N: usize> Send for {type_name_full}Seq<N> {{}}
unsafe impl<const N: usize> Sync for {type_name_full}Seq<N> {{}}
")
}

fn gen_c_extern_msg(module_name: &str, type_name: &str) -> String {
    format!(
        "
extern \"C\" {{
    fn {module_name}__msg__{type_name}__init(msg: *mut {type_name}) -> bool;
    fn {module_name}__msg__{type_name}__fini(msg: *mut {type_name});
    fn {module_name}__msg__{type_name}__are_equal(lhs: *const {type_name}, rhs: *const {type_name}) -> bool;
    fn {module_name}__msg__{type_name}__Sequence__init(msg: *mut {type_name}SeqRaw, size: usize) -> bool;
    fn {module_name}__msg__{type_name}__Sequence__fini(msg: *mut {type_name}SeqRaw);
    fn {module_name}__msg__{type_name}__Sequence__are_equal(lhs: *const {type_name}SeqRaw, rhs: *const {type_name}SeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__{module_name}__msg__{type_name}() -> *const rcl::rosidl_message_type_support_t;
}}
"
    )
}

fn gen_impl_for_msg(module_name: &str, type_name: &str) -> String {
    // generate impl and struct of sequence
    format!(
        "impl TopicMsg for {type_name} {{
    fn type_support() -> *const rcl::rosidl_message_type_support_t {{
        unsafe {{
            rosidl_typesupport_c__get_message_type_support_handle__{module_name}__msg__{type_name}()
        }}
    }}
}}

impl PartialEq for {type_name} {{
    fn eq(&self, other: &Self) -> bool {{
        unsafe {{
            {module_name}__msg__{type_name}__are_equal(self, other)
        }}
    }}
}}

impl<const N: usize> PartialEq for {type_name}Seq<N> {{
    fn eq(&self, other: &Self) -> bool {{
        unsafe {{
            let msg1 = {type_name}SeqRaw{{data: self.data, size: self.size, capacity: self.capacity}};
            let msg2 = {type_name}SeqRaw{{data: other.data, size: other.size, capacity: other.capacity}};
            {module_name}__msg__{type_name}__Sequence__are_equal(&msg1, &msg2)
        }}
    }}
}}
"
    )
}

fn gen_impl_for_srv(module_name: &str, type_name: &str) -> String {
    format!(
        "
pub struct {type_name};

impl ServiceMsg for {type_name} {{
    type Request = {type_name}Request;
    type Response = {type_name}Response;
    fn type_support() -> *const rcl::rosidl_service_type_support_t {{
        unsafe {{
            rosidl_typesupport_c__get_service_type_support_handle__{module_name}__srv__{type_name}()
        }}
    }}
}}

impl PartialEq for {type_name}Request {{
    fn eq(&self, other: &Self) -> bool {{
        unsafe {{
            {module_name}__srv__{type_name}_Request__are_equal(self, other)
        }}
    }}
}}

impl PartialEq for {type_name}Response {{
    fn eq(&self, other: &Self) -> bool {{
        unsafe {{
            {module_name}__srv__{type_name}_Response__are_equal(self, other)
        }}
    }}
}}

impl<const N: usize> PartialEq for {type_name}RequestSeq<N> {{
    fn eq(&self, other: &Self) -> bool {{
        unsafe {{
            let msg1 = {type_name}RequestSeqRaw{{data: self.data, size: self.size, capacity: self.capacity}};
            let msg2 = {type_name}RequestSeqRaw{{data: other.data, size: other.size, capacity: other.capacity}};
            {module_name}__srv__{type_name}_Request__Sequence__are_equal(&msg1, &msg2)
        }}
    }}
}}

impl<const N: usize> PartialEq for {type_name}ResponseSeq<N> {{
    fn eq(&self, other: &Self) -> bool {{
        unsafe {{
            let msg1 = {type_name}ResponseSeqRaw{{data: self.data, size: self.size, capacity: self.capacity}};
            let msg2 = {type_name}ResponseSeqRaw{{data: other.data, size: other.size, capacity: other.capacity}};
            {module_name}__srv__{type_name}_Response__Sequence__are_equal(&msg1, &msg2)
        }}
    }}
}}
"
    )
}

fn gen_c_extern_srv(module_name: &str, type_name: &str) -> String {
    format!(
        "
extern \"C\" {{
    fn {module_name}__srv__{type_name}_Request__init(msg: *mut {type_name}Request) -> bool;
    fn {module_name}__srv__{type_name}_Request__fini(msg: *mut {type_name}Request);
    fn {module_name}__srv__{type_name}_Request__Sequence__init(msg: *mut {type_name}RequestSeqRaw, size: usize) -> bool;
    fn {module_name}__srv__{type_name}_Request__Sequence__fini(msg: *mut {type_name}RequestSeqRaw);
    fn {module_name}__srv__{type_name}_Response__init(msg: *mut {type_name}Response) -> bool;
    fn {module_name}__srv__{type_name}_Response__fini(msg: *mut {type_name}Response);
    fn {module_name}__srv__{type_name}_Response__Sequence__init(msg: *mut {type_name}ResponseSeqRaw, size: usize) -> bool;
    fn {module_name}__srv__{type_name}_Response__Sequence__fini(msg: *mut {type_name}ResponseSeqRaw);
    fn {module_name}__srv__{type_name}_Request__are_equal(lhs: *const {type_name}Request, rhs: *const {type_name}Request) -> bool;
    fn {module_name}__srv__{type_name}_Response__are_equal(lhs: *const {type_name}Response, rhs: *const {type_name}Response) -> bool;
    fn {module_name}__srv__{type_name}_Request__Sequence__are_equal(lhs: *const {type_name}RequestSeqRaw, rhs: *const {type_name}RequestSeqRaw) -> bool;
    fn {module_name}__srv__{type_name}_Response__Sequence__are_equal(lhs: *const {type_name}ResponseSeqRaw, rhs: *const {type_name}ResponseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_service_type_support_handle__{module_name}__srv__{type_name}() -> *const rcl::rosidl_service_type_support_t;
}}
"
    )
}
