use crate::{
    parser::{self, ArrayInfo, Expr, TypeName},
    DynError, SafeDrive,
};
use convert_case::{Case, Casing};
use nom::{error::convert_error, Finish};
use std::{
    borrow::Cow,
    collections::{BTreeSet, VecDeque},
    fs::File,
    io::{Read, Write},
    path::Path,
};

#[derive(Debug, Clone)]
pub struct Generator<'a> {
    msgs: BTreeSet<String>,
    srvs: BTreeSet<String>,
    safe_drive: SafeDrive<'a>,
}

impl<'a> Generator<'a> {
    pub fn new(safe_drive: SafeDrive<'a>) -> Self {
        Self {
            msgs: Default::default(),
            srvs: Default::default(),
            safe_drive,
        }
    }

    pub fn generate(&mut self, out_dir: &Path, path: &Path, lib: &str) -> Result<(), DynError> {
        self.generate_depth(out_dir, path, lib)?;
        self.generate_crate(out_dir, lib)?;

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

        // Cargo.toml
        let safe_drive_dep = match &self.safe_drive {
            SafeDrive::Path(s) => format!("{{ path = \"{s}\" }}"),
            SafeDrive::Version(s) => format!(r#""{s}""#),
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

        Ok(())
    }

    fn generate_depth(&mut self, out_dir: &Path, path: &Path, lib: &str) -> Result<(), DynError> {
        if let Ok(metadata) = path.metadata() {
            if metadata.is_dir() {
                // directory
                for file in std::fs::read_dir(path)? {
                    if let Ok(file) = file {
                        self.generate_depth(out_dir, &file.path(), lib)?;
                    }
                }
            } else if metadata.is_file() {
                // file
                if let Some(ext) = path.extension() {
                    match ext.to_str() {
                        Some("msg") => {
                            self.generate_msg(out_dir, path, lib)?;
                        }
                        Some("srv") => todo!(),
                        _ => (),
                    }
                }
            } else if metadata.is_symlink() {
                todo!()
            }
        }

        Ok(())
    }

    fn generate_msg(&mut self, out_lib_dir: &Path, path: &Path, lib: &str) -> Result<(), DynError> {
        // TODO: check already generated

        // Read.
        let contents = {
            let mut f = File::open(path)?;
            let mut c = String::new();
            f.read_to_string(&mut c)?;
            c
        };

        // Parse.
        let exprs = match parser::parse_msg(&contents).finish() {
            Ok((_, exprs)) => exprs,
            Err(e) => {
                eprintln!("{}", convert_error(contents.as_str(), e));
                let msg = format!("failed to parse: {}", path.display());
                return Err(msg.into());
            }
        };

        // "{CamelFileName}.msg" to "{snake_file_name}.rs"
        let names: Vec<_> = path
            .file_name()
            .unwrap()
            .to_str()
            .unwrap()
            .split(".")
            .collect();
        let camel_file_name = names[0];
        let snake_file_name = camel_file_name.to_case(Case::Snake);
        let snake_file_name = mangle(&snake_file_name);

        // "{snake_file_name}.rs"
        let rs_file_name = format!("{snake_file_name}.rs");
        let rs_file_name = Path::new(&rs_file_name);

        self.msgs.insert(snake_file_name.into());

        // "{CamelFileName}"
        let rs_type_name = camel_file_name.to_string();

        let out_dir = out_lib_dir.join(lib).join("src").join("msg");

        // Create a directory.
        std::fs::create_dir_all(&out_dir)?;

        let out_file = out_dir.join(rs_file_name);

        let mut lines = VecDeque::<Cow<'static, str>>::new();

        lines.push_back(gen_c_extern_msg(lib, &rs_type_name).into());

        lines.push_back("#[repr(C)]".into());
        lines.push_back("#[derive(Debug)]".into());
        lines.push_back(format!("pub struct {rs_type_name} {{").into());

        if exprs.is_empty() {
            lines.push_back("    _structure_needs_at_least_one_member: u8;".into());
        } else {
            for expr in exprs {
                gen_member(&expr, &mut lines);
            }
        }

        lines.push_back("}".into());
        lines.push_back(gen_impl(lib, &rs_type_name, MsgType::Msg).into());
        lines.push_back(gen_impl_for_msg(lib, &rs_type_name).into());

        let mut f = File::create(out_file)?;
        for line in lines {
            f.write_fmt(format_args!("{}\n", line))?;
        }

        Ok(())
    }
}

fn mangle(var_name: &str) -> Cow<'_, str> {
    match var_name {
        "type" | "pub" | "fn" | "match" | "if" | "while" | "break" | "continue" | "unsafe"
        | "async" | "move" | "trait" | "impl" | "for" | "i8" | "u8" | "i16" | "u16" | "i32"
        | "u32" | "i64" | "u64" | "bool" | "char" => format!("{var_name}_").into(),
        _ => var_name.into(),
    }
}

fn gen_member(expr: &Expr, lines: &mut VecDeque<Cow<'static, str>>) {
    if let Expr::Variable {
        type_name,
        var_name,
        value: _,
        comment: _,
    } = expr
    {
        let line = match type_name {
            TypeName::Type {
                type_name,
                array_info,
            } => match array_info {
                ArrayInfo::NotArray => {
                    format!("    {var_name}: {};", gen_type(type_name.as_str()))
                }
                ArrayInfo::Dynamic => {
                    let type_name = gen_seq_type(&type_name, 0);
                    format!("    {var_name}: {type_name};")
                }
                ArrayInfo::Limited(size) => {
                    let type_name = gen_seq_type(&type_name, *size);
                    format!("    {var_name}: {type_name};")
                }
                ArrayInfo::Static(size) => format!("    {var_name}: [{type_name}; {size}];"),
            },
            TypeName::ScopedType {
                scope,
                type_name,
                array_info,
            } => match array_info {
                ArrayInfo::NotArray => format!("    {var_name}: {scope}::msg::{type_name};"),
                ArrayInfo::Dynamic => {
                    let type_name = gen_seq_type(&type_name, 0);
                    format!("    {var_name}: {scope}::msg::{type_name};")
                }
                ArrayInfo::Limited(size) => {
                    let type_name = gen_seq_type(&type_name, *size);
                    format!("    {var_name}: {scope}::msg::{type_name};")
                }
                ArrayInfo::Static(size) => format!("    {var_name}: [{type_name}; {size}];"),
            },
            TypeName::LimitedString {
                size: str_len,
                array_info,
            } => match array_info {
                ArrayInfo::NotArray => {
                    format!("    {var_name}: safe_drive::msg::RosString<{str_len}>;")
                }
                ArrayInfo::Dynamic => {
                    format!("    {var_name}: safe_drive::msg::RosStringSeq<{str_len}, 0>;")
                }
                ArrayInfo::Limited(size) => {
                    format!("    {var_name}: safe_drive::msg::RosStringSeq<{str_len}, {size}>;")
                }
                ArrayInfo::Static(size) => {
                    format!("    {var_name}: [safe_drive::msg::RosString<{str_len}>; {size}];")
                }
            },
            TypeName::String(array_info) => match array_info {
                ArrayInfo::NotArray => {
                    format!("    {var_name}: safe_drive::msg::RosString<0>;")
                }
                ArrayInfo::Dynamic => {
                    format!("    {var_name}: safe_drive::msg::RosStringSeq<0, 0>;")
                }
                ArrayInfo::Limited(size) => {
                    format!("    {var_name}: safe_drive::msg::RosStringSeq<0, {size}>;")
                }
                ArrayInfo::Static(size) => {
                    format!("    {var_name}: [safe_drive::msg::RosString<0>; {size}];")
                }
            },
        };

        lines.push_back(line.into());
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
        _ => format!("{type_str}Seq<{size}>"),
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
            format!("super::{mod_file}::{type_str}").into()
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum MsgType {
    Msg,
    SrvRequest,
    SrvResonse,
}

fn gen_impl(module_name: &str, type_name: &str, msg_type: MsgType) -> String {
    let (mid, c_func_mid, type_name_full) = match msg_type {
        MsgType::Msg => ("msg", "", format!("{type_name}")),
        MsgType::SrvRequest => ("srv", "_Request", format!("{type_name}Request")),
        MsgType::SrvResonse => ("srv", "_Response", format!("{type_name}Response")),
    };

    format!(
        "
impl {type_name_full} {{
    pub fn new() -> Self {{
        let mut msg: Self = unsafe {{ std::mem::MaybeUninit::zeroed().assume_init() }};
        unsafe {{ {module_name}__{mid}__{type_name}{c_func_mid}__init(&mut msg) }};
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
        "extern \"C\" {{
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
