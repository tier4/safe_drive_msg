//! # Transpiler from ROS2's message types to Rust's types.
//!
//! See https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md

pub(crate) mod generator;
pub(crate) mod idl;
pub(crate) mod parser;

use std::{
    collections::BTreeSet,
    error::Error,
    path::{Path, PathBuf},
};

pub type DynError = Box<dyn Error + Send + Sync + 'static>;

#[cfg(target_os = "windows")]
const SEP: char = ';';
#[cfg(not(target_os = "windows"))]
const SEP: char = ':';

#[derive(Debug, Clone, Copy)]
pub enum SafeDrive<'a> {
    Path(&'a str),
    Version(&'a str),
}

/// Transpile ROS2's message types to Rust's types.
/// Dependencies will be automatically
///
/// # Example
///
/// ```
/// use safe_drive_msg;
/// use std::path::Path;
///
/// let dependencies = ["std_msgs", "std_srvs"];
/// safe_drive_msg::depends(&Path::new("/tmp/output_dir"), &dependencies, safe_drive_msg::SafeDrive::Version("0.1"));
/// ```
pub fn depends(outdir: &Path, libs: &[&str], safe_drive: SafeDrive) -> Result<(), DynError> {
    let ament_paths = std::env::var("AMENT_PREFIX_PATH")?;
    let mut ament_paths: Vec<_> = ament_paths
        .split(SEP)
        .filter(|&p| p.len() > 0)
        .map(|p| std::path::Path::new(p).join("share"))
        .collect();
    if cfg!(target_os = "windows") {
        let cmake_paths_env = std::env::var("CMAKE_PREFIX_PATH");
        if let Ok(cmake_paths) = cmake_paths_env {
            let cmake_paths = cmake_paths
                .split(SEP)
                .filter(|&p| p.len() > 0)
                .map(|p| std::path::Path::new(p).join("share"));
            ament_paths.extend(cmake_paths);
        }
    }
    let libs: BTreeSet<_> = libs.iter().map(|e| e.to_string()).collect();

    std::fs::create_dir_all(outdir)?;

    generate_libs(outdir, &ament_paths, &libs, safe_drive)?;
    Ok(())
}

fn generate_libs(
    outdir: &Path,
    ament_paths: &Vec<PathBuf>,
    libs: &BTreeSet<String>,
    safe_drive: SafeDrive,
) -> Result<(), DynError> {
    let libs: BTreeSet<_> = libs.iter().collect();

    let outdir = std::path::Path::new(outdir);
    std::fs::create_dir_all(outdir)?;

    'lib: for lib in libs.iter() {
        for path in ament_paths.iter() {
            let resource = path
                .join("ament_index")
                .join("resource_index")
                .join("packages")
                .join(lib);

            if resource.exists() {
                let path = path.join(lib);
                if path.exists() {
                    let mut gen = generator::Generator::new(safe_drive);
                    gen.generate(outdir, &path, lib)?;

                    // Generate dependencies.
                    generate_libs(outdir, ament_paths, &gen.dependencies, safe_drive)?;

                    continue 'lib;
                } else {
                    eprintln!(
                        "{lib} is not found in {}",
                        path.to_str().unwrap_or_default()
                    );
                }
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn std_msgs() {
        depends(
            Path::new("/tmp/safe_drive_msg"),
            &["std_msgs"],
            SafeDrive::Version("0.2"),
        )
        .unwrap();
    }

    #[test]
    fn std_srvs() {
        depends(
            Path::new("/tmp/safe_drive_msg"),
            &["std_srvs"],
            SafeDrive::Version("0.2"),
        )
        .unwrap();
    }

    #[test]
    fn action_tutorials_interfaces() {
        depends(
            Path::new("/tmp/safe_drive_msg"),
            &["action_tutorials_interfaces"],
            SafeDrive::Version("0.2"),
        )
        .unwrap();
    }
}
