use std::env;
use std::path::Path;
use std::process::Command;
use walkdir::WalkDir;

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();

    let src_files: Vec<_> = WalkDir::new("./src/cdd")
        .follow_links(false)
        .max_depth(5)
        .into_iter()
        .filter_map(|e| e.ok())
        .filter_map(|entry| {
            let f_name = entry.path().to_str().unwrap();

            if f_name.ends_with("c") {
                return Some(entry.path().to_owned());
            }
            return None;
        })
        .collect();
    println!("{:?}", src_files);
    let mut objs = Vec::new();
    for src in src_files {
        let temp = src.to_str().unwrap();
        let temp_name = src.file_stem().unwrap();
        objs.push(temp_name.to_str().unwrap().to_owned());
        Command::new("arm-none-eabi-gcc")
            .args(&[temp, "-c", "-fPIC", "-mcpu=cortex-r5", "-o"])
            .arg(&format!("{}/{}", out_dir, temp_name.to_str().unwrap()))
            .status()
            .unwrap();
    }

    Command::new("arm-none-eabi-ar")
        .args(&["crs", "libcffi.a"])
        .args(&objs)
        .current_dir(&Path::new(&out_dir))
        .status()
        .unwrap();

    println!("cargo:rustc-link-search=native={}", out_dir);
    println!("cargo:rustc-link-lib=static=cffi");
    println!("cargo:rerun-if-changed=src/cdd/");
}
