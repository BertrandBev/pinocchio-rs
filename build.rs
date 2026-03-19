use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let cpp_dir = manifest_dir.join("cpp");

    // Build the Cpp lib
    let xmake_status = Command::new("xmake")
        .current_dir(&cpp_dir)
        .status()
        .expect("xmake cmd not found");
    assert!(xmake_status.success(), "xmake build failed");
    let include_dirs = get_include_dirs(&cpp_dir);

    // Compile the bridge
    cxx_build::bridge("src/lib.rs")
        .include(&cpp_dir.join("src"))
        .includes(include_dirs)
        .std("c++17")
        .flag_if_supported("-O3")
        .compile("pinocchio_bridge_cxx");

    // Link
    let lib_dir = cpp_dir.join("build").join("macosx/arm64/release");
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=static=pinocchio_bridge");

    // Link C++ runtime
    let target = env::var("TARGET").unwrap();
    if target.contains("apple") {
        println!("cargo:rustc-link-lib=c++");
    } else {
        println!("cargo:rustc-link-lib=stdc++");
    }

    // // Re-run if any C++ source or header changes
    println!("cargo:rerun-if-changed=cpp/src/pinocchio_bridge.cpp");
    println!("cargo:rerun-if-changed=cpp/src/pinocchio_bridge.h");
    println!("cargo:rerun-if-changed=cpp/xmake.lua");
}

fn get_include_dirs(cpp_dir: &PathBuf) -> Vec<String> {
    let output = Command::new("xmake")
        .args(["show", "-t", "pinocchio_bridge", "--json"])
        .current_dir(&cpp_dir)
        .output()
        .expect("xmake show failed");
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    json["sysincludedirs"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|obj| obj["value"].as_str().map(String::from))
        .collect()
}
