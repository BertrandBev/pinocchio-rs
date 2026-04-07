use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    // Asset plaform/arch validity
    lib_path().expect("invalid platform/arch");

    // Build the Cpp lib
    #[cfg(feature = "source_build")]
    source_build();
    #[cfg(not(feature = "source_build"))]
    download_binary();

    // Compile the bridge
    let cpp_dir = cpp_dir();
    let include_dirs = get_include_dirs(&cpp_dir);
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

// Utils
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

fn download_file(url: &str, dest: &PathBuf) {
    let response = ureq::get(url)
        .call()
        .expect(&format!("Failed to download {}", url));
    let mut file =
        std::fs::File::create(dest).expect(&format!("Failed to create file at {:?}", dest));
    let mut reader = response.into_body().into_reader();
    std::io::copy(&mut reader, &mut file).expect("Failed to write downloaded file");
}

#[cfg(feature = "source_build")]
fn source_build() {
    let xmake_status = Command::new("xmake")
        .current_dir(&cpp_dir())
        .status()
        .expect("xmake cmd not found");
    assert!(xmake_status.success(), "xmake build failed");
}

fn download_binary() {
    let path = lib_path().unwrap();
    let url = lib_url().unwrap();
    if !path.exists() {
        let lib_dir = lib_dir().unwrap();
        std::fs::create_dir_all(&lib_dir).expect("Failed to create output directory");
        println!("cargo:info=Downloading pre-built binary");
        download_file(&url, &path);
    }
}

fn cpp_dir() -> PathBuf {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    manifest_dir.join("cpp")
}

fn lib_dir() -> Option<PathBuf> {
    let platform = Plaform::get()?;
    let arch = Arch::get()?;
    Some(
        cpp_dir()
            .join(platform.name())
            .join(arch.name())
            .join("release")
            .join("release"),
    )
}

fn lib_name() -> &'static str {
    "libpinocchio.a"
}

fn lib_path() -> Option<PathBuf> {
    Some(lib_dir()?.join(lib_name()))
}

fn lib_url() -> Option<String> {
    const RELEASE_URL: &str = "https://github.com/BertrandBev/pinocchio-rs/releases/download";
    let version = env!("CARGO_PKG_VERSION");
    let platform = Plaform::get()?.name();
    let arch = Arch::get()?.name();
    Some(format!(
        "{RELEASE_URL}/v${version}/libpinocchio-{platform}-{arch}.a"
    ))
}

#[derive(Clone, Copy)]
enum Plaform {
    Mac,
    Linux,
    #[allow(unused)]
    Windows,
}

#[derive(Clone, Copy)]
enum Arch {
    X64,
    ARM64,
}

impl Plaform {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Mac => "macosx",
            Self::Linux => "linux",
            Self::Windows => "windows",
        }
    }

    pub fn get() -> Option<Self> {
        let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap();
        match target_os.as_str() {
            "macos" => Some(Self::Mac),
            "linux" => Some(Self::Linux),
            _ => None,
        }
    }
}

impl Arch {
    pub fn name(&self) -> &'static str {
        match self {
            Self::X64 => "x64",
            Self::ARM64 => "arm64",
        }
    }

    pub fn get() -> Option<Self> {
        let target_arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
        match target_arch.as_str() {
            "aarch64" => Some(Self::ARM64),
            "x86_64" => Some(Self::X64),
            _ => None,
        }
    }
}
