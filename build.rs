use std::env;
use std::path::PathBuf;

fn main() {
    // Skip download when generating docs
    if env::var("DOCS_RS").is_ok() {
        println!("cargo:rustc-cfg=docsrs");
        return;
    }

    // Asset plaform/arch validity
    Arch::get().expect("invalid arch");
    Plaform::get().expect("invalid platform");

    // Build the Cpp lib
    #[cfg(feature = "source_build")]
    source_build();
    #[cfg(not(feature = "source_build"))]
    download_binary();

    // Compile the bridge
    let cpp_dir = cpp_dir();
    cxx_build::bridge("src/lib.rs")
        .include(&cpp_dir.join("src"))
        .std("c++17")
        .flag_if_supported("-O3")
        .compile("pinocchio_bridge_cxx");

    // Link
    let lib_dir = lib_dir();
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=static=pinocchio");

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

#[cfg(feature = "source_build")]
fn source_build() {
    use std::process::Command;
    let mut cmd = Command::new("xmake");
    println!("cargo:warning=out_dir: {:?}", out_dir());
    println!("cargo:warning=cpp_dir: {:?}", cpp_dir());
    let cmd = cmd
        .args(["-y", "-P", cpp_dir().to_str().unwrap()])
        .current_dir(&out_dir());
    println!("cargo:warning=cmd: {:?}", cmd);
    let xmake_status = cmd.status().expect("xmake cmd error");
    assert!(xmake_status.success(), "xmake build failed");

    // Copy built lib
    let platform = Plaform::get().unwrap();
    let arch = Arch::get().unwrap();
    let src_dir = out_dir()
        .join("build")
        .join(platform.name())
        .join(arch.name())
        .join("release")
        .join(lib_name());
    let dst_dir = lib_dir();
    std::fs::create_dir_all(&dst_dir).expect("Failed to create output directory");
    std::fs::copy(src_dir, dst_dir.join(lib_name())).unwrap();
}

#[cfg(not(feature = "source_build"))]
fn download_binary() {
    let path = lib_path();
    let url = lib_url().unwrap();
    if !path.exists() {
        let lib_dir = lib_dir();
        std::fs::create_dir_all(&lib_dir).expect("Failed to create output directory");
        println!("cargo:info=Downloading pre-built binary");
        download_file(&url, &path);
    }
}

#[cfg(not(feature = "source_build"))]
fn download_file(url: &str, dest: &PathBuf) {
    let response = match ureq::get(url).call() {
        Ok(ok) => ok,
        Err(err) => {
            panic!(
                "Failed to download {}: {}\nTo build from source, enable the feature 'source_build'",
                url, err
            )
        }
    };

    let mut file =
        std::fs::File::create(dest).expect(&format!("Failed to create file at {:?}", dest));
    let mut reader = response.into_body().into_reader();
    std::io::copy(&mut reader, &mut file).expect("Failed to write downloaded file");
}

#[cfg(not(feature = "source_build"))]
fn lib_url() -> Option<String> {
    const RELEASE_URL: &str = "https://github.com/BertrandBev/pinocchio-rs/releases/download";
    let version = env::var("CARGO_PKG_VERSION").unwrap();
    let platform = Plaform::get()?.name();
    let arch = Arch::get()?.name();
    Some(format!(
        "{RELEASE_URL}/v{version}/libpinocchio-{platform}-{arch}.a"
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

// Path utils

fn cpp_dir() -> PathBuf {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    manifest_dir.join("cpp")
}

fn out_dir() -> PathBuf {
    PathBuf::from(env::var("OUT_DIR").unwrap())
}

fn lib_dir() -> PathBuf {
    let platform = Plaform::get().unwrap();
    let arch = Arch::get().unwrap();
    let version = env::var("CARGO_PKG_VERSION").unwrap();
    out_dir()
        .join("lib")
        .join(format!("{version}"))
        .join(platform.name())
        .join(arch.name())
        .join("release")
}

fn lib_name() -> &'static str {
    "libpinocchio.a"
}

#[cfg(not(feature = "source_build"))]
fn lib_path() -> PathBuf {
    lib_dir().join(lib_name())
}
