//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::{env, fs::File, io::Write, path::PathBuf};

fn main() {
    println!("cargo:rustc-link-arg-examples=--nmagic");
    println!("cargo:rustc-link-arg-examples=-Tlink.x");

    println!("cargo:rustc-link-arg-examples=-Tdefmt.x");

    println!("cargo:rerun-if-changed=../memory.x");

    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("../memory.x"))
        .unwrap()
        .write_all(include_bytes!("../memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
}
