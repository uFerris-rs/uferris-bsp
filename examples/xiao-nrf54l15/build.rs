//! Copies `memory.x` into a directory the linker always searches.
//!
//! The file needs to be available to the linker at build time, and putting it
//! in `OUT_DIR` keeps it out of the source tree of any crate that depends on
//! this one.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
