// 把 memory.x 暴露给链接器；cortex-m-rt 的 link.x 会 INCLUDE memory.x。
// 仅对 thumb* 裸机目标生效（宿主机 cargo test 不需要）。
use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=memory.x");
    let target = env::var("TARGET").unwrap();
    if target.starts_with("thumb") {
        let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
        println!("cargo:rustc-link-search=native={}", manifest.display());
    }
}
