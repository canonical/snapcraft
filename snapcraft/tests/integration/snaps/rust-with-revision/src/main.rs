extern crate log;
extern crate rustc_version_runtime;

use rustc_version_runtime::version;

fn main() {
    println!("Rust revision: {}", version());
}
