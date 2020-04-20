extern crate log;

fn main() {
    #[cfg(feature="conditional-feature-present")]
    println!("hello world");
}
