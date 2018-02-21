extern crate log;

fn main() {
    #[cfg(feature="conditional-feature-present")]
    println!("Conditional features work!");
    #[cfg(feature="conditional-feature-missing")]
    println!("Conditional features don't work!");
}
