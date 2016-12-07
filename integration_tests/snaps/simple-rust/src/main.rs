extern crate log;

fn main() {
    println!("There is rust on snaps!");
    #[cfg(feature="conditional-feature")]
    println!("Conditional features work!");
}
