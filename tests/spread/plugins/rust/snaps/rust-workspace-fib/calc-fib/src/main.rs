use std::env;
extern crate fib;

fn main() {
    let args: Vec<String> = env::args().collect();
 
    if args.len() < 2 {
        eprintln!("please specify number");
        return
    }

    let num_string = &args[1];
    let number: i32 = match num_string.parse() {
        Ok(n) => {
            n
        },
        Err(_) => {
            eprintln!("error: not an integer");
            return;
        }
    };

    let sum = fib::fib(number);
    println!("{}", sum);
}
