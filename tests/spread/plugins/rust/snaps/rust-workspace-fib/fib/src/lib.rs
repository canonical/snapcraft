// Terribly inefficient recursive fib routine.
pub fn fib(n: i32) -> u64 {
    if n < 0 {
        panic!("invalid: {}", n);
    }
    if n == 0 {
        return 0;
    } else if n == 1 {
        return 1;
    } else {
        return fib(n - 1) + fib(n - 2)
    }
}
