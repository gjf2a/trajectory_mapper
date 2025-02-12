// From https://www.perplexity.ai/search/i-m-writing-a-rust-program-it-Y44h9yh8SfaCEkGRyqwEtA

use smol::Timer;
use std::time::Duration;

async fn async_function_1() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        println!("Function 1 called!");
    }
}

async fn async_function_2() {
    loop {
        Timer::after(Duration::from_secs(2)).await;
        println!("Function 2 called!");
    }
}

fn main() {
    smol::block_on(async {
        smol::spawn(async_function_1()).detach();
        smol::spawn(async_function_2()).detach();
        loop {
            Timer::after(Duration::from_secs(3600)).await;
        }
    });
}
