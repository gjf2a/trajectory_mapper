use smol::Timer;
use futures::future;

async fn async_function_1() {
    Timer::after(std::time::Duration::from_secs(1)).await;
    println!("Function 1 called!");
}

async fn async_function_2() {
    Timer::after(std::time::Duration::from_secs(2)).await;
    println!("Function 2 called!");
}

fn main() {
    smol::block_on(async {
        // Create pinned versions of the two futures
        let mut fut1 = Box::pin(async_function_1());
        let mut fut2 = Box::pin(async_function_2());

        loop {
            // Use `future::select` to wait for either future to complete
            match future::select(&mut fut1, &mut fut2).await {
                future::Either::Left((_, _)) => {
                    // If `fut1` completes, restart it
                    println!("Restarting Function 1");
                    fut1 = Box::pin(async_function_1());
                }
                future::Either::Right((_, _)) => {
                    // If `fut2` completes, restart it
                    println!("Restarting Function 2");
                    fut2 = Box::pin(async_function_2());
                }
            }
        }
    });
}
