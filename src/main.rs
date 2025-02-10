use r2r::{Context, Node, QosProfile, nav_msgs::msg::Odometry};
use futures::{Future, stream::StreamExt, task::{Context as FuturesContext, Poll}};

use std::env;

fn main() {
	let args = env::args().collect::<Vec<_>>();
	if args.len() < 2 {
		println!("Usage: trajectory_mapper robot_name [-spin_time:millseconds]")
	} else {
		let mut period = 100;
		for arg in args.iter() {
			if arg.starts_with("-spin_time") {
				match parse_spin_time(arg.as_str()) {
					Ok(p) => {period = p;}
					Err(e) => {println!("Error in {arg}: {e}");}
				}
			}
		}
		if let Err(e) = runner(args[1].as_str(), period) {
			println!("Unrecoverable error: {e}");
		}
	}
}

fn parse_spin_time(arg: &str) -> anyhow::Result<u64> {
	arg.split(':').skip(1).next().ok_or(anyhow::Error::msg("Error: No colon"))?.parse().map_err(anyhow::Error::from)
}

fn runner(robot_name: &str, period: u64) -> anyhow::Result<()> {
	let odom_topic_name = format!("/{robot_name}/odom");
	let node_name = format!("{robot_name}_trajectory_mapper");
	let context = Context::create()?;
	let mut node = Node::create(context, node_name.as_str(), "")?;
	let qos = QosProfile::sensor_data();
	let mut odom_subscriber = node.subscribe::<Odometry>(odom_topic_name.as_str(), qos)?;

	loop {
		node.spin_once(std::time::Duration::from_millis(period));
		let mut future = Box::pin(odom_subscriber.next());
		if let Poll::Ready(Some(odom_msg)) = future.as_mut().poll(
		                &mut FuturesContext::from_waker(futures::task::noop_waker_ref()),
		            ) {
		   println!("{odom_msg:?}");
		} else {
			println!("No message");
		}
	}
	
}
