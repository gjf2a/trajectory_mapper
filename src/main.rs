use futures::{
    stream::StreamExt,
    task::{Context as FuturesContext, Poll},
    Future,
};
use r2r::std_msgs::msg::String as Ros2String;
use r2r::{nav_msgs::msg::Odometry, Context, Node, QosProfile};
use trajectory_mapper::TrajectoryBuilder;

use std::env;
use std::sync::Arc;

use crossbeam::atomic::AtomicCell;

// TODO: Subscribe to a topic to which motor TwistStamped messages are published.
//       Then use those to determine when a turn is occurring.

fn main() {
    let args = env::args().collect::<Vec<_>>();
    if args.len() < 2 {
        println!("Usage: trajectory_mapper robot_name [-spin_time:millseconds]")
    } else {
        let mut period = 100;
        for arg in args.iter() {
            if arg.starts_with("-spin_time") {
                match parse_spin_time(arg.as_str()) {
                    Ok(p) => {
                        period = p;
                    }
                    Err(e) => {
                        println!("Error in {arg}: {e}");
                    }
                }
            }
        }
        if let Err(e) = runner(args[1].as_str(), period) {
            println!("Unrecoverable error: {e}");
        }
    }
}

fn parse_spin_time(arg: &str) -> anyhow::Result<u64> {
    arg.split(':')
        .skip(1)
        .next()
        .ok_or(anyhow::Error::msg("Error: No colon"))?
        .parse()
        .map_err(anyhow::Error::from)
}

fn runner(robot_name: &str, period: u64) -> anyhow::Result<()> {
    let odom_topic_name = format!("/{robot_name}/odom");
    let estimate_topic_name = format!("/{robot_name}_pose_estimate");
    let node_name = format!("{robot_name}_trajectory_mapper");
    let context = Context::create()?;
    let mut node = Node::create(context, node_name.as_str(), "")?;
    let mut odom_subscriber =
        node.subscribe::<Odometry>(odom_topic_name.as_str(), QosProfile::sensor_data())?;
    let mut map = TrajectoryBuilder::default().build();

    let running = Arc::new(AtomicCell::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || r.store(false))?;

    let publisher = node
        .create_publisher::<Ros2String>(estimate_topic_name.as_str(), QosProfile::sensor_data())?;
    println!("Odometry reset:\nros2 service call /{robot_name}/reset_pose irobot_create_msgs/srv/ResetPose\n");
    println!("Starting {node_name}; subscribe to {estimate_topic_name}");
    while running.load() {
        node.spin_once(std::time::Duration::from_millis(period));
        let mut future = Box::pin(odom_subscriber.next());
        if let Poll::Ready(Some(odom_msg)) = future.as_mut().poll(&mut FuturesContext::from_waker(
            futures::task::noop_waker_ref(),
        )) {
            map.add(odom_msg.into());
            if let Some(estimate) = map.estimate() {
                let msg = Ros2String {
                    data: format!("{estimate:?}"),
                };
                if let Err(e) = publisher.publish(&msg) {
                    eprintln!("Error publishing {estimate:?}: {e}");
                }
            }
        }
    }

    println!("Node {node_name} shutting down.");

    Ok(())
}
