use futures::stream::StreamExt;
use r2r::geometry_msgs::msg::TwistStamped;
use r2r::{nav_msgs::msg::Odometry, Context, Node, QosProfile};
use r2r::{std_msgs::msg::String as Ros2String, Publisher};
use trajectory_mapper::{Trajectory, TrajectoryBuilder};

use std::env;
use std::sync::Arc;

use smol::{lock::Mutex, stream::Stream};

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
    let odom_topic = format!("/{robot_name}/odom");
    let vel_topic = format!("/{robot_name}/cmd_vel_stamped");
    let estimate_topic_name = format!("/{robot_name}_pose_estimate");
    let node_name = format!("{robot_name}_trajectory_mapper");
    let context = Context::create()?;
    let mut node = Node::create(context, node_name.as_str(), "")?;
    let odom_subscriber =
        node.subscribe::<Odometry>(odom_topic.as_str(), QosProfile::sensor_data())?;
    let vel_subscriber = node.subscribe::<TwistStamped>(vel_topic.as_str(), QosProfile::sensor_data())?;
    let map = Arc::new(Mutex::new(TrajectoryBuilder::default().build()));

    let running = Arc::new(AtomicCell::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || r.store(false))?;

    let publisher = node
        .create_publisher::<Ros2String>(estimate_topic_name.as_str(), QosProfile::sensor_data())?;
    println!("Odometry reset:\nros2 service call /{robot_name}/reset_pose irobot_create_msgs/srv/ResetPose\n");
    println!("Starting {node_name}; subscribe to {estimate_topic_name}");
    smol::block_on(async {
        smol::spawn(odom_handler(odom_subscriber, map.clone(), publisher)).detach();
        while running.load() {
            node.spin_once(std::time::Duration::from_millis(period));
            println!("Spinning...");
        }
    });

    println!("Node {node_name} shutting down.");

    Ok(())
}

async fn odom_handler<S>(
    mut odom_subscriber: S,
    map: Arc<Mutex<Trajectory>>,
    publisher: Publisher<Ros2String>,
) where
    S: Stream<Item = Odometry> + Unpin,
{
    loop {
        println!("awaiting...");
        if let Some(odom_msg) = odom_subscriber.next().await {
            println!("received {odom_msg:?}");
            if let Some(data) = {
                let mut map = map.lock().await;
                println!("Locked map");
                map.add(odom_msg.into());
                println!("Updated map");
                map.estimate().map(|estimate| format!("{estimate:?}"))
            } {
                println!("Publishing {data}");
                let msg = Ros2String { data };
                if let Err(e) = publisher.publish(&msg) {
                    eprintln!("Error publishing {msg:?}: {e}");
                }
            }
        }
    }
}
