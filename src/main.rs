use futures::stream::StreamExt;
use r2r::geometry_msgs::msg::TwistStamped;
use r2r::{nav_msgs::msg::Odometry, Context, Node, QosProfile};
use r2r::{std_msgs::msg::String as Ros2String, Publisher};
use trajectory_mapper::{TrajectoryBuilder, TrajectoryMap};

use std::env;
use std::sync::Arc;

use smol::{lock::Mutex, stream::Stream};

use crossbeam::atomic::AtomicCell;

fn main() {
    let args = env::args().collect::<Vec<_>>();
    if args.len() < 2 {
        println!("Usage: trajectory_mapper robot_name [-spin_time:millseconds] [-dim:widthXheight] [-meters_per_cell:meters_per_cell]");
    } else {
        match parse_args(&args) {
            Ok((period, builder)) => {
                if let Err(e) = runner(args[1].as_str(), period, builder) {
                    println!("Unrecoverable error: {e}");
                }
            }
            Err(e) => {
                println!("Error: {e}");
            }
        }
    }
}

fn parse_args(args: &Vec<String>) -> anyhow::Result<(u64, TrajectoryBuilder)> {
    let mut period = 100;
    let mut builder = TrajectoryBuilder::default();
    for arg in args.iter() {
        if arg.starts_with("-spin_time") {
            period = parse_spin_time(arg.as_str())?;
        }
        else if arg.starts_with("-dim") {
            let (width, height) = parse_dimension(arg.as_str())?;
            builder.dimensions(width, height);
        } 
        else if arg.starts_with("-meters_per_cell") {
           builder.meters_per_cell(parse_meters_per_cell(arg.as_str())?);
        }
    }
    Ok((period, builder))
}

fn get_past_colon(arg: &str) -> anyhow::Result<&str> {
    arg.split(':')
    .skip(1)
    .next()
    .ok_or(anyhow::Error::msg("Error in {arg}: No colon"))
}

fn parse_spin_time(arg: &str) -> anyhow::Result<u64> {
    get_past_colon(arg)?.parse().map_err(anyhow::Error::from)
}

fn parse_dimension(arg: &str) -> anyhow::Result<(f64, f64)> {
    let mut iter = get_past_colon(arg)?.split('X');
    let width = iter.next().ok_or(anyhow::Error::msg("Error in {arg}: No width given"))?.parse().map_err(anyhow::Error::from)?;
    let height = iter.next().ok_or(anyhow::Error::msg("Error in {arg}: No height given"))?.parse().map_err(anyhow::Error::from)?;
    Ok((width, height))
}

fn parse_meters_per_cell(arg: &str) -> anyhow::Result<f64> {
    get_past_colon(arg)?.parse().map_err(anyhow::Error::from)
}

fn runner(robot_name: &str, period: u64, builder: TrajectoryBuilder) -> anyhow::Result<()> {
    let odom_topic = format!("/{robot_name}/odom");
    let vel_topic = format!("/{robot_name}/cmd_vel_stamped");
    let map_topic_name = format!("/{robot_name}_trajectory_map");
    let node_name = format!("{robot_name}_trajectory_mapper");
    let context = Context::create()?;
    let mut node = Node::create(context, node_name.as_str(), "")?;
    let odom_subscriber =
        node.subscribe::<Odometry>(odom_topic.as_str(), QosProfile::sensor_data())?;
    let vel_subscriber =
        node.subscribe::<TwistStamped>(vel_topic.as_str(), QosProfile::sensor_data())?;
    let map = Arc::new(Mutex::new(builder.build()));

    let running = Arc::new(AtomicCell::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || r.store(false))?;

    let publisher = node
        .create_publisher::<Ros2String>(map_topic_name.as_str(), QosProfile::sensor_data())?;
    println!("Odometry reset:\nros2 service call /{robot_name}/reset_pose irobot_create_msgs/srv/ResetPose\n");
    println!("Starting {node_name}; subscribe to {map_topic_name}");
    smol::block_on(async {
        smol::spawn(odom_handler(odom_subscriber, map.clone(), publisher)).detach();
        smol::spawn(vel_handler(vel_subscriber, map.clone())).detach();
        while running.load() {
            node.spin_once(std::time::Duration::from_millis(period));
        }
    });

    println!("Node {node_name} shutting down.");

    Ok(())
}

async fn odom_handler<S>(
    mut odom_subscriber: S,
    map: Arc<Mutex<TrajectoryMap>>,
    publisher: Publisher<Ros2String>,
) where
    S: Stream<Item = Odometry> + Unpin,
{
    loop {
        if let Some(odom_msg) = odom_subscriber.next().await {
            if let Some(data) = {
                let mut map = map.lock().await;
                map.add_pose(odom_msg.into());
                map.as_python_dict()
            } {
                let msg = Ros2String { data };
                if let Err(e) = publisher.publish(&msg) {
                    eprintln!("Error publishing {msg:?}: {e}");
                }
            }
        }
    }
}

async fn vel_handler<S>(mut vel_subscriber: S, map: Arc<Mutex<TrajectoryMap>>)
where
    S: Stream<Item = TwistStamped> + Unpin,
{
    loop {
        if let Some(vel_msg) = vel_subscriber.next().await {
            let mut map = map.lock().await;
            map.add_move(vel_msg.into());
        }
    }
}
