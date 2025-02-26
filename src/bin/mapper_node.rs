use futures::stream::StreamExt;
use r2r::geometry_msgs::msg::TwistStamped;
use r2r::{Context, Node, QosProfile, nav_msgs::msg::Odometry};
use r2r::{Publisher, std_msgs::msg::String as Ros2String};
use trajectory_mapper::cmd::ArgVals;
use trajectory_mapper::{TrajectoryBuilder, TrajectoryMap, cmd};

use std::sync::Arc;

use smol::{lock::Mutex, stream::Stream};

use crossbeam::atomic::AtomicCell;

fn main() {
    let args = cmd::ArgVals::default();
    if args.len() < 2 {
        println!(
            "Usage: mapper_node robot_name [-spin_time=millseconds] [-dim=width,height] [-meters_per_cell=meters_per_cell]"
        );
    } else {
        match parse_args(&args) {
            Ok((period, builder)) => {
                if let Err(e) = runner(args.get_symbol(0), period, builder) {
                    println!("Unrecoverable error: {e}");
                }
            }
            Err(e) => {
                println!("Error: {e}");
            }
        }
    }
}

fn parse_args(args: &ArgVals) -> anyhow::Result<(u64, TrajectoryBuilder)> {
    let mut period = 100;
    let mut builder = TrajectoryBuilder::default();
    if let Some(spin_time) = args.get_value("-spin_time") {
        period = spin_time;
    }
    if let Some((width, height)) = args.get_duple("-dim") {
        builder.dimensions(width, height);
    }
    if let Some(mpc) = args.get_value("-meters_per_cell") {
        builder.meters_per_cell(mpc);
    }
    Ok((period, builder))
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
    let map = builder.build();
    let dimensions = map.grid_size();
    let map = Arc::new(Mutex::new(map));

    let running = Arc::new(AtomicCell::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || r.store(false))?;

    let publisher =
        node.create_publisher::<Ros2String>(map_topic_name.as_str(), QosProfile::sensor_data())?;
    println!(
        "Odometry reset:\nros2 service call /{robot_name}/reset_pose irobot_create_msgs/srv/ResetPose\n"
    );
    println!("Grid dimensions: {dimensions}");
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
