use futures::stream::StreamExt;
use r2r::{Context, Node, QosProfile, nav_msgs::msg::Odometry};
use r2r::{Publisher, std_msgs::msg::String as Ros2String};
use trajectory_mapper::cmd::ArgVals;
use trajectory_mapper::{RobotMoveState, TrajectoryBuilder, TrajectoryMap, cmd};

use std::sync::Arc;

use smol::{lock::Mutex, stream::Stream};

use crossbeam::atomic::AtomicCell;

fn main() {
    let args = cmd::ArgVals::default();
    if args.len() < 1 {
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
    let avoid_topic = format!("{robot_name}_map_avoiding");
    let map_topic_name = format!("/{robot_name}_trajectory_map");
    let node_name = format!("{robot_name}_trajectory_mapper");
    let context = Context::create()?;
    let mut node = Node::create(context, node_name.as_str(), "")?;
    let odom_subscriber =
        node.subscribe::<Odometry>(odom_topic.as_str(), QosProfile::sensor_data())?;
    let avoid_subscriber =
        node.subscribe::<Ros2String>(avoid_topic.as_str(), QosProfile::sensor_data())?;
    let map = builder.build();
    let dimensions = map.grid_size();
    let map = Arc::new(Mutex::new(map));

    let running = Arc::new(AtomicCell::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || r.store(false))?;

    let publisher =
        node.create_publisher::<Ros2String>(map_topic_name.as_str(), QosProfile::sensor_data())?;
    println!("Grid dimensions: {dimensions}");
    println!("Starting {node_name}");
    println!("Subscribe to {map_topic_name} for map information");
    println!(
        "Publish 'clear' or 'avoid' to {avoid_topic} to indicate whether robot is avoiding an obstacle"
    );
    smol::block_on(async {
        smol::spawn(odom_handler(odom_subscriber, map.clone(), publisher)).detach();
        smol::spawn(avoid_handler(avoid_subscriber, map.clone())).detach();
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

async fn avoid_handler<S>(mut avoid_subscriber: S, map: Arc<Mutex<TrajectoryMap>>)
where
    S: Stream<Item = Ros2String> + Unpin,
{
    loop {
        if let Some(avoid_msg) = avoid_subscriber.next().await {
            let mut map = map.lock().await;
            let move_state = match avoid_msg.data.as_str() {
                "avoid" => RobotMoveState::Turning,
                // Dodgy but effective. Error-handling is not a strong point of ROS2.
                _ => RobotMoveState::Forward,
            };
            map.add_move(move_state);
        }
    }
}
