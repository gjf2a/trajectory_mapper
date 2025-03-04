use std::fs;
use futures::stream::StreamExt;

use crossbeam::atomic::AtomicCell;
use r2r::{nav_msgs::msg::Odometry, std_msgs::msg::String as Ros2String, Context, Node, Publisher, QosProfile};
use trajectory_mapper::{cmd, executor::PathPlanExecutor, point::FloatPoint, RobotPose, TrajectoryMap};

use std::sync::Arc;
use smol::{lock::Mutex, stream::Stream};

fn main() {
    let args = cmd::ArgVals::default();
    if args.len() < 2 {
        println!("Usage: navigator_node robot_name map_filename [-spin_time=millseconds] [-tolerance=meters]");
    } else {
        let period = args.get_value("-spin_time").unwrap_or(100);
        let tolerance = args.get_value("-tolerance").unwrap_or(0.1);
        let map = TrajectoryMap::from_python_dict(fs::read_to_string(args.get_symbol(1)).unwrap().as_str());
        if let Err(e) = runner(args.get_symbol(0), map, period, tolerance) {
            println!("Unrecoverable error: {e}");
        }
    }
}

fn runner(robot_name: &str, map: TrajectoryMap, period: u64, tolerance: f64) -> anyhow::Result<()> {
    let odom_topic = format!("/{robot_name}/odom");
    let incoming_goal_topic = format!("/{robot_name}_goal");
    let outgoing_waypoint_topic = format!("/{robot_name}_waypoints");
    let node_name = format!("{robot_name}_navigator");
    let context = Context::create()?;
    let mut node = Node::create(context, node_name.as_str(), "")?;
    let odom_subscriber =
        node.subscribe::<Odometry>(odom_topic.as_str(), QosProfile::sensor_data())?;
    let goal_subscriber = node.subscribe::<Ros2String>(incoming_goal_topic.as_str(), QosProfile::sensor_data())?;
    
    let executor = Arc::new(Mutex::new(PathPlanExecutor::new(map)));

    let running = Arc::new(AtomicCell::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || r.store(false))?;

    let publisher =
        node.create_publisher::<Ros2String>(outgoing_waypoint_topic.as_str(), QosProfile::sensor_data())?;
    println!("Listening for goal locations on topic {incoming_goal_topic}.");
    println!("Publishing current waypoint on topic {outgoing_waypoint_topic}.");
    
    smol::block_on(async {
        smol::spawn(goal_handler(goal_subscriber, executor.clone())).detach();
        smol::spawn(odom_handler(odom_subscriber, executor.clone(), tolerance, publisher)).detach();
        while running.load() {
            node.spin_once(std::time::Duration::from_millis(period));
        }
    });

    println!("Node {node_name} shutting down.");

    Ok(())
}

async fn odom_handler<S>(
    mut odom_subscriber: S,
    executor: Arc<Mutex<PathPlanExecutor>>,
    tolerance: f64,
    publisher: Publisher<Ros2String>,
) where
    S: Stream<Item = Odometry> + Unpin,
{
    loop {
        if let Some(odom_msg) = odom_subscriber.next().await {
            let mut executor = executor.lock().await;
            if let Some(waypoint) = executor.waypoint() {
                let pose: RobotPose = odom_msg.into();
                if waypoint.euclidean_distance(pose.pos) < tolerance {
                    executor.advance();
                }
            }
            
            let data = match executor.waypoint() {
                None => format!("{{'status': 'stopped', 'waypoint': None}}"),
                Some(waypoint) => format!("{{'status': 'navigating', 'waypoint': ({}, {})", waypoint[0], waypoint[1])
            };

            let msg = Ros2String { data };
            if let Err(e) = publisher.publish(&msg) {
                eprintln!("Error publishing {msg:?}: {e}");
            }
        }
    }
}

async fn goal_handler<S>(mut avoid_subscriber: S, executor: Arc<Mutex<PathPlanExecutor>>)
where
    S: Stream<Item = Ros2String> + Unpin,
{
    loop {
        if let Some(goal_msg) = avoid_subscriber.next().await {
            let mut executor = executor.lock().await;
            if let Ok(goal) = goal_msg.data.parse::<FloatPoint>() {
                executor.make_plan(goal);   
            }
        }
    }
}
