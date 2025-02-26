use std::time::Instant;

use rand_distr::{Distribution, Normal};
use trajectory_mapper::{cmd::ArgVals, odometry_math::find_normalized_angle, particle_filter::ParticleFilter, point::{width_height_from, Point}, RobotPose, TrajectoryBuilder};

fn main() {
    let args = ArgVals::default();
    if args.len() < 5 {
        println!(
            "Usage: particle_expr filename num_particles=value linear_noise=value angular_noise=value num_runs=value"
        );
    } else {
        let filename = args.simple_vals[0].as_str();
        let num_particles = args.mapped_vals.get("num_particles").unwrap().parse::<usize>().unwrap();
        let linear_noise = args.mapped_vals.get("linear_noise").unwrap().parse::<f64>().unwrap();
        let angular_noise = args.mapped_vals.get("angular_noise").unwrap().parse::<f64>().unwrap();
        let num_runs = args.mapped_vals.get("num_runs").unwrap().parse::<usize>().unwrap();
        
        let points = RobotPose::from_file(filename).unwrap();
        let (width, height) = width_height_from(&points);
        let map = TrajectoryBuilder::default()
            .dimensions(width, height)
            .meters_per_cell(0.2)
            .build();

        let mut test_map = map.clone();
        for (pose, move_state) in points.iter() {
            test_map.add_move(*move_state);
            test_map.add_pose(*pose);
        }
        println!("Basic alignment: {:.2}", test_map.cumulative_alignment());
        let linear_normal = Normal::new(0.0, linear_noise).unwrap();
        let angular_normal = Normal::new(0.0, angular_noise).unwrap();

        for r in 0..num_runs {
            let start = Instant::now();
            let mut filter = ParticleFilter::new(&map, num_particles, |p| {
                let mut rng = rand::rng();
                RobotPose {
                    pos: Point::new([
                        p.pos[0] + linear_normal.sample(&mut rng),
                        p.pos[1] + linear_normal.sample(&mut rng),
                    ]),
                    theta: find_normalized_angle(p.theta + angular_normal.sample(&mut rng)),
                }
            });
            for (pose, move_state) in points.iter() {
                filter.iterate(*pose, *move_state);
            }
            let duration = Instant::now().duration_since(start).as_secs_f64();
            println!("Run {}/{num_runs}: {:.2} ({duration:.2}s)", r + 1, filter.current_best().cumulative_alignment());
        }
    }
}
