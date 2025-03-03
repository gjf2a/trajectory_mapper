use pancurses::{Input, endwin, initscr, noecho};
use rand::Rng;
use std::{collections::VecDeque, env};
use trajectory_mapper::{
    RobotMoveState, RobotPose, TrajectoryBuilder,
    odometry_math::find_normalized_angle,
    particle_filter::ParticleFilter,
    point::{Point, width_height_from},
};

fn main() {
    let args = env::args().collect::<Vec<_>>();
    let filename = if args.len() > 1 {
        args[1].as_str()
    } else {
        "points.txt"
    };
    let points = RobotPose::from_file(filename).unwrap();
    let (width, height) = width_height_from(&points);
    let map = TrajectoryBuilder::default()
        .dimensions(width, height)
        .meters_per_cell(0.2)
        .build();
    let grid = map.grid_size();
    let header = format!("{width:.2} x {height:.2} ({} x {})", grid[0], grid[1]);
    let mut filter = ParticleFilter::new(&map, 100, |p| {
        let mut rng = rand::rng();
        RobotPose {
            pos: Point::new([
                p.pos[0] + rng.random_range(-0.2..0.2),
                p.pos[1] + rng.random_range(-0.2..0.2),
            ]),
            theta: find_normalized_angle(p.theta + rng.random_range(-0.2..0.2)),
        }
    });
    visualize_map(
        header.as_str(),
        &mut filter,
        points.iter().copied().collect(),
        100,
    );
}

fn visualize_map<N: Clone + Fn(RobotPose) -> RobotPose>(
    header: &str,
    filter: &mut ParticleFilter<N>,
    mut points: VecDeque<(RobotPose, RobotMoveState)>,
    points_per_key: usize,
) {
    let window = initscr();
    window.keypad(true);
    noecho();

    let total_points = points.len();

    loop {
        window.clear();
        let (mut rows, mut columns) = window.get_max_yx();
        rows -= 3;
        columns -= 1;
        let map = filter.current_best();
        let grid_str = format!(
            "{header} {}/{} points alignment: {:.2}\n{}",
            total_points - points.len(),
            total_points,
            map.cumulative_alignment(),
            map.grid_str(columns, rows)
        );

        window.addstr(grid_str);
        match window.getch() {
            Some(Input::Character(c)) => match c {
                ' ' => {
                    let mut countdown = points_per_key;
                    while countdown > 0 && points.len() > 0 {
                        countdown -= 1;
                        let (pose, move_state) = points.pop_front().unwrap();
                        filter.iterate(pose, move_state);
                    }
                }
                'q' => break,
                _ => {}
            },
            Some(Input::KeyDC) => break,
            _ => (),
        }
    }

    endwin();
}
