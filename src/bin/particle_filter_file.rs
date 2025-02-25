use pancurses::{endwin, initscr, noecho, Input};
use std::{collections::VecDeque, env};
use rand::Rng;
use trajectory_mapper::{odometry_math::find_normalized_angle, particle_filter::ParticleFilter, point::Point, RobotMoveState, RobotPose, TrajectoryBuilder};

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
            pos: Point::new([p.pos[0] + rng.random_range(-0.2..0.2), p.pos[1] + rng.random_range(-0.2..0.2)]),
            theta: find_normalized_angle(p.theta + rng.random_range(-0.2..0.2))
        }
    });
    visualize_map(
        header.as_str(),
        &mut filter,
        points.iter().copied().collect(),
        100,
    );
}

fn width_height_from(points: &Vec<(RobotPose, RobotMoveState)>) -> (f64, f64) {
    let (mut min_x, mut min_y, mut max_x, mut max_y) = (0.0, 0.0, 0.0, 0.0);
    for (pose, _) in points.iter() {
        if pose.pos[0] < min_x {
            min_x = pose.pos[0];
        }
        if pose.pos[0] > max_x {
            max_x = pose.pos[0];
        }
        if pose.pos[1] < min_y {
            min_y = pose.pos[1];
        }
        if pose.pos[1] > max_y {
            max_y = pose.pos[1];
        }
    }
    (breadth(min_x, max_x), breadth(min_y, max_y))
}

fn breadth(lo: f64, hi: f64) -> f64 {
    let v = if lo.abs() > hi.abs() {
        lo.abs()
    } else {
        hi.abs()
    };
    v * 2.0
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
        let grid_size = map.grid_size();
        let grid_rows = grid_size[1] as i32;
        let grid_cols = grid_size[0] as i32;
        let row_grid_slice = round_quotient_up(grid_rows, rows);
        let col_grid_slice = round_quotient_up(grid_cols, columns);
        let mut grid_str = format!(
            "{header} {}/{} points alignment: {:.2}\n",
            total_points - points.len(),
            total_points,
            map.cumulative_alignment()
        );
        for row in 0..rows {
            let grid_row = row * grid_rows / rows;
            for col in 0..columns {
                let grid_col = col * grid_cols / columns;
                let free = map.free_space_within(
                    grid_row as u64,
                    grid_col as u64,
                    row_grid_slice as u64,
                    col_grid_slice as u64,
                );
                let obstacle = map.obstacle_within(
                    grid_row as u64,
                    grid_col as u64,
                    row_grid_slice as u64,
                    col_grid_slice as u64,
                );
                let c = if free && obstacle {
                    '?'
                } else if obstacle {
                    '#'
                } else if free {
                    'o'
                } else {
                    '.'
                };
                grid_str.push(c);
            }
            grid_str.push('\n');
        }

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

fn round_quotient_up(dividend: i32, divisor: i32) -> i32 {
    let mut quotient = dividend / divisor;
    if dividend % divisor > 0 {
        quotient += 1;
    }
    quotient
}
