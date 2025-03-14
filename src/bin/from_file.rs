use pancurses::{Input, endwin, initscr, noecho};
use std::{collections::VecDeque, env};
use trajectory_mapper::{RobotMoveState, RobotPose, TrajectoryBuilder, TrajectoryMap};

fn main() {
    let args = env::args().collect::<Vec<_>>();
    let filename = if args.len() > 1 {
        args[1].as_str()
    } else {
        "points.txt"
    };
    let points = RobotPose::from_file(filename).unwrap();
    let (width, height) = width_height_from(&points);
    let mut map = TrajectoryBuilder::default()
        .dimensions(width, height)
        .meters_per_cell(0.2)
        .build();
    let grid = map.grid_size();
    let header = format!("{width:.2} x {height:.2} ({} x {})", grid[0], grid[1]);
    visualize_map(
        header.as_str(),
        &mut map,
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

fn visualize_map(
    header: &str,
    map: &mut TrajectoryMap,
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
                        map.add_move(move_state);
                        map.add_pose(pose);
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
