use pancurses::{endwin, initscr, noecho, Input};
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
        .meters_per_cell(0.1)
        .build();
    let grid = map.grid_size();
    let header = format!("{width:.2} x {height:.2} ({} x {})", grid[0], grid[1]);
    visualize_map(
        header.as_str(),
        &mut map,
        points.iter().copied().collect(),
        1,
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
        let (rows, columns) = window.get_max_yx();
        let rows = rows as u64 - 3;
        let columns = columns as u64 - 1;
        let grid_size = map.grid_size();
        let grid_rows = grid_size[1];
        let grid_cols = grid_size[0];
        let row_grid_slice = round_quotient_up(grid_rows as i32, rows as i32) as u64;
        let col_grid_slice = round_quotient_up(grid_cols as i32, columns as i32) as u64;
        let mut grid_str = format!(
            "{header} {}/{} points ({})\n",
            total_points - points.len(),
            total_points,
            map.num_points()
        );
        for row in 0..rows {
            let grid_row = row * grid_rows / rows;
            for col in 0..columns {
                let grid_col = col * grid_cols / columns;
                let free =
                    map.free_space_within(grid_row, grid_col, row_grid_slice, col_grid_slice);
                let obstacle =
                    map.obstacle_within(grid_row, grid_col, row_grid_slice, col_grid_slice);
                let c = if free && obstacle {
                    'X'
                } else if free {
                    'O'
                } else if obstacle {
                    '#'
                } else {
                    '.'
                };
                grid_str.push(c);
            }
            grid_str.push('\n');
        }

        window.addstr(grid_str);
        window.refresh();

        match window.getch() {
            Some(Input::Character(c)) => match c {
                ' ' => {
                    let mut countdown = points_per_key;
                    while countdown > 0 && points.len() > 0 {
                        countdown -= 1;
                        let (pose, state) = points.pop_front().unwrap();
                        map.add_pose(pose);
                        map.add_move(state);
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
