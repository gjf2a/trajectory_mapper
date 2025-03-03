use trajectory_mapper::TrajectoryMap;
use std::{fs, env};
use pancurses::{Input, endwin, initscr, noecho};

fn main() {
    let args = env::args().collect::<Vec<_>>();
    if args.len() < 2 {
        println!("Usage: view_reachable filename");
    } else {
        let map = TrajectoryMap::from_python_dict(fs::read_to_string(args[1].as_str()).unwrap().as_str());
        let window = initscr();
        window.keypad(true);
        noecho();

        loop {
            window.clear();
            let (mut rows, mut columns) = window.get_max_yx();
            rows -= 2;
            columns -= 1;
            window.addstr(format!("dim: {:?} pos: {:?} ({:?})\n{}", map.width_height(), map.robot_position().unwrap().pos, map.start_cell(), map.grid_str_reachable(columns, rows)));
            match window.getch() {
                Some(Input::Character(c)) => match c {
                    'q' => break,
                    _ => {}
                },
                Some(Input::KeyDC) => break,
                _ => (),
            }
        }
        
        endwin();
    }
}