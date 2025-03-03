/*
Next steps:
1. Create an A* pathfinder.
2. Refactor the BinaryGrid so as to not require stating the size in advance.
3. Get some practice with the particle filter, to figure out some noise models
that create practical improvements. One idea might be to look for cells that are both
freespace and obstacles.
 */

use std::{
    cmp::max,
    collections::VecDeque,
    fmt::Display,
    fs::File,
    io::{BufRead, BufReader},
};

use anyhow::{Context, anyhow};
use bits::BitArray;
use itertools::Itertools;
use odometry_math::find_roll_pitch_yaw;
use point::{FloatPoint, GridPoint};
use r2r::{geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry};

use pest::Parser;
use pest_derive::Parser;
use search_iter::PrioritySearchIter;

#[derive(Parser)]
#[grammar = "map_python_dictionary.pest"]
struct PythonMapParser;

pub mod cmd;
pub mod odometry_math;
pub mod particle_filter;
pub mod point;
pub mod search_iter;

/*
Eventually, I would like to create a ROS2 message like this:
float64 x
float64 y
float64 theta
uint64 columns
uint64 rows
float64 meters_per_cell
uint64[] free_space
uint64[] obstacles
 */

#[derive(Copy, Clone, PartialEq, Debug, Default)]
pub struct RobotPose {
    pub pos: FloatPoint,
    pub theta: f64,
}

impl RobotPose {
    pub fn moved_forward(&self, distance: f64) -> Self {
        let (x, y) = (
            self.pos[0] + distance * self.theta.cos(),
            self.pos[1] + distance * self.theta.sin(),
        );
        Self {
            pos: FloatPoint::new([x, y]),
            theta: self.theta,
        }
    }

    pub fn from_file(filename: &str) -> anyhow::Result<Vec<(Self, RobotMoveState)>> {
        let file_in = File::open(filename)?;
        let reader = BufReader::new(file_in);
        let mut points = vec![];
        for line in reader.lines() {
            let line = line?
                .replace("(", "")
                .replace(")", "")
                .replace(",", "")
                .replace("'", "");
            let line = line.split_whitespace().collect::<Vec<_>>();
            let nums = line
                .iter()
                .take(3)
                .map(|s| s.parse::<f64>())
                .collect::<Result<Vec<_>, _>>()
                .context("Parse error with robot coordinates")?;
            if nums.len() >= 3 {
                let pose = RobotPose {
                    pos: FloatPoint::new([nums[0], nums[1]]),
                    theta: nums[2],
                };
                let mut state = RobotMoveState::Forward;
                if line.len() == 4 && line[3] == "Turning" {
                    state = RobotMoveState::Turning;
                }
                points.push((pose, state));
            } else {
                return Err(anyhow!("Illegal number of points: {}", nums.len()));
            }
        }
        Ok(points)
    }
}

impl From<Odometry> for RobotPose {
    fn from(value: Odometry) -> Self {
        let mut result = Self::default();
        result.pos[0] = value.pose.pose.position.x;
        result.pos[1] = value.pose.pose.position.y;
        let (_, _, theta) = find_roll_pitch_yaw(value);
        result.theta = theta;
        result
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub enum RobotMoveState {
    #[default]
    Forward,
    Turning,
}

impl From<TwistStamped> for RobotMoveState {
    fn from(value: TwistStamped) -> Self {
        if value.twist.angular.z > 0.0 {
            Self::Turning
        } else {
            Self::Forward
        }
    }
}

pub struct TrajectoryBuilder {
    robot_radius_meters: f64,
    width: f64,
    height: f64,
    meters_per_cell: f64,
}

impl Default for TrajectoryBuilder {
    fn default() -> Self {
        Self {
            robot_radius_meters: 0.35,
            width: 3.0,
            height: 3.0,
            meters_per_cell: 0.1,
        }
    }
}

impl TrajectoryBuilder {
    pub fn robot_radius(&mut self, robot_radius_meters: f64) -> &mut Self {
        self.robot_radius_meters = robot_radius_meters;
        self
    }

    pub fn dimensions(&mut self, width: f64, height: f64) -> &mut Self {
        self.width = width;
        self.height = height;
        self
    }

    pub fn meters_per_cell(&mut self, meters_per_cell: f64) -> &mut Self {
        self.meters_per_cell = meters_per_cell;
        self
    }

    pub fn build(&self) -> TrajectoryMap {
        TrajectoryMap {
            move_state: RobotMoveState::default(),
            robot_radius_meters: self.robot_radius_meters,
            position: None,
            obstacles: BinaryGrid::new(self.width, self.height, self.meters_per_cell),
            free_space: BinaryGrid::new(self.width, self.height, self.meters_per_cell),
            turn_in_progress: false,
            cumulative_alignment: 0.0,
            converter: GridPointConverter::new(self.meters_per_cell, self.width, self.height),
        }
    }
}

#[derive(Clone, Debug)]
pub struct TrajectoryMap {
    position: Option<RobotPose>,
    free_space: BinaryGrid,
    obstacles: BinaryGrid,
    robot_radius_meters: f64,
    move_state: RobotMoveState,
    turn_in_progress: bool,
    cumulative_alignment: f64,
    converter: GridPointConverter,
}

impl TrajectoryMap {
    pub fn add_pose(&mut self, pose: RobotPose) {
        match self.move_state {
            RobotMoveState::Forward => {
                self.turn_in_progress = false;
                self.position = Some(pose);
                self.free_space
                    .set_circle(pose.pos, self.reduced_robot_footprint());
            }
            RobotMoveState::Turning => {
                if !self.turn_in_progress {
                    self.turn_in_progress = true;
                    if let Some(prev) = self.position {
                        let obstacle = prev.moved_forward(self.robot_radius_meters);
                        self.obstacles
                            .set(self.obstacles.meters2cell(obstacle.pos), true);
                    }
                }
            }
        }
        self.cumulative_alignment += current_move_alignment(self);
    }

    pub fn meters_per_cell(&self) -> f64 {
        self.converter.meters_per_cell
    }

    fn reduced_robot_footprint(&self) -> f64 {
        if self.robot_radius_meters < self.meters_per_cell() {
            self.robot_radius_meters
        } else if self.robot_radius_meters < 2.0 * self.meters_per_cell() {
            self.meters_per_cell()
        } else {
            self.robot_radius_meters - self.meters_per_cell()
        }
    }

    pub fn add_move(&mut self, move_state: RobotMoveState) {
        self.move_state = move_state;
    }

    pub fn cumulative_alignment(&self) -> f64 {
        self.cumulative_alignment
    }

    pub fn has_unvisited_neighbor(&self, gp: &GridPoint) -> bool {
        for neighbor in gp.manhattan_neighbors() {
            if self.free_space.in_bounds(neighbor)
                && !self.free_space.is_set(neighbor)
                && !self.obstacles.is_set(neighbor)
            {
                return true;
            }
        }
        false
    }

    pub fn grid_size(&self) -> GridPoint {
        self.free_space.grid_size()
    }

    pub fn robot_grid_radius(&self) -> u64 {
        max(
            1,
            (self.robot_radius_meters / self.converter.meters_per_cell) as u64,
        )
    }

    pub fn path_to(&self, goal: FloatPoint) -> Option<Vec<FloatPoint>> {
        let start = self.free_space.meters2cell(self.position.unwrap().pos);
        let end = self.free_space.meters2cell(goal);
        let mut searcher = PrioritySearchIter::a_star(
            start,
            |p| {
                p.manhattan_neighbors()
                    .filter(|p| {
                        self.converter
                            .circle_grid_points(
                                self.converter.cell2meters(*p),
                                self.robot_radius_meters,
                            )
                            .all(|n| self.free_space.is_set(n) && !self.obstacles.is_set(n))
                    })
                    .map(|p| (p, 1))
                    .collect()
            },
            |p| end.manhattan_distance(*p),
        );
        let search_outcome = searcher.by_ref().inspect(|p| println!("Exploring {p} on the way to {end}")).find(|p| *p == end);
        match search_outcome {
            None => None,
            Some(p) => Some(
                searcher
                    .path_back_from(&p)
                    .iter()
                    .map(|p| self.converter.cell2meters(*p))
                    .collect(),
            ),
        }
    }

    pub fn free_space_within(
        &self,
        grid_row: u64,
        grid_col: u64,
        row_grid_slice: u64,
        col_grid_slice: u64,
    ) -> bool {
        within(
            &self.free_space,
            grid_row,
            grid_col,
            row_grid_slice,
            col_grid_slice,
        )
    }

    pub fn obstacle_within(
        &self,
        grid_row: u64,
        grid_col: u64,
        row_grid_slice: u64,
        col_grid_slice: u64,
    ) -> bool {
        within(
            &self.obstacles,
            grid_row,
            grid_col,
            row_grid_slice,
            col_grid_slice,
        )
    }

    pub fn estimate(&self) -> Option<RobotPose> {
        self.position
    }

    pub fn as_python_dict(&self) -> Option<String> {
        self.position.map(|p| {
            format!(
                "{{ 'x': {}, 
'y': {}, 
'theta': {}, 
'columns': {}, 
'rows': {}, 
'meters_per_cell': {},
'free_space_grid': {}, 
'obstacles_grid': {}}}",
                p.pos[0],
                p.pos[1],
                p.theta,
                //self.robot_radius_meters, // I should add this eventually.
                self.free_space.cols,
                self.free_space.rows,
                self.converter.meters_per_cell,
                vec2pyliststr(&self.free_space.bits.words()),
                vec2pyliststr(&self.obstacles.bits.words())
            )
        })
    }

    pub fn from_python_dict(pd: &str) -> Self {
        let mut result = TrajectoryBuilder::default().build();
        let mut x = None;
        let mut y = None;
        let mut theta = None;
        let mut columns = None;
        let mut rows = None;
        let mut meters_per_cell = None;
        let parsed = PythonMapParser::parse(Rule::dictionary, pd.trim()).unwrap();
        for pair in parsed {
            for a in pair.into_inner() {
                let mut a_iter = a.into_inner();
                let key = a_iter.next().unwrap().as_str();
                let value = a_iter.next().unwrap().as_str();
                match key {
                    "'x'" => x = Some(value.parse::<f64>().unwrap()),
                    "'y'" => y = Some(value.parse::<f64>().unwrap()),
                    "'theta'" => theta = Some(value.parse::<f64>().unwrap()),
                    "'robot_radius_meters'" => {
                        result.robot_radius_meters = value.parse::<f64>().unwrap()
                    }
                    "'columns'" => columns = Some(value.parse::<u64>().unwrap()),
                    "'rows'" => rows = Some(value.parse::<u64>().unwrap()),
                    "'meters_per_cell'" => meters_per_cell = Some(value.parse::<f64>().unwrap()),
                    "'free_space_grid'" => {
                        result.free_space = BinaryGrid::from_python_dict(
                            columns.unwrap(),
                            rows.unwrap(),
                            meters_per_cell.unwrap(),
                            value,
                        )
                    }
                    "'obstacles_grid'" => {
                        result.obstacles = BinaryGrid::from_python_dict(
                            columns.unwrap(),
                            rows.unwrap(),
                            meters_per_cell.unwrap(),
                            value,
                        )
                    }
                    _ => panic!("Unrecognized Python dictionary key: \"{key}\""),
                }
            }
        }

        result.position = Some(RobotPose {
            pos: FloatPoint::new([x.unwrap(), y.unwrap()]),
            theta: theta.unwrap(),
        });
        result
    }
}

/// Indicates how compatible the current move is with the map.
/// Bigger numbers are better.
fn current_move_alignment(map: &TrajectoryMap) -> f64 {
    match map.position {
        None => 0.0,
        Some(pose) => {
            (match map
                .obstacles
                .closest_1_to(map.obstacles.meters2cell(pose.pos))
            {
                None => match map.move_state {
                    RobotMoveState::Forward => map.robot_grid_radius(),
                    RobotMoveState::Turning => 0,
                },
                Some(d) => {
                    if d <= map.robot_grid_radius() {
                        d
                    } else {
                        match map.move_state {
                            RobotMoveState::Forward => map.robot_grid_radius(),
                            RobotMoveState::Turning => {
                                if d <= 2 * map.robot_grid_radius() {
                                    2 * map.robot_grid_radius() - d
                                } else {
                                    0
                                }
                            }
                        }
                    }
                }
            }) as f64
        }
    }
}

fn within(
    grid: &BinaryGrid,
    grid_row: u64,
    grid_col: u64,
    row_grid_slice: u64,
    col_grid_slice: u64,
) -> bool {
    for r in grid_row..grid_row + row_grid_slice {
        for c in grid_col..grid_col + col_grid_slice {
            if grid.is_set(GridPoint::new([c, r])) {
                return true;
            }
        }
    }
    false
}

fn vec2pyliststr<T: Display>(v: &Vec<T>) -> String {
    let strs = v.iter().map(|t| format!("{t}")).collect::<Vec<_>>();
    format!("[{}]", strs.join(","))
}

#[derive(Copy, Clone, Debug)]
pub struct GridPointConverter {
    meters_per_cell: f64,
    width: f64,
    height: f64,
}

impl GridPointConverter {
    pub fn new(meters_per_cell: f64, width: f64, height: f64) -> Self {
        Self {
            meters_per_cell,
            width,
            height,
        }
    }

    pub fn cell2meters(&self, cell: GridPoint) -> FloatPoint {
        (FloatPoint::new([cell[0] as f64, cell[1] as f64]) * self.meters_per_cell) - self.offset()
    }

    pub fn meters2cell(&self, meters: FloatPoint) -> GridPoint {
        let scaled = (meters + self.offset()) / self.meters_per_cell;
        GridPoint::new([scaled[0] as u64, scaled[1] as u64])
    }

    fn offset(&self) -> FloatPoint {
        FloatPoint::new([self.width, self.height]) / 2.0
    }

    pub fn circle_grid_points(
        &self,
        center: FloatPoint,
        radius: f64,
    ) -> impl Iterator<Item = GridPoint> {
        let radius_offset = FloatPoint::of(radius);
        let start = center - radius_offset;
        let end = center + radius_offset;
        let grid_start = self.meters2cell(start);
        let grid_end = self.meters2cell(end);
        (grid_start[0]..=grid_end[0])
            .cartesian_product(grid_start[1]..=grid_end[1])
            .map(|(col, row)| GridPoint::new([col, row]))
            .filter(move |gp| self.cell2meters(*gp).euclidean_distance(center) <= radius)
    }
}

/*
Details
* Reset odom before starting
* Center is (0, 0) in odom space
* Grid coordinates are positive; (0, 0) is lower left corner
 */
#[derive(Clone, Debug)]
pub struct BinaryGrid {
    rows: u64,
    cols: u64,
    converter: GridPointConverter,
    bits: BitArray,
}

impl Display for BinaryGrid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for row in 0..self.rows {
            for col in 0..self.cols {
                let value = if self.is_set(GridPoint::new([col, row])) {
                    1
                } else {
                    0
                };
                write!(f, "{value}")?;
            }
            write!(f, "\n")?;
        }
        Ok(())
    }
}

impl BinaryGrid {
    pub fn new(width: f64, height: f64, meters_per_cell: f64) -> Self {
        let cols = max(1, (width / meters_per_cell) as u64);
        let rows = max(1, (height / meters_per_cell) as u64);
        let converter = GridPointConverter::new(meters_per_cell, width, height);
        Self {
            rows,
            cols,
            converter,
            bits: BitArray::zeros(rows * cols),
        }
    }

    pub fn blank_copy(&self) -> Self {
        Self::new(
            self.converter.width,
            self.converter.height,
            self.converter.meters_per_cell,
        )
    }

    pub fn from_python_dict(cols: u64, rows: u64, meters_per_cell: f64, ints: &str) -> Self {
        let width = cols as f64 * meters_per_cell;
        let height = rows as f64 * meters_per_cell;
        let mut bits = BitArray::default();
        for value in ints[1..ints.len() - 1].split(",") {
            let value = value.trim().parse::<u64>().unwrap();
            bits.push_word(value);
        }
        Self {
            rows,
            cols,
            converter: GridPointConverter::new(meters_per_cell, width, height),
            bits,
        }
    }

    pub fn all_1s_iter(&self) -> impl Iterator<Item = GridPoint> {
        (0..self.cols)
            .zip(0..self.rows)
            .map(|(col, row)| GridPoint::new([col, row]))
            .filter(|gp| self.is_set(*gp))
    }

    pub fn all_1s(&self) -> Vec<GridPoint> {
        let mut result = vec![];
        for col in 0..self.cols {
            for row in 0..self.rows {
                let g = GridPoint::new([col, row]);
                if self.is_set(g) {
                    result.push(g);
                }
            }
        }
        result
    }

    /// Performs breadth-first search to give the Manhattan distance
    /// to the closest 1 to `gp`
    pub fn closest_1_to(&self, gp: GridPoint) -> Option<u64> {
        let mut visited = self.blank_copy();
        let mut queue = VecDeque::new();
        queue.push_back((0, gp));
        while let Some((distance, point)) = queue.pop_front() {
            if self.in_bounds(point) && !visited.is_set(gp) {
                visited.set(gp, true);
                if self.is_set(gp) {
                    return Some(distance);
                } else {
                    for neighbor in gp.manhattan_neighbors() {
                        queue.push_back((distance + 1, neighbor));
                    }
                }
            }
        }
        None
    }

    pub fn in_bounds(&self, gp: GridPoint) -> bool {
        self.ind(gp).is_some()
    }

    pub fn grid_size(&self) -> GridPoint {
        GridPoint::new([self.cols, self.rows])
    }

    pub fn is_set(&self, gp: GridPoint) -> bool {
        self.ind(gp).map_or(false, |i| self.bits.is_set(i))
    }

    pub fn set(&mut self, gp: GridPoint, value: bool) {
        self.ind(gp).map(|i| self.bits.set(i, value));
    }

    fn ind(&self, gp: GridPoint) -> Option<u64> {
        if gp[0] < self.cols && gp[1] < self.rows {
            Some(gp[0] + gp[1] * self.cols)
        } else {
            None
        }
    }

    pub fn cell2meters(&self, cell: GridPoint) -> FloatPoint {
        self.converter.cell2meters(cell)
    }

    pub fn meters2cell(&self, meters: FloatPoint) -> GridPoint {
        self.converter.meters2cell(meters)
    }

    pub fn set_circle(&mut self, center: FloatPoint, radius: f64) {
        let converter = self.converter;
        let circle_points = converter.circle_grid_points(center, radius);
        for circle_point in circle_points {
            self.set(circle_point, true);
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{BinaryGrid, TrajectoryMap, point::FloatPoint};

    const CIRCLE_1_STR: &str = "00000000000000000000
00000000100000000000
00000011111000000000
00000011111000000000
00000111111100000000
00000011111000000000
00000011111000000000
00000000100000000000
";

    #[test]
    fn test1() {
        let mut grid = BinaryGrid::new(10.0, 4.0, 0.5);
        let dim = grid.grid_size();
        assert_eq!(dim[0], 20);
        assert_eq!(dim[1], 8);
        grid.set_circle(FloatPoint::new([-1.0, 0.0]), 1.5);
        println!("{grid}");
        assert_eq!(format!("{grid}"), CIRCLE_1_STR);
    }

    const CIRCLE_2_STR: &str = "00100\n00000\n";

    #[test]
    fn test2() {
        let mut grid = BinaryGrid::new(10.0, 4.0, 2.0);
        let dim = grid.grid_size();
        assert_eq!(dim[0], 5);
        assert_eq!(dim[1], 2);
        grid.set_circle(FloatPoint::new([-1.0, 0.0]), 1.5);
        println!("{grid}");
        assert_eq!(format!("{grid}"), CIRCLE_2_STR);
    }

    const CIRCLE_3_STR: &str = "0000000000
0001110000
0001110000
0001110000
";

    #[test]
    fn test3() {
        let mut grid = BinaryGrid::new(10.0, 4.0, 1.0);
        let dim = grid.grid_size();
        assert_eq!(dim[0], 10);
        assert_eq!(dim[1], 4);
        grid.set_circle(FloatPoint::new([-1.0, 0.0]), 1.5);
        println!("{grid}");
        assert_eq!(format!("{grid}"), CIRCLE_3_STR);
    }

    const EDGE_STR: &str = "0000000001
0000000001
0000000000
0000000000
";

    #[test]
    fn test_edge() {
        let mut grid = BinaryGrid::new(10.0, 4.0, 1.0);
        grid.set_circle(FloatPoint::new([5.0, -1.5]), 1.5);
        println!("{grid}");
        assert_eq!(format!("{grid}"), EDGE_STR);
    }

    #[test]
    fn test_parse() {
        let map_input_str = std::fs::read_to_string("first_improved_map_reformatted").unwrap();
        let map = TrajectoryMap::from_python_dict(map_input_str.as_str());
        assert_eq!(map_input_str, map.as_python_dict().unwrap());
    }
}
