use std::{
    cmp::max,
    fmt::Display,
    fs::File,
    io::{BufRead, BufReader},
};

use anyhow::{anyhow, Context};
use bits::BitArray;
use point::{FloatPoint, GridPoint};
use r2r::{geometry_msgs::msg::TwistStamped, nav_msgs::msg::Odometry};

pub mod point;

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
            let line = line?.replace("(", "").replace(")", "").replace(",", "");
            let text = line.split_whitespace().collect::<Vec<_>>();
            let state = if text[3] == "Forward" {
                RobotMoveState::Forward
            } else {
                RobotMoveState::Turning
            };
            let parts = &text[..3]
                .iter()
                .map(|s| s.parse::<f64>())
                .collect::<Result<Vec<_>, _>>()
                .context("Parse error with robot coordinates")?;
            if parts.len() == 3 {
                let pose = RobotPose {
                    pos: FloatPoint::new([parts[0], parts[1]]),
                    theta: parts[2],
                };
                points.push((pose, state));
            } else {
                return Err(anyhow!("Illegal number of points: {}", parts.len()));
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
        let (q1, q2, q3, q0) = (
            value.pose.pose.orientation.x,
            value.pose.pose.orientation.y,
            value.pose.pose.orientation.z,
            value.pose.pose.orientation.w,
        );
        result.theta =
            (q0 * q3 + q1 * q2).atan2(q0.powf(2.0) + q1.powf(2.0) - q2.powf(2.0) - q3.powf(2.0));
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
            meters_per_cell: 0.5,
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
            num_points: 0,
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
    num_points: u64,
}

impl TrajectoryMap {
    pub fn add_pose(&mut self, pose: RobotPose) {
        self.num_points += 1;
        match self.move_state {
            RobotMoveState::Forward => {
                self.turn_in_progress = false;
                self.position = Some(pose);
                self.free_space
                    .set_circle(pose.pos, self.robot_radius_meters);
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
    }

    pub fn num_points(&self) -> u64 {self.num_points}

    pub fn grid_size(&self) -> GridPoint {
        self.free_space.grid_size()
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

    pub fn add_move(&mut self, move_state: RobotMoveState) {
        self.move_state = move_state;
    }

    pub fn estimate(&self) -> Option<RobotPose> {
        self.position
    }

    pub fn min_x_y_max_x_y(&self) -> (f64, f64, f64, f64) {
        todo!()
    }

    pub fn as_python_dict(&self) -> Option<String> {
        self.position.map(|p| {
            format!(
                "{{ 'x': {}, 'y': {}, 'theta': {}, 
'columns': {}, 'rows': {}, 'meters_per_cell': {},
'free_space': {}, 'obstacles': {}}}",
                p.pos[0],
                p.pos[1],
                p.theta,
                self.free_space.cols,
                self.free_space.rows,
                self.free_space.meters_per_cell,
                vec2pyliststr(&self.free_space.bits.words()),
                vec2pyliststr(&self.obstacles.bits.words())
            )
        })
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
    width: f64,
    height: f64,
    meters_per_cell: f64,
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
        Self {
            rows,
            cols,
            width,
            height,
            meters_per_cell,
            bits: BitArray::zeros(rows * cols),
        }
    }

    pub fn grid_size(&self) -> GridPoint {
        GridPoint::new([self.cols, self.rows])
    }

    pub fn is_set(&self, i: GridPoint) -> bool {
        self.ind(i).map_or(false, |i| self.bits.is_set(i))
    }

    pub fn set(&mut self, i: GridPoint, value: bool) {
        self.ind(i).map(|i| self.bits.set(i, value));
    }

    fn ind(&self, p: GridPoint) -> Option<u64> {
        if p[0] < self.cols && p[1] < self.rows {
            Some(p[0] + p[1] * self.cols)
        } else {
            None
        }
    }

    fn offset(&self) -> FloatPoint {
        FloatPoint::new([self.width, self.height]) / 2.0
    }

    pub fn cell2meters(&self, cell: GridPoint) -> FloatPoint {
        (FloatPoint::new([cell[0] as f64, cell[1] as f64]) * self.meters_per_cell) - self.offset()
    }

    pub fn meters2cell(&self, meters: FloatPoint) -> GridPoint {
        let scaled = (meters + self.offset()) / self.meters_per_cell;
        GridPoint::new([scaled[0] as u64, scaled[1] as u64])
    }

    pub fn set_circle(&mut self, center: FloatPoint, radius: f64) {
        let radius_offset = FloatPoint::of(radius);
        let start = center - radius_offset;
        let end = center + radius_offset;
        let grid_start = self.meters2cell(start);
        let grid_end = self.meters2cell(end);
        println!("grid_start: {grid_start}\tgrid_end: {grid_end}");
        for x_grid in grid_start[0]..=grid_end[0] {
            for y_grid in grid_start[1]..=grid_end[1] {
                let g = GridPoint::new([x_grid, y_grid]);
                let pt = self.cell2meters(g);
                println!(
                    "g: {g} pt: {pt} center: {center} distance: {} radius: {radius}",
                    pt.euclidean_distance(center)
                );
                if pt.euclidean_distance(center) <= radius {
                    self.set(g, true);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{point::FloatPoint, BinaryGrid};

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
}
