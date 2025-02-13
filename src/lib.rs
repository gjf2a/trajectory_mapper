use std::fmt::Display;

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
}

impl TrajectoryMap {
    pub fn add_pose(&mut self, pose: RobotPose) {
        match self.move_state {
            RobotMoveState::Forward => {
                self.turn_in_progress = false;
                self.position = Some(pose);
                self.free_space
                    .set_circle(pose.pos, self.robot_radius_meters, true);
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

    pub fn add_move(&mut self, move_state: RobotMoveState) {
        self.move_state = move_state;
    }

    pub fn estimate(&self) -> Option<RobotPose> {
        self.position
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

fn vec2pyliststr<T: Display>(v: &Vec<T>) -> String {
    let strs = v.iter().map(|t| format!("{t}")).collect::<Vec<_>>();
    format!("[{}]", strs.join(","))
}

/*
Strategy
* Maintain a free-space map
* All planning happens within that context
* Obstacles are not explicitly represented
* Particle filter observations
  * "there should be space here but I turned"
  * Those particles get eliminated

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

impl BinaryGrid {
    pub fn new(width: f64, height: f64, meters_per_cell: f64) -> Self {
        let rows = (width / meters_per_cell) as u64;
        let cols = (height / meters_per_cell) as u64;
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
        self.bits.is_set(self.ind(i))
    }

    pub fn set(&mut self, i: GridPoint, value: bool) {
        self.bits.set(self.ind(i), value);
    }

    fn ind(&self, p: GridPoint) -> u64 {
        p[0] + p[1] * self.cols
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

    pub fn set_circle(&mut self, center: FloatPoint, radius: f64, value: bool) {
        let radius_offset = FloatPoint::of(radius);
        let start = center - radius_offset;
        let end = center + radius_offset;
        let grid_start = self.meters2cell(start);
        let grid_end = self.meters2cell(end);
        for x_grid in grid_start[0]..=grid_end[0] {
            for y_grid in grid_start[1]..=grid_end[1] {
                let g = GridPoint::new([x_grid, y_grid]);
                let pt = self.cell2meters(g);
                self.set(g, value == (pt.euclidean_distance(center) <= radius));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test() {}
}
