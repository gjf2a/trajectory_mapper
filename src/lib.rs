use bits::BitArray;
use r2r::nav_msgs::msg::Odometry;

#[derive(Copy, Clone, PartialEq, Debug, Default)]
pub struct RobotPose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl From<Odometry> for RobotPose {
    fn from(value: Odometry) -> Self {
        let mut result = Self::default();
        result.x = value.pose.pose.position.x;
        result.y = value.pose.pose.position.y;
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

pub struct TrajectoryBuilder {
    robot_radius_meters: f64,
}

impl Default for TrajectoryBuilder {
    fn default() -> Self {
        Self { robot_radius_meters: 0.35 }
    }
}

impl TrajectoryBuilder {
    pub fn robot_radius(&mut self, robot_radius_meters: f64) -> &mut Self {
        self.robot_radius_meters = robot_radius_meters;
        self
    }

    pub fn build(&self) -> Trajectory {
        let mut result = Trajectory::default();
        result.robot_radius_meters = self.robot_radius_meters;
        result
    }
}

#[derive(Clone, Default, Debug)]
pub struct Trajectory {
    path: Vec<RobotPose>,
    robot_radius_meters: f64,
}

impl Trajectory {
    pub fn add(&mut self, pose: RobotPose) {
        self.path.push(pose)
    }

    pub fn estimate(&self) -> Option<RobotPose> {
        self.path.last().copied()
    }
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
pub struct OccupancyMap {
    rows: u64,
    cols: u64,
    width: f64,
    height: f64,
    meters_per_cell: f64,
    bits: BitArray,
}

impl OccupancyMap {
    pub fn new(width: f64, height: f64, meters_per_cell: f64) -> Self {
        let rows = (width / meters_per_cell) as u64;
        let cols = (height / meters_per_cell) as u64;
        Self {
            rows, cols, width, height, meters_per_cell, bits: BitArray::zeros(rows * cols)
        }
    }

    fn cell2meters(&self, cell: (u64, u64)) -> (f64, f64) {
        todo!()
    }

    fn meters2cell(&self, meters: (f64, f64)) -> (u64, u64) {
        todo!()
    }

    pub fn set_circle(&mut self, center: (f64, f64), radius: f64, value: bool) {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test() {}
}
