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

#[derive(Clone, Default, Debug)]
pub struct Trajectory {
    path: Vec<RobotPose>,
}

impl Trajectory {
    pub fn add(&mut self, pose: RobotPose) {
        self.path.push(pose)
    }

    pub fn estimate(&self) -> Option<RobotPose> {
        self.path.last().copied()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test() {}
}
