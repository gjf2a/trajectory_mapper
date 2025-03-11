use std::collections::VecDeque;

use itertools::Itertools;

use crate::{RobotPose, TrajectoryMap, point::FloatPoint};

pub struct PathPlanExecutor {
    map: TrajectoryMap,
    path: VecDeque<FloatPoint>,
    last_odom_reading: Option<RobotPose>,
}

impl PathPlanExecutor {
    pub fn new(map: TrajectoryMap) -> Self {
        Self {
            map,
            path: VecDeque::default(),
            last_odom_reading: None,
        }
    }

    pub fn waypoint(&self) -> Option<FloatPoint> {
        self.path.front().copied()
    }

    pub fn full_path_copy(&self) -> String {
        format!(
            "[{}]",
            self.path
                .iter()
                .map(|p| format!("({:.2}, {:.2})", p[0], p[1]))
                .join(",")
        )
    }

    pub fn advance(&mut self) {
        self.path.pop_front();
    }

    pub fn make_plan(&mut self, goal: FloatPoint) -> bool {
        if let Some(current) = self.last_odom_reading {
            if let Some(path) = self.map.path_to(current.pos, goal) {
                self.path = path;
                return true;
            }
        }
        false
    }

    pub fn goal(&self) -> Option<FloatPoint> {
        self.path.back().copied()
    }

    pub fn odom_reading(&mut self, pose: RobotPose) {
        self.last_odom_reading = Some(pose)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_generated_path() {}
}
