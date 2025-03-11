use std::collections::VecDeque;

use crate::{RobotPose, TrajectoryMap, point::FloatPoint, point_list_str};

pub struct PathPlanExecutor {
    map: TrajectoryMap,
    path: VecDeque<FloatPoint>,
    last_odom_reading: Option<RobotPose>,
    additional_obstacles: Vec<FloatPoint>,
}

impl PathPlanExecutor {
    pub fn new(map: TrajectoryMap) -> Self {
        Self {
            map,
            path: VecDeque::default(),
            last_odom_reading: None,
            additional_obstacles: Vec::new(),
        }
    }

    pub fn waypoint(&self) -> Option<FloatPoint> {
        self.path.front().copied()
    }

    pub fn struck_obstacle(&mut self) {
        if let Some(last_pose) = self.last_odom_reading {
            self.additional_obstacles
                .push(self.map.add_obstacle(last_pose));
        }
    }

    pub fn full_path_copy(&self) -> String {
        point_list_str(self.path.iter().copied())
    }

    pub fn additional_obstacles(&self) -> Option<String> {
        if self.additional_obstacles.len() > 0 {
            Some(point_list_str(self.additional_obstacles.iter().copied()))
        } else {
            None
        }
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
