use std::collections::VecDeque;

use crate::{point::FloatPoint, TrajectoryMap};

pub struct PathPlanExecutor {
    map: TrajectoryMap,
    path: VecDeque<FloatPoint>,
}

impl PathPlanExecutor {
    pub fn new(map: TrajectoryMap) -> Self {
        Self {map, path: VecDeque::default()}
    }

    pub fn waypoint(&self) -> Option<FloatPoint> {
        self.path.front().copied()
    }

    pub fn advance(&mut self) {
        self.path.pop_front();
    }

    pub fn make_plan(&mut self, goal: FloatPoint) {
        if let Some(path) = self.map.path_to(goal) {
            self.path = path;
        }
    }
}