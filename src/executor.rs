use std::collections::VecDeque;

use itertools::Itertools;

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

    pub fn full_path_copy(&self) -> String {
        format!("[{}]", self.path.iter().map(|p| format!{"{p}"}).join(","))
    }

    pub fn advance(&mut self) {
        self.path.pop_front();
    }

    pub fn make_plan(&mut self, goal: FloatPoint) {
        if let Some(path) = self.map.path_to(goal) {
            self.path = path;
        }
    }

    pub fn goal(&self) -> Option<FloatPoint> {
        self.path.back().copied()
    }
}