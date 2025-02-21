use distribution_select::Distribution;

use crate::{RobotMoveState, RobotPose, TrajectoryMap};

#[derive(Clone, Debug)]
pub struct ParticleFilter<N: Clone + Fn(RobotPose) -> RobotPose, F: Clone + Fn(&TrajectoryMap) -> f64> {
    particles: Vec<TrajectoryMap>,
    best_particle: TrajectoryMap,
    noise_func: N,
    fitness_func: F,
}

impl<N: Clone + Fn(RobotPose) -> RobotPose, F: Clone + Fn(&TrajectoryMap) -> f64> ParticleFilter<N, F> {
    pub fn new(initial_map: &TrajectoryMap, num_particles: usize, noise_func: N, fitness_func: F) -> Self {
        Self {
            particles: std::iter::repeat(initial_map.clone()).take(num_particles).collect(),
            best_particle: initial_map.clone(),
            noise_func,
            fitness_func
        }
    }

    pub fn iterate(&mut self, pose: RobotPose, move_state: RobotMoveState) {
        self.resample();
        self.add_measurement(pose, move_state);
    }

    fn resample(&mut self) {
        let mut distro = Distribution::new();
        let mut best_weight = 0.0;
        let mut best_i = 0;
        for (i, particle) in self.particles.iter().enumerate() {
            let weight = (self.fitness_func)(particle);
            if weight > best_weight {
                best_weight = weight;
                best_i = i;
            }
            distro.add(&i, weight + 1.0);
        }
        
        self.best_particle = self.particles[best_i].clone();

        let mut new_particles = vec![];
        for _ in 0..self.particles.len() {
            let choice = distro.random_pick();
            new_particles.push(self.particles[choice].clone());
        }

        std::mem::swap(&mut new_particles, &mut self.particles);
    }

    fn add_measurement(&mut self, pose: RobotPose, move_state: RobotMoveState) {
        for particle in self.particles.iter_mut() {
            particle.add_move(move_state);
            particle.add_pose((self.noise_func)(pose));
        }
    }
}