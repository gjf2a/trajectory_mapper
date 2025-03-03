use common_macros::hash_map;
use num::Integer;
use priority_queue::PriorityQueue;
use std::collections::{HashMap, VecDeque};
use std::fmt::Debug;
use std::hash::Hash;
use std::ops::Add;
use trait_set::trait_set;

trait_set! {
    pub trait SearchNode = Clone + Hash + Eq + Debug;
}

pub struct BfsIter<T: SearchNode, S: FnMut(&T) -> Vec<T>> {
    queue: VecDeque<(T, usize)>,
    depths: HashMap<T, usize>,
    parents: HashMap<T, Option<T>>,
    successor: S,
}

impl<T: SearchNode, S: FnMut(&T) -> Vec<T>> BfsIter<T, S> {
    pub fn new(start: T, successor: S) -> Self {
        let mut queue = VecDeque::new();
        queue.push_back((start.clone(), 0));
        Self {
            queue,
            depths: hash_map!(start.clone() => 0),
            successor,
            parents: hash_map!(start.clone() => None),
        }
    }

    pub fn multi_start<I: Iterator<Item = T>>(starts: I, successor: S) -> Self {
        let queue = starts.map(|t| (t, 0)).collect::<VecDeque<_>>();
        let depths = queue.iter().cloned().collect();
        let parents = queue.iter().map(|(t, _)| (t.clone(), None)).collect();
        Self {
            queue,
            depths,
            successor,
            parents,
        }
    }

    pub fn path_back_from(&self, node: &T) -> VecDeque<T> {
        path_back_from(node, &self.parents)
    }

    pub fn depth_for(&self, node: &T) -> usize {
        self.depths.get(node).copied().unwrap()
    }

    pub fn all_depths(&self) -> HashMap<T, usize> {
        self.depths.clone()
    }
}

impl<T: SearchNode, S: FnMut(&T) -> Vec<T>> Iterator for BfsIter<T, S> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        self.queue.pop_front().map(|(parent, depth)| {
            for child in (self.successor)(&parent) {
                if !self.depths.contains_key(&child) {
                    self.depths.insert(child.clone(), depth + 1);
                    self.parents.insert(child.clone(), Some(parent.clone()));
                    self.queue.push_back((child, depth + 1));
                }
            }
            parent
        })
    }
}

fn path_back_from<T: SearchNode>(node: &T, parents: &HashMap<T, Option<T>>) -> VecDeque<T> {
    let mut result = VecDeque::new();
    let mut current = node;
    result.push_back(current.clone());
    while let Some(parent) = parents.get(current).unwrap() {
        result.push_back(parent.clone());
        current = parent;
    }
    result
}

trait_set! {
    pub trait Estimator = Integer + Copy + Clone + Add<Output=Self> + PartialOrd + Ord + Debug + Default
}

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default, Ord)]
struct TotalEstimate<N: Estimator> {
    from_start: N,
    estimate_to_goal: N,
}

impl<N: Estimator> TotalEstimate<N> {
    fn next_cost(&self, step_cost: N, estimate_to_goal: N) -> Self {
        Self {
            from_start: self.from_start + step_cost,
            estimate_to_goal,
        }
    }

    fn total(&self) -> N {
        self.from_start + self.estimate_to_goal
    }
}

impl<N: Estimator> PartialOrd for TotalEstimate<N> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        other.total().partial_cmp(&self.total())
    }
}

pub struct PrioritySearchIter<
    N: Estimator,
    T: SearchNode,
    S: FnMut(&T) -> Vec<(T, N)>,
    H: Fn(&T) -> N,
> {
    queue: PriorityQueue<T, TotalEstimate<N>>,
    costs: HashMap<T, N>,
    parents: HashMap<T, Option<T>>,
    successor: S,
    heuristic: H,
}

impl<N: Estimator, T: SearchNode, S: FnMut(&T) -> Vec<(T, N)>, H: Fn(&T) -> N>
    PrioritySearchIter<N, T, S, H>
{
    pub fn a_star(start: T, successor: S, heuristic: H) -> Self {
        let mut queue = PriorityQueue::new();
        queue.push(start.clone(), TotalEstimate::default());
        Self {
            queue,
            costs: hash_map!(start.clone() => N::zero()),
            successor,
            parents: hash_map!(start.clone() => None),
            heuristic,
        }
    }

    pub fn path_back_from(&self, node: &T) -> VecDeque<T> {
        path_back_from(node, &self.parents)
    }

    pub fn cost_for(&self, node: &T) -> N {
        self.costs.get(node).copied().unwrap()
    }
}

impl<N: Estimator, T: SearchNode, S: FnMut(&T) -> Vec<(T, N)>>
    PrioritySearchIter<N, T, S, fn(&T) -> N>
{
    pub fn dijkstra(start: T, successor: S) -> Self {
        Self::a_star(start, successor, |_| N::zero())
    }
}

impl<N: Estimator, T: SearchNode, S: FnMut(&T) -> Vec<(T, N)>, H: Fn(&T) -> N> Iterator
    for PrioritySearchIter<N, T, S, H>
{
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        self.queue.pop().map(|(parent, cost)| {
            self.costs.insert(parent.clone(), cost.from_start);
            for (child, step_cost) in (self.successor)(&parent) {
                if !self.costs.contains_key(&child) {
                    let new_priority = cost.next_cost(step_cost, (self.heuristic)(&child));
                    match self.queue.get_priority(&child) {
                        Some(priority) => {
                            if new_priority > *priority {
                                self.parents.insert(child.clone(), Some(parent.clone()));
                                self.queue.change_priority(&child, new_priority);
                            }
                        }
                        None => {
                            self.parents.insert(child.clone(), Some(parent.clone()));
                            self.queue.push(child, new_priority);
                        }
                    }
                }
            }
            parent
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::{TrajectoryMap, point::FloatPoint};

    #[test]
    fn test_a_star() {
        let map = std::fs::read_to_string("first_improved_map").unwrap();
        let map = TrajectoryMap::from_python_dict(map.as_str());
        println!("{:?}", map.position);
        let goal = FloatPoint::new([1.5, -0.4]);
        let route = map.path_to(goal);
        assert!(route.is_some());
    }
}

/*

// At some point, I'll want to pay down some technical debt by rewriting these
// unit tests.

#[cfg(test)]
mod tests {
    use enum_iterator::all;

    use crate::{
        grid::GridCharWorld,
        multidim::{DirType, ManhattanDir, Position},
        search_iter::{path_back_from, BfsIter, PrioritySearchIter},
    };

    #[test]
    fn test_bfs() {
        println!("Test BFS");
        let max_dist = 2;
        let start = Position::default();
        println!("Starting BFS");
        let mut searcher = BfsIter::new(start, |n| {
            all::<ManhattanDir>()
                .map(move |d| d.neighbor(*n))
                .filter(|p| start.manhattan_distance(p) <= max_dist)
                .collect()
        });
        searcher.by_ref().last();
        println!("Search complete.");
        assert_eq!(searcher.parents.len(), 13);

        for node in searcher.parents.keys() {
            let len = path_back_from(node, &searcher.parents).len();
            println!("From {:?}: {}", node, len);
            assert!(len <= 1 + max_dist as usize);
        }
    }

    #[test]
    fn test_priority_bfs() {
        println!("Test Priority BFS");
        let max_dist = 2;
        let start = Position::default();
        println!("Starting BFS");
        let mut searcher = PrioritySearchIter::dijkstra(start, |n| {
            all::<ManhattanDir>()
                .map(|d| d.neighbor(*n))
                .filter(|p| p.manhattan_distance(&start) <= max_dist)
                .map(|p| (p, 1))
                .collect()
        });
        searcher.by_ref().last();
        println!("Search complete.");
        assert_eq!(searcher.parents.len(), 13);

        for node in searcher.parents.keys() {
            let len = path_back_from(node, &searcher.parents).len();
            println!("From {:?}: {}", node, len);
            assert!(len <= 1 + max_dist as usize);
        }
    }

    #[test]
    fn test_priority_a_star() {
        let maze_str = ".....##
###.###
#.....#
#.#.#.#
#...#..";
        let maze = maze_str.parse::<GridCharWorld>().unwrap();
        println!("{maze}");
        let start = Position::default();
        let exit = Position::from((
            maze.width() as isize - 1,
            maze.height() as isize - 1 as isize,
        ));
        println!("exit: {exit}");
        let mut searcher = PrioritySearchIter::a_star(
            start,
            |p| {
                all::<ManhattanDir>()
                    .map(|d| d.neighbor(*p))
                    .filter(|c| maze.value(*c).map_or(false, |v| v == '.'))
                    .map(|p| (p, 1))
                    .collect()
            },
            |p| exit.manhattan_distance(p),
        );
        let result = searcher.by_ref().find(|p| *p == exit).unwrap();
        assert_eq!(result, exit);
        let path = searcher.path_back_from(&result);
        println!("{path:?}");
    }
}
*/
