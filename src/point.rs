use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

use num_traits::{cast::ToPrimitive, Num};
use trait_set::trait_set;

trait_set! {
    pub trait NumType = ToPrimitive + Default + Num + Copy + AddAssign + SubAssign + MulAssign + DivAssign;
}

pub type GridPoint = Point<u64, 2>;
pub type FloatPoint = Point<f64, 2>;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Point<N: NumType, const S: usize> {
    coords: [N; S],
}

impl<N: NumType, const S: usize> Point<N, S> {
    pub fn new(coords: [N; S]) -> Self {
        Self { coords }
    }

    pub fn of(value: N) -> Self {
        Self { coords: [value; S] }
    }

    pub fn euclidean_distance(&self, other: Point<N, S>) -> f64 {
        (0..S)
            .map(|i| ((self[i] - other[i]).to_f64().expect("Shouldn't happen")).powf(2.0))
            .sum::<f64>()
            .sqrt()
    }
}

impl<N: NumType, const S: usize> Index<usize> for Point<N, S> {
    type Output = N;

    fn index(&self, index: usize) -> &Self::Output {
        &self.coords[index]
    }
}

impl<N: NumType, const S: usize> IndexMut<usize> for Point<N, S> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.coords[index]
    }
}

impl<N: NumType, const S: usize> Default for Point<N, S> {
    fn default() -> Self {
        Self {
            coords: [N::default(); S],
        }
    }
}

impl<N: NumType, const S: usize> Add for Point<N, S> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let mut result = self;
        result += rhs;
        result
    }
}

impl<N: NumType, const S: usize> AddAssign for Point<N, S> {
    fn add_assign(&mut self, rhs: Self) {
        for i in 0..S {
            self[i] += rhs[i];
        }
    }
}

impl<N: NumType + Neg<Output = N>, const S: usize> Neg for Point<N, S> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..S {
            result[i] = -self[i];
        }
        result
    }
}

impl<N: NumType + Neg<Output = N>, const S: usize> Sub for Point<N, S> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        self + -rhs
    }
}

impl<N: NumType + Neg<Output = N>, const S: usize> SubAssign for Point<N, S> {
    fn sub_assign(&mut self, rhs: Self) {
        *self += -rhs;
    }
}

impl<N: NumType, const S: usize> Mul<N> for Point<N, S> {
    type Output = Self;

    fn mul(self, rhs: N) -> Self::Output {
        let mut result = self;
        result *= rhs;
        result
    }
}

impl<N: NumType, const S: usize> MulAssign<N> for Point<N, S> {
    fn mul_assign(&mut self, rhs: N) {
        for i in 0..S {
            self[i] *= rhs;
        }
    }
}

impl<N: NumType, const S: usize> DivAssign<N> for Point<N, S> {
    fn div_assign(&mut self, rhs: N) {
        for i in 0..S {
            self[i] /= rhs;
        }
    }
}

impl<N: NumType, const S: usize> Div<N> for Point<N, S> {
    type Output = Self;

    fn div(self, rhs: N) -> Self::Output {
        let mut result = self;
        result /= rhs;
        result
    }
}
