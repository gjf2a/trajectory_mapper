use itertools::Itertools;

fn main() {
    let zipped = (0..3).cartesian_product(0..2).collect::<Vec<_>>();
    println!("{zipped:?}");
}
