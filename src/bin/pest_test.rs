use pest::Parser;
use pest_derive::Parser;

#[derive(Parser)]
#[grammar = "map_python_dictionary.pest"]
struct PythonMapParser;

fn main() {
    let input = std::fs::read_to_string("first_improved_map").unwrap();
    let parsed = PythonMapParser::parse(Rule::dictionary, input.trim()).unwrap();
    for pair in parsed {
        println!("{pair:?}");
    }
}