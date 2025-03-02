use pest::Parser;
use pest_derive::Parser;

#[derive(Parser)]
#[grammar = "map_python_dictionary.pest"]
struct PythonMapParser;

fn main() {
    let input = std::fs::read_to_string("first_improved_map").unwrap();
    let parsed = PythonMapParser::parse(Rule::dictionary, input.trim()).unwrap();
    for pair in parsed {
        for a in pair.into_inner() {
            let mut a_iter = a.into_inner();
            let key = a_iter.next().unwrap().as_str();
            let value = a_iter.next().unwrap().as_str();
            println!("key: {key}");
            println!("value: {value}\n");
        }
    }
}