use std::collections::HashMap;

#[derive(Debug)]
pub struct ArgVals {
    pub simple_vals: Vec<String>,
    pub mapped_vals: HashMap<String, String>,
}

impl ArgVals {
    pub fn len(&self) -> usize {
        self.simple_vals.len() + self.mapped_vals.len()
    }
}

impl Default for ArgVals {
    fn default() -> Self {
        let mut simple_vals = vec![];
        let mut mapped_vals = HashMap::default();
        for arg in std::env::args().skip(1) {
            if arg.contains("=") {
                let parts = arg.split("=").collect::<Vec<_>>();
                if parts.len() == 2 {
                    mapped_vals.insert(parts[0].to_string(), parts[1].to_string());
                } else {
                    simple_vals.push(arg);
                }
            } else {
                simple_vals.push(arg);
            }
        }
        Self {
            simple_vals,
            mapped_vals,
        }
    }
}
