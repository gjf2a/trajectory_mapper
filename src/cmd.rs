use std::{collections::HashMap, str::FromStr};

#[derive(Debug)]
pub struct ArgVals {
    simple_vals: Vec<String>,
    mapped_vals: HashMap<String, String>,
}

impl ArgVals {
    pub fn len(&self) -> usize {
        self.simple_vals.len() + self.mapped_vals.len()
    }

    pub fn num_symbols(&self) -> usize {
        self.simple_vals.len()
    }

    pub fn get_symbol(&self, i: usize) -> &str {
        self.simple_vals[i].as_str()
    }

    pub fn has_symbol(&self, symbol: &str) -> bool {
        self.simple_vals.iter().any(|s| s == symbol)
    }

    pub fn get_value<N: Copy + FromStr>(&self, key: &str) -> Option<N> {
        self.mapped_vals
            .get(key)
            .map(|v| v.parse::<N>())
            .and_then(|v| v.ok())
    }

    pub fn get_duple<N: Copy + FromStr>(&self, key: &str) -> Option<(N, N)> {
        self.mapped_vals.get(key).and_then(|v| {
            let values = v.split(",").collect::<Vec<_>>();
            if values.len() == 2 {
                match values[0].parse::<N>() {
                    Err(_) => None,
                    Ok(v1) => match values[1].parse::<N>() {
                        Err(_) => None,
                        Ok(v2) => Some((v1, v2)),
                    },
                }
            } else {
                None
            }
        })
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
