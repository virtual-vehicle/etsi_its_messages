use std::error::Error;

use std::cell::RefCell;
use std::collections::BTreeMap;

use rasn_compiler::prelude::*;

mod builder;
mod template;
mod utils;

#[derive(Debug, Default)]
pub struct Config {}

#[derive(Debug, Default)]
struct NameResolver {
    assigned: BTreeMap<String, String>,     // raw_key -> final
    used: BTreeMap<String, usize>,          // base -> count
}

impl NameResolver {
    fn resolve(&mut self, raw_key: &str) -> String {
        if let Some(v) = self.assigned.get(raw_key) {
            return v.clone();
        }

        let base = crate::common::to_ros_title_case(raw_key);

        let n = self.used.entry(base.clone()).or_insert(0);
        *n += 1;

        let final_name = if *n == 1 {
            base
        } else {
            format!("{}{}", base, *n)
        };

        self.assigned.insert(raw_key.to_string(), final_name.clone());
        final_name
    }
}

#[derive(Debug, Default)]
pub struct Msgs {
    config: Config,
    names: RefCell<NameResolver>,
}

impl Msgs {
    fn msg_name(&self, raw_key: &str) -> String {
        self.names.borrow_mut().resolve(raw_key)
    }
}

impl Backend for Msgs {
    type Config = Config;

    const FILE_EXTENSION: &'static str = ".msg";

    fn from_config(config: Self::Config) -> Self {
        Self {
            config,
            names: RefCell::new(NameResolver::default()),
        }
    }

    fn config(&self) -> &Self::Config {
        &self.config
    }

    fn generate_module(
        &self,
        tlds: Vec<ToplevelDefinition>,
    ) -> Result<GeneratedModule, GeneratorError> {
        let tlds = self.merge_tlds(tlds);
        let (pdus, warnings): (Vec<String>, Vec<Box<dyn Error>>) =
            tlds.into_iter().fold((vec![], vec![]), |mut acc, tld| {
            match self.generate_tld(tld) {
                Ok(s) => {
                s.len().gt(&0).then(|| {
                    acc.0.push(s)
                });
                acc
                }
                Err(e) => {
                acc.1.push(Box::new(e));
                acc
                }
            }
            });
        Ok(GeneratedModule {
            generated: Some(format!("{}", pdus.join("\n\n"))),
            warnings,
        })
    }

    fn generate(&self, tld: ToplevelDefinition) -> Result<String, GeneratorError> {
        self.generate_tld(tld).map(|ts| ts.to_string())
    }
}
