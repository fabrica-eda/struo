//! Explicit, equivalent register branches for physically separated consumers.
use super::{
    Ecp5Cell, Ecp5Netlist, mapped_cell_name, mapped_lut_profile, mapped_register_count,
    maximum_mapped_wire, replace_wire_in_cell_inputs, verify_mapped_equivalence_proof,
};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeSet, error::Error, fmt};

/// One exact mapped register and the combinational consumers of its replica.
/// A bus is expressed as several requests, without inventing timing observations
/// for its noncritical bits. Fresh physical implementation is still required.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct RegisterBranchReplication {
    /// Exact mapped flip-flop name.
    pub driver: String,
    /// Exact mapped LUT4 or CCU2C names whose uses of this register are moved.
    pub sinks: Vec<String>,
}

/// Equivalent changes actually applied to the mapped graph.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RegisterBranchReplicationReport {
    /// Added flip-flops with identical data, clock, enable and reset behavior.
    pub replicas: usize,
    /// Consumer input pins moved to those replicas.
    pub rewired_pins: usize,
}

/// An invalid request; the original mapped netlist remains unchanged.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RegisterBranchReplicationError(String);

impl fmt::Display for RegisterBranchReplicationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}
impl Error for RegisterBranchReplicationError {}

impl Ecp5Netlist {
    /// Clone exact register branches, keeping all existing register boundaries.
    ///
    /// Requests are atomic and may be applied repeatedly. Every named consumer
    /// must use the original Q wire. Only LUT4/CCU2C data inputs are eligible;
    /// clocks, resets, enables, IO and dedicated wide-LUT topology cannot change.
    ///
    /// # Errors
    /// Rejects missing or ineligible cells, empty/overlapping branches, absent
    /// connections, wire exhaustion, duplicate names and failed equivalence.
    #[allow(clippy::too_many_lines)]
    pub fn replicate_register_branches(
        &mut self,
        branches: &[RegisterBranchReplication],
    ) -> Result<RegisterBranchReplicationReport, RegisterBranchReplicationError> {
        if branches.is_empty() {
            return Ok(RegisterBranchReplicationReport::default());
        }
        self.validate_export_names()
            .map_err(|e| RegisterBranchReplicationError(e.to_string()))?;
        let mut candidate = self.clone();
        let mut names = candidate
            .cells
            .iter()
            .map(|c| mapped_cell_name(c).to_owned())
            .collect::<BTreeSet<_>>();
        let mut connections = BTreeSet::new();
        let mut next_wire = maximum_mapped_wire(self)
            .and_then(|n| n.checked_add(1))
            .ok_or_else(|| RegisterBranchReplicationError("mapped wire overflow".into()))?;
        let mut report = RegisterBranchReplicationReport::default();
        for branch in branches {
            let mut replica = self
                .cells
                .iter()
                .find(|c| mapped_cell_name(c) == branch.driver)
                .cloned()
                .ok_or_else(|| {
                    RegisterBranchReplicationError(format!("missing register {}", branch.driver))
                })?;
            let Ecp5Cell::FlipFlop { output, .. } = &replica else {
                return Err(RegisterBranchReplicationError(format!(
                    "{} is not a flip-flop",
                    branch.driver
                )));
            };
            let original_wire = *output;
            if branch.sinks.is_empty() {
                return Err(RegisterBranchReplicationError(
                    "empty register branch".into(),
                ));
            }
            let clone_wire = next_wire;
            next_wire = next_wire
                .checked_add(1)
                .ok_or_else(|| RegisterBranchReplicationError("mapped wire overflow".into()))?;
            for sink in &branch.sinks {
                if !connections.insert((&branch.driver, sink)) {
                    return Err(RegisterBranchReplicationError(format!(
                        "overlapping branch {} -> {sink}",
                        branch.driver
                    )));
                }
                let cell = candidate
                    .cells
                    .iter_mut()
                    .find(|c| mapped_cell_name(c) == sink)
                    .ok_or_else(|| {
                        RegisterBranchReplicationError(format!("missing sink {sink}"))
                    })?;
                if !matches!(cell, Ecp5Cell::Lut4 { .. } | Ecp5Cell::Ccu2c { .. }) {
                    return Err(RegisterBranchReplicationError(format!(
                        "{sink} is not a LUT4/CCU2C data consumer"
                    )));
                }
                let rewired = replace_wire_in_cell_inputs(cell, original_wire, clone_wire);
                if rewired == 0 {
                    return Err(RegisterBranchReplicationError(format!(
                        "{sink} does not consume {}",
                        branch.driver
                    )));
                }
                report.rewired_pins += rewired;
                candidate.placement_hints.remove(sink);
            }
            let mut serial = 0;
            let clone_name = loop {
                let name = format!("physical_replicate_{}_{serial}", branch.driver);
                if names.insert(name.clone()) {
                    break name;
                }
                serial += 1;
            };
            if let Ecp5Cell::FlipFlop { name, output, .. } = &mut replica {
                *name = clone_name;
                *output = clone_wire;
            }
            candidate.cells.push(replica);
            report.replicas += 1;
        }
        candidate.equivalence_proof.equivalent_logic_replications += report.replicas;
        candidate.equivalence_proof.equivalent_physical_rewires += report.rewired_pins;
        let profile = mapped_lut_profile(&candidate);
        candidate.retiming.selected_lut_depth = profile.data_depth;
        candidate.retiming.selected_critical_registers = profile.critical_depth.len();
        candidate.retiming.selected_period_ps = profile.data_period_ps;
        candidate.retiming.selected_overall_period_ps = profile.overall_period_ps;
        candidate.retiming.selected_registers = mapped_register_count(&candidate);
        candidate.retiming.equivalent_logic_replications =
            candidate.equivalence_proof.equivalent_logic_replications;
        candidate.retiming.equivalent_physical_rewires =
            candidate.equivalence_proof.equivalent_physical_rewires;
        candidate.retiming.equivalence_signed_off =
            verify_mapped_equivalence_proof(&candidate, candidate.retiming.applied);
        if !candidate.retiming.equivalence_signed_off {
            return Err(RegisterBranchReplicationError(
                "register branch equivalence failed".into(),
            ));
        }
        *self = candidate;
        Ok(report)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Bit, MappingOptions, map_to_ecp5_with_options};
    use struo_ir::{ClockEdge, Netlist, RegisterCell};

    fn fixture() -> (Ecp5Netlist, String, Vec<String>) {
        let mut source = Netlist::new("branches");
        let clock = source.add_input("clock");
        let data = source.add_input("data");
        let state = source.add_register_output("state");
        source.add_register(RegisterCell::new(
            "state",
            state,
            data,
            clock,
            ClockEdge::Rising,
            None,
            None,
        ));
        for index in 0..3 {
            let input = source.add_input(format!("in{index}"));
            let mixed = source.add_xor(state, input);
            source.add_output(format!("out{index}"), mixed);
        }
        let mapped = map_to_ecp5_with_options(
            &source,
            MappingOptions {
                retiming: false,
                ..MappingOptions::default()
            },
        )
        .unwrap();
        let (driver, wire) = mapped
            .cells
            .iter()
            .find_map(|cell| match cell {
                Ecp5Cell::FlipFlop { name, output, .. } => Some((name.clone(), *output)),
                _ => None,
            })
            .unwrap();
        let sinks = mapped
            .cells
            .iter()
            .filter_map(|cell| match cell {
                Ecp5Cell::Lut4 { name, inputs, .. } if inputs.contains(&Bit::Wire(wire)) => {
                    Some(name.clone())
                }
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(sinks.len(), 3);
        (mapped, driver, sinks)
    }

    #[test]
    fn repeated_branches_keep_unique_names_and_existing_consumers() {
        let (mut mapped, driver, sinks) = fixture();
        let original = mapped.clone();
        for sink in &sinks[..2] {
            let report = mapped
                .replicate_register_branches(&[RegisterBranchReplication {
                    driver: driver.clone(),
                    sinks: vec![sink.clone()],
                }])
                .unwrap();
            assert_eq!(
                report,
                RegisterBranchReplicationReport {
                    replicas: 1,
                    rewired_pins: 1
                }
            );
        }
        mapped.validate_export_names().unwrap();
        assert_eq!(
            mapped.retiming.selected_registers,
            original.retiming.selected_registers + 2
        );
        assert!(mapped.retiming.equivalence_signed_off);
        let untouched = |net: &Ecp5Netlist| {
            net.cells
                .iter()
                .find(|c| mapped_cell_name(c) == sinks[2])
                .unwrap()
                .clone()
        };
        assert_eq!(untouched(&mapped), untouched(&original));
    }

    #[test]
    fn invalid_second_request_rolls_back_first_request() {
        let (mut mapped, driver, sinks) = fixture();
        let before = mapped.clone();
        let valid = RegisterBranchReplication {
            driver,
            sinks: vec![sinks[0].clone()],
        };
        let missing = RegisterBranchReplication {
            driver: "missing".into(),
            sinks: vec![sinks[1].clone()],
        };
        assert!(
            mapped
                .replicate_register_branches(&[valid.clone(), missing])
                .is_err()
        );
        assert_eq!(mapped, before);
        assert!(
            mapped
                .replicate_register_branches(&[valid.clone(), valid])
                .is_err()
        );
        assert_eq!(mapped, before);
    }

    #[test]
    fn rejects_wrong_driver_and_absent_connection() {
        let (mut mapped, driver, sinks) = fixture();
        let before = mapped.clone();
        for request in [
            RegisterBranchReplication {
                driver: sinks[0].clone(),
                sinks: vec![sinks[1].clone()],
            },
            RegisterBranchReplication {
                driver: driver.clone(),
                sinks: Vec::new(),
            },
            RegisterBranchReplication {
                driver,
                sinks: vec!["missing".into()],
            },
        ] {
            assert!(mapped.replicate_register_branches(&[request]).is_err());
            assert_eq!(mapped, before);
        }
        // A previously moved sink exists but no longer consumes the original Q.
        let request = RegisterBranchReplication {
            driver: "ff_state".into(),
            sinks: vec![sinks[0].clone()],
        };
        mapped
            .replicate_register_branches(std::slice::from_ref(&request))
            .unwrap();
        let once = mapped.clone();
        assert!(mapped.replicate_register_branches(&[request]).is_err());
        assert_eq!(mapped, once);
    }
}
