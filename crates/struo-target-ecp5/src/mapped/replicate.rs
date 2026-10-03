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

/// One ordinary mapped LUT4 and the data consumers of an identical copy.
/// This names connectivity only; it makes no claim about physical timing.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct LogicBranchReplication {
    /// Exact mapped LUT4 name. Wide-mux and carry outputs are not eligible.
    pub driver: String,
    /// Exact mapped LUT4 or CCU2C data consumers to move to the copy.
    pub sinks: Vec<String>,
}

/// Equivalent combinational copies actually applied to the mapped graph.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LogicBranchReplicationReport {
    /// Added LUT4 cells with identical INIT and inputs.
    pub replicas: usize,
    /// Consumer data pins moved to those copies.
    pub rewired_pins: usize,
}

/// An invalid logic request; the original mapped netlist remains unchanged.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LogicBranchReplicationError(String);

impl fmt::Display for LogicBranchReplicationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}
impl Error for LogicBranchReplicationError {}

#[derive(Clone, Copy)]
enum BranchDriverKind {
    Register,
    Logic,
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
    pub fn replicate_register_branches(
        &mut self,
        branches: &[RegisterBranchReplication],
    ) -> Result<RegisterBranchReplicationReport, RegisterBranchReplicationError> {
        self.replicate_named_branches(branches, BranchDriverKind::Register)
    }

    /// Clone ordinary LUT4 branches without adding register stages.
    ///
    /// Each copy retains its driver's exact truth table and input wires. Only
    /// named LUT4/CCU2C data inputs move; clock, reset, enable, IO and dedicated
    /// wide-mux connections cannot be selected. Requests are atomic and can be
    /// repeated. Placement and routed STA must be recomputed after this change.
    ///
    /// # Errors
    /// Rejects missing/ineligible drivers or sinks, empty/overlapping branches,
    /// absent connections, wire exhaustion, duplicate names and failed proof.
    pub fn replicate_logic_branches(
        &mut self,
        branches: &[LogicBranchReplication],
    ) -> Result<LogicBranchReplicationReport, LogicBranchReplicationError> {
        let requests = branches
            .iter()
            .map(|branch| RegisterBranchReplication {
                driver: branch.driver.clone(),
                sinks: branch.sinks.clone(),
            })
            .collect::<Vec<_>>();
        self.replicate_named_branches(&requests, BranchDriverKind::Logic)
            .map(|report| LogicBranchReplicationReport {
                replicas: report.replicas,
                rewired_pins: report.rewired_pins,
            })
            .map_err(|error| LogicBranchReplicationError(error.0))
    }

    #[allow(clippy::too_many_lines)]
    fn replicate_named_branches(
        &mut self,
        branches: &[RegisterBranchReplication],
        kind: BranchDriverKind,
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
                    RegisterBranchReplicationError(format!("missing driver {}", branch.driver))
                })?;
            let original_wire = match (&replica, kind) {
                (Ecp5Cell::FlipFlop { output, .. }, BranchDriverKind::Register)
                | (Ecp5Cell::Lut4 { output, .. }, BranchDriverKind::Logic) => *output,
                _ => {
                    let expected = match kind {
                        BranchDriverKind::Register => "flip-flop",
                        BranchDriverKind::Logic => "LUT4",
                    };
                    return Err(RegisterBranchReplicationError(format!(
                        "{} is not a {expected}",
                        branch.driver
                    )));
                }
            };
            if branch.sinks.is_empty() {
                return Err(RegisterBranchReplicationError(
                    "empty replication branch".into(),
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
            match &mut replica {
                Ecp5Cell::FlipFlop { name, output, .. } | Ecp5Cell::Lut4 { name, output, .. } => {
                    *name = clone_name;
                    *output = clone_wire;
                }
                _ => unreachable!("driver kind was validated before rewiring"),
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
                "branch replication equivalence failed".into(),
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

    fn logic_fixture() -> (Ecp5Netlist, String, Vec<String>) {
        let (mut mapped, driver, sinks) = fixture();
        let extras = ["in0", "in1"].map(|name| {
            mapped
                .ports
                .iter()
                .find(|port| port.name == name)
                .unwrap()
                .bits[0]
        });
        let cell = mapped
            .cells
            .iter_mut()
            .find(|cell| mapped_cell_name(cell) == driver)
            .unwrap();
        let Ecp5Cell::FlipFlop {
            data,
            clock,
            output,
            ..
        } = cell.clone()
        else {
            unreachable!()
        };
        // Primitive-level fixture: out[i] = ((data & clock) | (in0 ^ in1)) ^ in[i].
        // Retain the original wire numbering and three distinct consumers.
        *cell = Ecp5Cell::Lut4 {
            name: "logic_driver".into(),
            inputs: [data, clock, extras[0], extras[1]],
            init: 0x8ff8,
            output,
        };
        (mapped, "logic_driver".into(), sinks)
    }

    fn check_logic_oracle(mapped: &Ecp5Netlist) {
        use crate::mapped::PortDirection;
        use std::collections::BTreeMap;
        for pattern in 0..32_u32 {
            let mut wires = BTreeMap::<u32, bool>::new();
            for port in &mapped.ports {
                if port.direction != PortDirection::Input {
                    continue;
                }
                let index = match port.name.as_str() {
                    "data" => 0,
                    "clock" => 1,
                    name => name.strip_prefix("in").unwrap().parse::<u32>().unwrap() + 2,
                };
                let Bit::Wire(wire) = port.bits[0] else {
                    panic!("fixture inputs are wires")
                };
                wires.insert(wire, pattern & (1 << index) != 0);
            }
            let value = |bit, wires: &BTreeMap<u32, bool>| match bit {
                Bit::Zero => Some(false),
                Bit::One => Some(true),
                Bit::Wire(wire) => wires.get(&wire).copied(),
            };
            let mut pending = mapped.cells.iter().collect::<Vec<_>>();
            while !pending.is_empty() {
                let before = pending.len();
                pending.retain(|cell| {
                    let Ecp5Cell::Lut4 {
                        inputs,
                        init,
                        output,
                        ..
                    } = cell
                    else {
                        panic!("logic fixture only contains LUTs")
                    };
                    let Some(bits) = inputs
                        .iter()
                        .map(|bit| value(*bit, &wires))
                        .collect::<Option<Vec<_>>>()
                    else {
                        return true;
                    };
                    let address = bits
                        .iter()
                        .enumerate()
                        .fold(0, |sum, (index, bit)| sum | (usize::from(*bit) << index));
                    wires.insert(*output, (init >> address) & 1 != 0);
                    false
                });
                assert!(pending.len() < before, "unresolved or cyclic fixture");
            }
            for port in &mapped.ports {
                if port.direction == PortDirection::Output {
                    let index = port
                        .name
                        .strip_prefix("out")
                        .unwrap()
                        .parse::<u32>()
                        .unwrap()
                        + 2;
                    let shared = (pattern & 3 == 3) || (((pattern >> 2) ^ (pattern >> 3)) & 1 != 0);
                    let expected = shared ^ (pattern & (1 << index) != 0);
                    assert_eq!(
                        value(port.bits[0], &wires),
                        Some(expected),
                        "{} pattern {pattern}",
                        port.name
                    );
                }
            }
        }
    }

    #[test]
    fn logic_branches_preserve_every_input_combination_and_unselected_consumers() {
        let (mut mapped, driver, sinks) = logic_fixture();
        check_logic_oracle(&mapped);
        let original = mapped.clone();
        for sink in &sinks[..2] {
            let report = mapped
                .replicate_logic_branches(&[LogicBranchReplication {
                    driver: driver.clone(),
                    sinks: vec![sink.clone()],
                }])
                .unwrap();
            assert_eq!(
                report,
                LogicBranchReplicationReport {
                    replicas: 1,
                    rewired_pins: 1
                }
            );
            check_logic_oracle(&mapped);
            assert!(mapped.retiming.equivalence_signed_off);
            assert_eq!(mapped.retiming.selected_registers, 0);
        }
        mapped.validate_export_names().unwrap();
        assert_eq!(mapped.cells.len(), original.cells.len() + 2);
        let untouched = |net: &Ecp5Netlist| {
            net.cells
                .iter()
                .find(|cell| mapped_cell_name(cell) == sinks[2])
                .unwrap()
                .clone()
        };
        assert_eq!(untouched(&mapped), untouched(&original));
    }

    #[test]
    fn logic_requests_are_atomic_and_reject_invalid_connections() {
        let (mut mapped, driver, sinks) = logic_fixture();
        let before = mapped.clone();
        let valid = LogicBranchReplication {
            driver: driver.clone(),
            sinks: vec![sinks[0].clone()],
        };
        for bad in [
            LogicBranchReplication {
                driver: "missing".into(),
                sinks: vec![sinks[1].clone()],
            },
            LogicBranchReplication {
                driver: driver.clone(),
                sinks: vec![],
            },
            LogicBranchReplication {
                driver: driver.clone(),
                sinks: vec!["missing".into()],
            },
            LogicBranchReplication {
                driver: sinks[1].clone(),
                sinks: vec![sinks[2].clone()],
            },
            valid.clone(),
        ] {
            assert!(
                mapped
                    .replicate_logic_branches(&[valid.clone(), bad])
                    .is_err()
            );
            assert_eq!(mapped, before);
        }
        mapped
            .replicate_logic_branches(std::slice::from_ref(&valid))
            .unwrap();
        let once = mapped.clone();
        assert!(mapped.replicate_logic_branches(&[valid]).is_err());
        assert_eq!(mapped, once);
        let (mut mapped, register, sinks) = fixture();
        let before = mapped.clone();
        assert!(
            mapped
                .replicate_logic_branches(&[LogicBranchReplication {
                    driver: register,
                    sinks
                }])
                .is_err()
        );
        assert_eq!(mapped, before);
    }

    #[test]
    fn logic_requests_cannot_rewire_register_controls_or_dedicated_muxes() {
        let (mut mapped, driver, sinks) = logic_fixture();
        let wire = mapped
            .cells
            .iter()
            .find_map(|cell| match cell {
                Ecp5Cell::Lut4 { name, output, .. } if name == &driver => Some(*output),
                _ => None,
            })
            .unwrap();
        let next_wire = maximum_mapped_wire(&mapped).unwrap() + 1;
        let (register_fixture, _, _) = fixture();
        let mut register = register_fixture
            .cells
            .into_iter()
            .find(|cell| matches!(cell, Ecp5Cell::FlipFlop { .. }))
            .unwrap();
        if let Ecp5Cell::FlipFlop {
            name,
            clock,
            data,
            output,
            ..
        } = &mut register
        {
            *name = "control_sink".into();
            *clock = Bit::Wire(wire);
            *data = Bit::Wire(wire);
            *output = next_wire;
        }
        mapped.cells.push(register);
        mapped.cells.push(Ecp5Cell::PfuMux {
            name: "wide_sink".into(),
            lut_true: Bit::Wire(wire),
            lut_false: Bit::Zero,
            select: Bit::One,
            output: next_wire + 1,
        });
        let before = mapped.clone();
        for sink in ["control_sink", "wide_sink"] {
            let requests = [
                LogicBranchReplication {
                    driver: driver.clone(),
                    sinks: vec![sinks[0].clone()],
                },
                LogicBranchReplication {
                    driver: driver.clone(),
                    sinks: vec![sink.into()],
                },
            ];
            assert!(mapped.replicate_logic_branches(&requests).is_err());
            assert_eq!(mapped, before);
        }
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
