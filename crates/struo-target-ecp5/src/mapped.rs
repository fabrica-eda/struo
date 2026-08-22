//! ECP5 technology mapping and nextpnr serialization.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::error::Error;
use std::fmt::{self, Display, Formatter};

use serde::Serialize;
use serde::ser::Serializer;
use struo_formal::{
    LogicFunction, RetimingCertificate, RetimingDomain, RetimingEdge, RetimingGraph,
    RetimingVertex, derive_retimed_graph, verify_retiming_certificate,
};
use struo_ir::{
    ActiveLevel, ArithmeticCell, ArithmeticOp, ClockEdge, ComparisonCell, MemoryCell, NetId,
    Netlist, PortDirection as IrPortDirection, ValidationError,
};

mod lut;

use lut::{
    BRAM_CLOCK_TO_OUTPUT_PS, CCU_CARRY_PS, CCU_INPUT_PS, CCU_SUM_PS, CutDatabase,
    FLIP_FLOP_CLOCK_TO_OUTPUT_PS, FLIP_FLOP_SETUP_PS, LUT_DELAY_PS, LutCover, LutEmitter,
    wire_delay_ps,
};

const RETIMING_PERIOD_MARGIN_NUMERATOR: u32 = 9;
const RETIMING_PERIOD_MARGIN_DENOMINATOR: u32 = 10;
// The pre-placement fanout model deliberately stays optimistic so LUT covering
// does not overfit one device floorplan.  Once primitives are fixed, retiming
// needs a routing guard per ordinary hop: measured ECP5 AXI paths average about
// 100 ps more than that model, while dedicated carry hops bypass this charge.
const MAPPED_ROUTE_GUARD_PS: u32 = 100;

/// A constant or numbered wire in a mapped ECP5 design.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Bit {
    /// Constant zero.
    Zero,
    /// Constant one.
    One,
    /// A numbered Yosys/nextpnr wire.
    Wire(u32),
}

impl Serialize for Bit {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        match self {
            Self::Zero => serializer.serialize_str("0"),
            Self::One => serializer.serialize_str("1"),
            Self::Wire(wire) => serializer.serialize_u32(*wire),
        }
    }
}

/// Direction of a physical port.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PortDirection {
    /// FPGA input.
    Input,
    /// FPGA output.
    Output,
}

/// One physical port with bits stored least-significant first.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MappedPort {
    /// Port name used by the LPF constraint file.
    pub name: String,
    /// Port direction.
    pub direction: PortDirection,
    /// Connected mapped bits, least-significant first.
    pub bits: Vec<Bit>,
}

/// Control input and its assertion level.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Control {
    /// Control signal.
    pub signal: Bit,
    /// Logical assertion level.
    pub active: ActiveLevel,
}

/// Reset behavior retained by a mapped flip-flop.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Reset {
    /// Reset signal.
    pub signal: Bit,
    /// Logical assertion level.
    pub active: ActiveLevel,
    /// Whether reset is asynchronous.
    pub asynchronous: bool,
    /// State loaded on reset.
    pub value: bool,
}

/// How retained word-level arithmetic is implemented on ECP5.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArithmeticMapping {
    /// Use CCU2C above four bits and LUTs for smaller operations.
    Auto,
    /// Always use the dedicated CCU2C carry chain.
    CarryChain,
    /// Implement a ripple carry using ordinary LUT4 cells.
    Lut4,
}

/// Technology-mapping choices used for experiments and regression tests.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MappingOptions {
    /// Arithmetic implementation strategy.
    pub arithmetic: ArithmeticMapping,
    /// Clock frequency used by timing-driven LUT covering.
    pub timing_goal_mhz: u32,
}

impl Default for MappingOptions {
    fn default() -> Self {
        Self {
            arithmetic: ArithmeticMapping::Auto,
            timing_goal_mhz: crate::ECP5_QOR_TARGET_MHZ,
        }
    }
}

#[derive(Debug)]
struct MappingDemand {
    roots: Vec<NetId>,
}

impl MappingDemand {
    fn collect(netlist: &Netlist) -> Self {
        let output_roots = netlist
            .ports()
            .iter()
            .filter(|port| port.direction() == IrPortDirection::Output)
            .flat_map(struo_ir::Port::bits)
            .map(|output| node_for(netlist, *output).inputs()[0]);
        let register_roots = netlist.registers().iter().flat_map(|register| {
            [register.data(), register.clock()]
                .into_iter()
                .chain(register.enable().map(|enable| enable.signal))
                .chain(register.reset().map(|reset| reset.signal))
        });
        let memory_roots = netlist.memories().iter().flat_map(|memory| {
            memory
                .read_address()
                .iter()
                .chain(memory.write_address())
                .chain(memory.write_data())
                .copied()
                .chain([memory.clock(), memory.write_enable().signal])
                .chain(memory.read_enable().map(|enable| enable.signal))
        });
        let arithmetic_roots = netlist
            .arithmetic()
            .iter()
            .flat_map(|cell| cell.lhs().iter().chain(cell.rhs()).copied());
        let comparison_roots = netlist
            .comparisons()
            .iter()
            .flat_map(|cell| cell.lhs().iter().chain(cell.rhs()).copied());
        Self {
            // Preserve duplicates: they are distinct physical sink pins and
            // therefore contribute to the fanout estimate used by covering.
            roots: output_roots
                .chain(register_roots)
                .chain(memory_roots)
                .chain(arithmetic_roots)
                .chain(comparison_roots)
                .collect(),
        }
    }
}

/// A physical ECP5 primitive.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Ecp5Cell {
    /// Four-input lookup table. Input A is the least-significant INIT index bit.
    Lut4 {
        /// Stable cell name.
        name: String,
        /// A, B, C, and D inputs.
        inputs: [Bit; 4],
        /// LUT output wire.
        output: u32,
        /// Complete LUT truth table.
        init: u16,
    },
    /// Two ECP5 LUT/carry slices connected through the dedicated carry path.
    Ccu2c {
        /// Stable cell name.
        name: String,
        /// A, B, C, and D inputs for each of the two slices.
        inputs: [[Bit; 4]; 2],
        /// Dedicated carry-chain input.
        carry_in: Bit,
        /// Sum output wires for the two slices.
        sums: [u32; 2],
        /// Dedicated carry-chain output wire.
        carry_out: u32,
        /// LUT truth tables for the two slices.
        init: [u16; 2],
        /// Whether each slice suppresses its incoming carry.
        inject: [bool; 2],
    },
    /// ECP5 slice flip-flop.
    FlipFlop {
        /// Stable cell name.
        name: String,
        /// D input.
        data: Bit,
        /// Q output wire.
        output: u32,
        /// Clock input.
        clock: Bit,
        /// Active clock edge.
        edge: ClockEdge,
        /// Optional clock enable.
        enable: Option<Control>,
        /// Optional local set/reset.
        reset: Option<Reset>,
    },
    /// One ECP5 18-Kibit embedded block RAM in simple-dual-port mode.
    BlockRam {
        /// Stable cell name.
        name: String,
        /// Logical number of words represented by the cell.
        depth: u32,
        /// Logical word width.
        word_width: u8,
        /// Configured DP16KD port width (1, 2, 4, 9, or 18).
        physical_width: u8,
        /// Fourteen physical write-address pins.
        write_address: Box<[Bit; 14]>,
        /// Logical write-data bits, least-significant first.
        write_data: Vec<Bit>,
        /// Write enable.
        write_enable: Control,
        /// Fourteen physical read-address pins.
        read_address: Box<[Bit; 14]>,
        /// Logical read-data output wires, least-significant first.
        read_data: Vec<u32>,
        /// Optional read clock enable.
        read_enable: Option<Control>,
        /// Shared port clock.
        clock: Bit,
        /// Shared active clock edge.
        edge: ClockEdge,
    },
}

/// Technology-mapped ECP5 netlist used by both simulation and nextpnr.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ecp5Netlist {
    name: String,
    ports: Vec<MappedPort>,
    cells: Vec<Ecp5Cell>,
    retiming: RetimingSelection,
}

/// Result of the automatic, technology-scored retiming search.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RetimingSelection {
    /// Whether automatic mapped-LUT retiming selected a certified candidate.
    pub applied: bool,
    /// Maximum LUT4 levels into a register before retiming.
    pub original_lut_depth: usize,
    /// Maximum LUT4 levels into a register in the selected mapping.
    pub selected_lut_depth: usize,
    /// Register data inputs at the original maximum LUT4 depth.
    pub original_critical_registers: usize,
    /// Register data inputs remaining at the maximum depth after retiming.
    pub selected_critical_registers: usize,
    /// Estimated register-to-register period before retiming.
    pub original_period_ps: u32,
    /// Estimated register-to-register period selected for mapping.
    pub selected_period_ps: u32,
    /// Mapped flip-flops before candidate selection.
    pub original_registers: usize,
    /// Mapped flip-flops after candidate selection.
    pub selected_registers: usize,
}

impl Ecp5Netlist {
    /// Returns the top name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns physical ports with source-level vector grouping preserved.
    #[must_use]
    pub fn ports(&self) -> &[MappedPort] {
        &self.ports
    }

    /// Returns physical primitives in dependency order.
    #[must_use]
    pub fn cells(&self) -> &[Ecp5Cell] {
        &self.cells
    }

    /// Returns the technology-scored retiming decision made during mapping.
    #[must_use]
    pub const fn retiming(&self) -> RetimingSelection {
        self.retiming
    }

    /// Serializes this exact mapped object to the Yosys JSON schema consumed by
    /// nextpnr-ecp5.
    ///
    /// # Errors
    ///
    /// Returns an error if JSON serialization fails.
    pub fn to_nextpnr_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(&JsonDesign::from(self))
    }
}

/// Maps target-independent Boolean nodes and registers to ECP5 primitives.
///
/// # Errors
///
/// Returns an error if the source netlist is invalid.
pub fn map_to_ecp5(netlist: &Netlist) -> Result<Ecp5Netlist, MappingError> {
    map_to_ecp5_with_options(netlist, MappingOptions::default())
}

/// Maps a target-independent netlist with explicit arithmetic choices.
///
/// # Errors
///
/// Returns an error if the source netlist is invalid.
pub fn map_to_ecp5_with_options(
    netlist: &Netlist,
    options: MappingOptions,
) -> Result<Ecp5Netlist, MappingError> {
    let (mut selected, _) = map_once(netlist, options)?;
    let original_cells = selected.cells.len();
    let original_registers = netlist.registers().len();
    let mut selected_registers = original_registers;
    let mut applied = false;
    let mapped_original_profile = mapped_lut_profile(&selected);
    let target_period_ps = 1_000_000u32
        .div_ceil(options.timing_goal_mhz.max(1))
        .saturating_mul(RETIMING_PERIOD_MARGIN_NUMERATOR)
        / RETIMING_PERIOD_MARGIN_DENOMINATOR;
    if let Some(retimed) = automatically_retime_mapped_luts(
        &selected,
        original_cells,
        original_registers,
        target_period_ps,
    ) {
        selected = retimed;
        selected_registers = mapped_register_count(&selected);
        applied = true;
    }
    let mapped_selected_profile = mapped_lut_profile(&selected);
    selected.retiming = RetimingSelection {
        applied,
        original_lut_depth: mapped_original_profile.data_depth,
        selected_lut_depth: mapped_selected_profile.data_depth,
        original_critical_registers: mapped_original_profile.critical_depth.len(),
        selected_critical_registers: mapped_selected_profile.critical_depth.len(),
        original_period_ps: mapped_original_profile.data_period_ps,
        selected_period_ps: mapped_selected_profile.data_period_ps,
        original_registers,
        selected_registers,
    };
    Ok(selected)
}

#[allow(clippy::too_many_lines)]
fn automatically_retime_mapped_luts(
    original: &Ecp5Netlist,
    original_cells: usize,
    original_registers: usize,
    target_period_ps: u32,
) -> Option<Ecp5Netlist> {
    let original_profile = mapped_lut_profile(original);
    let original_depth = original_profile.data_depth;
    let timing_driven = original_profile.data_period_ps > target_period_ps;
    let cell_limit = original_cells + original_cells.div_ceil(10);
    let register_limit = original_registers + original_registers.div_ceil(5);
    let mut frontier = original.clone();
    let mut plateau_budget = 2usize;
    for _ in 0..64 {
        let profile = mapped_lut_profile(&frontier);
        if (timing_driven && profile.data_period_ps <= target_period_ps)
            || (!timing_driven && profile.data_depth < original_depth)
        {
            return Some(frontier);
        }
        let mut best = None;
        let critical = if timing_driven {
            profile.critical_timing.clone()
        } else {
            profile.critical_depth.clone()
        };
        for &register in &critical {
            let Some(candidate) = backward_retime_lut(&frontier, register) else {
                continue;
            };
            let candidate_profile = mapped_lut_profile(&candidate);
            let candidate_registers = mapped_register_count(&candidate);
            if (!timing_driven && candidate_profile.overall_depth > original_profile.overall_depth)
                || candidate_profile.overall_period_ps > original_profile.overall_period_ps
                || candidate.cells.len() > cell_limit
                || candidate_registers > register_limit
            {
                continue;
            }
            let score = retiming_score(
                &candidate_profile,
                timing_driven,
                candidate.cells.len(),
                candidate_registers,
            );
            let frontier_score = retiming_score(
                &profile,
                timing_driven,
                frontier.cells.len(),
                mapped_register_count(&frontier),
            );
            if score < frontier_score
                && best
                    .as_ref()
                    .is_none_or(|(_, best_score): &(Ecp5Netlist, _)| score < *best_score)
            {
                best = Some((candidate, score));
            }
        }
        // Equal-delay endpoints form a timing cutset: moving any one endpoint
        // leaves the maximum unchanged, even though moving the whole cutset is
        // profitable. Cell indices stay valid when transformed high-to-low.
        let mut batch_registers = critical.clone();
        batch_registers.sort_unstable_by(|lhs, rhs| rhs.cmp(lhs));
        let mut batch = frontier.clone();
        let mut batch_moves = 0usize;
        for register in batch_registers {
            let Some(candidate) = backward_retime_lut(&batch, register) else {
                continue;
            };
            let candidate_profile = mapped_lut_profile(&candidate);
            let candidate_registers = mapped_register_count(&candidate);
            if (timing_driven || candidate_profile.overall_depth <= original_profile.overall_depth)
                && candidate_profile.overall_period_ps <= original_profile.overall_period_ps
                && candidate.cells.len() <= cell_limit
                && candidate_registers <= register_limit
            {
                batch = candidate;
                batch_moves += 1;
            }
        }
        let mut plateau_candidate = None;
        if batch_moves > 0 {
            let batch_profile = mapped_lut_profile(&batch);
            let batch_score = retiming_score(
                &batch_profile,
                timing_driven,
                batch.cells.len(),
                mapped_register_count(&batch),
            );
            let frontier_score = retiming_score(
                &profile,
                timing_driven,
                frontier.cells.len(),
                mapped_register_count(&frontier),
            );
            if batch_score < frontier_score
                && best
                    .as_ref()
                    .is_none_or(|(_, best_score): &(Ecp5Netlist, _)| batch_score < *best_score)
            {
                best = Some((batch, batch_score));
            } else if timing_driven
                && profile.data_period_ps > target_period_ps
                && batch_profile.data_period_ps == profile.data_period_ps
                && batch_moves == critical.len()
                && plateau_budget > 0
            {
                plateau_candidate = Some((batch, batch_score));
            }
        }
        let selected_plateau = best.is_none() && plateau_candidate.is_some();
        let Some((candidate, _)) = best.or(plateau_candidate) else {
            break;
        };
        if selected_plateau {
            plateau_budget -= 1;
        }
        frontier = candidate;
    }
    let selected_profile = mapped_lut_profile(&frontier);
    let improved = if timing_driven {
        (
            selected_profile.data_period_ps,
            selected_profile.critical_timing.len(),
        ) < (
            original_profile.data_period_ps,
            original_profile.critical_timing.len(),
        )
    } else {
        (
            selected_profile.data_depth,
            selected_profile.critical_depth.len(),
        ) < (
            original_profile.data_depth,
            original_profile.critical_depth.len(),
        )
    };
    improved.then_some(frontier)
}

fn retiming_score(
    profile: &MappedLutProfile,
    timing_driven: bool,
    cells: usize,
    registers: usize,
) -> (u64, u64, usize, usize, usize) {
    if timing_driven {
        (
            u64::from(profile.data_period_ps),
            u64::try_from(profile.critical_timing.len()).unwrap_or(u64::MAX),
            profile.data_depth,
            cells,
            registers,
        )
    } else {
        (
            u64::try_from(profile.data_depth).unwrap_or(u64::MAX),
            u64::try_from(profile.critical_depth.len()).unwrap_or(u64::MAX),
            usize::try_from(profile.data_period_ps).unwrap_or(usize::MAX),
            cells,
            registers,
        )
    }
}

fn mapped_register_count(netlist: &Ecp5Netlist) -> usize {
    netlist
        .cells
        .iter()
        .filter(|cell| matches!(cell, Ecp5Cell::FlipFlop { .. }))
        .count()
}

struct MappedLutProfile {
    data_depth: usize,
    critical_depth: Vec<usize>,
    overall_depth: usize,
    data_period_ps: u32,
    critical_timing: Vec<usize>,
    overall_period_ps: u32,
}

#[allow(clippy::too_many_lines)]
fn mapped_lut_profile(netlist: &Ecp5Netlist) -> MappedLutProfile {
    let depths = mapped_lut_depths(netlist);
    let data_endpoints = netlist
        .cells
        .iter()
        .enumerate()
        .filter_map(|(index, cell)| match cell {
            Ecp5Cell::FlipFlop { data, .. } => Some((index, bit_lut_depth(*data, &depths))),
            _ => None,
        })
        .collect::<Vec<_>>();
    let data_depth = data_endpoints
        .iter()
        .map(|(_, depth)| *depth)
        .max()
        .unwrap_or(0);
    let control_depths = netlist.cells.iter().filter_map(|cell| match cell {
        Ecp5Cell::FlipFlop { enable, .. } => {
            enable.map(|control| bit_lut_depth(control.signal, &depths))
        }
        Ecp5Cell::BlockRam {
            write_enable,
            read_enable,
            ..
        } => Some(
            read_enable
                .map_or(0, |control| bit_lut_depth(control.signal, &depths))
                .max(bit_lut_depth(write_enable.signal, &depths)),
        ),
        Ecp5Cell::Lut4 { .. } | Ecp5Cell::Ccu2c { .. } => None,
    });
    let output_depths = netlist
        .ports
        .iter()
        .filter(|port| port.direction == PortDirection::Output)
        .flat_map(|port| &port.bits)
        .map(|bit| bit_lut_depth(*bit, &depths));
    let overall_depth = [data_depth]
        .into_iter()
        .chain(control_depths)
        .chain(output_depths)
        .max()
        .unwrap_or(0);
    let critical_depth = data_endpoints
        .into_iter()
        .filter_map(|(index, depth)| (depth == data_depth && depth > 0).then_some(index))
        .collect();
    let arrivals = mapped_timing_arrivals(netlist);
    let fanouts = mapped_wire_fanouts(netlist);
    let timing_endpoints = netlist
        .cells
        .iter()
        .enumerate()
        .filter_map(|(index, cell)| match cell {
            Ecp5Cell::FlipFlop { data, .. } => {
                Some((index, mapped_setup_period(*data, &arrivals, &fanouts)))
            }
            _ => None,
        })
        .collect::<Vec<_>>();
    let data_period_ps = timing_endpoints
        .iter()
        .map(|(_, period)| *period)
        .max()
        .unwrap_or(0);
    let critical_timing = timing_endpoints
        .into_iter()
        .filter_map(|(index, period)| (period == data_period_ps && period > 0).then_some(index))
        .collect();
    let control_periods = netlist.cells.iter().filter_map(|cell| match cell {
        Ecp5Cell::FlipFlop { enable, .. } => {
            enable.map(|control| mapped_setup_period(control.signal, &arrivals, &fanouts))
        }
        Ecp5Cell::BlockRam {
            write_enable,
            read_enable,
            ..
        } => Some(
            read_enable
                .map_or(0, |control| {
                    mapped_setup_period(control.signal, &arrivals, &fanouts)
                })
                .max(mapped_setup_period(
                    write_enable.signal,
                    &arrivals,
                    &fanouts,
                )),
        ),
        Ecp5Cell::Lut4 { .. } | Ecp5Cell::Ccu2c { .. } => None,
    });
    let output_periods = netlist
        .ports
        .iter()
        .filter(|port| port.direction == PortDirection::Output)
        .flat_map(|port| &port.bits)
        .map(|bit| mapped_output_period(*bit, &arrivals, &fanouts));
    let overall_period_ps = [data_period_ps]
        .into_iter()
        .chain(control_periods)
        .chain(output_periods)
        .max()
        .unwrap_or(0);
    MappedLutProfile {
        data_depth,
        critical_depth,
        overall_depth,
        data_period_ps,
        critical_timing,
        overall_period_ps,
    }
}

fn mapped_lut_depths(netlist: &Ecp5Netlist) -> HashMap<u32, usize> {
    let mut depths = HashMap::<u32, usize>::new();
    for port in &netlist.ports {
        if port.direction == PortDirection::Input {
            for bit in &port.bits {
                if let Bit::Wire(wire) = bit {
                    depths.entry(*wire).or_insert(0);
                }
            }
        }
    }
    for cell in &netlist.cells {
        match cell {
            Ecp5Cell::FlipFlop { output, .. } => {
                depths.insert(*output, 0);
            }
            Ecp5Cell::BlockRam { read_data, .. } => {
                for output in read_data {
                    depths.insert(*output, 0);
                }
            }
            Ecp5Cell::Ccu2c {
                sums, carry_out, ..
            } => {
                for output in sums.iter().chain([carry_out]) {
                    depths.entry(*output).or_insert(0);
                }
            }
            Ecp5Cell::Lut4 { .. } => {}
        }
    }
    // Retimed cells need not remain in producer-before-consumer order. Iterate
    // to a fixed point so the score is independent of JSON cell ordering.
    for _ in 0..netlist.cells.len() {
        let mut progress = false;
        for cell in &netlist.cells {
            let Ecp5Cell::Lut4 { inputs, output, .. } = cell else {
                continue;
            };
            let input_depths = inputs
                .iter()
                .map(|input| match input {
                    Bit::Wire(wire) => depths.get(wire).copied(),
                    Bit::Zero | Bit::One => Some(0),
                })
                .collect::<Option<Vec<_>>>();
            if let Some(input_depths) = input_depths {
                let depth = 1 + input_depths.into_iter().max().unwrap_or(0);
                if depths.insert(*output, depth) != Some(depth) {
                    progress = true;
                }
            }
        }
        if !progress {
            break;
        }
    }
    depths
}

fn bit_lut_depth(bit: Bit, depths: &HashMap<u32, usize>) -> usize {
    match bit {
        Bit::Wire(wire) => depths.get(&wire).copied().unwrap_or(0),
        Bit::Zero | Bit::One => 0,
    }
}

fn mapped_timing_arrivals(netlist: &Ecp5Netlist) -> HashMap<u32, u32> {
    let fanouts = mapped_wire_fanouts(netlist);
    let carry_outputs = netlist
        .cells
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::Ccu2c { carry_out, .. } => Some(*carry_out),
            _ => None,
        })
        .collect::<HashSet<_>>();
    let mut arrivals = HashMap::<u32, u32>::new();
    for port in &netlist.ports {
        if port.direction == PortDirection::Input {
            for bit in &port.bits {
                if let Bit::Wire(wire) = bit {
                    arrivals.entry(*wire).or_insert(0);
                }
            }
        }
    }
    for cell in &netlist.cells {
        match cell {
            Ecp5Cell::FlipFlop { output, .. } => {
                arrivals.insert(*output, FLIP_FLOP_CLOCK_TO_OUTPUT_PS);
            }
            Ecp5Cell::BlockRam { read_data, .. } => {
                for output in read_data {
                    arrivals.insert(*output, BRAM_CLOCK_TO_OUTPUT_PS);
                }
            }
            Ecp5Cell::Lut4 { .. } | Ecp5Cell::Ccu2c { .. } => {}
        }
    }
    for _ in 0..netlist.cells.len() {
        let mut progress = false;
        for cell in &netlist.cells {
            match cell {
                Ecp5Cell::Lut4 { inputs, output, .. } => {
                    let input_arrivals = inputs
                        .iter()
                        .map(|input| mapped_routed_arrival(*input, &arrivals, &fanouts))
                        .collect::<Option<Vec<_>>>();
                    if let Some(input_arrivals) = input_arrivals {
                        let arrival = input_arrivals
                            .into_iter()
                            .max()
                            .unwrap_or(0)
                            .saturating_add(LUT_DELAY_PS);
                        progress |= arrivals.insert(*output, arrival) != Some(arrival);
                    }
                }
                Ecp5Cell::Ccu2c {
                    inputs,
                    carry_in,
                    sums,
                    carry_out,
                    ..
                } => {
                    let Some(first_inputs) = mapped_ccu_inputs(inputs[0], &arrivals, &fanouts)
                    else {
                        continue;
                    };
                    let Some(second_inputs) = mapped_ccu_inputs(inputs[1], &arrivals, &fanouts)
                    else {
                        continue;
                    };
                    let carry = match carry_in {
                        Bit::Zero | Bit::One => Some(CCU_CARRY_PS),
                        Bit::Wire(wire) if carry_outputs.contains(wire) => arrivals
                            .get(wire)
                            .map(|arrival| arrival.saturating_add(CCU_CARRY_PS)),
                        bit @ Bit::Wire(_) => mapped_routed_arrival(*bit, &arrivals, &fanouts)
                            .map(|arrival| arrival.saturating_add(CCU_INPUT_PS)),
                    };
                    let Some(carry) = carry else {
                        continue;
                    };
                    let first = first_inputs.max(carry);
                    let sum0 = first.saturating_add(CCU_SUM_PS);
                    let internal_carry = first.saturating_add(CCU_CARRY_PS);
                    let second = second_inputs.max(internal_carry);
                    let sum1 = second.saturating_add(CCU_SUM_PS);
                    let carry_out_arrival = second.saturating_add(CCU_CARRY_PS);
                    progress |= arrivals.insert(sums[0], sum0) != Some(sum0);
                    progress |= arrivals.insert(sums[1], sum1) != Some(sum1);
                    progress |=
                        arrivals.insert(*carry_out, carry_out_arrival) != Some(carry_out_arrival);
                }
                Ecp5Cell::FlipFlop { .. } | Ecp5Cell::BlockRam { .. } => {}
            }
        }
        if !progress {
            break;
        }
    }
    arrivals
}

fn mapped_ccu_inputs(
    inputs: [Bit; 4],
    arrivals: &HashMap<u32, u32>,
    fanouts: &HashMap<u32, usize>,
) -> Option<u32> {
    inputs
        .into_iter()
        .map(|input| mapped_routed_arrival(input, arrivals, fanouts))
        .collect::<Option<Vec<_>>>()
        .map(|arrivals| {
            arrivals
                .into_iter()
                .max()
                .unwrap_or(0)
                .saturating_add(CCU_INPUT_PS)
        })
}

fn mapped_routed_arrival(
    bit: Bit,
    arrivals: &HashMap<u32, u32>,
    fanouts: &HashMap<u32, usize>,
) -> Option<u32> {
    match bit {
        Bit::Zero | Bit::One => Some(0),
        Bit::Wire(wire) => arrivals.get(&wire).map(|arrival| {
            arrival
                .saturating_add(wire_delay_ps(fanouts.get(&wire).copied().unwrap_or(1)))
                .saturating_add(MAPPED_ROUTE_GUARD_PS)
        }),
    }
}

fn mapped_setup_period(
    bit: Bit,
    arrivals: &HashMap<u32, u32>,
    fanouts: &HashMap<u32, usize>,
) -> u32 {
    mapped_routed_arrival(bit, arrivals, fanouts)
        .unwrap_or(0)
        .saturating_add(FLIP_FLOP_SETUP_PS)
}

fn mapped_output_period(
    bit: Bit,
    arrivals: &HashMap<u32, u32>,
    fanouts: &HashMap<u32, usize>,
) -> u32 {
    mapped_routed_arrival(bit, arrivals, fanouts).unwrap_or(0)
}

fn mapped_wire_fanouts(netlist: &Ecp5Netlist) -> HashMap<u32, usize> {
    let mut fanouts = HashMap::new();
    for bit in netlist.cells.iter().flat_map(cell_input_bits).chain(
        netlist
            .ports
            .iter()
            .filter(|port| port.direction == PortDirection::Output)
            .flat_map(|port| &port.bits)
            .copied(),
    ) {
        if let Bit::Wire(wire) = bit {
            *fanouts.entry(wire).or_insert(0) += 1;
        }
    }
    fanouts
}

#[allow(clippy::too_many_lines)]
fn backward_retime_lut(netlist: &Ecp5Netlist, register_index: usize) -> Option<Ecp5Netlist> {
    let Ecp5Cell::FlipFlop {
        name: register_name,
        data: Bit::Wire(data_wire),
        output: register_output,
        clock,
        edge: clock_edge,
        enable,
        reset: Some(reset),
    } = netlist.cells.get(register_index)?
    else {
        return None;
    };
    if mapped_wire_is_clock_or_reset(netlist, *register_output) {
        return None;
    }
    let (lut_index, lut_name, lut_inputs, lut_init) =
        netlist
            .cells
            .iter()
            .enumerate()
            .find_map(|(index, cell)| match cell {
                Ecp5Cell::Lut4 {
                    name,
                    inputs,
                    output,
                    init,
                } if output == data_wire => Some((index, name.clone(), *inputs, *init)),
                _ => None,
            })?;
    let mut input_wires = lut_inputs
        .iter()
        .filter_map(|input| match input {
            Bit::Wire(wire) => Some(*wire),
            Bit::Zero | Bit::One => None,
        })
        .collect::<Vec<_>>();
    input_wires.sort_unstable();
    input_wires.dedup();
    if input_wires.is_empty() || input_wires.len() > 4 {
        return None;
    }
    let function = reduced_lut_function(lut_inputs, lut_init, &input_wires);
    let input_count = input_wires.len();
    let mut vertices = input_wires
        .iter()
        .map(|wire| RetimingVertex::boundary(format!("wire{wire}"), LogicFunction::new(0, 0)))
        .collect::<Vec<_>>();
    vertices.push(RetimingVertex::logic("lut", function));
    vertices.push(RetimingVertex::boundary("q", LogicFunction::new(1, 0b10)));
    let lut_vertex = input_count;
    let q_vertex = input_count + 1;
    let mut edges = (0..input_count)
        .map(|input| RetimingEdge::new(input, lut_vertex, Vec::new()))
        .collect::<Vec<_>>();
    edges.push(RetimingEdge::new(lut_vertex, q_vertex, vec![reset.value]));
    let before = RetimingGraph::new(
        RetimingDomain::new(
            format!("{clock:?}"),
            *clock_edge,
            format!("{:?}", reset.signal),
            reset.active,
            reset.asynchronous,
        ),
        vertices,
        edges,
    );
    let mut labels = vec![0; input_count + 2];
    labels[lut_vertex] = 1;
    let certificate = RetimingCertificate::new(labels);
    let after = derive_retimed_graph(&before, &certificate).ok()?;
    verify_retiming_certificate(&before, &after, &certificate).ok()?;

    // Allocate above the original maximum before removing the sink FF. If its
    // Q is the maximum wire, measuring afterwards would reuse Q for a new FF
    // and create two drivers when the retimed LUT takes over that same Q.
    let mut next_wire = maximum_mapped_wire(netlist)?.checked_add(1)?;
    let mut candidate = netlist.clone();
    candidate.cells.remove(register_index);
    let mut replacements = HashMap::new();
    for (input, reset_edge) in input_wires.iter().zip(after.edges()) {
        let output = next_wire;
        next_wire = next_wire.checked_add(1)?;
        replacements.insert(*input, output);
        candidate.cells.push(Ecp5Cell::FlipFlop {
            name: format!("retime_{register_name}_{input}"),
            data: Bit::Wire(*input),
            output,
            clock: *clock,
            edge: *clock_edge,
            enable: *enable,
            reset: Some(Reset {
                value: reset_edge.reset_values()[0],
                ..*reset
            }),
        });
    }
    let new_inputs = lut_inputs.map(|input| match input {
        Bit::Wire(wire) => Bit::Wire(replacements[&wire]),
        constant => constant,
    });
    let fanout = mapped_wire_fanout(netlist, *data_wire);
    let retimed_lut = Ecp5Cell::Lut4 {
        name: format!("retime_{lut_name}"),
        inputs: new_inputs,
        output: *register_output,
        init: lut_init,
    };
    if fanout == 1 {
        let lut_index = lut_index - usize::from(register_index < lut_index);
        candidate.cells[lut_index] = retimed_lut;
    } else {
        candidate.cells.push(retimed_lut);
    }
    prune_unobservable_retiming_cells(&mut candidate);
    Some(candidate)
}

fn prune_unobservable_retiming_cells(netlist: &mut Ecp5Netlist) {
    loop {
        let fanouts = mapped_wire_fanouts(netlist);
        let previous_len = netlist.cells.len();
        netlist.cells.retain(|cell| {
            !mapped_cell_name(cell).starts_with("retime_")
                || cell_output_bits(cell)
                    .into_iter()
                    .any(|bit| matches!(bit, Bit::Wire(wire) if fanouts.contains_key(&wire)))
        });
        if netlist.cells.len() == previous_len {
            break;
        }
    }
}

fn mapped_cell_name(cell: &Ecp5Cell) -> &str {
    match cell {
        Ecp5Cell::Lut4 { name, .. }
        | Ecp5Cell::Ccu2c { name, .. }
        | Ecp5Cell::FlipFlop { name, .. }
        | Ecp5Cell::BlockRam { name, .. } => name,
    }
}

fn mapped_wire_is_clock_or_reset(netlist: &Ecp5Netlist, wire: u32) -> bool {
    netlist.cells.iter().any(|cell| match cell {
        Ecp5Cell::FlipFlop { clock, reset, .. } => {
            *clock == Bit::Wire(wire)
                || reset.is_some_and(|control| control.signal == Bit::Wire(wire))
        }
        Ecp5Cell::BlockRam { clock, .. } => *clock == Bit::Wire(wire),
        Ecp5Cell::Lut4 { .. } | Ecp5Cell::Ccu2c { .. } => false,
    })
}

fn reduced_lut_function(inputs: [Bit; 4], init: u16, variables: &[u32]) -> LogicFunction {
    let mut truth_table = 0u64;
    for assignment in 0..(1usize << variables.len()) {
        let lut_index = inputs
            .iter()
            .enumerate()
            .fold(0usize, |index, (pin, input)| {
                let value = match input {
                    Bit::Zero => false,
                    Bit::One => true,
                    Bit::Wire(wire) => {
                        let variable = variables
                            .iter()
                            .position(|candidate| candidate == wire)
                            .expect("wire was collected as a variable");
                        assignment & (1usize << variable) != 0
                    }
                };
                index | (usize::from(value) << pin)
            });
        if init & (1u16 << lut_index) != 0 {
            truth_table |= 1u64 << assignment;
        }
    }
    LogicFunction::new(
        u8::try_from(variables.len()).expect("LUT has at most four variables"),
        truth_table,
    )
}

fn mapped_wire_fanout(netlist: &Ecp5Netlist, wire: u32) -> usize {
    netlist
        .cells
        .iter()
        .map(|cell| {
            cell_input_bits(cell)
                .into_iter()
                .filter(|bit| *bit == Bit::Wire(wire))
                .count()
        })
        .sum::<usize>()
        + netlist
            .ports
            .iter()
            .flat_map(|port| &port.bits)
            .filter(|bit| **bit == Bit::Wire(wire))
            .count()
}

fn maximum_mapped_wire(netlist: &Ecp5Netlist) -> Option<u32> {
    netlist
        .ports
        .iter()
        .flat_map(|port| &port.bits)
        .copied()
        .chain(netlist.cells.iter().flat_map(cell_output_bits))
        .filter_map(|bit| match bit {
            Bit::Wire(wire) => Some(wire),
            Bit::Zero | Bit::One => None,
        })
        .max()
}

fn cell_input_bits(cell: &Ecp5Cell) -> Vec<Bit> {
    match cell {
        Ecp5Cell::Lut4 { inputs, .. } => inputs.to_vec(),
        Ecp5Cell::Ccu2c {
            inputs, carry_in, ..
        } => inputs
            .iter()
            .flatten()
            .copied()
            .chain([*carry_in])
            .collect(),
        Ecp5Cell::FlipFlop {
            data,
            clock,
            enable,
            reset,
            ..
        } => [*data, *clock]
            .into_iter()
            .chain(enable.map(|control| control.signal))
            .chain(reset.map(|control| control.signal))
            .collect(),
        Ecp5Cell::BlockRam {
            write_address,
            write_data,
            write_enable,
            read_address,
            read_enable,
            clock,
            ..
        } => write_address
            .iter()
            .chain(write_data)
            .chain(read_address.iter())
            .copied()
            .chain([write_enable.signal, *clock])
            .chain(read_enable.map(|control| control.signal))
            .collect(),
    }
}

fn cell_output_bits(cell: &Ecp5Cell) -> Vec<Bit> {
    match cell {
        Ecp5Cell::Lut4 { output, .. } | Ecp5Cell::FlipFlop { output, .. } => {
            vec![Bit::Wire(*output)]
        }
        Ecp5Cell::Ccu2c {
            sums, carry_out, ..
        } => sums
            .iter()
            .copied()
            .chain([*carry_out])
            .map(Bit::Wire)
            .collect(),
        Ecp5Cell::BlockRam { read_data, .. } => read_data.iter().copied().map(Bit::Wire).collect(),
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct MappingQuality {
    period_ps: u32,
}

fn map_once(
    netlist: &Netlist,
    options: MappingOptions,
) -> Result<(Ecp5Netlist, MappingQuality), MappingError> {
    netlist.validate()?;
    let demand = MappingDemand::collect(netlist);
    let cuts = CutDatabase::analyze(netlist);
    let cover = LutCover::select(netlist, &cuts, &demand.roots, options);
    let (period_ps, _) = cover.estimated_register_period_ps(netlist);
    let quality = MappingQuality { period_ps };
    let mut emitter = LutEmitter::new(netlist, &cover);

    map_retained_cells(netlist, options, &mut emitter);

    for root in &demand.roots {
        emitter.map_net(*root);
    }

    for port in netlist
        .ports()
        .iter()
        .filter(|port| port.direction() == IrPortDirection::Output)
    {
        for output in port.bits() {
            let node = node_for(netlist, *output);
            let source = node.inputs()[0];
            let output_bit = emitter.mapped_net(source);
            emitter.alias_net(*output, output_bit);
        }
    }

    let (bits, mut cells) = emitter.finish();

    for register in netlist.registers() {
        cells.push(Ecp5Cell::FlipFlop {
            // nextpnr rejects a cell whose name is also a top-level IO name.
            // Keep primitive cells in a dedicated namespace even when an RTL
            // output is directly registered.
            name: format!("ff_{}", register.name()),
            data: mapped_bit(&bits, register.data()),
            output: wire_number(register.output()),
            clock: mapped_bit(&bits, register.clock()),
            edge: register.edge(),
            enable: register.enable().map(|enable| Control {
                signal: mapped_bit(&bits, enable.signal),
                active: enable.active,
            }),
            reset: register.reset().map(|reset| Reset {
                signal: mapped_bit(&bits, reset.signal),
                active: reset.active,
                asynchronous: reset.asynchronous,
                value: reset.value,
            }),
        });
    }

    for memory in netlist.memories() {
        map_memory(memory, &bits, &mut cells)?;
    }

    let ports = netlist
        .ports()
        .iter()
        .map(|port| MappedPort {
            name: port.name().into(),
            direction: match port.direction() {
                IrPortDirection::Input => PortDirection::Input,
                IrPortDirection::Output => PortDirection::Output,
            },
            bits: port
                .bits()
                .iter()
                .map(|net| mapped_bit(&bits, *net))
                .collect(),
        })
        .collect();

    Ok((
        Ecp5Netlist {
            name: netlist.name().into(),
            ports,
            cells,
            retiming: RetimingSelection {
                applied: false,
                original_lut_depth: 0,
                selected_lut_depth: 0,
                original_critical_registers: 0,
                selected_critical_registers: 0,
                original_period_ps: quality.period_ps,
                selected_period_ps: quality.period_ps,
                original_registers: netlist.registers().len(),
                selected_registers: netlist.registers().len(),
            },
        },
        quality,
    ))
}

const CCU2C_ARITH_INIT: u16 = 0x96aa;

#[derive(Clone, Copy)]
enum RetainedCell<'a> {
    Arithmetic(&'a ArithmeticCell),
    Comparison(&'a ComparisonCell),
}

fn map_retained_cells(netlist: &Netlist, options: MappingOptions, emitter: &mut LutEmitter<'_>) {
    let mut retained = netlist
        .arithmetic()
        .iter()
        .map(RetainedCell::Arithmetic)
        .chain(netlist.comparisons().iter().map(RetainedCell::Comparison))
        .collect::<Vec<_>>();
    retained.sort_by_key(|cell| cell.output_index());
    for cell in retained {
        match cell {
            RetainedCell::Arithmetic(arithmetic) => {
                let use_carry = match options.arithmetic {
                    ArithmeticMapping::Auto => arithmetic.outputs().len() > 4,
                    ArithmeticMapping::CarryChain => true,
                    ArithmeticMapping::Lut4 => false,
                };
                if use_carry {
                    map_arithmetic_carry(arithmetic, emitter);
                } else {
                    map_arithmetic_luts(arithmetic, emitter);
                }
            }
            RetainedCell::Comparison(comparison) => map_comparison_carry(comparison, emitter),
        }
    }
}

impl RetainedCell<'_> {
    fn output_index(self) -> u32 {
        match self {
            Self::Arithmetic(cell) => cell.outputs()[0].index(),
            Self::Comparison(cell) => cell.output().index(),
        }
    }
}

fn map_arithmetic_carry(arithmetic: &ArithmeticCell, emitter: &mut LutEmitter<'_>) {
    let subtract = arithmetic.operation() == ArithmeticOp::Subtract;
    let mut carry = Bit::from(subtract);
    for (pair, bit) in (0..arithmetic.outputs().len()).step_by(2).enumerate() {
        let mut inputs = [[Bit::Zero; 4]; 2];
        let mut sums = [0; 2];
        for slice in 0..2 {
            let index = bit + slice;
            if index < arithmetic.outputs().len() {
                inputs[slice] = [
                    emitter.map_net(arithmetic.lhs()[index]),
                    emitter.map_net(arithmetic.rhs()[index]),
                    Bit::from(subtract),
                    Bit::One,
                ];
                sums[slice] = wire_number(arithmetic.outputs()[index]);
                emitter.alias_net(arithmetic.outputs()[index], Bit::Wire(sums[slice]));
            } else {
                inputs[slice] = [Bit::Zero, Bit::Zero, Bit::from(subtract), Bit::One];
                sums[slice] = emitter.fresh_wire();
            }
        }
        let carry_out = emitter.fresh_wire();
        emitter.push_cell(Ecp5Cell::Ccu2c {
            name: format!("ccu_{}_{pair}", arithmetic.name()),
            inputs,
            carry_in: carry,
            sums,
            carry_out,
            init: [CCU2C_ARITH_INIT; 2],
            inject: [false; 2],
        });
        carry = Bit::Wire(carry_out);
    }
}

fn map_comparison_carry(comparison: &ComparisonCell, emitter: &mut LutEmitter<'_>) {
    let mut carry = Bit::from(comparison.operation().includes_equal());
    let pairs = comparison.lhs().len().div_ceil(2);
    for (pair, bit) in (0..comparison.lhs().len()).step_by(2).enumerate() {
        let mut inputs = [[Bit::Zero; 4]; 2];
        let mut sums = [0; 2];
        for slice in 0..2 {
            let index = bit + slice;
            inputs[slice] = if index < comparison.lhs().len() {
                [
                    emitter.map_net(comparison.rhs()[index]),
                    emitter.map_net(comparison.lhs()[index]),
                    Bit::One,
                    Bit::One,
                ]
            } else {
                // The unused high slice of an odd-width comparison computes
                // 0 + !0 + carry, propagating carry without changing it.
                [Bit::Zero, Bit::Zero, Bit::One, Bit::One]
            };
            sums[slice] = emitter.fresh_wire();
        }
        let last = pair + 1 == pairs;
        let carry_out = if last && !comparison.operation().is_signed() {
            wire_number(comparison.output())
        } else {
            emitter.fresh_wire()
        };
        emitter.push_cell(Ecp5Cell::Ccu2c {
            name: format!("ccu_{}_{pair}", comparison.name()),
            inputs,
            carry_in: carry,
            sums,
            carry_out,
            init: [CCU2C_ARITH_INIT; 2],
            inject: [false; 2],
        });
        carry = Bit::Wire(carry_out);
    }

    if comparison.operation().is_signed() {
        let output = wire_number(comparison.output());
        let most_significant = comparison.lhs().len() - 1;
        let lhs_sign = emitter.map_net(comparison.lhs()[most_significant]);
        let rhs_sign = emitter.map_net(comparison.rhs()[most_significant]);
        emitter.push_cell(Ecp5Cell::Lut4 {
            name: format!("lut_{}_signed", comparison.name()),
            inputs: [lhs_sign, rhs_sign, carry, Bit::Zero],
            output,
            init: signed_comparison_truth_table(),
        });
        emitter.alias_net(comparison.output(), Bit::Wire(output));
    } else {
        emitter.alias_net(comparison.output(), carry);
    }
}

fn signed_comparison_truth_table() -> u16 {
    (0..16).fold(0, |table, assignment| {
        let lhs_sign = assignment & 1 != 0;
        let rhs_sign = assignment & 2 != 0;
        let unsigned_relation = assignment & 4 != 0;
        let value = if lhs_sign == rhs_sign {
            unsigned_relation
        } else {
            lhs_sign
        };
        table | (u16::from(value) << assignment)
    })
}

fn map_arithmetic_luts(arithmetic: &ArithmeticCell, emitter: &mut LutEmitter<'_>) {
    let subtract = arithmetic.operation() == ArithmeticOp::Subtract;
    let mut carry = Bit::from(subtract);
    for bit in 0..arithmetic.outputs().len() {
        let lhs = emitter.map_net(arithmetic.lhs()[bit]);
        let rhs = emitter.map_net(arithmetic.rhs()[bit]);
        let output = wire_number(arithmetic.outputs()[bit]);
        emitter.push_cell(Ecp5Cell::Lut4 {
            name: format!("lut_{}_sum_{bit}", arithmetic.name()),
            inputs: [lhs, rhs, carry, Bit::Zero],
            output,
            init: arithmetic_truth_table(subtract, false),
        });
        emitter.alias_net(arithmetic.outputs()[bit], Bit::Wire(output));

        if bit + 1 < arithmetic.outputs().len() {
            let carry_out = emitter.fresh_wire();
            emitter.push_cell(Ecp5Cell::Lut4 {
                name: format!("lut_{}_carry_{bit}", arithmetic.name()),
                inputs: [lhs, rhs, carry, Bit::Zero],
                output: carry_out,
                init: arithmetic_truth_table(subtract, true),
            });
            carry = Bit::Wire(carry_out);
        }
    }
}

fn arithmetic_truth_table(subtract: bool, carry_output: bool) -> u16 {
    (0..16).fold(0, |table, assignment| {
        let lhs = assignment & 1 != 0;
        let rhs = assignment & 2 != 0;
        let carry = assignment & 4 != 0;
        let effective_rhs = rhs ^ subtract;
        let value = if carry_output {
            u8::from(lhs) + u8::from(effective_rhs) + u8::from(carry) >= 2
        } else {
            lhs ^ effective_rhs ^ carry
        };
        table | (u16::from(value) << assignment)
    })
}

fn node_for(netlist: &Netlist, net: NetId) -> &struo_ir::Node {
    &netlist.nodes()[net.index() as usize]
}

fn map_memory(
    memory: &MemoryCell,
    bits: &[Option<Bit>],
    cells: &mut Vec<Ecp5Cell>,
) -> Result<(), MappingError> {
    let geometry_error = || MappingError::UnsupportedMemoryGeometry {
        memory: memory.name().into(),
        depth: memory.depth(),
        width: memory.write_data().len(),
    };
    let chunk_width = maximum_block_ram_width(memory.depth()).ok_or_else(&geometry_error)?;
    let chunk_count = memory.write_data().len().div_ceil(usize::from(chunk_width));
    for (chunk, (write_data, read_data)) in memory
        .write_data()
        .chunks(usize::from(chunk_width))
        .zip(memory.read_data().chunks(usize::from(chunk_width)))
        .enumerate()
    {
        let word_width = u8::try_from(write_data.len()).map_err(|_| geometry_error())?;
        let physical_width =
            block_ram_width(memory.depth(), word_width).ok_or_else(&geometry_error)?;
        let mut write_address = physical_address(bits, memory.write_address(), physical_width);
        if physical_width == 18 {
            // In 18-bit mode ADA0/ADA1 are the two 9-bit byte enables rather
            // than address pins. This IR writes whole words only.
            write_address[0] = Bit::One;
            write_address[1] = Bit::One;
        }
        cells.push(Ecp5Cell::BlockRam {
            name: if chunk_count == 1 {
                format!("bram_{}", memory.name())
            } else {
                format!("bram_{}_{chunk}", memory.name())
            },
            depth: memory.depth(),
            word_width,
            physical_width,
            write_address: Box::new(write_address),
            write_data: write_data
                .iter()
                .map(|net| mapped_bit(bits, *net))
                .collect(),
            write_enable: Control {
                signal: mapped_bit(bits, memory.write_enable().signal),
                active: memory.write_enable().active,
            },
            read_address: Box::new(physical_address(
                bits,
                memory.read_address(),
                physical_width,
            )),
            read_data: read_data.iter().map(|net| wire_number(*net)).collect(),
            read_enable: memory.read_enable().map(|enable| Control {
                signal: mapped_bit(bits, enable.signal),
                active: enable.active,
            }),
            clock: mapped_bit(bits, memory.clock()),
            edge: memory.edge(),
        });
    }
    Ok(())
}

impl From<bool> for Bit {
    fn from(value: bool) -> Self {
        if value { Self::One } else { Self::Zero }
    }
}

fn wire_number(net: NetId) -> u32 {
    net.index()
        .checked_add(2)
        .expect("net identifier exceeds the Yosys JSON range")
}

fn wire_for(net: NetId) -> Bit {
    Bit::Wire(wire_number(net))
}

fn mapped_bit(bits: &[Option<Bit>], net: NetId) -> Bit {
    bits[net.index() as usize].expect("validated nodes are topologically ordered")
}

fn block_ram_width(depth: u32, word_width: u8) -> Option<u8> {
    [
        (1u8, 16_384u32),
        (2, 8_192),
        (4, 4_096),
        (9, 2_048),
        (18, 1_024),
    ]
    .into_iter()
    .find_map(|(width, capacity)| (word_width <= width && depth <= capacity).then_some(width))
}

fn maximum_block_ram_width(depth: u32) -> Option<u8> {
    [
        (18u8, 1_024u32),
        (9, 2_048),
        (4, 4_096),
        (2, 8_192),
        (1, 16_384),
    ]
    .into_iter()
    .find_map(|(width, capacity)| (depth <= capacity).then_some(width))
}

fn physical_address(bits: &[Option<Bit>], address: &[NetId], width: u8) -> [Bit; 14] {
    let shift = match width {
        1 => 0,
        2 => 1,
        4 => 2,
        9 => 3,
        18 => 4,
        _ => unreachable!("validated DP16KD width"),
    };
    let mut physical = [Bit::Zero; 14];
    for (target, source) in physical[shift..].iter_mut().zip(address) {
        *target = mapped_bit(bits, *source);
    }
    physical
}

/// ECP5 technology-mapping failure.
#[derive(Debug)]
pub enum MappingError {
    /// Source netlist is invalid.
    InvalidNetlist(ValidationError),
    /// A logical memory cannot be width-tiled into DP16KD primitives.
    UnsupportedMemoryGeometry {
        /// Memory name.
        memory: String,
        /// Requested word count.
        depth: u32,
        /// Requested word width.
        width: usize,
    },
}

impl Display for MappingError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidNetlist(error) => write!(formatter, "invalid netlist: {error}"),
            Self::UnsupportedMemoryGeometry {
                memory,
                depth,
                width,
            } => write!(
                formatter,
                "memory {memory} ({depth}x{width}) cannot be mapped to ECP5 DP16KD primitives"
            ),
        }
    }
}

impl Error for MappingError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidNetlist(error) => Some(error),
            Self::UnsupportedMemoryGeometry { .. } => None,
        }
    }
}

impl From<ValidationError> for MappingError {
    fn from(error: ValidationError) -> Self {
        Self::InvalidNetlist(error)
    }
}

#[derive(Serialize)]
struct JsonDesign {
    creator: &'static str,
    modules: BTreeMap<String, JsonModule>,
}

#[derive(Serialize)]
struct JsonModule {
    attributes: BTreeMap<String, String>,
    ports: BTreeMap<String, JsonPort>,
    cells: BTreeMap<String, JsonCell>,
    netnames: BTreeMap<String, JsonNet>,
}

#[derive(Serialize)]
struct JsonPort {
    direction: &'static str,
    bits: Vec<Bit>,
}

#[derive(Serialize)]
struct JsonCell {
    hide_name: u8,
    r#type: &'static str,
    parameters: BTreeMap<String, String>,
    attributes: BTreeMap<String, String>,
    port_directions: BTreeMap<String, &'static str>,
    connections: BTreeMap<String, Vec<Bit>>,
}

#[derive(Serialize)]
struct JsonNet {
    hide_name: u8,
    bits: Vec<Bit>,
    attributes: BTreeMap<String, String>,
}

impl From<&Ecp5Netlist> for JsonDesign {
    fn from(netlist: &Ecp5Netlist) -> Self {
        let ports = netlist
            .ports
            .iter()
            .map(|port| {
                (
                    port.name.clone(),
                    JsonPort {
                        direction: match port.direction {
                            PortDirection::Input => "input",
                            PortDirection::Output => "output",
                        },
                        bits: port.bits.clone(),
                    },
                )
            })
            .collect();
        let netnames = netlist
            .ports
            .iter()
            .map(|port| {
                (
                    port.name.clone(),
                    JsonNet {
                        hide_name: 0,
                        bits: port.bits.clone(),
                        attributes: BTreeMap::new(),
                    },
                )
            })
            .collect();
        let cells = netlist.cells.iter().map(json_cell).collect();
        Self {
            creator: "Struo",
            modules: [(
                netlist.name.clone(),
                JsonModule {
                    attributes: [("top".into(), format!("{:032b}", 1))]
                        .into_iter()
                        .collect(),
                    ports,
                    cells,
                    netnames,
                },
            )]
            .into_iter()
            .collect(),
        }
    }
}

fn json_cell(cell: &Ecp5Cell) -> (String, JsonCell) {
    match cell {
        Ecp5Cell::Lut4 {
            name,
            inputs,
            output,
            init,
        } => (name.clone(), json_lut(*inputs, *output, *init)),
        Ecp5Cell::Ccu2c {
            name,
            inputs,
            carry_in,
            sums,
            carry_out,
            init,
            inject,
        } => (
            name.clone(),
            json_ccu2c(*inputs, *carry_in, *sums, *carry_out, *init, *inject),
        ),
        Ecp5Cell::FlipFlop {
            name,
            data,
            output,
            clock,
            edge,
            enable,
            reset,
        } => (
            name.clone(),
            json_flip_flop(*data, *output, *clock, *edge, *enable, *reset),
        ),
        Ecp5Cell::BlockRam {
            name,
            physical_width,
            write_address,
            write_data,
            write_enable,
            read_address,
            read_data,
            read_enable,
            clock,
            edge,
            ..
        } => (
            name.clone(),
            json_block_ram(
                *physical_width,
                **write_address,
                write_data,
                *write_enable,
                **read_address,
                read_data,
                *read_enable,
                *clock,
                *edge,
            ),
        ),
    }
}

fn json_lut(inputs: [Bit; 4], output: u32, init: u16) -> JsonCell {
    let names = ["A", "B", "C", "D"];
    let mut connections = BTreeMap::new();
    for (name, bit) in names.into_iter().zip(inputs) {
        connections.insert(name.into(), vec![bit]);
    }
    connections.insert("Z".into(), vec![Bit::Wire(output)]);
    JsonCell {
        hide_name: 0,
        r#type: "LUT4",
        parameters: [("INIT".into(), format!("{init:016b}"))]
            .into_iter()
            .collect(),
        attributes: BTreeMap::new(),
        port_directions: [
            ("A".into(), "input"),
            ("B".into(), "input"),
            ("C".into(), "input"),
            ("D".into(), "input"),
            ("Z".into(), "output"),
        ]
        .into_iter()
        .collect(),
        connections,
    }
}

fn json_ccu2c(
    inputs: [[Bit; 4]; 2],
    carry_in: Bit,
    sums: [u32; 2],
    carry_out: u32,
    init: [u16; 2],
    inject: [bool; 2],
) -> JsonCell {
    let mut connections = BTreeMap::new();
    let mut port_directions = BTreeMap::new();
    connections.insert("CIN".into(), vec![carry_in]);
    port_directions.insert("CIN".into(), "input");
    for slice in 0..2 {
        for (port, bit) in ["A", "B", "C", "D"].into_iter().zip(inputs[slice]) {
            let name = format!("{port}{slice}");
            connections.insert(name.clone(), vec![bit]);
            port_directions.insert(name, "input");
        }
        let sum = format!("S{slice}");
        connections.insert(sum.clone(), vec![Bit::Wire(sums[slice])]);
        port_directions.insert(sum, "output");
    }
    connections.insert("COUT".into(), vec![Bit::Wire(carry_out)]);
    port_directions.insert("COUT".into(), "output");
    JsonCell {
        hide_name: 0,
        r#type: "CCU2C",
        parameters: [
            ("INIT0".into(), format!("{:016b}", init[0])),
            ("INIT1".into(), format!("{:016b}", init[1])),
            (
                "INJECT1_0".into(),
                if inject[0] { "YES" } else { "NO" }.into(),
            ),
            (
                "INJECT1_1".into(),
                if inject[1] { "YES" } else { "NO" }.into(),
            ),
        ]
        .into_iter()
        .collect(),
        attributes: BTreeMap::new(),
        port_directions,
        connections,
    }
}

fn json_flip_flop(
    data: Bit,
    output: u32,
    clock: Bit,
    edge: ClockEdge,
    enable: Option<Control>,
    reset: Option<Reset>,
) -> JsonCell {
    let parameters = [
        ("CEMUX".into(), control_mux(enable)),
        (
            "CLKMUX".into(),
            if edge == ClockEdge::Rising {
                "CLK".into()
            } else {
                "INV".into()
            },
        ),
        ("GSR".into(), "DISABLED".into()),
        (
            "LSRMUX".into(),
            reset.map_or_else(
                || "LSR".into(),
                |reset| match reset.active {
                    ActiveLevel::High => "LSR".into(),
                    ActiveLevel::Low => "INV".into(),
                },
            ),
        ),
        (
            "REGSET".into(),
            reset
                .map_or("RESET", |reset| if reset.value { "SET" } else { "RESET" })
                .into(),
        ),
        (
            "SRMODE".into(),
            reset
                .map_or("LSR_OVER_CE", |reset| {
                    if reset.asynchronous {
                        "ASYNC"
                    } else {
                        "LSR_OVER_CE"
                    }
                })
                .into(),
        ),
    ]
    .into_iter()
    .collect();
    JsonCell {
        hide_name: 0,
        r#type: "TRELLIS_FF",
        parameters,
        attributes: BTreeMap::new(),
        port_directions: [
            ("CLK".into(), "input"),
            ("LSR".into(), "input"),
            ("CE".into(), "input"),
            ("DI".into(), "input"),
            ("Q".into(), "output"),
        ]
        .into_iter()
        .collect(),
        connections: [
            ("CLK".into(), vec![clock]),
            (
                "LSR".into(),
                vec![reset.map_or(Bit::Zero, |reset| reset.signal)],
            ),
            (
                "CE".into(),
                vec![enable.map_or(Bit::One, |enable| enable.signal)],
            ),
            ("DI".into(), vec![data]),
            ("Q".into(), vec![Bit::Wire(output)]),
        ]
        .into_iter()
        .collect(),
    }
}

fn control_mux(control: Option<Control>) -> String {
    match control.map(|control| control.active) {
        None => "1 ".into(),
        Some(ActiveLevel::High) => "CE".into(),
        Some(ActiveLevel::Low) => "INV".into(),
    }
}

#[allow(clippy::too_many_arguments)]
fn json_block_ram(
    width: u8,
    write_address: [Bit; 14],
    write_data: &[Bit],
    write_enable: Control,
    read_address: [Bit; 14],
    read_data: &[u32],
    read_enable: Option<Control>,
    clock: Bit,
    edge: ClockEdge,
) -> JsonCell {
    let parameters = block_ram_parameters(width, write_enable, read_enable, edge);

    let mut port_directions = BTreeMap::new();
    let mut connections = BTreeMap::new();
    let mut input = |name: String, bit: Bit| {
        port_directions.insert(name.clone(), "input");
        connections.insert(name, vec![bit]);
    };
    for (index, bit) in write_address.into_iter().enumerate() {
        input(format!("ADA{index}"), bit);
    }
    for index in 0..18 {
        input(
            format!("DIA{index}"),
            write_data.get(index).copied().unwrap_or(Bit::Zero),
        );
    }
    input("CEA".into(), Bit::One);
    input("OCEA".into(), Bit::One);
    input("CLKA".into(), clock);
    input("WEA".into(), write_enable.signal);
    input("RSTA".into(), Bit::Zero);
    for index in 0..3 {
        input(format!("CSA{index}"), Bit::Zero);
    }
    for (index, bit) in read_address.into_iter().enumerate() {
        input(format!("ADB{index}"), bit);
    }
    for index in 0..18 {
        input(format!("DIB{index}"), Bit::Zero);
    }
    input(
        "CEB".into(),
        read_enable.map_or(Bit::One, |enable| enable.signal),
    );
    input("OCEB".into(), Bit::One);
    input("CLKB".into(), clock);
    input("WEB".into(), Bit::Zero);
    input("RSTB".into(), Bit::Zero);
    for index in 0..3 {
        input(format!("CSB{index}"), Bit::Zero);
    }
    for (index, wire) in read_data.iter().enumerate() {
        let name = format!("DOB{index}");
        port_directions.insert(name.clone(), "output");
        connections.insert(name, vec![Bit::Wire(*wire)]);
    }
    JsonCell {
        hide_name: 0,
        r#type: "DP16KD",
        parameters,
        attributes: BTreeMap::new(),
        port_directions,
        connections,
    }
}

fn block_ram_parameters(
    width: u8,
    write_enable: Control,
    read_enable: Option<Control>,
    edge: ClockEdge,
) -> BTreeMap<String, String> {
    let mut parameters = BTreeMap::from([
        ("DATA_WIDTH_A".into(), width.to_string()),
        ("DATA_WIDTH_B".into(), width.to_string()),
        ("REGMODE_A".into(), "NOREG".into()),
        ("REGMODE_B".into(), "NOREG".into()),
        ("RESETMODE".into(), "SYNC".into()),
        ("ASYNC_RESET_RELEASE".into(), "SYNC".into()),
        ("CSDECODE_A".into(), "0b000".into()),
        ("CSDECODE_B".into(), "0b000".into()),
        ("WRITEMODE_A".into(), "NORMAL".into()),
        ("WRITEMODE_B".into(), "NORMAL".into()),
        ("GSR".into(), "DISABLED".into()),
        (
            "CLKAMUX".into(),
            if edge == ClockEdge::Rising {
                "CLKA"
            } else {
                "INV"
            }
            .into(),
        ),
        (
            "CLKBMUX".into(),
            if edge == ClockEdge::Rising {
                "CLKB"
            } else {
                "INV"
            }
            .into(),
        ),
        (
            "WEAMUX".into(),
            match write_enable.active {
                ActiveLevel::High => "WEA",
                ActiveLevel::Low => "INV",
            }
            .into(),
        ),
    ]);
    parameters.insert("CEAMUX".into(), "1".into());
    parameters.insert(
        "CEBMUX".into(),
        match read_enable.map(|enable| enable.active) {
            None => "1",
            Some(ActiveLevel::High) => "CEB",
            Some(ActiveLevel::Low) => "INV",
        }
        .into(),
    );
    parameters
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;
    use std::num::NonZeroU32;

    use struo_ir::{
        ActiveLevel, ArithmeticOp, ClockEdge, ComparisonOp, EnableControl, MemoryCell, Netlist,
        RegisterCell, ResetControl,
    };

    use super::{
        ArithmeticMapping, Bit, Ecp5Cell, MappingOptions, backward_retime_lut, map_once,
        map_to_ecp5, map_to_ecp5_with_options,
    };

    fn arithmetic_netlist(width: u32, operation: ArithmeticOp) -> Netlist {
        let mut source = Netlist::new("arithmetic");
        let width = NonZeroU32::new(width).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let result = source.add_arithmetic(operation, &lhs, &rhs).unwrap();
        source.add_output_port("result", &result).unwrap();
        source
    }

    #[test]
    fn automatically_selects_retiming_only_when_lut_timing_improves() {
        let mut source = Netlist::new("retimed_lut_chain");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let inputs = (0..8)
            .map(|index| source.add_input(format!("input{index}")))
            .collect::<Vec<_>>();
        let reset_control = ResetControl {
            signal: reset,
            active: ActiveLevel::High,
            asynchronous: true,
            value: false,
        };
        let registered = inputs
            .iter()
            .enumerate()
            .map(|(index, input)| {
                let name = format!("input_q{index}");
                let output = source.add_register_output(&name);
                source.add_register(RegisterCell::new(
                    name,
                    output,
                    *input,
                    clock,
                    ClockEdge::Rising,
                    None,
                    Some(reset_control),
                ));
                output
            })
            .collect::<Vec<_>>();
        let reduced = registered[1..]
            .iter()
            .fold(registered[0], |value, input| source.add_and(value, *input));
        let output = source.add_register_output("result_q");
        source.add_register(RegisterCell::new(
            "result_q",
            output,
            reduced,
            clock,
            ClockEdge::Rising,
            None,
            Some(reset_control),
        ));
        source.add_output("result", output);

        let mapped = map_to_ecp5(&source).unwrap();

        assert!(mapped.retiming().applied, "{:?}", mapped.retiming());
        assert!(
            mapped.retiming().selected_lut_depth < mapped.retiming().original_lut_depth,
            "{:?}",
            mapped.retiming()
        );
    }

    #[test]
    fn backward_lut_retiming_derives_nonzero_input_reset() {
        let mut source = Netlist::new("retimed_not");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let input = source.add_input("input");
        let inverted = source.add_not(input);
        let output = source.add_register_output("result_q");
        source.add_register(RegisterCell::new(
            "result_q",
            output,
            inverted,
            clock,
            ClockEdge::Rising,
            None,
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::High,
                asynchronous: true,
                value: false,
            }),
        ));
        source.add_output("result", output);
        let (mut mapped, _) = map_once(&source, MappingOptions::default()).unwrap();
        mapped.cells.push(Ecp5Cell::Lut4 {
            name: "retime_dead_lut".into(),
            inputs: [Bit::Zero; 4],
            output: 10_000,
            init: 0,
        });
        let register = mapped
            .cells
            .iter()
            .position(
                |cell| matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name == "ff_result_q"),
            )
            .unwrap();

        let retimed = backward_retime_lut(&mapped, register).unwrap();

        assert!(retimed.cells.iter().any(|cell| {
            matches!(
                cell,
                Ecp5Cell::FlipFlop {
                    name,
                    reset: Some(super::Reset { value: true, .. }),
                    ..
                } if name.starts_with("retime_ff_result_q_")
            )
        }));
        assert!(!retimed.cells.iter().any(|cell| {
            matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name == "ff_result_q")
        }));
        assert!(
            !retimed.cells.iter().any(
                |cell| matches!(cell, Ecp5Cell::Lut4 { name, .. } if name == "retime_dead_lut")
            )
        );
        let outputs = retimed
            .cells
            .iter()
            .flat_map(super::cell_output_bits)
            .filter_map(|bit| match bit {
                Bit::Wire(wire) => Some(wire),
                Bit::Zero | Bit::One => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(
            outputs.len(),
            outputs.iter().copied().collect::<HashSet<_>>().len(),
            "retiming must not reuse the removed maximum Q wire"
        );
    }

    #[test]
    fn maps_wide_arithmetic_to_ccu2c() {
        let mapped = map_to_ecp5(&arithmetic_netlist(8, ArithmeticOp::Add)).unwrap();
        let carries = mapped
            .cells()
            .iter()
            .filter(|cell| matches!(cell, Ecp5Cell::Ccu2c { .. }))
            .count();
        assert_eq!(carries, 4);
        assert!(mapped.cells().iter().all(|cell| {
            matches!(
                cell,
                Ecp5Cell::Ccu2c {
                    init: [0x96aa, 0x96aa],
                    inject: [false, false],
                    ..
                }
            )
        }));

        let json: serde_json::Value =
            serde_json::from_str(&mapped.to_nextpnr_json().unwrap()).unwrap();
        let cells = json["modules"]["arithmetic"]["cells"].as_object().unwrap();
        assert!(cells.values().all(|cell| cell["type"] == "CCU2C"));
        assert!(cells.values().all(|cell| {
            cell["parameters"]["INIT0"] == "1001011010101010"
                && cell["parameters"]["INJECT1_0"] == "NO"
        }));
    }

    #[test]
    fn maps_mixed_retained_cells_in_dependency_order() {
        let mut source = Netlist::new("mixed_words");
        let width = NonZeroU32::new(5).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let sum = source
            .add_arithmetic(ArithmeticOp::Add, &lhs, &rhs)
            .unwrap();
        let less = source
            .add_comparison(ComparisonOp::LessThanUnsigned, &sum, &rhs)
            .unwrap();
        let incremented = source
            .add_arithmetic(ArithmeticOp::Add, &[less], &[lhs[0]])
            .unwrap();
        source.add_output("result", incremented[0]);

        let mapped = map_to_ecp5(&source).unwrap();

        assert!(
            mapped
                .cells()
                .iter()
                .any(|cell| matches!(cell, Ecp5Cell::Ccu2c { .. }))
        );
    }

    #[test]
    fn lut_arithmetic_option_provides_comparison_baseline() {
        let mapped = map_to_ecp5_with_options(
            &arithmetic_netlist(8, ArithmeticOp::Subtract),
            MappingOptions {
                arithmetic: ArithmeticMapping::Lut4,
                ..MappingOptions::default()
            },
        )
        .unwrap();
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
                .count(),
            15
        );
        assert!(
            !mapped
                .cells()
                .iter()
                .any(|cell| matches!(cell, Ecp5Cell::Ccu2c { .. }))
        );
    }

    #[test]
    fn maps_boolean_nodes_to_expected_lut_truth_tables() {
        let mut source = Netlist::new("logic");
        let select = source.add_input("select");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let and = source.add_and(lhs, rhs);
        let or = source.add_or(lhs, rhs);
        let xor = source.add_xor(lhs, rhs);
        let not = source.add_not(lhs);
        let mux = source.add_mux(select, lhs, rhs);
        source.add_output("and", and);
        source.add_output("or", or);
        source.add_output("xor", xor);
        source.add_output("not", not);
        source.add_output("mux", mux);
        let mapped = map_to_ecp5(&source).unwrap();

        let truth_tables = mapped
            .cells()
            .iter()
            .filter_map(|cell| match cell {
                Ecp5Cell::Lut4 { init, .. } => Some(*init),
                Ecp5Cell::Ccu2c { .. } | Ecp5Cell::FlipFlop { .. } | Ecp5Cell::BlockRam { .. } => {
                    None
                }
            })
            .collect::<Vec<_>>();
        assert_eq!(truth_tables, [0x8888, 0xeeee, 0x6666, 0x5555, 0xd8d8]);
    }

    #[test]
    fn collapses_a_four_input_cone_into_one_lut() {
        let mut source = Netlist::new("four_input");
        let a = source.add_input("a");
        let b = source.add_input("b");
        let c = source.add_input("c");
        let d = source.add_input("d");
        let ab = source.add_and(a, b);
        let cd = source.add_and(c, d);
        let result = source.add_or(ab, cd);
        source.add_output("y", result);

        let mapped = map_to_ecp5(&source).unwrap();
        let luts = mapped
            .cells()
            .iter()
            .filter_map(|cell| match cell {
                Ecp5Cell::Lut4 { inputs, init, .. } => Some((*inputs, *init)),
                Ecp5Cell::Ccu2c { .. } | Ecp5Cell::FlipFlop { .. } | Ecp5Cell::BlockRam { .. } => {
                    None
                }
            })
            .collect::<Vec<_>>();

        assert_eq!(
            luts,
            [(
                [Bit::Wire(2), Bit::Wire(3), Bit::Wire(4), Bit::Wire(5)],
                0xf888
            )]
        );
    }

    #[test]
    fn maps_a_five_input_cone_to_two_luts() {
        let mut source = Netlist::new("five_input");
        let inputs = source.add_input_port("a", NonZeroU32::new(5).unwrap());
        let lower = source.add_and(inputs[0], inputs[1]);
        let upper = source.add_and(inputs[2], inputs[3]);
        let four_input = source.add_or(lower, upper);
        let result = source.add_xor(four_input, inputs[4]);
        source.add_output("y", result);

        let mapped = map_to_ecp5(&source).unwrap();
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
                .count(),
            2
        );
    }

    #[test]
    fn recovers_area_by_reusing_an_already_mapped_cone() {
        let mut source = Netlist::new("shared_cone");
        let input_a = source.add_input("a");
        let input_b = source.add_input("b");
        let input_c = source.add_input("c");
        let input_d = source.add_input("d");
        let input_x = source.add_input("x");
        let shared = source.add_and(input_a, input_b);
        let other = source.add_and(input_c, input_d);
        let selected = source.add_and(shared, input_x);
        let result = source.add_or(selected, other);
        source.add_output("shared", shared);
        source.add_output("result", result);

        let mapped = map_to_ecp5(&source).unwrap();

        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
                .count(),
            2
        );
    }

    #[test]
    fn omits_unreachable_boolean_nodes() {
        let mut source = Netlist::new("dead_logic");
        let a = source.add_input("a");
        let b = source.add_input("b");
        let live = source.add_and(a, b);
        let _dead = source.add_or(a, b);
        source.add_output("y", live);

        let mapped = map_to_ecp5(&source).unwrap();
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
                .count(),
            1
        );
    }

    #[test]
    fn maps_async_low_reset_to_trellis_ff_parameters() {
        let mut source = Netlist::new("counter_bit");
        let clock = source.add_input("clk");
        let reset = source.add_input("rst_n");
        let output = source.add_register_output("state");
        let next = source.add_not(output);
        source.add_register(RegisterCell::new(
            "state",
            output,
            next,
            clock,
            ClockEdge::Rising,
            None,
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::Low,
                asynchronous: true,
                value: false,
            }),
        ));
        source.add_output("q", output);

        let mapped = map_to_ecp5(&source).unwrap();
        let json = mapped.to_nextpnr_json().unwrap();
        assert!(json.contains("\"type\": \"TRELLIS_FF\""));
        assert!(json.contains("\"SRMODE\": \"ASYNC\""));
        assert!(json.contains("\"LSRMUX\": \"INV\""));

        let json: serde_json::Value = serde_json::from_str(&json).unwrap();
        let connections = &json["modules"]["counter_bit"]["cells"]["ff_state"]["connections"];
        assert!(connections.get("DI").is_some());
        assert!(connections.get("M").is_none());
    }

    #[test]
    fn preserves_vector_ports_for_nextpnr() {
        let mut source = Netlist::new("passthrough");
        let input = source.add_input_port("input", NonZeroU32::new(3).unwrap());
        source.add_output_port("output", &input).unwrap();

        let mapped = map_to_ecp5(&source).unwrap();
        let json: serde_json::Value =
            serde_json::from_str(&mapped.to_nextpnr_json().unwrap()).unwrap();

        assert_eq!(mapped.ports().len(), 2);
        assert_eq!(mapped.ports()[0].bits.len(), 3);
        assert_eq!(
            json["modules"]["passthrough"]["ports"]["input"]["bits"]
                .as_array()
                .unwrap()
                .len(),
            3
        );
        assert_eq!(
            json["modules"]["passthrough"]["ports"]["output"]["bits"]
                .as_array()
                .unwrap()
                .len(),
            3
        );
    }

    #[test]
    fn maps_synchronous_memory_to_dp16kd() {
        let mut source = Netlist::new("scratchpad");
        let clock = source.add_input("clock");
        let write_enable = source.add_input("write_enable");
        let read_address = source.add_input_port("read_address", NonZeroU32::new(8).unwrap());
        let write_address = source.add_input_port("write_address", NonZeroU32::new(8).unwrap());
        let write_data = source.add_input_port("write_data", NonZeroU32::new(8).unwrap());
        let read_data = (0..8)
            .map(|bit| source.add_memory_output(format!("words[{bit}]")))
            .collect::<Vec<_>>();
        source.add_memory(MemoryCell::new(
            "words",
            256,
            read_address,
            read_data.clone(),
            None,
            write_address,
            write_data,
            EnableControl {
                signal: write_enable,
                active: ActiveLevel::High,
            },
            clock,
            ClockEdge::Rising,
        ));
        source.add_output_port("read_data", &read_data).unwrap();

        let mapped = map_to_ecp5(&source).unwrap();
        assert!(mapped.cells().iter().any(|cell| matches!(
            cell,
            Ecp5Cell::BlockRam {
                physical_width: 9,
                depth: 256,
                word_width: 8,
                ..
            }
        )));

        let json: serde_json::Value =
            serde_json::from_str(&mapped.to_nextpnr_json().unwrap()).unwrap();
        let cell = &json["modules"]["scratchpad"]["cells"]["bram_words"];
        assert_eq!(cell["type"], "DP16KD");
        assert_eq!(cell["parameters"]["DATA_WIDTH_A"], "9");
        assert_eq!(cell["connections"]["ADA3"][0], 12);
        assert_eq!(cell["connections"]["DOB7"][0], 35);
    }
}
