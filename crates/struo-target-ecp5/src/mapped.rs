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
const MAX_ENABLE_FANOUT_PER_REPLICA: usize = 16;

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
    equivalence_proof: MappedEquivalenceProof,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct MappedEquivalenceProof {
    certified_primitive_moves: usize,
    equivalent_register_merges: usize,
    equivalent_logic_replications: usize,
    unobservable_cells_removed: usize,
    valid: bool,
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
    /// Estimated worst data, control, or output period before retiming.
    pub original_overall_period_ps: u32,
    /// Estimated worst data, control, or output period after retiming.
    pub selected_overall_period_ps: u32,
    /// Mapped flip-flops before candidate selection.
    pub original_registers: usize,
    /// Mapped flip-flops after candidate selection.
    pub selected_registers: usize,
    /// Primitive retiming certificates composed into the selected result.
    pub certified_primitive_moves: usize,
    /// Structurally equivalent generated registers merged in the selected result.
    pub equivalent_register_merges: usize,
    /// Combinational cells replicated without changing their truth tables.
    pub equivalent_logic_replications: usize,
    /// Unobservable cells removed after certified retiming moves.
    pub unobservable_cells_removed: usize,
    /// Whether the complete selected transformation chain passed sign-off.
    pub equivalence_signed_off: bool,
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
    )
    .filter(|retimed| verify_mapped_equivalence_proof(retimed, true))
    {
        selected = retimed;
        selected_registers = mapped_register_count(&selected);
        applied = true;
    }
    let mapped_selected_profile = mapped_lut_profile(&selected);
    let equivalence_signed_off = verify_mapped_equivalence_proof(&selected, applied);
    selected.retiming = RetimingSelection {
        applied,
        original_lut_depth: mapped_original_profile.data_depth,
        selected_lut_depth: mapped_selected_profile.data_depth,
        original_critical_registers: mapped_original_profile.critical_depth.len(),
        selected_critical_registers: mapped_selected_profile.critical_depth.len(),
        original_period_ps: mapped_original_profile.data_period_ps,
        selected_period_ps: mapped_selected_profile.data_period_ps,
        original_overall_period_ps: mapped_original_profile.overall_period_ps,
        selected_overall_period_ps: mapped_selected_profile.overall_period_ps,
        original_registers,
        selected_registers,
        certified_primitive_moves: selected.equivalence_proof.certified_primitive_moves,
        equivalent_register_merges: selected.equivalence_proof.equivalent_register_merges,
        equivalent_logic_replications: selected.equivalence_proof.equivalent_logic_replications,
        unobservable_cells_removed: selected.equivalence_proof.unobservable_cells_removed,
        equivalence_signed_off,
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
    let timing_driven = original_profile.overall_period_ps > target_period_ps;
    let cell_limit = original_cells + original_cells.div_ceil(10);
    let register_limit = original_registers + original_registers.div_ceil(5);
    let control_candidate =
        replicate_high_fanout_enable_luts(original, MAX_ENABLE_FANOUT_PER_REPLICA);
    let control_profile = mapped_lut_profile(&control_candidate);
    let control_registers = mapped_register_count(&control_candidate);
    let original_enable_fanout =
        maximum_replicable_enable_fanout(original, MAX_ENABLE_FANOUT_PER_REPLICA);
    let control_enable_fanout =
        maximum_replicable_enable_fanout(&control_candidate, MAX_ENABLE_FANOUT_PER_REPLICA);
    let use_control = control_candidate.cells.len() <= cell_limit
        && control_registers <= register_limit
        && control_profile.overall_period_ps <= original_profile.overall_period_ps
        && (retiming_score(
            &control_profile,
            timing_driven,
            control_candidate.cells.len(),
            control_registers,
        ) < retiming_score(
            &original_profile,
            timing_driven,
            original.cells.len(),
            mapped_register_count(original),
        ) || control_enable_fanout < original_enable_fanout);
    let seed = if use_control {
        &control_candidate
    } else {
        original
    };
    let forward_candidate = forward_retime_registered_ccu_chains(seed, timing_driven);
    let forward_profile = mapped_lut_profile(&forward_candidate);
    let forward_registers = mapped_register_count(&forward_candidate);
    let use_forward = forward_candidate.cells.len() <= cell_limit
        && forward_registers <= register_limit
        && forward_profile.overall_period_ps <= original_profile.overall_period_ps
        && retiming_score(
            &forward_profile,
            timing_driven,
            forward_candidate.cells.len(),
            forward_registers,
        ) < retiming_score(
            &original_profile,
            timing_driven,
            original.cells.len(),
            mapped_register_count(original),
        );
    let mut frontier = if use_forward {
        forward_candidate
    } else {
        seed.clone()
    };
    let mut best_seen = frontier.clone();
    let mut bridge_budget = 2usize;
    for _ in 0..128 {
        let profile = mapped_lut_profile(&frontier);
        let best_profile = mapped_lut_profile(&best_seen);
        if retiming_score(
            &profile,
            timing_driven,
            frontier.cells.len(),
            mapped_register_count(&frontier),
        ) < retiming_score(
            &best_profile,
            timing_driven,
            best_seen.cells.len(),
            mapped_register_count(&best_seen),
        ) {
            best_seen = frontier.clone();
        }
        if (timing_driven && profile.overall_period_ps <= target_period_ps)
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
            let Some(mut candidate) = backward_retime_primitive(&frontier, register) else {
                continue;
            };
            merge_equivalent_flip_flops(&mut candidate);
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
        // profitable. Stable names survive index shifts caused by pruning
        // unobservable generated cells after every certified move.
        let batch_registers = critical
            .iter()
            .map(|index| mapped_cell_name(&frontier.cells[*index]).to_owned())
            .collect::<Vec<_>>();
        let mut batch = frontier.clone();
        let mut batch_moves = 0usize;
        for name in batch_registers {
            let Some(register) = batch
                .cells
                .iter()
                .position(|cell| mapped_cell_name(cell) == name)
            else {
                continue;
            };
            let Some(candidate) = backward_retime_primitive(&batch, register) else {
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
        let mut bridge_candidate = None;
        if timing_driven && bridge_budget > 0 {
            let ccu_period = profile
                .timing_endpoints
                .iter()
                .filter_map(|(index, period)| {
                    (*period > target_period_ps
                        && register_data_is_driven_by_ccu(&frontier, *index))
                    .then_some(*period)
                })
                .max();
            if let Some(ccu_period) = ccu_period {
                let endpoint_names = profile
                    .timing_endpoints
                    .iter()
                    .filter(|(index, period)| {
                        *period == ccu_period && register_data_is_driven_by_ccu(&frontier, *index)
                    })
                    .map(|(index, _)| mapped_cell_name(&frontier.cells[*index]).to_owned())
                    .collect::<Vec<_>>();
                let mut ccu_batch = frontier.clone();
                let mut ccu_moves = 0usize;
                for name in &endpoint_names {
                    let Some(index) = ccu_batch
                        .cells
                        .iter()
                        .position(|cell| mapped_cell_name(cell) == name)
                    else {
                        continue;
                    };
                    let Some(candidate) = backward_retime_ccu2c(&ccu_batch, index) else {
                        continue;
                    };
                    let candidate_profile = mapped_lut_profile(&candidate);
                    let candidate_registers = mapped_register_count(&candidate);
                    if candidate_profile.overall_period_ps <= original_profile.overall_period_ps
                        && candidate.cells.len() <= cell_limit
                        && candidate_registers <= register_limit
                    {
                        ccu_batch = candidate;
                        ccu_moves += 1;
                    }
                }
                if ccu_moves == endpoint_names.len() && ccu_moves > 0 {
                    merge_equivalent_flip_flops(&mut ccu_batch);
                    let batch_profile = mapped_lut_profile(&ccu_batch);
                    let batch_cells = ccu_batch.cells.len();
                    let batch_registers = mapped_register_count(&ccu_batch);
                    let score =
                        retiming_score(&batch_profile, timing_driven, batch_cells, batch_registers);
                    if bridge_candidate
                        .as_ref()
                        .is_none_or(|(_, bridge_score)| score < *bridge_score)
                    {
                        bridge_candidate = Some((ccu_batch, score));
                    }
                }
            }
        }
        if batch_moves > 0 {
            merge_equivalent_flip_flops(&mut batch);
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
            } else if bridge_candidate.is_none()
                && timing_driven
                && profile.data_period_ps > target_period_ps
                && batch_profile.data_period_ps
                    <= profile
                        .data_period_ps
                        .saturating_add(2 * MAPPED_ROUTE_GUARD_PS)
                && batch_moves == critical.len()
                && bridge_budget > 0
            {
                bridge_candidate = Some((batch, batch_score));
            }
        }
        let selected_bridge = best.is_none() && bridge_candidate.is_some();
        let Some((candidate, _)) = best.or(bridge_candidate) else {
            break;
        };
        if selected_bridge {
            bridge_budget -= 1;
        }
        frontier = candidate;
    }
    let frontier_profile = mapped_lut_profile(&frontier);
    let best_profile = mapped_lut_profile(&best_seen);
    if retiming_score(
        &frontier_profile,
        timing_driven,
        frontier.cells.len(),
        mapped_register_count(&frontier),
    ) < retiming_score(
        &best_profile,
        timing_driven,
        best_seen.cells.len(),
        mapped_register_count(&best_seen),
    ) {
        best_seen = frontier;
    }
    let replicated = replicate_high_fanout_enable_luts(&best_seen, MAX_ENABLE_FANOUT_PER_REPLICA);
    let best_profile = mapped_lut_profile(&best_seen);
    let replicated_profile = mapped_lut_profile(&replicated);
    if replicated.cells.len() <= cell_limit
        && mapped_register_count(&replicated) <= register_limit
        && replicated_profile.overall_period_ps <= best_profile.overall_period_ps
        && maximum_replicable_enable_fanout(&replicated, MAX_ENABLE_FANOUT_PER_REPLICA)
            < maximum_replicable_enable_fanout(&best_seen, MAX_ENABLE_FANOUT_PER_REPLICA)
    {
        best_seen = replicated;
    }
    let selected_profile = mapped_lut_profile(&best_seen);
    let improved = if timing_driven {
        (
            selected_profile.overall_period_ps,
            selected_profile.data_period_ps,
            selected_profile.critical_timing.len(),
            maximum_replicable_enable_fanout(&best_seen, MAX_ENABLE_FANOUT_PER_REPLICA),
        ) < (
            original_profile.overall_period_ps,
            original_profile.data_period_ps,
            original_profile.critical_timing.len(),
            maximum_replicable_enable_fanout(original, MAX_ENABLE_FANOUT_PER_REPLICA),
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
    improved.then_some(best_seen)
}

fn retiming_score(
    profile: &MappedLutProfile,
    timing_driven: bool,
    cells: usize,
    registers: usize,
) -> (u64, u64, u64, usize, usize, usize) {
    if timing_driven {
        (
            u64::from(profile.overall_period_ps),
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
            u64::from(profile.overall_period_ps),
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

fn verify_mapped_equivalence_proof(netlist: &Ecp5Netlist, applied: bool) -> bool {
    if !netlist.equivalence_proof.valid
        || (applied
            && netlist.equivalence_proof.certified_primitive_moves == 0
            && netlist.equivalence_proof.equivalent_logic_replications == 0)
    {
        return false;
    }
    let primary_inputs = netlist
        .ports
        .iter()
        .filter(|port| port.direction == PortDirection::Input)
        .flat_map(|port| &port.bits)
        .filter_map(|bit| match bit {
            Bit::Wire(wire) => Some(*wire),
            Bit::Zero | Bit::One => None,
        })
        .collect::<HashSet<_>>();
    let mut driven = primary_inputs.clone();
    for bit in netlist.cells.iter().flat_map(cell_output_bits) {
        let Bit::Wire(wire) = bit else {
            continue;
        };
        if !driven.insert(wire) {
            return false;
        }
    }
    netlist
        .cells
        .iter()
        .flat_map(cell_input_bits)
        .chain(
            netlist
                .ports
                .iter()
                .filter(|port| port.direction == PortDirection::Output)
                .flat_map(|port| &port.bits)
                .copied(),
        )
        .all(|bit| match bit {
            Bit::Wire(wire) => driven.contains(&wire),
            Bit::Zero | Bit::One => true,
        })
}

fn replicate_high_fanout_enable_luts(netlist: &Ecp5Netlist, max_fanout: usize) -> Ecp5Netlist {
    let max_fanout = max_fanout.max(1);
    let fanouts = mapped_wire_fanouts(netlist);
    let mut enable_sinks = BTreeMap::<u32, Vec<usize>>::new();
    for (index, cell) in netlist.cells.iter().enumerate() {
        let Ecp5Cell::FlipFlop {
            enable: Some(enable),
            ..
        } = cell
        else {
            continue;
        };
        let Bit::Wire(wire) = enable.signal else {
            continue;
        };
        let wire_fanout = fanouts.get(&wire).copied().unwrap_or(0);
        if wire_fanout > max_fanout && wire_fanout <= max_fanout.saturating_mul(2) {
            enable_sinks.entry(wire).or_default().push(index);
        }
    }

    let mut candidate = netlist.clone();
    let mut next_wire = maximum_mapped_wire(netlist)
        .and_then(|wire| wire.checked_add(1))
        .unwrap_or(1);
    for (wire, sinks) in enable_sinks {
        if sinks.len() <= max_fanout {
            continue;
        }
        let Some((driver_name, inputs, init)) = netlist.cells.iter().find_map(|cell| match cell {
            Ecp5Cell::Lut4 {
                name,
                inputs,
                output,
                init,
            } if *output == wire => Some((name.clone(), *inputs, *init)),
            _ => None,
        }) else {
            continue;
        };
        let non_enable_fanout = fanouts
            .get(&wire)
            .copied()
            .unwrap_or(0)
            .saturating_sub(sinks.len());
        let retained = max_fanout
            .saturating_sub(non_enable_fanout)
            .min(sinks.len());
        for (replica, chunk) in sinks[retained..].chunks(max_fanout).enumerate() {
            let output = next_wire;
            let Some(allocated) = next_wire.checked_add(1) else {
                return netlist.clone();
            };
            next_wire = allocated;
            for &sink in chunk {
                let Ecp5Cell::FlipFlop {
                    enable: Some(enable),
                    ..
                } = &mut candidate.cells[sink]
                else {
                    return netlist.clone();
                };
                if enable.signal != Bit::Wire(wire) {
                    return netlist.clone();
                }
                enable.signal = Bit::Wire(output);
            }
            candidate.cells.push(Ecp5Cell::Lut4 {
                name: format!("replicate_enable_{driver_name}_{replica}"),
                inputs,
                output,
                init,
            });
            candidate.equivalence_proof.equivalent_logic_replications += 1;
        }
    }
    candidate
}

fn maximum_replicable_enable_fanout(netlist: &Ecp5Netlist, max_fanout: usize) -> usize {
    let fanouts = mapped_wire_fanouts(netlist);
    let lut_outputs = netlist
        .cells
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::Lut4 { output, .. } => Some(*output),
            Ecp5Cell::Ccu2c { .. } | Ecp5Cell::FlipFlop { .. } | Ecp5Cell::BlockRam { .. } => None,
        })
        .collect::<HashSet<_>>();
    netlist
        .cells
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::FlipFlop {
                enable: Some(enable),
                ..
            } => match enable.signal {
                Bit::Wire(wire) if lut_outputs.contains(&wire) => fanouts
                    .get(&wire)
                    .copied()
                    .filter(|fanout| *fanout <= max_fanout.saturating_mul(2)),
                Bit::Wire(_) | Bit::Zero | Bit::One => None,
            },
            Ecp5Cell::Lut4 { .. }
            | Ecp5Cell::Ccu2c { .. }
            | Ecp5Cell::FlipFlop { .. }
            | Ecp5Cell::BlockRam { .. } => None,
        })
        .max()
        .unwrap_or(0)
}

fn register_data_is_driven_by_ccu(netlist: &Ecp5Netlist, register_index: usize) -> bool {
    let Some(Ecp5Cell::FlipFlop { data, .. }) = netlist.cells.get(register_index) else {
        return false;
    };
    netlist
        .cells
        .iter()
        .any(|cell| matches!(cell, Ecp5Cell::Ccu2c { .. }) && cell_output_bits(cell).contains(data))
}

struct MappedLutProfile {
    data_depth: usize,
    critical_depth: Vec<usize>,
    overall_depth: usize,
    data_period_ps: u32,
    critical_timing: Vec<usize>,
    timing_endpoints: Vec<(usize, u32)>,
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
        .iter()
        .filter_map(|(index, period)| (*period == data_period_ps && *period > 0).then_some(*index))
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
        timing_endpoints,
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

fn backward_retime_primitive(netlist: &Ecp5Netlist, register_index: usize) -> Option<Ecp5Netlist> {
    backward_retime_lut(netlist, register_index)
        .or_else(|| backward_retime_ccu2c(netlist, register_index))
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
    candidate.equivalence_proof.certified_primitive_moves += 1;
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CcuRetimeOutput {
    Sum0,
    Sum1,
    Carry,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CcuInputLocation {
    Pin { slice: usize, pin: usize },
    CarryIn,
}

#[derive(Clone, Copy)]
enum CcuCarrySource {
    External(Bit, CcuInputLocation),
    Internal(usize),
}

#[derive(Clone, Copy)]
enum CcuLogicValue {
    Constant(bool),
    Input(usize),
}

struct CcuBoundary {
    vertex: usize,
    bit: Bit,
    location: CcuInputLocation,
}

fn forward_retime_registered_ccu_chains(netlist: &Ecp5Netlist, timing_driven: bool) -> Ecp5Netlist {
    let mut selected = netlist.clone();
    for _ in 0..netlist.cells.len() {
        let profile = mapped_lut_profile(&selected);
        let selected_score = retiming_score(
            &profile,
            timing_driven,
            selected.cells.len(),
            mapped_register_count(&selected),
        );
        let chains = ccu_chain_names(&selected);
        let mut best = None;
        for chain in chains {
            let mut candidate = selected.clone();
            let mut moved = 0usize;
            for name in &chain {
                let Some(index) = candidate
                    .cells
                    .iter()
                    .position(|cell| mapped_cell_name(cell) == name)
                else {
                    break;
                };
                let Some(retimed) = forward_retime_ccu2c(&candidate, index) else {
                    break;
                };
                candidate = retimed;
                moved += 1;
            }
            if moved != chain.len() {
                continue;
            }
            let candidate_profile = mapped_lut_profile(&candidate);
            let score = retiming_score(
                &candidate_profile,
                timing_driven,
                candidate.cells.len(),
                mapped_register_count(&candidate),
            );
            if score < selected_score
                && best
                    .as_ref()
                    .is_none_or(|(_, best_score): &(Ecp5Netlist, _)| score < *best_score)
            {
                best = Some((candidate, score));
            }
        }
        let Some((candidate, _)) = best else {
            break;
        };
        selected = candidate;
    }
    selected
}

fn ccu_chain_names(netlist: &Ecp5Netlist) -> Vec<Vec<String>> {
    let ccus = netlist
        .cells
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::Ccu2c {
                name,
                carry_in,
                carry_out,
                ..
            } => Some((name.clone(), *carry_in, *carry_out)),
            Ecp5Cell::Lut4 { .. } | Ecp5Cell::FlipFlop { .. } | Ecp5Cell::BlockRam { .. } => None,
        })
        .collect::<Vec<_>>();
    let carry_producers = ccus
        .iter()
        .map(|(name, _, carry_out)| (*carry_out, name.as_str()))
        .collect::<HashMap<_, _>>();
    let successors = ccus
        .iter()
        .filter_map(|(name, carry_in, _)| match carry_in {
            Bit::Wire(wire) => carry_producers
                .get(wire)
                .map(|producer| ((*producer).to_owned(), name.clone())),
            Bit::Zero | Bit::One => None,
        })
        .collect::<HashMap<_, _>>();
    let roots = ccus
        .iter()
        .filter_map(|(name, carry_in, _)| {
            let has_producer = match carry_in {
                Bit::Wire(wire) => carry_producers.contains_key(wire),
                Bit::Zero | Bit::One => false,
            };
            (!has_producer).then_some(name.clone())
        })
        .collect::<Vec<_>>();
    let mut chains = Vec::new();
    let mut visited = HashSet::new();
    for root in roots {
        let mut chain = Vec::new();
        let mut cursor = Some(root);
        while let Some(name) = cursor {
            if !visited.insert(name.clone()) {
                break;
            }
            cursor = successors.get(&name).cloned();
            chain.push(name);
        }
        if !chain.is_empty() {
            chains.push(chain);
        }
    }
    for (name, _, _) in ccus {
        if visited.insert(name.clone()) {
            chains.push(vec![name]);
        }
    }
    chains
}

#[allow(clippy::too_many_lines)]
fn forward_retime_ccu2c(netlist: &Ecp5Netlist, ccu_index: usize) -> Option<Ecp5Netlist> {
    let Ecp5Cell::Ccu2c {
        name: ccu_name,
        inputs,
        carry_in,
        sums,
        carry_out,
        init,
        inject,
    } = netlist.cells.get(ccu_index)?
    else {
        return None;
    };
    let ccu_name = ccu_name.clone();
    let (inputs, carry_in, sums, carry_out, init, inject) =
        (*inputs, *carry_in, *sums, *carry_out, *init, *inject);

    let mut physical_uses = inputs
        .iter()
        .flatten()
        .copied()
        .chain([carry_in])
        .filter_map(|bit| match bit {
            Bit::Wire(wire) => Some(wire),
            Bit::Zero | Bit::One => None,
        })
        .fold(HashMap::<u32, usize>::new(), |mut uses, wire| {
            *uses.entry(wire).or_insert(0) += 1;
            uses
        });
    if inject[0]
        && let Bit::Wire(wire) = carry_in
        && let Some(uses) = physical_uses.get_mut(&wire)
    {
        *uses -= 1;
        if *uses == 0 {
            physical_uses.remove(&wire);
        }
    }
    if physical_uses.is_empty() {
        return None;
    }

    let mut input_data = HashMap::new();
    let mut input_resets = HashMap::new();
    let mut input_registers = HashSet::new();
    let mut domain: Option<(Bit, ClockEdge, Option<Control>, Reset)> = None;
    for (&wire, &local_uses) in &physical_uses {
        if mapped_wire_fanout(netlist, wire) != local_uses {
            return None;
        }
        let (register_index, data, clock, edge, enable, reset) =
            netlist
                .cells
                .iter()
                .enumerate()
                .find_map(|(index, cell)| match cell {
                    Ecp5Cell::FlipFlop {
                        data,
                        output,
                        clock,
                        edge,
                        enable,
                        reset: Some(reset),
                        ..
                    } if *output == wire => Some((index, *data, *clock, *edge, *enable, *reset)),
                    Ecp5Cell::Lut4 { .. }
                    | Ecp5Cell::Ccu2c { .. }
                    | Ecp5Cell::FlipFlop { .. }
                    | Ecp5Cell::BlockRam { .. } => None,
                })?;
        if let Some((domain_clock, domain_edge, domain_enable, domain_reset)) = domain {
            if (clock, edge, enable) != (domain_clock, domain_edge, domain_enable)
                || (reset.signal, reset.active, reset.asynchronous)
                    != (
                        domain_reset.signal,
                        domain_reset.active,
                        domain_reset.asynchronous,
                    )
            {
                return None;
            }
        } else {
            domain = Some((clock, edge, enable, reset));
        }
        input_data.insert(wire, data);
        input_resets.insert(wire, reset.value);
        input_registers.insert(register_index);
    }
    let (clock, clock_edge, enable, reset) = domain?;

    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let mut boundaries = Vec::new();
    let mut logic_vertices = Vec::new();
    let sum0_vertex = append_ccu_logic_vertex(
        &mut vertices,
        &mut edges,
        &mut boundaries,
        inputs[0],
        CcuCarrySource::External(carry_in, CcuInputLocation::CarryIn),
        init[0],
        inject[0],
        false,
        0,
        &mut logic_vertices,
        Some(&input_resets),
    );
    let carry0_vertex = append_ccu_logic_vertex(
        &mut vertices,
        &mut edges,
        &mut boundaries,
        inputs[0],
        CcuCarrySource::External(carry_in, CcuInputLocation::CarryIn),
        init[0],
        inject[0],
        true,
        0,
        &mut logic_vertices,
        Some(&input_resets),
    );
    let slice1_carry = if inject[1] {
        CcuCarrySource::External(Bit::Zero, CcuInputLocation::CarryIn)
    } else {
        CcuCarrySource::Internal(carry0_vertex)
    };
    let sum1_vertex = append_ccu_logic_vertex(
        &mut vertices,
        &mut edges,
        &mut boundaries,
        inputs[1],
        slice1_carry,
        init[1],
        inject[1],
        false,
        1,
        &mut logic_vertices,
        Some(&input_resets),
    );
    let carry1_vertex = append_ccu_logic_vertex(
        &mut vertices,
        &mut edges,
        &mut boundaries,
        inputs[1],
        slice1_carry,
        init[1],
        inject[1],
        true,
        1,
        &mut logic_vertices,
        Some(&input_resets),
    );
    let mut output_boundaries = Vec::new();
    for (name, logic) in [
        ("sum0", sum0_vertex),
        ("sum1", sum1_vertex),
        ("carry", carry1_vertex),
    ] {
        let boundary = vertices.len();
        vertices.push(RetimingVertex::boundary(name, LogicFunction::new(1, 0b10)));
        edges.push(RetimingEdge::new(logic, boundary, Vec::new()));
        output_boundaries.push((logic, boundary));
    }
    let before = RetimingGraph::new(
        RetimingDomain::new(
            format!("{clock:?}"),
            clock_edge,
            format!("{:?}", reset.signal),
            reset.active,
            reset.asynchronous,
        ),
        vertices,
        edges,
    );
    let mut labels = vec![0; before.vertices().len()];
    for vertex in logic_vertices {
        labels[vertex] = -1;
    }
    let certificate = RetimingCertificate::new(labels);
    let after = derive_retimed_graph(&before, &certificate).ok()?;
    verify_retiming_certificate(&before, &after, &certificate).ok()?;
    let output_resets = output_boundaries
        .iter()
        .map(|&(logic, boundary)| {
            after
                .edges()
                .iter()
                .find(|edge| edge.source() == logic && edge.target() == boundary)?
                .reset_values()
                .first()
                .copied()
        })
        .collect::<Option<Vec<_>>>()?;

    let mut candidate = netlist.clone();
    candidate.equivalence_proof.certified_primitive_moves += 1;
    let mut removed = input_registers.into_iter().collect::<Vec<_>>();
    removed.sort_unstable_by(|lhs, rhs| rhs.cmp(lhs));
    for index in removed {
        candidate.cells.remove(index);
    }
    let retimed_ccu_index = candidate
        .cells
        .iter()
        .position(|cell| mapped_cell_name(cell) == ccu_name)?;
    let rewrite = |bit: Bit| match bit {
        Bit::Wire(wire) => input_data.get(&wire).copied().unwrap_or(bit),
        Bit::Zero | Bit::One => bit,
    };
    let retimed_inputs = inputs.map(|slice| slice.map(rewrite));
    let retimed_carry_in = if inject[0] {
        Bit::Zero
    } else {
        rewrite(carry_in)
    };
    let mut next_wire = maximum_mapped_wire(netlist)?.checked_add(1)?;
    let internal_sums = [next_wire, next_wire.checked_add(1)?];
    next_wire = next_wire.checked_add(2)?;
    let internal_carry = next_wire;
    candidate.cells[retimed_ccu_index] = Ecp5Cell::Ccu2c {
        name: format!("retime_forward_{ccu_name}"),
        inputs: retimed_inputs,
        carry_in: retimed_carry_in,
        sums: internal_sums,
        carry_out: internal_carry,
        init,
        inject,
    };
    for (name, data, output, reset_value) in [
        ("sum0", internal_sums[0], sums[0], output_resets[0]),
        ("sum1", internal_sums[1], sums[1], output_resets[1]),
        ("carry", internal_carry, carry_out, output_resets[2]),
    ] {
        if mapped_wire_fanout(netlist, output) == 0 {
            continue;
        }
        candidate.cells.push(Ecp5Cell::FlipFlop {
            name: format!("retime_forward_{ccu_name}_{name}"),
            data: Bit::Wire(data),
            output,
            clock,
            edge: clock_edge,
            enable,
            reset: Some(Reset {
                value: reset_value,
                ..reset
            }),
        });
    }
    Some(candidate)
}

#[allow(clippy::too_many_lines)]
fn backward_retime_ccu2c(netlist: &Ecp5Netlist, register_index: usize) -> Option<Ecp5Netlist> {
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
    let (ccu_index, ccu_name, inputs, carry_in, sums, carry_out, init, inject, output) = netlist
        .cells
        .iter()
        .enumerate()
        .find_map(|(index, cell)| match cell {
            Ecp5Cell::Ccu2c {
                name,
                inputs,
                carry_in,
                sums,
                carry_out,
                init,
                inject,
            } => {
                let output = if sums[0] == *data_wire {
                    Some(CcuRetimeOutput::Sum0)
                } else if sums[1] == *data_wire {
                    Some(CcuRetimeOutput::Sum1)
                } else if *carry_out == *data_wire {
                    Some(CcuRetimeOutput::Carry)
                } else {
                    None
                }?;
                Some((
                    index,
                    name.clone(),
                    *inputs,
                    *carry_in,
                    *sums,
                    *carry_out,
                    *init,
                    *inject,
                    output,
                ))
            }
            Ecp5Cell::Lut4 { .. } | Ecp5Cell::FlipFlop { .. } | Ecp5Cell::BlockRam { .. } => None,
        })?;

    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let mut boundaries = Vec::new();
    let mut logic_vertices = Vec::new();
    let desired_vertex = match output {
        CcuRetimeOutput::Sum0 => append_ccu_logic_vertex(
            &mut vertices,
            &mut edges,
            &mut boundaries,
            inputs[0],
            CcuCarrySource::External(carry_in, CcuInputLocation::CarryIn),
            init[0],
            inject[0],
            false,
            0,
            &mut logic_vertices,
            None,
        ),
        CcuRetimeOutput::Sum1 | CcuRetimeOutput::Carry => {
            let slice1_carry = if inject[1] {
                CcuCarrySource::External(Bit::Zero, CcuInputLocation::CarryIn)
            } else {
                let carry_vertex = append_ccu_logic_vertex(
                    &mut vertices,
                    &mut edges,
                    &mut boundaries,
                    inputs[0],
                    CcuCarrySource::External(carry_in, CcuInputLocation::CarryIn),
                    init[0],
                    inject[0],
                    true,
                    0,
                    &mut logic_vertices,
                    None,
                );
                CcuCarrySource::Internal(carry_vertex)
            };
            append_ccu_logic_vertex(
                &mut vertices,
                &mut edges,
                &mut boundaries,
                inputs[1],
                slice1_carry,
                init[1],
                inject[1],
                output == CcuRetimeOutput::Carry,
                1,
                &mut logic_vertices,
                None,
            )
        }
    };
    let q_vertex = vertices.len();
    vertices.push(RetimingVertex::boundary("q", LogicFunction::new(1, 0b10)));
    edges.push(RetimingEdge::new(
        desired_vertex,
        q_vertex,
        vec![reset.value],
    ));
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
    let mut labels = vec![0; before.vertices().len()];
    for vertex in logic_vertices {
        labels[vertex] = 1;
    }
    let certificate = RetimingCertificate::new(labels);
    let after = derive_retimed_graph(&before, &certificate).ok()?;
    verify_retiming_certificate(&before, &after, &certificate).ok()?;

    let mut next_wire = maximum_mapped_wire(netlist)?.checked_add(1)?;
    let mut candidate = netlist.clone();
    candidate.equivalence_proof.certified_primitive_moves += 1;
    candidate.cells.remove(register_index);
    let uses_slice1 = output != CcuRetimeOutput::Sum0;
    let uses_slice0 = !uses_slice1 || !inject[1];
    let mut retimed_inputs = [[Bit::Zero; 4]; 2];
    if uses_slice0 {
        retimed_inputs[0] = inputs[0];
    }
    if uses_slice1 {
        retimed_inputs[1] = inputs[1];
    }
    let mut retimed_carry_in = if uses_slice0 && !inject[0] {
        carry_in
    } else {
        Bit::Zero
    };
    for boundary in boundaries {
        let Bit::Wire(input_wire) = boundary.bit else {
            continue;
        };
        let reset_value = after
            .edges()
            .iter()
            .find(|edge| edge.source() == boundary.vertex)?
            .reset_values()
            .first()
            .copied()?;
        let output_wire = next_wire;
        next_wire = next_wire.checked_add(1)?;
        let location_name = match boundary.location {
            CcuInputLocation::Pin { slice, pin } => format!("s{slice}p{pin}"),
            CcuInputLocation::CarryIn => "cin".into(),
        };
        candidate.cells.push(Ecp5Cell::FlipFlop {
            name: format!("retime_{register_name}_{ccu_name}_{location_name}"),
            data: Bit::Wire(input_wire),
            output: output_wire,
            clock: *clock,
            edge: *clock_edge,
            enable: *enable,
            reset: Some(Reset {
                value: reset_value,
                ..*reset
            }),
        });
        match boundary.location {
            CcuInputLocation::Pin { slice, pin } => {
                retimed_inputs[slice][pin] = Bit::Wire(output_wire);
            }
            CcuInputLocation::CarryIn => retimed_carry_in = Bit::Wire(output_wire),
        }
    }
    let mut retimed_sums = [0; 2];
    for (slice, sum) in retimed_sums.iter_mut().enumerate() {
        *sum = if output == [CcuRetimeOutput::Sum0, CcuRetimeOutput::Sum1][slice] {
            *register_output
        } else {
            let wire = next_wire;
            next_wire = next_wire.checked_add(1)?;
            wire
        };
    }
    let retimed_carry_out = if output == CcuRetimeOutput::Carry {
        *register_output
    } else {
        next_wire
    };
    let retimed_ccu = Ecp5Cell::Ccu2c {
        name: format!("retime_{ccu_name}"),
        inputs: retimed_inputs,
        carry_in: retimed_carry_in,
        sums: retimed_sums,
        carry_out: retimed_carry_out,
        init,
        inject,
    };
    let original_outputs = [sums[0], sums[1], carry_out];
    let replace_original = original_outputs
        .iter()
        .all(|wire| mapped_wire_fanout(netlist, *wire) == usize::from(*wire == *data_wire));
    if replace_original {
        let ccu_index = ccu_index - usize::from(register_index < ccu_index);
        candidate.cells[ccu_index] = retimed_ccu;
    } else {
        candidate.cells.push(retimed_ccu);
    }
    prune_unobservable_retiming_cells(&mut candidate);
    Some(candidate)
}

#[allow(clippy::too_many_arguments)]
fn append_ccu_logic_vertex(
    vertices: &mut Vec<RetimingVertex>,
    edges: &mut Vec<RetimingEdge>,
    boundaries: &mut Vec<CcuBoundary>,
    inputs: [Bit; 4],
    carry: CcuCarrySource,
    init: u16,
    inject: bool,
    carry_output: bool,
    slice: usize,
    logic_vertices: &mut Vec<usize>,
    input_resets: Option<&HashMap<u32, bool>>,
) -> usize {
    let mut sources = Vec::new();
    let mut logic_inputs = [CcuLogicValue::Constant(false); 4];
    for (pin, input) in inputs.into_iter().enumerate() {
        logic_inputs[pin] = append_ccu_external_input(
            vertices,
            boundaries,
            &mut sources,
            input,
            CcuInputLocation::Pin { slice, pin },
            input_resets,
        );
    }
    let carry_input = if inject {
        CcuLogicValue::Constant(false)
    } else {
        match carry {
            CcuCarrySource::External(bit, location) => append_ccu_external_input(
                vertices,
                boundaries,
                &mut sources,
                bit,
                location,
                input_resets,
            ),
            CcuCarrySource::Internal(vertex) => {
                let input = CcuLogicValue::Input(sources.len());
                sources.push((vertex, Vec::new()));
                input
            }
        }
    };
    let function = ccu_logic_function(logic_inputs, carry_input, init, carry_output, sources.len());
    let vertex = vertices.len();
    vertices.push(RetimingVertex::logic(
        if carry_output { "carry" } else { "sum" },
        function,
    ));
    logic_vertices.push(vertex);
    for (source, reset_values) in sources {
        edges.push(RetimingEdge::new(source, vertex, reset_values));
    }
    vertex
}

fn append_ccu_external_input(
    vertices: &mut Vec<RetimingVertex>,
    boundaries: &mut Vec<CcuBoundary>,
    sources: &mut Vec<(usize, Vec<bool>)>,
    bit: Bit,
    location: CcuInputLocation,
    input_resets: Option<&HashMap<u32, bool>>,
) -> CcuLogicValue {
    match bit {
        Bit::Zero => CcuLogicValue::Constant(false),
        Bit::One => CcuLogicValue::Constant(true),
        Bit::Wire(wire) => {
            let vertex = vertices.len();
            vertices.push(RetimingVertex::boundary(
                format!("wire{wire}"),
                LogicFunction::new(0, 0),
            ));
            boundaries.push(CcuBoundary {
                vertex,
                bit,
                location,
            });
            let input = CcuLogicValue::Input(sources.len());
            let reset_values = input_resets
                .and_then(|resets| resets.get(&wire))
                .copied()
                .map_or_else(Vec::new, |value| vec![value]);
            sources.push((vertex, reset_values));
            input
        }
    }
}

fn ccu_logic_function(
    inputs: [CcuLogicValue; 4],
    carry: CcuLogicValue,
    init: u16,
    carry_output: bool,
    input_count: usize,
) -> LogicFunction {
    let mut truth_table = 0u64;
    for assignment in 0..(1usize << input_count) {
        let values = inputs.map(|input| ccu_logic_value(input, assignment));
        let carry = ccu_logic_value(carry, assignment);
        let lut4_index = values
            .iter()
            .enumerate()
            .fold(0usize, |index, (pin, value)| {
                index | (usize::from(*value) << pin)
            });
        let lut2_index = usize::from(values[0]) | (usize::from(values[1]) << 1);
        let lut4 = init & (1u16 << lut4_index) != 0;
        let lut2 = init & (1u16 << lut2_index) != 0;
        let value = if carry_output {
            if lut4 { lut2 } else { carry }
        } else {
            lut4 ^ carry
        };
        truth_table |= u64::from(value) << assignment;
    }
    LogicFunction::new(
        u8::try_from(input_count).expect("CCU slice has at most five inputs"),
        truth_table,
    )
}

fn ccu_logic_value(value: CcuLogicValue, assignment: usize) -> bool {
    match value {
        CcuLogicValue::Constant(value) => value,
        CcuLogicValue::Input(input) => assignment & (1usize << input) != 0,
    }
}

fn merge_equivalent_flip_flops(netlist: &mut Ecp5Netlist) {
    const MAX_SHARED_FANOUT: usize = 2;
    loop {
        let mut canonical = Vec::<(
            Bit,
            u32,
            Bit,
            ClockEdge,
            Option<Control>,
            Option<Reset>,
            usize,
        )>::new();
        let mut duplicates = Vec::new();
        for (index, cell) in netlist.cells.iter().enumerate() {
            let Ecp5Cell::FlipFlop {
                name,
                data,
                output,
                clock,
                edge,
                enable,
                reset,
                ..
            } = cell
            else {
                continue;
            };
            if !name.starts_with("retime_") {
                continue;
            }
            let output_fanout = mapped_wire_fanout(netlist, *output);
            if let Some((_, canonical_output, .., canonical_fanout)) = canonical.iter_mut().find(
                |(
                    candidate_data,
                    _,
                    candidate_clock,
                    candidate_edge,
                    candidate_enable,
                    candidate_reset,
                    candidate_fanout,
                )| {
                    (
                        *candidate_data,
                        *candidate_clock,
                        *candidate_edge,
                        *candidate_enable,
                        *candidate_reset,
                    ) == (*data, *clock, *edge, *enable, *reset)
                        && *candidate_fanout + output_fanout <= MAX_SHARED_FANOUT
                },
            ) {
                duplicates.push((index, *output, *canonical_output));
                *canonical_fanout += output_fanout;
            } else {
                canonical.push((
                    *data,
                    *output,
                    *clock,
                    *edge,
                    *enable,
                    *reset,
                    output_fanout,
                ));
            }
        }
        if duplicates.is_empty() {
            break;
        }
        netlist.equivalence_proof.equivalent_register_merges += duplicates.len();
        for &(_, duplicate, canonical) in &duplicates {
            replace_mapped_wire_uses(netlist, duplicate, canonical);
        }
        duplicates.sort_unstable_by_key(|duplicate| std::cmp::Reverse(duplicate.0));
        for (index, _, _) in duplicates {
            netlist.cells.remove(index);
        }
    }
}

fn replace_mapped_wire_uses(netlist: &mut Ecp5Netlist, from: u32, to: u32) {
    let replace = |bit: &mut Bit| {
        if *bit == Bit::Wire(from) {
            *bit = Bit::Wire(to);
        }
    };
    for port in &mut netlist.ports {
        for bit in &mut port.bits {
            replace(bit);
        }
    }
    for cell in &mut netlist.cells {
        match cell {
            Ecp5Cell::Lut4 { inputs, .. } => {
                for bit in inputs {
                    replace(bit);
                }
            }
            Ecp5Cell::Ccu2c {
                inputs, carry_in, ..
            } => {
                for bit in inputs.iter_mut().flatten() {
                    replace(bit);
                }
                replace(carry_in);
            }
            Ecp5Cell::FlipFlop {
                data,
                clock,
                enable,
                reset,
                ..
            } => {
                replace(data);
                replace(clock);
                if let Some(control) = enable {
                    replace(&mut control.signal);
                }
                if let Some(control) = reset {
                    replace(&mut control.signal);
                }
            }
            Ecp5Cell::BlockRam {
                write_address,
                write_data,
                write_enable,
                read_address,
                read_enable,
                clock,
                ..
            } => {
                for bit in write_address
                    .iter_mut()
                    .chain(write_data)
                    .chain(read_address.iter_mut())
                {
                    replace(bit);
                }
                replace(&mut write_enable.signal);
                if let Some(control) = read_enable {
                    replace(&mut control.signal);
                }
                replace(clock);
            }
        }
    }
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
        netlist.equivalence_proof.unobservable_cells_removed += previous_len - netlist.cells.len();
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
                original_overall_period_ps: quality.period_ps,
                selected_overall_period_ps: quality.period_ps,
                original_registers: netlist.registers().len(),
                selected_registers: netlist.registers().len(),
                certified_primitive_moves: 0,
                equivalent_register_merges: 0,
                equivalent_logic_replications: 0,
                unobservable_cells_removed: 0,
                equivalence_signed_off: true,
            },
            equivalence_proof: MappedEquivalenceProof {
                valid: true,
                ..MappedEquivalenceProof::default()
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
        ArithmeticMapping, Bit, Ecp5Cell, MappingOptions, backward_retime_ccu2c,
        backward_retime_lut, ccu_chain_names, forward_retime_ccu2c, map_once, map_to_ecp5,
        map_to_ecp5_with_options, mapped_wire_fanout, merge_equivalent_flip_flops,
        replicate_high_fanout_enable_luts, verify_mapped_equivalence_proof,
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
        assert!(mapped.retiming().equivalence_signed_off);
        assert!(mapped.retiming().certified_primitive_moves > 0);
        assert!(
            mapped.retiming().selected_lut_depth < mapped.retiming().original_lut_depth,
            "{:?}",
            mapped.retiming()
        );
    }

    #[test]
    fn mapped_equivalence_signoff_rejects_an_invalid_final_netlist() {
        let source = arithmetic_netlist(8, ArithmeticOp::Add);
        let (mut mapped, _) = map_once(&source, MappingOptions::default()).unwrap();
        assert!(verify_mapped_equivalence_proof(&mapped, false));

        let duplicate = mapped.cells[0].clone();
        mapped.cells.push(duplicate);

        assert!(!verify_mapped_equivalence_proof(&mapped, false));
    }

    #[test]
    fn replicates_high_fanout_enable_logic_once() {
        let mut source = Netlist::new("replicated_enable");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let enable_lhs = source.add_input("enable_lhs");
        let enable_rhs = source.add_input("enable_rhs");
        let enable = source.add_and(enable_lhs, enable_rhs);
        let data = source.add_input("data");
        for index in 0..10 {
            let name = format!("value{index}");
            let output = source.add_register_output(&name);
            source.add_register(RegisterCell::new(
                name.clone(),
                output,
                data,
                clock,
                ClockEdge::Rising,
                Some(EnableControl {
                    signal: enable,
                    active: ActiveLevel::High,
                }),
                Some(ResetControl {
                    signal: reset,
                    active: ActiveLevel::High,
                    asynchronous: true,
                    value: false,
                }),
            ));
            source.add_output(format!("output{index}"), output);
        }
        let (mapped, _) = map_once(&source, MappingOptions::default()).unwrap();

        let replicated = replicate_high_fanout_enable_luts(&mapped, 5);

        assert_eq!(
            replicated.equivalence_proof.equivalent_logic_replications,
            1
        );
        assert!(verify_mapped_equivalence_proof(&replicated, true));
        assert!(replicated.cells.iter().all(|cell| {
            let Ecp5Cell::FlipFlop {
                enable: Some(enable),
                ..
            } = cell
            else {
                return true;
            };
            let Bit::Wire(wire) = enable.signal else {
                return true;
            };
            mapped_wire_fanout(&replicated, wire) <= 5
        }));
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
    fn backward_ccu_retiming_splits_a_carry_chain_with_a_certificate() {
        let mut source = Netlist::new("retimed_carry");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let lhs = source.add_input_port("lhs", NonZeroU32::new(8).unwrap());
        let rhs = source.add_input_port("rhs", NonZeroU32::new(8).unwrap());
        let sum = source
            .add_arithmetic(ArithmeticOp::Add, &lhs, &rhs)
            .unwrap();
        let output = source.add_register_output("result_q");
        source.add_register(RegisterCell::new(
            "result_q",
            output,
            sum[7],
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
        let (mapped, _) = map_once(&source, MappingOptions::default()).unwrap();
        let register = mapped
            .cells
            .iter()
            .position(
                |cell| matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name == "ff_result_q"),
            )
            .unwrap();
        let Ecp5Cell::FlipFlop {
            output: register_output,
            ..
        } = mapped.cells[register]
        else {
            unreachable!()
        };

        let retimed = backward_retime_ccu2c(&mapped, register).unwrap();

        assert!(!retimed.cells.iter().any(|cell| {
            matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name == "ff_result_q")
        }));
        assert!(retimed.cells.iter().any(|cell| {
            matches!(
                cell,
                Ecp5Cell::Ccu2c { name, sums, .. }
                    if name.starts_with("retime_ccu_") && sums[1] == register_output
            )
        }));
        assert!(
            retimed
                .cells
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name.starts_with("retime_ff_result_q_ccu_")))
                .count()
                >= 5
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
            outputs.iter().copied().collect::<HashSet<_>>().len()
        );
    }

    #[test]
    fn forward_ccu_retiming_moves_operand_registers_across_the_whole_chain() {
        let mut source = Netlist::new("forward_retimed_carry");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let lhs = source.add_input_port("lhs", NonZeroU32::new(8).unwrap());
        let rhs = source.add_input_port("rhs", NonZeroU32::new(8).unwrap());
        let reset_control = ResetControl {
            signal: reset,
            active: ActiveLevel::High,
            asynchronous: true,
            value: false,
        };
        let registered = lhs
            .iter()
            .chain(&rhs)
            .enumerate()
            .map(|(index, input)| {
                let name = format!("operand_q{index}");
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
        let sum = source
            .add_arithmetic(ArithmeticOp::Add, &registered[..8], &registered[8..])
            .unwrap();
        source.add_output_port("sum", &sum).unwrap();
        let (mapped, _) = map_once(&source, MappingOptions::default()).unwrap();

        let mut retimed = mapped.clone();
        for name in &ccu_chain_names(&mapped)[0] {
            let index = retimed
                .cells
                .iter()
                .position(|cell| super::mapped_cell_name(cell) == name)
                .unwrap();
            retimed = forward_retime_ccu2c(&retimed, index).unwrap();
        }

        assert_eq!(
            retimed
                .cells
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::FlipFlop { .. }))
                .count(),
            8
        );
        assert!(retimed.cells.iter().all(|cell| {
            !matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name.starts_with("ff_operand_q"))
        }));
        assert_eq!(
            retimed
                .cells
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::Ccu2c { name, .. } if name.starts_with("retime_forward_ccu_")))
                .count(),
            4
        );
    }

    #[test]
    fn equivalent_retiming_registers_are_shared_without_high_fanout() {
        let mut source = Netlist::new("shared_retiming_registers");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let data = source.add_input("data");
        for index in 0..3 {
            let name = format!("copy{index}");
            let output = source.add_register_output(&name);
            source.add_register(RegisterCell::new(
                name.clone(),
                output,
                data,
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
            source.add_output(format!("output{index}"), output);
        }
        let (mut mapped, _) = map_once(&source, MappingOptions::default()).unwrap();
        for cell in &mut mapped.cells {
            if let Ecp5Cell::FlipFlop { name, .. } = cell {
                *name = format!("retime_{name}");
            }
        }

        merge_equivalent_flip_flops(&mut mapped);

        let outputs = mapped
            .cells
            .iter()
            .filter_map(|cell| match cell {
                Ecp5Cell::FlipFlop { output, .. } => Some(*output),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(outputs.len(), 2);
        assert!(
            outputs
                .iter()
                .all(|output| mapped_wire_fanout(&mapped, *output) <= 2)
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
