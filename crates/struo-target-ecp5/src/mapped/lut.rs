//! ECP5 LUT4/wide-LUT cut analysis, cover selection, and emission.

use std::collections::BTreeSet;

use struo_ir::{ArithmeticCell, NetId, Netlist, NodeKind};

use super::{
    ArithmeticMapping, Bit, Ecp5Cell, IoTimingConstraints, L6_MUX_DELAY_PS, MappingOptions,
    PFU_MUX_DELAY_PS, node_for, wire_for, wire_number,
};

const LUT_INPUTS: usize = 4;
pub(super) const WIDE_LUT_INPUTS: usize = 7;
const CUT_LIMIT: usize = 64;
const MAX_AREA_RECOVERY_PASSES: usize = 8;
const CRITICALITY_NUMERATOR: u32 = 15;
const CRITICALITY_DENOMINATOR: u32 = 32;

// Conservative pre-route delay estimates for an ECP5 speed-grade 8. These
// are deliberately architecture-level costs rather than sign-off numbers;
// nextpnr remains the source of truth after placement and routing.
pub(super) const LUT_DELAY_PS: u32 = 100;
const ROUTE_BASE_PS: u32 = 300;
const ROUTE_FANOUT_STEP_PS: u32 = 40;
pub(super) const CCU_INPUT_PS: u32 = 200;
pub(super) const CCU_CARRY_PS: u32 = 60;
pub(super) const CCU_SUM_PS: u32 = 100;
pub(super) const BRAM_CLOCK_TO_OUTPUT_PS: u32 = 850;
pub(super) const FLIP_FLOP_CLOCK_TO_OUTPUT_PS: u32 = 300;
pub(super) const FLIP_FLOP_SETUP_PS: u32 = 120;
const BRAM_SETUP_PS: u32 = 300;

#[derive(Clone, Debug)]
struct Cut {
    leaves: Vec<NetId>,
}

/// Immutable feasible-cut analysis for one Boolean netlist.
pub(super) struct CutDatabase {
    cuts: Vec<Vec<Cut>>,
}

impl CutDatabase {
    pub(super) fn analyze(netlist: &Netlist, max_inputs: usize) -> Self {
        debug_assert!((LUT_INPUTS..=WIDE_LUT_INPUTS).contains(&max_inputs));
        let cuts = netlist
            .nodes()
            .iter()
            .map(|node| {
                if is_boolean(node.kind()) {
                    enumerate_cuts(netlist, node.output(), max_inputs)
                        .into_iter()
                        .map(|leaves| Cut { leaves })
                        .collect()
                } else {
                    Vec::new()
                }
            })
            .collect();
        Self { cuts }
    }

    fn for_net(&self, net: NetId) -> &[Cut] {
        &self.cuts[net.index() as usize]
    }
}

#[derive(Clone, Debug)]
struct LutPlan {
    leaves: Vec<NetId>,
    depth: usize,
    arrival_ps: Option<u32>,
    area: usize,
    root_area: usize,
}

/// Required-time-aware cover selected from a feasible-cut database.
pub(super) struct LutCover {
    plans: Vec<Option<LutPlan>>,
    arrivals: Vec<Option<u32>>,
    fanouts: Vec<usize>,
}

impl LutCover {
    pub(super) fn select(
        netlist: &Netlist,
        cuts: &CutDatabase,
        mapping_roots: &[NetId],
        options: MappingOptions,
        period_ps: u32,
        io_timing: &IoTimingConstraints,
    ) -> Self {
        let fanouts = structural_fanouts(netlist);
        let retained = RetainedTiming::new(netlist);
        let critical_arrival_ps =
            period_ps.saturating_mul(CRITICALITY_NUMERATOR) / CRITICALITY_DENOMINATOR;
        let (mut plans, mut arrivals) = select_initial_plans(
            netlist,
            cuts,
            &fanouts,
            &retained,
            options,
            critical_arrival_ps,
            io_timing,
        );
        let strict_required = required_times(
            netlist, &plans, &arrivals, &fanouts, &retained, options, period_ps, io_timing, false,
        );
        if !timing_is_valid(&arrivals, &strict_required) {
            (plans, arrivals) =
                select_initial_plans(netlist, cuts, &fanouts, &retained, options, 0, io_timing);
        }

        let original_plans = plans.clone();
        let original_arrivals = arrivals.clone();
        let strict_required = required_times(
            netlist, &plans, &arrivals, &fanouts, &retained, options, period_ps, io_timing, false,
        );
        let mut required = if timing_is_valid(&arrivals, &strict_required) {
            strict_required
        } else {
            required_times(
                netlist, &plans, &arrivals, &fanouts, &retained, options, period_ps, io_timing,
                true,
            )
        };
        let original_required = required.clone();
        RecoveryContext {
            netlist,
            cuts,
            roots: mapping_roots,
            fanouts: &fanouts,
            retained: &retained,
            options,
        }
        .recover_area(&mut plans, &mut arrivals, &mut required);
        if !timing_is_valid(&arrivals, &required) {
            // Area recovery is heuristic. Never let an unexpectedly coupled
            // timing update turn a release build into a slower cover.
            plans = original_plans;
            arrivals = original_arrivals;
            required = original_required;
        }
        debug_assert!(timing_is_valid(&arrivals, &required));
        Self {
            plans,
            arrivals,
            fanouts,
        }
    }

    fn plan(&self, net: NetId) -> Option<&LutPlan> {
        self.plans[net.index() as usize].as_ref()
    }

    pub(super) fn estimated_register_period_ps(&self, netlist: &Netlist) -> (u32, Vec<NetId>) {
        let endpoints = netlist
            .registers()
            .iter()
            .flat_map(|register| {
                [register.data()]
                    .into_iter()
                    .chain(register.enable().map(|enable| enable.signal))
            })
            .filter_map(|net| {
                let index = net.index() as usize;
                self.arrivals[index].map(|arrival| {
                    (
                        net,
                        arrival
                            .saturating_add(wire_delay_ps(self.fanouts[index]))
                            .saturating_add(FLIP_FLOP_SETUP_PS),
                    )
                })
            })
            .collect::<Vec<_>>();
        let maximum = endpoints
            .iter()
            .map(|(_, period)| *period)
            .max()
            .unwrap_or(0);
        let mut critical = endpoints
            .into_iter()
            .filter_map(|(net, period)| (period == maximum).then_some(net))
            .collect::<Vec<_>>();
        critical.sort_unstable();
        critical.dedup();
        (maximum, critical)
    }
}

fn select_initial_plans(
    netlist: &Netlist,
    cuts: &CutDatabase,
    fanouts: &[usize],
    retained: &RetainedTiming,
    options: MappingOptions,
    critical_arrival_ps: u32,
    io_timing: &IoTimingConstraints,
) -> (Vec<Option<LutPlan>>, Vec<Option<u32>>) {
    let mut plans = vec![None; netlist.nodes().len()];
    let mut arrivals = vec![None; netlist.nodes().len()];
    for (name, delay_ps) in &io_timing.input_delays_ps {
        let port = netlist
            .ports()
            .iter()
            .find(|port| port.name() == name)
            .expect("I/O timing constraints were validated");
        for bit in port.bits() {
            arrivals[bit.index() as usize] = Some(*delay_ps);
        }
    }
    for node in netlist.nodes() {
        let index = node.output().index() as usize;
        match node.kind() {
            NodeKind::And | NodeKind::Or | NodeKind::Xor | NodeKind::Not | NodeKind::Mux => {
                let plan = best_plan(
                    &plans,
                    &arrivals,
                    fanouts,
                    cuts.for_net(node.output()),
                    critical_arrival_ps,
                );
                arrivals[index] = plan.arrival_ps;
                plans[index] = Some(plan);
            }
            NodeKind::MemoryOutput(_) => arrivals[index] = Some(BRAM_CLOCK_TO_OUTPUT_PS),
            NodeKind::ArithmeticOutput(_) | NodeKind::ComparisonOutput(_) => {
                arrivals[index] = retained.output_arrival(
                    netlist,
                    node.output(),
                    &arrivals,
                    fanouts,
                    options.arithmetic,
                );
            }
            NodeKind::Output(_) => arrivals[index] = arrivals[node.inputs()[0].index() as usize],
            NodeKind::RegisterOutput(_) => {
                arrivals[index] = Some(FLIP_FLOP_CLOCK_TO_OUTPUT_PS);
            }
            NodeKind::Input(_) | NodeKind::Constant(_) => {}
        }
    }
    (plans, arrivals)
}

#[derive(Clone, Copy, Debug)]
enum RetainedOutput {
    Arithmetic { cell: usize, bit: usize },
    Comparison { cell: usize },
}

struct RetainedTiming {
    outputs: Vec<Option<RetainedOutput>>,
}

impl RetainedTiming {
    fn new(netlist: &Netlist) -> Self {
        let mut outputs = vec![None; netlist.nodes().len()];
        for (cell, arithmetic) in netlist.arithmetic().iter().enumerate() {
            for (bit, output) in arithmetic.outputs().iter().enumerate() {
                outputs[output.index() as usize] = Some(RetainedOutput::Arithmetic { cell, bit });
            }
        }
        for (cell, comparison) in netlist.comparisons().iter().enumerate() {
            outputs[comparison.output().index() as usize] =
                Some(RetainedOutput::Comparison { cell });
        }
        Self { outputs }
    }

    fn output_arrival(
        &self,
        netlist: &Netlist,
        output: NetId,
        arrivals: &[Option<u32>],
        fanouts: &[usize],
        arithmetic_mapping: ArithmeticMapping,
    ) -> Option<u32> {
        match self.outputs[output.index() as usize]
            .expect("a retained output belongs to a retained cell")
        {
            RetainedOutput::Arithmetic { cell, bit } => arithmetic_arrival(
                &netlist.arithmetic()[cell],
                bit,
                arrivals,
                fanouts,
                arithmetic_mapping,
            ),
            RetainedOutput::Comparison { cell } => {
                let comparison = &netlist.comparisons()[cell];
                comparison
                    .lhs()
                    .iter()
                    .zip(comparison.rhs())
                    .enumerate()
                    .flat_map(|(bit, (lhs, rhs))| {
                        let arc = comparison_arc_ps(
                            comparison.lhs().len(),
                            bit,
                            comparison.operation().is_signed(),
                        );
                        [*lhs, *rhs].into_iter().filter_map(move |input| {
                            arrivals[input.index() as usize].map(|arrival| {
                                arrival + wire_delay_ps(fanouts[input.index() as usize]) + arc
                            })
                        })
                    })
                    .max()
            }
        }
    }

    fn propagate_required(
        &self,
        netlist: &Netlist,
        output: NetId,
        required_output: u32,
        fanouts: &[usize],
        arithmetic_mapping: ArithmeticMapping,
        required: &mut [u32],
    ) {
        match self.outputs[output.index() as usize]
            .expect("a retained output belongs to a retained cell")
        {
            RetainedOutput::Arithmetic { cell, bit } => {
                let arithmetic = &netlist.arithmetic()[cell];
                for input_bit in 0..=bit {
                    let arc = arithmetic_arc_ps(arithmetic, bit, input_bit, arithmetic_mapping);
                    for input in [arithmetic.lhs()[input_bit], arithmetic.rhs()[input_bit]] {
                        tighten_required(input, required_output, arc, fanouts, required);
                    }
                }
                if let Some(carry_in) = arithmetic.carry_in() {
                    let arc = arithmetic_carry_arc_ps(arithmetic, bit, arithmetic_mapping);
                    tighten_required(carry_in, required_output, arc, fanouts, required);
                }
            }
            RetainedOutput::Comparison { cell } => {
                let comparison = &netlist.comparisons()[cell];
                for (bit, (lhs, rhs)) in comparison.lhs().iter().zip(comparison.rhs()).enumerate() {
                    let arc = comparison_arc_ps(
                        comparison.lhs().len(),
                        bit,
                        comparison.operation().is_signed(),
                    );
                    for input in [*lhs, *rhs] {
                        tighten_required(input, required_output, arc, fanouts, required);
                    }
                }
            }
        }
    }
}

fn structural_fanouts(netlist: &Netlist) -> Vec<usize> {
    let mut fanouts = vec![0usize; netlist.nodes().len()];
    let mut add = |net: NetId| fanouts[net.index() as usize] += 1;
    for node in netlist.nodes() {
        for input in node.inputs() {
            add(*input);
        }
    }
    for register in netlist.registers() {
        for input in [register.data(), register.clock()]
            .into_iter()
            .chain(register.enable().map(|enable| enable.signal))
            .chain(register.reset().map(|reset| reset.signal))
        {
            add(input);
        }
    }
    for memory in netlist.memories() {
        for input in memory
            .read_address()
            .iter()
            .chain(memory.write_address())
            .chain(memory.write_data())
            .copied()
            .chain([memory.clock(), memory.write_enable().signal])
            .chain(memory.read_enable().map(|enable| enable.signal))
            .chain(memory.second_port().into_iter().flat_map(|port| {
                port.read_address()
                    .iter()
                    .chain(port.write_address())
                    .chain(port.write_data())
                    .copied()
                    .chain([port.clock(), port.write_enable().signal])
                    .chain(port.read_enable().map(|enable| enable.signal))
            }))
        {
            add(input);
        }
    }
    for arithmetic in netlist.arithmetic() {
        for input in arithmetic
            .lhs()
            .iter()
            .chain(arithmetic.rhs())
            .copied()
            .chain(arithmetic.carry_in())
        {
            add(input);
        }
    }
    for comparison in netlist.comparisons() {
        for input in comparison.lhs().iter().chain(comparison.rhs()) {
            add(*input);
        }
    }
    fanouts
}

pub(super) fn wire_delay_ps(fanout: usize) -> u32 {
    let fanout = fanout.max(1);
    let levels = usize::BITS - (fanout - 1).leading_zeros();
    ROUTE_BASE_PS + ROUTE_FANOUT_STEP_PS * levels
}

fn arithmetic_uses_carry(arithmetic: &ArithmeticCell, mapping: ArithmeticMapping) -> bool {
    match mapping {
        ArithmeticMapping::Auto => arithmetic.outputs().len() > 4,
        ArithmeticMapping::CarryChain => true,
        ArithmeticMapping::Lut4 => false,
    }
}

fn arithmetic_arc_ps(
    arithmetic: &ArithmeticCell,
    output_bit: usize,
    input_bit: usize,
    mapping: ArithmeticMapping,
) -> u32 {
    debug_assert!(input_bit <= output_bit);
    if arithmetic_uses_carry(arithmetic, mapping) {
        CCU_INPUT_PS + CCU_CARRY_PS * u32::try_from(output_bit - input_bit).unwrap() + CCU_SUM_PS
    } else {
        LUT_DELAY_PS * u32::try_from(output_bit - input_bit + 1).unwrap()
    }
}

fn arithmetic_carry_arc_ps(
    arithmetic: &ArithmeticCell,
    output_bit: usize,
    mapping: ArithmeticMapping,
) -> u32 {
    if arithmetic_uses_carry(arithmetic, mapping) {
        CCU_CARRY_PS * u32::try_from(output_bit).unwrap() + CCU_SUM_PS
    } else {
        LUT_DELAY_PS * u32::try_from(output_bit + 1).unwrap()
    }
}

fn arithmetic_arrival(
    arithmetic: &ArithmeticCell,
    output_bit: usize,
    arrivals: &[Option<u32>],
    fanouts: &[usize],
    mapping: ArithmeticMapping,
) -> Option<u32> {
    let operand_arrival = (0..=output_bit)
        .flat_map(|input_bit| {
            let arc = arithmetic_arc_ps(arithmetic, output_bit, input_bit, mapping);
            [arithmetic.lhs()[input_bit], arithmetic.rhs()[input_bit]]
                .into_iter()
                .filter_map(move |input| {
                    arrivals[input.index() as usize].map(|arrival| {
                        arrival + wire_delay_ps(fanouts[input.index() as usize]) + arc
                    })
                })
        })
        .max();
    let carry_arrival = arithmetic.carry_in().and_then(|carry_in| {
        arrivals[carry_in.index() as usize].map(|arrival| {
            arrival
                + wire_delay_ps(fanouts[carry_in.index() as usize])
                + arithmetic_carry_arc_ps(arithmetic, output_bit, mapping)
        })
    });
    operand_arrival.into_iter().chain(carry_arrival).max()
}

fn comparison_arc_ps(width: usize, input_bit: usize, signed: bool) -> u32 {
    CCU_INPUT_PS
        + CCU_CARRY_PS * u32::try_from(width - input_bit).unwrap()
        + CCU_SUM_PS
        + if signed { LUT_DELAY_PS } else { 0 }
}

#[allow(clippy::too_many_arguments)]
fn required_times(
    netlist: &Netlist,
    plans: &[Option<LutPlan>],
    arrivals: &[Option<u32>],
    fanouts: &[usize],
    retained: &RetainedTiming,
    options: MappingOptions,
    period_ps: u32,
    io_timing: &IoTimingConstraints,
    preserve_initial: bool,
) -> Vec<u32> {
    let mut required = vec![u32::MAX; netlist.nodes().len()];
    let mut constrain = |net: NetId, sink_delay: u32| {
        let index = net.index() as usize;
        let Some(arrival) = arrivals[index] else {
            return;
        };
        let deadline = period_ps.saturating_sub(sink_delay);
        let deadline = if preserve_initial {
            deadline.max(arrival)
        } else {
            deadline
        };
        required[index] = required[index].min(deadline);
    };

    for (name, output_delay_ps) in &io_timing.output_delays_ps {
        let port = netlist
            .ports()
            .iter()
            .find(|port| port.name() == name)
            .expect("I/O timing constraints were validated");
        for output in port.bits() {
            constrain(node_for(netlist, *output).inputs()[0], *output_delay_ps);
        }
    }

    for register in netlist.registers() {
        constrain(register.data(), FLIP_FLOP_SETUP_PS);
        if let Some(enable) = register.enable() {
            constrain(enable.signal, FLIP_FLOP_SETUP_PS);
        }
        if let Some(reset) = register.reset() {
            constrain(reset.signal, FLIP_FLOP_SETUP_PS);
        }
    }
    for memory in netlist.memories() {
        for input in memory
            .read_address()
            .iter()
            .chain(memory.write_address())
            .chain(memory.write_data())
            .copied()
            .chain([memory.write_enable().signal])
            .chain(memory.read_enable().map(|enable| enable.signal))
            .chain(memory.second_port().into_iter().flat_map(|port| {
                port.read_address()
                    .iter()
                    .chain(port.write_address())
                    .chain(port.write_data())
                    .copied()
                    .chain([port.write_enable().signal])
                    .chain(port.read_enable().map(|enable| enable.signal))
            }))
        {
            constrain(input, BRAM_SETUP_PS);
        }
    }

    for index in (0..netlist.nodes().len()).rev() {
        if required[index] == u32::MAX {
            continue;
        }
        let node = &netlist.nodes()[index];
        match node.kind() {
            NodeKind::And | NodeKind::Or | NodeKind::Xor | NodeKind::Not | NodeKind::Mux => {
                let plan = plans[index]
                    .as_ref()
                    .expect("a Boolean node has a LUT plan");
                for (leaf_index, leaf) in plan.leaves.iter().enumerate() {
                    tighten_required(
                        *leaf,
                        required[index],
                        wide_lut_input_delay_ps(plan.leaves.len(), leaf_index),
                        fanouts,
                        &mut required,
                    );
                }
            }
            NodeKind::ArithmeticOutput(_) | NodeKind::ComparisonOutput(_) => {
                retained.propagate_required(
                    netlist,
                    node.output(),
                    required[index],
                    fanouts,
                    options.arithmetic,
                    &mut required,
                );
            }
            NodeKind::Input(_)
            | NodeKind::Constant(_)
            | NodeKind::RegisterOutput(_)
            | NodeKind::Output(_)
            | NodeKind::MemoryOutput(_) => {}
        }
    }
    required
}

fn tighten_required(
    input: NetId,
    required_output: u32,
    cell_delay: u32,
    fanouts: &[usize],
    required: &mut [u32],
) {
    let index = input.index() as usize;
    let deadline = required_output.saturating_sub(cell_delay + wire_delay_ps(fanouts[index]));
    required[index] = required[index].min(deadline);
}

fn timing_is_valid(arrivals: &[Option<u32>], required: &[u32]) -> bool {
    arrivals.iter().zip(required).all(|(arrival, required)| {
        arrival.is_none_or(|arrival| *required == u32::MAX || arrival <= *required)
    })
}

struct RecoveryContext<'a> {
    netlist: &'a Netlist,
    cuts: &'a CutDatabase,
    roots: &'a [NetId],
    fanouts: &'a [usize],
    retained: &'a RetainedTiming,
    options: MappingOptions,
}

impl RecoveryContext<'_> {
    fn recover_area(
        &self,
        plans: &mut [Option<LutPlan>],
        arrivals: &mut [Option<u32>],
        required: &mut [u32],
    ) {
        let mut references = vec![0usize; plans.len()];
        for root in self.roots {
            reference_node(*root, plans, &mut references);
        }

        for _ in 0..MAX_AREA_RECOVERY_PASSES {
            let mut changed = false;
            for index in (0..plans.len()).rev() {
                if references[index] == 0 || plans[index].is_none() {
                    continue;
                }
                let original = plans[index].as_ref().expect("checked above").clone();
                dereference_leaves(&original.leaves, plans, &mut references);
                let replacement = self.cuts.cuts[index]
                    .iter()
                    .map(|cut| plan_for_cut(plans, arrivals, self.fanouts, cut))
                    .filter(|plan| {
                        plan.arrival_ps
                            .is_none_or(|arrival| arrival <= required[index])
                    })
                    .map(|plan| {
                        let leaf_area = reference_leaves(&plan.leaves, plans, &mut references);
                        let removed = dereference_leaves(&plan.leaves, plans, &mut references);
                        debug_assert_eq!(leaf_area, removed);
                        (plan.root_area + leaf_area, plan)
                    })
                    .min_by_key(|(area, plan)| {
                        (
                            *area,
                            plan.arrival_ps,
                            plan.leaves.len(),
                            plan.leaves.clone(),
                        )
                    })
                    .map_or(original, |(_, plan)| plan);
                changed |=
                    replacement.leaves != plans[index].as_ref().expect("checked above").leaves;
                reference_leaves(&replacement.leaves, plans, &mut references);
                for (position, leaf) in replacement.leaves.iter().enumerate() {
                    let leaf_index = leaf.index() as usize;
                    if plans[leaf_index].is_some() {
                        let required_leaf = required[index].saturating_sub(
                            wide_lut_input_delay_ps(replacement.leaves.len(), position)
                                + wire_delay_ps(self.fanouts[leaf_index]),
                        );
                        required[leaf_index] = required[leaf_index].min(required_leaf);
                    }
                }
                plans[index] = Some(replacement);
            }
            refresh_arrivals(
                self.netlist,
                plans,
                arrivals,
                self.fanouts,
                self.retained,
                self.options.arithmetic,
            );
            if !changed {
                break;
            }
        }
    }
}

fn refresh_arrivals(
    netlist: &Netlist,
    plans: &mut [Option<LutPlan>],
    arrivals: &mut [Option<u32>],
    fanouts: &[usize],
    retained: &RetainedTiming,
    arithmetic_mapping: ArithmeticMapping,
) {
    for node in netlist.nodes() {
        let index = node.output().index() as usize;
        match node.kind() {
            NodeKind::And | NodeKind::Or | NodeKind::Xor | NodeKind::Not | NodeKind::Mux => {
                let leaves = plans[index]
                    .as_ref()
                    .expect("a Boolean node has a LUT plan")
                    .leaves
                    .clone();
                let plan = plan_for_cut(plans, arrivals, fanouts, &Cut { leaves });
                arrivals[index] = plan.arrival_ps;
                plans[index] = Some(plan);
            }
            NodeKind::MemoryOutput(_) => arrivals[index] = Some(BRAM_CLOCK_TO_OUTPUT_PS),
            NodeKind::ArithmeticOutput(_) | NodeKind::ComparisonOutput(_) => {
                arrivals[index] = retained.output_arrival(
                    netlist,
                    node.output(),
                    arrivals,
                    fanouts,
                    arithmetic_mapping,
                );
            }
            NodeKind::Output(_) => arrivals[index] = arrivals[node.inputs()[0].index() as usize],
            NodeKind::RegisterOutput(_) => {
                arrivals[index] = Some(FLIP_FLOP_CLOCK_TO_OUTPUT_PS);
            }
            NodeKind::Input(_) | NodeKind::Constant(_) => {}
        }
    }
}

fn reference_leaves(
    leaves: &[NetId],
    plans: &[Option<LutPlan>],
    references: &mut [usize],
) -> usize {
    leaves
        .iter()
        .map(|leaf| reference_node(*leaf, plans, references))
        .sum()
}

fn reference_node(net: NetId, plans: &[Option<LutPlan>], references: &mut [usize]) -> usize {
    let index = net.index() as usize;
    let Some(plan) = &plans[index] else {
        return 0;
    };
    references[index] += 1;
    if references[index] > 1 {
        return 0;
    }
    plan.root_area + reference_leaves(&plan.leaves, plans, references)
}

fn dereference_leaves(
    leaves: &[NetId],
    plans: &[Option<LutPlan>],
    references: &mut [usize],
) -> usize {
    leaves
        .iter()
        .map(|leaf| dereference_node(*leaf, plans, references))
        .sum()
}

fn dereference_node(net: NetId, plans: &[Option<LutPlan>], references: &mut [usize]) -> usize {
    let index = net.index() as usize;
    let Some(plan) = &plans[index] else {
        return 0;
    };
    debug_assert!(references[index] > 0);
    references[index] -= 1;
    if references[index] > 0 {
        return 0;
    }
    plan.root_area + dereference_leaves(&plan.leaves, plans, references)
}

/// Materializes only the selected LUT cover reachable from requested roots.
pub(super) struct LutEmitter<'a> {
    netlist: &'a Netlist,
    cover: &'a LutCover,
    bits: Vec<Option<Bit>>,
    cells: Vec<Ecp5Cell>,
    next_wire: u32,
}

impl<'a> LutEmitter<'a> {
    pub(super) fn new(netlist: &'a Netlist, cover: &'a LutCover) -> Self {
        let mut bits = vec![None; netlist.nodes().len()];
        for node in netlist.nodes() {
            let index = node.output().index() as usize;
            match node.kind() {
                NodeKind::Input(_) | NodeKind::RegisterOutput(_) | NodeKind::MemoryOutput(_) => {
                    bits[index] = Some(wire_for(node.output()));
                }
                NodeKind::Constant(value) => bits[index] = Some(Bit::from(*value)),
                NodeKind::And
                | NodeKind::Or
                | NodeKind::Xor
                | NodeKind::Not
                | NodeKind::Mux
                | NodeKind::Output(_)
                | NodeKind::ArithmeticOutput(_)
                | NodeKind::ComparisonOutput(_) => {}
            }
        }
        Self {
            netlist,
            cover,
            bits,
            cells: Vec::new(),
            next_wire: u32::try_from(netlist.nodes().len())
                .expect("netlist exceeds the Yosys JSON range")
                .checked_add(2)
                .expect("netlist exceeds the Yosys JSON range"),
        }
    }

    pub(super) fn map_net(&mut self, net: NetId) -> Bit {
        if let Some(bit) = self.bits[net.index() as usize] {
            return bit;
        }

        let plan = self
            .cover
            .plan(net)
            .cloned()
            .expect("only Boolean logic requires a LUT plan");
        let output = wire_number(net);
        let inputs = plan
            .leaves
            .iter()
            .map(|leaf| self.map_net(*leaf))
            .collect::<Vec<_>>();
        let truth = cut_truth_table(self.netlist, net, &plan.leaves);
        self.emit_lut_plan(net, &inputs, output, truth);
        let bit = Bit::Wire(output);
        self.bits[net.index() as usize] = Some(bit);
        bit
    }

    fn emit_lut_plan(&mut self, net: NetId, inputs: &[Bit], output: u32, truth: u128) {
        self.emit_wide_lut(&format!("lut{}", net.index()), inputs, output, truth);
    }

    fn emit_lut4(&mut self, name: String, inputs: &[Bit], output: u32, init: u16) {
        let mut padded = [Bit::Zero; LUT_INPUTS];
        padded[..inputs.len()].copy_from_slice(inputs);
        self.cells.push(Ecp5Cell::Lut4 {
            name,
            inputs: padded,
            output,
            init,
        });
    }

    fn emit_wide_lut(&mut self, name: &str, inputs: &[Bit], output: u32, truth: u128) {
        if inputs.len() <= LUT_INPUTS {
            self.emit_lut4(
                name.to_owned(),
                inputs,
                output,
                lut4_init(truth, inputs.len()),
            );
            return;
        }

        debug_assert!(inputs.len() <= WIDE_LUT_INPUTS);
        let cofactor_bits = 1usize << (inputs.len() - 1);
        let cofactor_mask = (1u128 << cofactor_bits) - 1;
        let data_zero = self.fresh_wire();
        let data_one = self.fresh_wire();
        self.emit_wide_lut(
            &format!("{name}_wide0"),
            &inputs[..inputs.len() - 1],
            data_zero,
            truth & cofactor_mask,
        );
        self.emit_wide_lut(
            &format!("{name}_wide1"),
            &inputs[..inputs.len() - 1],
            data_one,
            truth >> cofactor_bits,
        );
        if inputs.len() == 5 {
            self.cells.push(Ecp5Cell::PfuMux {
                name: name.to_owned(),
                lut_true: Bit::Wire(data_one),
                lut_false: Bit::Wire(data_zero),
                select: inputs[4],
                output,
            });
        } else {
            self.cells.push(Ecp5Cell::L6Mux21 {
                name: name.to_owned(),
                data_zero: Bit::Wire(data_zero),
                data_one: Bit::Wire(data_one),
                select: inputs[inputs.len() - 1],
                output,
            });
        }
    }

    pub(super) fn alias_net(&mut self, net: NetId, bit: Bit) {
        self.bits[net.index() as usize] = Some(bit);
    }

    pub(super) fn mapped_net(&self, net: NetId) -> Bit {
        self.bits[net.index() as usize].expect("mapping demand was materialized")
    }

    pub(super) fn fresh_wire(&mut self) -> u32 {
        let wire = self.next_wire;
        self.next_wire = self
            .next_wire
            .checked_add(1)
            .expect("mapped netlist exceeds the Yosys JSON range");
        wire
    }

    pub(super) fn push_cell(&mut self, cell: Ecp5Cell) {
        self.cells.push(cell);
    }

    pub(super) fn finish(self) -> (Vec<Option<Bit>>, Vec<Ecp5Cell>) {
        (self.bits, self.cells)
    }
}

fn lut4_init(truth: u128, inputs: usize) -> u16 {
    let assignments = 1usize << inputs;
    (0..16).fold(0, |init, assignment| {
        let value = (truth >> (assignment % assignments)) & 1;
        init | (u16::try_from(value).unwrap() << assignment)
    })
}

fn best_plan(
    plans: &[Option<LutPlan>],
    arrivals: &[Option<u32>],
    fanouts: &[usize],
    cuts: &[Cut],
    critical_arrival_ps: u32,
) -> LutPlan {
    let candidates = cuts
        .iter()
        .map(|cut| plan_for_cut(plans, arrivals, fanouts, cut))
        .collect::<Vec<_>>();
    let minimum_depth = candidates
        .iter()
        .map(|plan| plan.depth)
        .min()
        .expect("a Boolean node always has a direct-input cut");
    let minimum_depth_arrival = candidates
        .iter()
        .filter(|plan| plan.depth == minimum_depth)
        .filter_map(|plan| plan.arrival_ps)
        .min()
        .unwrap_or(0);
    if candidates[0].arrival_ps.is_some() && minimum_depth_arrival >= critical_arrival_ps {
        candidates
            .into_iter()
            .min_by_key(|plan| {
                (
                    plan.arrival_ps,
                    plan.depth,
                    plan.area,
                    plan.leaves.len(),
                    plan.leaves.clone(),
                )
            })
            .expect("the candidate set is non-empty")
    } else {
        candidates
            .into_iter()
            .filter(|plan| plan.depth == minimum_depth)
            .min_by_key(|plan| (plan.area, plan.leaves.len(), plan.leaves.clone()))
            .expect("the minimum-depth set is non-empty")
    }
}

fn plan_for_cut(
    plans: &[Option<LutPlan>],
    arrivals: &[Option<u32>],
    fanouts: &[usize],
    cut: &Cut,
) -> LutPlan {
    let mut leaves = cut.leaves.clone();
    if leaves.len() > LUT_INPUTS {
        leaves.sort_unstable_by_key(|leaf| {
            let index = leaf.index() as usize;
            (
                arrivals[index].unwrap_or(0) + wire_delay_ps(fanouts[index]),
                *leaf,
            )
        });
    }
    let root_area = wide_lut_area(leaves.len());
    let area = root_area
        + leaves
            .iter()
            .filter_map(|leaf| plans[leaf.index() as usize].as_ref())
            .map(|plan| plan.area)
            .sum::<usize>();
    let arrival_ps = leaves
        .iter()
        .enumerate()
        .filter_map(|(position, leaf)| {
            let index = leaf.index() as usize;
            arrivals[index].map(|arrival| {
                arrival
                    + wire_delay_ps(fanouts[index])
                    + wide_lut_input_delay_ps(leaves.len(), position)
            })
        })
        .max();
    let depth = 1 + leaves
        .iter()
        .filter_map(|leaf| plans[leaf.index() as usize].as_ref())
        .map(|plan| plan.depth)
        .max()
        .unwrap_or(0);
    LutPlan {
        leaves,
        depth,
        arrival_ps,
        area,
        root_area,
    }
}

fn wide_lut_area(inputs: usize) -> usize {
    match inputs {
        0..=4 => 1,
        5 => 2,
        6 => 4,
        7 => 8,
        _ => unreachable!("ECP5 wide LUTs support at most seven inputs here"),
    }
}

fn wide_lut_input_delay_ps(inputs: usize, position: usize) -> u32 {
    if inputs <= LUT_INPUTS {
        return LUT_DELAY_PS;
    }
    debug_assert!(inputs <= WIDE_LUT_INPUTS);
    debug_assert!(position < inputs);
    let l6_muxes = u32::try_from(inputs - position.max(5)).unwrap();
    match position {
        0..=3 => LUT_DELAY_PS + PFU_MUX_DELAY_PS + l6_muxes * L6_MUX_DELAY_PS,
        4 => PFU_MUX_DELAY_PS + l6_muxes * L6_MUX_DELAY_PS,
        _ => l6_muxes * L6_MUX_DELAY_PS,
    }
}

fn enumerate_cuts(netlist: &Netlist, root: NetId, max_inputs: usize) -> Vec<Vec<NetId>> {
    let mut first = node_for(netlist, root)
        .inputs()
        .iter()
        .copied()
        .filter(|net| !matches!(node_for(netlist, *net).kind(), NodeKind::Constant(_)))
        .collect::<Vec<_>>();
    first.sort_unstable();
    first.dedup();

    let mut seen = BTreeSet::from([first.clone()]);
    let mut cuts = vec![first];
    let mut cursor = 0;
    while cursor < cuts.len() && cuts.len() < CUT_LIMIT {
        let cut = cuts[cursor].clone();
        cursor += 1;
        for leaf in cut.iter().copied() {
            if !is_boolean(node_for(netlist, leaf).kind()) {
                continue;
            }
            let mut expanded = cut
                .iter()
                .copied()
                .filter(|candidate| *candidate != leaf)
                .chain(
                    node_for(netlist, leaf)
                        .inputs()
                        .iter()
                        .copied()
                        .filter(|net| {
                            !matches!(node_for(netlist, *net).kind(), NodeKind::Constant(_))
                        }),
                )
                .collect::<Vec<_>>();
            expanded.sort_unstable();
            expanded.dedup();
            if expanded.len() <= max_inputs && seen.insert(expanded.clone()) {
                cuts.push(expanded);
                if cuts.len() == CUT_LIMIT {
                    break;
                }
            }
        }
    }
    cuts
}

fn cut_truth_table(netlist: &Netlist, root: NetId, leaves: &[NetId]) -> u128 {
    (0..(1u128 << leaves.len())).fold(0, |table, assignment| {
        let mut values = vec![None; netlist.nodes().len()];
        let value = evaluate_cut(netlist, root, leaves, assignment, &mut values);
        table | (u128::from(value) << assignment)
    })
}

fn evaluate_cut(
    netlist: &Netlist,
    net: NetId,
    leaves: &[NetId],
    assignment: u128,
    values: &mut [Option<bool>],
) -> bool {
    if let Some(index) = leaves.iter().position(|leaf| *leaf == net) {
        return assignment & (1 << index) != 0;
    }
    if let Some(value) = values[net.index() as usize] {
        return value;
    }
    let node = node_for(netlist, net);
    let value = match node.kind() {
        NodeKind::Constant(value) => *value,
        NodeKind::And => {
            evaluate_cut(netlist, node.inputs()[0], leaves, assignment, values)
                & evaluate_cut(netlist, node.inputs()[1], leaves, assignment, values)
        }
        NodeKind::Or => {
            evaluate_cut(netlist, node.inputs()[0], leaves, assignment, values)
                | evaluate_cut(netlist, node.inputs()[1], leaves, assignment, values)
        }
        NodeKind::Xor => {
            evaluate_cut(netlist, node.inputs()[0], leaves, assignment, values)
                ^ evaluate_cut(netlist, node.inputs()[1], leaves, assignment, values)
        }
        NodeKind::Not => !evaluate_cut(netlist, node.inputs()[0], leaves, assignment, values),
        NodeKind::Mux => {
            if evaluate_cut(netlist, node.inputs()[0], leaves, assignment, values) {
                evaluate_cut(netlist, node.inputs()[1], leaves, assignment, values)
            } else {
                evaluate_cut(netlist, node.inputs()[2], leaves, assignment, values)
            }
        }
        NodeKind::Input(_)
        | NodeKind::RegisterOutput(_)
        | NodeKind::Output(_)
        | NodeKind::MemoryOutput(_)
        | NodeKind::ArithmeticOutput(_)
        | NodeKind::ComparisonOutput(_) => {
            unreachable!("a cut must stop before a source or output node")
        }
    };
    values[net.index() as usize] = Some(value);
    value
}

fn is_boolean(kind: &NodeKind) -> bool {
    matches!(
        kind,
        NodeKind::And | NodeKind::Or | NodeKind::Xor | NodeKind::Not | NodeKind::Mux
    )
}

#[cfg(test)]
mod tests {
    use std::num::NonZeroU32;

    use struo_ir::{ArithmeticOp, Netlist};

    use super::{
        ArithmeticMapping, Cut, RetainedTiming, best_plan, plan_for_cut, structural_fanouts,
    };

    #[test]
    fn carry_source_arrival_grows_toward_the_most_significant_bit() {
        let mut netlist = Netlist::new("carry_timing");
        let width = NonZeroU32::new(8).unwrap();
        let lhs = netlist.add_input_port("lhs", width);
        let rhs = netlist.add_input_port("rhs", width);
        let outputs = netlist
            .add_arithmetic(ArithmeticOp::Add, &lhs, &rhs)
            .unwrap();
        let retained = RetainedTiming::new(&netlist);
        let arrivals = vec![Some(0); netlist.nodes().len()];
        let fanouts = structural_fanouts(&netlist);

        let least_significant = retained.output_arrival(
            &netlist,
            outputs[0],
            &arrivals,
            &fanouts,
            ArithmeticMapping::CarryChain,
        );
        let most_significant = retained.output_arrival(
            &netlist,
            outputs[7],
            &arrivals,
            &fanouts,
            ArithmeticMapping::CarryChain,
        );

        assert!(most_significant.unwrap() > least_significant.unwrap());
    }

    #[test]
    fn structural_fanout_breaks_equal_timing_and_area_choices() {
        let mut netlist = Netlist::new("fanout_tie_break");
        let shared = netlist.add_input("shared");
        let crowded = netlist.add_input("crowded");
        let local = netlist.add_input("local");
        let plans = vec![None; netlist.nodes().len()];
        let arrivals = vec![Some(0); netlist.nodes().len()];
        let mut fanouts = vec![1; netlist.nodes().len()];
        fanouts[crowded.index() as usize] = 16;
        let cuts = [
            Cut {
                leaves: vec![shared, crowded],
            },
            Cut {
                leaves: vec![shared, local],
            },
        ];

        let selected = best_plan(&plans, &arrivals, &fanouts, &cuts, 0);

        assert_eq!(selected.leaves, vec![shared, local]);
    }

    #[test]
    fn wide_cut_places_the_latest_input_on_the_top_mux_select() {
        let mut netlist = Netlist::new("wide_pin_order");
        let inputs = (0..7)
            .map(|index| netlist.add_input(format!("input{index}")))
            .collect::<Vec<_>>();
        let plans = vec![None; netlist.nodes().len()];
        let mut arrivals = vec![Some(0); netlist.nodes().len()];
        arrivals[inputs[0].index() as usize] = Some(1_000);
        arrivals[inputs[1].index() as usize] = Some(500);
        arrivals[inputs[2].index() as usize] = Some(250);
        let fanouts = vec![1; netlist.nodes().len()];

        let plan = plan_for_cut(
            &plans,
            &arrivals,
            &fanouts,
            &Cut {
                leaves: inputs.clone(),
            },
        );

        assert_eq!(plan.leaves[4], inputs[2]);
        assert_eq!(plan.leaves[5], inputs[1]);
        assert_eq!(plan.leaves[6], inputs[0]);
        assert_eq!(plan.root_area, 8);
    }

    #[test]
    fn critical_timing_can_choose_a_faster_deeper_cut() {
        let mut netlist = Netlist::new("critical_cut");
        let slow_source = netlist.add_input("slow_source");
        let fast_source = netlist.add_input("fast_source");
        let internal = netlist.add_not(fast_source);
        let mut plans = vec![None; netlist.nodes().len()];
        plans[internal.index() as usize] = Some(super::LutPlan {
            leaves: vec![fast_source],
            depth: 1,
            arrival_ps: Some(500),
            area: 1,
            root_area: 1,
        });
        let mut arrivals = vec![Some(0); netlist.nodes().len()];
        arrivals[slow_source.index() as usize] = Some(1_500);
        arrivals[internal.index() as usize] = Some(500);
        let fanouts = vec![1; netlist.nodes().len()];
        let cuts = [
            Cut {
                leaves: vec![slow_source],
            },
            Cut {
                leaves: vec![internal],
            },
        ];

        let area_driven = best_plan(&plans, &arrivals, &fanouts, &cuts, u32::MAX);
        let timing_driven = best_plan(&plans, &arrivals, &fanouts, &cuts, 0);

        assert_eq!(area_driven.leaves, vec![slow_source]);
        assert_eq!(timing_driven.leaves, vec![internal]);
        assert!(timing_driven.depth > area_driven.depth);
        assert!(timing_driven.arrival_ps < area_driven.arrival_ps);
    }
}
