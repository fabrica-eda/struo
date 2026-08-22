//! Four-input cut analysis, cover selection, and LUT emission.

use std::collections::BTreeSet;

use struo_ir::{NetId, Netlist, NodeKind, PortDirection};

use super::{Bit, Ecp5Cell, node_for, wire_for, wire_number};

const LUT_INPUTS: usize = 4;
const CUT_LIMIT: usize = 64;
const AREA_RECOVERY_PASSES: usize = 3;

#[derive(Clone, Debug)]
struct Cut {
    leaves: Vec<NetId>,
}

/// Immutable feasible-cut analysis for one Boolean netlist.
pub(super) struct CutDatabase {
    cuts: Vec<Vec<Cut>>,
}

impl CutDatabase {
    pub(super) fn analyze(netlist: &Netlist) -> Self {
        let cuts = netlist
            .nodes()
            .iter()
            .map(|node| {
                if is_boolean(node.kind()) {
                    enumerate_cuts(netlist, node.output())
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
    area: usize,
}

/// Delay-first, area-second cover selected from a feasible-cut database.
pub(super) struct LutCover {
    plans: Vec<Option<LutPlan>>,
}

impl LutCover {
    pub(super) fn select(netlist: &Netlist, cuts: &CutDatabase) -> Self {
        let mut plans = vec![None; netlist.nodes().len()];
        for node in netlist.nodes() {
            if is_boolean(node.kind()) {
                plans[node.output().index() as usize] =
                    Some(best_plan(&plans, cuts.for_net(node.output())));
            }
        }
        let roots = mapping_roots(netlist);
        let (mut required, target_depth) = required_depths(&plans, &roots);
        recover_area(cuts, &roots, &mut plans, &mut required);
        debug_assert!(
            roots
                .iter()
                .filter_map(|root| plans[root.index() as usize].as_ref())
                .all(|plan| plan.depth <= target_depth)
        );
        Self { plans }
    }

    fn plan(&self, net: NetId) -> Option<&LutPlan> {
        self.plans[net.index() as usize].as_ref()
    }
}

fn mapping_roots(netlist: &Netlist) -> Vec<NetId> {
    let output_roots = netlist
        .ports()
        .iter()
        .filter(|port| port.direction() == PortDirection::Output)
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
    let mut roots = output_roots
        .chain(register_roots)
        .chain(memory_roots)
        .chain(arithmetic_roots)
        .chain(comparison_roots)
        .filter(|net| is_boolean(node_for(netlist, *net).kind()))
        .collect::<Vec<_>>();
    roots.sort_unstable();
    roots.dedup();
    roots
}

fn required_depths(plans: &[Option<LutPlan>], roots: &[NetId]) -> (Vec<usize>, usize) {
    let target = roots
        .iter()
        .filter_map(|root| plans[root.index() as usize].as_ref())
        .map(|plan| plan.depth)
        .max()
        .unwrap_or(0);
    let mut required = vec![usize::MAX; plans.len()];
    for root in roots {
        required[root.index() as usize] = target;
    }
    propagate_required(plans, &mut required);
    (required, target)
}

fn propagate_required(plans: &[Option<LutPlan>], required: &mut [usize]) {
    for index in (0..plans.len()).rev() {
        let Some(plan) = &plans[index] else {
            continue;
        };
        let required_here = required[index];
        if required_here == usize::MAX {
            continue;
        }
        let required_leaf = required_here.saturating_sub(1);
        for leaf in &plan.leaves {
            let leaf_index = leaf.index() as usize;
            if plans[leaf_index].is_some() {
                required[leaf_index] = required[leaf_index].min(required_leaf);
            }
        }
    }
}

fn recover_area(
    cuts: &CutDatabase,
    roots: &[NetId],
    plans: &mut [Option<LutPlan>],
    required: &mut [usize],
) {
    let mut references = vec![0usize; plans.len()];
    for root in roots {
        reference_node(*root, plans, &mut references);
    }

    for _ in 0..AREA_RECOVERY_PASSES {
        for index in (0..plans.len()).rev() {
            if references[index] == 0 || plans[index].is_none() {
                continue;
            }
            let original = plans[index].as_ref().expect("checked above").clone();
            dereference_leaves(&original.leaves, plans, &mut references);
            let replacement = cuts.cuts[index]
                .iter()
                .map(|cut| plan_for_cut(plans, cut))
                .filter(|plan| plan.depth <= required[index])
                .map(|plan| {
                    let area = reference_leaves(&plan.leaves, plans, &mut references);
                    let removed = dereference_leaves(&plan.leaves, plans, &mut references);
                    debug_assert_eq!(area, removed);
                    (area, plan)
                })
                .min_by_key(|(area, plan)| {
                    (*area, plan.depth, plan.leaves.len(), plan.leaves.clone())
                })
                .map_or(original, |(_, plan)| plan);
            reference_leaves(&replacement.leaves, plans, &mut references);
            let required_leaf = required[index].saturating_sub(1);
            for leaf in &replacement.leaves {
                let leaf_index = leaf.index() as usize;
                if plans[leaf_index].is_some() {
                    required[leaf_index] = required[leaf_index].min(required_leaf);
                }
            }
            plans[index] = Some(replacement);
        }
        refresh_depths(plans);
    }
}

fn refresh_depths(plans: &mut [Option<LutPlan>]) {
    for index in 0..plans.len() {
        let Some(plan) = &plans[index] else {
            continue;
        };
        let depth = 1 + plan
            .leaves
            .iter()
            .filter_map(|leaf| plans[leaf.index() as usize].as_ref())
            .map(|leaf_plan| leaf_plan.depth)
            .max()
            .unwrap_or(0);
        plans[index].as_mut().expect("checked above").depth = depth;
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
    1 + reference_leaves(&plan.leaves, plans, references)
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
    1 + dereference_leaves(&plan.leaves, plans, references)
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
        let mut inputs = [Bit::Zero; LUT_INPUTS];
        for (target, leaf) in inputs.iter_mut().zip(&plan.leaves) {
            *target = self.map_net(*leaf);
        }

        let output = wire_number(net);
        self.cells.push(Ecp5Cell::Lut4 {
            name: format!("lut{}", net.index()),
            inputs,
            output,
            init: cut_truth_table(self.netlist, net, &plan.leaves),
        });
        let bit = Bit::Wire(output);
        self.bits[net.index() as usize] = Some(bit);
        bit
    }

    pub(super) fn alias_net(&mut self, net: NetId, bit: Bit) {
        self.bits[net.index() as usize] = Some(bit);
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

fn best_plan(plans: &[Option<LutPlan>], cuts: &[Cut]) -> LutPlan {
    cuts.iter()
        .map(|cut| plan_for_cut(plans, cut))
        .min_by_key(|plan| {
            (
                plan.depth,
                plan.area,
                plan.leaves.len(),
                plan.leaves.clone(),
            )
        })
        .expect("a Boolean node always has a direct-input cut")
}

fn plan_for_cut(plans: &[Option<LutPlan>], cut: &Cut) -> LutPlan {
    let area = 1 + cut
        .leaves
        .iter()
        .filter_map(|leaf| plans[leaf.index() as usize].as_ref())
        .map(|plan| plan.area)
        .sum::<usize>();
    let depth = 1 + cut
        .leaves
        .iter()
        .filter_map(|leaf| plans[leaf.index() as usize].as_ref())
        .map(|plan| plan.depth)
        .max()
        .unwrap_or(0);
    LutPlan {
        leaves: cut.leaves.clone(),
        depth,
        area,
    }
}

fn enumerate_cuts(netlist: &Netlist, root: NetId) -> Vec<Vec<NetId>> {
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
            if expanded.len() <= LUT_INPUTS && seen.insert(expanded.clone()) {
                cuts.push(expanded);
                if cuts.len() == CUT_LIMIT {
                    break;
                }
            }
        }
    }
    cuts
}

fn cut_truth_table(netlist: &Netlist, root: NetId, leaves: &[NetId]) -> u16 {
    (0..16).fold(0, |table, assignment| {
        let mut values = vec![None; netlist.nodes().len()];
        let value = evaluate_cut(netlist, root, leaves, assignment, &mut values);
        table | (u16::from(value) << assignment)
    })
}

fn evaluate_cut(
    netlist: &Netlist,
    net: NetId,
    leaves: &[NetId],
    assignment: u16,
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
