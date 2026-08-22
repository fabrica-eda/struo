//! Four-input cut analysis, cover selection, and LUT emission.

use std::collections::BTreeSet;

use struo_ir::{NetId, Netlist, NodeKind};

use super::{Bit, Ecp5Cell, node_for, wire_for, wire_number};

const LUT_INPUTS: usize = 4;
const CUT_LIMIT: usize = 64;

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
        Self { plans }
    }

    fn plan(&self, net: NetId) -> Option<&LutPlan> {
        self.plans[net.index() as usize].as_ref()
    }
}

/// Materializes only the selected LUT cover reachable from requested roots.
pub(super) struct LutEmitter<'a> {
    netlist: &'a Netlist,
    cover: &'a LutCover,
    bits: Vec<Option<Bit>>,
    cells: Vec<Ecp5Cell>,
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
                | NodeKind::Output(_) => {}
            }
        }
        Self {
            netlist,
            cover,
            bits,
            cells: Vec::new(),
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

    pub(super) fn finish(self) -> (Vec<Option<Bit>>, Vec<Ecp5Cell>) {
        (self.bits, self.cells)
    }
}

fn best_plan(plans: &[Option<LutPlan>], cuts: &[Cut]) -> LutPlan {
    cuts.iter()
        .map(|cut| {
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
        })
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
        | NodeKind::MemoryOutput(_) => {
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
