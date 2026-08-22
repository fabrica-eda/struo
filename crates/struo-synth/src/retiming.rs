//! Timing-driven, certificate-checked register retiming.

use std::collections::{HashMap, HashSet, VecDeque};
use std::num::NonZeroU32;

use struo_formal::{
    LogicFunction, RetimingCertificate, RetimingDomain, RetimingEdge, RetimingGraph,
    RetimingVertex, derive_retimed_graph, verify_retiming_certificate,
};
use struo_ir::{
    ActiveLevel, ClockEdge, EnableControl, NetId, Netlist, NodeKind, PortDirection, RegisterCell,
    ResetControl,
};

use crate::{Pass, PassReport, SynthesisError};

/// Moves resettable registers across Boolean logic to cap unregistered depth.
///
/// Primary ports, retained word cells, memories, inverters, clocks, and resets
/// are fixed boundaries. Clock enables are represented as feedback muxes while
/// retiming and recovered by [`crate::InferRegisterEnables`] afterwards.
pub struct TimingDrivenRetiming {
    target_depth: usize,
    net_delays: HashMap<NetId, usize>,
    movable_nets: Option<HashSet<NetId>>,
}

impl TimingDrivenRetiming {
    /// Creates a pass targeting at most `target_depth` Boolean nodes between
    /// registers. A zero target is accepted but cannot move logic through I/O.
    #[must_use]
    pub fn new(target_depth: usize) -> Self {
        Self {
            target_depth,
            net_delays: HashMap::new(),
            movable_nets: None,
        }
    }

    /// Creates a pass with target-provided combinational delay annotations.
    /// Unannotated Boolean nodes retain the unit-delay model.
    #[must_use]
    pub fn with_net_delays(
        target_depth: usize,
        delays: impl IntoIterator<Item = (NetId, usize)>,
    ) -> Self {
        Self {
            target_depth,
            net_delays: delays.into_iter().collect(),
            movable_nets: None,
        }
    }

    /// Restricts target-annotated retiming to selected nets and their explicit
    /// register-output vertices.
    #[must_use]
    pub fn with_net_delays_and_focus(
        target_depth: usize,
        delays: impl IntoIterator<Item = (NetId, usize)>,
        movable_nets: impl IntoIterator<Item = NetId>,
    ) -> Self {
        Self {
            target_depth,
            net_delays: delays.into_iter().collect(),
            movable_nets: Some(movable_nets.into_iter().collect()),
        }
    }
}

impl Pass for TimingDrivenRetiming {
    fn name(&self) -> &'static str {
        "timing-driven-retiming"
    }

    fn run(&self, design: &mut Netlist) -> Result<PassReport, SynthesisError> {
        let Some(mut model) =
            Model::from_netlist(design, &self.net_delays, self.movable_nets.as_ref())
        else {
            return Ok(PassReport {
                pass: self.name(),
                message: "kept registers fixed: retiming requires one reset domain and no memories"
                    .into(),
            });
        };
        let before_depth = model.maximum_depth();
        let fixed_registers = model
            .eligible_registers
            .iter()
            .filter(|eligible| !**eligible)
            .count();
        let before_registers = model.register_count() + fixed_registers;
        let moved = model.retime_forward(self.target_depth);
        if moved == 0 {
            return Ok(PassReport {
                pass: self.name(),
                message: format!(
                    "no legal move improved the depth-{before_depth} register placement (target {})",
                    self.target_depth
                ),
            });
        }

        let certificate = RetimingCertificate::new(model.labels.clone());
        let after_graph = derive_retimed_graph(&model.before_graph, &certificate)
            .map_err(|error| SynthesisError::Transformation(error.to_string()))?;
        verify_retiming_certificate(&model.before_graph, &after_graph, &certificate).map_err(
            |error| {
                SynthesisError::Transformation(format!("invalid retiming certificate: {error}"))
            },
        )?;

        let rebuilt = model.rebuild(design, &after_graph)?;
        let after_depth = model.maximum_depth();
        let after_registers = model.register_count() + fixed_registers;
        rebuilt.validate()?;
        *design = rebuilt;

        Ok(PassReport {
            pass: self.name(),
            message: format!(
                "made {moved} certified moves: Boolean depth {before_depth} -> {after_depth}, registers {before_registers} -> {after_registers} (target {})",
                self.target_depth
            ),
        })
    }
}

#[derive(Clone, Copy, Debug)]
enum VertexKind {
    Net(NetId),
    EnableMux,
}

#[derive(Clone, Debug)]
struct Vertex {
    kind: VertexKind,
    formal: RetimingVertex,
    delay: usize,
    movable: bool,
}

#[derive(Clone, Debug)]
struct Edge {
    source: usize,
    target: usize,
    input: usize,
    original_weight: usize,
}

struct Model {
    vertices: Vec<Vertex>,
    edges: Vec<Edge>,
    labels: Vec<i32>,
    before_graph: RetimingGraph,
    vertex_for_net: HashMap<NetId, usize>,
    eligible_registers: Vec<bool>,
    clock: NetId,
    clock_edge: ClockEdge,
    reset: ResetControl,
}

impl Model {
    #[allow(clippy::too_many_lines)]
    fn from_netlist(
        design: &Netlist,
        net_delays: &HashMap<NetId, usize>,
        movable_nets: Option<&HashSet<NetId>>,
    ) -> Option<Self> {
        if !design.memories().is_empty() || design.registers().is_empty() {
            return None;
        }
        let signal_name = |net: NetId| match design.nodes().get(net.index() as usize)?.kind() {
            NodeKind::Input(name) => Some(name.clone()),
            _ => None,
        };
        let first = design
            .registers()
            .iter()
            .filter(|register| {
                register.reset().is_some()
                    && signal_name(register.clock()).is_some()
                    && register
                        .reset()
                        .and_then(|reset| signal_name(reset.signal))
                        .is_some()
            })
            .max_by_key(|candidate| {
                design
                    .registers()
                    .iter()
                    .filter(|register| {
                        register.clock() == candidate.clock()
                            && register.edge() == candidate.edge()
                            && register.reset() == candidate.reset()
                    })
                    .count()
            })?;
        let reset = first.reset()?;
        let clock = first.clock();
        let eligible_registers = design
            .registers()
            .iter()
            .map(|register| {
                register.clock() == clock
                    && register.edge() == first.edge()
                    && register.reset() == Some(reset)
            })
            .collect::<Vec<_>>();
        let domain = RetimingDomain::new(
            signal_name(clock)?,
            first.edge(),
            signal_name(reset.signal)?,
            reset.active,
            reset.asynchronous,
        );

        let mut fixed_fanin = HashSet::new();
        let mut work = Vec::new();
        for (index, register) in design.registers().iter().enumerate() {
            if !eligible_registers[index] {
                work.push(register.data());
                work.push(register.clock());
                work.extend(register.enable().map(|enable| enable.signal));
                work.extend(register.reset().map(|reset| reset.signal));
                work.push(register.output());
            }
        }
        while let Some(net) = work.pop() {
            if !fixed_fanin.insert(net) {
                continue;
            }
            let node = &design.nodes()[net.index() as usize];
            work.extend(node.inputs().iter().copied());
            if matches!(node.kind(), NodeKind::ArithmeticOutput(_)) {
                if let Some(cell) = design
                    .arithmetic()
                    .iter()
                    .find(|cell| cell.outputs().contains(&net))
                {
                    work.extend(cell.lhs().iter().chain(cell.rhs()).copied());
                }
            } else if matches!(node.kind(), NodeKind::ComparisonOutput(_))
                && let Some(cell) = design
                    .comparisons()
                    .iter()
                    .find(|cell| cell.output() == net)
            {
                work.extend(cell.lhs().iter().chain(cell.rhs()).copied());
            }
        }

        let mut vertices = Vec::with_capacity(design.nodes().len() + design.registers().len());
        let mut vertex_for_net = HashMap::new();
        for node in design.nodes() {
            let (function, mut boundary, default_delay) = vertex_properties(node.kind());
            let delay = net_delays
                .get(&node.output())
                .copied()
                .unwrap_or(default_delay);
            boundary |= fixed_fanin.contains(&node.output());
            let name = match node.kind() {
                NodeKind::Input(name)
                | NodeKind::RegisterOutput(name)
                | NodeKind::Output(name)
                | NodeKind::MemoryOutput(name)
                | NodeKind::ArithmeticOutput(name)
                | NodeKind::ComparisonOutput(name) => name.clone(),
                kind => format!("{kind:?}@{}", node.output()),
            };
            let formal = if boundary {
                RetimingVertex::boundary(name, function)
            } else {
                RetimingVertex::logic(name, function)
            };
            let index = vertices.len();
            vertices.push(Vertex {
                kind: VertexKind::Net(node.output()),
                formal,
                delay,
                movable: movable_nets.is_none_or(|nets| nets.contains(&node.output())),
            });
            vertex_for_net.insert(node.output(), index);
        }

        let mut enable_vertices = vec![None; design.registers().len()];
        for (index, register) in design.registers().iter().enumerate() {
            if eligible_registers[index] && register.enable().is_some() {
                enable_vertices[index] = Some(vertices.len());
                vertices.push(Vertex {
                    kind: VertexKind::EnableMux,
                    formal: RetimingVertex::logic(
                        format!("{}$enable", register.name()),
                        mux_function(),
                    ),
                    delay: 1,
                    movable: movable_nets.is_none_or(|nets| nets.contains(&register.output())),
                });
            }
        }

        let mut edges = Vec::new();
        for node in design.nodes() {
            let target = vertex_for_net[&node.output()];
            match node.kind() {
                NodeKind::RegisterOutput(_) => {}
                NodeKind::ArithmeticOutput(_) => {
                    let cell = design
                        .arithmetic()
                        .iter()
                        .find(|cell| cell.outputs().contains(&node.output()))?;
                    for (input, net) in cell.lhs().iter().chain(cell.rhs()).enumerate() {
                        push_edge(&mut edges, &vertex_for_net, *net, target, input, 0);
                    }
                }
                NodeKind::ComparisonOutput(_) => {
                    let cell = design
                        .comparisons()
                        .iter()
                        .find(|cell| cell.output() == node.output())?;
                    for (input, net) in cell.lhs().iter().chain(cell.rhs()).enumerate() {
                        push_edge(&mut edges, &vertex_for_net, *net, target, input, 0);
                    }
                }
                _ => {
                    for (input, net) in node.inputs().iter().enumerate() {
                        push_edge(&mut edges, &vertex_for_net, *net, target, input, 0);
                    }
                }
            }
        }

        for (index, register) in design.registers().iter().enumerate() {
            if !eligible_registers[index] {
                continue;
            }
            let q = vertex_for_net[&register.output()];
            if let (Some(enable), Some(mux)) = (register.enable(), enable_vertices[index]) {
                let (then_net, else_net) = match enable.active {
                    ActiveLevel::High => (register.data(), register.output()),
                    ActiveLevel::Low => (register.output(), register.data()),
                };
                push_edge(&mut edges, &vertex_for_net, enable.signal, mux, 0, 0);
                push_edge(&mut edges, &vertex_for_net, then_net, mux, 1, 0);
                push_edge(&mut edges, &vertex_for_net, else_net, mux, 2, 0);
                edges.push(Edge {
                    source: mux,
                    target: q,
                    input: 0,
                    original_weight: 1,
                });
            } else {
                push_edge(&mut edges, &vertex_for_net, register.data(), q, 0, 1);
            }
        }

        let before_graph = make_formal_graph(&domain, &vertices, &edges, reset.value);
        Some(Self {
            labels: vec![0; vertices.len()],
            vertices,
            edges,
            before_graph,
            vertex_for_net,
            eligible_registers,
            clock,
            clock_edge: first.edge(),
            reset,
        })
    }

    fn weights(&self) -> Vec<usize> {
        self.edges
            .iter()
            .map(|edge| {
                let weight = i64::try_from(edge.original_weight)
                    .expect("retiming edge weight fits i64")
                    + i64::from(self.labels[edge.target])
                    - i64::from(self.labels[edge.source]);
                usize::try_from(weight).expect("retiming keeps weights non-negative")
            })
            .collect()
    }

    fn topological_order(&self, weights: &[usize]) -> Option<Vec<usize>> {
        let mut indegree = vec![0usize; self.vertices.len()];
        let mut outgoing = vec![Vec::new(); self.vertices.len()];
        for (index, edge) in self.edges.iter().enumerate() {
            if weights[index] == 0 {
                indegree[edge.target] += 1;
                outgoing[edge.source].push(edge.target);
            }
        }
        let mut queue = indegree
            .iter()
            .enumerate()
            .filter_map(|(index, degree)| (*degree == 0).then_some(index))
            .collect::<VecDeque<_>>();
        let mut order = Vec::with_capacity(self.vertices.len());
        while let Some(vertex) = queue.pop_front() {
            order.push(vertex);
            for target in &outgoing[vertex] {
                indegree[*target] -= 1;
                if indegree[*target] == 0 {
                    queue.push_back(*target);
                }
            }
        }
        (order.len() == self.vertices.len()).then_some(order)
    }

    fn depths(&self) -> (usize, Vec<usize>, Vec<usize>) {
        let weights = self.weights();
        let order = self
            .topological_order(&weights)
            .expect("register-free subgraph remains acyclic");
        let mut arrival = vec![0usize; self.vertices.len()];
        let mut incoming = vec![Vec::new(); self.vertices.len()];
        let mut outgoing = vec![Vec::new(); self.vertices.len()];
        for (index, edge) in self.edges.iter().enumerate() {
            if weights[index] == 0 {
                incoming[edge.target].push(edge.source);
                outgoing[edge.source].push(edge.target);
            }
        }
        for vertex in &order {
            for source in &incoming[*vertex] {
                arrival[*vertex] =
                    arrival[*vertex].max(arrival[*source] + self.vertices[*vertex].delay);
            }
        }
        let mut tail = vec![0usize; self.vertices.len()];
        for vertex in order.iter().rev() {
            for target in &outgoing[*vertex] {
                tail[*vertex] = tail[*vertex].max(self.vertices[*target].delay + tail[*target]);
            }
        }
        let maximum = arrival.iter().copied().max().unwrap_or(0);
        (maximum, arrival, tail)
    }

    fn maximum_depth(&self) -> usize {
        self.timing_profile().0
    }

    fn timing_profile(&self) -> (usize, Vec<usize>) {
        let weights = self.weights();
        let (_, arrival, _) = self.depths();
        let stages = self
            .edges
            .iter()
            .enumerate()
            .filter_map(|(index, edge)| (weights[index] > 0).then_some(arrival[edge.source]))
            .collect::<Vec<_>>();
        (stages.iter().copied().max().unwrap_or(0), stages)
    }

    fn tails_to_registers(&self, weights: &[usize]) -> Vec<Option<usize>> {
        let order = self
            .topological_order(weights)
            .expect("register-free subgraph remains acyclic");
        let mut outgoing = vec![Vec::new(); self.vertices.len()];
        for (index, edge) in self.edges.iter().enumerate() {
            outgoing[edge.source].push((edge.target, weights[index]));
        }
        let mut tail = vec![None; self.vertices.len()];
        for vertex in order.iter().rev() {
            for (target, weight) in &outgoing[*vertex] {
                let candidate = if *weight > 0 {
                    Some(0)
                } else {
                    tail[*target].map(|depth| self.vertices[*target].delay + depth)
                };
                if let Some(candidate) = candidate {
                    tail[*vertex] = Some(tail[*vertex].unwrap_or(0).max(candidate));
                }
            }
        }
        tail
    }

    fn register_count(&self) -> usize {
        self.weights().into_iter().sum()
    }

    #[allow(clippy::too_many_lines)]
    fn retime_forward(&mut self, target: usize) -> usize {
        let initial_labels = self.labels.clone();
        let (initial_depth, initial_stages) = self.timing_profile();
        let initial_registers = self.register_count();
        let register_budget = initial_registers + initial_registers.div_ceil(6);
        let mut best_labels = initial_labels.clone();
        let mut best_score = depth_score(initial_depth, &initial_stages, initial_registers);
        let mut incoming = vec![Vec::new(); self.vertices.len()];
        let mut outgoing = vec![Vec::new(); self.vertices.len()];
        for (index, edge) in self.edges.iter().enumerate() {
            if edge.source != edge.target {
                incoming[edge.target].push(index);
                outgoing[edge.source].push(index);
            }
        }
        for _ in 0..64 {
            let weights = self.weights();
            let (_, arrival, _) = self.depths();
            let tail = self.tails_to_registers(&weights);
            let mut candidates = (0..self.vertices.len())
                .filter(|vertex| {
                    let formal = &self.vertices[*vertex].formal;
                    self.vertices[*vertex].movable
                        && !formal.is_boundary()
                        && !incoming[*vertex].is_empty()
                        && incoming[*vertex].iter().all(|edge| weights[*edge] > 0)
                        && incoming[*vertex]
                            .iter()
                            .map(|edge| arrival[self.edges[*edge].source])
                            .max()
                            .unwrap_or(0)
                            + self.vertices[*vertex].delay
                            <= target
                        && tail[*vertex]
                            .is_some_and(|tail| self.vertices[*vertex].delay + tail > target)
                })
                .collect::<Vec<_>>();
            if candidates.is_empty() {
                break;
            }
            candidates.sort_by_key(|vertex| {
                (
                    usize::MAX - (self.vertices[*vertex].delay + tail[*vertex].unwrap_or_default()),
                    outgoing[*vertex]
                        .len()
                        .saturating_sub(incoming[*vertex].len()),
                    *vertex,
                )
            });
            let mut projected_registers = self.register_count();
            candidates.retain(|vertex| {
                let delta = register_delta(outgoing[*vertex].len(), incoming[*vertex].len());
                let Some(next) = projected_registers.checked_add_signed(delta) else {
                    return false;
                };
                if next > register_budget {
                    return false;
                }
                projected_registers = next;
                true
            });
            if candidates.is_empty() {
                break;
            }
            for vertex in &candidates {
                self.labels[*vertex] -= 1;
            }
            let (depth, stages) = self.timing_profile();
            let registers = self.register_count();
            let score = depth_score(depth, &stages, registers);
            if registers <= register_budget && score < best_score {
                best_score = score;
                best_labels.clone_from(&self.labels);
            }
        }
        self.labels.clone_from(&initial_labels);
        for _ in 0..64 {
            let weights = self.weights();
            let (_, arrival, _) = self.depths();
            let mut candidates = (0..self.vertices.len())
                .filter(|vertex| {
                    let formal = &self.vertices[*vertex].formal;
                    self.vertices[*vertex].movable
                        && !formal.is_boundary()
                        && !outgoing[*vertex].is_empty()
                        && outgoing[*vertex].iter().all(|edge| weights[*edge] > 0)
                        && arrival[*vertex] > target
                })
                .collect::<Vec<_>>();
            if candidates.is_empty() {
                break;
            }
            candidates.sort_by_key(|vertex| {
                (
                    usize::MAX - arrival[*vertex],
                    incoming[*vertex]
                        .len()
                        .saturating_sub(outgoing[*vertex].len()),
                    *vertex,
                )
            });
            let mut projected_registers = self.register_count();
            candidates.retain(|vertex| {
                let delta = register_delta(incoming[*vertex].len(), outgoing[*vertex].len());
                let Some(next) = projected_registers.checked_add_signed(delta) else {
                    return false;
                };
                if next > register_budget {
                    return false;
                }
                projected_registers = next;
                true
            });
            if candidates.is_empty() {
                break;
            }
            for vertex in &candidates {
                self.labels[*vertex] += 1;
            }
            let (depth, stages) = self.timing_profile();
            let registers = self.register_count();
            let score = depth_score(depth, &stages, registers);
            if registers <= register_budget && score < best_score {
                best_score = score;
                best_labels.clone_from(&self.labels);
            }
        }
        self.labels = best_labels;
        if self.labels == initial_labels {
            0
        } else {
            self.labels
                .iter()
                .zip(initial_labels)
                .map(|(after, before)| after.abs_diff(before) as usize)
                .sum()
        }
    }

    #[allow(clippy::too_many_lines)]
    fn rebuild(&self, old: &Netlist, retimed: &RetimingGraph) -> Result<Netlist, SynthesisError> {
        let weights = self.weights();
        let order = self.topological_order(&weights).ok_or_else(|| {
            SynthesisError::Transformation("retiming produced a combinational cycle".into())
        })?;
        let mut new = Netlist::new(old.name());
        let mut vertex_net = vec![None; self.vertices.len()];

        for port in old
            .ports()
            .iter()
            .filter(|port| port.direction() == PortDirection::Input)
        {
            let width = NonZeroU32::new(
                u32::try_from(port.bits().len()).expect("port width fits the IR representation"),
            )
            .expect("ports are non-empty");
            let bits = new.add_input_port(port.name(), width);
            for (old_bit, new_bit) in port.bits().iter().zip(bits) {
                vertex_net[self.vertex_for_net[old_bit]] = Some(new_bit);
            }
        }

        for (index, register) in old.registers().iter().enumerate() {
            if !self.eligible_registers[index] {
                vertex_net[self.vertex_for_net[&register.output()]] =
                    Some(new.add_register_output(register.name()));
            }
        }

        let mut edge_registers = vec![Vec::new(); self.edges.len()];
        for (edge_index, weight) in weights.iter().enumerate() {
            for stage in 0..*weight {
                edge_registers[edge_index]
                    .push(new.add_register_output(format!("$retime_e{edge_index}_s{stage}")));
            }
        }

        let incoming = incoming_edges(&self.edges, self.vertices.len());
        let mut arithmetic_done = vec![false; old.arithmetic().len()];
        let mut comparison_done = vec![false; old.comparisons().len()];
        for vertex in order {
            if vertex_net[vertex].is_some() {
                continue;
            }
            let input = |slot: usize| -> Result<NetId, SynthesisError> {
                let edge = incoming[vertex]
                    .iter()
                    .find(|edge| self.edges[**edge].input == slot)
                    .copied()
                    .ok_or_else(|| {
                        SynthesisError::Transformation(format!(
                            "missing retiming input {slot} for vertex {vertex}"
                        ))
                    })?;
                edge_registers[edge]
                    .last()
                    .copied()
                    .or(vertex_net[self.edges[edge].source])
                    .ok_or_else(|| {
                        SynthesisError::Transformation(format!(
                            "retiming source for edge {edge} is unavailable"
                        ))
                    })
            };
            match self.vertices[vertex].kind {
                VertexKind::EnableMux => {
                    vertex_net[vertex] = Some(new.add_mux(input(0)?, input(1)?, input(2)?));
                }
                VertexKind::Net(net) => match old.nodes()[net.index() as usize].kind() {
                    NodeKind::Input(_) => unreachable!("input ports were reserved"),
                    NodeKind::Constant(value) => {
                        vertex_net[vertex] = Some(new.add_constant(*value));
                    }
                    NodeKind::And => vertex_net[vertex] = Some(new.add_and(input(0)?, input(1)?)),
                    NodeKind::Or => vertex_net[vertex] = Some(new.add_or(input(0)?, input(1)?)),
                    NodeKind::Xor => vertex_net[vertex] = Some(new.add_xor(input(0)?, input(1)?)),
                    NodeKind::Not => vertex_net[vertex] = Some(new.add_not(input(0)?)),
                    NodeKind::Mux => {
                        vertex_net[vertex] = Some(new.add_mux(input(0)?, input(1)?, input(2)?));
                    }
                    NodeKind::RegisterOutput(_) => vertex_net[vertex] = Some(input(0)?),
                    NodeKind::Output(_) => {}
                    NodeKind::MemoryOutput(_) => unreachable!("memories are excluded"),
                    NodeKind::ArithmeticOutput(_) => {
                        let cell_index = old
                            .arithmetic()
                            .iter()
                            .position(|cell| cell.outputs().contains(&net))
                            .expect("arithmetic output is connected");
                        if !arithmetic_done[cell_index] {
                            let cell = &old.arithmetic()[cell_index];
                            let width = cell.lhs().len();
                            let lhs = (0..width).map(&input).collect::<Result<Vec<_>, _>>()?;
                            let rhs = (width..width * 2)
                                .map(&input)
                                .collect::<Result<Vec<_>, _>>()?;
                            let outputs = new.add_arithmetic(cell.operation(), &lhs, &rhs)?;
                            for (old_output, output) in cell.outputs().iter().zip(outputs) {
                                vertex_net[self.vertex_for_net[old_output]] = Some(output);
                            }
                            arithmetic_done[cell_index] = true;
                        }
                    }
                    NodeKind::ComparisonOutput(_) => {
                        let cell_index = old
                            .comparisons()
                            .iter()
                            .position(|cell| cell.output() == net)
                            .expect("comparison output is connected");
                        if !comparison_done[cell_index] {
                            let cell = &old.comparisons()[cell_index];
                            let width = cell.lhs().len();
                            let lhs = (0..width).map(&input).collect::<Result<Vec<_>, _>>()?;
                            let rhs = (width..width * 2)
                                .map(&input)
                                .collect::<Result<Vec<_>, _>>()?;
                            let output = new.add_comparison(cell.operation(), &lhs, &rhs)?;
                            vertex_net[vertex] = Some(output);
                            comparison_done[cell_index] = true;
                        }
                    }
                },
            }
        }

        let new_clock = vertex_net[self.vertex_for_net[&self.clock]].expect("clock input exists");
        let new_reset =
            vertex_net[self.vertex_for_net[&self.reset.signal]].expect("reset input exists");
        for (edge_index, registers) in edge_registers.iter().enumerate() {
            let mut data = vertex_net[self.edges[edge_index].source].ok_or_else(|| {
                SynthesisError::Transformation(format!(
                    "missing source vertex for edge {edge_index}"
                ))
            })?;
            for (stage, output) in registers.iter().enumerate() {
                new.add_register(RegisterCell::new(
                    format!("$retime_e{edge_index}_s{stage}"),
                    *output,
                    data,
                    new_clock,
                    self.clock_edge,
                    None,
                    Some(ResetControl {
                        signal: new_reset,
                        value: retimed.edges()[edge_index].reset_values()[stage],
                        ..self.reset
                    }),
                ));
                data = *output;
            }
        }

        for (index, register) in old.registers().iter().enumerate() {
            if self.eligible_registers[index] {
                continue;
            }
            let mapped = |net: NetId| {
                vertex_net[self.vertex_for_net[&net]]
                    .expect("fixed register input is reconstructed")
            };
            new.add_register(RegisterCell::new(
                register.name(),
                mapped(register.output()),
                mapped(register.data()),
                mapped(register.clock()),
                register.edge(),
                register.enable().map(|enable| EnableControl {
                    signal: mapped(enable.signal),
                    active: enable.active,
                }),
                register.reset().map(|reset| ResetControl {
                    signal: mapped(reset.signal),
                    ..reset
                }),
            ));
        }

        for port in old
            .ports()
            .iter()
            .filter(|port| port.direction() == PortDirection::Output)
        {
            let sources = port
                .bits()
                .iter()
                .map(|bit| {
                    let vertex = self.vertex_for_net[bit];
                    let edge = incoming[vertex][0];
                    edge_registers[edge]
                        .last()
                        .copied()
                        .or(vertex_net[self.edges[edge].source])
                        .expect("output source is available")
                })
                .collect::<Vec<_>>();
            new.add_output_port(port.name(), &sources)?;
        }
        Ok(new)
    }
}

fn depth_score(maximum: usize, arrivals: &[usize], registers: usize) -> (usize, u128, usize) {
    let spread = arrivals
        .iter()
        .map(|depth| (*depth as u128) * (*depth as u128))
        .sum();
    (maximum, spread, registers)
}

fn register_delta(added: usize, removed: usize) -> isize {
    isize::try_from(added).expect("retiming degree fits isize")
        - isize::try_from(removed).expect("retiming degree fits isize")
}

fn push_edge(
    edges: &mut Vec<Edge>,
    vertices: &HashMap<NetId, usize>,
    source: NetId,
    target: usize,
    input: usize,
    weight: usize,
) {
    edges.push(Edge {
        source: vertices[&source],
        target,
        input,
        original_weight: weight,
    });
}

fn incoming_edges(edges: &[Edge], vertex_count: usize) -> Vec<Vec<usize>> {
    let mut incoming = vec![Vec::new(); vertex_count];
    for (index, edge) in edges.iter().enumerate() {
        incoming[edge.target].push(index);
    }
    incoming
}

fn make_formal_graph(
    domain: &RetimingDomain,
    vertices: &[Vertex],
    edges: &[Edge],
    reset_value: bool,
) -> RetimingGraph {
    let edges = edges
        .iter()
        .map(|edge| {
            RetimingEdge::new(
                edge.source,
                edge.target,
                vec![reset_value; edge.original_weight],
            )
        })
        .collect();
    RetimingGraph::new(
        domain.clone(),
        vertices
            .iter()
            .map(|vertex| vertex.formal.clone())
            .collect(),
        edges,
    )
}

fn vertex_properties(kind: &NodeKind) -> (LogicFunction, bool, usize) {
    match kind {
        NodeKind::Input(_) | NodeKind::Constant(_) => (LogicFunction::new(0, 0), true, 0),
        NodeKind::And => (LogicFunction::new(2, 0b1000), false, 1),
        NodeKind::Or => (LogicFunction::new(2, 0b1110), false, 1),
        NodeKind::Xor => (LogicFunction::new(2, 0b0110), false, 1),
        NodeKind::Not => (LogicFunction::new(1, 0b01), false, 1),
        NodeKind::Mux => (mux_function(), false, 1),
        NodeKind::RegisterOutput(_) => (LogicFunction::new(1, 0b10), false, 0),
        NodeKind::Output(_) => (LogicFunction::new(1, 0b10), true, 0),
        NodeKind::MemoryOutput(_)
        | NodeKind::ArithmeticOutput(_)
        | NodeKind::ComparisonOutput(_) => (LogicFunction::new(0, 0), true, 1),
    }
}

const fn mux_function() -> LogicFunction {
    LogicFunction::new(3, 0b1101_1000)
}

#[cfg(test)]
mod tests {
    use struo_formal::{EquivalenceStatus, TransitionSystem, prove_sequential_equivalence};
    use struo_ir::{ActiveLevel, ClockEdge, Netlist, RegisterCell, ResetControl};

    use super::TimingDrivenRetiming;
    use crate::Pass;

    #[test]
    fn balances_two_logic_levels_and_proves_the_rebuilt_machine() {
        let mut before = Netlist::new("retime_two_levels");
        let clock = before.add_input("clock");
        let reset = before.add_input("reset");
        let a = before.add_input("a");
        let b = before.add_input("b");
        let c = before.add_input("c");
        let control = ResetControl {
            signal: reset,
            active: ActiveLevel::High,
            asynchronous: true,
            value: false,
        };
        let qa = add_register(&mut before, "qa", a, clock, control);
        let qb = add_register(&mut before, "qb", b, clock, control);
        let qc = add_register(&mut before, "qc", c, clock, control);
        let first = before.add_and(qa, qb);
        let second = before.add_or(first, qc);
        let qy = add_register(&mut before, "qy", second, clock, control);
        before.add_output("y", qy);
        before.validate().unwrap();

        let gold = TransitionSystem::from_netlist(&before).unwrap();
        let mut after = before.clone();
        let report = TimingDrivenRetiming::new(1).run(&mut after).unwrap();
        let gate = TransitionSystem::from_netlist(&after).unwrap();

        assert!(
            report.message.contains("depth 2 -> 1"),
            "{}",
            report.message
        );
        assert_eq!(before.registers().len(), 4);
        assert!(after.registers().len() <= 4, "{}", report.message);
        assert_eq!(
            prove_sequential_equivalence(&gold, &gate, 3)
                .unwrap()
                .status(),
            EquivalenceStatus::Equivalent
        );
    }

    #[test]
    fn derives_reset_preimages_when_retiming_reset_to_one_state() {
        let mut before = Netlist::new("retime_reset_one");
        let clock = before.add_input("clock");
        let reset = before.add_input("reset");
        let input = before.add_input("input");
        let control = ResetControl {
            signal: reset,
            active: ActiveLevel::High,
            asynchronous: true,
            value: true,
        };
        let input_q = add_register(&mut before, "input_q", input, clock, control);
        let inverted = before.add_not(input_q);
        let output_q = add_register(&mut before, "output_q", inverted, clock, control);
        before.add_output("output", output_q);
        before.validate().unwrap();

        let gold = TransitionSystem::from_netlist(&before).unwrap();
        let mut after = before.clone();
        let report = TimingDrivenRetiming::with_net_delays_and_focus(0, [], [inverted])
            .run(&mut after)
            .unwrap();
        let gate = TransitionSystem::from_netlist(&after).unwrap();

        assert!(
            report.message.contains("certified moves"),
            "{}",
            report.message
        );
        assert!(
            after
                .registers()
                .iter()
                .any(|register| { register.reset().is_some_and(|reset| !reset.value) })
        );
        assert_eq!(
            prove_sequential_equivalence(&gold, &gate, 3)
                .unwrap()
                .status(),
            EquivalenceStatus::Equivalent
        );
    }

    fn add_register(
        netlist: &mut Netlist,
        name: &str,
        data: struo_ir::NetId,
        clock: struo_ir::NetId,
        reset: ResetControl,
    ) -> struo_ir::NetId {
        let output = netlist.add_register_output(name);
        netlist.add_register(RegisterCell::new(
            name,
            output,
            data,
            clock,
            ClockEdge::Rising,
            None,
            Some(reset),
        ));
        output
    }
}
