use std::collections::{BTreeMap, HashMap};
use std::error::Error;
use std::fmt::{self, Display, Formatter};

use struo_ir::ClockEdge;

use crate::aig::{Aig, Literal, Node, Variable};
use crate::sat::solve;
use crate::transition::TransitionSystem;

/// One cycle of primary-input values from a sequential counterexample.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct InputFrame {
    cycle: usize,
    inputs: BTreeMap<String, bool>,
}

impl InputFrame {
    /// Returns the zero-based cycle index.
    #[must_use]
    pub const fn cycle(&self) -> usize {
        self.cycle
    }

    /// Returns primary-input bit values by stable name.
    #[must_use]
    pub const fn inputs(&self) -> &BTreeMap<String, bool> {
        &self.inputs
    }
}

/// A concrete input trace that makes at least one output differ.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Counterexample {
    frames: Vec<InputFrame>,
    first_mismatch: usize,
    differing_outputs: Vec<String>,
}

impl Counterexample {
    /// Returns input values through the first mismatching cycle.
    #[must_use]
    pub fn frames(&self) -> &[InputFrame] {
        &self.frames
    }

    /// Returns the first cycle whose observable outputs differ.
    #[must_use]
    pub const fn first_mismatch(&self) -> usize {
        self.first_mismatch
    }

    /// Returns output bits that differ in that cycle.
    #[must_use]
    pub fn differing_outputs(&self) -> &[String] {
        &self.differing_outputs
    }
}

/// Proof outcome for two transition systems.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EquivalenceStatus {
    /// Base checking and k-induction proved unbounded output equivalence.
    Equivalent,
    /// A reachable output mismatch was found.
    NotEquivalent,
    /// No reachable mismatch was found, but induction did not close.
    Inconclusive,
}

/// Result of native sequential-equivalence checking.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EquivalenceResult {
    status: EquivalenceStatus,
    depth: usize,
    counterexample: Option<Counterexample>,
}

impl EquivalenceResult {
    /// Returns the proof outcome.
    #[must_use]
    pub const fn status(&self) -> EquivalenceStatus {
        self.status
    }

    /// Returns the induction depth proved or attempted.
    #[must_use]
    pub const fn depth(&self) -> usize {
        self.depth
    }

    /// Returns a concrete trace for a non-equivalent result.
    #[must_use]
    pub const fn counterexample(&self) -> Option<&Counterexample> {
        self.counterexample.as_ref()
    }
}

/// Proves observable sequential equivalence using complete base checks followed
/// by k-induction up to `max_depth`.
///
/// A successful result is unbounded: the base query proves the property from
/// reset through depth `k`, and the induction query proves that any `k`
/// consecutive equivalent cycles imply the next one. An inconclusive result is
/// never reported as equivalent.
///
/// # Errors
///
/// Returns an error for mismatched interfaces, clock domains, unknown initial
/// state, or a zero depth limit.
pub fn prove_sequential_equivalence(
    gold: &TransitionSystem,
    gate: &TransitionSystem,
    max_depth: usize,
) -> Result<EquivalenceResult, EquivalenceError> {
    validate_problem(gold, gate, max_depth)?;

    for depth in 1..=max_depth {
        if let Some(counterexample) = base_counterexample(gold, gate, depth) {
            return Ok(EquivalenceResult {
                status: EquivalenceStatus::NotEquivalent,
                depth,
                counterexample: Some(counterexample),
            });
        }
        if (base_preserves_named_state(gold, gate, depth)
            && induction_closes(gold, gate, depth, true))
            || induction_closes(gold, gate, depth, false)
        {
            return Ok(EquivalenceResult {
                status: EquivalenceStatus::Equivalent,
                depth,
                counterexample: None,
            });
        }
    }

    Ok(EquivalenceResult {
        status: EquivalenceStatus::Inconclusive,
        depth: max_depth,
        counterexample: None,
    })
}

fn validate_problem(
    gold: &TransitionSystem,
    gate: &TransitionSystem,
    max_depth: usize,
) -> Result<(), EquivalenceError> {
    if max_depth == 0 {
        return Err(EquivalenceError::ZeroDepth);
    }
    let gold_inputs = gold.input_names().collect::<Vec<_>>();
    let gate_inputs = gate.input_names().collect::<Vec<_>>();
    if gold_inputs != gate_inputs {
        return Err(EquivalenceError::InputInterface {
            gold: gold_inputs.into_iter().map(str::to_owned).collect(),
            gate: gate_inputs.into_iter().map(str::to_owned).collect(),
        });
    }
    let gold_outputs = gold.output_names().collect::<Vec<_>>();
    let gate_outputs = gate.output_names().collect::<Vec<_>>();
    if gold_outputs != gate_outputs {
        return Err(EquivalenceError::OutputInterface {
            gold: gold_outputs.into_iter().map(str::to_owned).collect(),
            gate: gate_outputs.into_iter().map(str::to_owned).collect(),
        });
    }
    let gold_domain = common_domain(gold)?;
    let gate_domain = common_domain(gate)?;
    if gold_domain != gate_domain {
        return Err(EquivalenceError::ClockDomain {
            gold: gold_domain,
            gate: gate_domain,
        });
    }
    for state in gold.states().chain(gate.states()) {
        if state.initial().is_none() {
            return Err(EquivalenceError::UnknownInitialState(state.name().into()));
        }
    }
    Ok(())
}

fn common_domain(
    system: &TransitionSystem,
) -> Result<Option<(String, ClockEdge)>, EquivalenceError> {
    let mut domain = None;
    for state in system.states() {
        let candidate = (state.domain().clock().to_owned(), state.domain().edge());
        if domain.as_ref().is_some_and(|domain| *domain != candidate) {
            return Err(EquivalenceError::MultipleClockDomains(system.name().into()));
        }
        domain = Some(candidate);
    }
    Ok(domain)
}

#[derive(Debug)]
struct FrameComparison {
    inputs: BTreeMap<String, Literal>,
    outputs: BTreeMap<String, (Literal, Literal)>,
    mismatch: Literal,
}

fn base_counterexample(
    gold: &TransitionSystem,
    gate: &TransitionSystem,
    depth: usize,
) -> Option<Counterexample> {
    let mut proof = Aig::new();
    let mut gold_state = initial_state(gold);
    let mut gate_state = initial_state(gate);
    let mut comparisons = Vec::with_capacity(depth + 1);
    let mut any_mismatch = Literal::FALSE;

    for cycle in 0..=depth {
        let inputs = proof_inputs(&mut proof, gold, cycle);
        let (gold_outputs, gold_next) = import_frame(gold, &mut proof, &inputs, &gold_state);
        let (gate_outputs, gate_next) = import_frame(gate, &mut proof, &inputs, &gate_state);
        let comparison = compare_outputs(&mut proof, inputs, gold_outputs, &gate_outputs);
        any_mismatch = proof.or(any_mismatch, comparison.mismatch);
        comparisons.push(comparison);
        gold_state = gold_next;
        gate_state = gate_next;
    }

    let model = solve(&proof, &[any_mismatch])?;
    let (first_mismatch, comparison) = comparisons
        .iter()
        .enumerate()
        .find(|(_, comparison)| Aig::evaluate(comparison.mismatch, &model))
        .expect("the satisfying model asserts at least one frame mismatch");
    let frames = comparisons
        .iter()
        .take(first_mismatch + 1)
        .enumerate()
        .map(|(cycle, comparison)| InputFrame {
            cycle,
            inputs: comparison
                .inputs
                .iter()
                .map(|(name, literal)| (name.clone(), Aig::evaluate(*literal, &model)))
                .collect(),
        })
        .collect();
    let differing_outputs = comparison
        .outputs
        .iter()
        .filter(|(_, (gold, gate))| Aig::evaluate(*gold, &model) != Aig::evaluate(*gate, &model))
        .map(|(name, _)| name.clone())
        .collect();
    Some(Counterexample {
        frames,
        first_mismatch,
        differing_outputs,
    })
}

fn base_preserves_named_state(
    gold: &TransitionSystem,
    gate: &TransitionSystem,
    depth: usize,
) -> bool {
    let mut proof = Aig::new();
    let mut gold_state = initial_state(gold);
    let mut gate_state = initial_state(gate);
    let mut any_mismatch = Literal::FALSE;

    for cycle in 0..=depth {
        let state_mismatch = compare_named_state(&mut proof, &gold_state, &gate_state);
        any_mismatch = proof.or(any_mismatch, state_mismatch);
        let inputs = proof_inputs(&mut proof, gold, cycle);
        let (_, gold_next) = import_frame(gold, &mut proof, &inputs, &gold_state);
        let (_, gate_next) = import_frame(gate, &mut proof, &inputs, &gate_state);
        gold_state = gold_next;
        gate_state = gate_next;
    }

    solve(&proof, &[any_mismatch]).is_none()
}

fn induction_closes(
    gold: &TransitionSystem,
    gate: &TransitionSystem,
    depth: usize,
    include_named_state: bool,
) -> bool {
    let mut proof = Aig::new();
    let (mut gold_state, mut gate_state) =
        free_product_state(&mut proof, gold, gate, include_named_state);
    let mut assumptions = Vec::with_capacity(depth + 1);

    for cycle in 0..=depth {
        let inputs = proof_inputs(&mut proof, gold, cycle);
        let (gold_outputs, gold_next) = import_frame(gold, &mut proof, &inputs, &gold_state);
        let (gate_outputs, gate_next) = import_frame(gate, &mut proof, &inputs, &gate_state);
        let comparison = compare_outputs(&mut proof, inputs, gold_outputs, &gate_outputs);
        let mismatch = if include_named_state {
            let state_mismatch = compare_named_state(&mut proof, &gold_state, &gate_state);
            proof.or(comparison.mismatch, state_mismatch)
        } else {
            comparison.mismatch
        };
        if cycle == depth {
            assumptions.push(mismatch);
        } else {
            assumptions.push(mismatch.negate());
        }
        gold_state = gold_next;
        gate_state = gate_next;
    }

    solve(&proof, &assumptions).is_none()
}

fn compare_named_state(
    proof: &mut Aig,
    gold: &BTreeMap<String, Literal>,
    gate: &BTreeMap<String, Literal>,
) -> Literal {
    gold.iter()
        .filter_map(|(name, gold)| gate.get(name).map(|gate| (*gold, *gate)))
        .fold(Literal::FALSE, |mismatch, (gold, gate)| {
            let differs = proof.xor(gold, gate);
            proof.or(mismatch, differs)
        })
}

fn initial_state(system: &TransitionSystem) -> BTreeMap<String, Literal> {
    system
        .states
        .iter()
        .map(|(name, state)| {
            let literal = if state.initial.expect("initial state was validated") {
                Literal::TRUE
            } else {
                Literal::FALSE
            };
            (name.clone(), literal)
        })
        .collect()
}

fn free_product_state(
    proof: &mut Aig,
    gold: &TransitionSystem,
    gate: &TransitionSystem,
    share_named_state: bool,
) -> (BTreeMap<String, Literal>, BTreeMap<String, Literal>) {
    let gold_state = gold
        .states
        .keys()
        .map(|name| {
            let scope = if share_named_state && gate.states.contains_key(name) {
                "matched"
            } else {
                "gold"
            };
            (
                name.clone(),
                proof.variable(Variable::Proof(format!("{scope}:state:{name}"))),
            )
        })
        .collect();
    let gate_state = gate
        .states
        .keys()
        .map(|name| {
            let scope = if share_named_state && gold.states.contains_key(name) {
                "matched"
            } else {
                "gate"
            };
            (
                name.clone(),
                proof.variable(Variable::Proof(format!("{scope}:state:{name}"))),
            )
        })
        .collect();
    (gold_state, gate_state)
}

fn proof_inputs(
    proof: &mut Aig,
    system: &TransitionSystem,
    cycle: usize,
) -> BTreeMap<String, Literal> {
    system
        .inputs
        .keys()
        .map(|name| {
            (
                name.clone(),
                proof.variable(Variable::Proof(format!("input:{cycle}:{name}"))),
            )
        })
        .collect()
}

fn import_frame(
    system: &TransitionSystem,
    proof: &mut Aig,
    inputs: &BTreeMap<String, Literal>,
    states: &BTreeMap<String, Literal>,
) -> (BTreeMap<String, Literal>, BTreeMap<String, Literal>) {
    let variables = inputs
        .iter()
        .map(|(name, literal)| (Variable::Input(name.clone()), *literal))
        .chain(
            states
                .iter()
                .map(|(name, literal)| (Variable::State(name.clone()), *literal)),
        )
        .collect::<HashMap<_, _>>();
    let imported = import_aig(&system.aig, proof, &variables);
    let outputs = system
        .outputs
        .iter()
        .map(|(name, literal)| (name.clone(), remap_literal(*literal, &imported)))
        .collect();
    let next = system
        .states
        .iter()
        .map(|(name, state)| (name.clone(), remap_literal(state.next, &imported)))
        .collect();
    (outputs, next)
}

fn import_aig(
    source: &Aig,
    target: &mut Aig,
    variables: &HashMap<Variable, Literal>,
) -> Vec<Literal> {
    let mut imported = vec![Literal::FALSE; source.nodes().len()];
    for (node_index, node) in source.nodes().iter().enumerate().skip(1) {
        imported[node_index] = match node {
            Node::Constant => unreachable!("only AIG node zero is constant"),
            Node::Variable(variable) => variables[variable],
            Node::And(lhs, rhs) => {
                let lhs = remap_literal(*lhs, &imported);
                let rhs = remap_literal(*rhs, &imported);
                target.and(lhs, rhs)
            }
        };
    }
    imported
}

fn remap_literal(literal: Literal, imported: &[Literal]) -> Literal {
    let mapped = imported[literal.node()];
    if literal.inverted() {
        mapped.negate()
    } else {
        mapped
    }
}

fn compare_outputs(
    proof: &mut Aig,
    inputs: BTreeMap<String, Literal>,
    gold: BTreeMap<String, Literal>,
    gate: &BTreeMap<String, Literal>,
) -> FrameComparison {
    let outputs = gold
        .into_iter()
        .map(|(name, gold)| {
            let gate = gate[&name];
            (name, (gold, gate))
        })
        .collect::<BTreeMap<_, _>>();
    let mismatch = outputs
        .values()
        .fold(Literal::FALSE, |mismatch, (gold, gate)| {
            let differs = proof.xor(*gold, *gate);
            proof.or(mismatch, differs)
        });
    FrameComparison {
        inputs,
        outputs,
        mismatch,
    }
}

/// Invalid native sequential-equivalence problem.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EquivalenceError {
    /// At least one input bit differs between designs.
    InputInterface {
        /// Gold input names.
        gold: Vec<String>,
        /// Gate input names.
        gate: Vec<String>,
    },
    /// At least one output bit differs between designs.
    OutputInterface {
        /// Gold output names.
        gold: Vec<String>,
        /// Gate output names.
        gate: Vec<String>,
    },
    /// One design contains more than one active clock domain.
    MultipleClockDomains(String),
    /// The two designs use different active clock domains.
    ClockDomain {
        /// Gold clock and edge, if state exists.
        gold: Option<(String, ClockEdge)>,
        /// Gate clock and edge, if state exists.
        gate: Option<(String, ClockEdge)>,
    },
    /// A state bit has no reset-derived initial value.
    UnknownInitialState(String),
    /// Induction depth must be positive.
    ZeroDepth,
}

impl Display for EquivalenceError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::InputInterface { gold, gate } => {
                write!(
                    formatter,
                    "input interfaces differ: gold={gold:?}, gate={gate:?}"
                )
            }
            Self::OutputInterface { gold, gate } => write!(
                formatter,
                "output interfaces differ: gold={gold:?}, gate={gate:?}"
            ),
            Self::MultipleClockDomains(design) => {
                write!(formatter, "design {design} contains multiple clock domains")
            }
            Self::ClockDomain { gold, gate } => {
                write!(
                    formatter,
                    "clock domains differ: gold={gold:?}, gate={gate:?}"
                )
            }
            Self::UnknownInitialState(state) => {
                write!(formatter, "state bit {state} has no known initial value")
            }
            Self::ZeroDepth => formatter.write_str("induction depth must be positive"),
        }
    }
}

impl Error for EquivalenceError {}

#[cfg(test)]
mod tests {
    use struo_ir::{ActiveLevel, ClockEdge, Netlist, RegisterCell, ResetControl};

    use super::{EquivalenceStatus, prove_sequential_equivalence};
    use crate::TransitionSystem;

    #[test]
    fn proves_equivalent_differently_structured_counters() {
        let gold = counter(false);
        let gate = counter(true);

        let result = prove_sequential_equivalence(&gold, &gate, 3).unwrap();

        assert_eq!(result.status(), EquivalenceStatus::Equivalent);
    }

    #[test]
    fn returns_a_replayable_counterexample() {
        let gold = counter(false);
        let mut gate_netlist = Netlist::new("bad_counter");
        let clock = gate_netlist.add_input("clock");
        let reset = gate_netlist.add_input("reset");
        let enable = gate_netlist.add_input("enable");
        let q = gate_netlist.add_register_output("q");
        gate_netlist.add_register(RegisterCell::new(
            "q",
            q,
            enable,
            clock,
            ClockEdge::Rising,
            None,
            Some(reset_control(reset)),
        ));
        gate_netlist.add_output("q_out", q);
        let gate = TransitionSystem::from_netlist(&gate_netlist).unwrap();

        let result = prove_sequential_equivalence(&gold, &gate, 3).unwrap();
        let counterexample = result.counterexample().unwrap();

        assert_eq!(result.status(), EquivalenceStatus::NotEquivalent);
        assert!(!counterexample.frames().is_empty());
        assert_eq!(counterexample.differing_outputs(), ["q_out"]);
    }

    #[test]
    fn discards_an_invalid_same_name_state_hint() {
        let gold = counter(false);
        let mut gate_netlist = Netlist::new("complement_counter");
        let clock = gate_netlist.add_input("clock");
        let reset = gate_netlist.add_input("reset");
        let enable = gate_netlist.add_input("enable");
        let q = gate_netlist.add_register_output("q");
        let toggled = gate_netlist.add_not(q);
        gate_netlist.add_register(RegisterCell::new(
            "q",
            q,
            toggled,
            clock,
            ClockEdge::Rising,
            Some(struo_ir::EnableControl {
                signal: enable,
                active: ActiveLevel::High,
            }),
            Some(ResetControl {
                value: true,
                ..reset_control(reset)
            }),
        ));
        let output = gate_netlist.add_not(q);
        gate_netlist.add_output("q_out", output);
        let gate = TransitionSystem::from_netlist(&gate_netlist).unwrap();

        let result = prove_sequential_equivalence(&gold, &gate, 3).unwrap();

        assert_eq!(result.status(), EquivalenceStatus::Equivalent);
    }

    fn counter(explicit_mux: bool) -> TransitionSystem {
        let mut design = Netlist::new("counter");
        let clock = design.add_input("clock");
        let reset = design.add_input("reset");
        let enable = design.add_input("enable");
        let q = design.add_register_output("q");
        let toggled = design.add_not(q);
        let data = if explicit_mux {
            design.add_mux(enable, toggled, q)
        } else {
            toggled
        };
        design.add_register(RegisterCell::new(
            "q",
            q,
            data,
            clock,
            ClockEdge::Rising,
            (!explicit_mux).then_some(struo_ir::EnableControl {
                signal: enable,
                active: ActiveLevel::High,
            }),
            Some(reset_control(reset)),
        ));
        design.add_output("q_out", q);
        TransitionSystem::from_netlist(&design).unwrap()
    }

    const fn reset_control(signal: struo_ir::NetId) -> ResetControl {
        ResetControl {
            signal,
            active: ActiveLevel::High,
            asynchronous: true,
            value: false,
        }
    }
}
