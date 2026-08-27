use std::collections::{BTreeMap, HashMap};
use std::error::Error;
use std::fmt::{self, Display, Formatter};

use struo_ir::{ActiveLevel, ArithmeticOp, ClockEdge, ComparisonOp, NetId, Netlist, NodeKind};

use crate::aig::{Aig, Literal, Variable};

/// Clock and reset mode associated with one state bit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct StateDomain {
    clock: String,
    edge: ClockEdge,
    asynchronous_reset: Option<bool>,
}

impl StateDomain {
    /// Returns the primary-input bit used as the clock.
    #[must_use]
    pub fn clock(&self) -> &str {
        &self.clock
    }

    /// Returns the active clock edge.
    #[must_use]
    pub const fn edge(&self) -> ClockEdge {
        self.edge
    }

    /// Returns whether the state bit has an asynchronous reset.
    #[must_use]
    pub const fn asynchronous_reset(&self) -> Option<bool> {
        self.asynchronous_reset
    }
}

/// One state variable and its normalized next-state function.
#[derive(Clone, Debug)]
pub struct StateBit {
    pub(crate) name: String,
    pub(crate) next: Literal,
    pub(crate) initial: Option<bool>,
    pub(crate) domain: StateDomain,
}

impl StateBit {
    /// Returns the stable state-bit name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the reset-derived initial value, or `None` for unconstrained state.
    #[must_use]
    pub const fn initial(&self) -> Option<bool> {
        self.initial
    }

    /// Returns the state domain.
    #[must_use]
    pub const fn domain(&self) -> &StateDomain {
        &self.domain
    }
}

/// A normalized, edge-sampled two-state transition system.
#[derive(Clone, Debug)]
pub struct TransitionSystem {
    name: String,
    pub(crate) aig: Aig,
    pub(crate) inputs: BTreeMap<String, Literal>,
    pub(crate) states: BTreeMap<String, StateBit>,
    pub(crate) outputs: BTreeMap<String, Literal>,
}

impl TransitionSystem {
    /// Lowers a validated Struo netlist into a native transition system.
    ///
    /// Register reset and enable controls are folded into next-state functions.
    /// Asynchronous reset is sampled at the active clock edge in this model;
    /// its asynchronous mode remains part of the state-domain identity.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid netlists, retained memories, or clocks that
    /// are not direct primary inputs.
    #[allow(clippy::too_many_lines)]
    pub fn from_netlist(design: &Netlist) -> Result<Self, TransitionError> {
        design
            .validate()
            .map_err(|error| TransitionError::InvalidNetlist(error.to_string()))?;
        if !design.memories().is_empty() {
            return Err(TransitionError::UnsupportedMemory);
        }

        let register_by_output = design
            .registers()
            .iter()
            .map(|register| (register.output(), register))
            .collect::<HashMap<_, _>>();
        let arithmetic_by_output = design
            .arithmetic()
            .iter()
            .enumerate()
            .flat_map(|(index, cell)| cell.outputs().iter().map(move |output| (*output, index)))
            .collect::<HashMap<_, _>>();
        let comparison_by_output = design
            .comparisons()
            .iter()
            .enumerate()
            .map(|(index, cell)| (cell.output(), index))
            .collect::<HashMap<_, _>>();

        let mut aig = Aig::new();
        let mut nets = vec![None; design.nodes().len()];
        let mut state_variables = HashMap::new();
        let mut inputs = BTreeMap::new();
        let mut outputs = BTreeMap::new();

        for node in design.nodes() {
            let net = node.output();
            if nets[net.index() as usize].is_some() {
                continue;
            }
            let literal = match node.kind() {
                NodeKind::Input(name) => {
                    let literal = aig.variable(Variable::Input(name.clone()));
                    inputs.insert(name.clone(), literal);
                    literal
                }
                NodeKind::Constant(value) => {
                    if *value {
                        Literal::TRUE
                    } else {
                        Literal::FALSE
                    }
                }
                NodeKind::RegisterOutput(_) => {
                    let register = register_by_output[&net];
                    let state = aig.variable(Variable::State(register.name().into()));
                    state_variables.insert(net, state);
                    if let Some(reset) = register.reset().filter(|reset| reset.asynchronous) {
                        let reset_signal = net_literal(&nets, reset.signal).map_err(|_| {
                            TransitionError::UnsupportedAsyncReset(register.name().into())
                        })?;
                        let asserted = asserted_literal(reset_signal, reset.active);
                        let reset_value = if reset.value {
                            Literal::TRUE
                        } else {
                            Literal::FALSE
                        };
                        aig.mux(asserted, reset_value, state)
                    } else {
                        state
                    }
                }
                NodeKind::And => {
                    let lhs = net_literal(&nets, node.inputs()[0])?;
                    let rhs = net_literal(&nets, node.inputs()[1])?;
                    aig.and(lhs, rhs)
                }
                NodeKind::Or => {
                    let lhs = net_literal(&nets, node.inputs()[0])?;
                    let rhs = net_literal(&nets, node.inputs()[1])?;
                    aig.or(lhs, rhs)
                }
                NodeKind::Xor => {
                    let lhs = net_literal(&nets, node.inputs()[0])?;
                    let rhs = net_literal(&nets, node.inputs()[1])?;
                    aig.xor(lhs, rhs)
                }
                NodeKind::Not => net_literal(&nets, node.inputs()[0])?.negate(),
                NodeKind::Mux => {
                    let condition = net_literal(&nets, node.inputs()[0])?;
                    let then_value = net_literal(&nets, node.inputs()[1])?;
                    let else_value = net_literal(&nets, node.inputs()[2])?;
                    aig.mux(condition, then_value, else_value)
                }
                NodeKind::Output(name) => {
                    let literal = net_literal(&nets, node.inputs()[0])?;
                    outputs.insert(name.clone(), literal);
                    literal
                }
                NodeKind::ArithmeticOutput(_) => {
                    let cell = &design.arithmetic()[arithmetic_by_output[&net]];
                    let lhs = cell
                        .lhs()
                        .iter()
                        .map(|net| net_literal(&nets, *net))
                        .collect::<Result<Vec<_>, _>>()?;
                    let rhs = cell
                        .rhs()
                        .iter()
                        .map(|net| net_literal(&nets, *net))
                        .collect::<Result<Vec<_>, _>>()?;
                    let carry_in = cell
                        .carry_in()
                        .map(|net| net_literal(&nets, net))
                        .transpose()?;
                    let result = lower_arithmetic(&mut aig, cell.operation(), &lhs, &rhs, carry_in);
                    for (output, literal) in cell.outputs().iter().zip(result) {
                        nets[output.index() as usize] = Some(literal);
                    }
                    nets[net.index() as usize].ok_or(TransitionError::UnloweredNet(net))?
                }
                NodeKind::ComparisonOutput(_) => {
                    let cell = &design.comparisons()[comparison_by_output[&net]];
                    let lhs = cell
                        .lhs()
                        .iter()
                        .map(|net| net_literal(&nets, *net))
                        .collect::<Result<Vec<_>, _>>()?;
                    let rhs = cell
                        .rhs()
                        .iter()
                        .map(|net| net_literal(&nets, *net))
                        .collect::<Result<Vec<_>, _>>()?;
                    lower_comparison(&mut aig, cell.operation(), &lhs, &rhs)
                }
                NodeKind::MemoryOutput(_) => return Err(TransitionError::UnsupportedMemory),
            };
            nets[net.index() as usize] = Some(literal);
        }

        let mut states = BTreeMap::new();
        for register in design.registers() {
            let current = state_variables[&register.output()];
            let data = net_literal(&nets, register.data())?;
            let clock = direct_input_name(design, register.clock())
                .ok_or_else(|| TransitionError::UnsupportedClock(register.name().into()))?;
            let mut next = if let Some(enable) = register.enable() {
                let asserted = asserted_literal(net_literal(&nets, enable.signal)?, enable.active);
                aig.mux(asserted, data, current)
            } else {
                data
            };
            if let Some(reset) = register.reset() {
                let asserted = asserted_literal(net_literal(&nets, reset.signal)?, reset.active);
                let reset_value = if reset.value {
                    Literal::TRUE
                } else {
                    Literal::FALSE
                };
                next = aig.mux(asserted, reset_value, next);
            }
            states.insert(
                register.name().into(),
                StateBit {
                    name: register.name().into(),
                    next,
                    initial: register.reset().map(|reset| reset.value),
                    domain: StateDomain {
                        clock,
                        edge: register.edge(),
                        asynchronous_reset: register.reset().map(|reset| reset.asynchronous),
                    },
                },
            );
        }

        Ok(Self {
            name: design.name().into(),
            aig,
            inputs,
            states,
            outputs,
        })
    }

    /// Returns the design name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns primary-input bit names in stable order.
    pub fn input_names(&self) -> impl Iterator<Item = &str> {
        self.inputs.keys().map(String::as_str)
    }

    /// Returns state bits in stable name order.
    pub fn states(&self) -> impl Iterator<Item = &StateBit> {
        self.states.values()
    }

    /// Returns primary-output bit names in stable order.
    pub fn output_names(&self) -> impl Iterator<Item = &str> {
        self.outputs.keys().map(String::as_str)
    }
}

fn net_literal(nets: &[Option<Literal>], net: NetId) -> Result<Literal, TransitionError> {
    nets.get(net.index() as usize)
        .copied()
        .flatten()
        .ok_or(TransitionError::UnloweredNet(net))
}

fn direct_input_name(design: &Netlist, net: NetId) -> Option<String> {
    let node = design.nodes().get(net.index() as usize)?;
    match node.kind() {
        NodeKind::Input(name) => Some(name.clone()),
        _ => None,
    }
}

fn asserted_literal(literal: Literal, active: ActiveLevel) -> Literal {
    if active == ActiveLevel::High {
        literal
    } else {
        literal.negate()
    }
}

fn lower_arithmetic(
    aig: &mut Aig,
    operation: ArithmeticOp,
    lhs: &[Literal],
    rhs: &[Literal],
    carry_in: Option<Literal>,
) -> Vec<Literal> {
    let mut carry = if operation == ArithmeticOp::Subtract {
        Literal::TRUE
    } else {
        carry_in.unwrap_or(Literal::FALSE)
    };
    lhs.iter()
        .zip(rhs)
        .map(|(lhs, rhs)| {
            let rhs = if operation == ArithmeticOp::Subtract {
                rhs.negate()
            } else {
                *rhs
            };
            let operands = aig.xor(*lhs, rhs);
            let sum = aig.xor(operands, carry);
            let lhs_rhs = aig.and(*lhs, rhs);
            let lhs_carry = aig.and(*lhs, carry);
            let rhs_carry = aig.and(rhs, carry);
            let partial_carry = aig.or(lhs_rhs, lhs_carry);
            carry = aig.or(partial_carry, rhs_carry);
            sum
        })
        .collect()
}

fn lower_comparison(
    aig: &mut Aig,
    operation: ComparisonOp,
    lhs: &[Literal],
    rhs: &[Literal],
) -> Literal {
    let mut less = Literal::FALSE;
    let mut equal = Literal::TRUE;
    let width = lhs.len();
    for (index, (lhs, rhs)) in lhs.iter().zip(rhs).enumerate() {
        let signed_bit = operation.is_signed() && index + 1 == width;
        let lhs = if signed_bit { lhs.negate() } else { *lhs };
        let rhs = if signed_bit { rhs.negate() } else { *rhs };
        let bit_less = aig.and(lhs.negate(), rhs);
        let bit_equal = aig.xor(lhs, rhs).negate();
        let lower_less = aig.and(bit_equal, less);
        less = aig.or(bit_less, lower_less);
        equal = aig.and(equal, bit_equal);
    }
    if operation.includes_equal() {
        aig.or(less, equal)
    } else {
        less
    }
}

/// Failure while constructing a native transition system.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TransitionError {
    /// The source netlist failed its structural validation.
    InvalidNetlist(String),
    /// Retained memories are not yet represented in the bit transition system.
    UnsupportedMemory,
    /// A register clock was not a direct primary input.
    UnsupportedClock(String),
    /// An asynchronous reset signal was not available before its register output.
    UnsupportedAsyncReset(String),
    /// A dependency was not available in topological order.
    UnloweredNet(NetId),
}

impl Display for TransitionError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidNetlist(reason) => write!(formatter, "invalid netlist: {reason}"),
            Self::UnsupportedMemory => {
                formatter.write_str("retained memories are not supported by native formal yet")
            }
            Self::UnsupportedClock(register) => write!(
                formatter,
                "register {register} does not use a direct primary-input clock"
            ),
            Self::UnsupportedAsyncReset(register) => write!(
                formatter,
                "register {register} has a non-topological asynchronous reset"
            ),
            Self::UnloweredNet(net) => write!(formatter, "net {net} was not lowered in order"),
        }
    }
}

impl Error for TransitionError {}

#[cfg(test)]
mod tests {
    use std::num::NonZeroU32;

    use struo_ir::{ArithmeticOp, ComparisonOp, Netlist};

    use super::TransitionSystem;
    use crate::aig::Literal;
    use crate::sat::solve;

    #[test]
    fn bit_blasts_retained_arithmetic_without_external_ir() {
        let mut design = Netlist::new("adder");
        let lhs = design.add_input("lhs");
        let rhs = design.add_input("rhs");
        let sum = design
            .add_arithmetic(ArithmeticOp::Add, &[lhs], &[rhs])
            .unwrap();
        design.add_output("sum", sum[0]);

        let system = TransitionSystem::from_netlist(&design).unwrap();

        assert_eq!(system.input_names().collect::<Vec<_>>(), ["lhs", "rhs"]);
        assert_eq!(system.output_names().collect::<Vec<_>>(), ["sum"]);
    }

    #[test]
    fn bit_blasted_words_match_exhaustive_four_bit_arithmetic() {
        let mut design = Netlist::new("word_operations");
        let lhs = design.add_input_port("lhs", NonZeroU32::new(4).unwrap());
        let rhs = design.add_input_port("rhs", NonZeroU32::new(4).unwrap());
        let carry = design.add_input("carry");
        let sum = design
            .add_arithmetic(ArithmeticOp::Add, &lhs, &rhs)
            .unwrap();
        let sum_with_carry = design.add_arithmetic_with_carry(&lhs, &rhs, carry).unwrap();
        let difference = design
            .add_arithmetic(ArithmeticOp::Subtract, &lhs, &rhs)
            .unwrap();
        let unsigned_less = design
            .add_comparison(ComparisonOp::LessThanUnsigned, &lhs, &rhs)
            .unwrap();
        let signed_less = design
            .add_comparison(ComparisonOp::LessThanSigned, &lhs, &rhs)
            .unwrap();
        design.add_output_port("sum", &sum).unwrap();
        design
            .add_output_port("sum_with_carry", &sum_with_carry)
            .unwrap();
        design.add_output_port("difference", &difference).unwrap();
        design.add_output("unsigned_less", unsigned_less);
        design.add_output("signed_less", signed_less);
        let system = TransitionSystem::from_netlist(&design).unwrap();

        for lhs in 0_u8..16 {
            for rhs in 0_u8..16 {
                for carry in 0_u8..2 {
                    let expected = [
                        ("sum", lhs.wrapping_add(rhs) & 0xf),
                        (
                            "sum_with_carry",
                            lhs.wrapping_add(rhs).wrapping_add(carry) & 0xf,
                        ),
                        ("difference", lhs.wrapping_sub(rhs) & 0xf),
                    ];
                    let mut mismatch = Literal::FALSE;
                    let mut aig = system.aig.clone();
                    for (port, value) in expected {
                        for bit in 0..4 {
                            let name = format!("{port}[{bit}]");
                            let actual = system.outputs[&name];
                            let expected = value & (1 << bit) != 0;
                            let differs = if expected { actual.negate() } else { actual };
                            mismatch = aig.or(mismatch, differs);
                        }
                    }
                    for (name, expected) in [
                        ("unsigned_less", lhs < rhs),
                        ("signed_less", signed_nibble(lhs) < signed_nibble(rhs)),
                    ] {
                        let actual = system.outputs[name];
                        let differs = if expected { actual.negate() } else { actual };
                        mismatch = aig.or(mismatch, differs);
                    }
                    let mut assumptions = Vec::with_capacity(10);
                    for (port, value) in [("lhs", lhs), ("rhs", rhs)] {
                        assumptions.extend((0..4).map(|bit| {
                            let literal = system.inputs[&format!("{port}[{bit}]")];
                            if value & (1 << bit) != 0 {
                                literal
                            } else {
                                literal.negate()
                            }
                        }));
                    }
                    assumptions.push(if carry != 0 {
                        system.inputs["carry"]
                    } else {
                        system.inputs["carry"].negate()
                    });
                    assumptions.push(mismatch);

                    assert!(
                        solve(&aig, &assumptions).is_none(),
                        "word lowering mismatch for lhs={lhs}, rhs={rhs}, carry={carry}"
                    );
                }
            }
        }
    }

    const fn signed_nibble(value: u8) -> i8 {
        if value & 8 == 0 {
            value.cast_signed()
        } else {
            value.cast_signed() - 16
        }
    }
}
