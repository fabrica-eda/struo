//! Technology-independent synthesis for Struo.

use std::collections::HashSet;
use std::error::Error;
use std::fmt::{self, Display, Formatter};
use std::num::NonZeroU32;

use struo_ir::{
    ActiveLevel, ClockEdge as IrClockEdge, EnableControl, NetId, Netlist, RegisterCell,
    ResetControl, ValidationError,
};
use struo_rtl::{
    BinaryOp, ClockEdge, Design, ExprId, ExprKind, Module, Polarity, PortDirection, ResetMode,
    RtlError, SignalId, UnaryOp,
};

/// Verifies hardware-semantic RTL before any information-losing lowering.
///
/// # Errors
///
/// Returns an error when the design is structurally invalid.
pub fn validate_rtl(design: &Design) -> Result<(), SynthesisError> {
    design.validate().map_err(SynthesisError::InvalidRtl)
}

/// Result of RTL lowering and target-independent logic synthesis.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SynthesisResult {
    /// Flat, bit-level logic ready for technology mapping.
    pub netlist: Netlist,
    /// Reports produced while lowering the design.
    pub reports: Vec<PassReport>,
}

/// Synthesizes the selected top module into a bit-level logic netlist.
///
/// Construction performs constant folding and structural hashing. Arithmetic
/// is bit-blasted into Boolean logic. Memories, hierarchy, and inout ports are
/// rejected until their semantics have dedicated lowering passes.
///
/// # Errors
///
/// Returns an error for invalid RTL, unsupported constructs, undriven bits,
/// combinational loops, non-constant reset values, or an invalid netlist.
pub fn synthesize(design: &Design) -> Result<SynthesisResult, SynthesisError> {
    validate_rtl(design)?;
    let module = design
        .top_module()
        .ok_or_else(|| SynthesisError::InvalidRtl(RtlError::MissingTop(design.top().into())))?;
    reject_unsupported(module)?;

    let mut lowering = Lowering::new(module);
    lowering.reserve_sources();
    lowering.index_assignments();
    lowering.connect_registers()?;
    lowering.connect_outputs()?;
    lowering.netlist.validate()?;

    let reports = vec![PassReport {
        pass: "lower-rtl",
        message: format!(
            "lowered {} expressions to {} Boolean nodes and {} registers",
            module.expressions().len(),
            lowering.netlist.nodes().len(),
            lowering.netlist.registers().len()
        ),
    }];
    Ok(SynthesisResult {
        netlist: lowering.netlist,
        reports,
    })
}

fn reject_unsupported(module: &Module) -> Result<(), SynthesisError> {
    if !module.memories().is_empty() {
        return Err(SynthesisError::Unsupported("memory inference".into()));
    }
    if !module.instances().is_empty() {
        return Err(SynthesisError::Unsupported(
            "hierarchy and black-box instances".into(),
        ));
    }
    if let Some(signal) = module
        .signals()
        .iter()
        .find(|signal| signal.direction() == Some(PortDirection::Inout))
    {
        return Err(SynthesisError::Unsupported(format!(
            "bidirectional port `{}`",
            signal.name()
        )));
    }
    Ok(())
}

type Driver = (ExprId, usize);

struct Lowering<'a> {
    module: &'a Module,
    netlist: Netlist,
    signal_bits: Vec<Vec<Option<NetId>>>,
    drivers: Vec<Vec<Option<Driver>>>,
    expression_bits: Vec<Option<Vec<NetId>>>,
    resolving: HashSet<(SignalId, usize)>,
}

impl<'a> Lowering<'a> {
    fn new(module: &'a Module) -> Self {
        let signal_bits = module
            .signals()
            .iter()
            .map(|signal| vec![None; signal.r#type().width.get() as usize])
            .collect();
        let drivers = module
            .signals()
            .iter()
            .map(|signal| vec![None; signal.r#type().width.get() as usize])
            .collect();
        Self {
            module,
            netlist: Netlist::new(module.name()),
            signal_bits,
            drivers,
            expression_bits: vec![None; module.expressions().len()],
            resolving: HashSet::new(),
        }
    }

    fn reserve_sources(&mut self) {
        for signal in self.module.signals() {
            if signal.direction() == Some(PortDirection::Input) {
                let width = NonZeroU32::new(signal.r#type().width.get())
                    .expect("RTL bit widths are non-zero");
                let nets = self.netlist.add_input_port(signal.name(), width);
                for (bit, net) in nets.into_iter().enumerate() {
                    self.signal_bits[signal.id().index() as usize][bit] = Some(net);
                }
            }
        }
        for register in self.module.registers() {
            let target = &self.module.signals()[register.target.index() as usize];
            for bit in 0..target.r#type().width.get() as usize {
                let net = self.netlist.add_register_output(bit_name(
                    &register.name,
                    target.r#type().width.get(),
                    bit,
                ));
                self.signal_bits[register.target.index() as usize][bit] = Some(net);
            }
        }
    }

    fn index_assignments(&mut self) {
        for assignment in self.module.assignments() {
            let signal = assignment.target.signal.index() as usize;
            for offset in 0..assignment.target.width.get() as usize {
                self.drivers[signal][assignment.target.lsb as usize + offset] =
                    Some((assignment.value, offset));
            }
        }
    }

    fn connect_registers(&mut self) -> Result<(), SynthesisError> {
        for register in self.module.registers() {
            let target = &self.module.signals()[register.target.index() as usize];
            let outputs = self.signal_bits[register.target.index() as usize]
                .iter()
                .map(|net| net.expect("register outputs were reserved"))
                .collect::<Vec<_>>();
            let data = self.lower_expression(register.next)?;
            let clock = self.resolve_signal_bit(register.clock, 0)?;
            let enable = register
                .enable
                .map(|enable| {
                    self.resolve_signal_bit(enable.signal, 0)
                        .map(|signal| EnableControl {
                            signal,
                            active: lower_polarity(enable.polarity),
                        })
                })
                .transpose()?;
            let reset_bits = register
                .reset
                .map(|reset| self.lower_expression(reset.value))
                .transpose()?;

            for bit in 0..target.r#type().width.get() as usize {
                let reset = match (register.reset, reset_bits.as_ref()) {
                    (Some(control), Some(bits)) => Some(ResetControl {
                        signal: self.resolve_signal_bit(control.signal, 0)?,
                        active: lower_polarity(control.polarity),
                        asynchronous: control.mode == ResetMode::Asynchronous,
                        value: self.netlist.constant_value(bits[bit]).ok_or_else(|| {
                            SynthesisError::NonConstantReset {
                                register: register.name.clone(),
                                bit,
                            }
                        })?,
                    }),
                    _ => None,
                };
                self.netlist.add_register(RegisterCell::new(
                    bit_name(&register.name, target.r#type().width.get(), bit),
                    outputs[bit],
                    data[bit],
                    clock,
                    match register.edge {
                        ClockEdge::Rising => IrClockEdge::Rising,
                        ClockEdge::Falling => IrClockEdge::Falling,
                    },
                    enable,
                    reset,
                ));
            }
        }
        Ok(())
    }

    fn connect_outputs(&mut self) -> Result<(), SynthesisError> {
        for signal in self.module.signals() {
            if signal.direction() == Some(PortDirection::Output) {
                let mut sources = Vec::with_capacity(signal.r#type().width.get() as usize);
                for bit in 0..signal.r#type().width.get() as usize {
                    sources.push(self.resolve_signal_bit(signal.id(), bit)?);
                }
                self.netlist.add_output_port(signal.name(), &sources)?;
            }
        }
        Ok(())
    }

    fn resolve_signal_bit(
        &mut self,
        signal: SignalId,
        bit: usize,
    ) -> Result<NetId, SynthesisError> {
        let signal_index = signal.index() as usize;
        if let Some(net) = self.signal_bits[signal_index][bit] {
            return Ok(net);
        }
        let signal_info = &self.module.signals()[signal_index];
        let driver =
            self.drivers[signal_index][bit].ok_or_else(|| SynthesisError::UndrivenSignalBit {
                signal: signal_info.name().into(),
                bit,
            })?;
        if !self.resolving.insert((signal, bit)) {
            return Err(SynthesisError::CombinationalLoop {
                signal: signal_info.name().into(),
                bit,
            });
        }
        let result = self.lower_expression(driver.0).map(|bits| bits[driver.1]);
        self.resolving.remove(&(signal, bit));
        let net = result?;
        self.signal_bits[signal_index][bit] = Some(net);
        Ok(net)
    }

    fn lower_expression(&mut self, id: ExprId) -> Result<Vec<NetId>, SynthesisError> {
        let index = id.index() as usize;
        if let Some(bits) = &self.expression_bits[index] {
            return Ok(bits.clone());
        }
        let expression = &self.module.expressions()[index];
        let width = expression.r#type().width.get() as usize;
        let bits = match expression.kind().clone() {
            ExprKind::Signal(slice) => (0..slice.width.get() as usize)
                .map(|offset| self.resolve_signal_bit(slice.signal, slice.lsb as usize + offset))
                .collect::<Result<_, _>>()?,
            ExprKind::Constant(value) => (0..value.width().get())
                .map(|bit| self.netlist.add_constant(value.bit(bit)))
                .collect(),
            ExprKind::Unary { op, input } => {
                let input = self.lower_expression(input)?;
                match op {
                    UnaryOp::BitNot => input
                        .into_iter()
                        .map(|net| self.netlist.add_not(net))
                        .collect(),
                    UnaryOp::LogicNot => {
                        let reduced = self.reduce_or(&input);
                        vec![self.netlist.add_not(reduced)]
                    }
                    UnaryOp::ReduceOr => vec![self.reduce_or(&input)],
                    UnaryOp::ReduceAnd => vec![self.reduce_and(&input)],
                    UnaryOp::ReduceXor => vec![self.reduce_xor(&input)],
                }
            }
            ExprKind::Binary { op, lhs, rhs } => {
                let lhs = self.lower_expression(lhs)?;
                let rhs = self.lower_expression(rhs)?;
                self.lower_binary(op, &lhs, &rhs)
            }
            ExprKind::Mux {
                condition,
                then_expr,
                else_expr,
            } => {
                let condition = self.lower_expression(condition)?[0];
                let then_bits = self.lower_expression(then_expr)?;
                let else_bits = self.lower_expression(else_expr)?;
                then_bits
                    .into_iter()
                    .zip(else_bits)
                    .map(|(then_net, else_net)| self.netlist.add_mux(condition, then_net, else_net))
                    .collect()
            }
            ExprKind::Concat(parts) => {
                let mut bits = Vec::with_capacity(width);
                for part in parts.into_iter().rev() {
                    bits.extend(self.lower_expression(part)?);
                }
                bits
            }
            ExprKind::Slice { input, lsb } => {
                let input = self.lower_expression(input)?;
                input[lsb as usize..lsb as usize + width].to_vec()
            }
        };
        self.expression_bits[index] = Some(bits.clone());
        Ok(bits)
    }

    fn lower_binary(&mut self, op: BinaryOp, lhs: &[NetId], rhs: &[NetId]) -> Vec<NetId> {
        match op {
            BinaryOp::And => self.bitwise(lhs, rhs, Netlist::add_and),
            BinaryOp::Or => self.bitwise(lhs, rhs, Netlist::add_or),
            BinaryOp::Xor => self.bitwise(lhs, rhs, Netlist::add_xor),
            BinaryOp::Add => self.add_words(lhs, rhs, false),
            BinaryOp::Sub => {
                let inverted = rhs
                    .iter()
                    .map(|&net| self.netlist.add_not(net))
                    .collect::<Vec<_>>();
                self.add_words(lhs, &inverted, true)
            }
            BinaryOp::Equal => vec![self.equal_words(lhs, rhs)],
            BinaryOp::NotEqual => {
                let equal = self.equal_words(lhs, rhs);
                vec![self.netlist.add_not(equal)]
            }
            BinaryOp::LessThanUnsigned => vec![self.less_unsigned(lhs, rhs)],
            BinaryOp::LessThanSigned => vec![self.less_signed(lhs, rhs)],
            BinaryOp::LessOrEqualUnsigned => {
                let less = self.less_unsigned(lhs, rhs);
                let equal = self.equal_words(lhs, rhs);
                vec![self.netlist.add_or(less, equal)]
            }
            BinaryOp::LessOrEqualSigned => {
                let less = self.less_signed(lhs, rhs);
                let equal = self.equal_words(lhs, rhs);
                vec![self.netlist.add_or(less, equal)]
            }
            BinaryOp::GreaterThanUnsigned => vec![self.less_unsigned(rhs, lhs)],
            BinaryOp::GreaterThanSigned => vec![self.less_signed(rhs, lhs)],
            BinaryOp::GreaterOrEqualUnsigned => {
                let less = self.less_unsigned(rhs, lhs);
                let equal = self.equal_words(lhs, rhs);
                vec![self.netlist.add_or(less, equal)]
            }
            BinaryOp::GreaterOrEqualSigned => {
                let less = self.less_signed(rhs, lhs);
                let equal = self.equal_words(lhs, rhs);
                vec![self.netlist.add_or(less, equal)]
            }
            BinaryOp::ShiftLeft => self.shift_left(lhs, rhs),
            BinaryOp::ShiftRightLogical => self.shift_right(lhs, rhs, false),
            BinaryOp::ShiftRightArithmetic => self.shift_right(lhs, rhs, true),
        }
    }

    fn equal_words(&mut self, lhs: &[NetId], rhs: &[NetId]) -> NetId {
        let mut result = self.netlist.add_constant(true);
        for (&lhs, &rhs) in lhs.iter().zip(rhs) {
            let different = self.netlist.add_xor(lhs, rhs);
            let same = self.netlist.add_not(different);
            result = self.netlist.add_and(result, same);
        }
        result
    }

    fn less_unsigned(&mut self, lhs: &[NetId], rhs: &[NetId]) -> NetId {
        let mut less = self.netlist.add_constant(false);
        let mut equal = self.netlist.add_constant(true);
        for (&lhs, &rhs) in lhs.iter().zip(rhs).rev() {
            let lhs_zero = self.netlist.add_not(lhs);
            let lhs_zero_rhs_one = self.netlist.add_and(lhs_zero, rhs);
            let first_difference_is_less = self.netlist.add_and(equal, lhs_zero_rhs_one);
            less = self.netlist.add_or(less, first_difference_is_less);
            let different = self.netlist.add_xor(lhs, rhs);
            let same = self.netlist.add_not(different);
            equal = self.netlist.add_and(equal, same);
        }
        less
    }

    fn less_signed(&mut self, lhs: &[NetId], rhs: &[NetId]) -> NetId {
        let unsigned_less = self.less_unsigned(lhs, rhs);
        let lhs_sign = lhs[lhs.len() - 1];
        let rhs_sign = rhs[rhs.len() - 1];
        let signs_differ = self.netlist.add_xor(lhs_sign, rhs_sign);
        self.netlist.add_mux(signs_differ, lhs_sign, unsigned_less)
    }

    fn shift_left(&mut self, input: &[NetId], amount: &[NetId]) -> Vec<NetId> {
        let zero = self.netlist.add_constant(false);
        let mut result = input.to_vec();
        for (stage, &select) in amount.iter().enumerate() {
            let distance = shift_distance(stage, input.len());
            let shifted = (0..input.len())
                .map(|bit| {
                    bit.checked_sub(distance)
                        .map_or(zero, |source| result[source])
                })
                .collect::<Vec<_>>();
            result = shifted
                .into_iter()
                .zip(result)
                .map(|(shifted, original)| self.netlist.add_mux(select, shifted, original))
                .collect();
        }
        result
    }

    fn shift_right(&mut self, input: &[NetId], amount: &[NetId], arithmetic: bool) -> Vec<NetId> {
        let zero = self.netlist.add_constant(false);
        let mut result = input.to_vec();
        for (stage, &select) in amount.iter().enumerate() {
            let distance = shift_distance(stage, input.len());
            let fill = if arithmetic {
                result[result.len() - 1]
            } else {
                zero
            };
            let shifted = (0..input.len())
                .map(|bit| {
                    bit.checked_add(distance)
                        .filter(|source| *source < input.len())
                        .map_or(fill, |source| result[source])
                })
                .collect::<Vec<_>>();
            result = shifted
                .into_iter()
                .zip(result)
                .map(|(shifted, original)| self.netlist.add_mux(select, shifted, original))
                .collect();
        }
        result
    }

    fn bitwise(
        &mut self,
        lhs: &[NetId],
        rhs: &[NetId],
        operation: fn(&mut Netlist, NetId, NetId) -> NetId,
    ) -> Vec<NetId> {
        lhs.iter()
            .zip(rhs)
            .map(|(&lhs, &rhs)| operation(&mut self.netlist, lhs, rhs))
            .collect()
    }

    fn add_words(&mut self, lhs: &[NetId], rhs: &[NetId], carry_in: bool) -> Vec<NetId> {
        let mut carry = self.netlist.add_constant(carry_in);
        let mut result = Vec::with_capacity(lhs.len());
        for (&lhs, &rhs) in lhs.iter().zip(rhs) {
            let propagate = self.netlist.add_xor(lhs, rhs);
            result.push(self.netlist.add_xor(propagate, carry));
            let generate = self.netlist.add_and(lhs, rhs);
            let carry_propagate = self.netlist.add_and(propagate, carry);
            carry = self.netlist.add_or(generate, carry_propagate);
        }
        result
    }

    fn reduce_or(&mut self, bits: &[NetId]) -> NetId {
        bits.iter()
            .fold(self.netlist.add_constant(false), |result, &bit| {
                self.netlist.add_or(result, bit)
            })
    }

    fn reduce_and(&mut self, bits: &[NetId]) -> NetId {
        bits.iter()
            .fold(self.netlist.add_constant(true), |result, &bit| {
                self.netlist.add_and(result, bit)
            })
    }

    fn reduce_xor(&mut self, bits: &[NetId]) -> NetId {
        bits.iter()
            .fold(self.netlist.add_constant(false), |result, &bit| {
                self.netlist.add_xor(result, bit)
            })
    }
}

fn shift_distance(stage: usize, width: usize) -> usize {
    if stage >= usize::BITS as usize {
        width
    } else {
        (1usize << stage).min(width)
    }
}

fn lower_polarity(polarity: Polarity) -> ActiveLevel {
    match polarity {
        Polarity::ActiveHigh => ActiveLevel::High,
        Polarity::ActiveLow => ActiveLevel::Low,
    }
}

fn bit_name(name: &str, width: u32, bit: usize) -> String {
    if width == 1 {
        name.into()
    } else {
        format!("{name}[{bit}]")
    }
}

/// Summary emitted after a synthesis pass runs.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PassReport {
    /// Stable pass name.
    pub pass: &'static str,
    /// Human-readable description of the result.
    pub message: String,
}

/// A transformation or analysis in the synthesis pipeline.
pub trait Pass {
    /// Returns a stable name for logs and diagnostics.
    fn name(&self) -> &'static str;

    /// Runs the pass on a design.
    ///
    /// # Errors
    ///
    /// Returns an error when the input is invalid or the transformation fails.
    fn run(&self, design: &mut Netlist) -> Result<PassReport, SynthesisError>;
}

/// An ordered sequence of synthesis passes.
#[derive(Default)]
pub struct Pipeline {
    passes: Vec<Box<dyn Pass>>,
}

impl Pipeline {
    /// Creates an empty pipeline.
    #[must_use]
    pub const fn new() -> Self {
        Self { passes: Vec::new() }
    }

    /// Appends a pass.
    pub fn push(&mut self, pass: impl Pass + 'static) {
        self.passes.push(Box::new(pass));
    }

    /// Runs each pass in order.
    ///
    /// # Errors
    ///
    /// Stops and returns the first pass failure.
    pub fn run(&self, design: &mut Netlist) -> Result<Vec<PassReport>, SynthesisError> {
        self.passes.iter().map(|pass| pass.run(design)).collect()
    }
}

/// Returns the default development pipeline.
#[must_use]
pub fn default_pipeline() -> Pipeline {
    let mut pipeline = Pipeline::new();
    pipeline.push(ValidateNetlist);
    pipeline
}

/// Verifies the structural invariants of a design.
pub struct ValidateNetlist;

impl Pass for ValidateNetlist {
    fn name(&self) -> &'static str {
        "validate"
    }

    fn run(&self, design: &mut Netlist) -> Result<PassReport, SynthesisError> {
        design.validate()?;
        Ok(PassReport {
            pass: self.name(),
            message: format!("{} nodes are structurally valid", design.nodes().len()),
        })
    }
}

/// A synthesis pipeline failure.
#[derive(Debug)]
pub enum SynthesisError {
    /// The frontend-independent RTL is structurally invalid.
    InvalidRtl(RtlError),
    /// The circuit representation is structurally invalid.
    InvalidNetlist(ValidationError),
    /// The input uses a construct that has no lowering yet.
    Unsupported(String),
    /// A signal bit has no driver.
    UndrivenSignalBit {
        /// Signal name.
        signal: String,
        /// Bit index.
        bit: usize,
    },
    /// Combinational assignments contain a feedback cycle.
    CombinationalLoop {
        /// Signal name at which the cycle was detected.
        signal: String,
        /// Bit index.
        bit: usize,
    },
    /// A register reset value did not fold to a constant.
    NonConstantReset {
        /// Register name.
        register: String,
        /// Bit index.
        bit: usize,
    },
}

impl Display for SynthesisError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRtl(error) => write!(formatter, "invalid RTL: {error}"),
            Self::InvalidNetlist(error) => write!(formatter, "invalid netlist: {error}"),
            Self::Unsupported(construct) => write!(formatter, "unsupported RTL: {construct}"),
            Self::UndrivenSignalBit { signal, bit } => {
                write!(formatter, "signal `{signal}` bit {bit} has no driver")
            }
            Self::CombinationalLoop { signal, bit } => {
                write!(
                    formatter,
                    "combinational loop at signal `{signal}` bit {bit}"
                )
            }
            Self::NonConstantReset { register, bit } => write!(
                formatter,
                "register `{register}` bit {bit} has a non-constant reset value"
            ),
        }
    }
}

impl Error for SynthesisError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidRtl(error) => Some(error),
            Self::InvalidNetlist(error) => Some(error),
            Self::Unsupported(_)
            | Self::UndrivenSignalBit { .. }
            | Self::CombinationalLoop { .. }
            | Self::NonConstantReset { .. } => None,
        }
    }
}

impl From<ValidationError> for SynthesisError {
    fn from(error: ValidationError) -> Self {
        Self::InvalidNetlist(error)
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use struo_ir::{Netlist, NodeKind};
    use struo_rtl::{
        BinaryOp, BitWidth, ClockEdge, Constant, Design, Module, Polarity, Port, PortDirection,
        Register, Reset, ResetMode, StateDomain, UnaryOp, ValueType,
    };

    use super::{default_pipeline, synthesize};

    fn bits(width: u32) -> ValueType {
        ValueType {
            width: BitWidth::new(width).unwrap(),
            signed: false,
            state: StateDomain::TwoState,
        }
    }

    fn adder_design(width: u32) -> Design {
        let mut module = Module::new("Adder");
        let lhs = module.add_port(Port {
            name: "lhs".into(),
            direction: PortDirection::Input,
            r#type: bits(width),
        });
        let rhs = module.add_port(Port {
            name: "rhs".into(),
            direction: PortDirection::Input,
            r#type: bits(width),
        });
        let output = module.add_port(Port {
            name: "sum".into(),
            direction: PortDirection::Output,
            r#type: bits(width),
        });
        let lhs = module.read(lhs).unwrap();
        let rhs = module.read(rhs).unwrap();
        let sum = module.binary(BinaryOp::Add, lhs, rhs).unwrap();
        module.assign(module.whole(output).unwrap(), sum).unwrap();
        let mut design = Design::new("Adder");
        design.add_module(module);
        design
    }

    #[test]
    fn lowers_wrapping_adder() {
        let synthesized = synthesize(&adder_design(4)).unwrap();
        assert_eq!(synthesized.netlist.registers().len(), 0);
        assert!(
            synthesized
                .netlist
                .nodes()
                .iter()
                .any(|node| matches!(node.kind(), NodeKind::Xor))
        );
        assert_eq!(
            synthesized
                .netlist
                .nodes()
                .iter()
                .filter(|node| matches!(node.kind(), NodeKind::Output(_)))
                .count(),
            4
        );
        for lhs in 0..16 {
            for rhs in 0..16 {
                let inputs = input_word("lhs", 4, lhs)
                    .into_iter()
                    .chain(input_word("rhs", 4, rhs))
                    .collect();
                let outputs = evaluate_combinational(&synthesized.netlist, &inputs);
                assert_eq!(output_word(&outputs, "sum", 4), (lhs + rhs) & 0xf);
            }
        }
    }

    #[test]
    fn lowers_counter_registers_with_async_reset() {
        let mut module = Module::new("Counter");
        let clock = module.add_port(Port {
            name: "clk".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let reset_signal = module.add_port(Port {
            name: "rst_n".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let count = module.add_port(Port {
            name: "count".into(),
            direction: PortDirection::Output,
            r#type: bits(8),
        });
        let count_value = module.read(count).unwrap();
        let one = module.constant(Constant::from_u64(BitWidth::new(8).unwrap(), 1));
        let next = module.binary(BinaryOp::Add, count_value, one).unwrap();
        let zero = module.constant(Constant::from_u64(BitWidth::new(8).unwrap(), 0));
        module
            .add_register(Register {
                name: "count_reg".into(),
                target: count,
                next,
                clock,
                edge: ClockEdge::Rising,
                enable: None,
                reset: Some(Reset {
                    signal: reset_signal,
                    mode: ResetMode::Asynchronous,
                    polarity: Polarity::ActiveLow,
                    value: zero,
                }),
            })
            .unwrap();
        let mut design = Design::new("Counter");
        design.add_module(module);

        let synthesized = synthesize(&design).unwrap();
        assert_eq!(synthesized.netlist.registers().len(), 8);
        assert!(synthesized.netlist.registers().iter().all(|register| {
            register
                .reset()
                .is_some_and(|reset| reset.asynchronous && !reset.value)
        }));
    }

    #[test]
    fn lowers_unsigned_and_signed_comparisons() {
        let operations = [
            ("eq", BinaryOp::Equal),
            ("ne", BinaryOp::NotEqual),
            ("ltu", BinaryOp::LessThanUnsigned),
            ("lts", BinaryOp::LessThanSigned),
            ("leu", BinaryOp::LessOrEqualUnsigned),
            ("les", BinaryOp::LessOrEqualSigned),
            ("gtu", BinaryOp::GreaterThanUnsigned),
            ("gts", BinaryOp::GreaterThanSigned),
            ("geu", BinaryOp::GreaterOrEqualUnsigned),
            ("ges", BinaryOp::GreaterOrEqualSigned),
        ];
        let mut module = Module::new("Comparator");
        let lhs_signal = module.add_port(Port {
            name: "lhs".into(),
            direction: PortDirection::Input,
            r#type: bits(4),
        });
        let rhs_signal = module.add_port(Port {
            name: "rhs".into(),
            direction: PortDirection::Input,
            r#type: bits(4),
        });
        let outputs = operations
            .iter()
            .map(|(name, _)| {
                module.add_port(Port {
                    name: (*name).into(),
                    direction: PortDirection::Output,
                    r#type: bits(1),
                })
            })
            .collect::<Vec<_>>();
        let lhs = module.read(lhs_signal).unwrap();
        let rhs = module.read(rhs_signal).unwrap();
        for ((_, operation), output) in operations.into_iter().zip(outputs) {
            let value = module.binary(operation, lhs, rhs).unwrap();
            let target = module.whole(output).unwrap();
            module.assign(target, value).unwrap();
        }
        let mut design = Design::new("Comparator");
        design.add_module(module);
        let synthesized = synthesize(&design).unwrap();

        for lhs in 0..16 {
            for rhs in 0..16 {
                let inputs = input_word("lhs", 4, lhs)
                    .into_iter()
                    .chain(input_word("rhs", 4, rhs))
                    .collect();
                let outputs = evaluate_combinational(&synthesized.netlist, &inputs);
                let lhs_signed = signed_nibble(lhs);
                let rhs_signed = signed_nibble(rhs);
                assert_eq!(outputs["eq"], lhs == rhs);
                assert_eq!(outputs["ne"], lhs != rhs);
                assert_eq!(outputs["ltu"], lhs < rhs);
                assert_eq!(outputs["lts"], lhs_signed < rhs_signed);
                assert_eq!(outputs["leu"], lhs <= rhs);
                assert_eq!(outputs["les"], lhs_signed <= rhs_signed);
                assert_eq!(outputs["gtu"], lhs > rhs);
                assert_eq!(outputs["gts"], lhs_signed > rhs_signed);
                assert_eq!(outputs["geu"], lhs >= rhs);
                assert_eq!(outputs["ges"], lhs_signed >= rhs_signed);
            }
        }
    }

    #[test]
    fn lowers_variable_barrel_shifts() {
        let operations = [
            ("left", BinaryOp::ShiftLeft),
            ("logical", BinaryOp::ShiftRightLogical),
            ("arithmetic", BinaryOp::ShiftRightArithmetic),
        ];
        let mut module = Module::new("Shifter");
        let value_signal = module.add_port(Port {
            name: "value".into(),
            direction: PortDirection::Input,
            r#type: bits(4),
        });
        let amount_signal = module.add_port(Port {
            name: "amount".into(),
            direction: PortDirection::Input,
            r#type: bits(3),
        });
        let outputs = operations
            .iter()
            .map(|(name, _)| {
                module.add_port(Port {
                    name: (*name).into(),
                    direction: PortDirection::Output,
                    r#type: bits(4),
                })
            })
            .collect::<Vec<_>>();
        let value = module.read(value_signal).unwrap();
        let amount = module.read(amount_signal).unwrap();
        for ((_, operation), output) in operations.into_iter().zip(outputs) {
            let shifted = module.binary(operation, value, amount).unwrap();
            let target = module.whole(output).unwrap();
            module.assign(target, shifted).unwrap();
        }
        let mut design = Design::new("Shifter");
        design.add_module(module);
        let synthesized = synthesize(&design).unwrap();

        for value in 0..16 {
            for amount in 0..8 {
                let inputs = input_word("value", 4, value)
                    .into_iter()
                    .chain(input_word("amount", 3, amount))
                    .collect();
                let outputs = evaluate_combinational(&synthesized.netlist, &inputs);
                let arithmetic = if amount >= 4 {
                    u64::from(value & 8 != 0) * 0xf
                } else {
                    (signed_nibble(value) >> amount).cast_unsigned() & 0xf
                };
                assert_eq!(output_word(&outputs, "left", 4), (value << amount) & 0xf);
                assert_eq!(output_word(&outputs, "logical", 4), value >> amount);
                assert_eq!(output_word(&outputs, "arithmetic", 4), arithmetic);
            }
        }
    }

    #[test]
    fn lowers_and_reduction() {
        let mut module = Module::new("Reduction");
        let input_signal = module.add_port(Port {
            name: "input".into(),
            direction: PortDirection::Input,
            r#type: bits(4),
        });
        let output = module.add_port(Port {
            name: "all".into(),
            direction: PortDirection::Output,
            r#type: bits(1),
        });
        let input = module.read(input_signal).unwrap();
        let reduced = module.unary(UnaryOp::ReduceAnd, input).unwrap();
        let target = module.whole(output).unwrap();
        module.assign(target, reduced).unwrap();
        let mut design = Design::new("Reduction");
        design.add_module(module);
        let synthesized = synthesize(&design).unwrap();

        for input in 0..16 {
            let inputs = input_word("input", 4, input);
            let outputs = evaluate_combinational(&synthesized.netlist, &inputs);
            assert_eq!(outputs["all"], input == 0xf);
        }
    }

    #[test]
    fn default_pipeline_validates_a_design() {
        let mut design = Netlist::new("inverter");
        let input = design.add_input("a");
        let inverted = design.add_not(input);
        design.add_output("y", inverted);

        let reports = default_pipeline().run(&mut design).unwrap();

        assert_eq!(reports.len(), 1);
        assert_eq!(reports[0].pass, "validate");
    }

    fn input_word(name: &str, width: usize, value: u64) -> HashMap<String, bool> {
        (0..width)
            .map(|bit| (format!("{name}[{bit}]"), value & (1 << bit) != 0))
            .collect()
    }

    fn signed_nibble(value: u64) -> i64 {
        if value & 8 == 0 {
            value.cast_signed()
        } else {
            value.cast_signed() - 16
        }
    }

    fn output_word(outputs: &HashMap<String, bool>, name: &str, width: usize) -> u64 {
        (0..width).fold(0, |value, bit| {
            value | (u64::from(outputs[&format!("{name}[{bit}]")]) << bit)
        })
    }

    fn evaluate_combinational(
        netlist: &Netlist,
        inputs: &HashMap<String, bool>,
    ) -> HashMap<String, bool> {
        let mut values = vec![false; netlist.nodes().len()];
        let mut outputs = HashMap::new();
        for node in netlist.nodes() {
            let value = match node.kind() {
                NodeKind::Input(name) => inputs[name],
                NodeKind::Constant(value) => *value,
                NodeKind::And => {
                    values[node.inputs()[0].index() as usize]
                        & values[node.inputs()[1].index() as usize]
                }
                NodeKind::Or => {
                    values[node.inputs()[0].index() as usize]
                        | values[node.inputs()[1].index() as usize]
                }
                NodeKind::Xor => {
                    values[node.inputs()[0].index() as usize]
                        ^ values[node.inputs()[1].index() as usize]
                }
                NodeKind::Not => !values[node.inputs()[0].index() as usize],
                NodeKind::Mux => {
                    if values[node.inputs()[0].index() as usize] {
                        values[node.inputs()[1].index() as usize]
                    } else {
                        values[node.inputs()[2].index() as usize]
                    }
                }
                NodeKind::RegisterOutput(_) => panic!("combinational test contains a register"),
                NodeKind::Output(name) => {
                    let value = values[node.inputs()[0].index() as usize];
                    outputs.insert(name.clone(), value);
                    value
                }
            };
            values[node.output().index() as usize] = value;
        }
        outputs
    }
}
