//! Frontend-independent RTL that preserves hardware intent.
//!
//! This representation deliberately mirrors the synthesizable portion of a
//! source-neutral frontend artifact: explicit-width expressions, assignments,
//! and edge-triggered storage. Analyzer-owned identities must not cross this
//! boundary.

use std::collections::HashSet;
use std::error::Error;
use std::fmt::{self, Display, Formatter};
use std::num::NonZeroU32;

/// A non-zero packed bit width.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct BitWidth(NonZeroU32);

impl BitWidth {
    /// Creates a valid width.
    ///
    /// # Errors
    ///
    /// Returns an error for zero-width values.
    pub fn new(bits: u32) -> Result<Self, RtlError> {
        NonZeroU32::new(bits).map(Self).ok_or(RtlError::ZeroWidth)
    }

    /// Returns the width in bits.
    #[must_use]
    pub const fn get(self) -> u32 {
        self.0.get()
    }
}

impl Display for BitWidth {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        self.get().fmt(formatter)
    }
}

/// The value domain expected before synthesis.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum StateDomain {
    /// Values contain only zero and one.
    TwoState,
    /// Values may also contain X and Z during RTL simulation.
    FourState,
}

/// Packed value type.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ValueType {
    /// Packed width.
    pub width: BitWidth,
    /// Whether arithmetic operations use signed interpretation.
    pub signed: bool,
    /// Simulation value domain.
    pub state: StateDomain,
}

/// Direction of a module port.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum PortDirection {
    /// Driven by the parent module.
    Input,
    /// Driven by this module.
    Output,
    /// Bidirectional physical interface.
    Inout,
}

/// A module boundary signal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Port {
    /// Source-level port name.
    pub name: String,
    /// Port direction.
    pub direction: PortDirection,
    /// Packed value type.
    pub r#type: ValueType,
}

/// Stable identity of a module-local signal.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct SignalId(u32);

impl SignalId {
    /// Returns the module-local numeric index.
    #[must_use]
    pub const fn index(self) -> u32 {
        self.0
    }
}

/// Stable identity of a typed expression.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ExprId(u32);

impl ExprId {
    /// Returns the module-local numeric index.
    #[must_use]
    pub const fn index(self) -> u32 {
        self.0
    }
}

/// One declared module signal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Signal {
    id: SignalId,
    name: String,
    direction: Option<PortDirection>,
    r#type: ValueType,
}

impl Signal {
    /// Returns the stable signal identity.
    #[must_use]
    pub const fn id(&self) -> SignalId {
        self.id
    }

    /// Returns the source-level name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the port direction, or `None` for an internal signal.
    #[must_use]
    pub const fn direction(&self) -> Option<PortDirection> {
        self.direction
    }

    /// Returns the packed value type.
    #[must_use]
    pub const fn r#type(&self) -> ValueType {
        self.r#type
    }
}

/// A statically selected packed range. Bit zero is the least-significant bit.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SignalSlice {
    /// Referenced signal.
    pub signal: SignalId,
    /// Least-significant selected bit.
    pub lsb: u32,
    /// Number of selected bits.
    pub width: BitWidth,
}

/// A concrete, two-state bit-vector constant stored least-significant word first.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Constant {
    width: BitWidth,
    words: Vec<u64>,
}

impl Constant {
    /// Creates a constant and masks unused bits in the final word.
    #[must_use]
    pub fn new(width: BitWidth, mut words: Vec<u64>) -> Self {
        let required_words = width.get().div_ceil(64) as usize;
        words.resize(required_words, 0);
        words.truncate(required_words);
        let remainder = width.get() % 64;
        if remainder != 0 {
            let mask = (1u64 << remainder) - 1;
            if let Some(last) = words.last_mut() {
                *last &= mask;
            }
        }
        Self { width, words }
    }

    /// Creates a constant from the low bits of a `u64`.
    #[must_use]
    pub fn from_u64(width: BitWidth, value: u64) -> Self {
        Self::new(width, vec![value])
    }

    /// Returns the packed width.
    #[must_use]
    pub const fn width(&self) -> BitWidth {
        self.width
    }

    /// Returns one bit, with bit zero as the least-significant bit.
    #[must_use]
    pub fn bit(&self, index: u32) -> bool {
        debug_assert!(index < self.width.get());
        (self.words[index as usize / 64] >> (index % 64)) & 1 != 0
    }
}

/// Supported unary RTL operations.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum UnaryOp {
    /// Bitwise inversion, preserving width.
    BitNot,
    /// Logical negation, producing one bit.
    LogicNot,
    /// OR reduction, producing one bit.
    ReduceOr,
    /// AND reduction, producing one bit.
    ReduceAnd,
    /// XOR reduction, producing one bit.
    ReduceXor,
}

/// Supported binary RTL operations.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum BinaryOp {
    /// Bitwise AND.
    And,
    /// Bitwise OR.
    Or,
    /// Bitwise XOR.
    Xor,
    /// Wrapping addition at the declared result width.
    Add,
    /// Wrapping subtraction at the declared result width.
    Sub,
    /// Equality comparison, producing one bit.
    Equal,
    /// Inequality comparison, producing one bit.
    NotEqual,
    /// Unsigned less-than comparison, producing one bit.
    LessThanUnsigned,
    /// Signed two's-complement less-than comparison, producing one bit.
    LessThanSigned,
    /// Unsigned less-than-or-equal comparison, producing one bit.
    LessOrEqualUnsigned,
    /// Signed two's-complement less-than-or-equal comparison, producing one bit.
    LessOrEqualSigned,
    /// Unsigned greater-than comparison, producing one bit.
    GreaterThanUnsigned,
    /// Signed two's-complement greater-than comparison, producing one bit.
    GreaterThanSigned,
    /// Unsigned greater-than-or-equal comparison, producing one bit.
    GreaterOrEqualUnsigned,
    /// Signed two's-complement greater-than-or-equal comparison, producing one bit.
    GreaterOrEqualSigned,
    /// Logical left shift, preserving the left operand width.
    ShiftLeft,
    /// Logical right shift, preserving the left operand width.
    ShiftRightLogical,
    /// Arithmetic right shift, preserving the left operand width.
    ShiftRightArithmetic,
}

impl BinaryOp {
    const fn is_comparison(self) -> bool {
        matches!(
            self,
            Self::Equal
                | Self::NotEqual
                | Self::LessThanUnsigned
                | Self::LessThanSigned
                | Self::LessOrEqualUnsigned
                | Self::LessOrEqualSigned
                | Self::GreaterThanUnsigned
                | Self::GreaterThanSigned
                | Self::GreaterOrEqualUnsigned
                | Self::GreaterOrEqualSigned
        )
    }

    const fn is_shift(self) -> bool {
        matches!(
            self,
            Self::ShiftLeft | Self::ShiftRightLogical | Self::ShiftRightArithmetic
        )
    }
}

/// One typed RTL expression node.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExprKind {
    /// A signal range read.
    Signal(SignalSlice),
    /// A concrete two-state constant.
    Constant(Constant),
    /// Unary operation.
    Unary {
        /// Operation.
        op: UnaryOp,
        /// Operand.
        input: ExprId,
    },
    /// Binary operation.
    Binary {
        /// Operation.
        op: BinaryOp,
        /// Left operand.
        lhs: ExprId,
        /// Right operand.
        rhs: ExprId,
    },
    /// One-bit conditional selection.
    Mux {
        /// One-bit condition.
        condition: ExprId,
        /// Value selected when the condition is one.
        then_expr: ExprId,
        /// Value selected when the condition is zero.
        else_expr: ExprId,
    },
    /// Concatenation ordered from most-significant part to least-significant part.
    Concat(Vec<ExprId>),
    /// Static slice of another expression.
    Slice {
        /// Source expression.
        input: ExprId,
        /// Least-significant selected bit.
        lsb: u32,
    },
}

/// Typed expression entry.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Expression {
    id: ExprId,
    kind: ExprKind,
    r#type: ValueType,
}

impl Expression {
    /// Returns the expression identity.
    #[must_use]
    pub const fn id(&self) -> ExprId {
        self.id
    }

    /// Returns the expression operation.
    #[must_use]
    pub const fn kind(&self) -> &ExprKind {
        &self.kind
    }

    /// Returns the explicit result type.
    #[must_use]
    pub const fn r#type(&self) -> ValueType {
        self.r#type
    }
}

/// One continuous or combinational assignment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Assignment {
    /// Assigned signal range.
    pub target: SignalSlice,
    /// Value expression.
    pub value: ExprId,
}

/// Active clock edge for a register.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ClockEdge {
    /// Rising clock edge.
    Rising,
    /// Falling clock edge.
    Falling,
}

/// Reset timing relative to the clock.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ResetMode {
    /// Reset is sampled on the active clock edge.
    Synchronous,
    /// Reset may change register state independently of the clock.
    Asynchronous,
}

/// Logical control polarity.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Polarity {
    /// Asserted at logic one.
    ActiveHigh,
    /// Asserted at logic zero.
    ActiveLow,
}

/// Optional register enable.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Enable {
    /// One-bit enable signal.
    pub signal: SignalId,
    /// Assertion polarity.
    pub polarity: Polarity,
}

/// Reset semantics attached to a register.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Reset {
    /// One-bit reset signal.
    pub signal: SignalId,
    /// Synchronous or asynchronous reset.
    pub mode: ResetMode,
    /// Assertion polarity.
    pub polarity: Polarity,
    /// Reset value expression matching the target width.
    pub value: ExprId,
}

/// A whole-signal register before bit blasting and technology mapping.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Register {
    /// Stable diagnostic name.
    pub name: String,
    /// Whole target signal.
    pub target: SignalId,
    /// Next-state expression.
    pub next: ExprId,
    /// One-bit clock signal.
    pub clock: SignalId,
    /// Active clock edge.
    pub edge: ClockEdge,
    /// Optional enable control.
    pub enable: Option<Enable>,
    /// Optional reset behavior.
    pub reset: Option<Reset>,
}

/// A memory that should remain recognizable for block-RAM inference.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Memory {
    /// Memory name.
    pub name: String,
    /// Width of each word.
    pub word: ValueType,
    /// Number of addressable words.
    pub depth: u32,
    /// Registered read latency in cycles.
    pub read_latency: u8,
    /// Address sampled by the synchronous read port.
    pub read_address: ExprId,
    /// Whole signal driven by the read port.
    pub read_data: SignalId,
    /// Optional read clock enable.
    pub read_enable: Option<Enable>,
    /// Address sampled by the synchronous write port.
    pub write_address: ExprId,
    /// Word written when `write_enable` is asserted.
    pub write_data: ExprId,
    /// Write-port enable and polarity.
    pub write_enable: Enable,
    /// Clock shared by the read and write ports.
    pub clock: SignalId,
    /// Active edge shared by the read and write ports.
    pub edge: ClockEdge,
}

/// A preserved module or black-box instance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Instance {
    /// Instance name in its parent.
    pub name: String,
    /// Referenced module name.
    pub module: String,
    /// Whether synthesis must retain this as an externally implemented cell.
    pub black_box: bool,
}

/// One hardware module before hierarchy lowering.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Module {
    name: String,
    ports: Vec<Port>,
    signals: Vec<Signal>,
    expressions: Vec<Expression>,
    assignments: Vec<Assignment>,
    registers: Vec<Register>,
    memories: Vec<Memory>,
    instances: Vec<Instance>,
}

impl Module {
    /// Creates an empty module.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            ports: Vec::new(),
            signals: Vec::new(),
            expressions: Vec::new(),
            assignments: Vec::new(),
            registers: Vec::new(),
            memories: Vec::new(),
            instances: Vec::new(),
        }
    }

    /// Returns the module name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns module ports in source order when the frontend provides it.
    #[must_use]
    pub fn ports(&self) -> &[Port] {
        &self.ports
    }

    /// Returns every port and internal signal.
    #[must_use]
    pub fn signals(&self) -> &[Signal] {
        &self.signals
    }

    /// Returns typed expressions in dependency order.
    #[must_use]
    pub fn expressions(&self) -> &[Expression] {
        &self.expressions
    }

    /// Returns combinational assignments.
    #[must_use]
    pub fn assignments(&self) -> &[Assignment] {
        &self.assignments
    }

    /// Returns state-holding registers.
    #[must_use]
    pub fn registers(&self) -> &[Register] {
        &self.registers
    }

    /// Returns inferred or declared memories.
    #[must_use]
    pub fn memories(&self) -> &[Memory] {
        &self.memories
    }

    /// Returns child instances.
    #[must_use]
    pub fn instances(&self) -> &[Instance] {
        &self.instances
    }

    /// Adds a port and returns its signal identity.
    pub fn add_port(&mut self, port: Port) -> SignalId {
        let id = self.next_signal_id();
        self.signals.push(Signal {
            id,
            name: port.name.clone(),
            direction: Some(port.direction),
            r#type: port.r#type,
        });
        self.ports.push(port);
        id
    }

    /// Adds an internal signal and returns its identity.
    pub fn add_signal(&mut self, name: impl Into<String>, r#type: ValueType) -> SignalId {
        let id = self.next_signal_id();
        self.signals.push(Signal {
            id,
            name: name.into(),
            direction: None,
            r#type,
        });
        id
    }

    /// Returns a whole-signal slice.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown signal.
    pub fn whole(&self, signal: SignalId) -> Result<SignalSlice, RtlError> {
        let signal_info = self.signal(signal)?;
        Ok(SignalSlice {
            signal,
            lsb: 0,
            width: signal_info.r#type.width,
        })
    }

    /// Returns a checked static signal slice.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown signal or out-of-range selection.
    pub fn slice(
        &self,
        signal: SignalId,
        lsb: u32,
        width: BitWidth,
    ) -> Result<SignalSlice, RtlError> {
        let signal_info = self.signal(signal)?;
        validate_range(lsb, width, signal_info.r#type.width)?;
        Ok(SignalSlice { signal, lsb, width })
    }

    /// Adds a whole-signal read expression.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown signal.
    pub fn read(&mut self, signal: SignalId) -> Result<ExprId, RtlError> {
        let slice = self.whole(signal)?;
        self.read_slice(slice)
    }

    /// Adds a signal-slice read expression.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid slice.
    pub fn read_slice(&mut self, slice: SignalSlice) -> Result<ExprId, RtlError> {
        let signal = self.signal(slice.signal)?;
        validate_range(slice.lsb, slice.width, signal.r#type.width)?;
        let r#type = ValueType {
            width: slice.width,
            signed: signal.r#type.signed && slice.lsb == 0 && slice.width == signal.r#type.width,
            state: signal.r#type.state,
        };
        Ok(self.push_expr(ExprKind::Signal(slice), r#type))
    }

    /// Adds a constant expression.
    pub fn constant(&mut self, value: Constant) -> ExprId {
        let r#type = ValueType {
            width: value.width(),
            signed: false,
            state: StateDomain::TwoState,
        };
        self.push_expr(ExprKind::Constant(value), r#type)
    }

    /// Adds a checked unary expression.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown operand.
    pub fn unary(&mut self, op: UnaryOp, input: ExprId) -> Result<ExprId, RtlError> {
        let input_type = self.expression(input)?.r#type;
        let r#type = match op {
            UnaryOp::BitNot => input_type,
            UnaryOp::LogicNot | UnaryOp::ReduceOr | UnaryOp::ReduceAnd | UnaryOp::ReduceXor => {
                ValueType {
                    width: BitWidth::new(1)?,
                    signed: false,
                    state: input_type.state,
                }
            }
        };
        Ok(self.push_expr(ExprKind::Unary { op, input }, r#type))
    }

    /// Adds a checked binary expression.
    ///
    /// # Errors
    ///
    /// Returns an error for unknown operands or mismatched widths.
    pub fn binary(&mut self, op: BinaryOp, lhs: ExprId, rhs: ExprId) -> Result<ExprId, RtlError> {
        let lhs_type = self.expression(lhs)?.r#type;
        let rhs_type = self.expression(rhs)?.r#type;
        if !op.is_shift() && lhs_type.width != rhs_type.width {
            return Err(RtlError::WidthMismatch {
                expected: lhs_type.width,
                actual: rhs_type.width,
            });
        }
        let r#type = if op.is_comparison() {
            ValueType {
                width: BitWidth::new(1)?,
                signed: false,
                state: merge_state(lhs_type.state, rhs_type.state),
            }
        } else if op.is_shift() {
            ValueType {
                width: lhs_type.width,
                signed: lhs_type.signed,
                state: merge_state(lhs_type.state, rhs_type.state),
            }
        } else {
            ValueType {
                width: lhs_type.width,
                signed: lhs_type.signed && rhs_type.signed,
                state: merge_state(lhs_type.state, rhs_type.state),
            }
        };
        Ok(self.push_expr(ExprKind::Binary { op, lhs, rhs }, r#type))
    }

    /// Adds a checked mux expression.
    ///
    /// # Errors
    ///
    /// Returns an error unless the condition is one bit and both arms have
    /// equal widths.
    pub fn mux(
        &mut self,
        condition: ExprId,
        then_expr: ExprId,
        else_expr: ExprId,
    ) -> Result<ExprId, RtlError> {
        let condition_type = self.expression(condition)?.r#type;
        if condition_type.width.get() != 1 {
            return Err(RtlError::InvalidControlWidth("mux condition".into()));
        }
        let then_type = self.expression(then_expr)?.r#type;
        let else_type = self.expression(else_expr)?.r#type;
        if then_type.width != else_type.width {
            return Err(RtlError::WidthMismatch {
                expected: then_type.width,
                actual: else_type.width,
            });
        }
        let r#type = ValueType {
            width: then_type.width,
            signed: then_type.signed && else_type.signed,
            state: merge_state(
                condition_type.state,
                merge_state(then_type.state, else_type.state),
            ),
        };
        Ok(self.push_expr(
            ExprKind::Mux {
                condition,
                then_expr,
                else_expr,
            },
            r#type,
        ))
    }

    /// Adds a concatenation ordered from most-significant to least-significant part.
    ///
    /// # Errors
    ///
    /// Returns an error for an empty concatenation, an unknown expression, or
    /// a width that exceeds the RTL representation.
    pub fn concat(&mut self, parts: Vec<ExprId>) -> Result<ExprId, RtlError> {
        if parts.is_empty() {
            return Err(RtlError::EmptyConcatenation);
        }
        let mut width = 0u32;
        let mut state = StateDomain::TwoState;
        for part in &parts {
            let part_type = self.expression(*part)?.r#type;
            width = width
                .checked_add(part_type.width.get())
                .ok_or(RtlError::WidthOverflow)?;
            state = merge_state(state, part_type.state);
        }
        let r#type = ValueType {
            width: BitWidth::new(width)?,
            signed: false,
            state,
        };
        Ok(self.push_expr(ExprKind::Concat(parts), r#type))
    }

    /// Adds a static expression slice.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown expression or out-of-range selection.
    pub fn expression_slice(
        &mut self,
        input: ExprId,
        lsb: u32,
        width: BitWidth,
    ) -> Result<ExprId, RtlError> {
        let input_type = self.expression(input)?.r#type;
        validate_range(lsb, width, input_type.width)?;
        let r#type = ValueType {
            width,
            signed: false,
            state: input_type.state,
        };
        Ok(self.push_expr(ExprKind::Slice { input, lsb }, r#type))
    }

    /// Adds a checked combinational assignment.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid target, an input target, or mismatched widths.
    pub fn assign(&mut self, target: SignalSlice, value: ExprId) -> Result<(), RtlError> {
        let signal = self.signal(target.signal)?;
        validate_range(target.lsb, target.width, signal.r#type.width)?;
        if signal.direction == Some(PortDirection::Input) {
            return Err(RtlError::DriveInput(signal.name.clone()));
        }
        let value_type = self.expression(value)?.r#type;
        if target.width != value_type.width {
            return Err(RtlError::WidthMismatch {
                expected: target.width,
                actual: value_type.width,
            });
        }
        self.assignments.push(Assignment { target, value });
        Ok(())
    }

    /// Adds a checked whole-signal register.
    ///
    /// # Errors
    ///
    /// Returns an error for unknown identities, invalid control widths, input
    /// targets, or mismatched next/reset widths.
    pub fn add_register(&mut self, register: Register) -> Result<(), RtlError> {
        let target = self.signal(register.target)?;
        if target.direction == Some(PortDirection::Input) {
            return Err(RtlError::DriveInput(target.name.clone()));
        }
        let next_type = self.expression(register.next)?.r#type;
        if next_type.width != target.r#type.width {
            return Err(RtlError::WidthMismatch {
                expected: target.r#type.width,
                actual: next_type.width,
            });
        }
        self.validate_control(register.clock, "clock")?;
        if let Some(enable) = register.enable {
            self.validate_control(enable.signal, "enable")?;
        }
        if let Some(reset) = register.reset {
            self.validate_control(reset.signal, "reset")?;
            let reset_type = self.expression(reset.value)?.r#type;
            if reset_type.width != target.r#type.width {
                return Err(RtlError::WidthMismatch {
                    expected: target.r#type.width,
                    actual: reset_type.width,
                });
            }
        }
        self.registers.push(register);
        Ok(())
    }

    /// Adds a memory declaration.
    pub fn add_memory(&mut self, memory: Memory) {
        self.memories.push(memory);
    }

    /// Adds an instance declaration.
    pub fn add_instance(&mut self, instance: Instance) {
        self.instances.push(instance);
    }

    /// Checks module-local structural invariants.
    ///
    /// # Errors
    ///
    /// Returns the first invalid identity, expression, driver, or declaration.
    pub fn validate(&self) -> Result<(), RtlError> {
        validate_name(&self.name, "module")?;
        let mut names = HashSet::new();
        for (index, signal) in self.signals.iter().enumerate() {
            if signal.id.index() as usize != index {
                return Err(RtlError::InvalidSignalIdentity(signal.id.index()));
            }
            validate_unique_name(&signal.name, "signal", &mut names)?;
        }
        if self.ports.len() > self.signals.len() {
            return Err(RtlError::PortSignalMismatch);
        }
        for (port, signal) in self.ports.iter().zip(&self.signals) {
            if port.name != signal.name
                || Some(port.direction) != signal.direction
                || port.r#type != signal.r#type
            {
                return Err(RtlError::PortSignalMismatch);
            }
        }

        for (index, expression) in self.expressions.iter().enumerate() {
            if expression.id.index() as usize != index {
                return Err(RtlError::InvalidExpressionIdentity(expression.id.index()));
            }
            self.validate_expression(expression)?;
        }

        let mut driven_bits = HashSet::new();
        for assignment in &self.assignments {
            self.validate_assignment(*assignment, &mut driven_bits)?;
        }
        for register in &self.registers {
            validate_name(&register.name, "register")?;
            let target = self.signal(register.target)?;
            for bit in 0..target.r#type.width.get() {
                if !driven_bits.insert((register.target, bit)) {
                    return Err(RtlError::MultipleDrivers {
                        signal: target.name.clone(),
                        bit,
                    });
                }
            }
            self.expression(register.next)?;
            self.validate_control(register.clock, "clock")?;
            if let Some(enable) = register.enable {
                self.validate_control(enable.signal, "enable")?;
            }
            if let Some(reset) = register.reset {
                self.validate_control(reset.signal, "reset")?;
                self.expression(reset.value)?;
            }
        }
        self.validate_memories(&mut names, &mut driven_bits)?;
        for instance in &self.instances {
            validate_unique_name(&instance.name, "instance", &mut names)?;
            validate_name(&instance.module, "instantiated module")?;
        }
        Ok(())
    }

    fn validate_memories<'a>(
        &'a self,
        names: &mut HashSet<&'a str>,
        driven_bits: &mut HashSet<(SignalId, u32)>,
    ) -> Result<(), RtlError> {
        for memory in &self.memories {
            validate_unique_name(&memory.name, "memory", names)?;
            if memory.depth == 0 {
                return Err(RtlError::ZeroDepth(memory.name.clone()));
            }
            if memory.read_latency != 1 {
                return Err(RtlError::UnsupportedMemoryReadLatency {
                    memory: memory.name.clone(),
                    latency: memory.read_latency,
                });
            }
            let address_width =
                BitWidth::new((u32::BITS - (memory.depth - 1).leading_zeros()).max(1))?;
            for address in [memory.read_address, memory.write_address] {
                let actual = self.expression(address)?.r#type.width;
                if actual != address_width {
                    return Err(RtlError::WidthMismatch {
                        expected: address_width,
                        actual,
                    });
                }
            }
            let read_data = self.signal(memory.read_data)?;
            if read_data.direction == Some(PortDirection::Input) {
                return Err(RtlError::DriveInput(read_data.name.clone()));
            }
            if read_data.r#type.width != memory.word.width {
                return Err(RtlError::WidthMismatch {
                    expected: memory.word.width,
                    actual: read_data.r#type.width,
                });
            }
            let write_width = self.expression(memory.write_data)?.r#type.width;
            if write_width != memory.word.width {
                return Err(RtlError::WidthMismatch {
                    expected: memory.word.width,
                    actual: write_width,
                });
            }
            self.validate_control(memory.clock, "memory clock")?;
            self.validate_control(memory.write_enable.signal, "memory write enable")?;
            if let Some(enable) = memory.read_enable {
                self.validate_control(enable.signal, "memory read enable")?;
            }
            for bit in 0..read_data.r#type.width.get() {
                if !driven_bits.insert((memory.read_data, bit)) {
                    return Err(RtlError::MultipleDrivers {
                        signal: read_data.name.clone(),
                        bit,
                    });
                }
            }
        }
        Ok(())
    }

    fn signal(&self, id: SignalId) -> Result<&Signal, RtlError> {
        self.signals
            .get(id.index() as usize)
            .ok_or(RtlError::UnknownSignal(id.index()))
    }

    fn expression(&self, id: ExprId) -> Result<&Expression, RtlError> {
        self.expressions
            .get(id.index() as usize)
            .ok_or(RtlError::UnknownExpression(id.index()))
    }

    fn validate_control(&self, id: SignalId, kind: &'static str) -> Result<(), RtlError> {
        let signal = self.signal(id)?;
        if signal.r#type.width.get() != 1 {
            return Err(RtlError::InvalidControlWidth(format!(
                "{kind} `{}`",
                signal.name
            )));
        }
        Ok(())
    }

    fn validate_expression(&self, expression: &Expression) -> Result<(), RtlError> {
        let current = expression.id.index();
        let validate_dependency = |id: ExprId| {
            if id.index() >= current {
                Err(RtlError::ExpressionOrder {
                    expression: current,
                    dependency: id.index(),
                })
            } else {
                self.expression(id).map(|_| ())
            }
        };
        match &expression.kind {
            ExprKind::Signal(slice) => {
                let signal = self.signal(slice.signal)?;
                validate_range(slice.lsb, slice.width, signal.r#type.width)?;
            }
            ExprKind::Constant(value) => {
                if value.width() != expression.r#type.width {
                    return Err(RtlError::WidthMismatch {
                        expected: expression.r#type.width,
                        actual: value.width(),
                    });
                }
            }
            ExprKind::Unary { input, .. } => validate_dependency(*input)?,
            ExprKind::Binary { lhs, rhs, .. } => {
                validate_dependency(*lhs)?;
                validate_dependency(*rhs)?;
            }
            ExprKind::Mux {
                condition,
                then_expr,
                else_expr,
            } => {
                validate_dependency(*condition)?;
                validate_dependency(*then_expr)?;
                validate_dependency(*else_expr)?;
            }
            ExprKind::Concat(parts) => {
                if parts.is_empty() {
                    return Err(RtlError::EmptyConcatenation);
                }
                for part in parts {
                    validate_dependency(*part)?;
                }
            }
            ExprKind::Slice { input, lsb } => {
                validate_dependency(*input)?;
                let source = self.expression(*input)?;
                validate_range(*lsb, expression.r#type.width, source.r#type.width)?;
            }
        }
        Ok(())
    }

    fn validate_assignment(
        &self,
        assignment: Assignment,
        driven_bits: &mut HashSet<(SignalId, u32)>,
    ) -> Result<(), RtlError> {
        let target = self.signal(assignment.target.signal)?;
        validate_range(
            assignment.target.lsb,
            assignment.target.width,
            target.r#type.width,
        )?;
        let value = self.expression(assignment.value)?;
        if value.r#type.width != assignment.target.width {
            return Err(RtlError::WidthMismatch {
                expected: assignment.target.width,
                actual: value.r#type.width,
            });
        }
        for bit in assignment.target.lsb..assignment.target.lsb + assignment.target.width.get() {
            if !driven_bits.insert((assignment.target.signal, bit)) {
                return Err(RtlError::MultipleDrivers {
                    signal: target.name.clone(),
                    bit,
                });
            }
        }
        Ok(())
    }

    fn push_expr(&mut self, kind: ExprKind, r#type: ValueType) -> ExprId {
        #[allow(clippy::cast_possible_truncation)]
        let id = ExprId(self.expressions.len() as u32);
        self.expressions.push(Expression { id, kind, r#type });
        id
    }

    #[allow(clippy::cast_possible_truncation)]
    fn next_signal_id(&self) -> SignalId {
        SignalId(self.signals.len() as u32)
    }
}

/// A complete design and its selected top module.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Design {
    top: String,
    modules: Vec<Module>,
}

impl Design {
    /// Creates an empty design for `top`.
    #[must_use]
    pub fn new(top: impl Into<String>) -> Self {
        Self {
            top: top.into(),
            modules: Vec::new(),
        }
    }

    /// Returns the selected top module name.
    #[must_use]
    pub fn top(&self) -> &str {
        &self.top
    }

    /// Returns all modules.
    #[must_use]
    pub fn modules(&self) -> &[Module] {
        &self.modules
    }

    /// Returns the selected top module.
    #[must_use]
    pub fn top_module(&self) -> Option<&Module> {
        self.modules.iter().find(|module| module.name == self.top)
    }

    /// Adds a module.
    pub fn add_module(&mut self, module: Module) {
        self.modules.push(module);
    }

    /// Validates module-local and design-wide invariants.
    ///
    /// # Errors
    ///
    /// Returns the first invalid module, duplicate module, missing top, or
    /// unresolved non-black-box instance.
    pub fn validate(&self) -> Result<(), RtlError> {
        validate_name(&self.top, "top module")?;
        let mut module_names = HashSet::new();
        for module in &self.modules {
            module.validate()?;
            if !module_names.insert(module.name.as_str()) {
                return Err(RtlError::DuplicateName(module.name.clone()));
            }
        }
        if !module_names.contains(self.top.as_str()) {
            return Err(RtlError::MissingTop(self.top.clone()));
        }
        for module in &self.modules {
            for instance in &module.instances {
                if !instance.black_box && !module_names.contains(instance.module.as_str()) {
                    return Err(RtlError::UnknownModule(instance.module.clone()));
                }
            }
        }
        Ok(())
    }
}

fn merge_state(lhs: StateDomain, rhs: StateDomain) -> StateDomain {
    if lhs == StateDomain::FourState || rhs == StateDomain::FourState {
        StateDomain::FourState
    } else {
        StateDomain::TwoState
    }
}

fn validate_range(lsb: u32, width: BitWidth, signal_width: BitWidth) -> Result<(), RtlError> {
    if lsb
        .checked_add(width.get())
        .is_none_or(|end| end > signal_width.get())
    {
        Err(RtlError::InvalidSlice {
            lsb,
            width,
            signal_width,
        })
    } else {
        Ok(())
    }
}

fn validate_name(name: &str, kind: &'static str) -> Result<(), RtlError> {
    if name.trim().is_empty() {
        Err(RtlError::EmptyName(kind))
    } else {
        Ok(())
    }
}

fn validate_unique_name<'a>(
    name: &'a str,
    kind: &'static str,
    names: &mut HashSet<&'a str>,
) -> Result<(), RtlError> {
    validate_name(name, kind)?;
    if names.insert(name) {
        Ok(())
    } else {
        Err(RtlError::DuplicateName(name.to_owned()))
    }
}

/// Invalid hardware-semantic RTL.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RtlError {
    /// A packed value has no bits.
    ZeroWidth,
    /// A packed width addition overflowed.
    WidthOverflow,
    /// A named entity has an empty name.
    EmptyName(&'static str),
    /// A name is reused in the same namespace.
    DuplicateName(String),
    /// The selected top does not exist.
    MissingTop(String),
    /// An instance references an unknown non-black-box module.
    UnknownModule(String),
    /// A memory has no entries.
    ZeroDepth(String),
    /// A memory read latency cannot be implemented by the inference path.
    UnsupportedMemoryReadLatency {
        /// Memory name.
        memory: String,
        /// Requested latency in clock cycles.
        latency: u8,
    },
    /// A signal identity does not exist.
    UnknownSignal(u32),
    /// An expression identity does not exist.
    UnknownExpression(u32),
    /// A signal identity does not match its table position.
    InvalidSignalIdentity(u32),
    /// An expression identity does not match its table position.
    InvalidExpressionIdentity(u32),
    /// A port entry does not match its signal entry.
    PortSignalMismatch,
    /// A slice lies outside the source value.
    InvalidSlice {
        /// Least-significant selected bit.
        lsb: u32,
        /// Selected width.
        width: BitWidth,
        /// Available width.
        signal_width: BitWidth,
    },
    /// Two connected values have different widths.
    WidthMismatch {
        /// Required width.
        expected: BitWidth,
        /// Actual width.
        actual: BitWidth,
    },
    /// A control expression is not one bit.
    InvalidControlWidth(String),
    /// A primary input is driven internally.
    DriveInput(String),
    /// An expression depends on itself or a later expression.
    ExpressionOrder {
        /// Expression being validated.
        expression: u32,
        /// Invalid dependency.
        dependency: u32,
    },
    /// A concatenation contains no parts.
    EmptyConcatenation,
    /// More than one assignment or register drives a bit.
    MultipleDrivers {
        /// Signal name.
        signal: String,
        /// Driven bit index.
        bit: u32,
    },
}

impl Display for RtlError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroWidth => formatter.write_str("packed bit width must be non-zero"),
            Self::WidthOverflow => formatter.write_str("packed bit width overflowed"),
            Self::EmptyName(kind) => write!(formatter, "{kind} name must not be empty"),
            Self::DuplicateName(name) => write!(formatter, "duplicate RTL name: {name}"),
            Self::MissingTop(name) => write!(formatter, "top module `{name}` does not exist"),
            Self::UnknownModule(name) => write!(formatter, "unknown module `{name}`"),
            Self::ZeroDepth(name) => write!(formatter, "memory `{name}` has zero depth"),
            Self::UnsupportedMemoryReadLatency { memory, latency } => write!(
                formatter,
                "memory `{memory}` has unsupported read latency {latency}; expected 1"
            ),
            Self::UnknownSignal(id) => write!(formatter, "unknown signal id {id}"),
            Self::UnknownExpression(id) => write!(formatter, "unknown expression id {id}"),
            Self::InvalidSignalIdentity(id) => write!(formatter, "invalid signal identity {id}"),
            Self::InvalidExpressionIdentity(id) => {
                write!(formatter, "invalid expression identity {id}")
            }
            Self::PortSignalMismatch => formatter.write_str("port and signal tables disagree"),
            Self::InvalidSlice {
                lsb,
                width,
                signal_width,
            } => write!(
                formatter,
                "slice [{lsb} +: {width}] exceeds signal width {signal_width}"
            ),
            Self::WidthMismatch { expected, actual } => {
                write!(formatter, "expected width {expected}, got {actual}")
            }
            Self::InvalidControlWidth(name) => {
                write!(formatter, "{name} must be exactly one bit")
            }
            Self::DriveInput(name) => write!(formatter, "input `{name}` cannot be driven"),
            Self::ExpressionOrder {
                expression,
                dependency,
            } => write!(
                formatter,
                "expression {expression} depends on non-previous expression {dependency}"
            ),
            Self::EmptyConcatenation => formatter.write_str("concatenation must not be empty"),
            Self::MultipleDrivers { signal, bit } => {
                write!(
                    formatter,
                    "signal `{signal}` bit {bit} has multiple drivers"
                )
            }
        }
    }
}

impl Error for RtlError {}

#[cfg(test)]
mod tests {
    use super::{
        BinaryOp, BitWidth, ClockEdge, Constant, Design, Module, Port, PortDirection, Register,
        StateDomain, ValueType,
    };

    fn bits(width: u32) -> ValueType {
        ValueType {
            width: BitWidth::new(width).unwrap(),
            signed: false,
            state: StateDomain::TwoState,
        }
    }

    #[test]
    fn valid_design_preserves_four_state_port_type() {
        let mut top = Module::new("Top");
        top.add_port(Port {
            name: "clk".into(),
            direction: PortDirection::Input,
            r#type: ValueType {
                width: BitWidth::new(1).unwrap(),
                signed: false,
                state: StateDomain::FourState,
            },
        });
        let mut design = Design::new("Top");
        design.add_module(top);

        assert_eq!(design.validate(), Ok(()));
        assert_eq!(
            design.modules()[0].ports()[0].r#type.state,
            StateDomain::FourState
        );
    }

    #[test]
    fn builds_typed_adder_and_register() {
        let mut top = Module::new("Counter");
        let clock = top.add_port(Port {
            name: "clock".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let q = top.add_port(Port {
            name: "q".into(),
            direction: PortDirection::Output,
            r#type: bits(8),
        });
        let q_expr = top.read(q).unwrap();
        let one = top.constant(Constant::from_u64(BitWidth::new(8).unwrap(), 1));
        let next = top.binary(BinaryOp::Add, q_expr, one).unwrap();
        top.add_register(Register {
            name: "q".into(),
            target: q,
            next,
            clock,
            edge: ClockEdge::Rising,
            enable: None,
            reset: None,
        })
        .unwrap();

        assert_eq!(top.validate(), Ok(()));
    }

    #[test]
    fn constant_masks_unused_high_bits() {
        let value = Constant::from_u64(BitWidth::new(5).unwrap(), 0xff);
        assert!((0..5).all(|bit| value.bit(bit)));
    }

    #[test]
    fn missing_top_is_rejected() {
        assert!(Design::new("Missing").validate().is_err());
    }
}
