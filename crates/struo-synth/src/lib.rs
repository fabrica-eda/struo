//! Technology-independent synthesis for Struo.

use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fmt::{self, Display, Formatter};
use std::num::NonZeroU32;

use struo_ir::{
    ActiveLevel, ArithmeticOp, ClockEdge as IrClockEdge, ComparisonOp, EnableControl, MemoryCell,
    MemoryPort as IrMemoryPort, MemoryStyle as IrMemoryStyle, NetId, Netlist, NodeKind,
    RegisterCell, ResetControl, ValidationError,
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
    /// Reports produced while lowering and optimizing the design.
    pub reports: Vec<PassReport>,
}

/// Target-independent synthesis pass controls.
///
/// The default preserves Struo's existing synthesis behavior. Individual
/// transformations can be disabled for `QoR` experiments without requiring
/// callers to reimplement RTL lowering.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SynthesisOptions {
    /// Replace direct register self-hold muxes with clock enables.
    pub infer_register_enables: bool,
    /// Remove enables from payload registers proven unobservable while invalid.
    pub relax_qualified_register_enables: bool,
}

impl Default for SynthesisOptions {
    fn default() -> Self {
        Self {
            infer_register_enables: true,
            relax_qualified_register_enables: true,
        }
    }
}

/// Synthesizes the selected top module into a bit-level logic netlist.
///
/// Construction performs constant folding and structural hashing. Word-level
/// addition, subtraction, and ordering comparisons are retained for target-specific carry mapping.
/// Synchronous simple-dual-port memories are retained for block-RAM mapping. Hierarchy
/// and inout ports are rejected until their semantics have dedicated passes.
///
/// # Errors
///
/// Returns an error for invalid RTL, unsupported constructs, undriven bits,
/// combinational loops, non-constant reset values, or an invalid netlist.
pub fn synthesize(design: &Design) -> Result<SynthesisResult, SynthesisError> {
    synthesize_with_options(design, SynthesisOptions::default())
}

/// Synthesizes the selected top module with explicit pass controls.
///
/// # Errors
///
/// Returns the same errors as [`synthesize`].
pub fn synthesize_with_options(
    design: &Design,
    options: SynthesisOptions,
) -> Result<SynthesisResult, SynthesisError> {
    validate_rtl(design)?;
    let module = design
        .top_module()
        .ok_or_else(|| SynthesisError::InvalidRtl(RtlError::MissingTop(design.top().into())))?;
    reject_unsupported(module)?;

    let mut lowering = Lowering::new(module);
    lowering.reserve_sources();
    lowering.index_assignments();
    lowering.connect_memories()?;
    lowering.connect_registers()?;
    lowering.connect_outputs()?;
    let mut netlist = lowering.netlist;
    let mut reports = vec![PassReport {
        pass: "lower-rtl",
        message: format!(
            "lowered {} expressions to {} nodes, {} arithmetic cells, {} comparison cells, {} registers, and {} memories",
            module.expressions().len(),
            netlist.nodes().len(),
            netlist.arithmetic().len(),
            netlist.comparisons().len(),
            netlist.registers().len(),
            netlist.memories().len()
        ),
    }];
    reports.extend(pipeline_with_options(options).run(&mut netlist)?);
    Ok(SynthesisResult { netlist, reports })
}

fn reject_unsupported(module: &Module) -> Result<(), SynthesisError> {
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
        for memory in self.module.memories() {
            for (suffix, read_data) in std::iter::once(("", memory.read_data)).chain(
                memory
                    .second_port
                    .as_ref()
                    .map(|port| ("_b", port.read_data)),
            ) {
                let target = &self.module.signals()[read_data.index() as usize];
                for bit in 0..target.r#type().width.get() as usize {
                    let net = self.netlist.add_memory_output(bit_name(
                        &format!("{}{suffix}", memory.name),
                        target.r#type().width.get(),
                        bit,
                    ));
                    self.signal_bits[read_data.index() as usize][bit] = Some(net);
                }
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

    fn connect_memories(&mut self) -> Result<(), SynthesisError> {
        for memory in self.module.memories() {
            let read_data = self.signal_bits[memory.read_data.index() as usize]
                .iter()
                .map(|net| net.expect("memory outputs were reserved"))
                .collect();
            let read_address = self.lower_expression(memory.read_address)?;
            let write_address = self.lower_expression(memory.write_address)?;
            let write_data = self.lower_expression(memory.write_data)?;
            let clock = self.resolve_signal_bit(memory.clock, 0)?;
            let write_enable = EnableControl {
                signal: self.resolve_signal_bit(memory.write_enable.signal, 0)?,
                active: lower_polarity(memory.write_enable.polarity),
            };
            let read_enable = memory
                .read_enable
                .map(|enable| {
                    self.resolve_signal_bit(enable.signal, 0)
                        .map(|signal| EnableControl {
                            signal,
                            active: lower_polarity(enable.polarity),
                        })
                })
                .transpose()?;
            let mut cell = MemoryCell::new(
                &memory.name,
                memory.depth,
                read_address,
                read_data,
                read_enable,
                write_address,
                write_data,
                write_enable,
                clock,
                match memory.edge {
                    ClockEdge::Rising => IrClockEdge::Rising,
                    ClockEdge::Falling => IrClockEdge::Falling,
                },
            )
            .with_read_latency(memory.read_latency)
            .with_style(match memory.style {
                struo_rtl::MemoryStyle::Auto => IrMemoryStyle::Auto,
                struo_rtl::MemoryStyle::Block => IrMemoryStyle::Block,
                struo_rtl::MemoryStyle::Distributed => IrMemoryStyle::Distributed,
            });
            if let Some(port) = &memory.second_port {
                let read_data = self.signal_bits[port.read_data.index() as usize]
                    .iter()
                    .map(|net| net.expect("memory outputs were reserved"))
                    .collect();
                let read_address = self.lower_expression(port.read_address)?;
                let write_address = self.lower_expression(port.write_address)?;
                let write_data = self.lower_expression(port.write_data)?;
                let write_enable = EnableControl {
                    signal: self.resolve_signal_bit(port.write_enable.signal, 0)?,
                    active: lower_polarity(port.write_enable.polarity),
                };
                let read_enable = port
                    .read_enable
                    .map(|enable| {
                        self.resolve_signal_bit(enable.signal, 0)
                            .map(|signal| EnableControl {
                                signal,
                                active: lower_polarity(enable.polarity),
                            })
                    })
                    .transpose()?;
                cell = cell.with_second_port(IrMemoryPort::new(
                    read_address,
                    read_data,
                    read_enable,
                    write_address,
                    write_data,
                    write_enable,
                    self.resolve_signal_bit(port.clock, 0)?,
                    match port.edge {
                        ClockEdge::Rising => IrClockEdge::Rising,
                        ClockEdge::Falling => IrClockEdge::Falling,
                    },
                ));
            }
            self.netlist.add_memory(cell);
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
                if op == BinaryOp::Add
                    && let Some(bits) = self.lower_add_with_carry(lhs, rhs)?
                {
                    bits
                } else {
                    let lhs = self.lower_expression(lhs)?;
                    let rhs = self.lower_expression(rhs)?;
                    self.lower_binary(op, &lhs, &rhs)
                }
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
                self.lower_expression_range(input, lsb as usize, width)?
            }
        };
        self.expression_bits[index] = Some(bits.clone());
        Ok(bits)
    }

    fn lower_add_with_carry(
        &mut self,
        lhs: ExprId,
        rhs: ExprId,
    ) -> Result<Option<Vec<NetId>>, SynthesisError> {
        let Some((add_lhs, add_rhs, carry)) = self
            .add_operands(lhs)
            .and_then(|(add_lhs, add_rhs)| {
                self.carry_lsb(rhs).map(|carry| (add_lhs, add_rhs, carry))
            })
            .or_else(|| {
                self.add_operands(rhs).and_then(|(add_lhs, add_rhs)| {
                    self.carry_lsb(lhs).map(|carry| (add_lhs, add_rhs, carry))
                })
            })
        else {
            return Ok(None);
        };
        let lhs = self.lower_expression(add_lhs)?;
        let rhs = self.lower_expression(add_rhs)?;
        let carry = self.lower_expression(carry)?[0];
        Ok(Some(
            self.netlist
                .add_arithmetic_with_carry(&lhs, &rhs, carry)
                .expect("validated RTL carry addition has equal, non-zero widths"),
        ))
    }

    fn add_operands(&self, id: ExprId) -> Option<(ExprId, ExprId)> {
        match self.module.expressions()[id.index() as usize].kind() {
            ExprKind::Binary {
                op: BinaryOp::Add,
                lhs,
                rhs,
            } => Some((*lhs, *rhs)),
            _ => None,
        }
    }

    fn carry_lsb(&self, id: ExprId) -> Option<ExprId> {
        let expression = &self.module.expressions()[id.index() as usize];
        if expression.r#type().width.get() == 1 {
            return Some(id);
        }
        match expression.kind() {
            ExprKind::Constant(value) if (1..value.width().get()).all(|bit| !value.bit(bit)) => {
                Some(id)
            }
            ExprKind::Concat(parts) => {
                let (&least_significant, upper) = parts.split_last()?;
                let lsb_width = self.module.expressions()[least_significant.index() as usize]
                    .r#type()
                    .width
                    .get();
                (lsb_width == 1 && upper.iter().all(|part| self.is_zero_expression(*part)))
                    .then_some(least_significant)
            }
            _ => None,
        }
    }

    fn is_zero_expression(&self, id: ExprId) -> bool {
        match self.module.expressions()[id.index() as usize].kind() {
            ExprKind::Constant(value) => (0..value.width().get()).all(|bit| !value.bit(bit)),
            ExprKind::Concat(parts) => parts.iter().all(|part| self.is_zero_expression(*part)),
            _ => false,
        }
    }

    fn lower_expression_range(
        &mut self,
        id: ExprId,
        lsb: usize,
        width: usize,
    ) -> Result<Vec<NetId>, SynthesisError> {
        if let Some(bits) = &self.expression_bits[id.index() as usize] {
            return Ok(bits[lsb..lsb + width].to_vec());
        }
        let expression = &self.module.expressions()[id.index() as usize];
        debug_assert!(lsb + width <= expression.r#type().width.get() as usize);
        // Keep bitwise expressions lazy across slices. Procedural partial
        // assignments can leave self-references in bits that a later write
        // discards; lowering the whole vector would report those dead edges
        // as combinational feedback.
        match expression.kind().clone() {
            ExprKind::Signal(slice) => (0..width)
                .map(|offset| {
                    self.resolve_signal_bit(slice.signal, slice.lsb as usize + lsb + offset)
                })
                .collect(),
            ExprKind::Constant(value) => Ok((lsb..lsb + width)
                .map(|bit| {
                    let bit = u32::try_from(bit).expect("RTL expression bit indices fit in u32");
                    self.netlist.add_constant(value.bit(bit))
                })
                .collect()),
            ExprKind::Unary {
                op: UnaryOp::BitNot,
                input,
            } => Ok(self
                .lower_expression_range(input, lsb, width)?
                .into_iter()
                .map(|net| self.netlist.add_not(net))
                .collect()),
            ExprKind::Binary { op, lhs, rhs }
                if matches!(op, BinaryOp::And | BinaryOp::Or | BinaryOp::Xor) =>
            {
                let lhs = self.lower_expression_range(lhs, lsb, width)?;
                let rhs = self.lower_expression_range(rhs, lsb, width)?;
                Ok(self.lower_binary(op, &lhs, &rhs))
            }
            ExprKind::Mux {
                condition,
                then_expr,
                else_expr,
            } => {
                let condition = self.lower_expression(condition)?[0];
                let then_bits = self.lower_expression_range(then_expr, lsb, width)?;
                let else_bits = self.lower_expression_range(else_expr, lsb, width)?;
                Ok(then_bits
                    .into_iter()
                    .zip(else_bits)
                    .map(|(then_net, else_net)| self.netlist.add_mux(condition, then_net, else_net))
                    .collect())
            }
            ExprKind::Slice {
                input,
                lsb: inner_lsb,
            } => self.lower_expression_range(input, inner_lsb as usize + lsb, width),
            ExprKind::Concat(parts) => {
                let range_end = lsb + width;
                let mut part_lsb = 0;
                let mut bits = Vec::with_capacity(width);
                for part in parts.into_iter().rev() {
                    let part_width = self.module.expressions()[part.index() as usize]
                        .r#type()
                        .width
                        .get() as usize;
                    let part_end = part_lsb + part_width;
                    let overlap_lsb = lsb.max(part_lsb);
                    let overlap_end = range_end.min(part_end);
                    if overlap_lsb < overlap_end {
                        bits.extend(self.lower_expression_range(
                            part,
                            overlap_lsb - part_lsb,
                            overlap_end - overlap_lsb,
                        )?);
                    }
                    part_lsb = part_end;
                    if part_lsb >= range_end {
                        break;
                    }
                }
                Ok(bits)
            }
            _ => {
                let bits = self.lower_expression(id)?;
                Ok(bits[lsb..lsb + width].to_vec())
            }
        }
    }

    fn lower_binary(&mut self, op: BinaryOp, lhs: &[NetId], rhs: &[NetId]) -> Vec<NetId> {
        match op {
            BinaryOp::And => self.bitwise(lhs, rhs, Netlist::add_and),
            BinaryOp::Or => self.bitwise(lhs, rhs, Netlist::add_or),
            BinaryOp::Xor => self.bitwise(lhs, rhs, Netlist::add_xor),
            BinaryOp::Add => self
                .netlist
                .add_arithmetic(ArithmeticOp::Add, lhs, rhs)
                .expect("validated RTL arithmetic has equal, non-zero widths"),
            BinaryOp::Sub => self
                .netlist
                .add_arithmetic(ArithmeticOp::Subtract, lhs, rhs)
                .expect("validated RTL arithmetic has equal, non-zero widths"),
            BinaryOp::Equal => vec![self.equal_words(lhs, rhs)],
            BinaryOp::NotEqual => {
                let equal = self.equal_words(lhs, rhs);
                vec![self.netlist.add_not(equal)]
            }
            BinaryOp::LessThanUnsigned => self.compare(ComparisonOp::LessThanUnsigned, lhs, rhs),
            BinaryOp::LessThanSigned => self.compare(ComparisonOp::LessThanSigned, lhs, rhs),
            BinaryOp::LessOrEqualUnsigned => {
                self.compare(ComparisonOp::LessOrEqualUnsigned, lhs, rhs)
            }
            BinaryOp::LessOrEqualSigned => self.compare(ComparisonOp::LessOrEqualSigned, lhs, rhs),
            BinaryOp::GreaterThanUnsigned => self.compare(ComparisonOp::LessThanUnsigned, rhs, lhs),
            BinaryOp::GreaterThanSigned => self.compare(ComparisonOp::LessThanSigned, rhs, lhs),
            BinaryOp::GreaterOrEqualUnsigned => {
                self.compare(ComparisonOp::LessOrEqualUnsigned, rhs, lhs)
            }
            BinaryOp::GreaterOrEqualSigned => {
                self.compare(ComparisonOp::LessOrEqualSigned, rhs, lhs)
            }
            BinaryOp::ShiftLeft => self.shift_left(lhs, rhs),
            BinaryOp::ShiftRightLogical => self.shift_right(lhs, rhs, false),
            BinaryOp::ShiftRightArithmetic => self.shift_right(lhs, rhs, true),
        }
    }

    fn equal_words(&mut self, lhs: &[NetId], rhs: &[NetId]) -> NetId {
        let same = lhs
            .iter()
            .zip(rhs)
            .map(|(&lhs, &rhs)| {
                let different = self.netlist.add_xor(lhs, rhs);
                self.netlist.add_not(different)
            })
            .collect::<Vec<_>>();
        self.reduce_tree(&same, true, Netlist::add_and)
    }

    fn compare(&mut self, operation: ComparisonOp, lhs: &[NetId], rhs: &[NetId]) -> Vec<NetId> {
        vec![
            self.netlist
                .add_comparison(operation, lhs, rhs)
                .expect("validated RTL comparisons have equal, non-zero widths"),
        ]
    }

    fn reduce_tree(
        &mut self,
        bits: &[NetId],
        identity: bool,
        operation: fn(&mut Netlist, NetId, NetId) -> NetId,
    ) -> NetId {
        if bits.is_empty() {
            return self.netlist.add_constant(identity);
        }
        let mut level = bits.to_vec();
        while level.len() > 1 {
            let mut next = Vec::with_capacity(level.len().div_ceil(2));
            for pair in level.chunks(2) {
                if let [lhs, rhs] = pair {
                    next.push(operation(&mut self.netlist, *lhs, *rhs));
                } else {
                    next.push(pair[0]);
                }
            }
            level = next;
        }
        level[0]
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

    fn reduce_or(&mut self, bits: &[NetId]) -> NetId {
        self.reduce_tree(bits, false, Netlist::add_or)
    }

    fn reduce_and(&mut self, bits: &[NetId]) -> NetId {
        self.reduce_tree(bits, true, Netlist::add_and)
    }

    fn reduce_xor(&mut self, bits: &[NetId]) -> NetId {
        self.reduce_tree(bits, false, Netlist::add_xor)
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
    pipeline_with_options(SynthesisOptions::default())
}

/// Returns the development pipeline selected by `options`.
#[must_use]
pub fn pipeline_with_options(options: SynthesisOptions) -> Pipeline {
    let mut pipeline = Pipeline::new();
    if options.infer_register_enables {
        pipeline.push(InferRegisterEnables);
    }
    if options.relax_qualified_register_enables {
        pipeline.push(RelaxQualifiedRegisterEnables);
    }
    pipeline.push(ValidateNetlist);
    pipeline
}

/// Replaces a direct register self-hold mux with the equivalent clock enable.
pub struct InferRegisterEnables;

impl Pass for InferRegisterEnables {
    fn name(&self) -> &'static str {
        "infer-register-enables"
    }

    fn run(&self, design: &mut Netlist) -> Result<PassReport, SynthesisError> {
        let rewrites = design
            .registers()
            .iter()
            .enumerate()
            .filter_map(|(index, register)| {
                if register.enable().is_some() {
                    return None;
                }
                let node = design.nodes().get(register.data().index() as usize)?;
                if node.output() != register.data() || !matches!(node.kind(), NodeKind::Mux) {
                    return None;
                }
                let [condition, then_net, else_net] = node.inputs() else {
                    unreachable!("validated mux nodes have three inputs");
                };
                if *else_net == register.output() {
                    Some((
                        index,
                        *then_net,
                        EnableControl {
                            signal: *condition,
                            active: ActiveLevel::High,
                        },
                    ))
                } else if *then_net == register.output() {
                    Some((
                        index,
                        *else_net,
                        EnableControl {
                            signal: *condition,
                            active: ActiveLevel::Low,
                        },
                    ))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        for (index, data, enable) in &rewrites {
            design.registers_mut()[*index].set_data_and_enable(*data, Some(*enable));
        }
        Ok(PassReport {
            pass: self.name(),
            message: format!(
                "converted {} register feedback muxes to clock enables",
                rewrites.len()
            ),
        })
    }
}

/// Removes a payload clock enable when a companion valid register proves that
/// the payload is unobservable while the enable is inactive.
///
/// This is a conservative sequential don't-care optimization. A candidate is
/// only rewritten when its enable is also the D input of a same-clock,
/// reset-to-zero valid register, and a structural influence analysis proves
/// that every state or output sink masks the candidate while that valid bit is
/// low. Candidates whose next value depends on their old value are retained.
pub struct RelaxQualifiedRegisterEnables;

impl Pass for RelaxQualifiedRegisterEnables {
    fn name(&self) -> &'static str {
        "relax-qualified-register-enables"
    }

    fn run(&self, design: &mut Netlist) -> Result<PassReport, SynthesisError> {
        let potential = design
            .registers()
            .iter()
            .enumerate()
            .filter_map(|(index, register)| {
                let enable = register.enable()?;
                if enable.active != ActiveLevel::High {
                    return None;
                }
                let qualifiers = design
                    .registers()
                    .iter()
                    .filter(|qualifier| {
                        qualifier.output() != register.output()
                            && qualifier.data() == enable.signal
                            && qualifier.enable().is_none()
                            && qualifier.clock() == register.clock()
                            && qualifier.edge() == register.edge()
                            && qualifier.reset() == register.reset()
                            && qualifier.reset().is_some_and(|reset| !reset.value)
                    })
                    .map(RegisterCell::output)
                    .collect::<Vec<_>>();
                (!qualifiers.is_empty()).then_some((index, register.data(), qualifiers))
            })
            .collect::<Vec<_>>();
        let potentially_relaxed = potential
            .iter()
            .map(|(index, _, _)| *index)
            .collect::<HashSet<_>>();
        let candidates = potential
            .into_iter()
            .filter(|(index, _, qualifiers)| {
                let payload = &design.registers()[*index];
                qualifiers.iter().any(|qualifier| {
                    qualified_payload_is_unobservable(
                        design,
                        payload,
                        *qualifier,
                        &potentially_relaxed,
                    )
                })
            })
            .map(|(index, data, _)| (index, data))
            .collect::<Vec<_>>();

        for (index, data) in &candidates {
            design.registers_mut()[*index].set_data_and_enable(*data, None);
        }

        Ok(PassReport {
            pass: self.name(),
            message: format!(
                "removed {} clock enables from valid-qualified payload registers",
                candidates.len()
            ),
        })
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Influence {
    known: Option<bool>,
    tainted: bool,
}

fn qualified_payload_is_unobservable(
    design: &Netlist,
    payload: &RegisterCell,
    qualifier: NetId,
    potentially_relaxed: &HashSet<usize>,
) -> bool {
    let influence = influence_with_qualifier_low(design, payload.output(), qualifier);

    if net_is_tainted(&influence, payload.data()) {
        return false;
    }

    if design
        .registers()
        .iter()
        .enumerate()
        .any(|(index, register)| {
            net_is_tainted(&influence, register.clock())
                || register
                    .reset()
                    .is_some_and(|reset| net_is_tainted(&influence, reset.signal))
                || register
                    .enable()
                    .is_some_and(|enable| net_is_tainted(&influence, enable.signal))
                || ((potentially_relaxed.contains(&index)
                    || !register_enable_is_inactive(register, &influence))
                    && net_is_tainted(&influence, register.data()))
        })
    {
        return false;
    }

    if design.memories().iter().any(|memory| {
        net_is_tainted(&influence, memory.clock())
            || net_is_tainted(&influence, memory.write_enable().signal)
            || memory
                .read_enable()
                .is_some_and(|enable| net_is_tainted(&influence, enable.signal))
            || (!control_is_inactive(memory.write_enable(), &influence)
                && memory
                    .write_address()
                    .iter()
                    .chain(memory.write_data())
                    .any(|net| net_is_tainted(&influence, *net)))
            || (!memory
                .read_enable()
                .is_some_and(|enable| control_is_inactive(enable, &influence))
                && memory
                    .read_address()
                    .iter()
                    .any(|net| net_is_tainted(&influence, *net)))
            || memory.second_port().is_some_and(|port| {
                net_is_tainted(&influence, port.clock())
                    || net_is_tainted(&influence, port.write_enable().signal)
                    || port
                        .read_enable()
                        .is_some_and(|enable| net_is_tainted(&influence, enable.signal))
                    || (!control_is_inactive(port.write_enable(), &influence)
                        && port
                            .write_address()
                            .iter()
                            .chain(port.write_data())
                            .any(|net| net_is_tainted(&influence, *net)))
                    || (!port
                        .read_enable()
                        .is_some_and(|enable| control_is_inactive(enable, &influence))
                        && port
                            .read_address()
                            .iter()
                            .any(|net| net_is_tainted(&influence, *net)))
            })
    }) {
        return false;
    }

    !design.nodes().iter().any(|node| {
        matches!(node.kind(), NodeKind::Output(_)) && net_is_tainted(&influence, node.output())
    })
}

fn register_enable_is_inactive(register: &RegisterCell, influence: &[Influence]) -> bool {
    register
        .enable()
        .is_some_and(|enable| control_is_inactive(enable, influence))
}

fn control_is_inactive(control: EnableControl, influence: &[Influence]) -> bool {
    influence[control.signal.index() as usize]
        .known
        .is_some_and(|value| value != (control.active == ActiveLevel::High))
}

fn net_is_tainted(influence: &[Influence], net: NetId) -> bool {
    influence[net.index() as usize].tainted
}

fn influence_with_qualifier_low(
    design: &Netlist,
    payload: NetId,
    qualifier: NetId,
) -> Vec<Influence> {
    let mut cell_inputs = HashMap::<NetId, Vec<NetId>>::new();
    for cell in design.arithmetic() {
        let inputs = cell
            .lhs()
            .iter()
            .chain(cell.rhs())
            .copied()
            .chain(cell.carry_in())
            .collect::<Vec<NetId>>();
        for output in cell.outputs() {
            cell_inputs.insert(*output, inputs.clone());
        }
    }
    for cell in design.comparisons() {
        cell_inputs.insert(
            cell.output(),
            cell.lhs().iter().chain(cell.rhs()).copied().collect(),
        );
    }

    let mut result = vec![Influence::default(); design.nodes().len()];
    for node in design.nodes() {
        let output = node.output();
        let value = if output == qualifier {
            Influence {
                known: Some(false),
                tainted: false,
            }
        } else if output == payload {
            Influence {
                known: None,
                tainted: true,
            }
        } else {
            match node.kind() {
                NodeKind::Constant(value) => Influence {
                    known: Some(*value),
                    tainted: false,
                },
                NodeKind::Not => {
                    let input = result[node.inputs()[0].index() as usize];
                    Influence {
                        known: input.known.map(|value| !value),
                        tainted: input.tainted,
                    }
                }
                NodeKind::And => influence_and(
                    result[node.inputs()[0].index() as usize],
                    result[node.inputs()[1].index() as usize],
                ),
                NodeKind::Or => influence_or(
                    result[node.inputs()[0].index() as usize],
                    result[node.inputs()[1].index() as usize],
                ),
                NodeKind::Xor => influence_xor(
                    result[node.inputs()[0].index() as usize],
                    result[node.inputs()[1].index() as usize],
                ),
                NodeKind::Mux => influence_mux(
                    result[node.inputs()[0].index() as usize],
                    result[node.inputs()[1].index() as usize],
                    result[node.inputs()[2].index() as usize],
                ),
                NodeKind::Output(_) => result[node.inputs()[0].index() as usize],
                NodeKind::ArithmeticOutput(_) | NodeKind::ComparisonOutput(_) => Influence {
                    known: None,
                    tainted: cell_inputs.get(&output).is_none_or(|inputs| {
                        inputs
                            .iter()
                            .any(|net| result[net.index() as usize].tainted)
                    }),
                },
                NodeKind::Input(_) | NodeKind::RegisterOutput(_) | NodeKind::MemoryOutput(_) => {
                    Influence::default()
                }
            }
        };
        result[output.index() as usize] = value;
    }
    result
}

fn influence_and(lhs: Influence, rhs: Influence) -> Influence {
    let known = if lhs.known == Some(false) || rhs.known == Some(false) {
        Some(false)
    } else if lhs.known == Some(true) && rhs.known == Some(true) {
        Some(true)
    } else {
        None
    };
    Influence {
        known,
        tainted: known != Some(false) && (lhs.tainted || rhs.tainted),
    }
}

fn influence_or(lhs: Influence, rhs: Influence) -> Influence {
    let known = if lhs.known == Some(true) || rhs.known == Some(true) {
        Some(true)
    } else if lhs.known == Some(false) && rhs.known == Some(false) {
        Some(false)
    } else {
        None
    };
    Influence {
        known,
        tainted: known != Some(true) && (lhs.tainted || rhs.tainted),
    }
}

fn influence_xor(lhs: Influence, rhs: Influence) -> Influence {
    Influence {
        known: lhs.known.zip(rhs.known).map(|(lhs, rhs)| lhs ^ rhs),
        tainted: lhs.tainted || rhs.tainted,
    }
}

fn influence_mux(condition: Influence, then_value: Influence, else_value: Influence) -> Influence {
    if let Some(condition) = condition.known {
        return if condition { then_value } else { else_value };
    }
    Influence {
        known: (then_value.known == else_value.known)
            .then_some(then_value.known)
            .flatten(),
        tainted: condition.tainted || then_value.tainted || else_value.tainted,
    }
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
    /// A checked synthesis transformation could not be constructed.
    Transformation(String),
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
            Self::Transformation(message) => {
                write!(formatter, "synthesis transformation failed: {message}")
            }
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
            | Self::NonConstantReset { .. }
            | Self::Transformation(_) => None,
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

    use struo_ir::{
        ActiveLevel, ArithmeticOp, ClockEdge as IrClockEdge, ComparisonOp, NetId, Netlist,
        NodeKind, RegisterCell, ResetControl,
    };
    use struo_rtl::{
        BinaryOp, BitWidth, ClockEdge, Constant, Design, Enable, Memory, Module, Polarity, Port,
        PortDirection, Register, Reset, ResetMode, StateDomain, UnaryOp, ValueType,
    };

    use super::{SynthesisOptions, default_pipeline, pipeline_with_options, synthesize};

    fn bits(width: u32) -> ValueType {
        ValueType {
            width: BitWidth::new(width).unwrap(),
            signed: false,
            state: StateDomain::TwoState,
        }
    }

    fn arithmetic_design(width: u32, operation: BinaryOp) -> Design {
        let mut module = Module::new("Arithmetic");
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
        let sum = module.binary(operation, lhs, rhs).unwrap();
        module.assign(module.whole(output).unwrap(), sum).unwrap();
        let mut design = Design::new("Arithmetic");
        design.add_module(module);
        design
    }

    fn add_with_carry_design(width: u32) -> Design {
        let mut module = Module::new("AddWithCarry");
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
        let carry = module.add_port(Port {
            name: "carry".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let output = module.add_port(Port {
            name: "sum".into(),
            direction: PortDirection::Output,
            r#type: bits(width),
        });
        let lhs = module.read(lhs).unwrap();
        let rhs = module.read(rhs).unwrap();
        let carry = module.read(carry).unwrap();
        let zeros = module.constant(Constant::from_u64(BitWidth::new(width - 1).unwrap(), 0));
        let carry = module.concat(vec![zeros, carry]).unwrap();
        let operands = module.binary(BinaryOp::Add, lhs, rhs).unwrap();
        let sum = module.binary(BinaryOp::Add, operands, carry).unwrap();
        module.assign(module.whole(output).unwrap(), sum).unwrap();
        let mut design = Design::new("AddWithCarry");
        design.add_module(module);
        design
    }

    #[test]
    fn retains_synchronous_memory_for_block_ram_mapping() {
        let mut module = Module::new("Scratchpad");
        let clock = module.add_port(Port {
            name: "clk".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let write_enable = module.add_port(Port {
            name: "write_enable".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let read_address_signal = module.add_port(Port {
            name: "read_address".into(),
            direction: PortDirection::Input,
            r#type: bits(8),
        });
        let write_address_signal = module.add_port(Port {
            name: "write_address".into(),
            direction: PortDirection::Input,
            r#type: bits(8),
        });
        let write_data_signal = module.add_port(Port {
            name: "write_data".into(),
            direction: PortDirection::Input,
            r#type: bits(8),
        });
        let read_data = module.add_port(Port {
            name: "read_data".into(),
            direction: PortDirection::Output,
            r#type: bits(8),
        });
        let read_address = module.read(read_address_signal).unwrap();
        let write_address = module.read(write_address_signal).unwrap();
        let write_data = module.read(write_data_signal).unwrap();
        module.add_memory(Memory {
            name: "words".into(),
            word: bits(8),
            depth: 256,
            style: struo_rtl::MemoryStyle::Auto,
            read_latency: 1,
            read_address,
            read_data,
            read_enable: None,
            write_address,
            write_data,
            write_enable: Enable {
                signal: write_enable,
                polarity: Polarity::ActiveHigh,
            },
            clock,
            edge: ClockEdge::Rising,
            second_port: None,
        });
        let mut design = Design::new("Scratchpad");
        design.add_module(module);

        let synthesized = synthesize(&design).unwrap();

        assert_eq!(synthesized.netlist.memories().len(), 1);
        let memory = &synthesized.netlist.memories()[0];
        assert_eq!(memory.name(), "words");
        assert_eq!(memory.depth(), 256);
        assert_eq!(memory.read_data().len(), 8);
        assert_eq!(memory.write_data().len(), 8);
        assert_eq!(synthesized.netlist.registers().len(), 0);
    }

    #[test]
    fn lowers_wrapping_adder() {
        let synthesized = synthesize(&arithmetic_design(4, BinaryOp::Add)).unwrap();
        assert_eq!(synthesized.netlist.registers().len(), 0);
        assert_eq!(synthesized.netlist.arithmetic().len(), 1);
        assert_eq!(
            synthesized.netlist.arithmetic()[0].operation(),
            ArithmeticOp::Add
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
    fn folds_zero_extended_carry_into_one_adder() {
        let synthesized = synthesize(&add_with_carry_design(4)).unwrap();
        assert_eq!(synthesized.netlist.arithmetic().len(), 1);
        assert!(synthesized.netlist.arithmetic()[0].carry_in().is_some());

        for lhs in 0..16 {
            for rhs in 0..16 {
                for carry in 0..2 {
                    let inputs = input_word("lhs", 4, lhs)
                        .into_iter()
                        .chain(input_word("rhs", 4, rhs))
                        .chain([("carry".into(), carry != 0)])
                        .collect();
                    let outputs = evaluate_combinational(&synthesized.netlist, &inputs);
                    assert_eq!(output_word(&outputs, "sum", 4), (lhs + rhs + carry) & 0xf);
                }
            }
        }
    }

    #[test]
    fn retains_wrapping_subtraction() {
        let synthesized = synthesize(&arithmetic_design(4, BinaryOp::Sub)).unwrap();
        assert_eq!(synthesized.netlist.arithmetic().len(), 1);
        assert_eq!(
            synthesized.netlist.arithmetic()[0].operation(),
            ArithmeticOp::Subtract
        );
        for lhs in 0u64..16 {
            for rhs in 0u64..16 {
                let inputs = input_word("lhs", 4, lhs)
                    .into_iter()
                    .chain(input_word("rhs", 4, rhs))
                    .collect();
                let outputs = evaluate_combinational(&synthesized.netlist, &inputs);
                assert_eq!(output_word(&outputs, "sum", 4), lhs.wrapping_sub(rhs) & 0xf);
            }
        }
    }

    #[test]
    fn ignores_discarded_self_references_beneath_slices() {
        let mut module = Module::new("FullyAssignedSlices");
        let low = module.add_port(Port {
            name: "low".into(),
            direction: PortDirection::Input,
            r#type: bits(8),
        });
        let high = module.add_port(Port {
            name: "high".into(),
            direction: PortDirection::Input,
            r#type: bits(8),
        });
        let output = module.add_port(Port {
            name: "output".into(),
            direction: PortDirection::Output,
            r#type: bits(16),
        });

        let previous = module.read(output).unwrap();
        let previous_high = module
            .expression_slice(previous, 8, BitWidth::new(8).unwrap())
            .unwrap();
        let low = module.read(low).unwrap();
        let partially_assigned = module.concat(vec![previous_high, low]).unwrap();
        let assigned_low = module
            .expression_slice(partially_assigned, 0, BitWidth::new(8).unwrap())
            .unwrap();
        let high = module.read(high).unwrap();
        let fully_assigned = module.concat(vec![high, assigned_low]).unwrap();
        module
            .assign(module.whole(output).unwrap(), fully_assigned)
            .unwrap();
        let mut design = Design::new("FullyAssignedSlices");
        design.add_module(module);

        let synthesized = synthesize(&design).unwrap();
        let inputs = input_word("low", 8, 0xaa)
            .into_iter()
            .chain(input_word("high", 8, 0xbb))
            .collect();
        let outputs = evaluate_combinational(&synthesized.netlist, &inputs);
        assert_eq!(output_word(&outputs, "output", 16), 0xbbaa);
    }

    #[test]
    fn ignores_discarded_self_references_beneath_vector_muxes() {
        let mut module = Module::new("FullyAssignedMuxSlices");
        let select = module.add_port(Port {
            name: "select".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let low_true = module.add_port(Port {
            name: "low_true".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let low_false = module.add_port(Port {
            name: "low_false".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let high = module.add_port(Port {
            name: "high".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let output = module.add_port(Port {
            name: "output".into(),
            direction: PortDirection::Output,
            r#type: bits(2),
        });

        let previous = module.read(output).unwrap();
        let previous_high = module
            .expression_slice(previous, 1, BitWidth::new(1).unwrap())
            .unwrap();
        let low_true = module.read(low_true).unwrap();
        let true_partial = module.concat(vec![previous_high, low_true]).unwrap();
        let low_false = module.read(low_false).unwrap();
        let false_partial = module.concat(vec![previous_high, low_false]).unwrap();
        let select = module.read(select).unwrap();
        let partial_mux = module.mux(select, true_partial, false_partial).unwrap();
        let assigned_low = module
            .expression_slice(partial_mux, 0, BitWidth::new(1).unwrap())
            .unwrap();
        let high = module.read(high).unwrap();
        let fully_assigned = module.concat(vec![high, assigned_low]).unwrap();
        module
            .assign(module.whole(output).unwrap(), fully_assigned)
            .unwrap();
        let mut design = Design::new("FullyAssignedMuxSlices");
        design.add_module(module);

        let synthesized = synthesize(&design).unwrap();
        for (select, expected) in [(false, 0b10), (true, 0b11)] {
            let inputs = HashMap::from([
                ("select".into(), select),
                ("low_true".into(), true),
                ("low_false".into(), false),
                ("high".into(), true),
            ]);
            let outputs = evaluate_combinational(&synthesized.netlist, &inputs);
            assert_eq!(output_word(&outputs, "output", 2), expected);
        }
    }

    #[test]
    fn balances_wide_reduction_logic() {
        let mut module = Module::new("WideReduction");
        let input_signal = module.add_port(Port {
            name: "input".into(),
            direction: PortDirection::Input,
            r#type: bits(32),
        });
        let output = module.add_port(Port {
            name: "all".into(),
            direction: PortDirection::Output,
            r#type: bits(1),
        });
        let input = module.read(input_signal).unwrap();
        let reduced = module.unary(UnaryOp::ReduceAnd, input).unwrap();
        module
            .assign(module.whole(output).unwrap(), reduced)
            .unwrap();
        let mut design = Design::new("WideReduction");
        design.add_module(module);

        let synthesized = synthesize(&design).unwrap();
        assert!(max_combinational_depth(&synthesized.netlist) <= 5);
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
    fn infers_clock_enables_from_register_self_hold_muxes() {
        for (polarity, hold_when_true) in [(ActiveLevel::High, false), (ActiveLevel::Low, true)] {
            let mut module = Module::new("EnabledRegister");
            let clock = module.add_port(Port {
                name: "clk".into(),
                direction: PortDirection::Input,
                r#type: bits(1),
            });
            let update_signal = module.add_port(Port {
                name: "update".into(),
                direction: PortDirection::Input,
                r#type: bits(1),
            });
            let data_signal = module.add_port(Port {
                name: "data".into(),
                direction: PortDirection::Input,
                r#type: bits(1),
            });
            let state_signal = module.add_port(Port {
                name: "state".into(),
                direction: PortDirection::Output,
                r#type: bits(1),
            });
            let update = module.read(update_signal).unwrap();
            let data = module.read(data_signal).unwrap();
            let state = module.read(state_signal).unwrap();
            let next = if hold_when_true {
                module.mux(update, state, data).unwrap()
            } else {
                module.mux(update, data, state).unwrap()
            };
            module
                .add_register(Register {
                    name: "state_reg".into(),
                    target: state_signal,
                    next,
                    clock,
                    edge: ClockEdge::Rising,
                    enable: None,
                    reset: None,
                })
                .unwrap();
            let mut design = Design::new("EnabledRegister");
            design.add_module(module);

            let synthesized = synthesize(&design).unwrap();
            let register = &synthesized.netlist.registers()[0];
            let enable = register.enable().unwrap();

            assert_eq!(enable.active, polarity);
            assert!(matches!(
                synthesized.netlist.nodes()[enable.signal.index() as usize].kind(),
                NodeKind::Input(name) if name == "update"
            ));
            assert!(matches!(
                synthesized.netlist.nodes()[register.data().index() as usize].kind(),
                NodeKind::Input(name) if name == "data"
            ));
            assert_eq!(
                synthesized.reports[1].message,
                "converted 1 register feedback muxes to clock enables"
            );
        }
    }

    #[test]
    fn preserves_explicit_clock_enable_over_feedback_mux_inference() {
        let mut module = Module::new("ExplicitEnable");
        let clock = module.add_port(Port {
            name: "clk".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let outer_enable = module.add_port(Port {
            name: "outer_enable".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let inner_enable = module.add_port(Port {
            name: "inner_enable".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let data_signal = module.add_port(Port {
            name: "data".into(),
            direction: PortDirection::Input,
            r#type: bits(1),
        });
        let state_signal = module.add_port(Port {
            name: "state".into(),
            direction: PortDirection::Output,
            r#type: bits(1),
        });
        let inner_enable_value = module.read(inner_enable).unwrap();
        let data = module.read(data_signal).unwrap();
        let state = module.read(state_signal).unwrap();
        let next = module.mux(inner_enable_value, data, state).unwrap();
        module
            .add_register(Register {
                name: "state_reg".into(),
                target: state_signal,
                next,
                clock,
                edge: ClockEdge::Rising,
                enable: Some(Enable {
                    signal: outer_enable,
                    polarity: Polarity::ActiveHigh,
                }),
                reset: None,
            })
            .unwrap();
        let mut design = Design::new("ExplicitEnable");
        design.add_module(module);

        let synthesized = synthesize(&design).unwrap();
        let register = &synthesized.netlist.registers()[0];
        let enable = register.enable().unwrap();

        assert!(matches!(
            synthesized.netlist.nodes()[enable.signal.index() as usize].kind(),
            NodeKind::Input(name) if name == "outer_enable"
        ));
        assert!(matches!(
            synthesized.netlist.nodes()[register.data().index() as usize].kind(),
            NodeKind::Mux
        ));
        assert_eq!(
            synthesized.reports[1].message,
            "converted 0 register feedback muxes to clock enables"
        );
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

        assert_eq!(reports.len(), 3);
        assert_eq!(reports[0].pass, "infer-register-enables");
        assert_eq!(reports[1].pass, "relax-qualified-register-enables");
        assert_eq!(reports[2].pass, "validate");
    }

    #[test]
    fn synthesis_options_can_disable_register_enable_passes() {
        let mut design = Netlist::new("feedback_mux");
        let clock = design.add_input("clock");
        let update = design.add_input("update");
        let data = design.add_input("data");
        let state = design.add_register_output("state");
        let next = design.add_mux(update, data, state);
        design.add_register(RegisterCell::new(
            "state",
            state,
            next,
            clock,
            IrClockEdge::Rising,
            None,
            None,
        ));
        design.add_output("state_out", state);

        let reports = pipeline_with_options(SynthesisOptions {
            infer_register_enables: false,
            relax_qualified_register_enables: false,
        })
        .run(&mut design)
        .unwrap();

        assert_eq!(reports.len(), 1);
        assert_eq!(reports[0].pass, "validate");
        assert_eq!(design.registers()[0].enable(), None);
        assert!(matches!(
            design.nodes()[design.registers()[0].data().index() as usize].kind(),
            NodeKind::Mux
        ));
    }

    #[test]
    fn removes_enable_from_valid_qualified_payload() {
        let mut design = qualified_payload_netlist(false, false);

        let reports = default_pipeline().run(&mut design).unwrap();
        let payload = design
            .registers()
            .iter()
            .find(|register| register.name() == "payload")
            .unwrap();

        assert_eq!(payload.enable(), None);
        assert_eq!(
            reports[1].message,
            "removed 1 clock enables from valid-qualified payload registers"
        );
    }

    #[test]
    fn retains_qualified_enable_when_payload_is_observable_or_self_dependent() {
        for (direct_output, self_dependent) in [(true, false), (false, true)] {
            let mut design = qualified_payload_netlist(direct_output, self_dependent);

            let reports = default_pipeline().run(&mut design).unwrap();
            let payload = design
                .registers()
                .iter()
                .find(|register| register.name() == "payload")
                .unwrap();

            assert!(payload.enable().is_some());
            assert_eq!(
                reports[1].message,
                "removed 0 clock enables from valid-qualified payload registers"
            );
        }
    }

    #[test]
    fn does_not_treat_payload_as_its_own_qualifier() {
        let mut design = Netlist::new("self_qualified_payload");
        let clock = design.add_input("clock");
        let reset = design.add_input("reset");
        let update = design.add_input("update");
        let payload = design.add_register_output("payload");
        let payload_next = design.add_mux(update, update, payload);
        design.add_register(RegisterCell::new(
            "payload",
            payload,
            payload_next,
            clock,
            IrClockEdge::Rising,
            None,
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::High,
                asynchronous: true,
                value: false,
            }),
        ));
        design.add_output("payload_out", payload);

        let reports = default_pipeline().run(&mut design).unwrap();

        assert!(design.registers()[0].enable().is_some());
        assert_eq!(
            reports[1].message,
            "removed 0 clock enables from valid-qualified payload registers"
        );
    }

    fn qualified_payload_netlist(direct_output: bool, self_dependent: bool) -> Netlist {
        let mut design = Netlist::new("qualified_payload");
        let clock = design.add_input("clock");
        let reset = design.add_input("reset");
        let update = design.add_input("update");
        let data = design.add_input("data");
        let valid = design.add_register_output("valid");
        let payload = design.add_register_output("payload");
        let observed = design.add_register_output("observed");
        let reset_control = Some(ResetControl {
            signal: reset,
            active: ActiveLevel::High,
            asynchronous: true,
            value: false,
        });
        let payload_data = if self_dependent {
            design.add_not(payload)
        } else {
            data
        };
        let payload_next = design.add_mux(update, payload_data, payload);
        let observed_next = design.add_mux(valid, payload, observed);
        design.add_register(RegisterCell::new(
            "valid",
            valid,
            update,
            clock,
            IrClockEdge::Rising,
            None,
            reset_control,
        ));
        design.add_register(RegisterCell::new(
            "payload",
            payload,
            payload_next,
            clock,
            IrClockEdge::Rising,
            None,
            reset_control,
        ));
        design.add_register(RegisterCell::new(
            "observed",
            observed,
            observed_next,
            clock,
            IrClockEdge::Rising,
            None,
            reset_control,
        ));
        design.add_output("observed_out", observed);
        if direct_output {
            design.add_output("payload_out", payload);
        }
        design
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
                NodeKind::RegisterOutput(_) | NodeKind::MemoryOutput(_) => {
                    panic!("combinational test contains state")
                }
                NodeKind::ArithmeticOutput(_) => {
                    let (cell, bit) = netlist
                        .arithmetic()
                        .iter()
                        .find_map(|cell| {
                            cell.outputs()
                                .iter()
                                .position(|output| *output == node.output())
                                .map(|bit| (cell, bit))
                        })
                        .expect("arithmetic output belongs to a cell");
                    let width = cell.outputs().len();
                    let lhs = cell
                        .lhs()
                        .iter()
                        .enumerate()
                        .fold(0u128, |word, (bit, net)| {
                            word | (u128::from(values[net.index() as usize]) << bit)
                        });
                    let rhs = cell
                        .rhs()
                        .iter()
                        .enumerate()
                        .fold(0u128, |word, (bit, net)| {
                            word | (u128::from(values[net.index() as usize]) << bit)
                        });
                    let mask = u128::MAX >> (u128::BITS as usize - width);
                    let result = match cell.operation() {
                        ArithmeticOp::Add => lhs.wrapping_add(rhs).wrapping_add(u128::from(
                            cell.carry_in()
                                .is_some_and(|carry| values[carry.index() as usize]),
                        )),
                        ArithmeticOp::Subtract => lhs.wrapping_sub(rhs),
                    } & mask;
                    result & (1 << bit) != 0
                }
                NodeKind::ComparisonOutput(_) => {
                    let cell = netlist
                        .comparisons()
                        .iter()
                        .find(|cell| cell.output() == node.output())
                        .expect("comparison output belongs to a cell");
                    let word = |nets: &[NetId]| {
                        nets.iter().enumerate().fold(0u128, |word, (bit, net)| {
                            word | (u128::from(values[net.index() as usize]) << bit)
                        })
                    };
                    let mut lhs = word(cell.lhs());
                    let mut rhs = word(cell.rhs());
                    if cell.operation().is_signed() {
                        let sign = 1 << (cell.lhs().len() - 1);
                        lhs ^= sign;
                        rhs ^= sign;
                    }
                    match cell.operation() {
                        ComparisonOp::LessThanUnsigned | ComparisonOp::LessThanSigned => lhs < rhs,
                        ComparisonOp::LessOrEqualUnsigned | ComparisonOp::LessOrEqualSigned => {
                            lhs <= rhs
                        }
                    }
                }
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

    fn max_combinational_depth(netlist: &Netlist) -> usize {
        let mut depths = vec![0; netlist.nodes().len()];
        let mut maximum = 0;
        for node in netlist.nodes() {
            let depth = match node.kind() {
                NodeKind::Input(_)
                | NodeKind::Constant(_)
                | NodeKind::RegisterOutput(_)
                | NodeKind::MemoryOutput(_) => 0,
                NodeKind::ArithmeticOutput(_) => {
                    netlist
                        .arithmetic()
                        .iter()
                        .find(|cell| cell.outputs().contains(&node.output()))
                        .into_iter()
                        .flat_map(|cell| {
                            cell.lhs()
                                .iter()
                                .chain(cell.rhs())
                                .copied()
                                .chain(cell.carry_in())
                        })
                        .map(|input| depths[input.index() as usize])
                        .max()
                        .unwrap_or(0)
                        + 1
                }
                NodeKind::ComparisonOutput(_) => {
                    netlist
                        .comparisons()
                        .iter()
                        .find(|cell| cell.output() == node.output())
                        .into_iter()
                        .flat_map(|cell| cell.lhs().iter().chain(cell.rhs()))
                        .map(|input| depths[input.index() as usize])
                        .max()
                        .unwrap_or(0)
                        + 1
                }
                NodeKind::Output(_) => depths[node.inputs()[0].index() as usize],
                NodeKind::And | NodeKind::Or | NodeKind::Xor | NodeKind::Not | NodeKind::Mux => {
                    node.inputs()
                        .iter()
                        .map(|input| depths[input.index() as usize])
                        .max()
                        .unwrap_or(0)
                        + 1
                }
            };
            depths[node.output().index() as usize] = depth;
            maximum = maximum.max(depth);
        }
        maximum
    }
}
