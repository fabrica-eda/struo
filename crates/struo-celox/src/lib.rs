//! Celox simulation artifacts for technology-mapped Struo netlists.
//!
//! This crate is intentionally a backend adapter. Source RTL is simulated by
//! Celox's Veryl frontend; only the post-technology-mapping object is converted
//! into a synthetic [`celox::FrontendArtifact`].

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt::{self, Display, Formatter};

use celox::frontend_sdk::{
    ActiveLevel as CeloxActiveLevel, BinaryOp as CeloxBinaryOp, BuildError, Constant, Edge, ExprId,
    FrontendArtifact, ModuleBuilder, SignalId, UnaryOp, ValueType,
};
use celox::{Simulator, SimulatorBuilder};
use struo_ir::{ActiveLevel, ClockEdge};
use struo_target_ecp5::{Bit, Control, Ecp5Cell, Ecp5Netlist, MappedPortDirection, Reset};

/// Converts the exact ECP5 target object into Celox's external-frontend format.
///
/// LUT truth tables are expanded as mux trees. `TRELLIS_FF` asynchronous resets
/// map directly to the SDK register reset, while synchronous resets are folded
/// into next-state logic with reset-over-enable priority. A dedicated `JTAGG`
/// has no package-pin interface in this artifact, so its fabric outputs use the
/// inactive TAP state; exercise the ordinary top-level ports before target
/// binding when simulating JTAG transactions.
///
/// # Errors
///
/// Returns an error for inconsistent target wiring or an SDK artifact that
/// fails construction or validation.
pub fn ecp5_frontend_artifact(
    netlist: &Ecp5Netlist,
) -> Result<FrontendArtifact, CeloxAdapterError> {
    let bit_type = ValueType::bits(1)?;
    let mut builder = ModuleBuilder::new(netlist.name())?;
    let mut wires = BTreeMap::new();
    let mut outputs = Vec::new();
    let flip_flop_banks = collect_flip_flop_banks(netlist.cells());

    for port in netlist.ports() {
        let port_type = ValueType::bits(port.bits.len())?;
        let signal = match port.direction {
            MappedPortDirection::Input | MappedPortDirection::Inout => {
                builder.input(&port.name, port_type)?
            }
            MappedPortDirection::Output => builder.output(&port.name, port_type)?,
        };
        match port.direction {
            MappedPortDirection::Input | MappedPortDirection::Inout => {
                for (lsb, bit) in port.bits.iter().enumerate() {
                    insert_wire(
                        &mut wires,
                        *bit,
                        WireRef {
                            signal,
                            lsb,
                            signal_width: port.bits.len(),
                        },
                    )?;
                }
            }
            MappedPortDirection::Output => outputs.push((signal, port.bits.clone())),
        }
    }

    for cell in netlist.cells() {
        reserve_cell_output(&mut builder, &mut wires, cell, bit_type)?;
    }
    reserve_flip_flop_banks(&mut builder, &mut wires, &flip_flop_banks)?;

    let constant_zero_signal = builder.internal("__struo_constant_zero", bit_type)?;
    let constant_one_signal = builder.internal("__struo_constant_one", bit_type)?;
    let constant_zero = builder.constant(Constant::two_state(0u8, 1)?);
    let constant_one = builder.constant(Constant::two_state(1u8, 1)?);
    builder.assign(builder.whole(constant_zero_signal)?, constant_zero)?;
    builder.assign(builder.whole(constant_one_signal)?, constant_one)?;

    let constants = Constants {
        zero_expression: constant_zero,
        one_expression: constant_one,
        zero_signal: constant_zero_signal,
        one_signal: constant_one_signal,
    };

    for cell in netlist.cells() {
        emit_cell(&mut builder, &wires, constants, cell)?;
    }
    for bank in &flip_flop_banks {
        emit_flip_flop_bank(&mut builder, &wires, constants, bank)?;
    }

    emit_outputs(&mut builder, &wires, constants, outputs)?;
    finish_artifact(builder)
}

/// Creates an in-memory Celox simulator builder for an ECP5 mapped netlist.
///
/// The mapped object is converted directly into a [`FrontendArtifact`] and
/// handed to Celox without JSON serialization or parsing.
///
/// # Errors
///
/// Returns an error when the mapped object cannot be represented by the Celox
/// frontend SDK.
pub fn ecp5_simulator(
    netlist: &Ecp5Netlist,
) -> Result<SimulatorBuilder<'static, Simulator>, CeloxAdapterError> {
    Ok(Simulator::from_frontend(ecp5_frontend_artifact(netlist)?))
}

fn finish_artifact(builder: ModuleBuilder) -> Result<FrontendArtifact, CeloxAdapterError> {
    let artifact = builder.finish();
    artifact.validate()?;
    Ok(artifact)
}

fn reserve_cell_output(
    builder: &mut ModuleBuilder,
    wires: &mut BTreeMap<u32, WireRef>,
    cell: &Ecp5Cell,
    bit_type: ValueType,
) -> Result<(), CeloxAdapterError> {
    let scalar = match cell {
        Ecp5Cell::Lut4 { name, output, .. } => {
            Some((*output, format!("__struo_lut_{name}_{output}")))
        }
        Ecp5Cell::FlipFlop { .. } => None,
        Ecp5Cell::Ccu2c {
            name,
            sums,
            carry_out,
            ..
        } => {
            for (label, wire) in [("sum0", sums[0]), ("sum1", sums[1]), ("carry", *carry_out)] {
                reserve_scalar(
                    builder,
                    wires,
                    wire,
                    format!("__struo_ccu_{name}_{label}_{wire}"),
                    bit_type,
                )?;
            }
            None
        }
        Ecp5Cell::BlockRam {
            name, read_data, ..
        } => {
            let signal = builder.internal(
                format!("__struo_{name}_read"),
                ValueType::bits(read_data.len())?,
            )?;
            for (lsb, wire) in read_data.iter().enumerate() {
                insert_wire(
                    wires,
                    Bit::Wire(*wire),
                    WireRef {
                        signal,
                        lsb,
                        signal_width: read_data.len(),
                    },
                )?;
            }
            None
        }
        Ecp5Cell::TrellisIo {
            name, fabric_input, ..
        } => Some((*fabric_input, format!("__struo_io_{name}_{fabric_input}"))),
        Ecp5Cell::Jtagg {
            name,
            tdi,
            clock,
            run_test_idle,
            shift,
            update,
            reset_n,
            clock_enable,
            ..
        } => {
            for (label, wire) in [
                ("tdi", *tdi),
                ("tck", *clock),
                ("rti1", run_test_idle[0]),
                ("rti2", run_test_idle[1]),
                ("shift", *shift),
                ("update", *update),
                ("rst_n", *reset_n),
                ("ce1", clock_enable[0]),
                ("ce2", clock_enable[1]),
            ] {
                reserve_scalar(
                    builder,
                    wires,
                    wire,
                    format!("__struo_jtagg_{name}_{label}_{wire}"),
                    bit_type,
                )?;
            }
            None
        }
    };
    if let Some((wire, name)) = scalar {
        reserve_scalar(builder, wires, wire, name, bit_type)?;
    }
    Ok(())
}

fn reserve_scalar(
    builder: &mut ModuleBuilder,
    wires: &mut BTreeMap<u32, WireRef>,
    wire: u32,
    name: String,
    bit_type: ValueType,
) -> Result<(), CeloxAdapterError> {
    let signal = builder.internal(name, bit_type)?;
    insert_wire(
        wires,
        Bit::Wire(wire),
        WireRef {
            signal,
            lsb: 0,
            signal_width: 1,
        },
    )
}

fn emit_cell(
    builder: &mut ModuleBuilder,
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    cell: &Ecp5Cell,
) -> Result<(), CeloxAdapterError> {
    match cell {
        Ecp5Cell::Lut4 {
            inputs,
            output,
            init,
            ..
        } => {
            let input_expressions = inputs
                .iter()
                .map(|bit| bit_expression(builder, wires, constants, *bit))
                .collect::<Result<Vec<_>, _>>()?;
            let value = lut_expression(builder, &input_expressions, *init, 0, 0, constants)?;
            let target = builder.whole(wire_ref(wires, *output)?.signal)?;
            builder.assign(target, value)?;
            Ok(())
        }
        Ecp5Cell::Ccu2c {
            inputs,
            carry_in,
            sums,
            carry_out,
            init,
            inject,
            ..
        } => emit_ccu2c(
            builder, wires, constants, *inputs, *carry_in, *sums, *carry_out, *init, *inject,
        ),
        Ecp5Cell::FlipFlop { .. } => Ok(()),
        Ecp5Cell::BlockRam {
            name,
            depth,
            word_width,
            physical_width,
            write_address,
            write_data,
            write_enable,
            read_address,
            read_data,
            read_enable,
            clock,
            edge,
        } => emit_block_ram(
            builder,
            wires,
            constants,
            name,
            *depth,
            *word_width,
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
        Ecp5Cell::TrellisIo {
            pad,
            fabric_output,
            fabric_input,
            tristate,
            ..
        } => {
            let pad = bit_expression(builder, wires, constants, Bit::Wire(*pad))?;
            let driven = bit_expression(builder, wires, constants, *fabric_output)?;
            let tristate = bit_expression(builder, wires, constants, *tristate)?;
            let resolved = builder.mux(tristate, pad, driven)?;
            let target = builder.whole(wire_ref(wires, *fabric_input)?.signal)?;
            builder.assign(target, resolved)?;
            Ok(())
        }
        Ecp5Cell::Jtagg {
            tdi,
            clock,
            run_test_idle,
            shift,
            update,
            reset_n,
            clock_enable,
            ..
        } => emit_idle_jtagg(
            builder,
            wires,
            constants,
            [
                *tdi,
                *clock,
                run_test_idle[0],
                run_test_idle[1],
                *shift,
                *update,
                clock_enable[0],
                clock_enable[1],
            ],
            *reset_n,
        ),
    }
}

fn emit_idle_jtagg(
    builder: &mut ModuleBuilder,
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    inactive_outputs: [u32; 8],
    reset_n: u32,
) -> Result<(), CeloxAdapterError> {
    for wire in inactive_outputs {
        let target = builder.whole(wire_ref(wires, wire)?.signal)?;
        builder.assign(target, constants.zero_expression)?;
    }
    let reset_target = builder.whole(wire_ref(wires, reset_n)?.signal)?;
    builder.assign(reset_target, constants.one_expression)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn emit_ccu2c(
    builder: &mut ModuleBuilder,
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    inputs: [[Bit; 4]; 2],
    carry_in: Bit,
    sums: [u32; 2],
    carry_out: u32,
    init: [u16; 2],
    inject: [bool; 2],
) -> Result<(), CeloxAdapterError> {
    let one_bit = ValueType::bits(1)?;
    let mut carry = bit_expression(builder, wires, constants, carry_in)?;
    for slice in 0..2 {
        let expressions = inputs[slice]
            .iter()
            .map(|bit| bit_expression(builder, wires, constants, *bit))
            .collect::<Result<Vec<_>, _>>()?;
        let lut4 = lut_expression(builder, &expressions, init[slice], 0, 0, constants)?;
        let lut2_inputs = [
            expressions[0],
            expressions[1],
            constants.zero_expression,
            constants.zero_expression,
        ];
        let lut2 = lut_expression(builder, &lut2_inputs, init[slice], 0, 0, constants)?;
        let gated_carry = if inject[slice] {
            constants.zero_expression
        } else {
            carry
        };
        let sum = builder.binary(CeloxBinaryOp::Xor, lut4, gated_carry, one_bit)?;
        builder.assign(builder.whole(wire_ref(wires, sums[slice])?.signal)?, sum)?;
        carry = builder.mux(lut4, gated_carry, lut2)?;
    }
    builder.assign(builder.whole(wire_ref(wires, carry_out)?.signal)?, carry)?;
    Ok(())
}

fn emit_outputs(
    builder: &mut ModuleBuilder,
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    outputs: Vec<(SignalId, Vec<Bit>)>,
) -> Result<(), CeloxAdapterError> {
    for (output, bits) in outputs {
        let parts = bits
            .iter()
            .rev()
            .map(|bit| bit_expression(builder, wires, constants, *bit))
            .collect::<Result<Vec<_>, _>>()?;
        let value = match parts.as_slice() {
            [value] => *value,
            _ => builder.concat(parts)?,
        };
        builder.assign(builder.whole(output)?, value)?;
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct Constants {
    zero_expression: ExprId,
    one_expression: ExprId,
    zero_signal: SignalId,
    one_signal: SignalId,
}

#[derive(Clone, Copy)]
struct WireRef {
    signal: SignalId,
    lsb: usize,
    signal_width: usize,
}

struct FlipFlopBank {
    data: Vec<Bit>,
    outputs: Vec<u32>,
    enables: Vec<Option<Control>>,
    resets: Vec<Option<Reset>>,
    clock: Bit,
    edge: ClockEdge,
    async_reset: Option<Reset>,
}

// A physical ECP5 netlist contains scalar TRELLIS_FF cells. Celox SDK
// registers sharing an event are separate processes, so a Q-to-D pipeline can
// otherwise observe a newly committed upstream Q on the same edge. Packing FFs
// sharing a clock and asynchronous-reset domain into one vector register gives
// every bit one atomic sample/commit boundary. Per-bit enables and synchronous
// resets are folded into the vector next-state expression.
fn collect_flip_flop_banks(cells: &[Ecp5Cell]) -> Vec<FlipFlopBank> {
    let mut banks: Vec<FlipFlopBank> = Vec::new();
    for cell in cells {
        let Ecp5Cell::FlipFlop {
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
        let async_reset = reset.filter(|reset| reset.asynchronous);
        if let Some(bank) = banks.iter_mut().find(|bank| {
            bank.clock == *clock
                && bank.edge == *edge
                && same_reset_control(bank.async_reset, async_reset)
        }) {
            bank.data.push(*data);
            bank.outputs.push(*output);
            bank.enables.push(*enable);
            bank.resets.push(*reset);
        } else {
            banks.push(FlipFlopBank {
                data: vec![*data],
                outputs: vec![*output],
                enables: vec![*enable],
                resets: vec![*reset],
                clock: *clock,
                edge: *edge,
                async_reset,
            });
        }
    }
    banks
}

fn same_reset_control(lhs: Option<Reset>, rhs: Option<Reset>) -> bool {
    match (lhs, rhs) {
        (None, None) => true,
        (Some(lhs), Some(rhs)) => {
            lhs.signal == rhs.signal
                && lhs.active == rhs.active
                && lhs.asynchronous == rhs.asynchronous
        }
        _ => false,
    }
}

fn reserve_flip_flop_banks(
    builder: &mut ModuleBuilder,
    wires: &mut BTreeMap<u32, WireRef>,
    banks: &[FlipFlopBank],
) -> Result<(), CeloxAdapterError> {
    for (index, bank) in banks.iter().enumerate() {
        let width = bank.outputs.len();
        let signal =
            builder.internal(format!("__struo_ff_bank_{index}"), ValueType::bits(width)?)?;
        for (lsb, output) in bank.outputs.iter().enumerate() {
            insert_wire(
                wires,
                Bit::Wire(*output),
                WireRef {
                    signal,
                    lsb,
                    signal_width: width,
                },
            )?;
        }
    }
    Ok(())
}

fn insert_wire(
    wires: &mut BTreeMap<u32, WireRef>,
    bit: Bit,
    reference: WireRef,
) -> Result<(), CeloxAdapterError> {
    if let Bit::Wire(wire) = bit
        && wires.insert(wire, reference).is_some()
    {
        return Err(CeloxAdapterError::DuplicateWire(wire));
    }
    Ok(())
}

fn wire_ref(wires: &BTreeMap<u32, WireRef>, wire: u32) -> Result<WireRef, CeloxAdapterError> {
    wires
        .get(&wire)
        .copied()
        .ok_or(CeloxAdapterError::MissingWire(wire))
}

fn bit_expression(
    builder: &mut ModuleBuilder,
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    bit: Bit,
) -> Result<ExprId, CeloxAdapterError> {
    match bit {
        Bit::Zero => Ok(constants.zero_expression),
        Bit::One => Ok(constants.one_expression),
        Bit::Wire(wire) => {
            let reference = wire_ref(wires, wire)?;
            let slice = builder.slice(reference.signal, reference.lsb, 1)?;
            Ok(builder.read_slice(slice)?)
        }
    }
}

fn bit_signal(
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    bit: Bit,
) -> Result<SignalId, CeloxAdapterError> {
    match bit {
        Bit::Zero => Ok(constants.zero_signal),
        Bit::One => Ok(constants.one_signal),
        Bit::Wire(wire) => {
            let reference = wire_ref(wires, wire)?;
            if reference.lsb != 0 || reference.signal_width != 1 {
                return Err(CeloxAdapterError::NonScalarControl(wire));
            }
            Ok(reference.signal)
        }
    }
}

fn lut_expression(
    builder: &mut ModuleBuilder,
    inputs: &[ExprId],
    init: u16,
    input_index: usize,
    table_index: usize,
    constants: Constants,
) -> Result<ExprId, CeloxAdapterError> {
    if input_index == 4 {
        return Ok(if init & (1 << table_index) == 0 {
            constants.zero_expression
        } else {
            constants.one_expression
        });
    }
    let low = lut_expression(
        builder,
        inputs,
        init,
        input_index + 1,
        table_index,
        constants,
    )?;
    let high = lut_expression(
        builder,
        inputs,
        init,
        input_index + 1,
        table_index | (1 << input_index),
        constants,
    )?;
    if low == high {
        Ok(low)
    } else {
        Ok(builder.mux(inputs[input_index], high, low)?)
    }
}

fn emit_flip_flop_bank(
    builder: &mut ModuleBuilder,
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    bank: &FlipFlopBank,
) -> Result<(), CeloxAdapterError> {
    let target_signal = wire_ref(wires, bank.outputs[0])?.signal;
    let target = builder.whole(target_signal)?;
    let mut next_bits = Vec::with_capacity(bank.data.len());
    for (index, data) in bank.data.iter().enumerate() {
        let mut next = bit_expression(builder, wires, constants, *data)?;
        if let Some(enable) = bank.enables[index] {
            let reference = wire_ref(wires, bank.outputs[index])?;
            let current =
                builder.read_slice(builder.slice(reference.signal, reference.lsb, 1)?)?;
            let condition = asserted_expression(builder, wires, constants, enable)?;
            next = builder.mux(condition, next, current)?;
        }
        if let Some(reset) = bank.resets[index]
            && !reset.asynchronous
        {
            let condition = asserted_expression(
                builder,
                wires,
                constants,
                Control {
                    signal: reset.signal,
                    active: reset.active,
                },
            )?;
            let value = if reset.value {
                constants.one_expression
            } else {
                constants.zero_expression
            };
            next = builder.mux(condition, value, next)?;
        }
        next_bits.push(next);
    }
    next_bits.reverse();
    let next = match next_bits.as_slice() {
        [next] => *next,
        _ => builder.concat(next_bits)?,
    };
    let reset_value = if bank.async_reset.is_some() {
        let parts = bank
            .resets
            .iter()
            .rev()
            .map(|reset| {
                if reset.is_some_and(|reset| reset.value) {
                    constants.one_expression
                } else {
                    constants.zero_expression
                }
            })
            .collect::<Vec<_>>();
        Some(match parts.as_slice() {
            [value] => *value,
            _ => builder.concat(parts)?,
        })
    } else {
        None
    };
    let async_reset = match (bank.async_reset, reset_value) {
        (Some(reset), Some(value)) if reset.asynchronous => Some(builder.async_reset(
            bit_signal(wires, constants, reset.signal)?,
            active_level(reset.active),
            value,
        )?),
        _ => None,
    };
    builder.register(
        target,
        next,
        bit_signal(wires, constants, bank.clock)?,
        match bank.edge {
            ClockEdge::Rising => Edge::Posedge,
            ClockEdge::Falling => Edge::Negedge,
        },
        async_reset,
        None,
    )?;
    Ok(())
}

fn asserted_expression(
    builder: &mut ModuleBuilder,
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    control: Control,
) -> Result<ExprId, CeloxAdapterError> {
    let expression = bit_expression(builder, wires, constants, control.signal)?;
    if control.active == ActiveLevel::High {
        Ok(expression)
    } else {
        Ok(builder.unary(UnaryOp::LogicNot, expression, ValueType::bits(1)?)?)
    }
}

fn active_level(active: ActiveLevel) -> CeloxActiveLevel {
    match active {
        ActiveLevel::High => CeloxActiveLevel::High,
        ActiveLevel::Low => CeloxActiveLevel::Low,
    }
}

fn bits_expression(
    builder: &mut ModuleBuilder,
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    bits: &[Bit],
) -> Result<ExprId, CeloxAdapterError> {
    let parts = bits
        .iter()
        .rev()
        .map(|bit| bit_expression(builder, wires, constants, *bit))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(match parts.as_slice() {
        [expression] => *expression,
        _ => builder.concat(parts)?,
    })
}

#[allow(clippy::too_many_arguments)]
fn emit_block_ram(
    builder: &mut ModuleBuilder,
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    name: &str,
    depth: u32,
    word_width: u8,
    physical_width: u8,
    write_address: [Bit; 14],
    write_data: &[Bit],
    write_enable: Control,
    read_address: [Bit; 14],
    read_data: &[u32],
    read_enable: Option<Control>,
    clock: Bit,
    edge: ClockEdge,
) -> Result<(), CeloxAdapterError> {
    let address_width = (u32::BITS - (depth - 1).leading_zeros()).max(1) as usize;
    let shift = match physical_width {
        1 => 0,
        2 => 1,
        4 => 2,
        9 => 3,
        18 => 4,
        _ => unreachable!("mapped DP16KD width"),
    };
    let write_address = bits_expression(
        builder,
        wires,
        constants,
        &write_address[shift..shift + address_width],
    )?;
    let read_address = bits_expression(
        builder,
        wires,
        constants,
        &read_address[shift..shift + address_width],
    )?;
    let write_data = bits_expression(builder, wires, constants, write_data)?;
    let write_asserted = asserted_expression(builder, wires, constants, write_enable)?;
    let word_type = ValueType::bits(usize::from(word_width))?;
    let one_bit = ValueType::bits(1)?;
    let clock = bit_signal(wires, constants, clock)?;
    let edge = match edge {
        ClockEdge::Rising => Edge::Posedge,
        ClockEdge::Falling => Edge::Negedge,
    };
    let zero_word = Constant::two_state(0u8, usize::from(word_width))?;
    let mut words = Vec::with_capacity(depth as usize);

    for index in 0..depth {
        let signal = builder.internal(format!("__struo_{name}_word_{index}"), word_type)?;
        builder.set_initial(signal, zero_word.clone())?;
        let current = builder.read(signal)?;
        let address_constant = builder.constant(Constant::two_state(index, address_width)?);
        let selected = builder.binary(
            CeloxBinaryOp::Equal,
            write_address,
            address_constant,
            one_bit,
        )?;
        let write = builder.binary(CeloxBinaryOp::LogicAnd, write_asserted, selected, one_bit)?;
        let next = builder.mux(write, write_data, current)?;
        builder.register(builder.whole(signal)?, next, clock, edge, None, None)?;
        words.push(current);
    }

    let mut selected = builder.constant(zero_word);
    for (index, word) in (0..depth).zip(words) {
        let address_constant = builder.constant(Constant::two_state(index, address_width)?);
        let matches = builder.binary(
            CeloxBinaryOp::Equal,
            read_address,
            address_constant,
            one_bit,
        )?;
        selected = builder.mux(matches, word, selected)?;
    }
    let output = wire_ref(wires, read_data[0])?.signal;
    let enable = read_enable
        .map(|enable| {
            bit_signal(wires, constants, enable.signal)
                .and_then(|signal| Ok(builder.enable(signal, active_level(enable.active))?))
        })
        .transpose()?;
    builder.register(builder.whole(output)?, selected, clock, edge, None, enable)?;
    Ok(())
}

/// Failure while converting a mapped target object into a Celox artifact.
#[derive(Debug)]
pub enum CeloxAdapterError {
    /// Celox SDK rejected a constructed object.
    Build(BuildError),
    /// More than one mapped object claimed the same wire.
    DuplicateWire(u32),
    /// A cell references a wire not declared by the target netlist.
    MissingWire(u32),
    /// A register control references one bit within a vector signal.
    NonScalarControl(u32),
}

impl Display for CeloxAdapterError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::Build(error) => write!(formatter, "Celox artifact construction failed: {error}"),
            Self::DuplicateWire(wire) => write!(formatter, "mapped wire {wire} is declared twice"),
            Self::MissingWire(wire) => write!(formatter, "mapped wire {wire} is not declared"),
            Self::NonScalarControl(wire) => {
                write!(
                    formatter,
                    "mapped wire {wire} is not a scalar control signal"
                )
            }
        }
    }
}

impl Error for CeloxAdapterError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Build(error) => Some(error),
            Self::DuplicateWire(_) | Self::MissingWire(_) | Self::NonScalarControl(_) => None,
        }
    }
}

impl From<BuildError> for CeloxAdapterError {
    fn from(error: BuildError) -> Self {
        Self::Build(error)
    }
}

#[cfg(test)]
mod tests {
    use std::num::NonZeroU32;

    use struo_ir::{
        ActiveLevel, ArithmeticOp, ClockEdge, EnableControl, Netlist, RegisterCell, ResetControl,
    };
    use struo_rtl::{
        BinaryOp, BitWidth, ClockEdge as RtlClockEdge, Constant, Design, Enable, Memory, Module,
        Polarity, Port, PortDirection, StateDomain, ValueType,
    };
    use struo_synth::synthesize;
    use struo_target_ecp5::{
        ArithmeticMapping, JtaggBinding, MappingOptions, OpenDrainIo, map_to_ecp5,
        map_to_ecp5_with_jtagg, map_to_ecp5_with_open_drain_ios, map_to_ecp5_with_options,
    };

    use super::{ecp5_frontend_artifact, ecp5_simulator};

    fn bits(width: u32) -> ValueType {
        ValueType {
            width: BitWidth::new(width).unwrap(),
            signed: false,
            state: StateDomain::TwoState,
        }
    }

    #[test]
    fn synthesized_carry_and_lut_arithmetic_are_exhaustively_equivalent() {
        for (rtl_operation, operation) in [
            (BinaryOp::Add, ArithmeticOp::Add),
            (BinaryOp::Sub, ArithmeticOp::Subtract),
        ] {
            let mut module = Module::new("arithmetic");
            let lhs = module.add_port(Port {
                name: "lhs".into(),
                direction: PortDirection::Input,
                r#type: bits(5),
            });
            let rhs = module.add_port(Port {
                name: "rhs".into(),
                direction: PortDirection::Input,
                r#type: bits(5),
            });
            let result = module.add_port(Port {
                name: "result".into(),
                direction: PortDirection::Output,
                r#type: bits(5),
            });
            let lhs = module.read(lhs).unwrap();
            let rhs = module.read(rhs).unwrap();
            let value = module.binary(rtl_operation, lhs, rhs).unwrap();
            module.assign(module.whole(result).unwrap(), value).unwrap();
            let mut design = Design::new("arithmetic");
            design.add_module(module);
            let synthesized = synthesize(&design).unwrap();

            for strategy in [ArithmeticMapping::CarryChain, ArithmeticMapping::Lut4] {
                let mapped = map_to_ecp5_with_options(
                    &synthesized.netlist,
                    MappingOptions {
                        arithmetic: strategy,
                        ..MappingOptions::default()
                    },
                )
                .unwrap();
                let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
                let lhs = simulator.signal("lhs");
                let rhs = simulator.signal("rhs");
                let result = simulator.signal("result");
                for lhs_value in 0u8..32 {
                    for rhs_value in 0u8..32 {
                        simulator
                            .modify(|io| {
                                io.set(lhs, lhs_value);
                                io.set(rhs, rhs_value);
                            })
                            .unwrap();
                        let expected = match operation {
                            ArithmeticOp::Add => lhs_value.wrapping_add(rhs_value),
                            ArithmeticOp::Subtract => lhs_value.wrapping_sub(rhs_value),
                        } & 0x1f;
                        assert_eq!(simulator.get(result), expected.into());
                    }
                }
            }
        }
    }

    #[test]
    fn synthesized_carry_comparisons_are_exhaustively_equivalent() {
        for width in [1u32, 2, 5] {
            let operations = [
                ("ltu", BinaryOp::LessThanUnsigned),
                ("leu", BinaryOp::LessOrEqualUnsigned),
                ("gtu", BinaryOp::GreaterThanUnsigned),
                ("geu", BinaryOp::GreaterOrEqualUnsigned),
                ("lts", BinaryOp::LessThanSigned),
                ("les", BinaryOp::LessOrEqualSigned),
                ("gts", BinaryOp::GreaterThanSigned),
                ("ges", BinaryOp::GreaterOrEqualSigned),
            ];
            let mut module = Module::new("comparisons");
            let lhs_signal = module.add_port(Port {
                name: "lhs".into(),
                direction: PortDirection::Input,
                r#type: bits(width),
            });
            let rhs_signal = module.add_port(Port {
                name: "rhs".into(),
                direction: PortDirection::Input,
                r#type: bits(width),
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
                module.assign(module.whole(output).unwrap(), value).unwrap();
            }
            let mut design = Design::new("comparisons");
            design.add_module(module);

            let synthesized = synthesize(&design).unwrap();
            assert_eq!(synthesized.netlist.comparisons().len(), 8);
            let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
            assert_eq!(
                mapped
                    .cells()
                    .iter()
                    .filter(|cell| matches!(cell, struo_target_ecp5::Ecp5Cell::Ccu2c { .. }))
                    .count(),
                8 * width.div_ceil(2) as usize
            );
            let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
            let lhs = simulator.signal("lhs");
            let rhs = simulator.signal("rhs");
            let ltu = simulator.signal("ltu");
            let leu = simulator.signal("leu");
            let gtu = simulator.signal("gtu");
            let geu = simulator.signal("geu");
            let lts = simulator.signal("lts");
            let les = simulator.signal("les");
            let gts = simulator.signal("gts");
            let ges = simulator.signal("ges");

            let limit = 1u8 << width;
            let sign = 1u8 << (width - 1);
            let modulus = 1i16 << width;
            for lhs_value in 0u8..limit {
                for rhs_value in 0u8..limit {
                    simulator
                        .modify(|io| {
                            io.set(lhs, lhs_value);
                            io.set(rhs, rhs_value);
                        })
                        .unwrap();
                    let lhs_signed = if lhs_value & sign == 0 {
                        i16::from(lhs_value)
                    } else {
                        i16::from(lhs_value) - modulus
                    };
                    let rhs_signed = if rhs_value & sign == 0 {
                        i16::from(rhs_value)
                    } else {
                        i16::from(rhs_value) - modulus
                    };
                    assert_eq!(simulator.get(ltu), u8::from(lhs_value < rhs_value).into());
                    assert_eq!(simulator.get(leu), u8::from(lhs_value <= rhs_value).into());
                    assert_eq!(simulator.get(gtu), u8::from(lhs_value > rhs_value).into());
                    assert_eq!(simulator.get(geu), u8::from(lhs_value >= rhs_value).into());
                    assert_eq!(simulator.get(lts), u8::from(lhs_signed < rhs_signed).into());
                    assert_eq!(
                        simulator.get(les),
                        u8::from(lhs_signed <= rhs_signed).into()
                    );
                    assert_eq!(simulator.get(gts), u8::from(lhs_signed > rhs_signed).into());
                    assert_eq!(
                        simulator.get(ges),
                        u8::from(lhs_signed >= rhs_signed).into()
                    );
                }
            }
        }
    }

    #[test]
    fn emits_lut_logic_as_a_valid_celox_artifact() {
        let mut source = Netlist::new("logic");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let value = source.add_xor(lhs, rhs);
        source.add_output("value", value);
        let mapped = map_to_ecp5(&source).unwrap();

        let artifact = ecp5_frontend_artifact(&mapped).unwrap();

        assert_eq!(artifact.module_name(), "logic");
        assert_eq!(artifact.port_order().len(), 3);
        assert_eq!(artifact.registers().len(), 0);
        assert!(!artifact.assignments().is_empty());
    }

    #[test]
    fn simulates_mapped_logic_without_json_round_trip() {
        let mut source = Netlist::new("logic");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let value = source.add_xor(lhs, rhs);
        source.add_output("value", value);
        let mapped = map_to_ecp5(&source).unwrap();

        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        let lhs = simulator.signal("lhs");
        let rhs = simulator.signal("rhs");
        let value = simulator.signal("value");
        simulator
            .modify(|io| {
                io.set(lhs, 1u8);
                io.set(rhs, 0u8);
            })
            .unwrap();

        assert_eq!(simulator.get(value), 1u8.into());
    }

    #[test]
    fn simulates_open_drain_pad_readback() {
        let mut source = Netlist::new("i2c_top");
        let sda_i = source.add_input("sda_i");
        let request = source.add_input("request");
        source.add_output("sda_drive_low", request);
        source.add_output("sampled_sda", sda_i);
        let mapped = map_to_ecp5_with_open_drain_ios(
            &source,
            &[OpenDrainIo::new("sda", "sda_i", "sda_drive_low")],
        )
        .unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        let sda = simulator.signal("sda");
        let request = simulator.signal("request");
        let sampled = simulator.signal("sampled_sda");

        simulator
            .modify(|io| {
                io.set(sda, 1u8);
                io.set(request, 0u8);
            })
            .unwrap();
        assert_eq!(simulator.get(sampled), 1u8.into());

        simulator
            .modify(|io| {
                io.set(sda, 1u8);
                io.set(request, 1u8);
            })
            .unwrap();
        assert_eq!(simulator.get(sampled), 0u8.into());
    }

    #[test]
    fn models_bound_jtagg_in_its_inactive_state() {
        let mut source = Netlist::new("debug_top");
        let mut fabric_outputs = Vec::new();
        for name in [
            "jtag_tdi",
            "jtag_tck",
            "jtag_rti1",
            "jtag_rti2",
            "jtag_shift",
            "jtag_update",
            "jtag_rst_n",
            "jtag_ce1",
            "jtag_ce2",
        ] {
            fabric_outputs.push((name, source.add_input(name)));
        }
        let zero = source.add_constant(false);
        source.add_output("jtag_tdo1", zero);
        source.add_output("jtag_tdo2", zero);
        source.add_output("observed_tdi", fabric_outputs[0].1);
        source.add_output("observed_reset_n", fabric_outputs[6].1);
        let mapped = map_to_ecp5_with_jtagg(&source, &JtaggBinding::with_prefix("jtag")).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        assert_eq!(simulator.get(simulator.signal("observed_tdi")), 0u8.into());
        assert_eq!(
            simulator.get(simulator.signal("observed_reset_n")),
            1u8.into()
        );
    }

    #[test]
    fn simulates_synthesized_address_decode_without_json() {
        let byte = ValueType {
            width: BitWidth::new(8).unwrap(),
            signed: false,
            state: StateDomain::TwoState,
        };
        let bit = ValueType {
            width: BitWidth::new(1).unwrap(),
            signed: false,
            state: StateDomain::TwoState,
        };
        let mut module = Module::new("AddressDecoder");
        let address = module.add_port(Port {
            name: "address".into(),
            direction: PortDirection::Input,
            r#type: byte,
        });
        let valid = module.add_port(Port {
            name: "valid".into(),
            direction: PortDirection::Input,
            r#type: bit,
        });
        let route = module.add_port(Port {
            name: "route".into(),
            direction: PortDirection::Output,
            r#type: ValueType {
                width: BitWidth::new(2).unwrap(),
                signed: false,
                state: StateDomain::TwoState,
            },
        });
        let address_value = module.read(address).unwrap();
        let valid_value = module.read(valid).unwrap();
        let limit_0 = module.constant(Constant::from_u64(BitWidth::new(8).unwrap(), 0x40));
        let base_1 = module.constant(Constant::from_u64(BitWidth::new(8).unwrap(), 0x80));
        let limit_1 = module.constant(Constant::from_u64(BitWidth::new(8).unwrap(), 0xc0));
        let hit_0 = module
            .binary(BinaryOp::LessThanUnsigned, address_value, limit_0)
            .unwrap();
        let above_base_1 = module
            .binary(BinaryOp::GreaterOrEqualUnsigned, address_value, base_1)
            .unwrap();
        let below_limit_1 = module
            .binary(BinaryOp::LessThanUnsigned, address_value, limit_1)
            .unwrap();
        let hit_1 = module
            .binary(BinaryOp::And, above_base_1, below_limit_1)
            .unwrap();
        let hit_0 = module.binary(BinaryOp::And, valid_value, hit_0).unwrap();
        let hit_1 = module.binary(BinaryOp::And, valid_value, hit_1).unwrap();
        let route_value = module.concat(vec![hit_1, hit_0]).unwrap();
        let route_target = module.whole(route).unwrap();
        module.assign(route_target, route_value).unwrap();
        let mut design = Design::new("AddressDecoder");
        design.add_module(module);

        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        let address = simulator.signal("address");
        let valid = simulator.signal("valid");
        let route = simulator.signal("route");
        for (address_value, expected) in [
            (0x00u8, 1u8),
            (0x3f, 1),
            (0x40, 0),
            (0x7f, 0),
            (0x80, 2),
            (0xbf, 2),
            (0xc0, 0),
        ] {
            simulator
                .modify(|io| {
                    io.set(address, address_value);
                    io.set(valid, 1u8);
                })
                .unwrap();
            assert_eq!(simulator.get(route), expected.into());
        }
        simulator.modify(|io| io.set(valid, 0u8)).unwrap();
        assert_eq!(simulator.get(route), 0u8.into());
    }

    #[test]
    fn simulates_mapped_block_ram_without_json_round_trip() {
        let mut module = Module::new("Scratchpad");
        let clock = module.add_port(Port {
            name: "clock".into(),
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
            r#type: bits(4),
        });
        let write_address_signal = module.add_port(Port {
            name: "write_address".into(),
            direction: PortDirection::Input,
            r#type: bits(4),
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
            depth: 16,
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
            edge: RtlClockEdge::Rising,
        });
        let mut design = Design::new("Scratchpad");
        design.add_module(module);
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        let clock_event = simulator.event("clock");
        let write_enable = simulator.signal("write_enable");
        let read_address = simulator.signal("read_address");
        let write_address = simulator.signal("write_address");
        let write_data = simulator.signal("write_data");
        let read_data = simulator.signal("read_data");

        simulator
            .modify(|io| {
                io.set(write_enable, 1u8);
                io.set(write_address, 5u8);
                io.set(write_data, 0xa5u8);
                io.set(read_address, 0u8);
            })
            .unwrap();
        simulator.tick(clock_event).unwrap();
        simulator
            .modify(|io| {
                io.set(write_enable, 0u8);
                io.set(read_address, 5u8);
            })
            .unwrap();
        simulator.tick(clock_event).unwrap();

        assert_eq!(simulator.get(read_data), 0xa5u8.into());
    }

    #[test]
    fn emits_trellis_ff_with_async_reset() {
        let mut source = Netlist::new("state_bit");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset_n");
        let state = source.add_register_output("state");
        let next = source.add_not(state);
        source.add_register(RegisterCell::new(
            "state",
            state,
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
        source.add_output("value", state);
        let mapped = map_to_ecp5(&source).unwrap();

        let artifact = ecp5_frontend_artifact(&mapped).unwrap();
        let register = artifact.registers()[0];

        assert_eq!(artifact.registers().len(), 1);
        assert!(register.async_reset().is_some());
        assert!(register.enable().is_none());
    }

    #[test]
    fn simulates_flip_flop_banks_without_same_edge_feedthrough() {
        let mut source = Netlist::new("pipeline");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset_n");
        let input = source.add_input("input");
        let stage0 = source.add_register_output("stage0");
        let stage1 = source.add_register_output("stage1");
        let stage2 = source.add_register_output("stage2");
        for (name, output, data, reset_value) in [
            ("stage0", stage0, input, false),
            ("stage1", stage1, stage0, false),
            ("stage2", stage2, stage1, true),
        ] {
            source.add_register(RegisterCell::new(
                name,
                output,
                data,
                clock,
                ClockEdge::Rising,
                None,
                Some(ResetControl {
                    signal: reset,
                    active: ActiveLevel::Low,
                    asynchronous: true,
                    value: reset_value,
                }),
            ));
        }
        source.add_output("stage0", stage0);
        source.add_output("stage1", stage1);
        source.add_output("stage2", stage2);
        let mapped = map_to_ecp5(&source).unwrap();
        let artifact = ecp5_frontend_artifact(&mapped).unwrap();
        assert_eq!(artifact.registers().len(), 1);

        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        let clock = simulator.event("clock");
        let reset = simulator.signal("reset_n");
        let input = simulator.signal("input");
        let stage0 = simulator.signal("stage0");
        let stage1 = simulator.signal("stage1");
        let stage2 = simulator.signal("stage2");
        simulator.modify(|io| io.set(reset, 0u8)).unwrap();
        simulator.tick(clock).unwrap();
        assert_eq!(simulator.get(stage0), 0u8.into());
        assert_eq!(simulator.get(stage1), 0u8.into());
        assert_eq!(simulator.get(stage2), 1u8.into());
        simulator
            .modify(|io| {
                io.set(reset, 1u8);
                io.set(input, 1u8);
            })
            .unwrap();

        simulator.tick(clock).unwrap();
        assert_eq!(simulator.get(stage0), 1u8.into());
        assert_eq!(simulator.get(stage1), 0u8.into());
        assert_eq!(simulator.get(stage2), 0u8.into());
        simulator.modify(|io| io.set(input, 0u8)).unwrap();
        simulator.tick(clock).unwrap();
        assert_eq!(simulator.get(stage0), 0u8.into());
        assert_eq!(simulator.get(stage1), 1u8.into());
        assert_eq!(simulator.get(stage2), 0u8.into());
        simulator.tick(clock).unwrap();
        assert_eq!(simulator.get(stage0), 0u8.into());
        assert_eq!(simulator.get(stage1), 0u8.into());
        assert_eq!(simulator.get(stage2), 1u8.into());
    }

    #[test]
    fn folds_synchronous_reset_over_enable_into_next_state() {
        let mut source = Netlist::new("sync_state");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let enable = source.add_input("enable");
        let state = source.add_register_output("state");
        let next = source.add_not(state);
        source.add_register(RegisterCell::new(
            "state",
            state,
            next,
            clock,
            ClockEdge::Rising,
            Some(EnableControl {
                signal: enable,
                active: ActiveLevel::High,
            }),
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::High,
                asynchronous: false,
                value: false,
            }),
        ));
        source.add_output("value", state);
        let mapped = map_to_ecp5(&source).unwrap();

        let artifact = ecp5_frontend_artifact(&mapped).unwrap();
        let register = artifact.registers()[0];

        assert!(register.async_reset().is_none());
        assert!(register.enable().is_none());
    }

    #[test]
    fn preserves_vector_ports_in_the_celox_artifact() {
        let mut source = Netlist::new("passthrough");
        let input = source.add_input_port("input", NonZeroU32::new(3).unwrap());
        source.add_output_port("output", &input).unwrap();
        let mapped = map_to_ecp5(&source).unwrap();

        let artifact = ecp5_frontend_artifact(&mapped).unwrap();
        let input = artifact
            .signals()
            .iter()
            .find(|signal| signal.name() == "input")
            .unwrap();
        let output = artifact
            .signals()
            .iter()
            .find(|signal| signal.name() == "output")
            .unwrap();

        assert_eq!(artifact.port_order().len(), 2);
        assert_eq!(input.value_type().width(), 3);
        assert_eq!(output.value_type().width(), 3);
    }
}
