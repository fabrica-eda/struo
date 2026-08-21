//! Celox simulation artifacts for technology-mapped Struo netlists.
//!
//! This crate is intentionally a backend adapter. Source RTL is simulated by
//! Celox's Veryl frontend; only the post-technology-mapping object is converted
//! into a synthetic [`celox::FrontendArtifact`].

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt::{self, Display, Formatter};

use celox::frontend_sdk::{
    ActiveLevel as CeloxActiveLevel, BuildError, Constant, Edge, ExprId, FrontendArtifact,
    ModuleBuilder, SignalId, UnaryOp, ValueType,
};
use celox::{Simulator, SimulatorBuilder};
use struo_ir::{ActiveLevel, ClockEdge};
use struo_target_ecp5::{Bit, Control, Ecp5Cell, Ecp5Netlist, MappedPortDirection, Reset};

/// Converts the exact ECP5 target object into Celox's external-frontend format.
///
/// LUT truth tables are expanded as mux trees. `TRELLIS_FF` asynchronous resets
/// map directly to the SDK register reset, while synchronous resets are folded
/// into next-state logic with reset-over-enable priority.
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

    for port in netlist.ports() {
        let port_type = ValueType::bits(port.bits.len())?;
        let signal = match port.direction {
            MappedPortDirection::Input => builder.input(&port.name, port_type)?,
            MappedPortDirection::Output => builder.output(&port.name, port_type)?,
        };
        match port.direction {
            MappedPortDirection::Input => {
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
        let (wire, name) = match cell {
            Ecp5Cell::Lut4 { name, output, .. } => {
                (*output, format!("__struo_lut_{name}_{output}"))
            }
            Ecp5Cell::FlipFlop { name, output, .. } => {
                (*output, format!("__struo_ff_{name}_{output}"))
            }
        };
        let signal = builder.internal(name, bit_type)?;
        insert_wire(
            &mut wires,
            Bit::Wire(wire),
            WireRef {
                signal,
                lsb: 0,
                signal_width: 1,
            },
        )?;
    }

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
        match cell {
            Ecp5Cell::Lut4 {
                inputs,
                output,
                init,
                ..
            } => {
                let input_expressions = inputs
                    .iter()
                    .map(|bit| bit_expression(&mut builder, &wires, constants, *bit))
                    .collect::<Result<Vec<_>, _>>()?;
                let value =
                    lut_expression(&mut builder, &input_expressions, *init, 0, 0, constants)?;
                let target = builder.whole(wire_ref(&wires, *output)?.signal)?;
                builder.assign(target, value)?;
            }
            Ecp5Cell::FlipFlop {
                data,
                output,
                clock,
                edge,
                enable,
                reset,
                ..
            } => emit_flip_flop(
                &mut builder,
                &wires,
                constants,
                *data,
                *output,
                *clock,
                *edge,
                *enable,
                *reset,
            )?,
        }
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

#[allow(clippy::too_many_arguments)]
fn emit_flip_flop(
    builder: &mut ModuleBuilder,
    wires: &BTreeMap<u32, WireRef>,
    constants: Constants,
    data: Bit,
    output: u32,
    clock: Bit,
    edge: ClockEdge,
    enable: Option<Control>,
    reset: Option<Reset>,
) -> Result<(), CeloxAdapterError> {
    let target_signal = wire_ref(wires, output)?.signal;
    let target = builder.whole(target_signal)?;
    let mut next = bit_expression(builder, wires, constants, data)?;
    let mut sdk_enable = match enable {
        Some(enable) => Some(builder.enable(
            bit_signal(wires, constants, enable.signal)?,
            active_level(enable.active),
        )?),
        None => None,
    };
    let async_reset = match reset {
        Some(reset) if reset.asynchronous => Some(builder.async_reset(
            bit_signal(wires, constants, reset.signal)?,
            active_level(reset.active),
            if reset.value {
                constants.one_expression
            } else {
                constants.zero_expression
            },
        )?),
        Some(reset) => {
            if let Some(enable) = enable {
                let current = builder.read(target_signal)?;
                let condition = asserted_expression(builder, wires, constants, enable)?;
                next = builder.mux(condition, next, current)?;
            }
            let condition = asserted_expression(
                builder,
                wires,
                constants,
                Control {
                    signal: reset.signal,
                    active: reset.active,
                },
            )?;
            next = builder.mux(
                condition,
                if reset.value {
                    constants.one_expression
                } else {
                    constants.zero_expression
                },
                next,
            )?;
            sdk_enable = None;
            None
        }
        None => None,
    };
    builder.register(
        target,
        next,
        bit_signal(wires, constants, clock)?,
        match edge {
            ClockEdge::Rising => Edge::Posedge,
            ClockEdge::Falling => Edge::Negedge,
        },
        async_reset,
        sdk_enable,
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

    use struo_ir::{ActiveLevel, ClockEdge, EnableControl, Netlist, RegisterCell, ResetControl};
    use struo_target_ecp5::map_to_ecp5;

    use super::{ecp5_frontend_artifact, ecp5_simulator};

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

        let mut simulator = ecp5_simulator(&mapped).unwrap().build_cranelift().unwrap();
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
