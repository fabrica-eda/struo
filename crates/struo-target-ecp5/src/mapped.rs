//! ECP5 technology mapping and nextpnr serialization.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::error::Error;
use std::fmt::{self, Display, Formatter};
use std::hash::{BuildHasherDefault, Hasher};

use serde::ser::Serializer;
use serde::{Deserialize, Serialize};
use struo_formal::{
    LogicFunction, RetimingCertificate, RetimingDomain, RetimingEdge, RetimingGraph,
    RetimingVertex, derive_retimed_graph, verify_retiming_certificate,
};
use struo_ir::{
    ActiveLevel, ArithmeticCell, ArithmeticOp, ClockEdge, ComparisonCell, MemoryCell, NetId,
    Netlist, PortDirection as IrPortDirection, ValidationError,
};

use crate::physical::{PhysicalFeedback, PhysicalLocation};

mod lut;

use lut::{
    BRAM_CLOCK_TO_OUTPUT_PS, CCU_CARRY_PS, CCU_INPUT_PS, CCU_SUM_PS, CutDatabase,
    FLIP_FLOP_CLOCK_TO_OUTPUT_PS, FLIP_FLOP_SETUP_PS, LUT_DELAY_PS, LutCover, LutEmitter,
    wire_delay_ps,
};

const RETIMING_PERIOD_MARGIN_NUMERATOR: u32 = 9;
const RETIMING_PERIOD_MARGIN_DENOMINATOR: u32 = 10;
// The pre-placement fanout model deliberately stays optimistic so LUT covering
// does not overfit one device floorplan.  Once primitives are fixed, retiming
// needs a routing guard per ordinary hop: measured ECP5 AXI paths average about
// 100 ps more than that model, while dedicated carry hops bypass this charge.
const MAPPED_ROUTE_GUARD_PS: u32 = 100;
const MAX_ENABLE_FANOUT_PER_REPLICA: usize = 16;
const PHYSICAL_REWRITE_MIN_GOAL_PERCENT: u32 = 98;
const PHYSICAL_RETIME_MIN_GOAL_PERCENT: u32 = 95;
const PHYSICAL_RETIME_MODEL_BRIDGE_PS: u32 = 400;

// These maps only contain trusted internal u32 wire IDs. Avoid the randomized
// string-oriented hashing cost in the timing model's repeated fixed-point scans.
struct WireHasher(u64);

impl Default for WireHasher {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Hasher for WireHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.0 = (self.0 ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    fn write_u32(&mut self, value: u32) {
        self.0 = (self.0 ^ u64::from(value)).wrapping_mul(0x9e37_79b9_7f4a_7c15);
    }
}

type WireMap<T> = HashMap<u32, T, BuildHasherDefault<WireHasher>>;

/// A constant or numbered wire in a mapped ECP5 design.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Bit {
    /// Constant zero.
    Zero,
    /// Constant one.
    One,
    /// A numbered Yosys/nextpnr wire.
    Wire(u32),
}

impl Serialize for Bit {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        match self {
            Self::Zero => serializer.serialize_str("0"),
            Self::One => serializer.serialize_str("1"),
            Self::Wire(wire) => serializer.serialize_u32(*wire),
        }
    }
}

/// Direction of a physical port.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PortDirection {
    /// FPGA input.
    Input,
    /// FPGA output.
    Output,
    /// Bidirectional FPGA pad.
    Inout,
}

/// One physical port with bits stored least-significant first.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MappedPort {
    /// Port name used by the LPF constraint file.
    pub name: String,
    /// Port direction.
    pub direction: PortDirection,
    /// Connected mapped bits, least-significant first.
    pub bits: Vec<Bit>,
}

/// Connects a split, scalar open-drain interface to one physical FPGA pad.
///
/// The input port continuously observes the pad. The active-high drive-low
/// port may only pull the pad low; when it is deasserted the pad is high
/// impedance and must be raised by an external pull-up resistor.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OpenDrainIo {
    /// Physical bidirectional port name used by the LPF constraint file.
    pub pin: String,
    /// Scalar input port through which the core observes the physical pad.
    pub input_port: String,
    /// Scalar output port which pulls the pad low when asserted.
    pub drive_low_port: String,
}

/// Maps a scalar top-level debug interface onto the ECP5 dedicated JTAG block.
///
/// The port directions are from the Veryl/core point of view: `tdo` ports are
/// core outputs consumed by `JTAGG`; every other named port is a core input
/// driven by `JTAGG`. [`JtaggBinding::with_prefix`] provides the conventional
/// `<prefix>_<signal>` spelling for all eleven fabric-side signals.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JtaggBinding {
    /// Data returned by extension registers one and two (`JTDO1`, `JTDO2`).
    pub tdo_ports: [String; 2],
    /// Registered data received from the external TAP (`JTDI`).
    pub tdi_port: String,
    /// Transport clock exported by the external TAP (`JTCK`); this is not a
    /// fabric system-clock or PLL binding.
    pub clock_port: String,
    /// Run-test/idle indications for extension registers one and two.
    pub run_test_idle_ports: [String; 2],
    /// Shift-DR indication (`JSHIFT`).
    pub shift_port: String,
    /// Update-DR indication (`JUPDATE`).
    pub update_port: String,
    /// Active-low TAP reset indication (`JRSTN`).
    pub reset_n_port: String,
    /// Extension-register clock enables (`JCE1`, `JCE2`).
    pub clock_enable_ports: [String; 2],
    /// Whether extension register one is present.
    pub extension_register_1: bool,
    /// Whether extension register two is present.
    pub extension_register_2: bool,
}

impl JtaggBinding {
    /// Creates a complete binding using `<prefix>_tdo1`, `<prefix>_tdo2`,
    /// `<prefix>_tdi`, `<prefix>_tck`, `<prefix>_rti1`, `<prefix>_rti2`,
    /// `<prefix>_shift`, `<prefix>_update`, `<prefix>_rst_n`,
    /// `<prefix>_ce1`, and `<prefix>_ce2`.
    #[must_use]
    pub fn with_prefix(prefix: &str) -> Self {
        let port = |suffix: &str| format!("{prefix}_{suffix}");
        Self {
            tdo_ports: [port("tdo1"), port("tdo2")],
            tdi_port: port("tdi"),
            clock_port: port("tck"),
            run_test_idle_ports: [port("rti1"), port("rti2")],
            shift_port: port("shift"),
            update_port: port("update"),
            reset_n_port: port("rst_n"),
            clock_enable_ports: [port("ce1"), port("ce2")],
            extension_register_1: true,
            extension_register_2: true,
        }
    }
}

/// One clock output of an ECP5 `EHXPLLL` primitive.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
pub enum PllOutput {
    /// Primary output (`CLKOP`).
    #[serde(rename = "CLKOP")]
    Clkop,
    /// Secondary output (`CLKOS`).
    #[serde(rename = "CLKOS")]
    Clkos,
    /// Secondary output two (`CLKOS2`).
    #[serde(rename = "CLKOS2")]
    Clkos2,
    /// Secondary output three (`CLKOS3`).
    #[serde(rename = "CLKOS3")]
    Clkos3,
}

impl PllOutput {
    fn port(self) -> &'static str {
        match self {
            Self::Clkop => "CLKOP",
            Self::Clkos => "CLKOS",
            Self::Clkos2 => "CLKOS2",
            Self::Clkos3 => "CLKOS3",
        }
    }
}

/// User-supplied top-boundary binding for an ECP5 `EHXPLLL`.
///
/// Struo owns only the boundary rewrite. The user supplies the PLL parameters
/// and frequency attributes, normally copied from `ecppll` or another
/// device-qualified clock configuration. The reference port remains physical;
/// the logical clock and lock inputs are removed and driven by the primitive.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct PllBinding {
    /// Physical reference-clock input retained on the mapped top.
    pub reference_clock_port: String,
    /// Logical core-clock input replaced by the selected PLL output.
    pub output_clock_port: String,
    /// Logical core input replaced by `LOCK`.
    pub locked_port: String,
    /// PLL output routed into the core.
    pub fabric_output: PllOutput,
    /// PLL output looped back to `CLKFB`.
    pub feedback_output: PllOutput,
    /// Raw `EHXPLLL` parameters written to nextpnr JSON.
    #[serde(default)]
    pub parameters: BTreeMap<String, String>,
    /// Raw `EHXPLLL` attributes written to nextpnr JSON.
    #[serde(default)]
    pub attributes: BTreeMap<String, String>,
}

impl PllBinding {
    /// Creates an empty user-configured PLL binding.
    #[must_use]
    pub fn new(
        reference_clock_port: impl Into<String>,
        output_clock_port: impl Into<String>,
        locked_port: impl Into<String>,
        fabric_output: PllOutput,
        feedback_output: PllOutput,
    ) -> Self {
        Self {
            reference_clock_port: reference_clock_port.into(),
            output_clock_port: output_clock_port.into(),
            locked_port: locked_port.into(),
            fabric_output,
            feedback_output,
            parameters: BTreeMap::new(),
            attributes: BTreeMap::new(),
        }
    }
}

impl OpenDrainIo {
    /// Creates one scalar open-drain I/O binding.
    #[must_use]
    pub fn new(
        pin: impl Into<String>,
        input_port: impl Into<String>,
        drive_low_port: impl Into<String>,
    ) -> Self {
        Self {
            pin: pin.into(),
            input_port: input_port.into(),
            drive_low_port: drive_low_port.into(),
        }
    }
}

/// Control input and its assertion level.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Control {
    /// Control signal.
    pub signal: Bit,
    /// Logical assertion level.
    pub active: ActiveLevel,
}

/// Reset behavior retained by a mapped flip-flop.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Reset {
    /// Reset signal.
    pub signal: Bit,
    /// Logical assertion level.
    pub active: ActiveLevel,
    /// Whether reset is asynchronous.
    pub asynchronous: bool,
    /// State loaded on reset.
    pub value: bool,
}

/// How retained word-level arithmetic is implemented on ECP5.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArithmeticMapping {
    /// Use CCU2C above four bits and LUTs for smaller operations.
    Auto,
    /// Always use the dedicated CCU2C carry chain.
    CarryChain,
    /// Implement a ripple carry using ordinary LUT4 cells.
    Lut4,
}

/// Technology-mapping choices used for experiments and regression tests.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MappingOptions {
    /// Arithmetic implementation strategy.
    pub arithmetic: ArithmeticMapping,
    /// Clock frequency used by timing-driven LUT covering.
    pub timing_goal_mhz: u32,
}

impl Default for MappingOptions {
    fn default() -> Self {
        Self {
            arithmetic: ArithmeticMapping::Auto,
            timing_goal_mhz: crate::ECP5_QOR_TARGET_MHZ,
        }
    }
}

/// Explicit timing delays for top-level ports in the mapper's single-period
/// timing model.
///
/// Ports absent from these maps are unconstrained. An input delay is the
/// arrival time at the FPGA boundary after the launching clock edge; an output
/// delay is the portion of the period reserved beyond the FPGA boundary.
#[derive(Clone, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
pub struct IoTimingConstraints {
    /// Maximum input arrival delay by source-level port name, in picoseconds.
    #[serde(default)]
    pub input_delays_ps: BTreeMap<String, u32>,
    /// Maximum external output delay by source-level port name, in picoseconds.
    #[serde(default)]
    pub output_delays_ps: BTreeMap<String, u32>,
}

impl IoTimingConstraints {
    /// Creates an empty set in which every top-level I/O path is unconstrained.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            input_delays_ps: BTreeMap::new(),
            output_delays_ps: BTreeMap::new(),
        }
    }

    /// Adds or replaces a maximum input delay in picoseconds.
    #[must_use]
    pub fn with_input_delay_ps(mut self, port: impl Into<String>, delay_ps: u32) -> Self {
        self.input_delays_ps.insert(port.into(), delay_ps);
        self
    }

    /// Adds or replaces a maximum external output delay in picoseconds.
    #[must_use]
    pub fn with_output_delay_ps(mut self, port: impl Into<String>, delay_ps: u32) -> Self {
        self.output_delays_ps.insert(port.into(), delay_ps);
        self
    }
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct ResolvedIoTiming {
    input_arrivals_ps: BTreeMap<u32, u32>,
    output_delays_ps: Vec<(Bit, u32)>,
}

#[derive(Debug)]
struct MappingDemand {
    roots: Vec<NetId>,
}

impl MappingDemand {
    fn collect(netlist: &Netlist, constant_registers: &HashMap<NetId, bool>) -> Self {
        let output_roots = netlist
            .ports()
            .iter()
            .filter(|port| port.direction() == IrPortDirection::Output)
            .flat_map(struo_ir::Port::bits)
            .map(|output| node_for(netlist, *output).inputs()[0]);
        let register_roots = netlist
            .registers()
            .iter()
            .filter(|register| !constant_registers.contains_key(&register.output()))
            .flat_map(|register| {
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
        let arithmetic_roots = netlist.arithmetic().iter().flat_map(|cell| {
            cell.lhs()
                .iter()
                .chain(cell.rhs())
                .copied()
                .chain(cell.carry_in())
        });
        let comparison_roots = netlist
            .comparisons()
            .iter()
            .flat_map(|cell| cell.lhs().iter().chain(cell.rhs()).copied());
        Self {
            // Preserve duplicates: they are distinct physical sink pins and
            // therefore contribute to the fanout estimate used by covering.
            roots: output_roots
                .chain(register_roots)
                .chain(memory_roots)
                .chain(arithmetic_roots)
                .chain(comparison_roots)
                .collect(),
        }
    }
}

/// A physical ECP5 primitive.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Ecp5Cell {
    /// Four-input lookup table. Input A is the least-significant INIT index bit.
    Lut4 {
        /// Stable cell name.
        name: String,
        /// A, B, C, and D inputs.
        inputs: [Bit; 4],
        /// LUT output wire.
        output: u32,
        /// Complete LUT truth table.
        init: u16,
    },
    /// Two ECP5 LUT/carry slices connected through the dedicated carry path.
    Ccu2c {
        /// Stable cell name.
        name: String,
        /// A, B, C, and D inputs for each of the two slices.
        inputs: [[Bit; 4]; 2],
        /// Dedicated carry-chain input.
        carry_in: Bit,
        /// Sum output wires for the two slices.
        sums: [u32; 2],
        /// Dedicated carry-chain output wire.
        carry_out: u32,
        /// LUT truth tables for the two slices.
        init: [u16; 2],
        /// Whether each slice suppresses its incoming carry.
        inject: [bool; 2],
    },
    /// ECP5 slice flip-flop.
    FlipFlop {
        /// Stable cell name.
        name: String,
        /// D input.
        data: Bit,
        /// Q output wire.
        output: u32,
        /// Clock input.
        clock: Bit,
        /// Active clock edge.
        edge: ClockEdge,
        /// Optional clock enable.
        enable: Option<Control>,
        /// Optional local set/reset.
        reset: Option<Reset>,
    },
    /// One ECP5 18-Kibit embedded block RAM in simple-dual-port mode.
    BlockRam {
        /// Stable cell name.
        name: String,
        /// Logical number of words represented by the cell.
        depth: u32,
        /// Logical word width.
        word_width: u8,
        /// Configured DP16KD port width (1, 2, 4, 9, or 18).
        physical_width: u8,
        /// Fourteen physical write-address pins.
        write_address: Box<[Bit; 14]>,
        /// Logical write-data bits, least-significant first.
        write_data: Vec<Bit>,
        /// Write enable.
        write_enable: Control,
        /// Fourteen physical read-address pins.
        read_address: Box<[Bit; 14]>,
        /// Logical read-data output wires, least-significant first.
        read_data: Vec<u32>,
        /// Optional read clock enable.
        read_enable: Option<Control>,
        /// Shared port clock.
        clock: Bit,
        /// Shared active clock edge.
        edge: ClockEdge,
    },
    /// Bidirectional ECP5 I/O buffer.
    TrellisIo {
        /// Stable cell name.
        name: String,
        /// Wire shared with the physical top-level pad.
        pad: u32,
        /// Value driven from the FPGA fabric toward the pad.
        fabric_output: Bit,
        /// Wire carrying the observed pad value into the FPGA fabric.
        fabric_input: u32,
        /// Active-high high-impedance control.
        tristate: Bit,
    },
    /// Dedicated ECP5 JTAG TAP access block.
    Jtagg {
        /// Stable cell name.
        name: String,
        /// Fabric data returned by extension registers one and two.
        tdo: [Bit; 2],
        /// Registered data received from the external TAP.
        tdi: u32,
        /// Clock exported by the external TAP.
        clock: u32,
        /// Run-test/idle indications for extension registers one and two.
        run_test_idle: [u32; 2],
        /// Shift-DR indication.
        shift: u32,
        /// Update-DR indication.
        update: u32,
        /// Active-low TAP reset indication.
        reset_n: u32,
        /// Extension-register clock enables.
        clock_enable: [u32; 2],
        /// Whether extension register one is present.
        extension_register_1: bool,
        /// Whether extension register two is present.
        extension_register_2: bool,
    },
    /// User-configured ECP5 phase-locked loop.
    Pll {
        /// Stable cell name.
        name: String,
        /// Physical reference-clock input.
        reference_clock: Bit,
        /// Internal feedback output wire.
        feedback_clock: u32,
        /// Fabric clock output wire.
        output_clock: u32,
        /// Lock indication output wire.
        locked: u32,
        /// PLL output routed into the core.
        fabric_output: PllOutput,
        /// PLL output looped back to `CLKFB`.
        feedback_output: PllOutput,
        /// Raw primitive parameters.
        parameters: BTreeMap<String, String>,
        /// Raw primitive attributes.
        attributes: BTreeMap<String, String>,
    },
}

/// Technology-mapped ECP5 netlist used by both simulation and nextpnr.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ecp5Netlist {
    name: String,
    ports: Vec<MappedPort>,
    cells: Vec<Ecp5Cell>,
    retiming: RetimingSelection,
    equivalence_proof: MappedEquivalenceProof,
    placement_hints: BTreeMap<String, String>,
    io_timing: ResolvedIoTiming,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct MappedEquivalenceProof {
    certified_primitive_moves: usize,
    equivalent_register_merges: usize,
    equivalent_logic_replications: usize,
    equivalent_physical_rewires: usize,
    unobservable_cells_removed: usize,
    valid: bool,
}

/// Result of the automatic, technology-scored retiming search.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RetimingSelection {
    /// Whether automatic mapped-LUT retiming selected a certified candidate.
    pub applied: bool,
    /// Maximum LUT4 levels into a register before retiming.
    pub original_lut_depth: usize,
    /// Maximum LUT4 levels into a register in the selected mapping.
    pub selected_lut_depth: usize,
    /// Register data inputs at the original maximum LUT4 depth.
    pub original_critical_registers: usize,
    /// Register data inputs remaining at the maximum depth after retiming.
    pub selected_critical_registers: usize,
    /// Estimated register-to-register period before retiming.
    pub original_period_ps: u32,
    /// Estimated register-to-register period selected for mapping.
    pub selected_period_ps: u32,
    /// Estimated worst synchronous or explicitly constrained I/O period before retiming.
    pub original_overall_period_ps: u32,
    /// Estimated worst synchronous or explicitly constrained I/O period after retiming.
    pub selected_overall_period_ps: u32,
    /// Mapped flip-flops before candidate selection.
    pub original_registers: usize,
    /// Mapped flip-flops after candidate selection.
    pub selected_registers: usize,
    /// Primitive retiming certificates composed into the selected result.
    pub certified_primitive_moves: usize,
    /// Structurally equivalent generated registers merged in the selected result.
    pub equivalent_register_merges: usize,
    /// Combinational cells replicated without changing their truth tables.
    pub equivalent_logic_replications: usize,
    /// Sinks reassigned between equivalent replicas using physical feedback.
    pub equivalent_physical_rewires: usize,
    /// Unobservable cells removed after certified retiming moves.
    pub unobservable_cells_removed: usize,
    /// Whether the complete selected transformation chain passed sign-off.
    pub equivalence_signed_off: bool,
}

impl Ecp5Netlist {
    /// Returns the top name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns physical ports with source-level vector grouping preserved.
    #[must_use]
    pub fn ports(&self) -> &[MappedPort] {
        &self.ports
    }

    /// Returns physical primitives in dependency order.
    #[must_use]
    pub fn cells(&self) -> &[Ecp5Cell] {
        &self.cells
    }

    /// Replaces logical clock/lock inputs with a user-configured `EHXPLLL`.
    ///
    /// The reference-clock port remains on the physical top. The output-clock
    /// and lock ports are removed and driven by the selected primitive output
    /// and `LOCK`. This is deliberately separate from [`Self::bind_jtagg`].
    ///
    /// Post-map cycle simulation models the configured fabric clock as the
    /// reference clock and `LOCK` as asserted; frequency generation and lock
    /// acquisition must be verified by implementation timing and hardware.
    ///
    /// # Errors
    ///
    /// Returns an error for repeated, missing, non-scalar, incorrectly
    /// directed, or constant input ports, or wire overflow.
    pub fn bind_pll(&mut self, binding: &PllBinding) -> Result<(), MappingError> {
        let roles = [
            &binding.reference_clock_port,
            &binding.output_clock_port,
            &binding.locked_port,
        ];
        let mut names = HashSet::new();
        let mut resolved = Vec::with_capacity(roles.len());
        for name in roles {
            if !names.insert(name.as_str()) {
                return Err(MappingError::PllPortRepeated(name.clone()));
            }
            let (index, port) = self
                .ports
                .iter()
                .enumerate()
                .find(|(_, port)| port.name == *name)
                .ok_or_else(|| MappingError::PllPortNotFound(name.clone()))?;
            if port.direction != PortDirection::Input {
                return Err(MappingError::PllPortDirection {
                    port: name.clone(),
                    actual: port.direction,
                });
            }
            if port.bits.len() != 1 {
                return Err(MappingError::PllPortNotScalar {
                    port: name.clone(),
                    width: port.bits.len(),
                });
            }
            let Bit::Wire(wire) = port.bits[0] else {
                return Err(MappingError::PllOutputIsConstant(name.clone()));
            };
            resolved.push((index, wire));
        }
        let output_clock = resolved[1].1;
        let feedback_clock = if binding.fabric_output == binding.feedback_output {
            output_clock
        } else {
            maximum_mapped_wire(self)
                .unwrap_or(1)
                .checked_add(1)
                .ok_or(MappingError::MappedWireOverflow)?
        };
        self.cells.push(Ecp5Cell::Pll {
            name: unique_cell_name("pll", &self.cells, None),
            reference_clock: Bit::Wire(resolved[0].1),
            feedback_clock,
            output_clock,
            locked: resolved[2].1,
            fabric_output: binding.fabric_output,
            feedback_output: binding.feedback_output,
            parameters: binding.parameters.clone(),
            attributes: binding.attributes.clone(),
        });
        let mut removed = [resolved[1].0, resolved[2].0];
        removed.sort_unstable_by(|lhs, rhs| rhs.cmp(lhs));
        for index in removed {
            self.ports.remove(index);
        }
        Ok(())
    }

    /// Replaces a scalar top-level JTAG fabric interface with the dedicated
    /// ECP5 `JTAGG` primitive.
    ///
    /// Keeping this operation at the target boundary lets the Veryl source use
    /// ordinary ports during RTL simulation. The returned mapped design no
    /// longer exposes those ports as package pins; they are connected to the
    /// device's built-in TAP instead.
    ///
    /// This operation only binds the JTAG transport. A user top or wrapper is
    /// responsible for any PLL primitive and reference/derived clock
    /// constraints required by fabric logic.
    ///
    /// # Errors
    ///
    /// Returns an error for a second `JTAGG`, repeated names, or missing,
    /// non-scalar, incorrectly directed, or constant-driven input ports.
    pub fn bind_jtagg(&mut self, binding: &JtaggBinding) -> Result<(), MappingError> {
        if self
            .cells
            .iter()
            .any(|cell| matches!(cell, Ecp5Cell::Jtagg { .. }))
        {
            return Err(MappingError::JtaggAlreadyBound);
        }

        let roles = [
            (&binding.tdo_ports[0], PortDirection::Output),
            (&binding.tdo_ports[1], PortDirection::Output),
            (&binding.tdi_port, PortDirection::Input),
            (&binding.clock_port, PortDirection::Input),
            (&binding.run_test_idle_ports[0], PortDirection::Input),
            (&binding.run_test_idle_ports[1], PortDirection::Input),
            (&binding.shift_port, PortDirection::Input),
            (&binding.update_port, PortDirection::Input),
            (&binding.reset_n_port, PortDirection::Input),
            (&binding.clock_enable_ports[0], PortDirection::Input),
            (&binding.clock_enable_ports[1], PortDirection::Input),
        ];
        let mut names = HashSet::new();
        let mut resolved = Vec::with_capacity(roles.len());
        for (name, expected) in roles {
            if !names.insert(name.as_str()) {
                return Err(MappingError::JtaggPortRepeated(name.clone()));
            }
            let (index, port) = self
                .ports
                .iter()
                .enumerate()
                .find(|(_, port)| port.name == *name)
                .ok_or_else(|| MappingError::JtaggPortNotFound(name.clone()))?;
            if port.direction != expected {
                return Err(MappingError::JtaggPortDirection {
                    port: name.clone(),
                    expected,
                    actual: port.direction,
                });
            }
            if port.bits.len() != 1 {
                return Err(MappingError::JtaggPortNotScalar {
                    port: name.clone(),
                    width: port.bits.len(),
                });
            }
            resolved.push((index, name.clone(), port.bits[0]));
        }

        let output_wire = |index: usize| match &resolved[index] {
            (_, _, Bit::Wire(wire)) => Ok(*wire),
            (_, name, Bit::Zero | Bit::One) => {
                Err(MappingError::JtaggOutputIsConstant(name.clone()))
            }
        };
        let cell = Ecp5Cell::Jtagg {
            name: unique_cell_name("jtagg", &self.cells, None),
            tdo: [resolved[0].2, resolved[1].2],
            tdi: output_wire(2)?,
            clock: output_wire(3)?,
            run_test_idle: [output_wire(4)?, output_wire(5)?],
            shift: output_wire(6)?,
            update: output_wire(7)?,
            reset_n: output_wire(8)?,
            clock_enable: [output_wire(9)?, output_wire(10)?],
            extension_register_1: binding.extension_register_1,
            extension_register_2: binding.extension_register_2,
        };

        let mut removed = resolved
            .into_iter()
            .map(|(index, _, _)| index)
            .collect::<Vec<_>>();
        removed.sort_unstable_by(|lhs, rhs| rhs.cmp(lhs));
        for index in removed {
            self.ports.remove(index);
        }
        self.cells.push(cell);
        Ok(())
    }

    /// Replaces one split scalar input/output pair with an open-drain pad.
    ///
    /// Mapping the core first keeps its two-state verification interface
    /// intact. This boundary operation then removes `input_port` and
    /// `drive_low_port`, adds the bidirectional `pin`, and emits a
    /// `TRELLIS_IO` which can only drive zero or high impedance.
    ///
    /// # Errors
    ///
    /// Returns an error for missing, non-scalar, incorrectly directed, or
    /// conflicting ports, or if no mapped wire number remains available.
    pub fn bind_open_drain_io(&mut self, binding: &OpenDrainIo) -> Result<(), MappingError> {
        let input_index = self
            .ports
            .iter()
            .position(|port| port.name == binding.input_port)
            .ok_or_else(|| MappingError::IoPortNotFound(binding.input_port.clone()))?;
        let drive_index = self
            .ports
            .iter()
            .position(|port| port.name == binding.drive_low_port)
            .ok_or_else(|| MappingError::IoPortNotFound(binding.drive_low_port.clone()))?;
        if input_index == drive_index {
            return Err(MappingError::IoPortsMustDiffer {
                input: binding.input_port.clone(),
                drive_low: binding.drive_low_port.clone(),
            });
        }
        if self.ports.iter().any(|port| port.name == binding.pin) {
            return Err(MappingError::IoPortAlreadyExists(binding.pin.clone()));
        }

        let input_port = &self.ports[input_index];
        if input_port.direction != PortDirection::Input {
            return Err(MappingError::IoPortDirection {
                port: binding.input_port.clone(),
                expected: PortDirection::Input,
                actual: input_port.direction,
            });
        }
        let drive_port = &self.ports[drive_index];
        if drive_port.direction != PortDirection::Output {
            return Err(MappingError::IoPortDirection {
                port: binding.drive_low_port.clone(),
                expected: PortDirection::Output,
                actual: drive_port.direction,
            });
        }
        if input_port.bits.len() != 1 {
            return Err(MappingError::IoPortNotScalar {
                port: binding.input_port.clone(),
                width: input_port.bits.len(),
            });
        }
        if drive_port.bits.len() != 1 {
            return Err(MappingError::IoPortNotScalar {
                port: binding.drive_low_port.clone(),
                width: drive_port.bits.len(),
            });
        }
        let Bit::Wire(fabric_input) = input_port.bits[0] else {
            return Err(MappingError::IoInputIsConstant(binding.input_port.clone()));
        };
        let drive_low = drive_port.bits[0];
        let pad = maximum_mapped_wire(self)
            .unwrap_or(1)
            .checked_add(1)
            .ok_or(MappingError::MappedWireOverflow)?;
        let tristate = match drive_low {
            Bit::Zero => Bit::One,
            Bit::One => Bit::Zero,
            Bit::Wire(wire) => {
                let inverted = pad.checked_add(1).ok_or(MappingError::MappedWireOverflow)?;
                self.cells.push(Ecp5Cell::Lut4 {
                    name: unique_cell_name(
                        &format!("io_{}_drive_low_invert", binding.pin),
                        &self.cells,
                        None,
                    ),
                    inputs: [Bit::Wire(wire), Bit::Zero, Bit::Zero, Bit::Zero],
                    output: inverted,
                    // A is the least-significant LUT index bit.
                    init: 0x5555,
                });
                Bit::Wire(inverted)
            }
        };
        self.cells.push(Ecp5Cell::TrellisIo {
            name: unique_cell_name(&format!("io_{}", binding.pin), &self.cells, None),
            pad,
            fabric_output: Bit::Zero,
            fabric_input,
            tristate,
        });

        let insertion_index = input_index.min(drive_index);
        let mut removed = [input_index, drive_index];
        removed.sort_unstable_by(|lhs, rhs| rhs.cmp(lhs));
        for index in removed {
            self.ports.remove(index);
        }
        self.ports.insert(
            insertion_index,
            MappedPort {
                name: binding.pin.clone(),
                direction: PortDirection::Inout,
                bits: vec![Bit::Wire(pad)],
            },
        );
        Ok(())
    }

    /// Atomically binds multiple split interfaces to open-drain pads.
    ///
    /// # Errors
    ///
    /// Returns the first binding error without changing this netlist.
    pub fn bind_open_drain_ios(&mut self, bindings: &[OpenDrainIo]) -> Result<(), MappingError> {
        let mut candidate = self.clone();
        for binding in bindings {
            candidate.bind_open_drain_io(binding)?;
        }
        *self = candidate;
        Ok(())
    }

    /// Returns the technology-scored retiming decision made during mapping.
    #[must_use]
    pub const fn retiming(&self) -> RetimingSelection {
        self.retiming
    }

    /// Serializes this exact mapped object to the Yosys JSON schema consumed by
    /// nextpnr-ecp5.
    ///
    /// # Errors
    ///
    /// Returns an error if two cells share a name or JSON serialization fails.
    pub fn to_nextpnr_json(&self) -> Result<String, NextpnrJsonError> {
        let mut seen = HashSet::new();
        if let Some(duplicate) = self
            .cells
            .iter()
            .map(mapped_cell_name)
            .find(|name| !seen.insert((*name).to_owned()))
        {
            return Err(NextpnrJsonError::DuplicateCellName {
                name: duplicate.to_owned(),
            });
        }
        serde_json::to_string_pretty(&JsonDesign::from(self))
            .map_err(NextpnrJsonError::Serialization)
    }

    /// Applies equivalent local rewrites using locations and routed timing
    /// returned by a deterministic draft implementation.
    #[must_use]
    pub fn apply_physical_feedback(&self, feedback: &PhysicalFeedback) -> Self {
        let mut refined = self.clone();
        if feedback.meets_timing_goal() || !physical_feedback_matches_netlist(&refined, feedback) {
            return refined;
        }
        let (replicas, critical_rewires, cluster_rewires) =
            if feedback.is_near_timing_closure(PHYSICAL_REWRITE_MIN_GOAL_PERCENT) {
                let (replicas, critical_rewires) =
                    replicate_physically_critical_luts(&mut refined, feedback);
                let cluster_rewires = recluster_replicated_enable_sinks(&mut refined, feedback);
                (replicas, critical_rewires, cluster_rewires)
            } else {
                (0, 0, 0)
            };
        let rewires = critical_rewires + cluster_rewires;
        let physical_retiming_moves =
            if rewires == 0 && feedback.is_near_timing_closure(PHYSICAL_RETIME_MIN_GOAL_PERCENT) {
                physically_retime_reported_cones(&mut refined, feedback)
            } else {
                0
            };
        if rewires == 0 && physical_retiming_moves == 0 {
            return refined;
        }
        refined.placement_hints = if physical_retiming_moves == 0 {
            refined
                .cells
                .iter()
                .filter_map(|cell| {
                    let name = mapped_cell_name(cell);
                    let bel = feedback.bel(name)?;
                    physical_bel_is_compatible(cell, bel).then(|| (name.to_owned(), bel.to_owned()))
                })
                .collect()
        } else {
            BTreeMap::new()
        };
        refined.equivalence_proof.equivalent_logic_replications += replicas;
        refined.equivalence_proof.equivalent_physical_rewires += rewires;
        let profile = mapped_lut_profile(&refined);
        refined.retiming.applied |= physical_retiming_moves > 0;
        refined.retiming.selected_lut_depth = profile.data_depth;
        refined.retiming.selected_critical_registers = profile.critical_depth.len();
        refined.retiming.selected_period_ps = profile.data_period_ps;
        refined.retiming.selected_overall_period_ps = profile.overall_period_ps;
        refined.retiming.selected_registers = mapped_register_count(&refined);
        refined.retiming.certified_primitive_moves =
            refined.equivalence_proof.certified_primitive_moves;
        refined.retiming.equivalent_register_merges =
            refined.equivalence_proof.equivalent_register_merges;
        refined.retiming.equivalent_logic_replications =
            refined.equivalence_proof.equivalent_logic_replications;
        refined.retiming.equivalent_physical_rewires =
            refined.equivalence_proof.equivalent_physical_rewires;
        refined.retiming.unobservable_cells_removed =
            refined.equivalence_proof.unobservable_cells_removed;
        refined.retiming.equivalence_signed_off =
            verify_mapped_equivalence_proof(&refined, refined.retiming.applied);
        if refined.retiming.equivalence_signed_off {
            refined
        } else {
            self.clone()
        }
    }

    /// Returns a deterministic, bounded set of equivalent physical-synthesis
    /// candidates. The first entry preserves the single-candidate behavior of
    /// [`Self::apply_physical_feedback`]; later entries extend a routed
    /// critical-cone retime by one additional certified primitive move.
    #[must_use]
    pub fn physical_feedback_candidates(&self, feedback: &PhysicalFeedback) -> Vec<Self> {
        let first = self.apply_physical_feedback(feedback);
        if first == *self {
            return Vec::new();
        }
        let first_physical_moves = first
            .equivalence_proof
            .certified_primitive_moves
            .saturating_sub(self.equivalence_proof.certified_primitive_moves);
        let mut candidates = vec![first.clone()];
        if first_physical_moves == 0 {
            return candidates;
        }
        for forward in physically_forward_retime_reported_luts(self, feedback) {
            if candidates.len() >= crate::MAX_PHYSICAL_CANDIDATES {
                break;
            }
            if !candidates.contains(&forward) {
                candidates.push(forward);
            }
        }
        let original_names = self
            .cells
            .iter()
            .map(mapped_cell_name)
            .collect::<HashSet<_>>();
        let critical_cells = feedback
            .critical_paths()
            .iter()
            .filter(|path| path.register_to_register)
            .flat_map(|path| &path.cells)
            .map(String::as_str)
            .collect::<Vec<_>>();
        let path_driven_new_registers = first
            .cells
            .iter()
            .enumerate()
            .filter_map(|(index, cell)| {
                let Ecp5Cell::FlipFlop {
                    name,
                    data: Bit::Wire(data),
                    ..
                } = cell
                else {
                    return None;
                };
                (!original_names.contains(name.as_str())
                    && first.cells.iter().any(|driver| {
                        critical_cells.iter().any(|physical| {
                            physical_path_matches_mapped_cell(physical, mapped_cell_name(driver))
                        }) && cell_output_bits(driver).contains(&Bit::Wire(*data))
                    }))
                .then_some((name.clone(), index))
            })
            .collect::<BTreeMap<_, _>>();
        for (_, register) in path_driven_new_registers {
            if candidates.len() >= crate::MAX_PHYSICAL_CANDIDATES {
                break;
            }
            let Some(mut extended) = backward_retime_primitive(&first, register) else {
                continue;
            };
            merge_equivalent_flip_flops(&mut extended);
            if !physical_retime_step_is_bounded(
                &first,
                &extended,
                2 * PHYSICAL_RETIME_MODEL_BRIDGE_PS,
            ) {
                continue;
            }
            let Some(extended) = finalize_physical_retiming_candidate(extended) else {
                continue;
            };
            if !candidates.contains(&extended) {
                candidates.push(extended);
            }
        }
        candidates
    }
}

fn physically_forward_retime_reported_luts(
    netlist: &Ecp5Netlist,
    feedback: &PhysicalFeedback,
) -> Vec<Ecp5Netlist> {
    let physical_cells = feedback
        .critical_paths()
        .iter()
        .filter(|path| path.register_to_register)
        .flat_map(|path| &path.cells)
        .map(String::as_str)
        .collect::<HashSet<_>>();
    let lut_names = netlist
        .cells
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::Lut4 { name, .. } if physical_cells.contains(name.as_str()) => {
                Some(name.clone())
            }
            _ => None,
        })
        .collect::<BTreeSet<_>>();
    lut_names
        .into_iter()
        .filter_map(|name| {
            let index = netlist
                .cells
                .iter()
                .position(|cell| mapped_cell_name(cell) == name)?;
            let mut candidate = forward_retime_lut(netlist, index)?;
            merge_equivalent_flip_flops(&mut candidate);
            physical_retime_step_is_bounded(
                netlist,
                &candidate,
                2 * PHYSICAL_RETIME_MODEL_BRIDGE_PS,
            )
            .then(|| finalize_physical_retiming_candidate(candidate))?
        })
        .collect()
}

fn finalize_physical_retiming_candidate(mut candidate: Ecp5Netlist) -> Option<Ecp5Netlist> {
    candidate.placement_hints.clear();
    let profile = mapped_lut_profile(&candidate);
    candidate.retiming.applied = true;
    candidate.retiming.selected_lut_depth = profile.data_depth;
    candidate.retiming.selected_critical_registers = profile.critical_depth.len();
    candidate.retiming.selected_period_ps = profile.data_period_ps;
    candidate.retiming.selected_overall_period_ps = profile.overall_period_ps;
    candidate.retiming.selected_registers = mapped_register_count(&candidate);
    candidate.retiming.certified_primitive_moves =
        candidate.equivalence_proof.certified_primitive_moves;
    candidate.retiming.equivalent_register_merges =
        candidate.equivalence_proof.equivalent_register_merges;
    candidate.retiming.equivalent_logic_replications =
        candidate.equivalence_proof.equivalent_logic_replications;
    candidate.retiming.equivalent_physical_rewires =
        candidate.equivalence_proof.equivalent_physical_rewires;
    candidate.retiming.unobservable_cells_removed =
        candidate.equivalence_proof.unobservable_cells_removed;
    candidate.retiming.equivalence_signed_off = verify_mapped_equivalence_proof(&candidate, true);
    candidate
        .retiming
        .equivalence_signed_off
        .then_some(candidate)
}

fn physical_path_matches_mapped_cell(physical: &str, mapped: &str) -> bool {
    physical == mapped
        || physical
            .strip_prefix(mapped)
            .is_some_and(|suffix| suffix.starts_with('$'))
}

fn physical_retime_step_is_bounded(
    before: &Ecp5Netlist,
    candidate: &Ecp5Netlist,
    model_bridge_ps: u32,
) -> bool {
    let before_profile = mapped_lut_profile(before);
    let candidate_profile = mapped_lut_profile(candidate);
    candidate.cells.len() <= before.cells.len() + 8
        && mapped_register_count(candidate) <= mapped_register_count(before) + 4
        && candidate_profile.data_period_ps
            <= before_profile
                .data_period_ps
                .saturating_add(model_bridge_ps)
        && candidate_profile.overall_period_ps
            <= before_profile
                .overall_period_ps
                .saturating_add(model_bridge_ps)
        && verify_mapped_equivalence_proof(candidate, true)
}

fn physically_retime_reported_cones(
    netlist: &mut Ecp5Netlist,
    feedback: &PhysicalFeedback,
) -> usize {
    const MAX_PHYSICAL_RETIMING_MOVES: usize = 4;

    let sink_names = feedback
        .critical_paths()
        .iter()
        .filter(|path| path.register_to_register)
        .filter_map(|path| {
            path.cells.iter().rev().find(|name| {
                netlist.cells.iter().any(|cell| {
                    mapped_cell_name(cell) == name.as_str()
                        && matches!(cell, Ecp5Cell::FlipFlop { .. })
                })
            })
        })
        .cloned()
        .collect::<BTreeSet<_>>();
    let mut moves = 0usize;
    for sink_name in sink_names {
        if moves >= MAX_PHYSICAL_RETIMING_MOVES {
            break;
        }
        let Some(register) = netlist
            .cells
            .iter()
            .position(|cell| mapped_cell_name(cell) == sink_name)
        else {
            continue;
        };
        let Some(mut candidate) = backward_retime_primitive(netlist, register) else {
            continue;
        };
        merge_equivalent_flip_flops(&mut candidate);
        if !physical_retime_step_is_bounded(netlist, &candidate, PHYSICAL_RETIME_MODEL_BRIDGE_PS) {
            continue;
        }
        *netlist = candidate;
        moves += 1;
    }
    moves
}

fn physical_feedback_matches_netlist(netlist: &Ecp5Netlist, feedback: &PhysicalFeedback) -> bool {
    let mut expected = 0usize;
    let mut missing = 0usize;
    for cell in &netlist.cells {
        if matches!(cell, Ecp5Cell::Ccu2c { .. }) {
            continue;
        }
        expected += 1;
        match feedback.bel(mapped_cell_name(cell)) {
            Some(bel) if physical_bel_is_compatible(cell, bel) => {}
            Some(_) => return false,
            None => missing += 1,
        }
    }
    expected > 0 && missing <= 2usize.max(expected / 100)
}

fn physical_bel_is_compatible(cell: &Ecp5Cell, bel: &str) -> bool {
    match cell {
        Ecp5Cell::Lut4 { .. } => bel.contains(".K"),
        Ecp5Cell::FlipFlop { .. } => bel.contains(".FF"),
        Ecp5Cell::BlockRam { .. } => bel.contains("DP16KD"),
        Ecp5Cell::TrellisIo { .. } => bel.contains("PIO"),
        Ecp5Cell::Jtagg { .. } => bel.contains("JTAGG"),
        Ecp5Cell::Pll { .. } => bel.contains("PLL"),
        Ecp5Cell::Ccu2c { .. } => false,
    }
}

/// Maps target-independent Boolean nodes and registers to ECP5 primitives.
///
/// # Errors
///
/// Returns an error if the source netlist is invalid.
pub fn map_to_ecp5(netlist: &Netlist) -> Result<Ecp5Netlist, MappingError> {
    map_to_ecp5_with_options(netlist, MappingOptions::default())
}

/// Maps a core and binds its split open-drain interfaces to physical pads.
///
/// # Errors
///
/// Returns an error if logic mapping or any I/O binding fails.
pub fn map_to_ecp5_with_open_drain_ios(
    netlist: &Netlist,
    bindings: &[OpenDrainIo],
) -> Result<Ecp5Netlist, MappingError> {
    let mut mapped = map_to_ecp5(netlist)?;
    mapped.bind_open_drain_ios(bindings)?;
    Ok(mapped)
}

/// Maps a core and binds its scalar debug interface to the dedicated ECP5 TAP.
///
/// # Errors
///
/// Returns an error if logic mapping or the `JTAGG` binding fails.
pub fn map_to_ecp5_with_jtagg(
    netlist: &Netlist,
    binding: &JtaggBinding,
) -> Result<Ecp5Netlist, MappingError> {
    let mut mapped = map_to_ecp5(netlist)?;
    mapped.bind_jtagg(binding)?;
    Ok(mapped)
}

/// Maps a core and applies a user-configured ECP5 PLL boundary binding.
///
/// # Errors
///
/// Returns an error if logic mapping or the PLL binding fails.
pub fn map_to_ecp5_with_pll(
    netlist: &Netlist,
    binding: &PllBinding,
) -> Result<Ecp5Netlist, MappingError> {
    let mut mapped = map_to_ecp5(netlist)?;
    mapped.bind_pll(binding)?;
    Ok(mapped)
}

/// Maps a target-independent netlist with explicit arithmetic choices.
///
/// # Errors
///
/// Returns an error if the source netlist is invalid.
pub fn map_to_ecp5_with_options(
    netlist: &Netlist,
    options: MappingOptions,
) -> Result<Ecp5Netlist, MappingError> {
    map_to_ecp5_with_constraints(netlist, options, &IoTimingConstraints::new())
}

/// Maps a target-independent netlist with explicit implementation and I/O
/// timing constraints.
///
/// Top-level ports omitted from `io_timing` remain unconstrained. The global
/// frequency applies to synchronous paths and to only those I/O paths named by
/// these constraints.
///
/// # Errors
///
/// Returns an error if the source netlist or an I/O timing constraint is
/// invalid.
pub fn map_to_ecp5_with_constraints(
    netlist: &Netlist,
    options: MappingOptions,
    io_timing: &IoTimingConstraints,
) -> Result<Ecp5Netlist, MappingError> {
    validate_io_timing(netlist, io_timing)?;
    let (mut selected, _) = map_once(netlist, options, io_timing)?;
    let original_cells = selected.cells.len();
    let original_registers = netlist.registers().len();
    let mut selected_registers = original_registers;
    let mut applied = false;
    let mapped_original_profile = mapped_lut_profile(&selected);
    let target_period_ps = 1_000_000u32
        .div_ceil(options.timing_goal_mhz.max(1))
        .saturating_mul(RETIMING_PERIOD_MARGIN_NUMERATOR)
        / RETIMING_PERIOD_MARGIN_DENOMINATOR;
    if let Some(retimed) = automatically_retime_mapped_luts(
        &selected,
        original_cells,
        original_registers,
        target_period_ps,
    )
    .filter(|retimed| verify_mapped_equivalence_proof(retimed, true))
    {
        selected = retimed;
        selected_registers = mapped_register_count(&selected);
        applied = true;
    }
    split_branched_carry_outs(&mut selected);
    let mapped_selected_profile = mapped_lut_profile(&selected);
    let equivalence_signed_off = verify_mapped_equivalence_proof(&selected, applied);
    selected.retiming = RetimingSelection {
        applied,
        original_lut_depth: mapped_original_profile.data_depth,
        selected_lut_depth: mapped_selected_profile.data_depth,
        original_critical_registers: mapped_original_profile.critical_depth.len(),
        selected_critical_registers: mapped_selected_profile.critical_depth.len(),
        original_period_ps: mapped_original_profile.data_period_ps,
        selected_period_ps: mapped_selected_profile.data_period_ps,
        original_overall_period_ps: mapped_original_profile.overall_period_ps,
        selected_overall_period_ps: mapped_selected_profile.overall_period_ps,
        original_registers,
        selected_registers,
        certified_primitive_moves: selected.equivalence_proof.certified_primitive_moves,
        equivalent_register_merges: selected.equivalence_proof.equivalent_register_merges,
        equivalent_logic_replications: selected.equivalence_proof.equivalent_logic_replications,
        equivalent_physical_rewires: selected.equivalence_proof.equivalent_physical_rewires,
        unobservable_cells_removed: selected.equivalence_proof.unobservable_cells_removed,
        equivalence_signed_off,
    };
    Ok(selected)
}

#[allow(clippy::too_many_lines)]
fn automatically_retime_mapped_luts(
    original: &Ecp5Netlist,
    original_cells: usize,
    original_registers: usize,
    target_period_ps: u32,
) -> Option<Ecp5Netlist> {
    let original_profile = mapped_lut_profile(original);
    let original_depth = original_profile.data_depth;
    let baseline_burden = routing_burden(original);
    let adjusted_overall = |profile: &MappedLutProfile, cells_now: &Ecp5Netlist| {
        congestion_adjusted_overall(
            profile.overall_period_ps,
            routing_burden(cells_now),
            baseline_burden,
        )
    };
    let timing_driven = original_profile.overall_period_ps > target_period_ps;
    let cell_limit = original_cells + original_cells.div_ceil(10);
    let register_limit = original_registers + original_registers.div_ceil(5);
    // Combinational primitives deserve their own ceiling: a candidate can
    // grow LUTs enormously while keeping cells and registers inside their
    // caps, and the routing-burden term alone does not catch that (splitting
    // one loaded net into many cheap ones lowers the burden).
    let original_comb = mapped_comb_count(original);
    let comb_limit = original_comb + original_comb.div_ceil(4) + 32;
    let mut control_candidate =
        replicate_high_fanout_enable_luts(original, MAX_ENABLE_FANOUT_PER_REPLICA);
    split_branched_carry_outs(&mut control_candidate);
    let control_profile = mapped_lut_profile(&control_candidate);
    let control_registers = mapped_register_count(&control_candidate);
    let original_enable_fanout =
        maximum_replicable_enable_fanout(original, MAX_ENABLE_FANOUT_PER_REPLICA);
    let control_enable_fanout =
        maximum_replicable_enable_fanout(&control_candidate, MAX_ENABLE_FANOUT_PER_REPLICA);
    let use_control = control_candidate.cells.len() <= cell_limit
        && control_registers <= register_limit
        && mapped_comb_count(&control_candidate) <= comb_limit
        && adjusted_overall(&control_profile, &control_candidate)
            <= original_profile.overall_period_ps
        && (retiming_score(
            &control_profile,
            timing_driven,
            control_candidate.cells.len(),
            control_registers,
            adjusted_overall(&control_profile, &control_candidate),
        ) < retiming_score(
            &original_profile,
            timing_driven,
            original.cells.len(),
            mapped_register_count(original),
            original_profile.overall_period_ps,
        ) || control_enable_fanout < original_enable_fanout);
    let seed = if use_control {
        &control_candidate
    } else {
        original
    };
    let mut forward_candidate = forward_retime_registered_ccu_chains(seed, timing_driven);
    split_branched_carry_outs(&mut forward_candidate);
    let forward_profile = mapped_lut_profile(&forward_candidate);
    let forward_registers = mapped_register_count(&forward_candidate);
    let use_forward = forward_candidate.cells.len() <= cell_limit
        && forward_registers <= register_limit
        && mapped_comb_count(&forward_candidate) <= comb_limit
        && adjusted_overall(&forward_profile, &forward_candidate)
            <= original_profile.overall_period_ps
        && retiming_score(
            &forward_profile,
            timing_driven,
            forward_candidate.cells.len(),
            forward_registers,
            adjusted_overall(&forward_profile, &forward_candidate),
        ) < retiming_score(
            &original_profile,
            timing_driven,
            original.cells.len(),
            mapped_register_count(original),
            original_profile.overall_period_ps,
        );
    let mut frontier = if use_forward {
        forward_candidate
    } else {
        seed.clone()
    };
    let mut best_seen = frontier.clone();
    let mut bridge_budget = 2usize;
    for _ in 0..128 {
        let profile = mapped_lut_profile(&frontier);
        let best_profile = mapped_lut_profile(&best_seen);
        if retiming_score(
            &profile,
            timing_driven,
            frontier.cells.len(),
            mapped_register_count(&frontier),
            adjusted_overall(&profile, &frontier),
        ) < retiming_score(
            &best_profile,
            timing_driven,
            best_seen.cells.len(),
            mapped_register_count(&best_seen),
            adjusted_overall(&best_profile, &best_seen),
        ) {
            best_seen = frontier.clone();
        }
        if (timing_driven && adjusted_overall(&profile, &frontier) <= target_period_ps)
            || (!timing_driven && profile.data_depth < original_depth)
        {
            return Some(frontier);
        }
        let mut best = None;
        let critical = if timing_driven {
            profile.critical_timing.clone()
        } else {
            profile.critical_depth.clone()
        };
        for &register in &critical {
            let Some(mut candidate) = backward_retime_primitive(&frontier, register) else {
                continue;
            };
            merge_equivalent_flip_flops(&mut candidate);
            let candidate_profile = mapped_lut_profile(&candidate);
            let candidate_registers = mapped_register_count(&candidate);
            let candidate_overall = adjusted_overall(&candidate_profile, &candidate);
            if (!timing_driven && candidate_profile.overall_depth > original_profile.overall_depth)
                || candidate_overall > original_profile.overall_period_ps
                || candidate.cells.len() > cell_limit
                || candidate_registers > register_limit
                || mapped_comb_count(&candidate) > comb_limit
            {
                continue;
            }
            let score = retiming_score(
                &candidate_profile,
                timing_driven,
                candidate.cells.len(),
                candidate_registers,
                candidate_overall,
            );
            let frontier_score = retiming_score(
                &profile,
                timing_driven,
                frontier.cells.len(),
                mapped_register_count(&frontier),
                adjusted_overall(&profile, &frontier),
            );
            if score < frontier_score
                && best
                    .as_ref()
                    .is_none_or(|(_, best_score): &(Ecp5Netlist, _)| score < *best_score)
                && carry_outs_are_point_to_point(&candidate)
            {
                best = Some((candidate, score));
            }
        }
        // Equal-delay endpoints form a timing cutset: moving any one endpoint
        // leaves the maximum unchanged, even though moving the whole cutset is
        // profitable. Stable names survive index shifts caused by pruning
        // unobservable generated cells after every certified move.
        let batch_registers = critical
            .iter()
            .map(|index| mapped_cell_name(&frontier.cells[*index]).to_owned())
            .collect::<Vec<_>>();
        let mut batch = frontier.clone();
        let mut batch_moves = 0usize;
        for name in batch_registers {
            let Some(register) = batch
                .cells
                .iter()
                .position(|cell| mapped_cell_name(cell) == name)
            else {
                continue;
            };
            let Some(candidate) = backward_retime_primitive(&batch, register) else {
                continue;
            };
            let mut candidate = candidate;
            split_branched_carry_outs(&mut candidate);
            let candidate_profile = mapped_lut_profile(&candidate);
            let candidate_registers = mapped_register_count(&candidate);
            if (timing_driven || candidate_profile.overall_depth <= original_profile.overall_depth)
                && candidate_profile.overall_period_ps <= original_profile.overall_period_ps
                && candidate.cells.len() <= cell_limit
                && candidate_registers <= register_limit
                && carry_outs_are_point_to_point(&candidate)
            {
                batch = candidate;
                batch_moves += 1;
            }
        }
        let mut bridge_candidate = None;
        if timing_driven && bridge_budget > 0 {
            let ccu_period = profile
                .timing_endpoints
                .iter()
                .filter_map(|(index, period)| {
                    (*period > target_period_ps
                        && register_data_is_driven_by_ccu(&frontier, *index))
                    .then_some(*period)
                })
                .max();
            if let Some(ccu_period) = ccu_period {
                let endpoint_names = profile
                    .timing_endpoints
                    .iter()
                    .filter(|(index, period)| {
                        *period == ccu_period && register_data_is_driven_by_ccu(&frontier, *index)
                    })
                    .map(|(index, _)| mapped_cell_name(&frontier.cells[*index]).to_owned())
                    .collect::<Vec<_>>();
                let mut ccu_batch = frontier.clone();
                let mut ccu_moves = 0usize;
                for name in &endpoint_names {
                    let Some(index) = ccu_batch
                        .cells
                        .iter()
                        .position(|cell| mapped_cell_name(cell) == name)
                    else {
                        continue;
                    };
                    let Some(candidate) = backward_retime_ccu2c(&ccu_batch, index) else {
                        continue;
                    };
                    let mut candidate = candidate;
                    split_branched_carry_outs(&mut candidate);
                    let candidate_profile = mapped_lut_profile(&candidate);
                    let candidate_registers = mapped_register_count(&candidate);
                    if adjusted_overall(&candidate_profile, &candidate)
                        <= original_profile.overall_period_ps
                        && candidate.cells.len() <= cell_limit
                        && candidate_registers <= register_limit
                    {
                        ccu_batch = candidate;
                        ccu_moves += 1;
                    }
                }
                if ccu_moves == endpoint_names.len() && ccu_moves > 0 {
                    merge_equivalent_flip_flops(&mut ccu_batch);
                    let batch_profile = mapped_lut_profile(&ccu_batch);
                    let batch_cells = ccu_batch.cells.len();
                    let batch_registers = mapped_register_count(&ccu_batch);
                    let score = retiming_score(
                        &batch_profile,
                        timing_driven,
                        batch_cells,
                        batch_registers,
                        adjusted_overall(&batch_profile, &ccu_batch),
                    );
                    if bridge_candidate
                        .as_ref()
                        .is_none_or(|(_, bridge_score)| score < *bridge_score)
                    {
                        bridge_candidate = Some((ccu_batch, score));
                    }
                }
            }
        }
        if batch_moves > 0 {
            merge_equivalent_flip_flops(&mut batch);
            let batch_profile = mapped_lut_profile(&batch);
            let batch_score = retiming_score(
                &batch_profile,
                timing_driven,
                batch.cells.len(),
                mapped_register_count(&batch),
                adjusted_overall(&batch_profile, &batch),
            );
            let frontier_score = retiming_score(
                &profile,
                timing_driven,
                frontier.cells.len(),
                mapped_register_count(&frontier),
                adjusted_overall(&profile, &frontier),
            );
            if batch_score < frontier_score
                && best
                    .as_ref()
                    .is_none_or(|(_, best_score): &(Ecp5Netlist, _)| batch_score < *best_score)
            {
                best = Some((batch, batch_score));
            } else if bridge_candidate.is_none()
                && timing_driven
                && profile.data_period_ps > target_period_ps
                && batch_profile.data_period_ps
                    <= profile
                        .data_period_ps
                        .saturating_add(2 * MAPPED_ROUTE_GUARD_PS)
                && batch_moves == critical.len()
                && bridge_budget > 0
            {
                bridge_candidate = Some((batch, batch_score));
            }
        }
        let selected_bridge = best.is_none() && bridge_candidate.is_some();
        let Some((candidate, _)) = best.or(bridge_candidate) else {
            break;
        };
        if selected_bridge {
            bridge_budget -= 1;
        }
        frontier = candidate;
    }
    let frontier_profile = mapped_lut_profile(&frontier);
    let best_profile = mapped_lut_profile(&best_seen);
    if retiming_score(
        &frontier_profile,
        timing_driven,
        frontier.cells.len(),
        mapped_register_count(&frontier),
        adjusted_overall(&frontier_profile, &frontier),
    ) < retiming_score(
        &best_profile,
        timing_driven,
        best_seen.cells.len(),
        mapped_register_count(&best_seen),
        adjusted_overall(&best_profile, &best_seen),
    ) {
        best_seen = frontier;
    }
    let replicated = replicate_high_fanout_enable_luts(&best_seen, MAX_ENABLE_FANOUT_PER_REPLICA);
    let best_profile = mapped_lut_profile(&best_seen);
    let replicated_profile = mapped_lut_profile(&replicated);
    if replicated.cells.len() <= cell_limit
        && mapped_register_count(&replicated) <= register_limit
        && replicated_profile.overall_period_ps <= best_profile.overall_period_ps
        && maximum_replicable_enable_fanout(&replicated, MAX_ENABLE_FANOUT_PER_REPLICA)
            < maximum_replicable_enable_fanout(&best_seen, MAX_ENABLE_FANOUT_PER_REPLICA)
    {
        best_seen = replicated;
    }
    let selected_profile = mapped_lut_profile(&best_seen);
    let improved = if timing_driven {
        (
            selected_profile.overall_period_ps,
            selected_profile.data_period_ps,
            selected_profile.critical_timing.len(),
            maximum_replicable_enable_fanout(&best_seen, MAX_ENABLE_FANOUT_PER_REPLICA),
        ) < (
            original_profile.overall_period_ps,
            original_profile.data_period_ps,
            original_profile.critical_timing.len(),
            maximum_replicable_enable_fanout(original, MAX_ENABLE_FANOUT_PER_REPLICA),
        )
    } else {
        (
            selected_profile.data_depth,
            selected_profile.critical_depth.len(),
        ) < (
            original_profile.data_depth,
            original_profile.critical_depth.len(),
        )
    };
    improved.then_some(best_seen)
}

/// Retiming moves add primitives, and added primitives lengthen real routes
/// everywhere even though flat per-hop estimates cannot see it. Charge each
/// percent of routing-burden growth beyond a small allowance against the
/// claimed period so aggressive trajectories pay for their own congestion.
const CONGESTION_ALLOWANCE_PERCENT: u64 = 2;
const CONGESTION_PENALTY_PS_PER_PERCENT: u64 = 250;

fn congestion_adjusted_overall(overall_period_ps: u32, burden: u64, baseline_burden: u64) -> u32 {
    let growth_ppm =
        (burden.saturating_sub(baseline_burden)).saturating_mul(1_000_000) / baseline_burden.max(1);
    let allowance_ppm = CONGESTION_ALLOWANCE_PERCENT * 10_000;
    let excess_ppm = growth_ppm.saturating_sub(allowance_ppm);
    overall_period_ps.saturating_add(
        u32::try_from(excess_ppm.saturating_mul(CONGESTION_PENALTY_PS_PER_PERCENT) / 10_000)
            .unwrap_or(u32::MAX),
    )
}

fn retiming_score(
    profile: &MappedLutProfile,
    timing_driven: bool,
    cells: usize,
    registers: usize,
    overall_period_ps: u32,
) -> (u64, u64, u64, usize, usize, usize) {
    if timing_driven {
        (
            u64::from(overall_period_ps),
            u64::from(profile.data_period_ps),
            u64::try_from(profile.critical_timing.len()).unwrap_or(u64::MAX),
            profile.data_depth,
            cells,
            registers,
        )
    } else {
        (
            u64::try_from(profile.data_depth).unwrap_or(u64::MAX),
            u64::try_from(profile.critical_depth.len()).unwrap_or(u64::MAX),
            u64::from(overall_period_ps),
            usize::try_from(profile.data_period_ps).unwrap_or(usize::MAX),
            cells,
            registers,
        )
    }
}

fn mapped_comb_count(netlist: &Ecp5Netlist) -> usize {
    netlist
        .cells
        .iter()
        .filter(|cell| matches!(cell, Ecp5Cell::Lut4 { .. } | Ecp5Cell::Ccu2c { .. }))
        .count()
}

fn mapped_register_count(netlist: &Ecp5Netlist) -> usize {
    netlist
        .cells
        .iter()
        .filter(|cell| matches!(cell, Ecp5Cell::FlipFlop { .. }))
        .count()
}

fn verify_mapped_equivalence_proof(netlist: &Ecp5Netlist, applied: bool) -> bool {
    if !netlist.equivalence_proof.valid
        || (applied
            && netlist.equivalence_proof.certified_primitive_moves == 0
            && netlist.equivalence_proof.equivalent_logic_replications == 0)
    {
        return false;
    }
    let primary_inputs = netlist
        .ports
        .iter()
        .filter(|port| port.direction == PortDirection::Input)
        .flat_map(|port| &port.bits)
        .filter_map(|bit| match bit {
            Bit::Wire(wire) => Some(*wire),
            Bit::Zero | Bit::One => None,
        })
        .collect::<HashSet<_>>();
    let mut driven = primary_inputs.clone();
    for bit in netlist.cells.iter().flat_map(cell_output_bits) {
        let Bit::Wire(wire) = bit else {
            continue;
        };
        if !driven.insert(wire) {
            return false;
        }
    }
    netlist
        .cells
        .iter()
        .flat_map(cell_input_bits)
        .chain(
            netlist
                .ports
                .iter()
                .filter(|port| port.direction == PortDirection::Output)
                .flat_map(|port| &port.bits)
                .copied(),
        )
        .all(|bit| match bit {
            Bit::Wire(wire) => driven.contains(&wire),
            Bit::Zero | Bit::One => true,
        })
}

fn replicate_high_fanout_enable_luts(netlist: &Ecp5Netlist, max_fanout: usize) -> Ecp5Netlist {
    let max_fanout = max_fanout.max(1);
    let fanouts = mapped_wire_fanouts(netlist);
    let mut enable_sinks = BTreeMap::<u32, Vec<usize>>::new();
    for (index, cell) in netlist.cells.iter().enumerate() {
        let Ecp5Cell::FlipFlop {
            enable: Some(enable),
            ..
        } = cell
        else {
            continue;
        };
        let Bit::Wire(wire) = enable.signal else {
            continue;
        };
        let wire_fanout = fanouts.get(&wire).copied().unwrap_or(0);
        if wire_fanout > max_fanout && wire_fanout <= max_fanout.saturating_mul(2) {
            enable_sinks.entry(wire).or_default().push(index);
        }
    }

    let mut candidate = netlist.clone();
    let mut next_wire = maximum_mapped_wire(netlist)
        .and_then(|wire| wire.checked_add(1))
        .unwrap_or(1);
    for (wire, sinks) in enable_sinks {
        if sinks.len() <= max_fanout {
            continue;
        }
        let Some((driver_name, inputs, init)) = netlist.cells.iter().find_map(|cell| match cell {
            Ecp5Cell::Lut4 {
                name,
                inputs,
                output,
                init,
            } if *output == wire => Some((name.clone(), *inputs, *init)),
            _ => None,
        }) else {
            continue;
        };
        let non_enable_fanout = fanouts
            .get(&wire)
            .copied()
            .unwrap_or(0)
            .saturating_sub(sinks.len());
        let retained = max_fanout
            .saturating_sub(non_enable_fanout)
            .min(sinks.len());
        for (replica, chunk) in sinks[retained..].chunks(max_fanout).enumerate() {
            let output = next_wire;
            let Some(allocated) = next_wire.checked_add(1) else {
                return netlist.clone();
            };
            next_wire = allocated;
            for &sink in chunk {
                let Ecp5Cell::FlipFlop {
                    enable: Some(enable),
                    ..
                } = &mut candidate.cells[sink]
                else {
                    return netlist.clone();
                };
                if enable.signal != Bit::Wire(wire) {
                    return netlist.clone();
                }
                enable.signal = Bit::Wire(output);
            }
            candidate.cells.push(Ecp5Cell::Lut4 {
                name: format!("replicate_enable_{driver_name}_{replica}"),
                inputs,
                output,
                init,
            });
            candidate.equivalence_proof.equivalent_logic_replications += 1;
        }
    }
    candidate
}

fn maximum_replicable_enable_fanout(netlist: &Ecp5Netlist, max_fanout: usize) -> usize {
    let fanouts = mapped_wire_fanouts(netlist);
    let lut_outputs = netlist
        .cells
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::Lut4 { output, .. } => Some(*output),
            Ecp5Cell::Ccu2c { .. }
            | Ecp5Cell::FlipFlop { .. }
            | Ecp5Cell::BlockRam { .. }
            | Ecp5Cell::TrellisIo { .. }
            | Ecp5Cell::Jtagg { .. }
            | Ecp5Cell::Pll { .. } => None,
        })
        .collect::<HashSet<_>>();
    netlist
        .cells
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::FlipFlop {
                enable: Some(enable),
                ..
            } => match enable.signal {
                Bit::Wire(wire) if lut_outputs.contains(&wire) => fanouts
                    .get(&wire)
                    .copied()
                    .filter(|fanout| *fanout <= max_fanout.saturating_mul(2)),
                Bit::Wire(_) | Bit::Zero | Bit::One => None,
            },
            Ecp5Cell::Lut4 { .. }
            | Ecp5Cell::Ccu2c { .. }
            | Ecp5Cell::FlipFlop { .. }
            | Ecp5Cell::BlockRam { .. }
            | Ecp5Cell::TrellisIo { .. }
            | Ecp5Cell::Jtagg { .. }
            | Ecp5Cell::Pll { .. } => None,
        })
        .max()
        .unwrap_or(0)
}

#[allow(clippy::too_many_lines)]
fn recluster_replicated_enable_sinks(
    netlist: &mut Ecp5Netlist,
    feedback: &PhysicalFeedback,
) -> usize {
    let timed_enable_drivers = feedback
        .net_timings()
        .iter()
        .filter(|timing| {
            timing
                .endpoints
                .iter()
                .any(|endpoint| endpoint.port == "CE" && endpoint.delay_ps > endpoint.budget_ps)
        })
        .map(|timing| timing.driver.as_str())
        .collect::<HashSet<_>>();
    let mut groups = BTreeMap::<String, Vec<(String, u32, [Bit; 4], u16)>>::new();
    for cell in &netlist.cells {
        let Ecp5Cell::Lut4 {
            name,
            inputs,
            output,
            init,
        } = cell
        else {
            continue;
        };
        let Some(replica) = name.strip_prefix("replicate_enable_") else {
            continue;
        };
        let Some((origin, replica_index)) = replica.rsplit_once('_') else {
            continue;
        };
        if replica_index.parse::<usize>().is_err() {
            continue;
        }
        groups
            .entry(origin.to_owned())
            .or_default()
            .push((name.clone(), *output, *inputs, *init));
    }

    let mut rewires = 0usize;
    for (origin, replicas) in groups {
        let Some(Ecp5Cell::Lut4 {
            inputs: origin_inputs,
            output: origin_output,
            init: origin_init,
            ..
        }) = netlist
            .cells
            .iter()
            .find(|cell| mapped_cell_name(cell) == origin)
        else {
            continue;
        };
        if replicas.len() != 1 {
            continue;
        }
        let (replica_name, replica_output, replica_inputs, replica_init) = &replicas[0];
        if replica_inputs != origin_inputs || replica_init != origin_init {
            continue;
        }
        if !timed_enable_drivers.contains(origin.as_str())
            && !timed_enable_drivers.contains(replica_name.as_str())
        {
            continue;
        }
        let outputs = [*origin_output, *replica_output];
        let mut sinks = netlist
            .cells
            .iter()
            .enumerate()
            .filter_map(|(index, cell)| {
                let Ecp5Cell::FlipFlop {
                    name,
                    enable: Some(enable),
                    ..
                } = cell
                else {
                    return None;
                };
                let Bit::Wire(wire) = enable.signal else {
                    return None;
                };
                outputs.contains(&wire).then(|| {
                    feedback
                        .location(name)
                        .map(|location| (index, name.clone(), wire, location))
                })?
            })
            .collect::<Vec<_>>();
        let total_sinks = netlist
            .cells
            .iter()
            .filter(|cell| {
                matches!(cell, Ecp5Cell::FlipFlop { enable: Some(enable), .. }
                    if matches!(enable.signal, Bit::Wire(wire) if outputs.contains(&wire)))
            })
            .count();
        if sinks.len() != total_sinks || sinks.len() < 2 {
            continue;
        }
        let (min_x, max_x, min_y, max_y) = sinks.iter().fold(
            (i32::MAX, i32::MIN, i32::MAX, i32::MIN),
            |(min_x, max_x, min_y, max_y), (_, _, _, location)| {
                (
                    min_x.min(location.x),
                    max_x.max(location.x),
                    min_y.min(location.y),
                    max_y.max(location.y),
                )
            },
        );
        let split_x = max_x - min_x >= max_y - min_y;
        sinks.sort_by_key(|(_, name, _, location)| {
            if split_x {
                (location.x, location.y, name.clone())
            } else {
                (location.y, location.x, name.clone())
            }
        });
        let split = sinks.len() / 2;
        let (first, second) = sinks.split_at(split);
        let assignments = match (feedback.location(&origin), feedback.location(replica_name)) {
            (Some(origin_location), Some(replica_location))
                if cluster_distance(first, origin_location)
                    + cluster_distance(second, replica_location)
                    > cluster_distance(first, replica_location)
                        + cluster_distance(second, origin_location) =>
            {
                [outputs[1], outputs[0]]
            }
            _ => outputs,
        };
        for (&output, cluster) in assignments.iter().zip([first, second]) {
            for &(index, _, previous, _) in cluster {
                if previous == output {
                    continue;
                }
                let Ecp5Cell::FlipFlop {
                    enable: Some(enable),
                    ..
                } = &mut netlist.cells[index]
                else {
                    unreachable!("sink indices were collected from enabled flip-flops")
                };
                enable.signal = Bit::Wire(output);
                rewires += 1;
            }
        }
    }
    rewires
}

fn replicate_physically_critical_luts(
    netlist: &mut Ecp5Netlist,
    feedback: &PhysicalFeedback,
) -> (usize, usize) {
    const MAX_PHYSICAL_REPLICAS: usize = 16;
    const MAX_REPLICATED_NET_FANOUT: usize = 16;

    let mut next_wire = maximum_mapped_wire(netlist)
        .and_then(|wire| wire.checked_add(1))
        .unwrap_or(1);
    let mut replicas = 0usize;
    let mut rewires = 0usize;
    for timing in feedback.net_timings() {
        if replicas >= MAX_PHYSICAL_REPLICAS {
            break;
        }
        let violating_sinks = timing
            .endpoints
            .iter()
            .filter(|endpoint| {
                endpoint.delay_ps > endpoint.budget_ps
                    && !matches!(endpoint.port.as_str(), "CLK" | "LSR" | "CE")
            })
            .map(|endpoint| endpoint.cell.as_str())
            .collect::<HashSet<_>>();
        let eligible_sinks = timing
            .endpoints
            .iter()
            .filter(|endpoint| !matches!(endpoint.port.as_str(), "CLK" | "LSR" | "CE"))
            .map(|endpoint| endpoint.cell.as_str())
            .collect::<HashSet<_>>();
        if violating_sinks.is_empty()
            || violating_sinks.len() >= eligible_sinks.len()
            || eligible_sinks.len() > MAX_REPLICATED_NET_FANOUT
        {
            continue;
        }
        let Some((inputs, output, init)) = netlist.cells.iter().find_map(|cell| match cell {
            Ecp5Cell::Lut4 {
                name,
                inputs,
                output,
                init,
            } if name == &timing.driver => Some((*inputs, *output, *init)),
            _ => None,
        }) else {
            continue;
        };
        let clone_output = next_wire;
        let Some(allocated) = next_wire.checked_add(1) else {
            break;
        };
        next_wire = allocated;
        let mut clone_rewires = 0usize;
        for cell in &mut netlist.cells {
            if violating_sinks.contains(mapped_cell_name(cell)) {
                clone_rewires += replace_wire_in_cell_inputs(cell, output, clone_output);
            }
        }
        if clone_rewires == 0 || mapped_wire_fanout(netlist, output) == 0 {
            for cell in &mut netlist.cells {
                replace_wire_in_cell_inputs(cell, clone_output, output);
            }
            continue;
        }
        netlist.cells.push(Ecp5Cell::Lut4 {
            name: format!("physical_replicate_{}_{replicas}", timing.driver),
            inputs,
            output: clone_output,
            init,
        });
        replicas += 1;
        rewires += clone_rewires;
    }
    (replicas, rewires)
}

fn replace_wire_in_cell_inputs(cell: &mut Ecp5Cell, from: u32, to: u32) -> usize {
    let mut replacements = 0usize;
    let mut replace = |bit: &mut Bit| {
        if *bit == Bit::Wire(from) {
            *bit = Bit::Wire(to);
            replacements += 1;
        }
    };
    match cell {
        Ecp5Cell::Lut4 { inputs, .. } => {
            for input in inputs {
                replace(input);
            }
        }
        Ecp5Cell::Ccu2c {
            inputs, carry_in, ..
        } => {
            for input in inputs.iter_mut().flatten() {
                replace(input);
            }
            replace(carry_in);
        }
        Ecp5Cell::FlipFlop {
            data,
            clock,
            enable,
            reset,
            ..
        } => {
            replace(data);
            replace(clock);
            if let Some(enable) = enable {
                replace(&mut enable.signal);
            }
            if let Some(reset) = reset {
                replace(&mut reset.signal);
            }
        }
        Ecp5Cell::BlockRam {
            write_address,
            write_data,
            write_enable,
            read_address,
            read_enable,
            clock,
            ..
        } => {
            for bit in write_address
                .iter_mut()
                .chain(write_data)
                .chain(read_address.iter_mut())
            {
                replace(bit);
            }
            replace(&mut write_enable.signal);
            if let Some(read_enable) = read_enable {
                replace(&mut read_enable.signal);
            }
            replace(clock);
        }
        Ecp5Cell::TrellisIo {
            fabric_output,
            tristate,
            ..
        } => {
            replace(fabric_output);
            replace(tristate);
        }
        Ecp5Cell::Jtagg { tdo, .. } => {
            for bit in tdo {
                replace(bit);
            }
        }
        Ecp5Cell::Pll {
            reference_clock, ..
        } => replace(reference_clock),
    }
    replacements
}

fn cluster_distance(
    cluster: &[(usize, String, u32, PhysicalLocation)],
    driver: PhysicalLocation,
) -> i64 {
    cluster
        .iter()
        .map(|(_, _, _, sink)| {
            i64::from((sink.x - driver.x).abs()) + i64::from((sink.y - driver.y).abs())
        })
        .sum()
}

fn register_data_is_driven_by_ccu(netlist: &Ecp5Netlist, register_index: usize) -> bool {
    let Some(Ecp5Cell::FlipFlop { data, .. }) = netlist.cells.get(register_index) else {
        return false;
    };
    netlist
        .cells
        .iter()
        .any(|cell| matches!(cell, Ecp5Cell::Ccu2c { .. }) && cell_output_bits(cell).contains(data))
}

struct MappedLutProfile {
    data_depth: usize,
    critical_depth: Vec<usize>,
    overall_depth: usize,
    data_period_ps: u32,
    critical_timing: Vec<usize>,
    timing_endpoints: Vec<(usize, u32)>,
    overall_period_ps: u32,
}

#[allow(clippy::too_many_lines)]
fn mapped_lut_profile(netlist: &Ecp5Netlist) -> MappedLutProfile {
    let depths = mapped_lut_depths(netlist);
    let data_endpoints = netlist
        .cells
        .iter()
        .enumerate()
        .filter_map(|(index, cell)| match cell {
            Ecp5Cell::FlipFlop { data, .. } => Some((index, bit_lut_depth(*data, &depths))),
            _ => None,
        })
        .collect::<Vec<_>>();
    let data_depth = data_endpoints
        .iter()
        .map(|(_, depth)| *depth)
        .max()
        .unwrap_or(0);
    let control_depths = netlist.cells.iter().filter_map(|cell| match cell {
        Ecp5Cell::FlipFlop { enable, .. } => {
            enable.map(|control| bit_lut_depth(control.signal, &depths))
        }
        Ecp5Cell::BlockRam {
            write_enable,
            read_enable,
            ..
        } => Some(
            read_enable
                .map_or(0, |control| bit_lut_depth(control.signal, &depths))
                .max(bit_lut_depth(write_enable.signal, &depths)),
        ),
        Ecp5Cell::Lut4 { .. }
        | Ecp5Cell::Ccu2c { .. }
        | Ecp5Cell::TrellisIo { .. }
        | Ecp5Cell::Jtagg { .. }
        | Ecp5Cell::Pll { .. } => None,
    });
    let output_depths = netlist
        .io_timing
        .output_delays_ps
        .iter()
        .map(|(bit, _)| bit_lut_depth(*bit, &depths));
    let overall_depth = [data_depth]
        .into_iter()
        .chain(control_depths)
        .chain(output_depths)
        .max()
        .unwrap_or(0);
    let critical_depth = data_endpoints
        .into_iter()
        .filter_map(|(index, depth)| (depth == data_depth && depth > 0).then_some(index))
        .collect();
    let arrivals = mapped_timing_arrivals(netlist);
    let fanouts = mapped_wire_fanouts(netlist);
    let timing_endpoints = netlist
        .cells
        .iter()
        .enumerate()
        .filter_map(|(index, cell)| match cell {
            Ecp5Cell::FlipFlop { data, .. } => {
                mapped_setup_period(*data, &arrivals, &fanouts).map(|period| (index, period))
            }
            _ => None,
        })
        .collect::<Vec<_>>();
    let data_period_ps = timing_endpoints
        .iter()
        .map(|(_, period)| *period)
        .max()
        .unwrap_or(0);
    let critical_timing = timing_endpoints
        .iter()
        .filter_map(|(index, period)| (*period == data_period_ps && *period > 0).then_some(*index))
        .collect();
    let control_periods = netlist.cells.iter().filter_map(|cell| match cell {
        Ecp5Cell::FlipFlop { enable, .. } => {
            enable.and_then(|control| mapped_setup_period(control.signal, &arrivals, &fanouts))
        }
        Ecp5Cell::BlockRam {
            write_enable,
            read_enable,
            ..
        } => read_enable
            .and_then(|control| mapped_setup_period(control.signal, &arrivals, &fanouts))
            .into_iter()
            .chain(mapped_setup_period(
                write_enable.signal,
                &arrivals,
                &fanouts,
            ))
            .max(),
        Ecp5Cell::Lut4 { .. }
        | Ecp5Cell::Ccu2c { .. }
        | Ecp5Cell::TrellisIo { .. }
        | Ecp5Cell::Jtagg { .. }
        | Ecp5Cell::Pll { .. } => None,
    });
    let output_periods = netlist
        .io_timing
        .output_delays_ps
        .iter()
        .filter_map(|(bit, delay_ps)| {
            mapped_output_period(*bit, &arrivals, &fanouts)
                .map(|period| period.saturating_add(*delay_ps))
        });
    let overall_period_ps = [data_period_ps]
        .into_iter()
        .chain(control_periods)
        .chain(output_periods)
        .max()
        .unwrap_or(0);
    MappedLutProfile {
        data_depth,
        critical_depth,
        overall_depth,
        data_period_ps,
        critical_timing,
        timing_endpoints,
        overall_period_ps,
    }
}

fn mapped_lut_depths(netlist: &Ecp5Netlist) -> WireMap<usize> {
    let mut depths = WireMap::default();
    depths.reserve(netlist.cells.len());
    depths.extend(
        netlist
            .io_timing
            .input_arrivals_ps
            .keys()
            .map(|wire| (*wire, 0)),
    );
    for cell in &netlist.cells {
        match cell {
            Ecp5Cell::FlipFlop { output, .. } => {
                depths.insert(*output, 0);
            }
            Ecp5Cell::BlockRam { read_data, .. } => {
                for output in read_data {
                    depths.insert(*output, 0);
                }
            }
            Ecp5Cell::Lut4 { .. }
            | Ecp5Cell::Ccu2c { .. }
            | Ecp5Cell::TrellisIo { .. }
            | Ecp5Cell::Jtagg { .. }
            | Ecp5Cell::Pll { .. } => {}
        }
    }
    // Retimed cells need not remain in producer-before-consumer order. Iterate
    // to a fixed point so the score is independent of JSON cell ordering.
    for _ in 0..netlist.cells.len() {
        let mut progress = false;
        for cell in &netlist.cells {
            match cell {
                Ecp5Cell::Lut4 { inputs, output, .. } => {
                    let depth = inputs
                        .iter()
                        .filter_map(|input| match input {
                            Bit::Wire(wire) => depths.get(wire).copied(),
                            Bit::Zero | Bit::One => None,
                        })
                        .max()
                        .map(|depth| depth + 1);
                    if let Some(depth) = depth
                        && depths.insert(*output, depth) != Some(depth)
                    {
                        progress = true;
                    }
                }
                Ecp5Cell::Ccu2c {
                    inputs,
                    carry_in,
                    sums,
                    carry_out,
                    ..
                } => {
                    let depth = inputs
                        .iter()
                        .flatten()
                        .chain([carry_in])
                        .filter_map(|input| match input {
                            Bit::Wire(wire) => depths.get(wire).copied(),
                            Bit::Zero | Bit::One => None,
                        })
                        .max();
                    if let Some(depth) = depth {
                        for output in sums.iter().chain([carry_out]) {
                            if depths.insert(*output, depth) != Some(depth) {
                                progress = true;
                            }
                        }
                    }
                }
                Ecp5Cell::FlipFlop { .. }
                | Ecp5Cell::BlockRam { .. }
                | Ecp5Cell::TrellisIo { .. }
                | Ecp5Cell::Jtagg { .. }
                | Ecp5Cell::Pll { .. } => {}
            }
        }
        if !progress {
            break;
        }
    }
    depths
}

fn bit_lut_depth(bit: Bit, depths: &WireMap<usize>) -> usize {
    match bit {
        Bit::Wire(wire) => depths.get(&wire).copied().unwrap_or(0),
        Bit::Zero | Bit::One => 0,
    }
}

#[allow(clippy::too_many_lines)]
fn mapped_timing_arrivals(netlist: &Ecp5Netlist) -> WireMap<u32> {
    let fanouts = mapped_wire_fanouts(netlist);
    let carry_outputs = netlist
        .cells
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::Ccu2c { carry_out, .. } => Some(*carry_out),
            _ => None,
        })
        .collect::<HashSet<_>>();
    let mut arrivals = WireMap::default();
    arrivals.reserve(netlist.cells.len());
    arrivals.extend(
        netlist
            .io_timing
            .input_arrivals_ps
            .iter()
            .map(|(wire, delay_ps)| (*wire, *delay_ps)),
    );
    for cell in &netlist.cells {
        match cell {
            Ecp5Cell::FlipFlop { output, .. } => {
                arrivals.insert(*output, FLIP_FLOP_CLOCK_TO_OUTPUT_PS);
            }
            Ecp5Cell::BlockRam { read_data, .. } => {
                for output in read_data {
                    arrivals.insert(*output, BRAM_CLOCK_TO_OUTPUT_PS);
                }
            }
            Ecp5Cell::Lut4 { .. }
            | Ecp5Cell::Ccu2c { .. }
            | Ecp5Cell::TrellisIo { .. }
            | Ecp5Cell::Jtagg { .. }
            | Ecp5Cell::Pll { .. } => {}
        }
    }
    for _ in 0..netlist.cells.len() {
        let mut progress = false;
        for cell in &netlist.cells {
            match cell {
                Ecp5Cell::Lut4 { inputs, output, .. } => {
                    let arrival = inputs
                        .iter()
                        .filter_map(|input| mapped_routed_arrival(*input, &arrivals, &fanouts))
                        .max()
                        .map(|arrival| arrival.saturating_add(LUT_DELAY_PS));
                    if let Some(arrival) = arrival {
                        progress |= arrivals.insert(*output, arrival) != Some(arrival);
                    }
                }
                Ecp5Cell::Ccu2c {
                    inputs,
                    carry_in,
                    sums,
                    carry_out,
                    ..
                } => {
                    let first_inputs = mapped_ccu_inputs(inputs[0], &arrivals, &fanouts);
                    let second_inputs = mapped_ccu_inputs(inputs[1], &arrivals, &fanouts);
                    let carry = match carry_in {
                        Bit::Zero | Bit::One => None,
                        Bit::Wire(wire) if carry_outputs.contains(wire) => arrivals
                            .get(wire)
                            .map(|arrival| arrival.saturating_add(CCU_CARRY_PS)),
                        bit @ Bit::Wire(_) => mapped_routed_arrival(*bit, &arrivals, &fanouts)
                            .map(|arrival| arrival.saturating_add(CCU_INPUT_PS)),
                    };
                    let first = first_inputs.into_iter().chain(carry).max();
                    let internal_carry = first.map(|arrival| arrival.saturating_add(CCU_CARRY_PS));
                    if let Some(first) = first {
                        let sum0 = first.saturating_add(CCU_SUM_PS);
                        progress |= arrivals.insert(sums[0], sum0) != Some(sum0);
                    }
                    let second = second_inputs.into_iter().chain(internal_carry).max();
                    if let Some(second) = second {
                        let sum1 = second.saturating_add(CCU_SUM_PS);
                        let carry_out_arrival = second.saturating_add(CCU_CARRY_PS);
                        progress |= arrivals.insert(sums[1], sum1) != Some(sum1);
                        progress |= arrivals.insert(*carry_out, carry_out_arrival)
                            != Some(carry_out_arrival);
                    }
                }
                Ecp5Cell::FlipFlop { .. }
                | Ecp5Cell::BlockRam { .. }
                | Ecp5Cell::TrellisIo { .. }
                | Ecp5Cell::Jtagg { .. }
                | Ecp5Cell::Pll { .. } => {}
            }
        }
        if !progress {
            break;
        }
    }
    arrivals
}

fn mapped_ccu_inputs(
    inputs: [Bit; 4],
    arrivals: &WireMap<u32>,
    fanouts: &WireMap<usize>,
) -> Option<u32> {
    inputs
        .into_iter()
        .filter_map(|input| mapped_routed_arrival(input, arrivals, fanouts))
        .max()
        .map(|arrival| arrival.saturating_add(CCU_INPUT_PS))
}

fn mapped_routed_arrival(
    bit: Bit,
    arrivals: &WireMap<u32>,
    fanouts: &WireMap<usize>,
) -> Option<u32> {
    match bit {
        Bit::Zero | Bit::One => None,
        Bit::Wire(wire) => arrivals.get(&wire).map(|arrival| {
            arrival
                .saturating_add(wire_delay_ps(fanouts.get(&wire).copied().unwrap_or(1)))
                .saturating_add(MAPPED_ROUTE_GUARD_PS)
        }),
    }
}

fn mapped_setup_period(bit: Bit, arrivals: &WireMap<u32>, fanouts: &WireMap<usize>) -> Option<u32> {
    mapped_routed_arrival(bit, arrivals, fanouts)
        .map(|arrival| arrival.saturating_add(FLIP_FLOP_SETUP_PS))
}

fn mapped_output_period(
    bit: Bit,
    arrivals: &WireMap<u32>,
    fanouts: &WireMap<usize>,
) -> Option<u32> {
    mapped_routed_arrival(bit, arrivals, fanouts)
}

/// Structural wiring burden: the sum of per-net fanout-weighted route
/// estimates over every driver in the netlist. Growth of this value tracks
/// the placement pressure that added primitives create, which flat per-hop
/// delays cannot see.
fn routing_burden(netlist: &Ecp5Netlist) -> u64 {
    let fanouts = mapped_wire_fanouts(netlist);
    fanouts
        .iter()
        .map(|(&wire, &fanout)| {
            let _ = wire;
            u64::from(wire_delay_ps(fanout.max(1))) * u64::try_from(fanout).unwrap_or(1)
        })
        .sum()
}

fn mapped_wire_fanouts(netlist: &Ecp5Netlist) -> WireMap<usize> {
    let mut fanouts = WireMap::default();
    fanouts.reserve(netlist.cells.len());
    for cell in &netlist.cells {
        for_each_cell_input_bit(cell, |bit| {
            if let Bit::Wire(wire) = bit {
                *fanouts.entry(wire).or_insert(0) += 1;
            }
        });
    }
    for bit in netlist
        .ports
        .iter()
        .filter(|port| port.direction == PortDirection::Output)
        .flat_map(|port| &port.bits)
    {
        if let Bit::Wire(wire) = bit {
            *fanouts.entry(*wire).or_insert(0) += 1;
        }
    }
    fanouts
}

fn backward_retime_primitive(netlist: &Ecp5Netlist, register_index: usize) -> Option<Ecp5Netlist> {
    backward_retime_lut(netlist, register_index)
        .or_else(|| backward_retime_ccu2c(netlist, register_index))
}

#[allow(clippy::too_many_lines)]
fn backward_retime_lut(netlist: &Ecp5Netlist, register_index: usize) -> Option<Ecp5Netlist> {
    let Ecp5Cell::FlipFlop {
        name: register_name,
        data: Bit::Wire(data_wire),
        output: register_output,
        clock,
        edge: clock_edge,
        enable,
        reset: Some(reset),
    } = netlist.cells.get(register_index)?
    else {
        return None;
    };
    if mapped_wire_is_clock_or_reset(netlist, *register_output) {
        return None;
    }
    let (lut_index, lut_name, lut_inputs, lut_init) =
        netlist
            .cells
            .iter()
            .enumerate()
            .find_map(|(index, cell)| match cell {
                Ecp5Cell::Lut4 {
                    name,
                    inputs,
                    output,
                    init,
                } if output == data_wire => Some((index, name.clone(), *inputs, *init)),
                _ => None,
            })?;
    let mut input_wires = lut_inputs
        .iter()
        .filter_map(|input| match input {
            Bit::Wire(wire) => Some(*wire),
            Bit::Zero | Bit::One => None,
        })
        .collect::<Vec<_>>();
    input_wires.sort_unstable();
    input_wires.dedup();
    if input_wires.is_empty() || input_wires.len() > 4 {
        return None;
    }
    let function = reduced_lut_function(lut_inputs, lut_init, &input_wires);
    let input_count = input_wires.len();
    let mut vertices = input_wires
        .iter()
        .map(|wire| RetimingVertex::boundary(format!("wire{wire}"), LogicFunction::new(0, 0)))
        .collect::<Vec<_>>();
    vertices.push(RetimingVertex::logic("lut", function));
    vertices.push(RetimingVertex::boundary("q", LogicFunction::new(1, 0b10)));
    let lut_vertex = input_count;
    let q_vertex = input_count + 1;
    let mut edges = (0..input_count)
        .map(|input| RetimingEdge::new(input, lut_vertex, Vec::new()))
        .collect::<Vec<_>>();
    edges.push(RetimingEdge::new(lut_vertex, q_vertex, vec![reset.value]));
    let before = RetimingGraph::new(
        RetimingDomain::new(
            format!("{clock:?}"),
            *clock_edge,
            format!("{:?}", reset.signal),
            reset.active,
            reset.asynchronous,
        ),
        vertices,
        edges,
    );
    let mut labels = vec![0; input_count + 2];
    labels[lut_vertex] = 1;
    let certificate = RetimingCertificate::new(labels);
    let after = derive_retimed_graph(&before, &certificate).ok()?;
    verify_retiming_certificate(&before, &after, &certificate).ok()?;

    // Allocate above the original maximum before removing the sink FF. If its
    // Q is the maximum wire, measuring afterwards would reuse Q for a new FF
    // and create two drivers when the retimed LUT takes over that same Q.
    let mut next_wire = maximum_mapped_wire(netlist)?.checked_add(1)?;
    let mut candidate = netlist.clone();
    candidate.equivalence_proof.certified_primitive_moves += 1;
    candidate.cells.remove(register_index);
    let mut replacements = HashMap::new();
    for (input, reset_edge) in input_wires.iter().zip(after.edges()) {
        let output = next_wire;
        next_wire = next_wire.checked_add(1)?;
        replacements.insert(*input, output);
        candidate.cells.push(Ecp5Cell::FlipFlop {
            name: format!("retime_{register_name}_{input}"),
            data: Bit::Wire(*input),
            output,
            clock: *clock,
            edge: *clock_edge,
            enable: *enable,
            reset: Some(Reset {
                value: reset_edge.reset_values()[0],
                ..*reset
            }),
        });
    }
    let new_inputs = lut_inputs.map(|input| match input {
        Bit::Wire(wire) => Bit::Wire(replacements[&wire]),
        constant => constant,
    });
    let fanout = mapped_wire_fanout(netlist, *data_wire);
    // The retimed LUT takes over the register output. When the original LUT
    // keeps other sinks it stays in the netlist, and either branch can collide
    // with a replica name created by an earlier move. Keep every pushed or
    // renamed cell unique among all cells except the one being replaced.
    let replacement_index = if fanout == 1 {
        Some(lut_index - usize::from(register_index < lut_index))
    } else {
        None
    };
    let retimed_lut = Ecp5Cell::Lut4 {
        name: unique_cell_name(
            &format!("retime_{lut_name}"),
            &candidate.cells,
            replacement_index,
        ),
        inputs: new_inputs,
        output: *register_output,
        init: lut_init,
    };
    if let Some(replacement_index) = replacement_index {
        candidate.cells[replacement_index] = retimed_lut;
    } else {
        candidate.cells.push(retimed_lut);
    }
    prune_unobservable_retiming_cells(&mut candidate);
    Some(candidate)
}

#[allow(clippy::too_many_lines)]
fn forward_retime_lut(netlist: &Ecp5Netlist, lut_index: usize) -> Option<Ecp5Netlist> {
    let Ecp5Cell::Lut4 {
        name: lut_name,
        inputs: lut_inputs,
        output: lut_output,
        init: lut_init,
    } = netlist.cells.get(lut_index)?
    else {
        return None;
    };
    let physical_uses = lut_inputs
        .iter()
        .filter_map(|input| match input {
            Bit::Wire(wire) => Some(*wire),
            Bit::Zero | Bit::One => None,
        })
        .fold(HashMap::<u32, usize>::new(), |mut uses, wire| {
            *uses.entry(wire).or_insert(0) += 1;
            uses
        });
    if physical_uses.is_empty() {
        return None;
    }
    let mut input_data = HashMap::new();
    let mut input_resets = HashMap::new();
    let mut input_registers = HashSet::new();
    let mut domain: Option<(Bit, ClockEdge, Option<Control>, Reset)> = None;
    for (&wire, &local_uses) in &physical_uses {
        if mapped_wire_fanout(netlist, wire) != local_uses {
            return None;
        }
        let (index, data, clock, edge, enable, reset) =
            netlist
                .cells
                .iter()
                .enumerate()
                .find_map(|(index, cell)| match cell {
                    Ecp5Cell::FlipFlop {
                        data,
                        output,
                        clock,
                        edge,
                        enable,
                        reset: Some(reset),
                        ..
                    } if *output == wire => Some((index, *data, *clock, *edge, *enable, *reset)),
                    _ => None,
                })?;
        if let Some((domain_clock, domain_edge, domain_enable, domain_reset)) = domain {
            if (clock, edge, enable) != (domain_clock, domain_edge, domain_enable)
                || (reset.signal, reset.active, reset.asynchronous)
                    != (
                        domain_reset.signal,
                        domain_reset.active,
                        domain_reset.asynchronous,
                    )
            {
                return None;
            }
        } else {
            domain = Some((clock, edge, enable, reset));
        }
        input_data.insert(wire, data);
        input_resets.insert(wire, reset.value);
        input_registers.insert(index);
    }
    let (clock, clock_edge, enable, reset) = domain?;
    let mut input_wires = physical_uses.keys().copied().collect::<Vec<_>>();
    input_wires.sort_unstable();
    let function = reduced_lut_function(*lut_inputs, *lut_init, &input_wires);
    let input_count = input_wires.len();
    let mut vertices = input_wires
        .iter()
        .map(|wire| RetimingVertex::boundary(format!("wire{wire}"), LogicFunction::new(0, 0)))
        .collect::<Vec<_>>();
    vertices.push(RetimingVertex::logic("lut", function));
    vertices.push(RetimingVertex::boundary("q", LogicFunction::new(1, 0b10)));
    let lut_vertex = input_count;
    let q_vertex = input_count + 1;
    let mut edges = input_wires
        .iter()
        .enumerate()
        .map(|(input, wire)| RetimingEdge::new(input, lut_vertex, vec![input_resets[wire]]))
        .collect::<Vec<_>>();
    edges.push(RetimingEdge::new(lut_vertex, q_vertex, Vec::new()));
    let before = RetimingGraph::new(
        RetimingDomain::new(
            format!("{clock:?}"),
            clock_edge,
            format!("{:?}", reset.signal),
            reset.active,
            reset.asynchronous,
        ),
        vertices,
        edges,
    );
    let mut labels = vec![0; input_count + 2];
    labels[lut_vertex] = -1;
    let certificate = RetimingCertificate::new(labels);
    let after = derive_retimed_graph(&before, &certificate).ok()?;
    verify_retiming_certificate(&before, &after, &certificate).ok()?;
    let output_reset = after
        .edges()
        .iter()
        .find(|edge| edge.source() == lut_vertex && edge.target() == q_vertex)?
        .reset_values()
        .first()
        .copied()?;

    let new_lut_output = maximum_mapped_wire(netlist)?.checked_add(1)?;
    let mut candidate = netlist.clone();
    candidate.equivalence_proof.certified_primitive_moves += 1;
    let mut removed = input_registers.into_iter().collect::<Vec<_>>();
    removed.sort_unstable_by(|lhs, rhs| rhs.cmp(lhs));
    for index in removed {
        candidate.cells.remove(index);
    }
    let lut_index = candidate
        .cells
        .iter()
        .position(|cell| mapped_cell_name(cell) == lut_name)?;
    candidate.cells[lut_index] = Ecp5Cell::Lut4 {
        name: format!("forward_{lut_name}"),
        inputs: lut_inputs.map(|input| match input {
            Bit::Wire(wire) => input_data[&wire],
            constant => constant,
        }),
        output: new_lut_output,
        init: *lut_init,
    };
    candidate.cells.push(Ecp5Cell::FlipFlop {
        name: format!("forward_ff_{lut_name}"),
        data: Bit::Wire(new_lut_output),
        output: *lut_output,
        clock,
        edge: clock_edge,
        enable,
        reset: Some(Reset {
            value: output_reset,
            ..reset
        }),
    });
    prune_unobservable_retiming_cells(&mut candidate);
    Some(candidate)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CcuRetimeOutput {
    Sum0,
    Sum1,
    Carry,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CcuInputLocation {
    Pin { slice: usize, pin: usize },
    CarryIn,
}

#[derive(Clone, Copy)]
enum CcuCarrySource {
    External(Bit, CcuInputLocation),
    Internal(usize),
}

#[derive(Clone, Copy)]
enum CcuLogicValue {
    Constant(bool),
    Input(usize),
}

struct CcuBoundary {
    vertex: usize,
    bit: Bit,
    location: CcuInputLocation,
}

fn forward_retime_registered_ccu_chains(netlist: &Ecp5Netlist, timing_driven: bool) -> Ecp5Netlist {
    let baseline_burden = routing_burden(netlist);
    let adjusted_overall = |profile: &MappedLutProfile, cells_now: &Ecp5Netlist| {
        congestion_adjusted_overall(
            profile.overall_period_ps,
            routing_burden(cells_now),
            baseline_burden,
        )
    };
    let mut selected = netlist.clone();
    for _ in 0..netlist.cells.len() {
        let profile = mapped_lut_profile(&selected);
        let selected_score = retiming_score(
            &profile,
            timing_driven,
            selected.cells.len(),
            mapped_register_count(&selected),
            adjusted_overall(&profile, &selected),
        );
        let chains = ccu_chain_names(&selected);
        let mut best = None;
        for chain in chains {
            let mut candidate = selected.clone();
            let mut moved = 0usize;
            for name in &chain {
                let Some(index) = candidate
                    .cells
                    .iter()
                    .position(|cell| mapped_cell_name(cell) == name)
                else {
                    break;
                };
                let Some(retimed) = forward_retime_ccu2c(&candidate, index) else {
                    break;
                };
                candidate = retimed;
                moved += 1;
            }
            if moved != chain.len() {
                continue;
            }
            let candidate_profile = mapped_lut_profile(&candidate);
            let score = retiming_score(
                &candidate_profile,
                timing_driven,
                candidate.cells.len(),
                mapped_register_count(&candidate),
                adjusted_overall(&candidate_profile, &candidate),
            );
            if score < selected_score
                && best
                    .as_ref()
                    .is_none_or(|(_, best_score): &(Ecp5Netlist, _)| score < *best_score)
            {
                best = Some((candidate, score));
            }
        }
        let Some((candidate, _)) = best else {
            break;
        };
        selected = candidate;
    }
    selected
}

fn ccu_chain_names(netlist: &Ecp5Netlist) -> Vec<Vec<String>> {
    let ccus = netlist
        .cells
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::Ccu2c {
                name,
                carry_in,
                carry_out,
                ..
            } => Some((name.clone(), *carry_in, *carry_out)),
            Ecp5Cell::Lut4 { .. }
            | Ecp5Cell::FlipFlop { .. }
            | Ecp5Cell::BlockRam { .. }
            | Ecp5Cell::TrellisIo { .. }
            | Ecp5Cell::Jtagg { .. }
            | Ecp5Cell::Pll { .. } => None,
        })
        .collect::<Vec<_>>();
    let carry_producers = ccus
        .iter()
        .map(|(name, _, carry_out)| (*carry_out, name.as_str()))
        .collect::<HashMap<_, _>>();
    let successors = ccus
        .iter()
        .filter_map(|(name, carry_in, _)| match carry_in {
            Bit::Wire(wire) => carry_producers
                .get(wire)
                .map(|producer| ((*producer).to_owned(), name.clone())),
            Bit::Zero | Bit::One => None,
        })
        .collect::<HashMap<_, _>>();
    let roots = ccus
        .iter()
        .filter_map(|(name, carry_in, _)| {
            let has_producer = match carry_in {
                Bit::Wire(wire) => carry_producers.contains_key(wire),
                Bit::Zero | Bit::One => false,
            };
            (!has_producer).then_some(name.clone())
        })
        .collect::<Vec<_>>();
    let mut chains = Vec::new();
    let mut visited = HashSet::new();
    for root in roots {
        let mut chain = Vec::new();
        let mut cursor = Some(root);
        while let Some(name) = cursor {
            if !visited.insert(name.clone()) {
                break;
            }
            cursor = successors.get(&name).cloned();
            chain.push(name);
        }
        if !chain.is_empty() {
            chains.push(chain);
        }
    }
    for (name, _, _) in ccus {
        if visited.insert(name.clone()) {
            chains.push(vec![name]);
        }
    }
    chains
}

#[allow(clippy::too_many_lines)]
fn forward_retime_ccu2c(netlist: &Ecp5Netlist, ccu_index: usize) -> Option<Ecp5Netlist> {
    let Ecp5Cell::Ccu2c {
        name: ccu_name,
        inputs,
        carry_in,
        sums,
        carry_out,
        init,
        inject,
    } = netlist.cells.get(ccu_index)?
    else {
        return None;
    };
    let ccu_name = ccu_name.clone();
    let (inputs, carry_in, sums, carry_out, init, inject) =
        (*inputs, *carry_in, *sums, *carry_out, *init, *inject);

    let mut physical_uses = inputs
        .iter()
        .flatten()
        .copied()
        .chain([carry_in])
        .filter_map(|bit| match bit {
            Bit::Wire(wire) => Some(wire),
            Bit::Zero | Bit::One => None,
        })
        .fold(HashMap::<u32, usize>::new(), |mut uses, wire| {
            *uses.entry(wire).or_insert(0) += 1;
            uses
        });
    if inject[0]
        && let Bit::Wire(wire) = carry_in
        && let Some(uses) = physical_uses.get_mut(&wire)
    {
        *uses -= 1;
        if *uses == 0 {
            physical_uses.remove(&wire);
        }
    }
    if physical_uses.is_empty() {
        return None;
    }

    let mut input_data = HashMap::new();
    let mut input_resets = HashMap::new();
    let mut input_registers = HashSet::new();
    let mut domain: Option<(Bit, ClockEdge, Option<Control>, Reset)> = None;
    for (&wire, &local_uses) in &physical_uses {
        if mapped_wire_fanout(netlist, wire) != local_uses {
            return None;
        }
        let (register_index, data, clock, edge, enable, reset) =
            netlist
                .cells
                .iter()
                .enumerate()
                .find_map(|(index, cell)| match cell {
                    Ecp5Cell::FlipFlop {
                        data,
                        output,
                        clock,
                        edge,
                        enable,
                        reset: Some(reset),
                        ..
                    } if *output == wire => Some((index, *data, *clock, *edge, *enable, *reset)),
                    Ecp5Cell::Lut4 { .. }
                    | Ecp5Cell::Ccu2c { .. }
                    | Ecp5Cell::FlipFlop { .. }
                    | Ecp5Cell::BlockRam { .. }
                    | Ecp5Cell::TrellisIo { .. }
                    | Ecp5Cell::Jtagg { .. }
                    | Ecp5Cell::Pll { .. } => None,
                })?;
        if let Some((domain_clock, domain_edge, domain_enable, domain_reset)) = domain {
            if (clock, edge, enable) != (domain_clock, domain_edge, domain_enable)
                || (reset.signal, reset.active, reset.asynchronous)
                    != (
                        domain_reset.signal,
                        domain_reset.active,
                        domain_reset.asynchronous,
                    )
            {
                return None;
            }
        } else {
            domain = Some((clock, edge, enable, reset));
        }
        input_data.insert(wire, data);
        input_resets.insert(wire, reset.value);
        input_registers.insert(register_index);
    }
    let (clock, clock_edge, enable, reset) = domain?;

    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let mut boundaries = Vec::new();
    let mut logic_vertices = Vec::new();
    let sum0_vertex = append_ccu_logic_vertex(
        &mut vertices,
        &mut edges,
        &mut boundaries,
        inputs[0],
        CcuCarrySource::External(carry_in, CcuInputLocation::CarryIn),
        init[0],
        inject[0],
        false,
        0,
        &mut logic_vertices,
        Some(&input_resets),
    );
    let carry0_vertex = append_ccu_logic_vertex(
        &mut vertices,
        &mut edges,
        &mut boundaries,
        inputs[0],
        CcuCarrySource::External(carry_in, CcuInputLocation::CarryIn),
        init[0],
        inject[0],
        true,
        0,
        &mut logic_vertices,
        Some(&input_resets),
    );
    let slice1_carry = if inject[1] {
        CcuCarrySource::External(Bit::Zero, CcuInputLocation::CarryIn)
    } else {
        CcuCarrySource::Internal(carry0_vertex)
    };
    let sum1_vertex = append_ccu_logic_vertex(
        &mut vertices,
        &mut edges,
        &mut boundaries,
        inputs[1],
        slice1_carry,
        init[1],
        inject[1],
        false,
        1,
        &mut logic_vertices,
        Some(&input_resets),
    );
    let carry1_vertex = append_ccu_logic_vertex(
        &mut vertices,
        &mut edges,
        &mut boundaries,
        inputs[1],
        slice1_carry,
        init[1],
        inject[1],
        true,
        1,
        &mut logic_vertices,
        Some(&input_resets),
    );
    let mut output_boundaries = Vec::new();
    for (name, logic) in [
        ("sum0", sum0_vertex),
        ("sum1", sum1_vertex),
        ("carry", carry1_vertex),
    ] {
        let boundary = vertices.len();
        vertices.push(RetimingVertex::boundary(name, LogicFunction::new(1, 0b10)));
        edges.push(RetimingEdge::new(logic, boundary, Vec::new()));
        output_boundaries.push((logic, boundary));
    }
    let before = RetimingGraph::new(
        RetimingDomain::new(
            format!("{clock:?}"),
            clock_edge,
            format!("{:?}", reset.signal),
            reset.active,
            reset.asynchronous,
        ),
        vertices,
        edges,
    );
    let mut labels = vec![0; before.vertices().len()];
    for vertex in logic_vertices {
        labels[vertex] = -1;
    }
    let certificate = RetimingCertificate::new(labels);
    let after = derive_retimed_graph(&before, &certificate).ok()?;
    verify_retiming_certificate(&before, &after, &certificate).ok()?;
    let output_resets = output_boundaries
        .iter()
        .map(|&(logic, boundary)| {
            after
                .edges()
                .iter()
                .find(|edge| edge.source() == logic && edge.target() == boundary)?
                .reset_values()
                .first()
                .copied()
        })
        .collect::<Option<Vec<_>>>()?;

    let mut candidate = netlist.clone();
    candidate.equivalence_proof.certified_primitive_moves += 1;
    let mut removed = input_registers.into_iter().collect::<Vec<_>>();
    removed.sort_unstable_by(|lhs, rhs| rhs.cmp(lhs));
    for index in removed {
        candidate.cells.remove(index);
    }
    let retimed_ccu_index = candidate
        .cells
        .iter()
        .position(|cell| mapped_cell_name(cell) == ccu_name)?;
    let rewrite = |bit: Bit| match bit {
        Bit::Wire(wire) => input_data.get(&wire).copied().unwrap_or(bit),
        Bit::Zero | Bit::One => bit,
    };
    let retimed_inputs = inputs.map(|slice| slice.map(rewrite));
    let retimed_carry_in = if inject[0] {
        Bit::Zero
    } else {
        rewrite(carry_in)
    };
    let mut next_wire = maximum_mapped_wire(netlist)?.checked_add(1)?;
    let internal_sums = [next_wire, next_wire.checked_add(1)?];
    next_wire = next_wire.checked_add(2)?;
    let internal_carry = next_wire;
    candidate.cells[retimed_ccu_index] = Ecp5Cell::Ccu2c {
        name: format!("retime_forward_{ccu_name}"),
        inputs: retimed_inputs,
        carry_in: retimed_carry_in,
        sums: internal_sums,
        carry_out: internal_carry,
        init,
        inject,
    };
    for (name, data, output, reset_value) in [
        ("sum0", internal_sums[0], sums[0], output_resets[0]),
        ("sum1", internal_sums[1], sums[1], output_resets[1]),
        ("carry", internal_carry, carry_out, output_resets[2]),
    ] {
        if mapped_wire_fanout(netlist, output) == 0 {
            continue;
        }
        candidate.cells.push(Ecp5Cell::FlipFlop {
            name: format!("retime_forward_{ccu_name}_{name}"),
            data: Bit::Wire(data),
            output,
            clock,
            edge: clock_edge,
            enable,
            reset: Some(Reset {
                value: reset_value,
                ..reset
            }),
        });
    }
    Some(candidate)
}

#[allow(clippy::too_many_lines)]
fn backward_retime_ccu2c(netlist: &Ecp5Netlist, register_index: usize) -> Option<Ecp5Netlist> {
    let Ecp5Cell::FlipFlop {
        name: register_name,
        data: Bit::Wire(data_wire),
        output: register_output,
        clock,
        edge: clock_edge,
        enable,
        reset: Some(reset),
    } = netlist.cells.get(register_index)?
    else {
        return None;
    };
    if mapped_wire_is_clock_or_reset(netlist, *register_output) {
        return None;
    }
    let (ccu_index, ccu_name, inputs, carry_in, sums, carry_out, init, inject, output) = netlist
        .cells
        .iter()
        .enumerate()
        .find_map(|(index, cell)| match cell {
            Ecp5Cell::Ccu2c {
                name,
                inputs,
                carry_in,
                sums,
                carry_out,
                init,
                inject,
            } => {
                let output = if sums[0] == *data_wire {
                    Some(CcuRetimeOutput::Sum0)
                } else if sums[1] == *data_wire {
                    Some(CcuRetimeOutput::Sum1)
                } else if *carry_out == *data_wire {
                    Some(CcuRetimeOutput::Carry)
                } else {
                    None
                }?;
                Some((
                    index,
                    name.clone(),
                    *inputs,
                    *carry_in,
                    *sums,
                    *carry_out,
                    *init,
                    *inject,
                    output,
                ))
            }
            Ecp5Cell::Lut4 { .. }
            | Ecp5Cell::FlipFlop { .. }
            | Ecp5Cell::BlockRam { .. }
            | Ecp5Cell::TrellisIo { .. }
            | Ecp5Cell::Jtagg { .. }
            | Ecp5Cell::Pll { .. } => None,
        })?;

    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let mut boundaries = Vec::new();
    let mut logic_vertices = Vec::new();
    let desired_vertex = match output {
        CcuRetimeOutput::Sum0 => append_ccu_logic_vertex(
            &mut vertices,
            &mut edges,
            &mut boundaries,
            inputs[0],
            CcuCarrySource::External(carry_in, CcuInputLocation::CarryIn),
            init[0],
            inject[0],
            false,
            0,
            &mut logic_vertices,
            None,
        ),
        CcuRetimeOutput::Sum1 | CcuRetimeOutput::Carry => {
            let slice1_carry = if inject[1] {
                CcuCarrySource::External(Bit::Zero, CcuInputLocation::CarryIn)
            } else {
                let carry_vertex = append_ccu_logic_vertex(
                    &mut vertices,
                    &mut edges,
                    &mut boundaries,
                    inputs[0],
                    CcuCarrySource::External(carry_in, CcuInputLocation::CarryIn),
                    init[0],
                    inject[0],
                    true,
                    0,
                    &mut logic_vertices,
                    None,
                );
                CcuCarrySource::Internal(carry_vertex)
            };
            append_ccu_logic_vertex(
                &mut vertices,
                &mut edges,
                &mut boundaries,
                inputs[1],
                slice1_carry,
                init[1],
                inject[1],
                output == CcuRetimeOutput::Carry,
                1,
                &mut logic_vertices,
                None,
            )
        }
    };
    let q_vertex = vertices.len();
    vertices.push(RetimingVertex::boundary("q", LogicFunction::new(1, 0b10)));
    edges.push(RetimingEdge::new(
        desired_vertex,
        q_vertex,
        vec![reset.value],
    ));
    let before = RetimingGraph::new(
        RetimingDomain::new(
            format!("{clock:?}"),
            *clock_edge,
            format!("{:?}", reset.signal),
            reset.active,
            reset.asynchronous,
        ),
        vertices,
        edges,
    );
    let mut labels = vec![0; before.vertices().len()];
    for vertex in logic_vertices {
        labels[vertex] = 1;
    }
    let certificate = RetimingCertificate::new(labels);
    let after = derive_retimed_graph(&before, &certificate).ok()?;
    verify_retiming_certificate(&before, &after, &certificate).ok()?;

    let mut next_wire = maximum_mapped_wire(netlist)?.checked_add(1)?;
    let mut candidate = netlist.clone();
    candidate.equivalence_proof.certified_primitive_moves += 1;
    candidate.cells.remove(register_index);
    let uses_slice1 = output != CcuRetimeOutput::Sum0;
    let uses_slice0 = !uses_slice1 || !inject[1];
    let mut retimed_inputs = [[Bit::Zero; 4]; 2];
    if uses_slice0 {
        retimed_inputs[0] = inputs[0];
    }
    if uses_slice1 {
        retimed_inputs[1] = inputs[1];
    }
    let mut retimed_carry_in = if uses_slice0 && !inject[0] {
        carry_in
    } else {
        Bit::Zero
    };
    for boundary in boundaries {
        let Bit::Wire(input_wire) = boundary.bit else {
            continue;
        };
        let reset_value = after
            .edges()
            .iter()
            .find(|edge| edge.source() == boundary.vertex)?
            .reset_values()
            .first()
            .copied()?;
        let output_wire = next_wire;
        next_wire = next_wire.checked_add(1)?;
        let location_name = match boundary.location {
            CcuInputLocation::Pin { slice, pin } => format!("s{slice}p{pin}"),
            CcuInputLocation::CarryIn => "cin".into(),
        };
        candidate.cells.push(Ecp5Cell::FlipFlop {
            name: format!("retime_{register_name}_{ccu_name}_{location_name}"),
            data: Bit::Wire(input_wire),
            output: output_wire,
            clock: *clock,
            edge: *clock_edge,
            enable: *enable,
            reset: Some(Reset {
                value: reset_value,
                ..*reset
            }),
        });
        match boundary.location {
            CcuInputLocation::Pin { slice, pin } => {
                retimed_inputs[slice][pin] = Bit::Wire(output_wire);
            }
            CcuInputLocation::CarryIn => retimed_carry_in = Bit::Wire(output_wire),
        }
    }
    let mut retimed_sums = [0; 2];
    for (slice, sum) in retimed_sums.iter_mut().enumerate() {
        *sum = if output == [CcuRetimeOutput::Sum0, CcuRetimeOutput::Sum1][slice] {
            *register_output
        } else {
            let wire = next_wire;
            next_wire = next_wire.checked_add(1)?;
            wire
        };
    }
    let retimed_carry_out = if output == CcuRetimeOutput::Carry {
        *register_output
    } else {
        next_wire
    };
    let original_outputs = [sums[0], sums[1], carry_out];
    let replace_original = original_outputs
        .iter()
        .all(|wire| mapped_wire_fanout(netlist, *wire) == usize::from(*wire == *data_wire));
    // The retimed CCU2C takes over the register output. When the original CCU
    // keeps other sinks it stays in the netlist, and either branch can collide
    // with a replica name created by an earlier move. Keep every pushed or
    // renamed cell unique among all cells except the one being replaced.
    let replacement_index = if replace_original {
        Some(ccu_index - usize::from(register_index < ccu_index))
    } else {
        None
    };
    let retimed_ccu = Ecp5Cell::Ccu2c {
        name: unique_cell_name(
            &format!("retime_{ccu_name}"),
            &candidate.cells,
            replacement_index,
        ),
        inputs: retimed_inputs,
        carry_in: retimed_carry_in,
        sums: retimed_sums,
        carry_out: retimed_carry_out,
        init,
        inject,
    };
    if let Some(replacement_index) = replacement_index {
        candidate.cells[replacement_index] = retimed_ccu;
    } else {
        candidate.cells.push(retimed_ccu);
    }
    prune_unobservable_retiming_cells(&mut candidate);
    Some(candidate)
}

#[allow(clippy::too_many_arguments)]
fn append_ccu_logic_vertex(
    vertices: &mut Vec<RetimingVertex>,
    edges: &mut Vec<RetimingEdge>,
    boundaries: &mut Vec<CcuBoundary>,
    inputs: [Bit; 4],
    carry: CcuCarrySource,
    init: u16,
    inject: bool,
    carry_output: bool,
    slice: usize,
    logic_vertices: &mut Vec<usize>,
    input_resets: Option<&HashMap<u32, bool>>,
) -> usize {
    let mut sources = Vec::new();
    let mut logic_inputs = [CcuLogicValue::Constant(false); 4];
    for (pin, input) in inputs.into_iter().enumerate() {
        logic_inputs[pin] = append_ccu_external_input(
            vertices,
            boundaries,
            &mut sources,
            input,
            CcuInputLocation::Pin { slice, pin },
            input_resets,
        );
    }
    let carry_input = if inject {
        CcuLogicValue::Constant(false)
    } else {
        match carry {
            CcuCarrySource::External(bit, location) => append_ccu_external_input(
                vertices,
                boundaries,
                &mut sources,
                bit,
                location,
                input_resets,
            ),
            CcuCarrySource::Internal(vertex) => {
                let input = CcuLogicValue::Input(sources.len());
                sources.push((vertex, Vec::new()));
                input
            }
        }
    };
    let function = ccu_logic_function(logic_inputs, carry_input, init, carry_output, sources.len());
    let vertex = vertices.len();
    vertices.push(RetimingVertex::logic(
        if carry_output { "carry" } else { "sum" },
        function,
    ));
    logic_vertices.push(vertex);
    for (source, reset_values) in sources {
        edges.push(RetimingEdge::new(source, vertex, reset_values));
    }
    vertex
}

fn append_ccu_external_input(
    vertices: &mut Vec<RetimingVertex>,
    boundaries: &mut Vec<CcuBoundary>,
    sources: &mut Vec<(usize, Vec<bool>)>,
    bit: Bit,
    location: CcuInputLocation,
    input_resets: Option<&HashMap<u32, bool>>,
) -> CcuLogicValue {
    match bit {
        Bit::Zero => CcuLogicValue::Constant(false),
        Bit::One => CcuLogicValue::Constant(true),
        Bit::Wire(wire) => {
            let vertex = vertices.len();
            vertices.push(RetimingVertex::boundary(
                format!("wire{wire}"),
                LogicFunction::new(0, 0),
            ));
            boundaries.push(CcuBoundary {
                vertex,
                bit,
                location,
            });
            let input = CcuLogicValue::Input(sources.len());
            let reset_values = input_resets
                .and_then(|resets| resets.get(&wire))
                .copied()
                .map_or_else(Vec::new, |value| vec![value]);
            sources.push((vertex, reset_values));
            input
        }
    }
}

fn ccu_logic_function(
    inputs: [CcuLogicValue; 4],
    carry: CcuLogicValue,
    init: u16,
    carry_output: bool,
    input_count: usize,
) -> LogicFunction {
    let mut truth_table = 0u64;
    for assignment in 0..(1usize << input_count) {
        let values = inputs.map(|input| ccu_logic_value(input, assignment));
        let carry = ccu_logic_value(carry, assignment);
        let lut4_index = values
            .iter()
            .enumerate()
            .fold(0usize, |index, (pin, value)| {
                index | (usize::from(*value) << pin)
            });
        let lut2_index = usize::from(values[0]) | (usize::from(values[1]) << 1);
        let lut4 = init & (1u16 << lut4_index) != 0;
        let lut2 = init & (1u16 << lut2_index) != 0;
        let value = if carry_output {
            if lut4 { lut2 } else { carry }
        } else {
            lut4 ^ carry
        };
        truth_table |= u64::from(value) << assignment;
    }
    LogicFunction::new(
        u8::try_from(input_count).expect("CCU slice has at most five inputs"),
        truth_table,
    )
}

fn ccu_logic_value(value: CcuLogicValue, assignment: usize) -> bool {
    match value {
        CcuLogicValue::Constant(value) => value,
        CcuLogicValue::Input(input) => assignment & (1usize << input) != 0,
    }
}

fn merge_equivalent_flip_flops(netlist: &mut Ecp5Netlist) {
    const MAX_SHARED_FANOUT: usize = 2;
    loop {
        let fanouts = mapped_wire_fanouts(netlist);
        let mut canonical = Vec::<(
            Bit,
            u32,
            Bit,
            ClockEdge,
            Option<Control>,
            Option<Reset>,
            usize,
        )>::new();
        let mut duplicates = Vec::new();
        for (index, cell) in netlist.cells.iter().enumerate() {
            let Ecp5Cell::FlipFlop {
                name,
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
            if !name.starts_with("retime_") {
                continue;
            }
            let output_fanout = fanouts.get(output).copied().unwrap_or(0);
            if let Some((_, canonical_output, .., canonical_fanout)) = canonical.iter_mut().find(
                |(
                    candidate_data,
                    _,
                    candidate_clock,
                    candidate_edge,
                    candidate_enable,
                    candidate_reset,
                    candidate_fanout,
                )| {
                    (
                        *candidate_data,
                        *candidate_clock,
                        *candidate_edge,
                        *candidate_enable,
                        *candidate_reset,
                    ) == (*data, *clock, *edge, *enable, *reset)
                        && *candidate_fanout + output_fanout <= MAX_SHARED_FANOUT
                },
            ) {
                duplicates.push((index, *output, *canonical_output));
                *canonical_fanout += output_fanout;
            } else {
                canonical.push((
                    *data,
                    *output,
                    *clock,
                    *edge,
                    *enable,
                    *reset,
                    output_fanout,
                ));
            }
        }
        if duplicates.is_empty() {
            break;
        }
        netlist.equivalence_proof.equivalent_register_merges += duplicates.len();
        for &(_, duplicate, canonical) in &duplicates {
            replace_mapped_wire_uses(netlist, duplicate, canonical);
        }
        duplicates.sort_unstable_by_key(|duplicate| std::cmp::Reverse(duplicate.0));
        for (index, _, _) in duplicates {
            netlist.cells.remove(index);
        }
    }
}

fn replace_mapped_wire_uses(netlist: &mut Ecp5Netlist, from: u32, to: u32) {
    let replace = |bit: &mut Bit| {
        if *bit == Bit::Wire(from) {
            *bit = Bit::Wire(to);
        }
    };
    for port in &mut netlist.ports {
        for bit in &mut port.bits {
            replace(bit);
        }
    }
    for cell in &mut netlist.cells {
        match cell {
            Ecp5Cell::Lut4 { inputs, .. } => {
                for bit in inputs {
                    replace(bit);
                }
            }
            Ecp5Cell::Ccu2c {
                inputs, carry_in, ..
            } => {
                for bit in inputs.iter_mut().flatten() {
                    replace(bit);
                }
                replace(carry_in);
            }
            Ecp5Cell::FlipFlop {
                data,
                clock,
                enable,
                reset,
                ..
            } => {
                replace(data);
                replace(clock);
                if let Some(control) = enable {
                    replace(&mut control.signal);
                }
                if let Some(control) = reset {
                    replace(&mut control.signal);
                }
            }
            Ecp5Cell::BlockRam {
                write_address,
                write_data,
                write_enable,
                read_address,
                read_enable,
                clock,
                ..
            } => {
                for bit in write_address
                    .iter_mut()
                    .chain(write_data)
                    .chain(read_address.iter_mut())
                {
                    replace(bit);
                }
                replace(&mut write_enable.signal);
                if let Some(control) = read_enable {
                    replace(&mut control.signal);
                }
                replace(clock);
            }
            Ecp5Cell::TrellisIo {
                fabric_output,
                tristate,
                ..
            } => {
                replace(fabric_output);
                replace(tristate);
            }
            Ecp5Cell::Jtagg { tdo, .. } => {
                for bit in tdo {
                    replace(bit);
                }
            }
            Ecp5Cell::Pll {
                reference_clock, ..
            } => replace(reference_clock),
        }
    }
}

fn prune_unobservable_retiming_cells(netlist: &mut Ecp5Netlist) {
    loop {
        let fanouts = mapped_wire_fanouts(netlist);
        let previous_len = netlist.cells.len();
        netlist.cells.retain(|cell| {
            !mapped_cell_name(cell).starts_with("retime_")
                || cell_output_bits(cell)
                    .into_iter()
                    .any(|bit| matches!(bit, Bit::Wire(wire) if fanouts.contains_key(&wire)))
        });
        netlist.equivalence_proof.unobservable_cells_removed += previous_len - netlist.cells.len();
        if netlist.cells.len() == previous_len {
            break;
        }
    }
}

fn mapped_cell_name(cell: &Ecp5Cell) -> &str {
    match cell {
        Ecp5Cell::Lut4 { name, .. }
        | Ecp5Cell::Ccu2c { name, .. }
        | Ecp5Cell::FlipFlop { name, .. }
        | Ecp5Cell::BlockRam { name, .. }
        | Ecp5Cell::TrellisIo { name, .. }
        | Ecp5Cell::Jtagg { name, .. }
        | Ecp5Cell::Pll { name, .. } => name,
    }
}

fn unique_cell_name(base: &str, cells: &[Ecp5Cell], skip: Option<usize>) -> String {
    let mut name = base.to_owned();
    let mut suffix = 2usize;
    while cells
        .iter()
        .enumerate()
        .any(|(index, cell)| index != skip.unwrap_or(usize::MAX) && mapped_cell_name(cell) == name)
    {
        name = format!("{base}_{suffix}");
        suffix += 1;
    }
    name
}

fn mapped_wire_is_clock_or_reset(netlist: &Ecp5Netlist, wire: u32) -> bool {
    netlist.cells.iter().any(|cell| match cell {
        Ecp5Cell::FlipFlop { clock, reset, .. } => {
            *clock == Bit::Wire(wire)
                || reset.is_some_and(|control| control.signal == Bit::Wire(wire))
        }
        Ecp5Cell::BlockRam { clock, .. } => *clock == Bit::Wire(wire),
        Ecp5Cell::Lut4 { .. }
        | Ecp5Cell::Ccu2c { .. }
        | Ecp5Cell::TrellisIo { .. }
        | Ecp5Cell::Jtagg { .. }
        | Ecp5Cell::Pll { .. } => false,
    })
}

fn reduced_lut_function(inputs: [Bit; 4], init: u16, variables: &[u32]) -> LogicFunction {
    let mut truth_table = 0u64;
    for assignment in 0..(1usize << variables.len()) {
        let lut_index = inputs
            .iter()
            .enumerate()
            .fold(0usize, |index, (pin, input)| {
                let value = match input {
                    Bit::Zero => false,
                    Bit::One => true,
                    Bit::Wire(wire) => {
                        let variable = variables
                            .iter()
                            .position(|candidate| candidate == wire)
                            .expect("wire was collected as a variable");
                        assignment & (1usize << variable) != 0
                    }
                };
                index | (usize::from(value) << pin)
            });
        if init & (1u16 << lut_index) != 0 {
            truth_table |= 1u64 << assignment;
        }
    }
    LogicFunction::new(
        u8::try_from(variables.len()).expect("LUT has at most four variables"),
        truth_table,
    )
}

fn mapped_wire_fanout(netlist: &Ecp5Netlist, wire: u32) -> usize {
    let mut fanout = 0usize;
    for cell in &netlist.cells {
        for_each_cell_input_bit(cell, |bit| {
            fanout += usize::from(bit == Bit::Wire(wire));
        });
    }
    fanout
        + netlist
            .ports
            .iter()
            .flat_map(|port| &port.bits)
            .filter(|bit| **bit == Bit::Wire(wire))
            .count()
}

fn maximum_mapped_wire(netlist: &Ecp5Netlist) -> Option<u32> {
    netlist
        .ports
        .iter()
        .flat_map(|port| &port.bits)
        .copied()
        .chain(netlist.cells.iter().flat_map(cell_output_bits))
        .filter_map(|bit| match bit {
            Bit::Wire(wire) => Some(wire),
            Bit::Zero | Bit::One => None,
        })
        .max()
}

fn cell_input_bits(cell: &Ecp5Cell) -> Vec<Bit> {
    let mut bits = Vec::new();
    for_each_cell_input_bit(cell, |bit| bits.push(bit));
    bits
}

fn for_each_cell_input_bit(cell: &Ecp5Cell, mut visit: impl FnMut(Bit)) {
    match cell {
        Ecp5Cell::Lut4 { inputs, .. } => {
            for bit in inputs {
                visit(*bit);
            }
        }
        Ecp5Cell::Ccu2c {
            inputs, carry_in, ..
        } => {
            for bit in inputs.iter().flatten() {
                visit(*bit);
            }
            visit(*carry_in);
        }
        Ecp5Cell::FlipFlop {
            data,
            clock,
            enable,
            reset,
            ..
        } => {
            visit(*data);
            visit(*clock);
            if let Some(control) = enable {
                visit(control.signal);
            }
            if let Some(control) = reset {
                visit(control.signal);
            }
        }
        Ecp5Cell::BlockRam {
            write_address,
            write_data,
            write_enable,
            read_address,
            read_enable,
            clock,
            ..
        } => {
            for bit in write_address
                .iter()
                .chain(write_data)
                .chain(read_address.iter())
            {
                visit(*bit);
            }
            visit(write_enable.signal);
            visit(*clock);
            if let Some(control) = read_enable {
                visit(control.signal);
            }
        }
        Ecp5Cell::TrellisIo {
            fabric_output,
            tristate,
            ..
        } => {
            visit(*fabric_output);
            visit(*tristate);
        }
        Ecp5Cell::Jtagg { tdo, .. } => {
            for bit in tdo {
                visit(*bit);
            }
        }
        Ecp5Cell::Pll {
            reference_clock,
            feedback_clock,
            ..
        } => {
            visit(*reference_clock);
            visit(Bit::Wire(*feedback_clock));
        }
    }
}

fn cell_output_bits(cell: &Ecp5Cell) -> Vec<Bit> {
    match cell {
        Ecp5Cell::Lut4 { output, .. } | Ecp5Cell::FlipFlop { output, .. } => {
            vec![Bit::Wire(*output)]
        }
        Ecp5Cell::Ccu2c {
            sums, carry_out, ..
        } => sums
            .iter()
            .copied()
            .chain([*carry_out])
            .map(Bit::Wire)
            .collect(),
        Ecp5Cell::BlockRam { read_data, .. } => read_data.iter().copied().map(Bit::Wire).collect(),
        Ecp5Cell::TrellisIo { fabric_input, .. } => vec![Bit::Wire(*fabric_input)],
        Ecp5Cell::Jtagg {
            tdi,
            clock,
            run_test_idle,
            shift,
            update,
            reset_n,
            clock_enable,
            ..
        } => [*tdi, *clock, *shift, *update, *reset_n]
            .into_iter()
            .chain(*run_test_idle)
            .chain(*clock_enable)
            .map(Bit::Wire)
            .collect(),
        Ecp5Cell::Pll {
            feedback_clock,
            output_clock,
            locked,
            ..
        } => [*feedback_clock, *output_clock, *locked]
            .into_iter()
            .map(Bit::Wire)
            .collect(),
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct MappingQuality {
    period_ps: u32,
}

fn constant_register_values(netlist: &Netlist) -> HashMap<NetId, bool> {
    netlist
        .registers()
        .iter()
        .filter_map(|register| {
            let value = netlist.constant_value(register.data())?;
            if register.reset().is_none_or(|reset| reset.value == value) {
                Some((register.output(), value))
            } else {
                None
            }
        })
        .collect()
}

/// Constant flip-flop outputs must be folded before any consumer logic is
/// materialized so every sink resolves to the literal bit.
fn fold_constant_registers(
    netlist: &Netlist,
    constant_registers: &HashMap<NetId, bool>,
    emitter: &mut LutEmitter<'_>,
) {
    for register in netlist.registers() {
        if let Some(value) = constant_registers.get(&register.output()) {
            emitter.alias_net(register.output(), Bit::from(*value));
        }
    }
}

/// ECP5 carry routing is a dedicated chain: one CCU2C carry-out may feed only
/// one downstream carry-in. Chained certified retiming moves can leave a
/// carry-out driving several slices, which nextpnr cannot pack; candidates
/// must keep every chain link point-to-point.
fn carry_outs_are_point_to_point(netlist: &Ecp5Netlist) -> bool {
    // Only downstream carry-ins compete for the dedicated carry routing;
    // fabric loads (flip-flop data, LUT pins) hang off chain ends legally.
    let mut cin_consumers: HashMap<u32, usize> = HashMap::new();
    for cell in &netlist.cells {
        if let Ecp5Cell::Ccu2c {
            carry_in: Bit::Wire(wire),
            ..
        } = cell
        {
            *cin_consumers.entry(*wire).or_insert(0) += 1;
        }
    }
    netlist.cells.iter().all(|cell| match cell {
        Ecp5Cell::Ccu2c { carry_out, .. } => {
            cin_consumers.get(carry_out).copied().unwrap_or(0) <= 1
        }
        _ => true,
    })
}

fn split_branched_carry_outs(netlist: &mut Ecp5Netlist) -> usize {
    // Repairing a branch moves it one slice upstream (the replica shares the
    // parent's carry-in), so passes must repeat until the cascade reaches a
    // chain root whose carry-in is external.
    let mut total = 0usize;
    while {
        let replicas = split_branched_carry_outs_once(netlist);
        total += replicas;
        replicas > 0
    } {}
    total
}

fn split_branched_carry_outs_once(netlist: &mut Ecp5Netlist) -> usize {
    let mut next_wire = maximum_mapped_wire(netlist)
        .and_then(|wire| wire.checked_add(1))
        .unwrap_or(1);
    let mut replicas = 0usize;
    let mut index = 0usize;
    while index < netlist.cells.len() {
        index += 1;
        let Some((inputs, carry_in, _sums, carry_out, init, inject)) =
            (match &netlist.cells[index - 1] {
                Ecp5Cell::Ccu2c {
                    name: _,
                    inputs,
                    carry_in,
                    sums,
                    carry_out,
                    init,
                    inject,
                } => Some((*inputs, *carry_in, *sums, *carry_out, *init, *inject)),
                _ => None,
            })
        else {
            continue;
        };
        let carry_wire = carry_out;
        let consumers = netlist
            .cells
            .iter()
            .enumerate()
            .filter(|(consumer_index, cell)| match cell {
                Ecp5Cell::Ccu2c {
                    carry_in: Bit::Wire(wire),
                    ..
                } => *consumer_index != index - 1 && *wire == carry_wire,
                _ => false,
            })
            .map(|(consumer_index, _)| consumer_index)
            .collect::<Vec<_>>();
        for consumer_index in consumers.into_iter().skip(1) {
            let Some(clone_output) = next_wire.checked_add(0) else {
                break;
            };
            let Some(allocated) = next_wire.checked_add(3) else {
                break;
            };
            // One carry-out plus two sum outputs.
            let clone_sums = [next_wire + 1, next_wire + 2];
            next_wire = allocated;
            let name = match &netlist.cells[index - 1] {
                Ecp5Cell::Ccu2c { name, .. } => name.clone(),
                _ => unreachable!("matched Ccu2c above"),
            };
            replace_wire_in_cell_inputs(
                &mut netlist.cells[consumer_index],
                carry_wire,
                clone_output,
            );
            netlist.cells.push(Ecp5Cell::Ccu2c {
                name: unique_cell_name(&format!("{name}_carry{replicas}"), &netlist.cells, None),
                inputs,
                carry_in,
                sums: clone_sums,
                carry_out: clone_output,
                init,
                inject,
            });
            replicas += 1;
        }
    }
    if replicas > 0 {
        netlist.equivalence_proof.equivalent_logic_replications += replicas;
    }
    replicas
}

fn validate_io_timing(
    netlist: &Netlist,
    constraints: &IoTimingConstraints,
) -> Result<(), MappingError> {
    let validate = |name: &str, expected: IrPortDirection| {
        let port = netlist
            .ports()
            .iter()
            .find(|port| port.name() == name)
            .ok_or_else(|| MappingError::TimingPortNotFound(name.to_owned()))?;
        if port.direction() != expected {
            return Err(MappingError::TimingPortDirection {
                port: name.to_owned(),
                expected: match expected {
                    IrPortDirection::Input => PortDirection::Input,
                    IrPortDirection::Output => PortDirection::Output,
                },
                actual: match port.direction() {
                    IrPortDirection::Input => PortDirection::Input,
                    IrPortDirection::Output => PortDirection::Output,
                },
            });
        }
        Ok(())
    };
    for name in constraints.input_delays_ps.keys() {
        validate(name, IrPortDirection::Input)?;
    }
    for name in constraints.output_delays_ps.keys() {
        validate(name, IrPortDirection::Output)?;
    }
    Ok(())
}

fn resolve_io_timing(ports: &[MappedPort], constraints: &IoTimingConstraints) -> ResolvedIoTiming {
    let mut resolved = ResolvedIoTiming::default();
    for (name, delay_ps) in &constraints.input_delays_ps {
        let port = ports
            .iter()
            .find(|port| port.name == *name)
            .expect("I/O timing constraints were validated");
        for bit in &port.bits {
            if let Bit::Wire(wire) = bit {
                resolved.input_arrivals_ps.insert(*wire, *delay_ps);
            }
        }
    }
    for (name, delay_ps) in &constraints.output_delays_ps {
        let port = ports
            .iter()
            .find(|port| port.name == *name)
            .expect("I/O timing constraints were validated");
        resolved
            .output_delays_ps
            .extend(port.bits.iter().map(|bit| (*bit, *delay_ps)));
    }
    resolved
}

#[allow(clippy::too_many_lines)]
fn map_once(
    netlist: &Netlist,
    options: MappingOptions,
    io_timing: &IoTimingConstraints,
) -> Result<(Ecp5Netlist, MappingQuality), MappingError> {
    netlist.validate()?;
    // ECP5 loads every flip-flop through GSR at configuration and REGSET
    // selects the loaded value, so a flip-flop whose data is a constant and
    // whose reset (when present) asserts the same constant never leaves its
    // configured state: it is indistinguishable from a constant driver. A
    // reset asserting a different value still forces the flip-flop to leave
    // that state after the first clock edge, so it must stay a flip-flop.
    let constant_registers = constant_register_values(netlist);
    let demand = MappingDemand::collect(netlist, &constant_registers);
    let cuts = CutDatabase::analyze(netlist);
    let cover = LutCover::select(netlist, &cuts, &demand.roots, options, io_timing);
    let (period_ps, _) = cover.estimated_register_period_ps(netlist);
    let quality = MappingQuality { period_ps };
    let mut emitter = LutEmitter::new(netlist, &cover);

    fold_constant_registers(netlist, &constant_registers, &mut emitter);

    map_retained_cells(netlist, options, &mut emitter);

    for root in &demand.roots {
        emitter.map_net(*root);
    }

    for port in netlist
        .ports()
        .iter()
        .filter(|port| port.direction() == IrPortDirection::Output)
    {
        for output in port.bits() {
            let node = node_for(netlist, *output);
            let source = node.inputs()[0];
            let output_bit = emitter.mapped_net(source);
            emitter.alias_net(*output, output_bit);
        }
    }

    let (bits, mut cells) = emitter.finish();

    for register in netlist.registers() {
        if constant_registers.contains_key(&register.output()) {
            continue;
        }
        cells.push(Ecp5Cell::FlipFlop {
            // nextpnr rejects a cell whose name is also a top-level IO name.
            // Keep primitive cells in a dedicated namespace even when an RTL
            // output is directly registered.
            name: format!("ff_{}", register.name()),
            data: mapped_bit(&bits, register.data()),
            output: wire_number(register.output()),
            clock: mapped_bit(&bits, register.clock()),
            edge: register.edge(),
            enable: register.enable().map(|enable| Control {
                signal: mapped_bit(&bits, enable.signal),
                active: enable.active,
            }),
            reset: register.reset().map(|reset| Reset {
                signal: mapped_bit(&bits, reset.signal),
                active: reset.active,
                asynchronous: reset.asynchronous,
                value: reset.value,
            }),
        });
    }

    for memory in netlist.memories() {
        map_memory(memory, &bits, &mut cells)?;
    }

    let ports = netlist
        .ports()
        .iter()
        .map(|port| MappedPort {
            name: port.name().into(),
            direction: match port.direction() {
                IrPortDirection::Input => PortDirection::Input,
                IrPortDirection::Output => PortDirection::Output,
            },
            bits: port
                .bits()
                .iter()
                .map(|net| mapped_bit(&bits, *net))
                .collect(),
        })
        .collect::<Vec<_>>();
    let io_timing = resolve_io_timing(&ports, io_timing);

    Ok((
        Ecp5Netlist {
            name: netlist.name().into(),
            ports,
            cells,
            retiming: RetimingSelection {
                applied: false,
                original_lut_depth: 0,
                selected_lut_depth: 0,
                original_critical_registers: 0,
                selected_critical_registers: 0,
                original_period_ps: quality.period_ps,
                selected_period_ps: quality.period_ps,
                original_overall_period_ps: quality.period_ps,
                selected_overall_period_ps: quality.period_ps,
                original_registers: netlist.registers().len(),
                selected_registers: netlist.registers().len(),
                certified_primitive_moves: 0,
                equivalent_register_merges: 0,
                equivalent_logic_replications: 0,
                equivalent_physical_rewires: 0,
                unobservable_cells_removed: 0,
                equivalence_signed_off: true,
            },
            equivalence_proof: MappedEquivalenceProof {
                valid: true,
                ..MappedEquivalenceProof::default()
            },
            placement_hints: BTreeMap::new(),
            io_timing,
        },
        quality,
    ))
}

const CCU2C_ARITH_INIT: u16 = 0x96aa;

#[derive(Clone, Copy)]
enum RetainedCell<'a> {
    Arithmetic(&'a ArithmeticCell),
    Comparison(&'a ComparisonCell),
}

fn map_retained_cells(netlist: &Netlist, options: MappingOptions, emitter: &mut LutEmitter<'_>) {
    let mut retained = netlist
        .arithmetic()
        .iter()
        .map(RetainedCell::Arithmetic)
        .chain(netlist.comparisons().iter().map(RetainedCell::Comparison))
        .collect::<Vec<_>>();
    retained.sort_by_key(|cell| cell.output_index());
    for cell in retained {
        match cell {
            RetainedCell::Arithmetic(arithmetic) => {
                let use_carry = match options.arithmetic {
                    ArithmeticMapping::Auto => arithmetic.outputs().len() > 4,
                    ArithmeticMapping::CarryChain => true,
                    ArithmeticMapping::Lut4 => false,
                };
                if use_carry {
                    map_arithmetic_carry(arithmetic, emitter);
                } else {
                    map_arithmetic_luts(arithmetic, emitter);
                }
            }
            RetainedCell::Comparison(comparison) => map_comparison_carry(comparison, emitter),
        }
    }
}

impl RetainedCell<'_> {
    fn output_index(self) -> u32 {
        match self {
            Self::Arithmetic(cell) => cell.outputs()[0].index(),
            Self::Comparison(cell) => cell.output().index(),
        }
    }
}

fn map_arithmetic_carry(arithmetic: &ArithmeticCell, emitter: &mut LutEmitter<'_>) {
    let subtract = arithmetic.operation() == ArithmeticOp::Subtract;
    let mut carry = arithmetic
        .carry_in()
        .map_or(Bit::from(subtract), |carry| emitter.map_net(carry));
    for (pair, bit) in (0..arithmetic.outputs().len()).step_by(2).enumerate() {
        let mut inputs = [[Bit::Zero; 4]; 2];
        let mut sums = [0; 2];
        for slice in 0..2 {
            let index = bit + slice;
            if index < arithmetic.outputs().len() {
                inputs[slice] = [
                    emitter.map_net(arithmetic.lhs()[index]),
                    emitter.map_net(arithmetic.rhs()[index]),
                    Bit::from(subtract),
                    Bit::One,
                ];
                sums[slice] = wire_number(arithmetic.outputs()[index]);
                emitter.alias_net(arithmetic.outputs()[index], Bit::Wire(sums[slice]));
            } else {
                inputs[slice] = [Bit::Zero, Bit::Zero, Bit::from(subtract), Bit::One];
                sums[slice] = emitter.fresh_wire();
            }
        }
        let carry_out = emitter.fresh_wire();
        emitter.push_cell(Ecp5Cell::Ccu2c {
            name: format!("ccu_{}_{pair}", arithmetic.name()),
            inputs,
            carry_in: carry,
            sums,
            carry_out,
            init: [CCU2C_ARITH_INIT; 2],
            inject: [false; 2],
        });
        carry = Bit::Wire(carry_out);
    }
}

fn map_comparison_carry(comparison: &ComparisonCell, emitter: &mut LutEmitter<'_>) {
    let mut carry = Bit::from(comparison.operation().includes_equal());
    let pairs = comparison.lhs().len().div_ceil(2);
    for (pair, bit) in (0..comparison.lhs().len()).step_by(2).enumerate() {
        let mut inputs = [[Bit::Zero; 4]; 2];
        let mut sums = [0; 2];
        for slice in 0..2 {
            let index = bit + slice;
            inputs[slice] = if index < comparison.lhs().len() {
                [
                    emitter.map_net(comparison.rhs()[index]),
                    emitter.map_net(comparison.lhs()[index]),
                    Bit::One,
                    Bit::One,
                ]
            } else {
                // The unused high slice of an odd-width comparison computes
                // 0 + !0 + carry, propagating carry without changing it.
                [Bit::Zero, Bit::Zero, Bit::One, Bit::One]
            };
            sums[slice] = emitter.fresh_wire();
        }
        let last = pair + 1 == pairs;
        let carry_out = if last && !comparison.operation().is_signed() {
            wire_number(comparison.output())
        } else {
            emitter.fresh_wire()
        };
        emitter.push_cell(Ecp5Cell::Ccu2c {
            name: format!("ccu_{}_{pair}", comparison.name()),
            inputs,
            carry_in: carry,
            sums,
            carry_out,
            init: [CCU2C_ARITH_INIT; 2],
            inject: [false; 2],
        });
        carry = Bit::Wire(carry_out);
    }

    if comparison.operation().is_signed() {
        let output = wire_number(comparison.output());
        let most_significant = comparison.lhs().len() - 1;
        let lhs_sign = emitter.map_net(comparison.lhs()[most_significant]);
        let rhs_sign = emitter.map_net(comparison.rhs()[most_significant]);
        emitter.push_cell(Ecp5Cell::Lut4 {
            name: format!("lut_{}_signed", comparison.name()),
            inputs: [lhs_sign, rhs_sign, carry, Bit::Zero],
            output,
            init: signed_comparison_truth_table(),
        });
        emitter.alias_net(comparison.output(), Bit::Wire(output));
    } else {
        emitter.alias_net(comparison.output(), carry);
    }
}

fn signed_comparison_truth_table() -> u16 {
    (0..16).fold(0, |table, assignment| {
        let lhs_sign = assignment & 1 != 0;
        let rhs_sign = assignment & 2 != 0;
        let unsigned_relation = assignment & 4 != 0;
        let value = if lhs_sign == rhs_sign {
            unsigned_relation
        } else {
            lhs_sign
        };
        table | (u16::from(value) << assignment)
    })
}

fn map_arithmetic_luts(arithmetic: &ArithmeticCell, emitter: &mut LutEmitter<'_>) {
    let subtract = arithmetic.operation() == ArithmeticOp::Subtract;
    let mut carry = arithmetic
        .carry_in()
        .map_or(Bit::from(subtract), |carry| emitter.map_net(carry));
    for bit in 0..arithmetic.outputs().len() {
        let lhs = emitter.map_net(arithmetic.lhs()[bit]);
        let rhs = emitter.map_net(arithmetic.rhs()[bit]);
        let output = wire_number(arithmetic.outputs()[bit]);
        emitter.push_cell(Ecp5Cell::Lut4 {
            name: format!("lut_{}_sum_{bit}", arithmetic.name()),
            inputs: [lhs, rhs, carry, Bit::Zero],
            output,
            init: arithmetic_truth_table(subtract, false),
        });
        emitter.alias_net(arithmetic.outputs()[bit], Bit::Wire(output));

        if bit + 1 < arithmetic.outputs().len() {
            let carry_out = emitter.fresh_wire();
            emitter.push_cell(Ecp5Cell::Lut4 {
                name: format!("lut_{}_carry_{bit}", arithmetic.name()),
                inputs: [lhs, rhs, carry, Bit::Zero],
                output: carry_out,
                init: arithmetic_truth_table(subtract, true),
            });
            carry = Bit::Wire(carry_out);
        }
    }
}

fn arithmetic_truth_table(subtract: bool, carry_output: bool) -> u16 {
    (0..16).fold(0, |table, assignment| {
        let lhs = assignment & 1 != 0;
        let rhs = assignment & 2 != 0;
        let carry = assignment & 4 != 0;
        let effective_rhs = rhs ^ subtract;
        let value = if carry_output {
            u8::from(lhs) + u8::from(effective_rhs) + u8::from(carry) >= 2
        } else {
            lhs ^ effective_rhs ^ carry
        };
        table | (u16::from(value) << assignment)
    })
}

fn node_for(netlist: &Netlist, net: NetId) -> &struo_ir::Node {
    &netlist.nodes()[net.index() as usize]
}

fn map_memory(
    memory: &MemoryCell,
    bits: &[Option<Bit>],
    cells: &mut Vec<Ecp5Cell>,
) -> Result<(), MappingError> {
    let geometry_error = || MappingError::UnsupportedMemoryGeometry {
        memory: memory.name().into(),
        depth: memory.depth(),
        width: memory.write_data().len(),
    };
    let chunk_width = maximum_block_ram_width(memory.depth()).ok_or_else(&geometry_error)?;
    let chunk_count = memory.write_data().len().div_ceil(usize::from(chunk_width));
    for (chunk, (write_data, read_data)) in memory
        .write_data()
        .chunks(usize::from(chunk_width))
        .zip(memory.read_data().chunks(usize::from(chunk_width)))
        .enumerate()
    {
        let word_width = u8::try_from(write_data.len()).map_err(|_| geometry_error())?;
        let physical_width =
            block_ram_width(memory.depth(), word_width).ok_or_else(&geometry_error)?;
        let mut write_address = physical_address(bits, memory.write_address(), physical_width);
        if physical_width == 18 {
            // In 18-bit mode ADA0/ADA1 are the two 9-bit byte enables rather
            // than address pins. This IR writes whole words only.
            write_address[0] = Bit::One;
            write_address[1] = Bit::One;
        }
        cells.push(Ecp5Cell::BlockRam {
            name: if chunk_count == 1 {
                format!("bram_{}", memory.name())
            } else {
                format!("bram_{}_{chunk}", memory.name())
            },
            depth: memory.depth(),
            word_width,
            physical_width,
            write_address: Box::new(write_address),
            write_data: write_data
                .iter()
                .map(|net| mapped_bit(bits, *net))
                .collect(),
            write_enable: Control {
                signal: mapped_bit(bits, memory.write_enable().signal),
                active: memory.write_enable().active,
            },
            read_address: Box::new(physical_address(
                bits,
                memory.read_address(),
                physical_width,
            )),
            read_data: read_data.iter().map(|net| wire_number(*net)).collect(),
            read_enable: memory.read_enable().map(|enable| Control {
                signal: mapped_bit(bits, enable.signal),
                active: enable.active,
            }),
            clock: mapped_bit(bits, memory.clock()),
            edge: memory.edge(),
        });
    }
    Ok(())
}

impl From<bool> for Bit {
    fn from(value: bool) -> Self {
        if value { Self::One } else { Self::Zero }
    }
}

fn wire_number(net: NetId) -> u32 {
    net.index()
        .checked_add(2)
        .expect("net identifier exceeds the Yosys JSON range")
}

fn wire_for(net: NetId) -> Bit {
    Bit::Wire(wire_number(net))
}

fn mapped_bit(bits: &[Option<Bit>], net: NetId) -> Bit {
    bits[net.index() as usize].expect("validated nodes are topologically ordered")
}

fn block_ram_width(depth: u32, word_width: u8) -> Option<u8> {
    [
        (1u8, 16_384u32),
        (2, 8_192),
        (4, 4_096),
        (9, 2_048),
        (18, 1_024),
    ]
    .into_iter()
    .find_map(|(width, capacity)| (word_width <= width && depth <= capacity).then_some(width))
}

fn maximum_block_ram_width(depth: u32) -> Option<u8> {
    [
        (18u8, 1_024u32),
        (9, 2_048),
        (4, 4_096),
        (2, 8_192),
        (1, 16_384),
    ]
    .into_iter()
    .find_map(|(width, capacity)| (depth <= capacity).then_some(width))
}

fn physical_address(bits: &[Option<Bit>], address: &[NetId], width: u8) -> [Bit; 14] {
    let shift = match width {
        1 => 0,
        2 => 1,
        4 => 2,
        9 => 3,
        18 => 4,
        _ => unreachable!("validated DP16KD width"),
    };
    let mut physical = [Bit::Zero; 14];
    for (target, source) in physical[shift..].iter_mut().zip(address) {
        *target = mapped_bit(bits, *source);
    }
    physical
}

/// ECP5 technology-mapping failure.
#[derive(Debug)]
pub enum MappingError {
    /// Source netlist is invalid.
    InvalidNetlist(ValidationError),
    /// A named I/O timing port does not exist.
    TimingPortNotFound(String),
    /// An I/O timing constraint was attached to a port of the wrong direction.
    TimingPortDirection {
        /// Port name.
        port: String,
        /// Direction required by the constraint.
        expected: PortDirection,
        /// Actual port direction.
        actual: PortDirection,
    },
    /// A logical memory cannot be width-tiled into DP16KD primitives.
    UnsupportedMemoryGeometry {
        /// Memory name.
        memory: String,
        /// Requested word count.
        depth: u32,
        /// Requested word width.
        width: usize,
    },
    /// A named split I/O port does not exist.
    IoPortNotFound(String),
    /// The physical pad name conflicts with an existing mapped port.
    IoPortAlreadyExists(String),
    /// The two logical sides of an open-drain binding name the same port.
    IoPortsMustDiffer {
        /// Core input port.
        input: String,
        /// Core drive-low output port.
        drive_low: String,
    },
    /// A split I/O port has the wrong direction.
    IoPortDirection {
        /// Port name.
        port: String,
        /// Direction required by the binding role.
        expected: PortDirection,
        /// Actual mapped direction.
        actual: PortDirection,
    },
    /// Open-drain bindings currently operate on scalar ports.
    IoPortNotScalar {
        /// Port name.
        port: String,
        /// Actual bit width.
        width: usize,
    },
    /// A mapped input unexpectedly contains a constant instead of a wire.
    IoInputIsConstant(String),
    /// No wire number remains for an inserted I/O primitive.
    MappedWireOverflow,
    /// The mapped design already contains the device's single JTAG block.
    JtaggAlreadyBound,
    /// A named JTAG fabric port does not exist.
    JtaggPortNotFound(String),
    /// One top-level port was assigned to more than one JTAG role.
    JtaggPortRepeated(String),
    /// A JTAG fabric port has the wrong direction.
    JtaggPortDirection {
        /// Port name.
        port: String,
        /// Direction required by the binding role.
        expected: PortDirection,
        /// Actual mapped direction.
        actual: PortDirection,
    },
    /// JTAG bindings operate on scalar ports.
    JtaggPortNotScalar {
        /// Port name.
        port: String,
        /// Actual bit width.
        width: usize,
    },
    /// A JTAGG output cannot drive a constant top-level input.
    JtaggOutputIsConstant(String),
    /// A named PLL boundary port does not exist.
    PllPortNotFound(String),
    /// One top-level port was assigned to more than one PLL role.
    PllPortRepeated(String),
    /// A PLL boundary port is not a core input.
    PllPortDirection {
        /// Port name.
        port: String,
        /// Actual mapped direction.
        actual: PortDirection,
    },
    /// PLL boundary bindings operate on scalar ports.
    PllPortNotScalar {
        /// Port name.
        port: String,
        /// Actual bit width.
        width: usize,
    },
    /// A PLL output cannot drive a constant logical input.
    PllOutputIsConstant(String),
}

impl Display for MappingError {
    #[allow(clippy::too_many_lines)]
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidNetlist(error) => write!(formatter, "invalid netlist: {error}"),
            Self::TimingPortNotFound(port) => {
                write!(formatter, "I/O timing port `{port}` was not found")
            }
            Self::TimingPortDirection {
                port,
                expected,
                actual,
            } => write!(
                formatter,
                "I/O timing port `{port}` has direction {actual:?}, expected {expected:?}"
            ),
            Self::UnsupportedMemoryGeometry {
                memory,
                depth,
                width,
            } => write!(
                formatter,
                "memory {memory} ({depth}x{width}) cannot be mapped to ECP5 DP16KD primitives"
            ),
            Self::IoPortNotFound(port) => {
                write!(formatter, "open-drain I/O port `{port}` was not found")
            }
            Self::IoPortAlreadyExists(port) => {
                write!(
                    formatter,
                    "open-drain physical port `{port}` already exists"
                )
            }
            Self::IoPortsMustDiffer { input, drive_low } => write!(
                formatter,
                "open-drain input `{input}` and drive-low output `{drive_low}` must be different ports"
            ),
            Self::IoPortDirection {
                port,
                expected,
                actual,
            } => write!(
                formatter,
                "open-drain port `{port}` has direction {actual:?}, expected {expected:?}"
            ),
            Self::IoPortNotScalar { port, width } => write!(
                formatter,
                "open-drain port `{port}` has width {width}, expected a scalar port"
            ),
            Self::IoInputIsConstant(port) => write!(
                formatter,
                "open-drain input port `{port}` is a constant rather than a mapped wire"
            ),
            Self::MappedWireOverflow => formatter
                .write_str("mapped wire number overflow while inserting a target primitive"),
            Self::JtaggAlreadyBound => {
                formatter.write_str("the ECP5 netlist already contains a JTAGG primitive")
            }
            Self::JtaggPortNotFound(port) => {
                write!(formatter, "JTAGG fabric port `{port}` was not found")
            }
            Self::JtaggPortRepeated(port) => {
                write!(
                    formatter,
                    "JTAGG fabric port `{port}` is assigned more than once"
                )
            }
            Self::JtaggPortDirection {
                port,
                expected,
                actual,
            } => write!(
                formatter,
                "JTAGG fabric port `{port}` has direction {actual:?}, expected {expected:?}"
            ),
            Self::JtaggPortNotScalar { port, width } => write!(
                formatter,
                "JTAGG fabric port `{port}` has width {width}, expected a scalar port"
            ),
            Self::JtaggOutputIsConstant(port) => write!(
                formatter,
                "JTAGG output port `{port}` is a constant rather than a mapped wire"
            ),
            Self::PllPortNotFound(port) => {
                write!(formatter, "PLL boundary port `{port}` was not found")
            }
            Self::PllPortRepeated(port) => {
                write!(
                    formatter,
                    "PLL boundary port `{port}` is assigned more than once"
                )
            }
            Self::PllPortDirection { port, actual } => write!(
                formatter,
                "PLL boundary port `{port}` has direction {actual:?}, expected Input"
            ),
            Self::PllPortNotScalar { port, width } => write!(
                formatter,
                "PLL boundary port `{port}` has width {width}, expected a scalar port"
            ),
            Self::PllOutputIsConstant(port) => write!(
                formatter,
                "PLL output port `{port}` is a constant rather than a mapped wire"
            ),
        }
    }
}

impl Error for MappingError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidNetlist(error) => Some(error),
            Self::UnsupportedMemoryGeometry { .. }
            | Self::TimingPortNotFound(_)
            | Self::TimingPortDirection { .. }
            | Self::IoPortNotFound(_)
            | Self::IoPortAlreadyExists(_)
            | Self::IoPortsMustDiffer { .. }
            | Self::IoPortDirection { .. }
            | Self::IoPortNotScalar { .. }
            | Self::IoInputIsConstant(_)
            | Self::MappedWireOverflow
            | Self::JtaggAlreadyBound
            | Self::JtaggPortNotFound(_)
            | Self::JtaggPortRepeated(_)
            | Self::JtaggPortDirection { .. }
            | Self::JtaggPortNotScalar { .. }
            | Self::JtaggOutputIsConstant(_)
            | Self::PllPortNotFound(_)
            | Self::PllPortRepeated(_)
            | Self::PllPortDirection { .. }
            | Self::PllPortNotScalar { .. }
            | Self::PllOutputIsConstant(_) => None,
        }
    }
}

impl From<ValidationError> for MappingError {
    fn from(error: ValidationError) -> Self {
        Self::InvalidNetlist(error)
    }
}

/// nextpnr Yosys-JSON serialization failure.
#[derive(Debug)]
pub enum NextpnrJsonError {
    /// Two mapped cells share a name, so the JSON cell map would drop one.
    DuplicateCellName {
        /// Duplicated cell name.
        name: String,
    },
    /// JSON serialization failed.
    Serialization(serde_json::Error),
}

impl Display for NextpnrJsonError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateCellName { name } => {
                write!(formatter, "duplicate mapped cell name `{name}`")
            }
            Self::Serialization(error) => write!(formatter, "JSON serialization failed: {error}"),
        }
    }
}

impl Error for NextpnrJsonError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::DuplicateCellName { .. } => None,
            Self::Serialization(error) => Some(error),
        }
    }
}

#[derive(Serialize)]
struct JsonDesign {
    creator: &'static str,
    modules: BTreeMap<String, JsonModule>,
}

#[derive(Serialize)]
struct JsonModule {
    attributes: BTreeMap<String, String>,
    ports: BTreeMap<String, JsonPort>,
    cells: BTreeMap<String, JsonCell>,
    netnames: BTreeMap<String, JsonNet>,
}

#[derive(Serialize)]
struct JsonPort {
    direction: &'static str,
    bits: Vec<Bit>,
}

#[derive(Serialize)]
struct JsonCell {
    hide_name: u8,
    r#type: &'static str,
    parameters: BTreeMap<String, String>,
    attributes: BTreeMap<String, String>,
    port_directions: BTreeMap<String, &'static str>,
    connections: BTreeMap<String, Vec<Bit>>,
}

#[derive(Serialize)]
struct JsonNet {
    hide_name: u8,
    bits: Vec<Bit>,
    attributes: BTreeMap<String, String>,
}

impl From<&Ecp5Netlist> for JsonDesign {
    fn from(netlist: &Ecp5Netlist) -> Self {
        let ports = netlist
            .ports
            .iter()
            .map(|port| {
                (
                    port.name.clone(),
                    JsonPort {
                        direction: match port.direction {
                            PortDirection::Input => "input",
                            PortDirection::Output => "output",
                            PortDirection::Inout => "inout",
                        },
                        bits: port.bits.clone(),
                    },
                )
            })
            .collect();
        let netnames = netlist
            .ports
            .iter()
            .map(|port| {
                (
                    port.name.clone(),
                    JsonNet {
                        hide_name: 0,
                        bits: port.bits.clone(),
                        attributes: BTreeMap::new(),
                    },
                )
            })
            .collect();
        let mut cells = netlist
            .cells
            .iter()
            .map(json_cell)
            .collect::<BTreeMap<_, _>>();
        for (name, bel) in &netlist.placement_hints {
            let Some(cell) = cells.get_mut(name) else {
                continue;
            };
            cell.attributes.insert("NEXTPNR_BEL".into(), bel.clone());
            cell.attributes
                .insert("BEL_STRENGTH".into(), format!("{:032b}", 1));
        }
        Self {
            creator: "Struo",
            modules: [(
                netlist.name.clone(),
                JsonModule {
                    attributes: [("top".into(), format!("{:032b}", 1))]
                        .into_iter()
                        .collect(),
                    ports,
                    cells,
                    netnames,
                },
            )]
            .into_iter()
            .collect(),
        }
    }
}

#[allow(clippy::too_many_lines)]
fn json_cell(cell: &Ecp5Cell) -> (String, JsonCell) {
    match cell {
        Ecp5Cell::Lut4 {
            name,
            inputs,
            output,
            init,
        } => (name.clone(), json_lut(*inputs, *output, *init)),
        Ecp5Cell::Ccu2c {
            name,
            inputs,
            carry_in,
            sums,
            carry_out,
            init,
            inject,
        } => (
            name.clone(),
            json_ccu2c(*inputs, *carry_in, *sums, *carry_out, *init, *inject),
        ),
        Ecp5Cell::FlipFlop {
            name,
            data,
            output,
            clock,
            edge,
            enable,
            reset,
        } => (
            name.clone(),
            json_flip_flop(*data, *output, *clock, *edge, *enable, *reset),
        ),
        Ecp5Cell::BlockRam {
            name,
            physical_width,
            write_address,
            write_data,
            write_enable,
            read_address,
            read_data,
            read_enable,
            clock,
            edge,
            ..
        } => (
            name.clone(),
            json_block_ram(
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
        ),
        Ecp5Cell::TrellisIo {
            name,
            pad,
            fabric_output,
            fabric_input,
            tristate,
        } => (
            name.clone(),
            json_trellis_io(*pad, *fabric_output, *fabric_input, *tristate),
        ),
        Ecp5Cell::Jtagg {
            name,
            tdo,
            tdi,
            clock,
            run_test_idle,
            shift,
            update,
            reset_n,
            clock_enable,
            extension_register_1,
            extension_register_2,
        } => (
            name.clone(),
            json_jtagg(
                *tdo,
                *tdi,
                *clock,
                *run_test_idle,
                *shift,
                *update,
                *reset_n,
                *clock_enable,
                *extension_register_1,
                *extension_register_2,
            ),
        ),
        Ecp5Cell::Pll {
            name,
            reference_clock,
            feedback_clock,
            output_clock,
            locked,
            fabric_output,
            feedback_output,
            parameters,
            attributes,
        } => (
            name.clone(),
            json_pll(
                *reference_clock,
                *feedback_clock,
                *output_clock,
                *locked,
                *fabric_output,
                *feedback_output,
                parameters,
                attributes,
            ),
        ),
    }
}

#[allow(clippy::too_many_arguments)]
fn json_pll(
    reference_clock: Bit,
    feedback_clock: u32,
    output_clock: u32,
    locked: u32,
    fabric_output: PllOutput,
    feedback_output: PllOutput,
    parameters: &BTreeMap<String, String>,
    attributes: &BTreeMap<String, String>,
) -> JsonCell {
    let mut port_directions = BTreeMap::from([
        ("RST".into(), "input"),
        ("STDBY".into(), "input"),
        ("CLKI".into(), "input"),
        ("CLKFB".into(), "input"),
        ("PHASESEL0".into(), "input"),
        ("PHASESEL1".into(), "input"),
        ("PHASEDIR".into(), "input"),
        ("PHASESTEP".into(), "input"),
        ("PHASELOADREG".into(), "input"),
        ("PLLWAKESYNC".into(), "input"),
        ("ENCLKOP".into(), "input"),
        ("LOCK".into(), "output"),
    ]);
    let mut connections = BTreeMap::from([
        ("RST".into(), vec![Bit::Zero]),
        ("STDBY".into(), vec![Bit::Zero]),
        ("CLKI".into(), vec![reference_clock]),
        ("CLKFB".into(), vec![Bit::Wire(feedback_clock)]),
        ("PHASESEL0".into(), vec![Bit::Zero]),
        ("PHASESEL1".into(), vec![Bit::Zero]),
        ("PHASEDIR".into(), vec![Bit::One]),
        ("PHASESTEP".into(), vec![Bit::One]),
        ("PHASELOADREG".into(), vec![Bit::One]),
        ("PLLWAKESYNC".into(), vec![Bit::Zero]),
        ("ENCLKOP".into(), vec![Bit::Zero]),
        ("LOCK".into(), vec![Bit::Wire(locked)]),
    ]);
    port_directions.insert(feedback_output.port().into(), "output");
    connections.insert(
        feedback_output.port().into(),
        vec![Bit::Wire(feedback_clock)],
    );
    port_directions.insert(fabric_output.port().into(), "output");
    connections.insert(fabric_output.port().into(), vec![Bit::Wire(output_clock)]);
    JsonCell {
        hide_name: 0,
        r#type: "EHXPLLL",
        parameters: parameters.clone(),
        attributes: attributes.clone(),
        port_directions,
        connections,
    }
}

#[allow(clippy::too_many_arguments)]
fn json_jtagg(
    tdo: [Bit; 2],
    tdi: u32,
    clock: u32,
    run_test_idle: [u32; 2],
    shift: u32,
    update: u32,
    reset_n: u32,
    clock_enable: [u32; 2],
    extension_register_1: bool,
    extension_register_2: bool,
) -> JsonCell {
    JsonCell {
        hide_name: 0,
        r#type: "JTAGG",
        parameters: [
            (
                "ER1".into(),
                if extension_register_1 {
                    "ENABLED"
                } else {
                    "DISABLED"
                }
                .into(),
            ),
            (
                "ER2".into(),
                if extension_register_2 {
                    "ENABLED"
                } else {
                    "DISABLED"
                }
                .into(),
            ),
        ]
        .into_iter()
        .collect(),
        attributes: BTreeMap::new(),
        port_directions: [
            ("JTDO1".into(), "input"),
            ("JTDO2".into(), "input"),
            ("JTDI".into(), "output"),
            ("JTCK".into(), "output"),
            ("JRTI1".into(), "output"),
            ("JRTI2".into(), "output"),
            ("JSHIFT".into(), "output"),
            ("JUPDATE".into(), "output"),
            ("JRSTN".into(), "output"),
            ("JCE1".into(), "output"),
            ("JCE2".into(), "output"),
        ]
        .into_iter()
        .collect(),
        connections: [
            ("JTDO1".into(), vec![tdo[0]]),
            ("JTDO2".into(), vec![tdo[1]]),
            ("JTDI".into(), vec![Bit::Wire(tdi)]),
            ("JTCK".into(), vec![Bit::Wire(clock)]),
            ("JRTI1".into(), vec![Bit::Wire(run_test_idle[0])]),
            ("JRTI2".into(), vec![Bit::Wire(run_test_idle[1])]),
            ("JSHIFT".into(), vec![Bit::Wire(shift)]),
            ("JUPDATE".into(), vec![Bit::Wire(update)]),
            ("JRSTN".into(), vec![Bit::Wire(reset_n)]),
            ("JCE1".into(), vec![Bit::Wire(clock_enable[0])]),
            ("JCE2".into(), vec![Bit::Wire(clock_enable[1])]),
        ]
        .into_iter()
        .collect(),
    }
}

fn json_trellis_io(pad: u32, fabric_output: Bit, fabric_input: u32, tristate: Bit) -> JsonCell {
    JsonCell {
        hide_name: 0,
        r#type: "TRELLIS_IO",
        parameters: [("DIR".into(), "BIDIR".into())].into_iter().collect(),
        attributes: [("PULLMODE".into(), "NONE".into())].into_iter().collect(),
        port_directions: [
            ("B".into(), "inout"),
            ("I".into(), "input"),
            ("O".into(), "output"),
            ("T".into(), "input"),
        ]
        .into_iter()
        .collect(),
        connections: [
            ("B".into(), vec![Bit::Wire(pad)]),
            ("I".into(), vec![fabric_output]),
            ("O".into(), vec![Bit::Wire(fabric_input)]),
            ("T".into(), vec![tristate]),
        ]
        .into_iter()
        .collect(),
    }
}

fn json_lut(inputs: [Bit; 4], output: u32, init: u16) -> JsonCell {
    let names = ["A", "B", "C", "D"];
    let mut connections = BTreeMap::new();
    for (name, bit) in names.into_iter().zip(inputs) {
        connections.insert(name.into(), vec![bit]);
    }
    connections.insert("Z".into(), vec![Bit::Wire(output)]);
    JsonCell {
        hide_name: 0,
        r#type: "LUT4",
        parameters: [("INIT".into(), format!("{init:016b}"))]
            .into_iter()
            .collect(),
        attributes: BTreeMap::new(),
        port_directions: [
            ("A".into(), "input"),
            ("B".into(), "input"),
            ("C".into(), "input"),
            ("D".into(), "input"),
            ("Z".into(), "output"),
        ]
        .into_iter()
        .collect(),
        connections,
    }
}

fn json_ccu2c(
    inputs: [[Bit; 4]; 2],
    carry_in: Bit,
    sums: [u32; 2],
    carry_out: u32,
    init: [u16; 2],
    inject: [bool; 2],
) -> JsonCell {
    let mut connections = BTreeMap::new();
    let mut port_directions = BTreeMap::new();
    connections.insert("CIN".into(), vec![carry_in]);
    port_directions.insert("CIN".into(), "input");
    for slice in 0..2 {
        for (port, bit) in ["A", "B", "C", "D"].into_iter().zip(inputs[slice]) {
            let name = format!("{port}{slice}");
            connections.insert(name.clone(), vec![bit]);
            port_directions.insert(name, "input");
        }
        let sum = format!("S{slice}");
        connections.insert(sum.clone(), vec![Bit::Wire(sums[slice])]);
        port_directions.insert(sum, "output");
    }
    connections.insert("COUT".into(), vec![Bit::Wire(carry_out)]);
    port_directions.insert("COUT".into(), "output");
    JsonCell {
        hide_name: 0,
        r#type: "CCU2C",
        parameters: [
            ("INIT0".into(), format!("{:016b}", init[0])),
            ("INIT1".into(), format!("{:016b}", init[1])),
            (
                "INJECT1_0".into(),
                if inject[0] { "YES" } else { "NO" }.into(),
            ),
            (
                "INJECT1_1".into(),
                if inject[1] { "YES" } else { "NO" }.into(),
            ),
        ]
        .into_iter()
        .collect(),
        attributes: BTreeMap::new(),
        port_directions,
        connections,
    }
}

fn json_flip_flop(
    data: Bit,
    output: u32,
    clock: Bit,
    edge: ClockEdge,
    enable: Option<Control>,
    reset: Option<Reset>,
) -> JsonCell {
    let parameters = [
        ("CEMUX".into(), control_mux(enable)),
        (
            "CLKMUX".into(),
            if edge == ClockEdge::Rising {
                "CLK".into()
            } else {
                "INV".into()
            },
        ),
        ("GSR".into(), "DISABLED".into()),
        (
            "LSRMUX".into(),
            reset.map_or_else(
                || "LSR".into(),
                |reset| match reset.active {
                    ActiveLevel::High => "LSR".into(),
                    ActiveLevel::Low => "INV".into(),
                },
            ),
        ),
        (
            "REGSET".into(),
            reset
                .map_or("RESET", |reset| if reset.value { "SET" } else { "RESET" })
                .into(),
        ),
        (
            "SRMODE".into(),
            reset
                .map_or("LSR_OVER_CE", |reset| {
                    if reset.asynchronous {
                        "ASYNC"
                    } else {
                        "LSR_OVER_CE"
                    }
                })
                .into(),
        ),
    ]
    .into_iter()
    .collect();
    JsonCell {
        hide_name: 0,
        r#type: "TRELLIS_FF",
        parameters,
        attributes: BTreeMap::new(),
        port_directions: [
            ("CLK".into(), "input"),
            ("LSR".into(), "input"),
            ("CE".into(), "input"),
            ("DI".into(), "input"),
            ("Q".into(), "output"),
        ]
        .into_iter()
        .collect(),
        connections: [
            ("CLK".into(), vec![clock]),
            (
                "LSR".into(),
                vec![reset.map_or(Bit::Zero, |reset| reset.signal)],
            ),
            (
                "CE".into(),
                vec![enable.map_or(Bit::One, |enable| enable.signal)],
            ),
            ("DI".into(), vec![data]),
            ("Q".into(), vec![Bit::Wire(output)]),
        ]
        .into_iter()
        .collect(),
    }
}

fn control_mux(control: Option<Control>) -> String {
    match control.map(|control| control.active) {
        None => "1 ".into(),
        Some(ActiveLevel::High) => "CE".into(),
        Some(ActiveLevel::Low) => "INV".into(),
    }
}

#[allow(clippy::too_many_arguments)]
fn json_block_ram(
    width: u8,
    write_address: [Bit; 14],
    write_data: &[Bit],
    write_enable: Control,
    read_address: [Bit; 14],
    read_data: &[u32],
    read_enable: Option<Control>,
    clock: Bit,
    edge: ClockEdge,
) -> JsonCell {
    let parameters = block_ram_parameters(width, write_enable, read_enable, edge);

    let mut port_directions = BTreeMap::new();
    let mut connections = BTreeMap::new();
    let mut input = |name: String, bit: Bit| {
        port_directions.insert(name.clone(), "input");
        connections.insert(name, vec![bit]);
    };
    for (index, bit) in write_address.into_iter().enumerate() {
        input(format!("ADA{index}"), bit);
    }
    for index in 0..18 {
        input(
            format!("DIA{index}"),
            write_data.get(index).copied().unwrap_or(Bit::Zero),
        );
    }
    input("CEA".into(), Bit::One);
    input("OCEA".into(), Bit::One);
    input("CLKA".into(), clock);
    input("WEA".into(), write_enable.signal);
    input("RSTA".into(), Bit::Zero);
    for index in 0..3 {
        input(format!("CSA{index}"), Bit::Zero);
    }
    for (index, bit) in read_address.into_iter().enumerate() {
        input(format!("ADB{index}"), bit);
    }
    for index in 0..18 {
        input(format!("DIB{index}"), Bit::Zero);
    }
    input(
        "CEB".into(),
        read_enable.map_or(Bit::One, |enable| enable.signal),
    );
    input("OCEB".into(), Bit::One);
    input("CLKB".into(), clock);
    input("WEB".into(), Bit::Zero);
    input("RSTB".into(), Bit::Zero);
    for index in 0..3 {
        input(format!("CSB{index}"), Bit::Zero);
    }
    for (index, wire) in read_data.iter().enumerate() {
        let name = format!("DOB{index}");
        port_directions.insert(name.clone(), "output");
        connections.insert(name, vec![Bit::Wire(*wire)]);
    }
    JsonCell {
        hide_name: 0,
        r#type: "DP16KD",
        parameters,
        attributes: BTreeMap::new(),
        port_directions,
        connections,
    }
}

fn block_ram_parameters(
    width: u8,
    write_enable: Control,
    read_enable: Option<Control>,
    edge: ClockEdge,
) -> BTreeMap<String, String> {
    let mut parameters = BTreeMap::from([
        ("DATA_WIDTH_A".into(), width.to_string()),
        ("DATA_WIDTH_B".into(), width.to_string()),
        ("REGMODE_A".into(), "NOREG".into()),
        ("REGMODE_B".into(), "NOREG".into()),
        ("RESETMODE".into(), "SYNC".into()),
        ("ASYNC_RESET_RELEASE".into(), "SYNC".into()),
        ("CSDECODE_A".into(), "0b000".into()),
        ("CSDECODE_B".into(), "0b000".into()),
        ("WRITEMODE_A".into(), "NORMAL".into()),
        ("WRITEMODE_B".into(), "NORMAL".into()),
        ("GSR".into(), "DISABLED".into()),
        (
            "CLKAMUX".into(),
            if edge == ClockEdge::Rising {
                "CLKA"
            } else {
                "INV"
            }
            .into(),
        ),
        (
            "CLKBMUX".into(),
            if edge == ClockEdge::Rising {
                "CLKB"
            } else {
                "INV"
            }
            .into(),
        ),
        (
            "WEAMUX".into(),
            match write_enable.active {
                ActiveLevel::High => "WEA",
                ActiveLevel::Low => "INV",
            }
            .into(),
        ),
    ]);
    parameters.insert("CEAMUX".into(), "1".into());
    parameters.insert(
        "CEBMUX".into(),
        match read_enable.map(|enable| enable.active) {
            None => "1",
            Some(ActiveLevel::High) => "CEB",
            Some(ActiveLevel::Low) => "INV",
        }
        .into(),
    );
    parameters
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;
    use std::num::NonZeroU32;

    use struo_ir::{
        ActiveLevel, ArithmeticOp, ClockEdge, ComparisonOp, EnableControl, MemoryCell, Netlist,
        RegisterCell, ResetControl,
    };

    use super::{
        ArithmeticMapping, Bit, Ecp5Cell, Ecp5Netlist, IoTimingConstraints, JtaggBinding,
        MappingOptions, NextpnrJsonError, OpenDrainIo, PllBinding, PllOutput, PortDirection,
        backward_retime_ccu2c, backward_retime_lut, carry_outs_are_point_to_point, ccu_chain_names,
        forward_retime_ccu2c, forward_retime_lut, map_once, map_to_ecp5,
        map_to_ecp5_with_constraints, map_to_ecp5_with_jtagg, map_to_ecp5_with_open_drain_ios,
        map_to_ecp5_with_options, map_to_ecp5_with_pll, mapped_cell_name, mapped_wire_fanout,
        merge_equivalent_flip_flops, replicate_high_fanout_enable_luts, split_branched_carry_outs,
        verify_mapped_equivalence_proof,
    };
    use crate::PhysicalFeedback;

    fn arithmetic_netlist(width: u32, operation: ArithmeticOp) -> Netlist {
        let mut source = Netlist::new("arithmetic");
        let width = NonZeroU32::new(width).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let result = source.add_arithmetic(operation, &lhs, &rhs).unwrap();
        source.add_output_port("result", &result).unwrap();
        source
    }

    fn combinational_io_netlist() -> Netlist {
        let mut source = Netlist::new("combinational_io");
        let inputs = source.add_input_port("inputs", NonZeroU32::new(12).unwrap());
        let result = inputs[1..]
            .iter()
            .fold(inputs[0], |value, input| source.add_and(value, *input));
        source.add_output("result", result);
        source
    }

    #[test]
    fn unconstrained_io_paths_are_not_scored_against_the_frequency_goal() {
        let source = combinational_io_netlist();
        let slow = map_to_ecp5_with_options(
            &source,
            MappingOptions {
                timing_goal_mhz: 100,
                ..MappingOptions::default()
            },
        )
        .unwrap();
        let fast = map_to_ecp5_with_options(
            &source,
            MappingOptions {
                timing_goal_mhz: 1_000,
                ..MappingOptions::default()
            },
        )
        .unwrap();

        assert_eq!(slow.cells(), fast.cells());
        assert_eq!(slow.retiming().original_overall_period_ps, 0);
        assert_eq!(fast.retiming().original_overall_period_ps, 0);
    }

    #[test]
    fn explicit_io_delays_constrain_the_named_paths() {
        let source = combinational_io_netlist();
        let constraints = IoTimingConstraints::new()
            .with_input_delay_ps("inputs", 500)
            .with_output_delay_ps("result", 750);

        let mapped = map_to_ecp5_with_constraints(
            &source,
            MappingOptions {
                timing_goal_mhz: 250,
                ..MappingOptions::default()
            },
            &constraints,
        )
        .unwrap();

        assert!(mapped.retiming().original_overall_period_ps > 1_250);
    }

    #[test]
    fn input_to_register_and_register_to_output_paths_require_io_delays() {
        let mut source = Netlist::new("sequential_io");
        let clock = source.add_input("clock");
        let input = source.add_input("input");
        let auxiliary = source.add_input("auxiliary");
        let data = source.add_and(input, auxiliary);
        let registered = source.add_register_output("registered");
        source.add_register(RegisterCell::new(
            "registered",
            registered,
            data,
            clock,
            ClockEdge::Rising,
            None,
            None,
        ));
        let result = source.add_xor(registered, auxiliary);
        source.add_output("result", result);

        let unconstrained = map_to_ecp5(&source).unwrap();
        assert_eq!(unconstrained.retiming().original_period_ps, 0);
        assert_eq!(unconstrained.retiming().original_overall_period_ps, 0);

        let constrained_inputs = IoTimingConstraints::new()
            .with_input_delay_ps("input", 400)
            .with_input_delay_ps("auxiliary", 400);
        let input_timed =
            map_to_ecp5_with_constraints(&source, MappingOptions::default(), &constrained_inputs)
                .unwrap();
        assert!(input_timed.retiming().original_period_ps > 400);

        let constrained_output = IoTimingConstraints::new().with_output_delay_ps("result", 600);
        let output_timed =
            map_to_ecp5_with_constraints(&source, MappingOptions::default(), &constrained_output)
                .unwrap();
        assert!(output_timed.retiming().original_overall_period_ps > 600);
    }

    #[test]
    fn rejects_an_io_delay_on_a_port_of_the_wrong_direction() {
        let source = combinational_io_netlist();
        let constraints = IoTimingConstraints::new().with_input_delay_ps("result", 500);

        let error = map_to_ecp5_with_constraints(&source, MappingOptions::default(), &constraints)
            .unwrap_err();

        assert!(matches!(
            error,
            super::MappingError::TimingPortDirection {
                expected: PortDirection::Input,
                actual: PortDirection::Output,
                ..
            }
        ));
    }

    #[test]
    fn automatically_selects_retiming_only_when_lut_timing_improves() {
        let mut source = Netlist::new("retimed_lut_chain");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let inputs = (0..8)
            .map(|index| source.add_input(format!("input{index}")))
            .collect::<Vec<_>>();
        let reset_control = ResetControl {
            signal: reset,
            active: ActiveLevel::High,
            asynchronous: true,
            value: false,
        };
        let registered = inputs
            .iter()
            .enumerate()
            .map(|(index, input)| {
                let name = format!("input_q{index}");
                let output = source.add_register_output(&name);
                source.add_register(RegisterCell::new(
                    name,
                    output,
                    *input,
                    clock,
                    ClockEdge::Rising,
                    None,
                    Some(reset_control),
                ));
                output
            })
            .collect::<Vec<_>>();
        let reduced = registered[1..]
            .iter()
            .fold(registered[0], |value, input| source.add_and(value, *input));
        let output = source.add_register_output("result_q");
        source.add_register(RegisterCell::new(
            "result_q",
            output,
            reduced,
            clock,
            ClockEdge::Rising,
            None,
            Some(reset_control),
        ));
        source.add_output("result", output);

        let mapped = map_to_ecp5(&source).unwrap();

        assert!(mapped.retiming().applied, "{:?}", mapped.retiming());
        assert!(mapped.retiming().equivalence_signed_off);
        assert!(mapped.retiming().certified_primitive_moves > 0);
        assert!(
            mapped.retiming().selected_lut_depth < mapped.retiming().original_lut_depth,
            "{:?}",
            mapped.retiming()
        );
    }

    #[test]
    fn folds_constant_flip_flops_that_gsr_initialization_makes_constant() {
        let mut source = Netlist::new("constant_ff");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let zero = source.add_constant(false);
        let one = source.add_constant(true);

        // No reset: GSR loads the configured value, so the flip-flop output
        // equals its constant data from configuration onward.
        let unreset_zero_q = source.add_register_output("unreset_zero_q");
        source.add_register(RegisterCell::new(
            "unreset_zero_q",
            unreset_zero_q,
            zero,
            clock,
            ClockEdge::Rising,
            None,
            None,
        ));
        source.add_output("unreset_zero", unreset_zero_q);
        let unreset_one_q = source.add_register_output("unreset_one_q");
        source.add_register(RegisterCell::new(
            "unreset_one_q",
            unreset_one_q,
            one,
            clock,
            ClockEdge::Rising,
            None,
            None,
        ));
        source.add_output("unreset_one", unreset_one_q);

        // Reset asserting the same constant also stays constant forever.
        let matched_reset_q = source.add_register_output("matched_reset_q");
        source.add_register(RegisterCell::new(
            "matched_reset_q",
            matched_reset_q,
            zero,
            clock,
            ClockEdge::Rising,
            None,
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::High,
                asynchronous: true,
                value: false,
            }),
        ));
        source.add_output("matched_reset", matched_reset_q);

        // A reset asserting the opposite value forces the output away from
        // the constant after the first clock edge, so it must stay a flip-flop.
        let mismatched_reset_q = source.add_register_output("mismatched_reset_q");
        source.add_register(RegisterCell::new(
            "mismatched_reset_q",
            mismatched_reset_q,
            zero,
            clock,
            ClockEdge::Rising,
            None,
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::High,
                asynchronous: true,
                value: true,
            }),
        ));
        source.add_output("mismatched_reset", mismatched_reset_q);

        let (mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        let flip_flops = mapped
            .cells
            .iter()
            .filter(|cell| matches!(cell, Ecp5Cell::FlipFlop { .. }))
            .count();
        assert_eq!(flip_flops, 1, "only the mismatched-reset register remains");

        let bits = |name: &str| {
            mapped
                .ports
                .iter()
                .find(|port| port.name == name)
                .unwrap_or_else(|| panic!("port `{name}` missing"))
                .bits
                .clone()
        };
        assert_eq!(bits("unreset_zero"), vec![Bit::Zero]);
        assert_eq!(bits("unreset_one"), vec![Bit::One]);
        assert_eq!(bits("matched_reset"), vec![Bit::Zero]);
        assert!(matches!(bits("mismatched_reset")[0], Bit::Wire(_)));
    }

    /// Chain a -> b -> c where b's carry-out branches into c and d.
    fn branched_carry_netlist() -> Ecp5Netlist {
        let (base, _) = map_once(
            &arithmetic_netlist(8, ArithmeticOp::Add),
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        let mut netlist = base;
        netlist.cells.clear();
        netlist.ports.clear();
        netlist.equivalence_proof.equivalent_logic_replications = 0;
        let constant = Bit::Zero;
        let slice = |name: &str, cin: Bit, cout: u32, first: u32| Ecp5Cell::Ccu2c {
            name: name.into(),
            inputs: [[constant; 4]; 2],
            carry_in: cin,
            sums: [first, first + 1],
            carry_out: cout,
            init: [0xaaaa, 0xaaaa],
            inject: [false, false],
        };
        netlist.cells.push(slice("a", constant, 120, 110));
        netlist.cells.push(slice("b", Bit::Wire(120), 121, 112));
        netlist.cells.push(slice("c", Bit::Wire(121), 122, 114));
        netlist.cells.push(slice("d", Bit::Wire(121), 123, 116));
        netlist
    }

    #[test]
    fn splits_branched_carry_outs_until_chains_are_point_to_point() {
        let mut netlist = branched_carry_netlist();

        assert!(!carry_outs_are_point_to_point(&netlist));

        let replicas = split_branched_carry_outs(&mut netlist);

        // The branch at b is repaired by cloning b for d, which branches a;
        // the cascade terminates once a is cloned for the replica of b.
        assert_eq!(replicas, 2);
        assert!(carry_outs_are_point_to_point(&netlist));
        assert_eq!(
            netlist.equivalence_proof.equivalent_logic_replications,
            replicas
        );

        // Two independent point-to-point chains now cover the six slices:
        // the originals feed c, and one full replica chain feeds d.
        let slices = netlist
            .cells
            .iter()
            .filter_map(|cell| match cell {
                Ecp5Cell::Ccu2c {
                    name,
                    carry_in,
                    carry_out,
                    init,
                    ..
                } => Some((name.clone(), *carry_in, *carry_out, *init)),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(slices.len(), 6);

        let consumers_of = |wire: u32| {
            slices
                .iter()
                .filter(|(_, carry_in, ..)| *carry_in == Bit::Wire(wire))
                .map(|(name, ..)| name.clone())
                .collect::<Vec<_>>()
        };
        let cout = |name: &str| match slices.iter().find(|(candidate, ..)| candidate == name) {
            Some(&(_, _, carry_out, _)) => carry_out,
            None => panic!("slice {name} missing"),
        };
        let init_of = |name: &str| match slices.iter().find(|(candidate, ..)| candidate == name) {
            Some(&(.., init)) => init,
            None => panic!("slice {name} missing"),
        };

        assert_eq!(consumers_of(cout("a")), vec!["b".to_owned()]);
        assert_eq!(consumers_of(cout("b")), vec!["c".to_owned()]);

        let producer_of = |wire: u32| {
            slices
                .iter()
                .find(|&(_, _, carry_out, _)| *carry_out == wire)
                .unwrap_or_else(|| panic!("no producer drives wire {wire}"))
        };

        let slice_by_name = |name: &str| slices.iter().find(|(candidate, ..)| candidate == name);

        // d must be fed by a replica of b whose own carry-in is fed by a
        // replica of a rooted at an external constant, never by the original
        // a or b (which would re-create the branch one level up).
        let d_cin = slice_by_name("d").map(|&(_, cin, ..)| cin).unwrap();
        let Bit::Wire(feeder_wire) = d_cin else {
            panic!("d lost its carry connection");
        };
        let (feeder_name, _, _, _) = producer_of(feeder_wire);
        assert_ne!(feeder_name, "b");
        assert_eq!(init_of(feeder_name), init_of("b"));

        let feeder_cin = slice_by_name(feeder_name).map(|&(_, cin, ..)| cin).unwrap();
        let Bit::Wire(root_wire) = feeder_cin else {
            panic!("expected the replica chain to continue to a root");
        };
        let (root_name, root_cin, _, _) = producer_of(root_wire);
        assert_ne!(root_name, "a");
        assert_eq!(init_of(root_name), init_of("a"));
        assert_eq!(*root_cin, Bit::Zero);

        assert_eq!(split_branched_carry_outs(&mut netlist), 0);
        assert!(carry_outs_are_point_to_point(&netlist));
    }

    #[test]
    fn treats_dead_end_carry_outs_as_point_to_point() {
        let mut netlist = branched_carry_netlist();
        netlist.cells.clear();
        netlist.cells.push(Ecp5Cell::Ccu2c {
            name: "lone".into(),
            inputs: [[Bit::Zero; 4]; 2],
            carry_in: Bit::Zero,
            sums: [210, 211],
            carry_out: 220,
            init: [0xaaaa, 0xaaaa],
            inject: [false, false],
        });
        assert!(carry_outs_are_point_to_point(&netlist));
        assert_eq!(split_branched_carry_outs(&mut netlist), 0);
    }

    #[test]
    fn mapped_equivalence_signoff_rejects_an_invalid_final_netlist() {
        let source = arithmetic_netlist(8, ArithmeticOp::Add);
        let (mut mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        assert!(verify_mapped_equivalence_proof(&mapped, false));

        let duplicate = mapped.cells[0].clone();
        mapped.cells.push(duplicate);

        assert!(!verify_mapped_equivalence_proof(&mapped, false));
    }

    #[test]
    fn replicates_high_fanout_enable_logic_once() {
        let mut source = Netlist::new("replicated_enable");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let enable_lhs = source.add_input("enable_lhs");
        let enable_rhs = source.add_input("enable_rhs");
        let enable = source.add_and(enable_lhs, enable_rhs);
        let data = source.add_input("data");
        for index in 0..10 {
            let name = format!("value{index}");
            let output = source.add_register_output(&name);
            source.add_register(RegisterCell::new(
                name.clone(),
                output,
                data,
                clock,
                ClockEdge::Rising,
                Some(EnableControl {
                    signal: enable,
                    active: ActiveLevel::High,
                }),
                Some(ResetControl {
                    signal: reset,
                    active: ActiveLevel::High,
                    asynchronous: true,
                    value: false,
                }),
            ));
            source.add_output(format!("output{index}"), output);
        }
        let (mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();

        let replicated = replicate_high_fanout_enable_luts(&mapped, 5);

        assert_eq!(
            replicated.equivalence_proof.equivalent_logic_replications,
            1
        );
        assert!(verify_mapped_equivalence_proof(&replicated, true));
        assert!(replicated.cells.iter().all(|cell| {
            let Ecp5Cell::FlipFlop {
                enable: Some(enable),
                ..
            } = cell
            else {
                return true;
            };
            let Bit::Wire(wire) = enable.signal else {
                return true;
            };
            mapped_wire_fanout(&replicated, wire) <= 5
        }));
    }

    #[test]
    fn physical_feedback_replicates_only_the_violating_lut_branch() {
        let mut source = Netlist::new("physical_feedback");
        let clock = source.add_input("clock");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let data = source.add_and(lhs, rhs);
        for index in 0..2 {
            let name = format!("value{index}");
            let output = source.add_register_output(&name);
            source.add_register(RegisterCell::new(
                name.clone(),
                output,
                data,
                clock,
                ClockEdge::Rising,
                None,
                None,
            ));
            source.add_output(format!("output{index}"), output);
        }
        let (mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        let (driver, output) = mapped
            .cells
            .iter()
            .find_map(|cell| match cell {
                Ecp5Cell::Lut4 { name, output, .. }
                    if mapped_wire_fanout(&mapped, *output) == 2 =>
                {
                    Some((name.clone(), *output))
                }
                _ => None,
            })
            .unwrap();
        let sinks = mapped
            .cells
            .iter()
            .filter_map(|cell| match cell {
                Ecp5Cell::FlipFlop {
                    name,
                    data: Bit::Wire(wire),
                    ..
                } if *wire == output => Some(name.clone()),
                _ => None,
            })
            .collect::<Vec<_>>();
        let report = format!(
            r#"{{"fmax":{{"clock":{{"achieved":319.0,"constraint":320.0}}}},"detailed_net_timings":[{{"driver":"{driver}","net":"critical","endpoints":[{{"cell":"{}","port":"DI","delay":4.0,"budget":3.0}},{{"cell":"{}","port":"DI","delay":2.0,"budget":3.0}}]}}]}}"#,
            sinks[0], sinks[1]
        );
        let placed = format!(
            r#"{{"modules":{{"physical_feedback":{{"cells":{{"{driver}":{{"attributes":{{"NEXTPNR_BEL":"X1/Y1/SLICEA.K0"}}}},"{}":{{"attributes":{{"NEXTPNR_BEL":"X8/Y8/SLICEA.FF0"}}}},"{}":{{"attributes":{{"NEXTPNR_BEL":"X2/Y2/SLICEA.FF1"}}}}}}}}}}}}"#,
            sinks[0], sinks[1]
        );
        let feedback = PhysicalFeedback::from_nextpnr_json(&report, &placed).unwrap();

        let refined = mapped.apply_physical_feedback(&feedback);

        assert_eq!(refined.cells.len(), mapped.cells.len() + 1);
        assert_eq!(refined.retiming.equivalent_physical_rewires, 1);
        assert_eq!(
            refined.retiming.equivalent_logic_replications,
            mapped.retiming.equivalent_logic_replications + 1
        );
        assert!(refined.retiming.equivalence_signed_off);
        let json = refined.to_nextpnr_json().unwrap();
        assert!(json.contains("physical_replicate_"));
        assert!(json.contains("NEXTPNR_BEL"));

        let far_from_closure =
            PhysicalFeedback::from_nextpnr_json(&report.replace("319.0", "300.0"), &placed)
                .unwrap();
        assert_eq!(mapped.apply_physical_feedback(&far_from_closure), mapped);

        let incompatible = PhysicalFeedback::from_nextpnr_json(
            &report,
            &placed.replace("SLICEA.K0", "SLICEA.FF0"),
        )
        .unwrap();
        assert_eq!(mapped.apply_physical_feedback(&incompatible), mapped);

        let passing =
            PhysicalFeedback::from_nextpnr_json(&report.replace("319.0", "321.0"), &placed)
                .unwrap();
        assert_eq!(mapped.apply_physical_feedback(&passing), mapped);
    }

    #[test]
    fn physical_critical_path_guides_a_certified_cone_retime() {
        let mut source = Netlist::new("physical_retime");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let data = source.add_and(lhs, rhs);
        let output = source.add_register_output("value");
        source.add_register(RegisterCell::new(
            "value",
            output,
            data,
            clock,
            ClockEdge::Rising,
            None,
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::High,
                asynchronous: true,
                value: false,
            }),
        ));
        let result = source.add_register_output("result");
        source.add_register(RegisterCell::new(
            "result",
            result,
            output,
            clock,
            ClockEdge::Rising,
            None,
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::High,
                asynchronous: true,
                value: false,
            }),
        ));
        source.add_output("output", result);
        let io_timing = IoTimingConstraints::new()
            .with_input_delay_ps("lhs", 0)
            .with_input_delay_ps("rhs", 0);
        let (mapped, _) = map_once(&source, MappingOptions::default(), &io_timing).unwrap();
        let driver = mapped
            .cells
            .iter()
            .find_map(|cell| match cell {
                Ecp5Cell::Lut4 { name, .. } => Some(name.clone()),
                _ => None,
            })
            .unwrap();
        let sink = mapped
            .cells
            .iter()
            .find_map(|cell| match cell {
                Ecp5Cell::FlipFlop { name, .. } if name == "ff_value" => Some(name.clone()),
                _ => None,
            })
            .unwrap();
        let result_sink = mapped
            .cells
            .iter()
            .find_map(|cell| match cell {
                Ecp5Cell::FlipFlop { name, .. } if name == "ff_result" => Some(name.clone()),
                _ => None,
            })
            .unwrap();
        let report = format!(
            r#"{{"critical_paths":[{{"from":"posedge clk","path":[{{"delay":3.2,"from":{{"cell":"source_ff"}},"to":{{"cell":"{driver}"}}}},{{"delay":0.1,"from":{{"cell":"{driver}"}},"to":{{"cell":"{sink}"}}}}],"to":"posedge clk"}}],"fmax":{{"clock":{{"achieved":310.0,"constraint":320.0}}}}}}"#
        );
        let placed = format!(
            r#"{{"modules":{{"physical_retime":{{"cells":{{"{driver}":{{"attributes":{{"NEXTPNR_BEL":"X1/Y1/SLICEA.K0"}}}},"{sink}":{{"attributes":{{"NEXTPNR_BEL":"X2/Y2/SLICEA.FF0"}}}},"{result_sink}":{{"attributes":{{"NEXTPNR_BEL":"X3/Y3/SLICEA.FF1"}}}}}}}}}}}}"#
        );
        let feedback = PhysicalFeedback::from_nextpnr_json(&report, &placed).unwrap();

        let refined = mapped.apply_physical_feedback(&feedback);

        assert_eq!(
            refined.retiming.certified_primitive_moves,
            mapped.retiming.certified_primitive_moves + 1
        );
        assert_eq!(refined.retiming.equivalent_physical_rewires, 0);
        assert!(refined.retiming.applied);
        assert!(refined.retiming.equivalence_signed_off);
        assert!(
            refined
                .cells
                .iter()
                .any(|cell| mapped_cell_name(cell).starts_with("retime_ff_value"))
        );
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn physical_feedback_emits_bounded_opposite_direction_candidates() {
        let mut source = Netlist::new("physical_candidates");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let reset_control = ResetControl {
            signal: reset,
            active: ActiveLevel::High,
            asynchronous: true,
            value: false,
        };
        let lhs_q = source.add_register_output("lhs_q");
        source.add_register(RegisterCell::new(
            "lhs_q",
            lhs_q,
            lhs,
            clock,
            ClockEdge::Rising,
            None,
            Some(reset_control),
        ));
        let rhs_q = source.add_register_output("rhs_q");
        source.add_register(RegisterCell::new(
            "rhs_q",
            rhs_q,
            rhs,
            clock,
            ClockEdge::Rising,
            None,
            Some(reset_control),
        ));
        let data = source.add_and(lhs_q, rhs_q);
        let result_q = source.add_register_output("result_q");
        source.add_register(RegisterCell::new(
            "result_q",
            result_q,
            data,
            clock,
            ClockEdge::Rising,
            None,
            Some(reset_control),
        ));
        source.add_output("result", result_q);
        let (mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        let lut = mapped
            .cells
            .iter()
            .find_map(|cell| match cell {
                Ecp5Cell::Lut4 { name, .. } => Some(name.clone()),
                _ => None,
            })
            .unwrap();
        let sink = mapped
            .cells
            .iter()
            .find_map(|cell| match cell {
                Ecp5Cell::FlipFlop { name, .. } if name == "ff_result_q" => Some(name.clone()),
                _ => None,
            })
            .unwrap();
        let placed_cells = mapped
            .cells
            .iter()
            .enumerate()
            .map(|(index, cell)| {
                let bel = match cell {
                    Ecp5Cell::Lut4 { .. } => format!("X{index}/Y1/SLICEA.K0"),
                    Ecp5Cell::FlipFlop { .. } => format!("X{index}/Y1/SLICEA.FF0"),
                    Ecp5Cell::Ccu2c { .. }
                    | Ecp5Cell::BlockRam { .. }
                    | Ecp5Cell::TrellisIo { .. }
                    | Ecp5Cell::Jtagg { .. }
                    | Ecp5Cell::Pll { .. } => unreachable!(),
                };
                format!(
                    r#""{}":{{"attributes":{{"NEXTPNR_BEL":"{bel}"}}}}"#,
                    mapped_cell_name(cell)
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let report = format!(
            r#"{{"critical_paths":[{{"from":"posedge clk","path":[{{"delay":2.8,"from":{{"cell":"ff_lhs_q"}},"to":{{"cell":"{lut}"}}}},{{"delay":0.4,"from":{{"cell":"{lut}"}},"to":{{"cell":"{sink}"}}}}],"to":"posedge clk"}}],"fmax":{{"clock":{{"achieved":310.0,"constraint":320.0}}}}}}"#
        );
        let placed = [
            r#"{"modules":{"physical_candidates":{"cells":{"#,
            &placed_cells,
            r"}}}}",
        ]
        .concat();
        let feedback = PhysicalFeedback::from_nextpnr_json(&report, &placed).unwrap();

        let candidates = mapped.physical_feedback_candidates(&feedback);

        assert_eq!(candidates.len(), 2);
        assert_ne!(candidates[0], candidates[1]);
        assert!(candidates.iter().all(|candidate| {
            candidate.retiming.applied
                && candidate.retiming.equivalence_signed_off
                && verify_mapped_equivalence_proof(candidate, true)
        }));
        assert!(candidates.iter().any(|candidate| {
            candidate
                .cells
                .iter()
                .any(|cell| mapped_cell_name(cell).starts_with("forward_ff_"))
        }));
    }

    #[test]
    fn forward_lut_retiming_moves_a_registered_input_cut_to_the_output() {
        let mut source = Netlist::new("forward_lut");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let reset_control = ResetControl {
            signal: reset,
            active: ActiveLevel::High,
            asynchronous: true,
            value: false,
        };
        let lhs_q = source.add_register_output("lhs_q");
        source.add_register(RegisterCell::new(
            "lhs_q",
            lhs_q,
            lhs,
            clock,
            ClockEdge::Rising,
            None,
            Some(reset_control),
        ));
        let rhs_q = source.add_register_output("rhs_q");
        source.add_register(RegisterCell::new(
            "rhs_q",
            rhs_q,
            rhs,
            clock,
            ClockEdge::Rising,
            None,
            Some(reset_control),
        ));
        let result = source.add_and(lhs_q, rhs_q);
        source.add_output("result", result);
        let (mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        let lut = mapped
            .cells
            .iter()
            .position(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
            .unwrap();

        let retimed = forward_retime_lut(&mapped, lut).unwrap();

        assert_eq!(
            retimed
                .cells
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::FlipFlop { .. }))
                .count(),
            1
        );
        assert_eq!(retimed.equivalence_proof.certified_primitive_moves, 1);
        assert!(verify_mapped_equivalence_proof(&retimed, true));
        assert!(retimed.cells.iter().any(|cell| {
            matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name.starts_with("forward_ff_"))
        }));
    }

    #[test]
    fn backward_lut_retiming_derives_nonzero_input_reset() {
        let mut source = Netlist::new("retimed_not");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let input = source.add_input("input");
        let inverted = source.add_not(input);
        let output = source.add_register_output("result_q");
        source.add_register(RegisterCell::new(
            "result_q",
            output,
            inverted,
            clock,
            ClockEdge::Rising,
            None,
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::High,
                asynchronous: true,
                value: false,
            }),
        ));
        source.add_output("result", output);
        let (mut mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        mapped.cells.push(Ecp5Cell::Lut4 {
            name: "retime_dead_lut".into(),
            inputs: [Bit::Zero; 4],
            output: 10_000,
            init: 0,
        });
        let register = mapped
            .cells
            .iter()
            .position(
                |cell| matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name == "ff_result_q"),
            )
            .unwrap();

        let retimed = backward_retime_lut(&mapped, register).unwrap();

        assert!(retimed.cells.iter().any(|cell| {
            matches!(
                cell,
                Ecp5Cell::FlipFlop {
                    name,
                    reset: Some(super::Reset { value: true, .. }),
                    ..
                } if name.starts_with("retime_ff_result_q_")
            )
        }));
        assert!(!retimed.cells.iter().any(|cell| {
            matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name == "ff_result_q")
        }));
        assert!(
            !retimed.cells.iter().any(
                |cell| matches!(cell, Ecp5Cell::Lut4 { name, .. } if name == "retime_dead_lut")
            )
        );
        let outputs = retimed
            .cells
            .iter()
            .flat_map(super::cell_output_bits)
            .filter_map(|bit| match bit {
                Bit::Wire(wire) => Some(wire),
                Bit::Zero | Bit::One => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(
            outputs.len(),
            outputs.iter().copied().collect::<HashSet<_>>().len(),
            "retiming must not reuse the removed maximum Q wire"
        );
    }

    #[test]
    fn backward_lut_retimings_keep_cell_names_unique_across_replicas() {
        let mut source = Netlist::new("shared_lut_retime");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let and = source.add_and(lhs, rhs);
        let reset_control = ResetControl {
            signal: reset,
            active: ActiveLevel::High,
            asynchronous: true,
            value: false,
        };
        for name in ["a_q", "b_q"] {
            let output = source.add_register_output(name);
            source.add_register(RegisterCell::new(
                name,
                output,
                and,
                clock,
                ClockEdge::Rising,
                None,
                Some(reset_control),
            ));
            source.add_output(name, output);
        }
        let (mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        let register = |netlist: &Ecp5Netlist, name: &str| {
            netlist
                .cells
                .iter()
                .position(|cell| matches!(cell, Ecp5Cell::FlipFlop { name: cell_name, .. } if cell_name == name))
                .unwrap()
        };
        let first = backward_retime_lut(&mapped, register(&mapped, "ff_a_q")).unwrap();
        let second = backward_retime_lut(&first, register(&first, "ff_b_q")).unwrap();

        let names = second
            .cells
            .iter()
            .map(mapped_cell_name)
            .collect::<HashSet<_>>();
        assert_eq!(
            names.len(),
            second.cells.len(),
            "every retimed cell must keep a unique name"
        );
        let json = second.to_nextpnr_json().unwrap();
        assert_eq!(
            serde_json::from_str::<serde_json::Value>(&json).unwrap()["modules"][second.name()]
                ["cells"]
                .as_object()
                .unwrap()
                .len(),
            second.cells.len(),
            "serialization must preserve every cell"
        );
    }

    #[test]
    fn nextpnr_json_rejects_duplicate_cell_names() {
        let mut source = Netlist::new("duplicated_cell");
        let clock = source.add_input("clock");
        let input = source.add_input("input");
        let output_net = source.add_register_output("result_q");
        source.add_register(RegisterCell::new(
            "result_q",
            output_net,
            input,
            clock,
            ClockEdge::Rising,
            None,
            None,
        ));
        source.add_output("result", output_net);
        let (mut mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        let mut duplicate = mapped.cells[0].clone();
        if let Ecp5Cell::FlipFlop { name, output, .. } = &mut duplicate {
            *name = mapped_cell_name(&mapped.cells[0]).to_owned();
            *output = output.checked_add(1).unwrap();
        }
        mapped.cells.push(duplicate);

        assert!(matches!(
            mapped.to_nextpnr_json(),
            Err(NextpnrJsonError::DuplicateCellName { .. })
        ));
    }

    #[test]
    fn backward_ccu_retiming_splits_a_carry_chain_with_a_certificate() {
        let mut source = Netlist::new("retimed_carry");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let lhs = source.add_input_port("lhs", NonZeroU32::new(8).unwrap());
        let rhs = source.add_input_port("rhs", NonZeroU32::new(8).unwrap());
        let sum = source
            .add_arithmetic(ArithmeticOp::Add, &lhs, &rhs)
            .unwrap();
        let output = source.add_register_output("result_q");
        source.add_register(RegisterCell::new(
            "result_q",
            output,
            sum[7],
            clock,
            ClockEdge::Rising,
            None,
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::High,
                asynchronous: true,
                value: false,
            }),
        ));
        source.add_output("result", output);
        let (mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        let register = mapped
            .cells
            .iter()
            .position(
                |cell| matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name == "ff_result_q"),
            )
            .unwrap();
        let Ecp5Cell::FlipFlop {
            output: register_output,
            ..
        } = mapped.cells[register]
        else {
            unreachable!()
        };

        let retimed = backward_retime_ccu2c(&mapped, register).unwrap();

        assert!(!retimed.cells.iter().any(|cell| {
            matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name == "ff_result_q")
        }));
        assert!(retimed.cells.iter().any(|cell| {
            matches!(
                cell,
                Ecp5Cell::Ccu2c { name, sums, .. }
                    if name.starts_with("retime_ccu_") && sums[1] == register_output
            )
        }));
        assert!(
            retimed
                .cells
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name.starts_with("retime_ff_result_q_ccu_")))
                .count()
                >= 5
        );
        let outputs = retimed
            .cells
            .iter()
            .flat_map(super::cell_output_bits)
            .filter_map(|bit| match bit {
                Bit::Wire(wire) => Some(wire),
                Bit::Zero | Bit::One => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(
            outputs.len(),
            outputs.iter().copied().collect::<HashSet<_>>().len()
        );
    }

    #[test]
    fn forward_ccu_retiming_moves_operand_registers_across_the_whole_chain() {
        let mut source = Netlist::new("forward_retimed_carry");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let lhs = source.add_input_port("lhs", NonZeroU32::new(8).unwrap());
        let rhs = source.add_input_port("rhs", NonZeroU32::new(8).unwrap());
        let reset_control = ResetControl {
            signal: reset,
            active: ActiveLevel::High,
            asynchronous: true,
            value: false,
        };
        let registered = lhs
            .iter()
            .chain(&rhs)
            .enumerate()
            .map(|(index, input)| {
                let name = format!("operand_q{index}");
                let output = source.add_register_output(&name);
                source.add_register(RegisterCell::new(
                    name,
                    output,
                    *input,
                    clock,
                    ClockEdge::Rising,
                    None,
                    Some(reset_control),
                ));
                output
            })
            .collect::<Vec<_>>();
        let sum = source
            .add_arithmetic(ArithmeticOp::Add, &registered[..8], &registered[8..])
            .unwrap();
        source.add_output_port("sum", &sum).unwrap();
        let (mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();

        let mut retimed = mapped.clone();
        for name in &ccu_chain_names(&mapped)[0] {
            let index = retimed
                .cells
                .iter()
                .position(|cell| super::mapped_cell_name(cell) == name)
                .unwrap();
            retimed = forward_retime_ccu2c(&retimed, index).unwrap();
        }

        assert_eq!(
            retimed
                .cells
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::FlipFlop { .. }))
                .count(),
            8
        );
        assert!(retimed.cells.iter().all(|cell| {
            !matches!(cell, Ecp5Cell::FlipFlop { name, .. } if name.starts_with("ff_operand_q"))
        }));
        assert_eq!(
            retimed
                .cells
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::Ccu2c { name, .. } if name.starts_with("retime_forward_ccu_")))
                .count(),
            4
        );
    }

    #[test]
    fn equivalent_retiming_registers_are_shared_without_high_fanout() {
        let mut source = Netlist::new("shared_retiming_registers");
        let clock = source.add_input("clock");
        let reset = source.add_input("reset");
        let data = source.add_input("data");
        for index in 0..3 {
            let name = format!("copy{index}");
            let output = source.add_register_output(&name);
            source.add_register(RegisterCell::new(
                name.clone(),
                output,
                data,
                clock,
                ClockEdge::Rising,
                None,
                Some(ResetControl {
                    signal: reset,
                    active: ActiveLevel::High,
                    asynchronous: true,
                    value: false,
                }),
            ));
            source.add_output(format!("output{index}"), output);
        }
        let (mut mapped, _) = map_once(
            &source,
            MappingOptions::default(),
            &IoTimingConstraints::new(),
        )
        .unwrap();
        for cell in &mut mapped.cells {
            if let Ecp5Cell::FlipFlop { name, .. } = cell {
                *name = format!("retime_{name}");
            }
        }

        merge_equivalent_flip_flops(&mut mapped);

        let outputs = mapped
            .cells
            .iter()
            .filter_map(|cell| match cell {
                Ecp5Cell::FlipFlop { output, .. } => Some(*output),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(outputs.len(), 2);
        assert!(
            outputs
                .iter()
                .all(|output| mapped_wire_fanout(&mapped, *output) <= 2)
        );
    }

    #[test]
    fn maps_wide_arithmetic_to_ccu2c() {
        let mapped = map_to_ecp5(&arithmetic_netlist(8, ArithmeticOp::Add)).unwrap();
        let carries = mapped
            .cells()
            .iter()
            .filter(|cell| matches!(cell, Ecp5Cell::Ccu2c { .. }))
            .count();
        assert_eq!(carries, 4);
        assert!(mapped.cells().iter().all(|cell| {
            matches!(
                cell,
                Ecp5Cell::Ccu2c {
                    init: [0x96aa, 0x96aa],
                    inject: [false, false],
                    ..
                }
            )
        }));

        let json: serde_json::Value =
            serde_json::from_str(&mapped.to_nextpnr_json().unwrap()).unwrap();
        let cells = json["modules"]["arithmetic"]["cells"].as_object().unwrap();
        assert!(cells.values().all(|cell| cell["type"] == "CCU2C"));
        assert!(cells.values().all(|cell| {
            cell["parameters"]["INIT0"] == "1001011010101010"
                && cell["parameters"]["INJECT1_0"] == "NO"
        }));
    }

    #[test]
    fn maps_addition_carry_in_to_the_first_ccu2c() {
        let mut source = Netlist::new("add_with_carry");
        let width = NonZeroU32::new(8).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let carry = source.add_input("carry");
        let result = source.add_arithmetic_with_carry(&lhs, &rhs, carry).unwrap();
        source.add_output_port("result", &result).unwrap();

        let mapped = map_to_ecp5(&source).unwrap();
        let carry_bit = mapped
            .ports()
            .iter()
            .find(|port| port.name == "carry")
            .unwrap()
            .bits[0];
        let ccus = mapped
            .cells()
            .iter()
            .filter_map(|cell| match cell {
                Ecp5Cell::Ccu2c { carry_in, .. } => Some(*carry_in),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(ccus.len(), 4);
        assert_eq!(ccus[0], carry_bit);
    }

    #[test]
    fn maps_mixed_retained_cells_in_dependency_order() {
        let mut source = Netlist::new("mixed_words");
        let width = NonZeroU32::new(5).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let sum = source
            .add_arithmetic(ArithmeticOp::Add, &lhs, &rhs)
            .unwrap();
        let less = source
            .add_comparison(ComparisonOp::LessThanUnsigned, &sum, &rhs)
            .unwrap();
        let incremented = source
            .add_arithmetic(ArithmeticOp::Add, &[less], &[lhs[0]])
            .unwrap();
        source.add_output("result", incremented[0]);

        let mapped = map_to_ecp5(&source).unwrap();

        assert!(
            mapped
                .cells()
                .iter()
                .any(|cell| matches!(cell, Ecp5Cell::Ccu2c { .. }))
        );
    }

    #[test]
    fn lut_arithmetic_option_provides_comparison_baseline() {
        let mapped = map_to_ecp5_with_options(
            &arithmetic_netlist(8, ArithmeticOp::Subtract),
            MappingOptions {
                arithmetic: ArithmeticMapping::Lut4,
                ..MappingOptions::default()
            },
        )
        .unwrap();
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
                .count(),
            15
        );
        assert!(
            !mapped
                .cells()
                .iter()
                .any(|cell| matches!(cell, Ecp5Cell::Ccu2c { .. }))
        );
    }

    #[test]
    fn maps_boolean_nodes_to_expected_lut_truth_tables() {
        let mut source = Netlist::new("logic");
        let select = source.add_input("select");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let and = source.add_and(lhs, rhs);
        let or = source.add_or(lhs, rhs);
        let xor = source.add_xor(lhs, rhs);
        let not = source.add_not(lhs);
        let mux = source.add_mux(select, lhs, rhs);
        source.add_output("and", and);
        source.add_output("or", or);
        source.add_output("xor", xor);
        source.add_output("not", not);
        source.add_output("mux", mux);
        let mapped = map_to_ecp5(&source).unwrap();

        let truth_tables = mapped
            .cells()
            .iter()
            .filter_map(|cell| match cell {
                Ecp5Cell::Lut4 { init, .. } => Some(*init),
                Ecp5Cell::Ccu2c { .. }
                | Ecp5Cell::FlipFlop { .. }
                | Ecp5Cell::BlockRam { .. }
                | Ecp5Cell::TrellisIo { .. }
                | Ecp5Cell::Jtagg { .. }
                | Ecp5Cell::Pll { .. } => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(truth_tables, [0x8888, 0xeeee, 0x6666, 0x5555, 0xd8d8]);
    }

    #[test]
    fn collapses_a_four_input_cone_into_one_lut() {
        let mut source = Netlist::new("four_input");
        let a = source.add_input("a");
        let b = source.add_input("b");
        let c = source.add_input("c");
        let d = source.add_input("d");
        let ab = source.add_and(a, b);
        let cd = source.add_and(c, d);
        let result = source.add_or(ab, cd);
        source.add_output("y", result);

        let mapped = map_to_ecp5(&source).unwrap();
        let luts = mapped
            .cells()
            .iter()
            .filter_map(|cell| match cell {
                Ecp5Cell::Lut4 { inputs, init, .. } => Some((*inputs, *init)),
                Ecp5Cell::Ccu2c { .. }
                | Ecp5Cell::FlipFlop { .. }
                | Ecp5Cell::BlockRam { .. }
                | Ecp5Cell::TrellisIo { .. }
                | Ecp5Cell::Jtagg { .. }
                | Ecp5Cell::Pll { .. } => None,
            })
            .collect::<Vec<_>>();

        assert_eq!(
            luts,
            [(
                [Bit::Wire(2), Bit::Wire(3), Bit::Wire(4), Bit::Wire(5)],
                0xf888
            )]
        );
    }

    #[test]
    fn maps_a_five_input_cone_to_two_luts() {
        let mut source = Netlist::new("five_input");
        let inputs = source.add_input_port("a", NonZeroU32::new(5).unwrap());
        let lower = source.add_and(inputs[0], inputs[1]);
        let upper = source.add_and(inputs[2], inputs[3]);
        let four_input = source.add_or(lower, upper);
        let result = source.add_xor(four_input, inputs[4]);
        source.add_output("y", result);

        let mapped = map_to_ecp5(&source).unwrap();
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
                .count(),
            2
        );
    }

    #[test]
    fn recovers_area_by_reusing_an_already_mapped_cone() {
        let mut source = Netlist::new("shared_cone");
        let input_a = source.add_input("a");
        let input_b = source.add_input("b");
        let input_c = source.add_input("c");
        let input_d = source.add_input("d");
        let input_x = source.add_input("x");
        let shared = source.add_and(input_a, input_b);
        let other = source.add_and(input_c, input_d);
        let selected = source.add_and(shared, input_x);
        let result = source.add_or(selected, other);
        source.add_output("shared", shared);
        source.add_output("result", result);

        let mapped = map_to_ecp5(&source).unwrap();

        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
                .count(),
            2
        );
    }

    #[test]
    fn omits_unreachable_boolean_nodes() {
        let mut source = Netlist::new("dead_logic");
        let a = source.add_input("a");
        let b = source.add_input("b");
        let live = source.add_and(a, b);
        let _dead = source.add_or(a, b);
        source.add_output("y", live);

        let mapped = map_to_ecp5(&source).unwrap();
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
                .count(),
            1
        );
    }

    #[test]
    fn maps_async_low_reset_to_trellis_ff_parameters() {
        let mut source = Netlist::new("counter_bit");
        let clock = source.add_input("clk");
        let reset = source.add_input("rst_n");
        let output = source.add_register_output("state");
        let next = source.add_not(output);
        source.add_register(RegisterCell::new(
            "state",
            output,
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
        source.add_output("q", output);

        let mapped = map_to_ecp5(&source).unwrap();
        let json = mapped.to_nextpnr_json().unwrap();
        assert!(json.contains("\"type\": \"TRELLIS_FF\""));
        assert!(json.contains("\"SRMODE\": \"ASYNC\""));
        assert!(json.contains("\"LSRMUX\": \"INV\""));

        let json: serde_json::Value = serde_json::from_str(&json).unwrap();
        let connections = &json["modules"]["counter_bit"]["cells"]["ff_state"]["connections"];
        assert!(connections.get("DI").is_some());
        assert!(connections.get("M").is_none());
    }

    #[test]
    fn preserves_vector_ports_for_nextpnr() {
        let mut source = Netlist::new("passthrough");
        let input = source.add_input_port("input", NonZeroU32::new(3).unwrap());
        source.add_output_port("output", &input).unwrap();

        let mapped = map_to_ecp5(&source).unwrap();
        let json: serde_json::Value =
            serde_json::from_str(&mapped.to_nextpnr_json().unwrap()).unwrap();

        assert_eq!(mapped.ports().len(), 2);
        assert_eq!(mapped.ports()[0].bits.len(), 3);
        assert_eq!(
            json["modules"]["passthrough"]["ports"]["input"]["bits"]
                .as_array()
                .unwrap()
                .len(),
            3
        );
        assert_eq!(
            json["modules"]["passthrough"]["ports"]["output"]["bits"]
                .as_array()
                .unwrap()
                .len(),
            3
        );
    }

    #[test]
    fn binds_scalar_top_ports_to_jtagg() {
        let mut source = Netlist::new("debug_top");
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
            source.add_input(name);
        }
        let zero = source.add_constant(false);
        source.add_output("jtag_tdo1", zero);
        source.add_output("jtag_tdo2", zero);
        let mut binding = JtaggBinding::with_prefix("jtag");
        binding.extension_register_2 = false;

        let mapped = map_to_ecp5_with_jtagg(&source, &binding).unwrap();

        assert!(mapped.ports().is_empty());
        assert!(matches!(
            mapped.cells(),
            [Ecp5Cell::Jtagg {
                tdo: [Bit::Zero, Bit::Zero],
                extension_register_1: true,
                extension_register_2: false,
                ..
            }]
        ));
        let json: serde_json::Value =
            serde_json::from_str(&mapped.to_nextpnr_json().unwrap()).unwrap();
        let cell = &json["modules"]["debug_top"]["cells"]["jtagg"];
        assert_eq!(cell["type"], "JTAGG");
        assert_eq!(cell["parameters"]["ER1"], "ENABLED");
        assert_eq!(cell["parameters"]["ER2"], "DISABLED");
        assert_eq!(cell["connections"]["JTDO1"][0], "0");
        assert_eq!(cell["port_directions"]["JTDO1"], "input");
        assert_eq!(cell["port_directions"]["JTDI"], "output");
        assert!(
            json["modules"]["debug_top"]["ports"]
                .as_object()
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn jtagg_binding_errors_leave_the_netlist_unchanged() {
        let mut source = Netlist::new("incomplete_debug_top");
        source.add_input("jtag_tdi");
        let mut mapped = map_to_ecp5(&source).unwrap();
        let original = mapped.clone();

        assert!(
            mapped
                .bind_jtagg(&JtaggBinding::with_prefix("jtag"))
                .is_err()
        );
        assert_eq!(mapped, original);
    }

    #[test]
    fn binds_user_configured_pll_to_logical_clock_ports() {
        let mut source = Netlist::new("pll_top");
        source.add_input("clk");
        source.add_input("clk_250");
        source.add_input("pll_locked");
        let mut binding = PllBinding::new(
            "clk",
            "clk_250",
            "pll_locked",
            PllOutput::Clkos,
            PllOutput::Clkop,
        );
        binding.parameters.extend(
            [
                ("CLKI_DIV", "3"),
                ("CLKFB_DIV", "5"),
                ("CLKOP_DIV", "25"),
                ("CLKOS_DIV", "2"),
                ("FEEDBK_PATH", "CLKOP"),
            ]
            .map(|(name, value)| (name.into(), value.into())),
        );
        binding
            .attributes
            .insert("FREQUENCY_PIN_CLKI".into(), "12".into());

        let mapped = map_to_ecp5_with_pll(&source, &binding).unwrap();

        assert_eq!(
            mapped
                .ports()
                .iter()
                .map(|port| port.name.as_str())
                .collect::<Vec<_>>(),
            ["clk"]
        );
        assert!(matches!(
            mapped.cells(),
            [Ecp5Cell::Pll {
                fabric_output: PllOutput::Clkos,
                feedback_output: PllOutput::Clkop,
                ..
            }]
        ));
        let json: serde_json::Value =
            serde_json::from_str(&mapped.to_nextpnr_json().unwrap()).unwrap();
        let cell = &json["modules"]["pll_top"]["cells"]["pll"];
        assert_eq!(cell["type"], "EHXPLLL");
        assert_eq!(cell["parameters"]["CLKI_DIV"], "3");
        assert_eq!(cell["attributes"]["FREQUENCY_PIN_CLKI"], "12");
        assert_eq!(cell["connections"]["CLKFB"], cell["connections"]["CLKOP"]);
        assert_ne!(cell["connections"]["CLKOP"], cell["connections"]["CLKOS"]);
        assert_eq!(cell["port_directions"]["CLKOS"], "output");
    }

    #[test]
    fn binds_split_open_drain_interface_to_trellis_io() {
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

        assert!(
            mapped
                .ports()
                .iter()
                .all(|port| { port.name != "sda_i" && port.name != "sda_drive_low" })
        );
        let pin = mapped
            .ports()
            .iter()
            .find(|port| port.name == "sda")
            .unwrap();
        assert_eq!(pin.direction, PortDirection::Inout);
        assert_eq!(pin.bits.len(), 1);
        assert!(
            mapped
                .cells()
                .iter()
                .any(|cell| matches!(cell, Ecp5Cell::Lut4 { init: 0x5555, .. }))
        );
        let Ecp5Cell::TrellisIo {
            pad,
            fabric_output,
            fabric_input,
            tristate,
            ..
        } = mapped
            .cells()
            .iter()
            .find(|cell| matches!(cell, Ecp5Cell::TrellisIo { .. }))
            .unwrap()
        else {
            unreachable!()
        };
        assert_eq!(pin.bits, vec![Bit::Wire(*pad)]);
        assert_eq!(*fabric_output, Bit::Zero);
        assert_eq!(*fabric_input, sda_i.index() + 2);
        assert!(matches!(tristate, Bit::Wire(_)));

        let json: serde_json::Value =
            serde_json::from_str(&mapped.to_nextpnr_json().unwrap()).unwrap();
        let module = &json["modules"]["i2c_top"];
        assert_eq!(module["ports"]["sda"]["direction"], "inout");
        assert!(module["ports"].get("sda_i").is_none());
        assert!(module["ports"].get("sda_drive_low").is_none());
        let io = module["cells"]
            .as_object()
            .unwrap()
            .values()
            .find(|cell| cell["type"] == "TRELLIS_IO")
            .unwrap();
        assert_eq!(io["parameters"]["DIR"], "BIDIR");
        assert_eq!(io["connections"]["I"][0], "0");
        assert_eq!(io["port_directions"]["B"], "inout");
    }

    #[test]
    fn constant_released_open_drain_needs_no_inverter_lut() {
        let mut source = Netlist::new("released_i2c");
        source.add_input("sda_i");
        let released = source.add_constant(false);
        source.add_output("sda_drive_low", released);

        let mapped = map_to_ecp5_with_open_drain_ios(
            &source,
            &[OpenDrainIo::new("sda", "sda_i", "sda_drive_low")],
        )
        .unwrap();

        assert!(
            !mapped
                .cells()
                .iter()
                .any(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
        );
        assert!(mapped.cells().iter().any(|cell| matches!(
            cell,
            Ecp5Cell::TrellisIo {
                fabric_output: Bit::Zero,
                tristate: Bit::One,
                ..
            }
        )));
    }

    #[test]
    fn maps_synchronous_memory_to_dp16kd() {
        let mut source = Netlist::new("scratchpad");
        let clock = source.add_input("clock");
        let write_enable = source.add_input("write_enable");
        let read_address = source.add_input_port("read_address", NonZeroU32::new(8).unwrap());
        let write_address = source.add_input_port("write_address", NonZeroU32::new(8).unwrap());
        let write_data = source.add_input_port("write_data", NonZeroU32::new(8).unwrap());
        let read_data = (0..8)
            .map(|bit| source.add_memory_output(format!("words[{bit}]")))
            .collect::<Vec<_>>();
        source.add_memory(MemoryCell::new(
            "words",
            256,
            read_address,
            read_data.clone(),
            None,
            write_address,
            write_data,
            EnableControl {
                signal: write_enable,
                active: ActiveLevel::High,
            },
            clock,
            ClockEdge::Rising,
        ));
        source.add_output_port("read_data", &read_data).unwrap();

        let mapped = map_to_ecp5(&source).unwrap();
        assert!(mapped.cells().iter().any(|cell| matches!(
            cell,
            Ecp5Cell::BlockRam {
                physical_width: 9,
                depth: 256,
                word_width: 8,
                ..
            }
        )));

        let json: serde_json::Value =
            serde_json::from_str(&mapped.to_nextpnr_json().unwrap()).unwrap();
        let cell = &json["modules"]["scratchpad"]["cells"]["bram_words"];
        assert_eq!(cell["type"], "DP16KD");
        assert_eq!(cell["parameters"]["DATA_WIDTH_A"], "9");
        assert_eq!(cell["connections"]["ADA3"][0], 12);
        assert_eq!(cell["connections"]["DOB7"][0], 35);
    }
}
