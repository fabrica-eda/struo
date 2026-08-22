//! ECP5 technology mapping and nextpnr serialization.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt::{self, Display, Formatter};

use serde::Serialize;
use serde::ser::Serializer;
use struo_ir::{
    ActiveLevel, ClockEdge, MemoryCell, NetId, Netlist, NodeKind, PortDirection as IrPortDirection,
    ValidationError,
};

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
}

/// Technology-mapped ECP5 netlist used by both simulation and nextpnr.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ecp5Netlist {
    name: String,
    ports: Vec<MappedPort>,
    cells: Vec<Ecp5Cell>,
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

    /// Serializes this exact mapped object to the Yosys JSON schema consumed by
    /// nextpnr-ecp5.
    ///
    /// # Errors
    ///
    /// Returns an error if JSON serialization fails.
    pub fn to_nextpnr_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(&JsonDesign::from(self))
    }
}

/// Maps target-independent Boolean nodes and registers to ECP5 primitives.
///
/// # Errors
///
/// Returns an error if the source netlist is invalid.
pub fn map_to_ecp5(netlist: &Netlist) -> Result<Ecp5Netlist, MappingError> {
    netlist.validate()?;
    let mut mapper = LutMapper::new(netlist);

    for port in netlist
        .ports()
        .iter()
        .filter(|port| port.direction() == IrPortDirection::Output)
    {
        for output in port.bits() {
            let node = node_for(netlist, *output);
            let source = node.inputs()[0];
            let output_bit = mapper.map_net(source);
            mapper.bits[output.index() as usize] = Some(output_bit);
        }
    }

    for register in netlist.registers() {
        mapper.map_net(register.data());
        mapper.map_net(register.clock());
        if let Some(enable) = register.enable() {
            mapper.map_net(enable.signal);
        }
        if let Some(reset) = register.reset() {
            mapper.map_net(reset.signal);
        }
    }

    for memory in netlist.memories() {
        for net in memory
            .read_address()
            .iter()
            .chain(memory.write_address())
            .chain(memory.write_data())
            .copied()
            .chain([memory.clock(), memory.write_enable().signal])
            .chain(memory.read_enable().map(|enable| enable.signal))
        {
            mapper.map_net(net);
        }
    }

    for register in netlist.registers() {
        mapper.cells.push(Ecp5Cell::FlipFlop {
            // nextpnr rejects a cell whose name is also a top-level IO name.
            // Keep primitive cells in a dedicated namespace even when an RTL
            // output is directly registered.
            name: format!("ff_{}", register.name()),
            data: mapped_bit(&mapper.bits, register.data()),
            output: wire_number(register.output()),
            clock: mapped_bit(&mapper.bits, register.clock()),
            edge: register.edge(),
            enable: register.enable().map(|enable| Control {
                signal: mapped_bit(&mapper.bits, enable.signal),
                active: enable.active,
            }),
            reset: register.reset().map(|reset| Reset {
                signal: mapped_bit(&mapper.bits, reset.signal),
                active: reset.active,
                asynchronous: reset.asynchronous,
                value: reset.value,
            }),
        });
    }

    for memory in netlist.memories() {
        map_memory(memory, &mapper.bits, &mut mapper.cells)?;
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
                .map(|net| mapped_bit(&mapper.bits, *net))
                .collect(),
        })
        .collect();

    Ok(Ecp5Netlist {
        name: netlist.name().into(),
        ports,
        cells: mapper.cells,
    })
}

const LUT_INPUTS: usize = 4;
const CUT_LIMIT: usize = 64;

#[derive(Clone, Debug)]
struct LutPlan {
    leaves: Vec<NetId>,
    depth: usize,
    area: usize,
}

struct LutMapper<'a> {
    netlist: &'a Netlist,
    bits: Vec<Option<Bit>>,
    cells: Vec<Ecp5Cell>,
    plans: Vec<Option<LutPlan>>,
}

impl<'a> LutMapper<'a> {
    fn new(netlist: &'a Netlist) -> Self {
        let mut bits = vec![None; netlist.nodes().len()];
        let mut plans = vec![None; netlist.nodes().len()];

        for node in netlist.nodes() {
            let index = node.output().index() as usize;
            match node.kind() {
                NodeKind::Input(_) | NodeKind::RegisterOutput(_) | NodeKind::MemoryOutput(_) => {
                    bits[index] = Some(wire_for(node.output()));
                }
                NodeKind::Constant(value) => bits[index] = Some(Bit::from(*value)),
                NodeKind::And | NodeKind::Or | NodeKind::Xor | NodeKind::Not | NodeKind::Mux => {
                    plans[index] = Some(best_lut_plan(netlist, &plans, node.output()));
                }
                NodeKind::Output(_) => {}
            }
        }

        Self {
            netlist,
            bits,
            cells: Vec::new(),
            plans,
        }
    }

    fn map_net(&mut self, net: NetId) -> Bit {
        if let Some(bit) = self.bits[net.index() as usize] {
            return bit;
        }

        let plan = self.plans[net.index() as usize]
            .clone()
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
}

fn best_lut_plan(netlist: &Netlist, plans: &[Option<LutPlan>], root: NetId) -> LutPlan {
    enumerate_cuts(netlist, root)
        .into_iter()
        .map(|leaves| {
            let area = 1 + leaves
                .iter()
                .filter_map(|leaf| plans[leaf.index() as usize].as_ref())
                .map(|plan| plan.area)
                .sum::<usize>();
            let depth = 1 + leaves
                .iter()
                .filter_map(|leaf| plans[leaf.index() as usize].as_ref())
                .map(|plan| plan.depth)
                .max()
                .unwrap_or(0);
            LutPlan {
                leaves,
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
    /// A logical memory cannot be width-tiled into DP16KD primitives.
    UnsupportedMemoryGeometry {
        /// Memory name.
        memory: String,
        /// Requested word count.
        depth: u32,
        /// Requested word width.
        width: usize,
    },
}

impl Display for MappingError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidNetlist(error) => write!(formatter, "invalid netlist: {error}"),
            Self::UnsupportedMemoryGeometry {
                memory,
                depth,
                width,
            } => write!(
                formatter,
                "memory {memory} ({depth}x{width}) cannot be mapped to ECP5 DP16KD primitives"
            ),
        }
    }
}

impl Error for MappingError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidNetlist(error) => Some(error),
            Self::UnsupportedMemoryGeometry { .. } => None,
        }
    }
}

impl From<ValidationError> for MappingError {
    fn from(error: ValidationError) -> Self {
        Self::InvalidNetlist(error)
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
        let cells = netlist
            .cells
            .iter()
            .map(|cell| match cell {
                Ecp5Cell::Lut4 {
                    name,
                    inputs,
                    output,
                    init,
                } => (name.clone(), json_lut(*inputs, *output, *init)),
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
            })
            .collect();
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
    use std::num::NonZeroU32;

    use struo_ir::{
        ActiveLevel, ClockEdge, EnableControl, MemoryCell, Netlist, RegisterCell, ResetControl,
    };

    use super::{Bit, Ecp5Cell, map_to_ecp5};

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
                Ecp5Cell::FlipFlop { .. } | Ecp5Cell::BlockRam { .. } => None,
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
                Ecp5Cell::FlipFlop { .. } | Ecp5Cell::BlockRam { .. } => None,
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
