//! Technology-independent, bit-level circuit representation used by Struo.

use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fmt::{self, Display, Formatter};
use std::num::NonZeroU32;

/// Identifies a one-bit net within a [`Netlist`].
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct NetId(u32);

impl NetId {
    /// Returns the stable numeric index of this net.
    #[must_use]
    pub const fn index(self) -> u32 {
        self.0
    }
}

impl Display for NetId {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        write!(formatter, "n{}", self.0)
    }
}

/// A combinational node or a reserved state output in a netlist.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub enum NodeKind {
    /// A named primary input bit.
    Input(String),
    /// A constant logic value.
    Constant(bool),
    /// A two-input AND gate.
    And,
    /// A two-input OR gate.
    Or,
    /// A two-input XOR gate.
    Xor,
    /// A one-input inverter.
    Not,
    /// A one-bit mux with inputs ordered as condition, then, else.
    Mux,
    /// The Q output of a state element. Its D/control inputs live in
    /// [`RegisterCell`], allowing feedback without violating node order.
    RegisterOutput(String),
    /// A named primary output bit.
    Output(String),
    /// A named output bit driven by a synchronous memory read port.
    MemoryOutput(String),
    /// A bit produced by a retained word-level arithmetic cell.
    ArithmeticOutput(String),
    /// The result of a retained word-level comparison cell.
    ComparisonOutput(String),
}

impl NodeKind {
    const fn expected_input_count(&self) -> usize {
        match self {
            Self::Input(_)
            | Self::Constant(_)
            | Self::RegisterOutput(_)
            | Self::MemoryOutput(_)
            | Self::ArithmeticOutput(_)
            | Self::ComparisonOutput(_) => 0,
            Self::Not | Self::Output(_) => 1,
            Self::And | Self::Or | Self::Xor => 2,
            Self::Mux => 3,
        }
    }
}

/// A node and the net that it drives.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Node {
    kind: NodeKind,
    inputs: Vec<NetId>,
    output: NetId,
}

/// Direction of a grouped primary port.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum PortDirection {
    /// Input to the synthesized design.
    Input,
    /// Output from the synthesized design.
    Output,
}

/// A primary port with bits stored least-significant first.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Port {
    name: String,
    direction: PortDirection,
    bits: Vec<NetId>,
}

impl Port {
    /// Returns the source-level port name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the port direction.
    #[must_use]
    pub const fn direction(&self) -> PortDirection {
        self.direction
    }

    /// Returns port bits least-significant first.
    #[must_use]
    pub fn bits(&self) -> &[NetId] {
        &self.bits
    }
}

impl Node {
    /// Returns the node operation.
    #[must_use]
    pub const fn kind(&self) -> &NodeKind {
        &self.kind
    }

    /// Returns the input nets in port order.
    #[must_use]
    pub fn inputs(&self) -> &[NetId] {
        &self.inputs
    }

    /// Returns the net driven by this node.
    #[must_use]
    pub const fn output(&self) -> NetId {
        self.output
    }
}

/// Active clock edge for a bit-level register.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ClockEdge {
    /// Rising edge.
    Rising,
    /// Falling edge.
    Falling,
}

/// Logical assertion level for a one-bit control.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ActiveLevel {
    /// Asserted when high.
    High,
    /// Asserted when low.
    Low,
}

/// Optional register enable.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EnableControl {
    /// Enable net.
    pub signal: NetId,
    /// Logical assertion level.
    pub active: ActiveLevel,
}

/// Optional register reset.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ResetControl {
    /// Reset net.
    pub signal: NetId,
    /// Logical assertion level.
    pub active: ActiveLevel,
    /// Whether reset is asynchronous to the clock.
    pub asynchronous: bool,
    /// Value loaded into this bit while reset is asserted.
    pub value: bool,
}

/// One technology-independent, one-bit storage element.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RegisterCell {
    name: String,
    output: NetId,
    data: NetId,
    clock: NetId,
    edge: ClockEdge,
    enable: Option<EnableControl>,
    reset: Option<ResetControl>,
}

/// A retained word-level arithmetic operation.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ArithmeticOp {
    /// Wrapping unsigned/signed addition (identical at the bit level).
    Add,
    /// Wrapping two's-complement subtraction.
    Subtract,
}

/// One combinational word-level arithmetic cell.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArithmeticCell {
    name: String,
    operation: ArithmeticOp,
    lhs: Vec<NetId>,
    rhs: Vec<NetId>,
    carry_in: Option<NetId>,
    outputs: Vec<NetId>,
}

/// A retained word-level comparison operation.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ComparisonOp {
    /// Unsigned less-than.
    LessThanUnsigned,
    /// Unsigned less-than-or-equal.
    LessOrEqualUnsigned,
    /// Signed two's-complement less-than.
    LessThanSigned,
    /// Signed two's-complement less-than-or-equal.
    LessOrEqualSigned,
}

impl ComparisonOp {
    /// Returns whether the relation includes equal operands.
    #[must_use]
    pub const fn includes_equal(self) -> bool {
        matches!(self, Self::LessOrEqualUnsigned | Self::LessOrEqualSigned)
    }

    /// Returns whether operands use two's-complement signed ordering.
    #[must_use]
    pub const fn is_signed(self) -> bool {
        matches!(self, Self::LessThanSigned | Self::LessOrEqualSigned)
    }
}

/// One combinational word-level comparison cell.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComparisonCell {
    name: String,
    operation: ComparisonOp,
    lhs: Vec<NetId>,
    rhs: Vec<NetId>,
    output: NetId,
}

impl ComparisonCell {
    /// Returns the stable diagnostic name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the comparison operation.
    #[must_use]
    pub const fn operation(&self) -> ComparisonOp {
        self.operation
    }

    /// Returns left operand bits least-significant first.
    #[must_use]
    pub fn lhs(&self) -> &[NetId] {
        &self.lhs
    }

    /// Returns right operand bits least-significant first.
    #[must_use]
    pub fn rhs(&self) -> &[NetId] {
        &self.rhs
    }

    /// Returns the one-bit comparison result.
    #[must_use]
    pub const fn output(&self) -> NetId {
        self.output
    }
}

impl ArithmeticCell {
    /// Returns the stable diagnostic name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the arithmetic operation.
    #[must_use]
    pub const fn operation(&self) -> ArithmeticOp {
        self.operation
    }

    /// Returns left operand bits least-significant first.
    #[must_use]
    pub fn lhs(&self) -> &[NetId] {
        &self.lhs
    }

    /// Returns right operand bits least-significant first.
    #[must_use]
    pub fn rhs(&self) -> &[NetId] {
        &self.rhs
    }

    /// Returns the optional one-bit carry input for addition.
    #[must_use]
    pub const fn carry_in(&self) -> Option<NetId> {
        self.carry_in
    }

    /// Returns result bits least-significant first.
    #[must_use]
    pub fn outputs(&self) -> &[NetId] {
        &self.outputs
    }
}

/// A second independently clocked read/write port on a memory cell.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemoryPort {
    read_address: Vec<NetId>,
    read_data: Vec<NetId>,
    read_enable: Option<EnableControl>,
    write_address: Vec<NetId>,
    write_data: Vec<NetId>,
    write_enable: EnableControl,
    clock: NetId,
    edge: ClockEdge,
}

impl MemoryPort {
    /// Creates a fully connected synchronous read/write port.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        read_address: Vec<NetId>,
        read_data: Vec<NetId>,
        read_enable: Option<EnableControl>,
        write_address: Vec<NetId>,
        write_data: Vec<NetId>,
        write_enable: EnableControl,
        clock: NetId,
        edge: ClockEdge,
    ) -> Self {
        Self {
            read_address,
            read_data,
            read_enable,
            write_address,
            write_data,
            write_enable,
            clock,
            edge,
        }
    }

    /// Returns read-address bits least-significant first.
    #[must_use]
    pub fn read_address(&self) -> &[NetId] {
        &self.read_address
    }
    /// Returns read-data bits least-significant first.
    #[must_use]
    pub fn read_data(&self) -> &[NetId] {
        &self.read_data
    }
    /// Returns the optional read clock enable.
    #[must_use]
    pub const fn read_enable(&self) -> Option<EnableControl> {
        self.read_enable
    }
    /// Returns write-address bits least-significant first.
    #[must_use]
    pub fn write_address(&self) -> &[NetId] {
        &self.write_address
    }
    /// Returns write-data bits least-significant first.
    #[must_use]
    pub fn write_data(&self) -> &[NetId] {
        &self.write_data
    }
    /// Returns the write enable.
    #[must_use]
    pub const fn write_enable(&self) -> EnableControl {
        self.write_enable
    }
    /// Returns the port clock.
    #[must_use]
    pub const fn clock(&self) -> NetId {
        self.clock
    }
    /// Returns the active clock edge.
    #[must_use]
    pub const fn edge(&self) -> ClockEdge {
        self.edge
    }
}

/// Requested physical implementation of a retained memory.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum MemoryStyle {
    /// Let the target choose an implementation.
    #[default]
    Auto,
    /// Require embedded block RAM.
    Block,
    /// Require LUT-based distributed RAM.
    Distributed,
}

/// One memory retained for target-specific mapping.
///
/// Addresses and words are stored least-significant bit first. The read and
/// write operations within each port share a clock and edge. A latency-one
/// read samples its address on the active edge; a latency-zero read is
/// asynchronous. An optional second port has its own clock and edge.
/// Simultaneous accesses to the same address have target-specific collision
/// behavior.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemoryCell {
    name: String,
    depth: u32,
    style: MemoryStyle,
    read_latency: u8,
    read_address: Vec<NetId>,
    read_data: Vec<NetId>,
    read_enable: Option<EnableControl>,
    write_address: Vec<NetId>,
    write_data: Vec<NetId>,
    write_enable: EnableControl,
    clock: NetId,
    edge: ClockEdge,
    second_port: Option<MemoryPort>,
}

impl MemoryCell {
    /// Creates a fully connected synchronous memory cell.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: impl Into<String>,
        depth: u32,
        read_address: Vec<NetId>,
        read_data: Vec<NetId>,
        read_enable: Option<EnableControl>,
        write_address: Vec<NetId>,
        write_data: Vec<NetId>,
        write_enable: EnableControl,
        clock: NetId,
        edge: ClockEdge,
    ) -> Self {
        Self {
            name: name.into(),
            depth,
            style: MemoryStyle::Auto,
            read_latency: 1,
            read_address,
            read_data,
            read_enable,
            write_address,
            write_data,
            write_enable,
            clock,
            edge,
            second_port: None,
        }
    }

    /// Selects the requested physical memory implementation.
    #[must_use]
    pub const fn with_style(mut self, style: MemoryStyle) -> Self {
        self.style = style;
        self
    }

    /// Selects the read latency in cycles (zero for an asynchronous read).
    #[must_use]
    pub const fn with_read_latency(mut self, read_latency: u8) -> Self {
        self.read_latency = read_latency;
        self
    }

    /// Adds a second independently clocked read/write port.
    #[must_use]
    pub fn with_second_port(mut self, port: MemoryPort) -> Self {
        self.second_port = Some(port);
        self
    }

    /// Returns the stable diagnostic name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the logical number of words.
    #[must_use]
    pub const fn depth(&self) -> u32 {
        self.depth
    }

    /// Returns the requested physical implementation.
    #[must_use]
    pub const fn style(&self) -> MemoryStyle {
        self.style
    }

    /// Returns the read latency in cycles.
    #[must_use]
    pub const fn read_latency(&self) -> u8 {
        self.read_latency
    }

    /// Returns read-address bits least-significant first.
    #[must_use]
    pub fn read_address(&self) -> &[NetId] {
        &self.read_address
    }

    /// Returns read-data bits least-significant first.
    #[must_use]
    pub fn read_data(&self) -> &[NetId] {
        &self.read_data
    }

    /// Returns the optional read clock enable.
    #[must_use]
    pub const fn read_enable(&self) -> Option<EnableControl> {
        self.read_enable
    }

    /// Returns write-address bits least-significant first.
    #[must_use]
    pub fn write_address(&self) -> &[NetId] {
        &self.write_address
    }

    /// Returns write-data bits least-significant first.
    #[must_use]
    pub fn write_data(&self) -> &[NetId] {
        &self.write_data
    }

    /// Returns the write enable.
    #[must_use]
    pub const fn write_enable(&self) -> EnableControl {
        self.write_enable
    }

    /// Returns the shared clock.
    #[must_use]
    pub const fn clock(&self) -> NetId {
        self.clock
    }

    /// Returns the active clock edge.
    #[must_use]
    pub const fn edge(&self) -> ClockEdge {
        self.edge
    }

    /// Returns the optional second port.
    #[must_use]
    pub const fn second_port(&self) -> Option<&MemoryPort> {
        self.second_port.as_ref()
    }
}

impl RegisterCell {
    /// Creates a fully connected one-bit register.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        output: NetId,
        data: NetId,
        clock: NetId,
        edge: ClockEdge,
        enable: Option<EnableControl>,
        reset: Option<ResetControl>,
    ) -> Self {
        Self {
            name: name.into(),
            output,
            data,
            clock,
            edge,
            enable,
            reset,
        }
    }

    /// Returns the stable diagnostic name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the Q net.
    #[must_use]
    pub const fn output(&self) -> NetId {
        self.output
    }

    /// Returns the D net.
    #[must_use]
    pub const fn data(&self) -> NetId {
        self.data
    }

    /// Returns the clock net.
    #[must_use]
    pub const fn clock(&self) -> NetId {
        self.clock
    }

    /// Returns the active clock edge.
    #[must_use]
    pub const fn edge(&self) -> ClockEdge {
        self.edge
    }

    /// Returns the optional enable control.
    #[must_use]
    pub const fn enable(&self) -> Option<EnableControl> {
        self.enable
    }

    /// Replaces the D input and optional clock enable.
    ///
    /// Transformations must preserve the original state-transition behavior;
    /// [`Netlist::validate`] checks that the replacement nets exist.
    pub fn set_data_and_enable(&mut self, data: NetId, enable: Option<EnableControl>) {
        self.data = data;
        self.enable = enable;
    }

    /// Returns the optional reset control.
    #[must_use]
    pub const fn reset(&self) -> Option<ResetControl> {
        self.reset
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum LogicKey {
    And(NetId, NetId),
    Or(NetId, NetId),
    Xor(NetId, NetId),
    Not(NetId),
    Mux(NetId, NetId, NetId),
}

/// A flat, bit-level logic netlist.
///
/// Combinational nodes are topologically ordered. Register Q outputs are
/// reserved as source nodes, while their D/control arcs are stored separately.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Netlist {
    name: String,
    nodes: Vec<Node>,
    ports: Vec<Port>,
    registers: Vec<RegisterCell>,
    memories: Vec<MemoryCell>,
    arithmetic: Vec<ArithmeticCell>,
    comparisons: Vec<ComparisonCell>,
    next_net: u32,
    constants: [Option<NetId>; 2],
    logic_cache: HashMap<LogicKey, NetId>,
}

impl Netlist {
    /// Creates an empty netlist.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            nodes: Vec::new(),
            ports: Vec::new(),
            registers: Vec::new(),
            memories: Vec::new(),
            arithmetic: Vec::new(),
            comparisons: Vec::new(),
            next_net: 0,
            constants: [None, None],
            logic_cache: HashMap::new(),
        }
    }

    /// Returns the design name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns all nodes in topological order.
    #[must_use]
    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    /// Returns grouped primary ports in source order.
    #[must_use]
    pub fn ports(&self) -> &[Port] {
        &self.ports
    }

    /// Returns all one-bit registers.
    #[must_use]
    pub fn registers(&self) -> &[RegisterCell] {
        &self.registers
    }

    /// Returns mutable one-bit registers for synthesis transformations.
    #[must_use]
    pub fn registers_mut(&mut self) -> &mut [RegisterCell] {
        &mut self.registers
    }

    /// Returns inferred synchronous memories.
    #[must_use]
    pub fn memories(&self) -> &[MemoryCell] {
        &self.memories
    }

    /// Returns retained word-level arithmetic cells in dependency order.
    #[must_use]
    pub fn arithmetic(&self) -> &[ArithmeticCell] {
        &self.arithmetic
    }

    /// Returns retained word-level comparison cells in dependency order.
    #[must_use]
    pub fn comparisons(&self) -> &[ComparisonCell] {
        &self.comparisons
    }

    /// Adds a primary input and returns its net.
    pub fn add_input(&mut self, name: impl Into<String>) -> NetId {
        let name = name.into();
        let net = self.add_node(NodeKind::Input(name.clone()), Vec::new());
        self.ports.push(Port {
            name,
            direction: PortDirection::Input,
            bits: vec![net],
        });
        net
    }

    /// Adds a grouped primary input and returns bits least-significant first.
    pub fn add_input_port(&mut self, name: impl Into<String>, width: NonZeroU32) -> Vec<NetId> {
        let name = name.into();
        let bits = (0..width.get())
            .map(|bit| {
                self.add_node(
                    NodeKind::Input(port_bit_name(&name, width.get(), bit)),
                    Vec::new(),
                )
            })
            .collect::<Vec<_>>();
        self.ports.push(Port {
            name,
            direction: PortDirection::Input,
            bits: bits.clone(),
        });
        bits
    }

    /// Adds or reuses a constant and returns its net.
    pub fn add_constant(&mut self, value: bool) -> NetId {
        let index = usize::from(value);
        if let Some(net) = self.constants[index] {
            return net;
        }
        let net = self.add_node(NodeKind::Constant(value), Vec::new());
        self.constants[index] = Some(net);
        net
    }

    /// Reserves a register Q output so next-state logic may read it.
    pub fn add_register_output(&mut self, name: impl Into<String>) -> NetId {
        self.add_node(NodeKind::RegisterOutput(name.into()), Vec::new())
    }

    /// Reserves one memory read-data output.
    pub fn add_memory_output(&mut self, name: impl Into<String>) -> NetId {
        self.add_node(NodeKind::MemoryOutput(name.into()), Vec::new())
    }

    /// Connects a previously reserved register output to its D/control nets.
    pub fn add_register(&mut self, register: RegisterCell) {
        self.registers.push(register);
    }

    /// Connects previously reserved read outputs to a synchronous memory.
    pub fn add_memory(&mut self, memory: MemoryCell) {
        self.memories.push(memory);
    }

    /// Adds a wrapping word-level arithmetic operation.
    ///
    /// Operands and result bits are stored least-significant first and must
    /// have the same non-zero width.
    ///
    /// # Errors
    ///
    /// Returns an error for empty, mismatched, or unrepresentable widths.
    pub fn add_arithmetic(
        &mut self,
        operation: ArithmeticOp,
        lhs: &[NetId],
        rhs: &[NetId],
    ) -> Result<Vec<NetId>, ValidationError> {
        self.add_arithmetic_inner(operation, lhs, rhs, None)
    }

    /// Adds a wrapping word-level addition with a one-bit carry input.
    ///
    /// Operands and result bits are stored least-significant first and must
    /// have the same non-zero width.
    ///
    /// # Errors
    ///
    /// Returns an error for empty, mismatched, or unrepresentable widths.
    pub fn add_arithmetic_with_carry(
        &mut self,
        lhs: &[NetId],
        rhs: &[NetId],
        carry_in: NetId,
    ) -> Result<Vec<NetId>, ValidationError> {
        self.add_arithmetic_inner(ArithmeticOp::Add, lhs, rhs, Some(carry_in))
    }

    fn add_arithmetic_inner(
        &mut self,
        operation: ArithmeticOp,
        lhs: &[NetId],
        rhs: &[NetId],
        carry_in: Option<NetId>,
    ) -> Result<Vec<NetId>, ValidationError> {
        let name = format!("arith{}", self.arithmetic.len());
        if lhs.is_empty() || lhs.len() != rhs.len() {
            return Err(ValidationError::ArithmeticWidth(name));
        }
        let width =
            u32::try_from(lhs.len()).map_err(|_| ValidationError::ArithmeticWidth(name.clone()))?;
        let outputs = (0..width)
            .map(|bit| {
                self.add_node(
                    NodeKind::ArithmeticOutput(port_bit_name(&name, width, bit)),
                    Vec::new(),
                )
            })
            .collect::<Vec<_>>();
        self.arithmetic.push(ArithmeticCell {
            name,
            operation,
            lhs: lhs.to_vec(),
            rhs: rhs.to_vec(),
            carry_in,
            outputs: outputs.clone(),
        });
        Ok(outputs)
    }

    /// Adds a word-level comparison and returns its one-bit result.
    ///
    /// # Errors
    ///
    /// Returns an error for empty or mismatched operands.
    pub fn add_comparison(
        &mut self,
        operation: ComparisonOp,
        lhs: &[NetId],
        rhs: &[NetId],
    ) -> Result<NetId, ValidationError> {
        let name = format!("compare{}", self.comparisons.len());
        if lhs.is_empty() || lhs.len() != rhs.len() {
            return Err(ValidationError::ComparisonWidth(name));
        }
        let output = self.add_node(NodeKind::ComparisonOutput(name.clone()), Vec::new());
        self.comparisons.push(ComparisonCell {
            name,
            operation,
            lhs: lhs.to_vec(),
            rhs: rhs.to_vec(),
            output,
        });
        Ok(output)
    }

    /// Adds or reuses a two-input AND gate.
    pub fn add_and(&mut self, lhs: NetId, rhs: NetId) -> NetId {
        if lhs == rhs {
            return lhs;
        }
        match (self.constant_value(lhs), self.constant_value(rhs)) {
            (Some(false), _) | (_, Some(false)) => return self.add_constant(false),
            (Some(true), _) => return rhs,
            (_, Some(true)) => return lhs,
            _ => {}
        }
        let (lhs, rhs) = ordered_pair(lhs, rhs);
        self.add_cached(LogicKey::And(lhs, rhs), NodeKind::And, vec![lhs, rhs])
    }

    /// Adds or reuses a two-input OR gate.
    pub fn add_or(&mut self, lhs: NetId, rhs: NetId) -> NetId {
        if lhs == rhs {
            return lhs;
        }
        match (self.constant_value(lhs), self.constant_value(rhs)) {
            (Some(true), _) | (_, Some(true)) => return self.add_constant(true),
            (Some(false), _) => return rhs,
            (_, Some(false)) => return lhs,
            _ => {}
        }
        let (lhs, rhs) = ordered_pair(lhs, rhs);
        self.add_cached(LogicKey::Or(lhs, rhs), NodeKind::Or, vec![lhs, rhs])
    }

    /// Adds or reuses a two-input XOR gate.
    pub fn add_xor(&mut self, lhs: NetId, rhs: NetId) -> NetId {
        if lhs == rhs {
            return self.add_constant(false);
        }
        match (self.constant_value(lhs), self.constant_value(rhs)) {
            (Some(false), _) => return rhs,
            (_, Some(false)) => return lhs,
            (Some(true), _) => return self.add_not(rhs),
            (_, Some(true)) => return self.add_not(lhs),
            _ => {}
        }
        let (lhs, rhs) = ordered_pair(lhs, rhs);
        self.add_cached(LogicKey::Xor(lhs, rhs), NodeKind::Xor, vec![lhs, rhs])
    }

    /// Adds or reuses an inverter.
    pub fn add_not(&mut self, input: NetId) -> NetId {
        if let Some(value) = self.constant_value(input) {
            return self.add_constant(!value);
        }
        if let Some(Node {
            kind: NodeKind::Not,
            inputs,
            ..
        }) = self.node_for_net(input)
        {
            return inputs[0];
        }
        self.add_cached(LogicKey::Not(input), NodeKind::Not, vec![input])
    }

    /// Adds or reuses a mux.
    pub fn add_mux(&mut self, condition: NetId, then_net: NetId, else_net: NetId) -> NetId {
        if then_net == else_net {
            return then_net;
        }
        if let Some(value) = self.constant_value(condition) {
            return if value { then_net } else { else_net };
        }
        self.add_cached(
            LogicKey::Mux(condition, then_net, else_net),
            NodeKind::Mux,
            vec![condition, then_net, else_net],
        )
    }

    /// Adds a primary output.
    pub fn add_output(&mut self, name: impl Into<String>, source: NetId) -> NetId {
        let name = name.into();
        let net = self.add_node(NodeKind::Output(name.clone()), vec![source]);
        self.ports.push(Port {
            name,
            direction: PortDirection::Output,
            bits: vec![net],
        });
        net
    }

    /// Adds a grouped primary output from sources stored least-significant first.
    ///
    /// # Errors
    ///
    /// Returns an error when no source bits are supplied.
    pub fn add_output_port(
        &mut self,
        name: impl Into<String>,
        sources: &[NetId],
    ) -> Result<Vec<NetId>, ValidationError> {
        if sources.is_empty() {
            return Err(ValidationError::EmptyPortWidth);
        }
        let name = name.into();
        let width = u32::try_from(sources.len()).map_err(|_| ValidationError::PortWidthOverflow)?;
        let bits = sources
            .iter()
            .zip(0..width)
            .map(|(source, bit)| {
                self.add_node(
                    NodeKind::Output(port_bit_name(&name, width, bit)),
                    vec![*source],
                )
            })
            .collect::<Vec<_>>();
        self.ports.push(Port {
            name,
            direction: PortDirection::Output,
            bits: bits.clone(),
        });
        Ok(bits)
    }

    /// Returns a constant value when `net` is driven by a constant node.
    #[must_use]
    pub fn constant_value(&self, net: NetId) -> Option<bool> {
        match self.node_for_net(net)?.kind {
            NodeKind::Constant(value) => Some(value),
            _ => None,
        }
    }

    /// Checks structural invariants required by synthesis backends.
    ///
    /// # Errors
    ///
    /// Returns the first structural problem found in the design.
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.name.trim().is_empty() {
            return Err(ValidationError::EmptyDesignName);
        }

        let mut defined_nets = HashSet::new();
        let mut port_names = HashSet::new();
        let mut register_outputs = HashSet::new();
        let mut memory_outputs = HashSet::new();
        let mut arithmetic_outputs = HashSet::new();
        let mut comparison_outputs = HashSet::new();

        for node in &self.nodes {
            let expected = node.kind.expected_input_count();
            if node.inputs.len() != expected {
                return Err(ValidationError::WrongInputCount {
                    net: node.output,
                    expected,
                    actual: node.inputs.len(),
                });
            }
            for input in &node.inputs {
                if !defined_nets.contains(input) {
                    return Err(ValidationError::UndefinedNet(*input));
                }
            }
            if !defined_nets.insert(node.output) {
                return Err(ValidationError::MultipleDrivers(node.output));
            }
            match &node.kind {
                NodeKind::Input(name) | NodeKind::Output(name) => {
                    if name.trim().is_empty() {
                        return Err(ValidationError::EmptyPortName);
                    }
                    if !port_names.insert(name.as_str()) {
                        return Err(ValidationError::DuplicatePortName(name.clone()));
                    }
                }
                NodeKind::RegisterOutput(name) => {
                    if name.trim().is_empty() {
                        return Err(ValidationError::EmptyRegisterName);
                    }
                    register_outputs.insert(node.output);
                }
                NodeKind::MemoryOutput(name) => {
                    if name.trim().is_empty() {
                        return Err(ValidationError::EmptyMemoryName);
                    }
                    memory_outputs.insert(node.output);
                }
                NodeKind::ArithmeticOutput(name) => {
                    if name.trim().is_empty() {
                        return Err(ValidationError::EmptyArithmeticName);
                    }
                    arithmetic_outputs.insert(node.output);
                }
                NodeKind::ComparisonOutput(name) => {
                    if name.trim().is_empty() {
                        return Err(ValidationError::EmptyComparisonName);
                    }
                    comparison_outputs.insert(node.output);
                }
                _ => {}
            }
        }

        let mut grouped_port_names = HashSet::new();
        for port in &self.ports {
            if port.name.trim().is_empty() {
                return Err(ValidationError::EmptyPortName);
            }
            if port.bits.is_empty() {
                return Err(ValidationError::EmptyPortWidth);
            }
            if !grouped_port_names.insert(port.name.as_str()) {
                return Err(ValidationError::DuplicatePortName(port.name.clone()));
            }
            for bit in &port.bits {
                let node = self
                    .node_for_net(*bit)
                    .ok_or(ValidationError::UndefinedNet(*bit))?;
                let expected = match port.direction {
                    PortDirection::Input => matches!(node.kind, NodeKind::Input(_)),
                    PortDirection::Output => matches!(node.kind, NodeKind::Output(_)),
                };
                if !expected {
                    return Err(ValidationError::InvalidPortBit(*bit));
                }
            }
        }

        self.validate_registers(&defined_nets, &register_outputs)?;
        self.validate_memories(&defined_nets, &memory_outputs)?;
        self.validate_arithmetic(&defined_nets, &arithmetic_outputs)?;
        self.validate_comparisons(&defined_nets, &comparison_outputs)?;

        Ok(())
    }

    fn validate_registers(
        &self,
        defined_nets: &HashSet<NetId>,
        register_outputs: &HashSet<NetId>,
    ) -> Result<(), ValidationError> {
        let mut connected_outputs = HashSet::new();
        let mut register_names = HashSet::new();
        for register in &self.registers {
            if !register_names.insert(register.name.as_str()) {
                return Err(ValidationError::DuplicateRegisterName(
                    register.name.clone(),
                ));
            }
            if !register_outputs.contains(&register.output) {
                return Err(ValidationError::InvalidRegisterOutput(register.output));
            }
            if !connected_outputs.insert(register.output) {
                return Err(ValidationError::MultipleDrivers(register.output));
            }
            for input in [register.data, register.clock]
                .into_iter()
                .chain(register.enable.map(|enable| enable.signal))
                .chain(register.reset.map(|reset| reset.signal))
            {
                if !defined_nets.contains(&input) {
                    return Err(ValidationError::UndefinedNet(input));
                }
            }
        }
        if let Some(output) = register_outputs.difference(&connected_outputs).next() {
            return Err(ValidationError::UnconnectedRegister(*output));
        }
        Ok(())
    }

    fn validate_memories(
        &self,
        defined_nets: &HashSet<NetId>,
        memory_outputs: &HashSet<NetId>,
    ) -> Result<(), ValidationError> {
        let mut connected_memory_outputs = HashSet::new();
        let mut memory_names = HashSet::new();
        for memory in &self.memories {
            if memory.name.trim().is_empty() {
                return Err(ValidationError::EmptyMemoryName);
            }
            if !memory_names.insert(memory.name.as_str()) {
                return Err(ValidationError::DuplicateMemoryName(memory.name.clone()));
            }
            if memory.depth == 0 {
                return Err(ValidationError::ZeroMemoryDepth(memory.name.clone()));
            }
            if memory.read_address.is_empty()
                || memory.read_address.len() != memory.write_address.len()
            {
                return Err(ValidationError::MemoryAddressWidth(memory.name.clone()));
            }
            if memory.read_data.is_empty() || memory.read_data.len() != memory.write_data.len() {
                return Err(ValidationError::MemoryWordWidth(memory.name.clone()));
            }
            let address_capacity = u32::try_from(memory.read_address.len())
                .ok()
                .and_then(|shift| 1u128.checked_shl(shift))
                .unwrap_or(u128::MAX);
            if u128::from(memory.depth) > address_capacity {
                return Err(ValidationError::MemoryAddressWidth(memory.name.clone()));
            }
            for output in &memory.read_data {
                if !memory_outputs.contains(output) {
                    return Err(ValidationError::InvalidMemoryOutput(*output));
                }
                if !connected_memory_outputs.insert(*output) {
                    return Err(ValidationError::MultipleDrivers(*output));
                }
            }
            for input in memory
                .read_address
                .iter()
                .chain(&memory.write_address)
                .chain(&memory.write_data)
                .copied()
                .chain([memory.clock, memory.write_enable.signal])
                .chain(memory.read_enable.map(|enable| enable.signal))
            {
                if !defined_nets.contains(&input) {
                    return Err(ValidationError::UndefinedNet(input));
                }
            }
            if let Some(port) = &memory.second_port {
                if port.read_address.is_empty()
                    || port.read_address.len() != port.write_address.len()
                    || port.read_address.len() != memory.read_address.len()
                {
                    return Err(ValidationError::MemoryAddressWidth(memory.name.clone()));
                }
                if port.read_data.is_empty()
                    || port.read_data.len() != port.write_data.len()
                    || port.read_data.len() != memory.read_data.len()
                {
                    return Err(ValidationError::MemoryWordWidth(memory.name.clone()));
                }
                for output in &port.read_data {
                    if !memory_outputs.contains(output) {
                        return Err(ValidationError::InvalidMemoryOutput(*output));
                    }
                    if !connected_memory_outputs.insert(*output) {
                        return Err(ValidationError::MultipleDrivers(*output));
                    }
                }
                for input in port
                    .read_address
                    .iter()
                    .chain(&port.write_address)
                    .chain(&port.write_data)
                    .copied()
                    .chain([port.clock, port.write_enable.signal])
                    .chain(port.read_enable.map(|enable| enable.signal))
                {
                    if !defined_nets.contains(&input) {
                        return Err(ValidationError::UndefinedNet(input));
                    }
                }
            }
        }
        if let Some(output) = memory_outputs.difference(&connected_memory_outputs).next() {
            return Err(ValidationError::UnconnectedMemory(*output));
        }
        Ok(())
    }

    fn validate_arithmetic(
        &self,
        defined_nets: &HashSet<NetId>,
        arithmetic_outputs: &HashSet<NetId>,
    ) -> Result<(), ValidationError> {
        let mut connected_outputs = HashSet::new();
        let mut names = HashSet::new();
        for arithmetic in &self.arithmetic {
            if arithmetic.name.trim().is_empty() {
                return Err(ValidationError::EmptyArithmeticName);
            }
            if !names.insert(arithmetic.name.as_str()) {
                return Err(ValidationError::DuplicateArithmeticName(
                    arithmetic.name.clone(),
                ));
            }
            if arithmetic.lhs.is_empty()
                || arithmetic.lhs.len() != arithmetic.rhs.len()
                || arithmetic.lhs.len() != arithmetic.outputs.len()
            {
                return Err(ValidationError::ArithmeticWidth(arithmetic.name.clone()));
            }
            for input in arithmetic
                .lhs
                .iter()
                .chain(&arithmetic.rhs)
                .chain(&arithmetic.carry_in)
            {
                if !defined_nets.contains(input) {
                    return Err(ValidationError::UndefinedNet(*input));
                }
            }
            for output in &arithmetic.outputs {
                if !arithmetic_outputs.contains(output) {
                    return Err(ValidationError::InvalidArithmeticOutput(*output));
                }
                if !connected_outputs.insert(*output) {
                    return Err(ValidationError::MultipleDrivers(*output));
                }
            }
        }
        if let Some(output) = arithmetic_outputs.difference(&connected_outputs).next() {
            return Err(ValidationError::UnconnectedArithmetic(*output));
        }
        Ok(())
    }

    fn validate_comparisons(
        &self,
        defined_nets: &HashSet<NetId>,
        comparison_outputs: &HashSet<NetId>,
    ) -> Result<(), ValidationError> {
        let mut connected_outputs = HashSet::new();
        let mut names = HashSet::new();
        for comparison in &self.comparisons {
            if comparison.name.trim().is_empty() {
                return Err(ValidationError::EmptyComparisonName);
            }
            if !names.insert(comparison.name.as_str()) {
                return Err(ValidationError::DuplicateComparisonName(
                    comparison.name.clone(),
                ));
            }
            if comparison.lhs.is_empty() || comparison.lhs.len() != comparison.rhs.len() {
                return Err(ValidationError::ComparisonWidth(comparison.name.clone()));
            }
            for input in comparison.lhs.iter().chain(&comparison.rhs) {
                if !defined_nets.contains(input) {
                    return Err(ValidationError::UndefinedNet(*input));
                }
            }
            if !comparison_outputs.contains(&comparison.output) {
                return Err(ValidationError::InvalidComparisonOutput(comparison.output));
            }
            if !connected_outputs.insert(comparison.output) {
                return Err(ValidationError::MultipleDrivers(comparison.output));
            }
        }
        if let Some(output) = comparison_outputs.difference(&connected_outputs).next() {
            return Err(ValidationError::UnconnectedComparison(*output));
        }
        Ok(())
    }

    fn add_cached(&mut self, key: LogicKey, kind: NodeKind, inputs: Vec<NetId>) -> NetId {
        if let Some(net) = self.logic_cache.get(&key) {
            return *net;
        }
        let net = self.add_node(kind, inputs);
        self.logic_cache.insert(key, net);
        net
    }

    fn add_node(&mut self, kind: NodeKind, inputs: Vec<NetId>) -> NetId {
        let output = NetId(self.next_net);
        self.next_net = self
            .next_net
            .checked_add(1)
            .expect("netlist exceeded the supported net count");
        self.nodes.push(Node {
            kind,
            inputs,
            output,
        });
        output
    }

    fn node_for_net(&self, net: NetId) -> Option<&Node> {
        self.nodes.get(net.index() as usize)
    }
}

fn ordered_pair(lhs: NetId, rhs: NetId) -> (NetId, NetId) {
    if lhs <= rhs { (lhs, rhs) } else { (rhs, lhs) }
}

fn port_bit_name(name: &str, width: u32, bit: u32) -> String {
    if width == 1 {
        name.into()
    } else {
        format!("{name}[{bit}]")
    }
}

/// A structural netlist validation failure.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ValidationError {
    /// The design name is empty.
    EmptyDesignName,
    /// A port name is empty.
    EmptyPortName,
    /// A grouped port has no bits.
    EmptyPortWidth,
    /// A grouped port width does not fit the IR identifier space.
    PortWidthOverflow,
    /// A register name is empty.
    EmptyRegisterName,
    /// A memory name is empty.
    EmptyMemoryName,
    /// An arithmetic cell name is empty.
    EmptyArithmeticName,
    /// A comparison cell name is empty.
    EmptyComparisonName,
    /// More than one port has the same name.
    DuplicatePortName(String),
    /// More than one register has the same name.
    DuplicateRegisterName(String),
    /// More than one memory has the same name.
    DuplicateMemoryName(String),
    /// More than one arithmetic cell has the same name.
    DuplicateArithmeticName(String),
    /// More than one comparison cell has the same name.
    DuplicateComparisonName(String),
    /// An arithmetic cell has empty or mismatched operand/result widths.
    ArithmeticWidth(String),
    /// A comparison cell has empty or mismatched operands.
    ComparisonWidth(String),
    /// A memory has no words.
    ZeroMemoryDepth(String),
    /// A memory has invalid or mismatched address widths.
    MemoryAddressWidth(String),
    /// A memory has invalid or mismatched word widths.
    MemoryWordWidth(String),
    /// A node consumes an undefined net.
    UndefinedNet(NetId),
    /// More than one object drives the same net.
    MultipleDrivers(NetId),
    /// A register does not reference a reserved state output.
    InvalidRegisterOutput(NetId),
    /// A reserved state output has no register cell.
    UnconnectedRegister(NetId),
    /// A memory does not reference a reserved read-data output.
    InvalidMemoryOutput(NetId),
    /// A reserved memory read-data output has no memory cell.
    UnconnectedMemory(NetId),
    /// An arithmetic cell does not reference a reserved result output.
    InvalidArithmeticOutput(NetId),
    /// A reserved arithmetic output has no arithmetic cell.
    UnconnectedArithmetic(NetId),
    /// A comparison cell does not reference a reserved result output.
    InvalidComparisonOutput(NetId),
    /// A reserved comparison output has no comparison cell.
    UnconnectedComparison(NetId),
    /// A grouped port references a node with the wrong direction.
    InvalidPortBit(NetId),
    /// A node has the wrong number of inputs.
    WrongInputCount {
        /// Net driven by the invalid node.
        net: NetId,
        /// Required number of inputs.
        expected: usize,
        /// Supplied number of inputs.
        actual: usize,
    },
}

impl Display for ValidationError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyDesignName => formatter.write_str("design name must not be empty"),
            Self::EmptyPortName => formatter.write_str("port name must not be empty"),
            Self::EmptyPortWidth => formatter.write_str("port width must not be zero"),
            Self::PortWidthOverflow => formatter.write_str("port width exceeds the IR limit"),
            Self::EmptyRegisterName => formatter.write_str("register name must not be empty"),
            Self::EmptyMemoryName => formatter.write_str("memory name must not be empty"),
            Self::EmptyArithmeticName => {
                formatter.write_str("arithmetic cell name must not be empty")
            }
            Self::EmptyComparisonName => {
                formatter.write_str("comparison cell name must not be empty")
            }
            Self::DuplicatePortName(name) => write!(formatter, "duplicate port name: {name}"),
            Self::DuplicateRegisterName(name) => {
                write!(formatter, "duplicate register name: {name}")
            }
            Self::DuplicateMemoryName(name) => write!(formatter, "duplicate memory name: {name}"),
            Self::DuplicateArithmeticName(name) => {
                write!(formatter, "duplicate arithmetic cell name: {name}")
            }
            Self::DuplicateComparisonName(name) => {
                write!(formatter, "duplicate comparison cell name: {name}")
            }
            Self::ArithmeticWidth(name) => {
                write!(formatter, "arithmetic cell {name} has an invalid width")
            }
            Self::ComparisonWidth(name) => {
                write!(formatter, "comparison cell {name} has an invalid width")
            }
            Self::ZeroMemoryDepth(name) => write!(formatter, "memory {name} has zero depth"),
            Self::MemoryAddressWidth(name) => {
                write!(formatter, "memory {name} has an invalid address width")
            }
            Self::MemoryWordWidth(name) => {
                write!(formatter, "memory {name} has an invalid word width")
            }
            Self::UndefinedNet(net) => write!(formatter, "net {net} is undefined"),
            Self::MultipleDrivers(net) => write!(formatter, "net {net} has multiple drivers"),
            Self::InvalidRegisterOutput(net) => {
                write!(formatter, "register output {net} was not reserved")
            }
            Self::UnconnectedRegister(net) => {
                write!(formatter, "reserved register output {net} is not connected")
            }
            Self::InvalidMemoryOutput(net) => {
                write!(formatter, "memory output {net} was not reserved")
            }
            Self::UnconnectedMemory(net) => {
                write!(formatter, "reserved memory output {net} is not connected")
            }
            Self::InvalidArithmeticOutput(net) => {
                write!(formatter, "arithmetic output {net} was not reserved")
            }
            Self::UnconnectedArithmetic(net) => {
                write!(
                    formatter,
                    "reserved arithmetic output {net} is not connected"
                )
            }
            Self::InvalidComparisonOutput(net) => {
                write!(formatter, "comparison output {net} was not reserved")
            }
            Self::UnconnectedComparison(net) => {
                write!(
                    formatter,
                    "reserved comparison output {net} is not connected"
                )
            }
            Self::InvalidPortBit(net) => {
                write!(formatter, "net {net} has the wrong node kind for its port")
            }
            Self::WrongInputCount {
                net,
                expected,
                actual,
            } => write!(
                formatter,
                "node driving {net} expects {expected} inputs, but has {actual}"
            ),
        }
    }
}

impl Error for ValidationError {}

#[cfg(test)]
mod tests {
    use super::{ArithmeticOp, ClockEdge, ComparisonOp, Netlist, NodeKind, RegisterCell};

    #[test]
    fn valid_and_gate_netlist_passes_validation() {
        let mut design = Netlist::new("and_gate");
        let lhs = design.add_input("a");
        let rhs = design.add_input("b");
        let result = design.add_and(lhs, rhs);
        design.add_output("y", result);

        assert_eq!(design.nodes().len(), 4);
        assert_eq!(design.validate(), Ok(()));
    }

    #[test]
    fn builder_folds_constants_and_hashes_logic() {
        let mut design = Netlist::new("optimized");
        let input = design.add_input("a");
        let zero = design.add_constant(false);
        let first = design.add_xor(input, zero);
        let second = design.add_and(first, input);
        let third = design.add_and(input, first);

        assert_eq!(first, input);
        assert_eq!(second, input);
        assert_eq!(third, input);
        assert!(
            !design
                .nodes()
                .iter()
                .any(|node| matches!(node.kind(), NodeKind::And | NodeKind::Xor))
        );
    }

    #[test]
    fn register_feedback_is_structurally_valid() {
        let mut design = Netlist::new("toggle");
        let clock = design.add_input("clock");
        let q = design.add_register_output("q");
        let d = design.add_not(q);
        design.add_register(RegisterCell::new(
            "q",
            q,
            d,
            clock,
            ClockEdge::Rising,
            None,
            None,
        ));
        design.add_output("q", q);

        assert_eq!(design.validate(), Ok(()));
        assert_eq!(design.registers().len(), 1);
    }

    #[test]
    fn word_arithmetic_outputs_are_structurally_valid() {
        let mut design = Netlist::new("add");
        let lhs = design.add_input("lhs");
        let rhs = design.add_input("rhs");
        let result = design
            .add_arithmetic(ArithmeticOp::Add, &[lhs], &[rhs])
            .unwrap();
        design.add_output("result", result[0]);

        assert_eq!(design.validate(), Ok(()));
        assert_eq!(design.arithmetic()[0].outputs(), result);
        assert!(matches!(
            design.nodes()[result[0].index() as usize].kind(),
            NodeKind::ArithmeticOutput(_)
        ));
    }

    #[test]
    fn word_comparison_output_is_structurally_valid() {
        let mut design = Netlist::new("compare");
        let lhs = design.add_input("lhs");
        let rhs = design.add_input("rhs");
        let result = design
            .add_comparison(ComparisonOp::LessThanUnsigned, &[lhs], &[rhs])
            .unwrap();
        design.add_output("result", result);

        assert_eq!(design.validate(), Ok(()));
        assert_eq!(design.comparisons()[0].output(), result);
        assert!(matches!(
            design.nodes()[result.index() as usize].kind(),
            NodeKind::ComparisonOutput(_)
        ));
    }

    #[test]
    fn duplicate_port_name_fails_validation() {
        let mut design = Netlist::new("bad_ports");
        design.add_input("a");
        design.add_input("a");

        assert!(design.validate().is_err());
    }
}
