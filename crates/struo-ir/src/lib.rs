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
}

impl NodeKind {
    const fn expected_input_count(&self) -> usize {
        match self {
            Self::Input(_) | Self::Constant(_) | Self::RegisterOutput(_) => 0,
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

    /// Connects a previously reserved register output to its D/control nets.
    pub fn add_register(&mut self, register: RegisterCell) {
        self.registers.push(register);
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
    /// More than one port has the same name.
    DuplicatePortName(String),
    /// More than one register has the same name.
    DuplicateRegisterName(String),
    /// A node consumes an undefined net.
    UndefinedNet(NetId),
    /// More than one object drives the same net.
    MultipleDrivers(NetId),
    /// A register does not reference a reserved state output.
    InvalidRegisterOutput(NetId),
    /// A reserved state output has no register cell.
    UnconnectedRegister(NetId),
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
            Self::DuplicatePortName(name) => write!(formatter, "duplicate port name: {name}"),
            Self::DuplicateRegisterName(name) => {
                write!(formatter, "duplicate register name: {name}")
            }
            Self::UndefinedNet(net) => write!(formatter, "net {net} is undefined"),
            Self::MultipleDrivers(net) => write!(formatter, "net {net} has multiple drivers"),
            Self::InvalidRegisterOutput(net) => {
                write!(formatter, "register output {net} was not reserved")
            }
            Self::UnconnectedRegister(net) => {
                write!(formatter, "reserved register output {net} is not connected")
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
    use super::{ClockEdge, Netlist, NodeKind, RegisterCell};

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
    fn duplicate_port_name_fails_validation() {
        let mut design = Netlist::new("bad_ports");
        design.add_input("a");
        design.add_input("a");

        assert!(design.validate().is_err());
    }
}
