//! Technology-independent circuit representation used by Struo.

use std::collections::HashSet;
use std::error::Error;
use std::fmt::{self, Display, Formatter};

/// Identifies a one-bit net within a [`Netlist`].
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct NetId(u32);

impl Display for NetId {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        write!(formatter, "n{}", self.0)
    }
}

/// A combinational node in a netlist.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NodeKind {
    /// A named primary input.
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
    /// A named primary output.
    Output(String),
}

impl NodeKind {
    const fn expected_input_count(&self) -> usize {
        match self {
            Self::Input(_) | Self::Constant(_) => 0,
            Self::Not | Self::Output(_) => 1,
            Self::And | Self::Or | Self::Xor => 2,
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

/// A flat, topologically ordered, one-bit logic netlist.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Netlist {
    name: String,
    nodes: Vec<Node>,
    next_net: u32,
}

impl Netlist {
    /// Creates an empty netlist.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            nodes: Vec::new(),
            next_net: 0,
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

    /// Adds a primary input and returns its net.
    pub fn add_input(&mut self, name: impl Into<String>) -> NetId {
        self.add_node(NodeKind::Input(name.into()), Vec::new())
    }

    /// Adds a constant and returns its net.
    pub fn add_constant(&mut self, value: bool) -> NetId {
        self.add_node(NodeKind::Constant(value), Vec::new())
    }

    /// Adds a two-input AND gate.
    pub fn add_and(&mut self, lhs: NetId, rhs: NetId) -> NetId {
        self.add_node(NodeKind::And, vec![lhs, rhs])
    }

    /// Adds a two-input OR gate.
    pub fn add_or(&mut self, lhs: NetId, rhs: NetId) -> NetId {
        self.add_node(NodeKind::Or, vec![lhs, rhs])
    }

    /// Adds a two-input XOR gate.
    pub fn add_xor(&mut self, lhs: NetId, rhs: NetId) -> NetId {
        self.add_node(NodeKind::Xor, vec![lhs, rhs])
    }

    /// Adds an inverter.
    pub fn add_not(&mut self, input: NetId) -> NetId {
        self.add_node(NodeKind::Not, vec![input])
    }

    /// Adds a primary output.
    pub fn add_output(&mut self, name: impl Into<String>, source: NetId) -> NetId {
        self.add_node(NodeKind::Output(name.into()), vec![source])
    }

    /// Checks structural invariants required by synthesis passes.
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

            if let NodeKind::Input(name) | NodeKind::Output(name) = &node.kind {
                if name.trim().is_empty() {
                    return Err(ValidationError::EmptyPortName);
                }
                if !port_names.insert(name.as_str()) {
                    return Err(ValidationError::DuplicatePortName(name.clone()));
                }
            }
        }

        Ok(())
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
}

/// A structural netlist validation failure.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ValidationError {
    /// The design name is empty.
    EmptyDesignName,
    /// A port name is empty.
    EmptyPortName,
    /// More than one port has the same name.
    DuplicatePortName(String),
    /// A node consumes a net before it is defined.
    UndefinedNet(NetId),
    /// More than one node drives the same net.
    MultipleDrivers(NetId),
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
            Self::DuplicatePortName(name) => write!(formatter, "duplicate port name: {name}"),
            Self::UndefinedNet(net) => write!(formatter, "net {net} is used before it is defined"),
            Self::MultipleDrivers(net) => write!(formatter, "net {net} has multiple drivers"),
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
    use super::Netlist;

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
    fn duplicate_port_name_fails_validation() {
        let mut design = Netlist::new("bad_ports");
        design.add_input("a");
        design.add_input("a");

        assert!(design.validate().is_err());
    }
}
