//! Frontend-independent RTL that preserves hardware intent.
//!
//! This is the boundary between language-specific analysis and synthesis. In
//! particular, clocks, resets, registers, memories, hierarchy, bit widths, and
//! two/four-state types must still be explicit at this stage.

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

/// Logical reset polarity.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Polarity {
    /// Asserted at logic one.
    ActiveHigh,
    /// Asserted at logic zero.
    ActiveLow,
}

/// Reset semantics attached to a register.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Reset {
    /// Reset signal name before net lowering.
    pub signal: String,
    /// Synchronous or asynchronous reset.
    pub mode: ResetMode,
    /// Assertion polarity.
    pub polarity: Polarity,
    /// Packed reset value, least-significant word first.
    pub value: Vec<u64>,
}

/// A register before bit blasting and technology mapping.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Register {
    /// Register name.
    pub name: String,
    /// Stored value type.
    pub r#type: ValueType,
    /// Clock signal name.
    pub clock: String,
    /// Active clock edge.
    pub edge: ClockEdge,
    /// Optional clock-enable signal.
    pub enable: Option<String>,
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

    /// Adds a port.
    pub fn add_port(&mut self, port: Port) {
        self.ports.push(port);
    }

    /// Adds a register.
    pub fn add_register(&mut self, register: Register) {
        self.registers.push(register);
    }

    /// Adds a memory.
    pub fn add_memory(&mut self, memory: Memory) {
        self.memories.push(memory);
    }

    /// Adds an instance.
    pub fn add_instance(&mut self, instance: Instance) {
        self.instances.push(instance);
    }

    /// Checks module-local structural invariants.
    ///
    /// # Errors
    ///
    /// Returns the first invalid or duplicate name found.
    pub fn validate(&self) -> Result<(), RtlError> {
        validate_name(&self.name, "module")?;
        let mut names = HashSet::new();

        for port in &self.ports {
            validate_unique_name(&port.name, "port", &mut names)?;
        }
        for register in &self.registers {
            validate_unique_name(&register.name, "register", &mut names)?;
            validate_name(&register.clock, "clock signal")?;
            if let Some(enable) = &register.enable {
                validate_name(enable, "enable signal")?;
            }
            if let Some(reset) = &register.reset {
                validate_name(&reset.signal, "reset signal")?;
            }
        }
        for memory in &self.memories {
            validate_unique_name(&memory.name, "memory", &mut names)?;
            if memory.depth == 0 {
                return Err(RtlError::ZeroDepth(memory.name.clone()));
            }
        }
        for instance in &self.instances {
            validate_unique_name(&instance.name, "instance", &mut names)?;
            validate_name(&instance.module, "instantiated module")?;
        }
        Ok(())
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
}

impl Display for RtlError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroWidth => formatter.write_str("packed bit width must be non-zero"),
            Self::EmptyName(kind) => write!(formatter, "{kind} name must not be empty"),
            Self::DuplicateName(name) => write!(formatter, "duplicate RTL name: {name}"),
            Self::MissingTop(name) => write!(formatter, "top module `{name}` does not exist"),
            Self::UnknownModule(name) => write!(formatter, "unknown module `{name}`"),
            Self::ZeroDepth(name) => write!(formatter, "memory `{name}` has zero depth"),
        }
    }
}

impl Error for RtlError {}

#[cfg(test)]
mod tests {
    use super::{BitWidth, Design, Module, Port, PortDirection, StateDomain, ValueType};

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
    fn missing_top_is_rejected() {
        assert!(Design::new("Missing").validate().is_err());
    }
}
