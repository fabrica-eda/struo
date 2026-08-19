//! Adapter boundary from Veryl analyzer IR to [`struo_rtl`].
//!
//! The analyzer dependency is pinned intentionally. Analyzer-native IDs and
//! resource-table handles must not escape this crate.

use std::error::Error;
use std::fmt::{self, Display, Formatter};

use struo_rtl::{BitWidth, Design, Module, Port, PortDirection, RtlError, StateDomain, ValueType};
use veryl_analyzer::ir::{Component, Declaration, Ir, VarKind};
use veryl_parser::resource_table;

/// Exact Veryl analyzer release supported by this adapter.
pub const SUPPORTED_VERYL_VERSION: &str = "0.20.3";

/// Counts source constructs whose semantic lowering must be implemented.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LoweringInventory {
    /// Combinational procedural declarations.
    pub combinational_blocks: usize,
    /// Sequential procedural declarations.
    pub sequential_blocks: usize,
    /// Module instances.
    pub instances: usize,
    /// External components and unsupported declarations.
    pub unsupported: usize,
}

impl LoweringInventory {
    /// Returns true only when the analyzed design has no body constructs left
    /// to lower.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.combinational_blocks == 0
            && self.sequential_blocks == 0
            && self.instances == 0
            && self.unsupported == 0
    }
}

/// A partial import used to establish and test the analyzer compatibility
/// boundary before procedural lowering is implemented.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AnalyzedShell {
    /// Frontend-independent module and port metadata.
    pub design: Design,
    /// Body constructs that have not yet crossed the boundary.
    pub pending: LoweringInventory,
}

impl AnalyzedShell {
    /// Converts the shell into synthesis input only when no behavior was
    /// dropped by the adapter.
    ///
    /// # Errors
    ///
    /// Returns an error while any declaration still needs semantic lowering,
    /// or when the resulting RTL design is structurally invalid.
    pub fn require_fully_lowered(self) -> Result<Design, ImportError> {
        if !self.pending.is_empty() {
            return Err(ImportError::UnloweredBehavior(self.pending));
        }
        self.design.validate()?;
        Ok(self.design)
    }
}

/// Imports module and port metadata from a fully analyzed Veryl IR.
///
/// This function deliberately reports all unlowered bodies in [`AnalyzedShell::pending`].
/// Synthesis must reject a shell unless the pending inventory is empty; it is
/// never valid to silently drop analyzed behavior.
///
/// # Errors
///
/// Returns an error for unresolved resource-table strings, missing variables,
/// non-concrete widths, or widths beyond Struo's current representation.
pub fn import_analyzed_shell(
    ir: &Ir,
    top: impl Into<String>,
) -> Result<AnalyzedShell, ImportError> {
    let mut design = Design::new(top);
    let mut pending = LoweringInventory::default();

    for component in &ir.components {
        match component {
            Component::Module(source) => {
                let module_name = resolve_name(source.name)?;
                let mut module = Module::new(module_name);
                let mut ports = source
                    .ports
                    .iter()
                    .map(|(path, id)| {
                        let variable = source
                            .variables
                            .get(id)
                            .ok_or_else(|| ImportError::MissingVariable(path.to_string()))?;
                        let direction = match variable.kind {
                            VarKind::Input => PortDirection::Input,
                            VarKind::Output => PortDirection::Output,
                            VarKind::Inout => PortDirection::Inout,
                            _ => return Err(ImportError::NonPort(path.to_string())),
                        };
                        let width = variable
                            .total_width()
                            .ok_or_else(|| ImportError::NonConcreteWidth(path.to_string()))?;
                        let width = u32::try_from(width)
                            .map_err(|_| ImportError::WidthTooLarge(path.to_string()))?;
                        Ok(Port {
                            name: path.to_string(),
                            direction,
                            r#type: ValueType {
                                width: BitWidth::new(width)?,
                                signed: variable.r#type.signed,
                                state: if variable.r#type.is_4state() {
                                    StateDomain::FourState
                                } else {
                                    StateDomain::TwoState
                                },
                            },
                        })
                    })
                    .collect::<Result<Vec<_>, ImportError>>()?;
                ports.sort_by(|lhs, rhs| lhs.name.cmp(&rhs.name));
                for port in ports {
                    module.add_port(port);
                }

                for declaration in &source.declarations {
                    match declaration {
                        Declaration::Comb(_) => pending.combinational_blocks += 1,
                        Declaration::Ff(_) => pending.sequential_blocks += 1,
                        Declaration::Inst(_) => pending.instances += 1,
                        Declaration::Null => {}
                        Declaration::External(_)
                        | Declaration::Initial(_)
                        | Declaration::Final(_)
                        | Declaration::Unsupported(_) => pending.unsupported += 1,
                    }
                }
                design.add_module(module);
            }
            Component::Interface(_) | Component::SystemVerilog(_) => pending.unsupported += 1,
        }
    }

    Ok(AnalyzedShell { design, pending })
}

fn resolve_name(id: veryl_parser::resource_table::StrId) -> Result<String, ImportError> {
    resource_table::get_str_value(id).ok_or(ImportError::MissingResourceString)
}

/// Failure while projecting analyzer-owned data into Struo RTL.
#[derive(Debug)]
pub enum ImportError {
    /// A `StrId` was not present in Veryl's resource table.
    MissingResourceString,
    /// A module port referenced an absent variable.
    MissingVariable(String),
    /// A supposedly exported port was not an input, output, or inout.
    NonPort(String),
    /// A port width still depends on an unevaluated parameter.
    NonConcreteWidth(String),
    /// A concrete width does not fit in Struo's current width type.
    WidthTooLarge(String),
    /// Behavior remains in analyzer-native declarations.
    UnloweredBehavior(LoweringInventory),
    /// The projected RTL metadata is invalid.
    InvalidRtl(RtlError),
}

impl Display for ImportError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingResourceString => {
                formatter.write_str("Veryl string table entry is missing")
            }
            Self::MissingVariable(path) => write!(formatter, "Veryl port `{path}` has no variable"),
            Self::NonPort(path) => write!(formatter, "Veryl entry `{path}` is not a port"),
            Self::NonConcreteWidth(path) => {
                write!(
                    formatter,
                    "Veryl port `{path}` does not have a concrete width"
                )
            }
            Self::WidthTooLarge(path) => write!(formatter, "Veryl port `{path}` is too wide"),
            Self::UnloweredBehavior(pending) => write!(
                formatter,
                "Veryl behavior is not fully lowered: {} comb, {} ff, {} instances, {} unsupported",
                pending.combinational_blocks,
                pending.sequential_blocks,
                pending.instances,
                pending.unsupported,
            ),
            Self::InvalidRtl(error) => write!(formatter, "invalid imported RTL: {error}"),
        }
    }
}

impl Error for ImportError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidRtl(error) => Some(error),
            _ => None,
        }
    }
}

impl From<RtlError> for ImportError {
    fn from(error: RtlError) -> Self {
        Self::InvalidRtl(error)
    }
}

#[cfg(test)]
mod tests {
    use veryl_analyzer::ir::Ir;

    use super::{SUPPORTED_VERYL_VERSION, import_analyzed_shell};

    #[test]
    fn empty_analyzer_ir_is_not_silently_made_valid() {
        let imported = import_analyzed_shell(&Ir::default(), "Top").unwrap();

        assert_eq!(SUPPORTED_VERYL_VERSION, "0.20.3");
        assert!(imported.design.validate().is_err());
    }
}
