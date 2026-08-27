//! Adapter boundary from Veryl analyzer IR to [`struo_rtl`].
//!
//! The analyzer dependency is pinned intentionally. Analyzer-native IDs and
//! resource-table handles must not escape this crate.

use std::error::Error;
use std::fmt::{self, Display, Formatter};
use std::fs;
use std::path::{Path, PathBuf};

use struo_rtl::{BitWidth, Design, Module, Port, PortDirection, RtlError, StateDomain, ValueType};
use veryl_analyzer::ir::{Component, Declaration, Ir, VarKind};
use veryl_analyzer::{Analyzer, Context};
use veryl_metadata::Metadata;
use veryl_parser::{Parser, resource_table};

mod lower;

pub use lower::lower_analyzed_ir;

/// Analyzes one self-contained Veryl source and lowers its selected top.
///
/// This convenience entry point exists for direct source-to-synthesis flows.
/// Use [`analyze_project_and_lower`] for manifest-backed projects with multiple
/// compilation units or dependencies.
///
/// # Errors
///
/// Returns an error for parser or analyzer diagnostics, metadata setup, or
/// semantic lowering failures.
pub fn analyze_and_lower(source: &str, project: &str, top: &str) -> Result<Design, ImportError> {
    let metadata = Metadata::create_default(project)
        .map_err(|error| ImportError::AnalysisFailed(error.to_string()))?;
    reset_analyzer();
    let parsed = Parser::parse(source, &"")
        .map_err(|error| ImportError::AnalysisFailed(error.to_string()))?;
    analyze_parsed_and_lower(&metadata, &[(project.to_owned(), parsed)], top)
}

/// Loads and analyzes an entire Veryl project, then lowers its selected top.
///
/// `project` may point either to a project directory or directly to its
/// `Veryl.toml`. All sources declared by the manifest, the Veryl standard
/// library, and locked dependency sources participate in the same analyzer
/// passes, matching Veryl's project compilation model.
///
/// # Errors
///
/// Returns an error when the manifest or a source cannot be loaded, dependency
/// resolution fails, Veryl reports a diagnostic, or semantic lowering fails.
pub fn analyze_project_and_lower(
    project: impl AsRef<Path>,
    top: &str,
) -> Result<Design, ImportError> {
    let project = project.as_ref();
    let manifest = if project.is_dir() {
        project.join("Veryl.toml")
    } else {
        project.to_path_buf()
    };
    let mut metadata = Metadata::load(&manifest).map_err(|error| {
        ImportError::AnalysisFailed(format!(
            "failed to load Veryl project `{}`: {error}",
            manifest.display()
        ))
    })?;
    let files: &[PathBuf] = &[];
    let paths = metadata.paths(files, true, true).map_err(|error| {
        ImportError::AnalysisFailed(format!(
            "failed to resolve sources for `{}`: {error}",
            manifest.display()
        ))
    })?;

    reset_analyzer();
    let mut parsed = Vec::with_capacity(paths.len());
    for path in paths {
        let source = fs::read_to_string(&path.src).map_err(|error| {
            ImportError::AnalysisFailed(format!(
                "failed to read Veryl source `{}`: {error}",
                path.src.display()
            ))
        })?;
        let parser = Parser::parse(&source, &path.src)
            .map_err(|error| ImportError::AnalysisFailed(error.to_string()))?;
        parsed.push((path.prj, parser));
    }

    analyze_parsed_and_lower(&metadata, &parsed, top)
}

fn reset_analyzer() {
    // Veryl's resource, symbol, and attribute tables are thread-local. Reset
    // this worker's analyzer state without serializing independent analyses on
    // other threads.
    veryl_analyzer::symbol_table::clear();
    veryl_analyzer::attribute_table::clear();
}

fn analyze_parsed_and_lower(
    metadata: &Metadata,
    parsed: &[(String, Parser)],
    top: &str,
) -> Result<Design, ImportError> {
    let analyzer = Analyzer::new(metadata);
    let mut context = Context::default();
    let mut ir = Ir::default();

    let mut pass1 = Vec::new();
    for (project, parser) in parsed {
        pass1.append(&mut analyzer.analyze_pass1(project, &parser.veryl));
    }
    if !pass1.is_empty() {
        return Err(ImportError::AnalysisFailed(format!("{pass1:?}")));
    }
    let post1 = Analyzer::analyze_post_pass1();
    if !post1.is_empty() {
        return Err(ImportError::AnalysisFailed(format!("{post1:?}")));
    }

    let mut pass2 = Vec::new();
    for (project, parser) in parsed {
        context.set_project_name(project);
        pass2.append(&mut analyzer.analyze_pass2(&parser.veryl, &mut context, Some(&mut ir)));
    }
    if !pass2.is_empty() {
        return Err(ImportError::AnalysisFailed(format!("{pass2:?}")));
    }
    let post2 = Analyzer::analyze_post_pass2(&ir);
    if !post2.is_empty() {
        return Err(ImportError::AnalysisFailed(format!("{post2:?}")));
    }
    lower_analyzed_ir(&ir, top)
}

/// Exact Veryl analyzer release supported by this adapter.
pub const SUPPORTED_VERYL_VERSION: &str = "0.20.3";

/// Requested treatment of an unpacked Veryl array during memory inference.
///
/// Veryl 0.20.3 does not accept tool-defined attribute names, so source code
/// selects this policy through its portable `SystemVerilog` attribute escape:
/// `#[sv("struo_memory = \"required\"")]`. The other accepted values are
/// `preferred` and `forbidden`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum MemoryInferencePolicy {
    /// Infer a memory when the access pattern is recognized.
    #[default]
    Preferred,
    /// Reject the design unless the array is inferred as a memory.
    Required,
    /// Keep the array in ordinary logic and never infer a memory.
    Forbidden,
}

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
    /// The analyzed construct is not implemented by the semantic lowerer.
    UnsupportedBehavior(String),
    /// A source directive used an unknown memory-inference policy.
    InvalidMemoryInferencePolicy {
        /// Array carrying the directive.
        memory: String,
        /// Unsupported directive value.
        value: String,
    },
    /// An array carried incompatible memory-inference directives.
    ConflictingMemoryInferencePolicies(String),
    /// A required array did not match the supported memory contract.
    RequiredMemoryInferenceFailed {
        /// Array that was required to become a memory.
        memory: String,
        /// First unsupported part of its access pattern.
        reason: String,
    },
    /// The requested top module was not present in the analyzer IR.
    MissingTop(String),
    /// Veryl parsing, analysis, or metadata setup failed.
    AnalysisFailed(String),
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
            Self::UnsupportedBehavior(description) => {
                write!(formatter, "unsupported Veryl behavior: {description}")
            }
            Self::InvalidMemoryInferencePolicy { memory, value } => write!(
                formatter,
                "memory `{memory}` has invalid `struo_memory` policy `{value}`; expected `preferred`, `required`, or `forbidden`"
            ),
            Self::ConflictingMemoryInferencePolicies(memory) => write!(
                formatter,
                "memory `{memory}` has conflicting `struo_memory` policies"
            ),
            Self::RequiredMemoryInferenceFailed { memory, reason } => write!(
                formatter,
                "block-memory inference was required for `{memory}`, but failed: {reason}"
            ),
            Self::MissingTop(top) => write!(formatter, "Veryl top module `{top}` was not found"),
            Self::AnalysisFailed(diagnostic) => {
                write!(formatter, "Veryl analysis failed: {diagnostic}")
            }
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
    use std::fs;

    use tempfile::tempdir;
    use veryl_analyzer::ir::Ir;

    use super::{SUPPORTED_VERYL_VERSION, analyze_project_and_lower, import_analyzed_shell};

    #[test]
    fn empty_analyzer_ir_is_not_silently_made_valid() {
        let imported = import_analyzed_shell(&Ir::default(), "Top").unwrap();

        assert_eq!(SUPPORTED_VERYL_VERSION, "0.20.3");
        assert!(imported.design.validate().is_err());
    }

    #[test]
    fn analyzes_all_compilation_units_in_a_project() {
        let directory = tempdir().unwrap();
        let source_directory = directory.path().join("src");
        fs::create_dir(&source_directory).unwrap();
        fs::write(
            directory.path().join("Veryl.toml"),
            r#"[project]
name = "multi_file"
version = "0.1.0"

[build]
sources = ["src"]
exclude_std = true
target = { type = "directory", path = "target" }
"#,
        )
        .unwrap();
        fs::write(
            source_directory.join("Child.veryl"),
            r"module Child (
    value : input  logic<8>,
    result: output logic<8>,
) {
    always_comb {
        result = value + 8'h01;
    }
}
",
        )
        .unwrap();
        fs::write(
            source_directory.join("Top.veryl"),
            r"module Top (
    value : input  logic<8>,
    result: output logic<8>,
) {
    inst child: Child (
        value : value ,
        result: result,
    );
}
",
        )
        .unwrap();

        let design = analyze_project_and_lower(directory.path(), "Top").unwrap();
        let top = design.top_module().unwrap();
        assert_eq!(top.name(), "Top");
        assert!(top.instances().is_empty());
    }
}
