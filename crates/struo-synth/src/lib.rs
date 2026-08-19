//! Synthesis pass infrastructure for Struo.

use std::error::Error;
use std::fmt::{self, Display, Formatter};

use struo_ir::{Netlist, ValidationError};
use struo_rtl::{Design, RtlError};

/// Verifies hardware-semantic RTL before any information-losing lowering.
///
/// # Errors
///
/// Returns an error when hierarchy, names, or target-independent state are
/// structurally invalid.
pub fn validate_rtl(design: &Design) -> Result<(), SynthesisError> {
    design.validate().map_err(SynthesisError::InvalidRtl)
}

/// Summary emitted after a synthesis pass runs.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PassReport {
    /// Stable pass name.
    pub pass: &'static str,
    /// Human-readable description of the result.
    pub message: String,
}

/// A transformation or analysis in the synthesis pipeline.
pub trait Pass {
    /// Returns a stable name for logs and diagnostics.
    fn name(&self) -> &'static str;

    /// Runs the pass on a design.
    ///
    /// # Errors
    ///
    /// Returns an error when the input is invalid or the transformation fails.
    fn run(&self, design: &mut Netlist) -> Result<PassReport, SynthesisError>;
}

/// An ordered sequence of synthesis passes.
#[derive(Default)]
pub struct Pipeline {
    passes: Vec<Box<dyn Pass>>,
}

impl Pipeline {
    /// Creates an empty pipeline.
    #[must_use]
    pub const fn new() -> Self {
        Self { passes: Vec::new() }
    }

    /// Appends a pass.
    pub fn push(&mut self, pass: impl Pass + 'static) {
        self.passes.push(Box::new(pass));
    }

    /// Runs each pass in order.
    ///
    /// # Errors
    ///
    /// Stops and returns the first pass failure.
    pub fn run(&self, design: &mut Netlist) -> Result<Vec<PassReport>, SynthesisError> {
        self.passes.iter().map(|pass| pass.run(design)).collect()
    }
}

/// Returns the default development pipeline.
#[must_use]
pub fn default_pipeline() -> Pipeline {
    let mut pipeline = Pipeline::new();
    pipeline.push(ValidateNetlist);
    pipeline
}

/// Verifies the structural invariants of a design.
pub struct ValidateNetlist;

impl Pass for ValidateNetlist {
    fn name(&self) -> &'static str {
        "validate"
    }

    fn run(&self, design: &mut Netlist) -> Result<PassReport, SynthesisError> {
        design.validate()?;
        Ok(PassReport {
            pass: self.name(),
            message: format!("{} nodes are structurally valid", design.nodes().len()),
        })
    }
}

/// A synthesis pipeline failure.
#[derive(Debug)]
pub enum SynthesisError {
    /// The frontend-independent RTL is structurally invalid.
    InvalidRtl(RtlError),
    /// The circuit representation is structurally invalid.
    InvalidNetlist(ValidationError),
}

impl Display for SynthesisError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRtl(error) => write!(formatter, "invalid RTL: {error}"),
            Self::InvalidNetlist(error) => write!(formatter, "invalid netlist: {error}"),
        }
    }
}

impl Error for SynthesisError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidRtl(error) => Some(error),
            Self::InvalidNetlist(error) => Some(error),
        }
    }
}

impl From<ValidationError> for SynthesisError {
    fn from(error: ValidationError) -> Self {
        Self::InvalidNetlist(error)
    }
}

#[cfg(test)]
mod tests {
    use struo_ir::Netlist;

    use super::default_pipeline;

    #[test]
    fn default_pipeline_validates_a_design() {
        let mut design = Netlist::new("inverter");
        let input = design.add_input("a");
        let inverted = design.add_not(input);
        design.add_output("y", inverted);

        let reports = default_pipeline().run(&mut design).unwrap();

        assert_eq!(reports.len(), 1);
        assert_eq!(reports[0].pass, "validate");
    }
}
