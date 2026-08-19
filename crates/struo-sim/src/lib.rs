//! Simulation artifacts and mandatory verification gates.
//!
//! Struo treats post-synthesis simulation and equivalence as release criteria,
//! not optional diagnostics. A bitstream-producing command should call
//! [`VerificationReport::authorize_bitstream`] before packaging or programming.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt::{self, Display, Formatter};

/// A verification stage that can block bitstream release.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum VerificationStage {
    /// Simulation of hardware-semantic RTL.
    RtlSimulation,
    /// Formal or exhaustive comparison of RTL and synthesized netlist.
    SynthesisEquivalence,
    /// Check that every synthesized primitive has a simulation model.
    BlackBoxCheck,
    /// Simulation of the technology-mapped synthesized netlist.
    PostSynthesisSimulation,
    /// Successful device packing, placement, and routing.
    PlaceAndRoute,
    /// Timing constraints met after routing.
    TimingClosure,
}

impl Display for VerificationStage {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        let name = match self {
            Self::RtlSimulation => "rtl-simulation",
            Self::SynthesisEquivalence => "synthesis-equivalence",
            Self::BlackBoxCheck => "black-box-check",
            Self::PostSynthesisSimulation => "post-synthesis-simulation",
            Self::PlaceAndRoute => "place-and-route",
            Self::TimingClosure => "timing-closure",
        };
        formatter.write_str(name)
    }
}

/// Outcome recorded for a verification stage.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum StageOutcome {
    /// The stage completed successfully.
    Passed,
    /// The stage ran and found a problem.
    Failed(String),
    /// The stage was intentionally not run.
    Skipped(String),
}

/// Set of stages required before an artifact may be released.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerificationPolicy {
    required: BTreeSet<VerificationStage>,
}

impl VerificationPolicy {
    /// Policy for hardware whose incorrect behavior can cause harm.
    #[must_use]
    pub fn safety_critical() -> Self {
        Self {
            required: [
                VerificationStage::RtlSimulation,
                VerificationStage::SynthesisEquivalence,
                VerificationStage::BlackBoxCheck,
                VerificationStage::PostSynthesisSimulation,
                VerificationStage::PlaceAndRoute,
                VerificationStage::TimingClosure,
            ]
            .into_iter()
            .collect(),
        }
    }

    /// Returns the required stages.
    #[must_use]
    pub fn required(&self) -> &BTreeSet<VerificationStage> {
        &self.required
    }
}

impl Default for VerificationPolicy {
    fn default() -> Self {
        Self::safety_critical()
    }
}

/// Results associated with one reproducible synthesis artifact set.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct VerificationReport {
    outcomes: BTreeMap<VerificationStage, StageOutcome>,
}

impl VerificationReport {
    /// Creates an empty report. An empty report never authorizes a bitstream.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            outcomes: BTreeMap::new(),
        }
    }

    /// Records or replaces a stage result.
    pub fn record(&mut self, stage: VerificationStage, outcome: StageOutcome) {
        self.outcomes.insert(stage, outcome);
    }

    /// Returns all recorded outcomes.
    #[must_use]
    pub fn outcomes(&self) -> &BTreeMap<VerificationStage, StageOutcome> {
        &self.outcomes
    }

    /// Enforces the requested policy before bitstream packaging/programming.
    ///
    /// # Errors
    ///
    /// Returns every required stage that is absent, failed, or skipped.
    pub fn authorize_bitstream(&self, policy: &VerificationPolicy) -> Result<(), ReleaseBlocked> {
        let blocked = policy
            .required()
            .iter()
            .filter_map(|stage| match self.outcomes.get(stage) {
                Some(StageOutcome::Passed) => None,
                Some(StageOutcome::Failed(reason) | StageOutcome::Skipped(reason)) => {
                    Some((*stage, reason.clone()))
                }
                None => Some((*stage, "not run".into())),
            })
            .collect::<Vec<_>>();

        if blocked.is_empty() {
            Ok(())
        } else {
            Err(ReleaseBlocked { blocked })
        }
    }
}

/// Refusal to produce or program a bitstream.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReleaseBlocked {
    blocked: Vec<(VerificationStage, String)>,
}

impl ReleaseBlocked {
    /// Returns the stages preventing release and their reasons.
    #[must_use]
    pub fn blocked(&self) -> &[(VerificationStage, String)] {
        &self.blocked
    }
}

impl Display for ReleaseBlocked {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        formatter.write_str("bitstream release blocked by ")?;
        for (index, (stage, reason)) in self.blocked.iter().enumerate() {
            if index != 0 {
                formatter.write_str(", ")?;
            }
            write!(formatter, "{stage} ({reason})")?;
        }
        Ok(())
    }
}

impl Error for ReleaseBlocked {}

#[cfg(test)]
mod tests {
    use super::{StageOutcome, VerificationPolicy, VerificationReport, VerificationStage};

    #[test]
    fn post_synthesis_simulation_cannot_be_omitted() {
        let mut report = VerificationReport::new();
        for stage in VerificationPolicy::safety_critical().required() {
            if *stage != VerificationStage::PostSynthesisSimulation {
                report.record(*stage, StageOutcome::Passed);
            }
        }

        let error = report
            .authorize_bitstream(&VerificationPolicy::safety_critical())
            .unwrap_err();

        assert_eq!(
            error.blocked()[0].0,
            VerificationStage::PostSynthesisSimulation
        );
    }

    #[test]
    fn all_required_evidence_authorizes_release() {
        let policy = VerificationPolicy::safety_critical();
        let mut report = VerificationReport::new();
        for stage in policy.required() {
            report.record(*stage, StageOutcome::Passed);
        }

        assert_eq!(report.authorize_bitstream(&policy), Ok(()));
    }
}
