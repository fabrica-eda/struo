//! Lattice ECP5 target descriptions and reproducible tool recipes.

mod mapped;
mod physical;

pub use mapped::{
    ArithmeticMapping, Bit, Control, Ecp5Cell, Ecp5Netlist, MappedPort, MappingError,
    MappingOptions, PortDirection as MappedPortDirection, Reset, RetimingSelection, map_to_ecp5,
    map_to_ecp5_with_options,
};
pub use physical::{PhysicalFeedback, PhysicalLocation, PhysicalNetTiming, PhysicalTimingEndpoint};

use struo_sim::{ReleaseBlocked, VerificationPolicy, VerificationReport, VerificationStage};

/// Immutable target identity used in artifact manifests.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BoardProfile {
    /// Lattice board ordering code.
    pub board: &'static str,
    /// Exact populated FPGA device.
    pub device: &'static str,
    /// Speed grade printed in the device ordering code.
    pub speed_grade: u8,
    /// nextpnr architecture/device selector.
    pub nextpnr_device: &'static str,
    /// nextpnr package selector.
    pub nextpnr_package: &'static str,
    /// Default FTDI-provided board clock.
    pub default_clock_hz: u32,
    /// Hardware setup needed for the default clock.
    pub clock_setup: &'static str,
}

/// Initial Struo hardware target.
pub const LFE5UM5G_85F_EVN: BoardProfile = BoardProfile {
    board: "LFE5UM5G-85F-EVN",
    device: "LFE5UM5G-85F-8BG381",
    speed_grade: 8,
    nextpnr_device: "--um5g-85k",
    nextpnr_package: "CABGA381",
    default_clock_hz: 12_000_000,
    clock_setup: "short JP2 to connect the FTDI 12 MHz clock to FPGA pin A10",
};

/// Default implementation-quality target for ECP5 speed-grade 8 designs.
///
/// This constrains place-and-route quality independently of the 12 MHz
/// reference clock used by the no-PLL evaluation-board smoke test.
pub const ECP5_QOR_TARGET_MHZ: u32 = 300;

/// A subprocess invocation represented without a shell.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ToolCommand {
    /// Executable name.
    pub program: &'static str,
    /// Individual process arguments.
    pub args: Vec<String>,
    /// Verification evidence produced on success, if any.
    pub evidence: Option<VerificationStage>,
}

/// Conventional paths belonging to one immutable build directory.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FlowArtifacts {
    /// Serialized mapped netlist consumed by nextpnr.
    pub mapped_json: String,
    /// Post-draft netlist annotated with exact physical placements.
    pub draft_placed_json: String,
    /// Detailed routed timing observations from the draft implementation.
    pub draft_report: String,
    /// Throwaway routed configuration used to obtain physical feedback.
    pub draft_config: String,
    /// Equivalent mapped netlist refined using draft physical feedback.
    pub refined_json: String,
    /// nextpnr textual configuration.
    pub routed_config: String,
    /// Packed FPGA bitstream.
    pub bitstream: String,
    /// Optional SVF programming stream.
    pub svf: String,
    /// Board LPF copied into the artifact directory.
    pub constraints: String,
}

impl FlowArtifacts {
    /// Returns conventional filenames below `root`.
    #[must_use]
    pub fn under(root: &str) -> Self {
        let root = root.trim_end_matches('/');
        Self {
            mapped_json: format!("{root}/design.json"),
            draft_placed_json: format!("{root}/design.draft.json"),
            draft_report: format!("{root}/design.draft-report.json"),
            draft_config: format!("{root}/design.draft.config"),
            refined_json: format!("{root}/design.refined.json"),
            routed_config: format!("{root}/design.config"),
            bitstream: format!("{root}/design.bit"),
            svf: format!("{root}/design.svf"),
            constraints: format!("{root}/board.lpf"),
        }
    }
}

/// Open-source ECP5 synthesis and implementation recipe.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ecp5Flow {
    /// Synthesizable top module.
    pub top: String,
    /// Physical board target.
    pub board: BoardProfile,
    /// Frequency used to drive timing optimization and sign-off.
    pub timing_goal_mhz: u32,
    /// Files produced during the flow.
    pub artifacts: FlowArtifacts,
}

impl Ecp5Flow {
    /// Creates a flow for the initial board target.
    #[must_use]
    pub fn evaluation_board(top: impl Into<String>, artifact_root: &str) -> Self {
        Self {
            top: top.into(),
            board: LFE5UM5G_85F_EVN,
            timing_goal_mhz: ECP5_QOR_TARGET_MHZ,
            artifacts: FlowArtifacts::under(artifact_root),
        }
    }

    /// Deterministic routed draft used only to return physical observations to
    /// synthesis. Timing failure is allowed because this is not sign-off.
    #[must_use]
    pub fn draft_place_and_route_command(&self) -> ToolCommand {
        let mut command = self
            .place_and_route_command_for(&self.artifacts.mapped_json, &self.artifacts.draft_config);
        command.args.extend([
            "--write".into(),
            self.artifacts.draft_placed_json.clone(),
            "--report".into(),
            self.artifacts.draft_report.clone(),
            "--detailed-timing-report".into(),
            "--timing-allow-fail".into(),
        ]);
        command.evidence = None;
        command
    }

    /// nextpnr command for the exact UM5G-85K CABGA381 device.
    #[must_use]
    pub fn place_and_route_command(&self) -> ToolCommand {
        self.place_and_route_command_for(&self.artifacts.mapped_json, &self.artifacts.routed_config)
    }

    /// Final implementation command for a netlist refined from the matching
    /// deterministic draft run.
    #[must_use]
    pub fn refined_place_and_route_command(&self) -> ToolCommand {
        self.place_and_route_command_for(
            &self.artifacts.refined_json,
            &self.artifacts.routed_config,
        )
    }

    fn place_and_route_command_for(&self, json: &str, config: &str) -> ToolCommand {
        ToolCommand {
            program: "nextpnr-ecp5",
            args: vec![
                self.board.nextpnr_device.into(),
                "--package".into(),
                self.board.nextpnr_package.into(),
                "--speed".into(),
                self.board.speed_grade.to_string(),
                "--json".into(),
                json.into(),
                "--lpf".into(),
                self.artifacts.constraints.clone(),
                "--textcfg".into(),
                config.into(),
                "--freq".into(),
                self.timing_goal_mhz.to_string(),
                "--placer-budgets".into(),
                "--placer-heap-timingweight".into(),
                "30".into(),
                "--tmg-ripup".into(),
                "--seed".into(),
                "1".into(),
            ],
            evidence: Some(VerificationStage::PlaceAndRoute),
        }
    }

    /// Project Trellis bitstream pack command, available only after every
    /// required verification stage passed.
    ///
    /// # Errors
    ///
    /// Returns the missing, skipped, or failed release gates instead of a
    /// command that could produce a programmable bitstream.
    pub fn pack_command(
        &self,
        report: &VerificationReport,
        policy: &VerificationPolicy,
    ) -> Result<ToolCommand, ReleaseBlocked> {
        report.authorize_bitstream(policy)?;
        Ok(ToolCommand {
            program: "ecppack",
            args: vec![
                "--svf".into(),
                self.artifacts.svf.clone(),
                self.artifacts.routed_config.clone(),
                self.artifacts.bitstream.clone(),
            ],
            evidence: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use struo_sim::{VerificationPolicy, VerificationReport};

    use super::{ECP5_QOR_TARGET_MHZ, Ecp5Flow, LFE5UM5G_85F_EVN};

    #[test]
    fn board_profile_selects_the_exact_fpga() {
        assert_eq!(LFE5UM5G_85F_EVN.device, "LFE5UM5G-85F-8BG381");
        assert_eq!(LFE5UM5G_85F_EVN.nextpnr_device, "--um5g-85k");
        assert_eq!(LFE5UM5G_85F_EVN.nextpnr_package, "CABGA381");
    }

    #[test]
    fn place_and_route_uses_um5g_85k_cabga381_speed_8() {
        let command = Ecp5Flow::evaluation_board("Top", "build/Top").place_and_route_command();

        assert!(command.args.iter().any(|arg| arg == "--um5g-85k"));
        assert!(
            command
                .args
                .windows(2)
                .any(|args| args == ["--package", "CABGA381"])
        );
        assert!(command.args.windows(2).any(|args| args == ["--speed", "8"]));
        assert!(
            command
                .args
                .windows(2)
                .any(|args| args == ["--freq", "300"])
        );
        assert!(command.args.iter().any(|arg| arg == "--placer-budgets"));
        assert!(command.args.iter().any(|arg| arg == "--tmg-ripup"));
        assert!(command.args.windows(2).any(|args| args == ["--seed", "1"]));
        assert!(
            command
                .args
                .windows(2)
                .any(|args| args == ["--placer-heap-timingweight", "30"])
        );
        assert_eq!(ECP5_QOR_TARGET_MHZ, 300);
    }

    #[test]
    fn physical_feedback_flow_uses_a_detailed_deterministic_draft() {
        let flow = Ecp5Flow::evaluation_board("Top", "build/Top");
        let draft = flow.draft_place_and_route_command();
        let final_run = flow.refined_place_and_route_command();

        assert!(
            draft
                .args
                .iter()
                .any(|arg| arg == "--detailed-timing-report")
        );
        assert!(draft.args.iter().any(|arg| arg == "--timing-allow-fail"));
        assert!(
            draft
                .args
                .windows(2)
                .any(|args| { args == ["--write", "build/Top/design.draft.json"] })
        );
        assert!(
            draft
                .args
                .windows(2)
                .any(|args| { args == ["--report", "build/Top/design.draft-report.json"] })
        );
        assert!(
            final_run
                .args
                .windows(2)
                .any(|args| { args == ["--json", "build/Top/design.refined.json"] })
        );
        assert_eq!(draft.evidence, None);
    }

    #[test]
    fn bitstream_packaging_is_unavailable_without_verification() {
        let flow = Ecp5Flow::evaluation_board("Top", "build/Top");

        assert!(
            flow.pack_command(
                &VerificationReport::new(),
                &VerificationPolicy::safety_critical(),
            )
            .is_err()
        );
    }
}
