//! Lattice ECP5 target descriptions and reproducible tool recipes.

mod mapped;
mod physical;

pub use mapped::{
    ArithmeticMapping, Bit, Control, Ecp5Cell, Ecp5Netlist, MappedPort, MappingError,
    MappingOptions, NextpnrJsonError, PortDirection as MappedPortDirection, Reset,
    RetimingSelection, map_to_ecp5, map_to_ecp5_with_options,
};
pub use physical::{
    PhysicalCriticalPath, PhysicalFeedback, PhysicalLocation, PhysicalNetTiming,
    PhysicalTimingEndpoint,
};

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

/// Maximum number of proof-signed physical-synthesis candidates per draft.
pub const MAX_PHYSICAL_CANDIDATES: usize = 3;

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
    /// Post-route candidate netlist produced from the refined mapping.
    pub refined_placed_json: String,
    /// Detailed timing report for the refined implementation candidate.
    pub refined_report: String,
    /// Routed configuration for the refined implementation candidate.
    pub refined_config: String,
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
            refined_placed_json: format!("{root}/design.refined-placed.json"),
            refined_report: format!("{root}/design.refined-report.json"),
            refined_config: format!("{root}/design.refined.config"),
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
        self.physical_candidate_place_and_route_command(0)
    }

    /// Returns same-seed implementation commands for the bounded physical
    /// candidates emitted by synthesis. `candidate_count` is the number
    /// reported by the synthesis command, not an optimization-mode choice.
    #[must_use]
    pub fn physical_candidate_place_and_route_commands(
        &self,
        candidate_count: usize,
    ) -> Vec<ToolCommand> {
        (0..candidate_count.min(MAX_PHYSICAL_CANDIDATES))
            .map(|index| self.physical_candidate_place_and_route_command(index))
            .collect()
    }

    fn physical_candidate_place_and_route_command(&self, index: usize) -> ToolCommand {
        let (json, config, placed, report) = self.physical_candidate_artifacts(index);
        let mut command = self.place_and_route_command_for(&json, &config);
        command.args.extend([
            "--write".into(),
            placed,
            "--report".into(),
            report,
            "--detailed-timing-report".into(),
            "--timing-allow-fail".into(),
        ]);
        command.evidence = None;
        command
    }

    fn physical_candidate_artifacts(&self, index: usize) -> (String, String, String, String) {
        if index == 0 {
            return (
                self.artifacts.refined_json.clone(),
                self.artifacts.refined_config.clone(),
                self.artifacts.refined_placed_json.clone(),
                self.artifacts.refined_report.clone(),
            );
        }
        let stem = self
            .artifacts
            .refined_json
            .strip_suffix(".json")
            .unwrap_or(&self.artifacts.refined_json);
        let stem = format!("{stem}.candidate-{index}");
        (
            format!("{stem}.json"),
            format!("{stem}.config"),
            format!("{stem}-placed.json"),
            format!("{stem}-report.json"),
        )
    }

    /// Selects the routed draft unless the refined candidate improves every
    /// clock that changed and regresses none of them.
    #[must_use]
    pub fn select_physical_config(
        &self,
        draft: &PhysicalFeedback,
        refined: &PhysicalFeedback,
    ) -> &str {
        if refined.improves_timing_over(draft) {
            &self.artifacts.refined_config
        } else {
            &self.artifacts.draft_config
        }
    }

    /// Selects the best monotonically improving implementation from a routed
    /// draft and the ordered bounded candidate set. Candidates that regress
    /// any reported clock are ignored.
    #[must_use]
    pub fn select_physical_candidate_config(
        &self,
        draft: &PhysicalFeedback,
        candidates: &[PhysicalFeedback],
    ) -> String {
        let mut best = draft;
        let mut selected = self.artifacts.draft_config.clone();
        for (index, candidate) in candidates.iter().take(MAX_PHYSICAL_CANDIDATES).enumerate() {
            if candidate.improves_timing_over(best) {
                best = candidate;
                selected = self.physical_candidate_artifacts(index).1;
            }
        }
        selected
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
        Ok(self.pack_command_for_config(&self.artifacts.routed_config))
    }

    /// Packs the faster of a routed draft and its equivalent refined
    /// candidate, automatically rolling back a timing regression.
    ///
    /// # Errors
    ///
    /// Returns the missing, skipped, or failed release gates.
    pub fn pack_physical_command(
        &self,
        report: &VerificationReport,
        policy: &VerificationPolicy,
        draft: &PhysicalFeedback,
        refined: &PhysicalFeedback,
    ) -> Result<ToolCommand, ReleaseBlocked> {
        report.authorize_bitstream(policy)?;
        Ok(self.pack_command_for_config(self.select_physical_config(draft, refined)))
    }

    /// Packs the best same-seed physical candidate, or the draft when every
    /// candidate is slower.
    ///
    /// # Errors
    ///
    /// Returns the missing, skipped, or failed release gates.
    pub fn pack_physical_candidates_command(
        &self,
        report: &VerificationReport,
        policy: &VerificationPolicy,
        draft: &PhysicalFeedback,
        candidates: &[PhysicalFeedback],
    ) -> Result<ToolCommand, ReleaseBlocked> {
        report.authorize_bitstream(policy)?;
        let config = self.select_physical_candidate_config(draft, candidates);
        Ok(self.pack_command_for_config(&config))
    }

    fn pack_command_for_config(&self, config: &str) -> ToolCommand {
        ToolCommand {
            program: "ecppack",
            args: vec![
                "--svf".into(),
                self.artifacts.svf.clone(),
                config.into(),
                self.artifacts.bitstream.clone(),
            ],
            evidence: None,
        }
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
        assert!(
            final_run
                .args
                .windows(2)
                .any(|args| { args == ["--textcfg", "build/Top/design.refined.config"] })
        );
        assert!(
            final_run
                .args
                .windows(2)
                .any(|args| { args == ["--report", "build/Top/design.refined-report.json"] })
        );
        assert!(
            final_run
                .args
                .windows(2)
                .any(|args| { args == ["--write", "build/Top/design.refined-placed.json"] })
        );
        assert_eq!(draft.evidence, None);
        assert_eq!(final_run.evidence, None);
    }

    #[test]
    fn physical_flow_routes_the_bounded_candidate_set_at_the_same_seed() {
        let flow = Ecp5Flow::evaluation_board("Top", "build/Top");

        let commands = flow.physical_candidate_place_and_route_commands(3);

        assert_eq!(commands.len(), 3);
        for command in &commands {
            assert!(command.args.windows(2).any(|args| args == ["--seed", "1"]));
            assert!(command.args.iter().any(|arg| arg == "--timing-allow-fail"));
        }
        assert!(
            commands[0]
                .args
                .windows(2)
                .any(|args| args == ["--json", "build/Top/design.refined.json"])
        );
        assert!(
            commands[1]
                .args
                .windows(2)
                .any(|args| { args == ["--json", "build/Top/design.refined.candidate-1.json"] })
        );
        assert!(commands[2].args.windows(2).any(|args| {
            args == [
                "--report",
                "build/Top/design.refined.candidate-2-report.json",
            ]
        }));
        assert_eq!(
            flow.physical_candidate_place_and_route_commands(99).len(),
            3
        );
    }

    #[test]
    fn physical_flow_rolls_back_a_slower_candidate() {
        use super::PhysicalFeedback;

        let flow = Ecp5Flow::evaluation_board("Top", "build/Top");
        let placed = r#"{"modules":{"Top":{"cells":{}}}}"#;
        let draft = PhysicalFeedback::from_nextpnr_json(
            r#"{"fmax":{"clk":{"achieved":310.0,"constraint":320.0}}}"#,
            placed,
        )
        .unwrap();
        let slower = PhysicalFeedback::from_nextpnr_json(
            r#"{"fmax":{"clk":{"achieved":305.0,"constraint":320.0}}}"#,
            placed,
        )
        .unwrap();
        let faster = PhysicalFeedback::from_nextpnr_json(
            r#"{"fmax":{"clk":{"achieved":315.0,"constraint":320.0}}}"#,
            placed,
        )
        .unwrap();

        assert_eq!(
            flow.select_physical_config(&draft, &slower),
            "build/Top/design.draft.config"
        );
        assert_eq!(
            flow.select_physical_config(&draft, &faster),
            "build/Top/design.refined.config"
        );
        let fastest = PhysicalFeedback::from_nextpnr_json(
            r#"{"fmax":{"clk":{"achieved":318.0,"constraint":320.0}}}"#,
            placed,
        )
        .unwrap();
        assert_eq!(
            flow.select_physical_candidate_config(&draft, &[slower, faster, fastest]),
            "build/Top/design.refined.candidate-2.config"
        );
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
