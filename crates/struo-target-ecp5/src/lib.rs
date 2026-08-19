//! Lattice ECP5 target descriptions and reproducible tool recipes.

use struo_sim::{
    ReleaseBlocked, SimulationRecipe, SimulationSource, SimulationSourceKind, VerificationPolicy,
    VerificationReport, VerificationStage,
};

/// Board constraints distributed with this crate.
pub const LFE5UM5G_85F_EVN_LPF: &str = include_str!("../../../boards/lfe5um5g-85f-evn/base.lpf");

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
    /// Struo-emitted synthesizable Verilog.
    pub rtl_verilog: String,
    /// Yosys JSON consumed by nextpnr.
    pub synthesized_json: String,
    /// Technology-mapped Verilog used for gate-level simulation.
    pub synthesized_verilog: String,
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
            rtl_verilog: format!("{root}/design.rtl.v"),
            synthesized_json: format!("{root}/design.synth.json"),
            synthesized_verilog: format!("{root}/design.synth.v"),
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
            artifacts: FlowArtifacts::under(artifact_root),
        }
    }

    /// Yosys command that emits both nextpnr JSON and a technology-mapped
    /// Verilog netlist. The latter must not be discarded.
    #[must_use]
    pub fn synthesis_command(&self) -> ToolCommand {
        let script = format!(
            "hierarchy -check -top {top}; synth_ecp5 -top {top} -json {json}; \
             write_verilog -noattr -norename {netlist}",
            top = self.top,
            json = self.artifacts.synthesized_json,
            netlist = self.artifacts.synthesized_verilog,
        );
        ToolCommand {
            program: "yosys",
            args: vec!["-p".into(), script, self.artifacts.rtl_verilog.clone()],
            evidence: None,
        }
    }

    /// Yosys check that rejects missing and black-box primitive models before
    /// gate-level simulation.
    #[must_use]
    pub fn black_box_check_command(&self) -> ToolCommand {
        let script = format!(
            "read_verilog +/ecp5/cells_sim.v; read_verilog {netlist}; \
             hierarchy -check -simcheck -top {top}",
            netlist = self.artifacts.synthesized_verilog,
            top = self.top,
        );
        ToolCommand {
            program: "yosys",
            args: vec!["-p".into(), script],
            evidence: Some(VerificationStage::BlackBoxCheck),
        }
    }

    /// Gate-level simulation inputs. `+/ecp5/cells_sim.v` must be resolved
    /// beneath the active Yosys data directory before invoking Icarus or
    /// Verilator. Black-box-only declarations are intentionally not included.
    #[must_use]
    pub fn post_synthesis_simulation(&self, testbench: impl Into<String>) -> SimulationRecipe {
        SimulationRecipe::post_synthesis(
            self.top.clone(),
            vec![
                SimulationSource {
                    kind: SimulationSourceKind::PrimitiveModels,
                    path: "+/ecp5/cells_sim.v".into(),
                },
                SimulationSource {
                    kind: SimulationSourceKind::SynthesizedNetlist,
                    path: self.artifacts.synthesized_verilog.clone(),
                },
                SimulationSource {
                    kind: SimulationSourceKind::Testbench,
                    path: testbench.into(),
                },
            ],
        )
    }

    /// nextpnr command for the exact UM5G-85K CABGA381 device.
    #[must_use]
    pub fn place_and_route_command(&self) -> ToolCommand {
        ToolCommand {
            program: "nextpnr-ecp5",
            args: vec![
                self.board.nextpnr_device.into(),
                "--package".into(),
                self.board.nextpnr_package.into(),
                "--speed".into(),
                self.board.speed_grade.to_string(),
                "--json".into(),
                self.artifacts.synthesized_json.clone(),
                "--lpf".into(),
                self.artifacts.constraints.clone(),
                "--textcfg".into(),
                self.artifacts.routed_config.clone(),
                "--freq".into(),
                "12".into(),
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
    use struo_sim::{
        SimulationSourceKind, VerificationPolicy, VerificationReport, VerificationStage,
    };

    use super::{Ecp5Flow, LFE5UM5G_85F_EVN, LFE5UM5G_85F_EVN_LPF};

    #[test]
    fn board_profile_selects_the_exact_fpga() {
        assert_eq!(LFE5UM5G_85F_EVN.device, "LFE5UM5G-85F-8BG381");
        assert_eq!(LFE5UM5G_85F_EVN.nextpnr_device, "--um5g-85k");
        assert_eq!(LFE5UM5G_85F_EVN.nextpnr_package, "CABGA381");
        assert!(LFE5UM5G_85F_EVN_LPF.contains("SITE \"A10\""));
    }

    #[test]
    fn synthesized_netlist_is_a_required_simulation_input() {
        let flow = Ecp5Flow::evaluation_board("Top", "build/Top");
        let simulation = flow.post_synthesis_simulation("tests/Top_tb.sv");

        assert_eq!(simulation.stage, VerificationStage::PostSynthesisSimulation);
        assert!(simulation.reject_black_boxes);
        assert!(simulation.sources.iter().any(|source| {
            source.kind == SimulationSourceKind::SynthesizedNetlist
                && source.path.ends_with("design.synth.v")
        }));
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
