//! Lattice ECP5 target descriptions and reproducible tool recipes.

mod mapped;

pub use mapped::{
    Bit, Control, Ecp5Cell, Ecp5Netlist, MappedPort, MappingError,
    PortDirection as MappedPortDirection, Reset, map_to_ecp5,
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
                self.artifacts.mapped_json.clone(),
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
    use struo_sim::{VerificationPolicy, VerificationReport};

    use super::{Ecp5Flow, LFE5UM5G_85F_EVN};

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
