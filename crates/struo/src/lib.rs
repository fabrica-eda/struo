//! The single entry point for the Struo logic synthesis toolkit.
//!
//! The default features provide the complete Veryl-to-ECP5 flow, including
//! post-synthesis simulation through Celox. Individual layers remain available
//! as modules so applications do not need to list each implementation crate as
//! a direct dependency.

/// Formal equivalence checking and retiming certificates.
pub use struo_formal as formal;
/// Technology-independent synthesis netlist.
pub use struo_ir as ir;
/// Frontend-independent RTL representation.
pub use struo_rtl as rtl;
/// Verification policies and release gates.
pub use struo_sim as sim;
/// Synthesis pipeline and optimization passes.
pub use struo_synth as synth;
pub use struo_synth::{
    SynthesisError, SynthesisOptions, SynthesisResult, synthesize, synthesize_with_options,
};

/// Veryl project analysis and lowering.
#[cfg(feature = "veryl")]
pub use struo_frontend_veryl as frontend;
#[cfg(feature = "veryl")]
pub use struo_frontend_veryl::{ImportError, analyze_and_lower, analyze_project_and_lower};

/// FPGA technology targets.
#[cfg(feature = "ecp5")]
pub mod target {
    /// Lattice ECP5 mapping and implementation flow.
    pub use struo_target_ecp5 as ecp5;
}
#[cfg(feature = "ecp5")]
pub use struo_target_ecp5::{
    Ecp5Netlist, IoTimingConstraints, JtaggBinding, MappingError, MappingOptions,
    OocClockConstraint, OocPortConstraint, OocTimingConstraints, OpenDrainIo, PllBinding,
    PllOutput, RegisterEnableFanoutConstraint, RegisterEnableFanoutError,
    RegisterEnableFanoutReport, map_to_ecp5, map_to_ecp5_ooc, map_to_ecp5_with_constraints,
    map_to_ecp5_with_jtagg, map_to_ecp5_with_open_drain_ios, map_to_ecp5_with_options,
    map_to_ecp5_with_pll,
};

/// Celox adapter for post-synthesis simulation.
#[cfg(feature = "celox")]
pub use struo_celox as celox;
#[cfg(feature = "celox")]
pub use struo_celox::{CeloxAdapterError, ecp5_frontend_artifact, ecp5_simulator};
