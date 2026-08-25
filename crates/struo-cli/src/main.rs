//! Command-line entry point for Struo.

use std::error::Error;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::Parser;
use tracing::Level;
use tracing_subscriber::EnvFilter;

use struo_celox::ecp5_simulator;
use struo_frontend_veryl::analyze_project_and_lower;
use struo_rtl::Design;
use struo_synth::synthesize;
use struo_target_ecp5::{ECP5_QOR_TARGET_MHZ, MappingOptions, map_to_ecp5_with_options};

/// Synthesize a Veryl project to an ECP5 netlist.
#[derive(Parser)]
#[command(name = "struo", version, about, propagate_version = true)]
struct Cli {
    /// Veryl project directory or Veryl.toml.
    project: PathBuf,

    /// Top module name.
    #[arg(short, long)]
    top: String,

    /// Write the mapped nextpnr JSON here.
    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Timing goal in MHz (default: 300).
    #[arg(long)]
    timing_goal_mhz: Option<u32>,

    /// Raise diagnostic logging (-v info to stderr by default; -vv debug).
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,
}

fn init_logging(verbose: u8) {
    let default_filter = match verbose {
        0 => Level::INFO.as_str(),
        1 => Level::DEBUG.as_str(),
        _ => "trace",
    };
    let filter =
        EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new(default_filter));
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .init();
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    init_logging(cli.verbose);
    match run(&cli) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            tracing::error!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: &Cli) -> Result<(), Box<dyn Error>> {
    let timing_goal_mhz = cli.timing_goal_mhz.unwrap_or(ECP5_QOR_TARGET_MHZ);
    if timing_goal_mhz == 0 {
        return Err("timing goal must be greater than zero".into());
    }
    let design = load_design(&cli.project, &cli.top)?;
    synthesize_and_map(&design, &cli.top, timing_goal_mhz, cli.output.as_deref())
}

fn load_design(input: &Path, top: &str) -> Result<Design, Box<dyn Error>> {
    if input.is_dir() {
        return Ok(analyze_project_and_lower(input, top)?);
    }
    if input.file_name().is_some_and(|name| name == "Veryl.toml") {
        return Ok(analyze_project_and_lower(input, top)?);
    }

    Err(format!(
        "input `{}` must be a Veryl project directory or Veryl.toml",
        input.display()
    )
    .into())
}

fn synthesize_and_map(
    design: &Design,
    label: &str,
    timing_goal_mhz: u32,
    mapped_path: Option<&Path>,
) -> Result<(), Box<dyn Error>> {
    let synthesized = synthesize(design)?;
    for report in &synthesized.reports {
        tracing::info!("{}: {}", report.pass, report.message);
    }
    let mapped = map_to_ecp5_with_options(
        &synthesized.netlist,
        MappingOptions {
            timing_goal_mhz,
            ..MappingOptions::default()
        },
    )?;
    log_retiming_decision(&mapped);
    tracing::info!(
        "{label}: {} Boolean nodes, {} registers, {} ECP5 cells, goal {timing_goal_mhz} MHz",
        synthesized.netlist.nodes().len(),
        synthesized.netlist.registers().len(),
        mapped.cells().len()
    );
    ecp5_simulator(&mapped)?.build_native()?;
    if let Some(path) = mapped_path {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, mapped.to_nextpnr_json()?)?;
        println!("{}", path.display());
    }
    Ok(())
}

fn log_retiming_decision(mapped: &struo_target_ecp5::Ecp5Netlist) {
    let decision = mapped.retiming();
    let action = if decision.applied {
        "selected"
    } else {
        "kept original"
    };
    tracing::info!(
        "retiming: {action}; LUT depth {} -> {}, critical register inputs {} -> {}, data period {} -> {} ps, overall period {} -> {} ps, registers {} -> {}, certified moves {}, register merges {}, logic replicas {}, physical rewires {}, dead cells {}, equivalence sign-off {}",
        decision.original_lut_depth,
        decision.selected_lut_depth,
        decision.original_critical_registers,
        decision.selected_critical_registers,
        decision.original_period_ps,
        decision.selected_period_ps,
        decision.original_overall_period_ps,
        decision.selected_overall_period_ps,
        decision.original_registers,
        decision.selected_registers,
        decision.certified_primitive_moves,
        decision.equivalent_register_merges,
        decision.equivalent_logic_replications,
        decision.equivalent_physical_rewires,
        decision.unobservable_cells_removed,
        if decision.equivalence_signed_off {
            "passed"
        } else {
            "failed"
        }
    );
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use clap::{CommandFactory, Parser as _};

    use super::{Cli, load_design};

    #[test]
    fn parses_the_documented_invocation_shapes() {
        Cli::command().debug_assert();

        let cli = Cli::try_parse_from([
            "struo",
            "bench/designs/counter32",
            "--top",
            "counter32",
            "--output",
            "out.json",
            "--timing-goal-mhz",
            "310",
            "-vv",
        ])
        .unwrap();
        assert_eq!(cli.project, Path::new("bench/designs/counter32"));
        assert_eq!(cli.top, "counter32");
        assert_eq!(cli.output.as_deref(), Some(Path::new("out.json")));
        assert_eq!(cli.timing_goal_mhz, Some(310));
        assert_eq!(cli.verbose, 2);

        let defaults = Cli::try_parse_from(["struo", "Veryl.toml", "--top", "Top"]).unwrap();
        assert_eq!(defaults.output, None);
        assert_eq!(defaults.timing_goal_mhz, None);
        assert_eq!(defaults.verbose, 0);

        let project =
            Cli::try_parse_from(["struo", "bench/designs/counter32", "--top", "counter32"])
                .unwrap();
        assert_eq!(project.project, Path::new("bench/designs/counter32"));
        assert!(load_design(Path::new("design.veryl"), "Top").is_err());

        assert!(Cli::try_parse_from(["struo", "project"]).is_err());
        assert!(
            Cli::try_parse_from(["struo", "project", "--top", "T", "--timing-goal-mhz", "0"])
                .is_ok()
        );
    }
}
