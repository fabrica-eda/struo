//! Example: synthesize the closed-system AXI4 board wrapper, optionally with
//! physical feedback from a routed draft.
//!
//! Usage: cargo run -p struo-cli --example axi4-self-test --
//!       [mapped-json-path] [timing-goal-mhz] [draft-report] [draft-placed-json]

use std::error::Error;

use struo::target::ecp5::{ECP5_QOR_TARGET_MHZ, PhysicalFeedback};
use struo::{MappingOptions, ecp5_simulator, map_to_ecp5_with_options, synthesize};
use struo_example_axi4_smartconnect::axi4_crossbar_self_test;

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let mapped_path = args.next();
    let timing_goal_mhz = args
        .next()
        .map(|value| value.parse::<u32>())
        .transpose()?
        .unwrap_or(ECP5_QOR_TARGET_MHZ);
    if timing_goal_mhz == 0 {
        return Err("timing goal must be greater than zero".into());
    }
    let draft_report = args.next();
    let draft_placed_json = args.next();

    let design = axi4_crossbar_self_test()?;
    let synthesized = synthesize(&design)?;
    for report in &synthesized.reports {
        println!("{}: {}", report.pass, report.message);
    }
    let mapped = map_to_ecp5_with_options(
        &synthesized.netlist,
        MappingOptions {
            timing_goal_mhz,
            ..MappingOptions::default()
        },
    )?;
    let (mapped, additional_candidates) = match (draft_report, draft_placed_json) {
        (Some(report), Some(placed)) => {
            let feedback = PhysicalFeedback::from_nextpnr_json(
                &std::fs::read_to_string(report)?,
                &std::fs::read_to_string(placed)?,
            )?;
            let mut candidates = mapped.physical_feedback_candidates(&feedback).into_iter();
            let refined = candidates.next().unwrap_or_else(|| mapped.clone());
            let additional_candidates = candidates.collect::<Vec<_>>();
            println!(
                "physical-feedback: {} candidates, {} equivalent physical rewires",
                additional_candidates.len() + usize::from(refined != mapped),
                refined.retiming().equivalent_physical_rewires,
            );
            (refined, additional_candidates)
        }
        (None, None) => (mapped, Vec::new()),
        (Some(_), None) | (None, Some(_)) => {
            return Err("draft report and placed netlist must be provided together".into());
        }
    };
    let decision = mapped.retiming();
    println!(
        "retiming: {}; certified moves {}, register merges {}, logic replicas {}, equivalence sign-off {}",
        if decision.applied {
            "selected"
        } else {
            "kept original"
        },
        decision.certified_primitive_moves,
        decision.equivalent_register_merges,
        decision.equivalent_logic_replications,
        if decision.equivalence_signed_off {
            "passed"
        } else {
            "failed"
        }
    );
    println!(
        "Veryl AXI4 self-test at {timing_goal_mhz} MHz: {} Boolean nodes, {} registers, {} ECP5 cells",
        synthesized.netlist.nodes().len(),
        synthesized.netlist.registers().len(),
        mapped.cells().len()
    );
    ecp5_simulator(&mapped)?.build_native()?;
    println!("Celox native post-map compile: passed without JSON serialization");
    if let Some(path) = mapped_path {
        std::fs::write(&path, mapped.to_nextpnr_json()?)?;
        println!("mapped JSON: {path}");
        let path = std::path::Path::new(&path);
        let stem = path
            .file_stem()
            .and_then(|stem| stem.to_str())
            .unwrap_or("design");
        for (index, candidate) in additional_candidates.iter().enumerate() {
            let candidate_path =
                path.with_file_name(format!("{stem}.candidate-{}.json", index + 1));
            std::fs::write(&candidate_path, candidate.to_nextpnr_json()?)?;
            println!("physical candidate JSON: {}", candidate_path.display());
        }
    }
    Ok(())
}
