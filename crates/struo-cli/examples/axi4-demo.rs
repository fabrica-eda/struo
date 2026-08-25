//! Example: analyze, synthesize, and compile the 2x2 AXI4 crossbar.
//!
//! Usage: cargo run -p struo-cli --example axi4-demo -- [mapped-json-path]

use std::error::Error;

use struo::{ecp5_simulator, map_to_ecp5, synthesize};
use struo_example_axi4_smartconnect::axi4_crossbar_2x2;

fn main() -> Result<(), Box<dyn Error>> {
    let mapped_path = std::env::args().nth(1);
    let design = axi4_crossbar_2x2()?;
    let synthesized = synthesize(&design)?;
    for report in &synthesized.reports {
        println!("{}: {}", report.pass, report.message);
    }
    let mapped = map_to_ecp5(&synthesized.netlist)?;
    let decision = mapped.retiming();
    println!(
        "retiming: {}; certified moves {}, register merges {}, logic replicas {}",
        if decision.applied {
            "selected"
        } else {
            "kept original"
        },
        decision.certified_primitive_moves,
        decision.equivalent_register_merges,
        decision.equivalent_logic_replications,
    );
    println!(
        "Veryl AXI4 crossbar: {} Boolean nodes, {} registers, {} ECP5 cells",
        synthesized.netlist.nodes().len(),
        synthesized.netlist.registers().len(),
        mapped.cells().len()
    );
    ecp5_simulator(&mapped)?.build_native()?;
    println!("Celox native post-map compile: passed without JSON serialization");
    if let Some(path) = mapped_path {
        std::fs::write(&path, mapped.to_nextpnr_json()?)?;
        println!("mapped JSON: {path}");
    }
    Ok(())
}
