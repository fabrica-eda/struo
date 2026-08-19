//! Command-line entry point for Struo.

use std::env;
use std::error::Error;
use std::process::ExitCode;

use struo_ir::Netlist;
use struo_sim::VerificationPolicy;
use struo_synth::default_pipeline;
use struo_target_ecp5::Ecp5Flow;

const USAGE: &str = "\
Struo FPGA synthesis playground

Usage:
  struo demo    run the built-in AND-gate design
  struo help    show this help
";

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    match env::args().nth(1).as_deref() {
        Some("demo") => run_demo(),
        None | Some("help" | "-h" | "--help") => {
            print!("{USAGE}");
            Ok(())
        }
        Some(command) => Err(format!("unknown command `{command}`\n\n{USAGE}").into()),
    }
}

fn run_demo() -> Result<(), Box<dyn Error>> {
    let mut design = Netlist::new("and_gate");
    let lhs = design.add_input("a");
    let rhs = design.add_input("b");
    let result = design.add_and(lhs, rhs);
    design.add_output("y", result);

    println!("design: {}", design.name());
    for report in default_pipeline().run(&mut design)? {
        println!("{}: {}", report.pass, report.message);
    }
    let flow = Ecp5Flow::evaluation_board("and_gate", "build/and_gate");
    println!("target: {} ({})", flow.board.board, flow.board.device);
    println!("clock setup: {}", flow.board.clock_setup);
    println!(
        "release gates: {} required",
        VerificationPolicy::safety_critical().required().len()
    );
    Ok(())
}
