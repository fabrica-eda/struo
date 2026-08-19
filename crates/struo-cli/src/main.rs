//! Command-line entry point for Struo.

use std::env;
use std::error::Error;
use std::process::ExitCode;

use std::fs;

use struo_celox::ecp5_frontend_artifact;
use struo_rtl::{
    BinaryOp, BitWidth, ClockEdge, Constant, Design, Module, Polarity, Port, PortDirection,
    Register, Reset, ResetMode, StateDomain, ValueType,
};
use struo_sim::VerificationPolicy;
use struo_synth::synthesize;
use struo_target_ecp5::{Ecp5Flow, map_to_ecp5};

const USAGE: &str = "\
Struo FPGA synthesis playground

Usage:
  struo demo [NEXTPNR_JSON] [CELOX_JSON]
                       synthesize the ECP5 EVN blinky and write backend artifacts
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
        Some("demo") => run_demo(env::args().nth(2).as_deref(), env::args().nth(3).as_deref()),
        None | Some("help" | "-h" | "--help") => {
            print!("{USAGE}");
            Ok(())
        }
        Some(command) => Err(format!("unknown command `{command}`\n\n{USAGE}").into()),
    }
}

fn run_demo(nextpnr_path: Option<&str>, celox_path: Option<&str>) -> Result<(), Box<dyn Error>> {
    let bit = ValueType {
        width: BitWidth::new(1)?,
        signed: false,
        state: StateDomain::TwoState,
    };
    let mut module = Module::new("ecp5_evn_blinky");
    let clock = module.add_port(Port {
        name: "clk".into(),
        direction: PortDirection::Input,
        r#type: bit,
    });
    let reset_signal = module.add_port(Port {
        name: "btn".into(),
        direction: PortDirection::Input,
        r#type: bit,
    });
    let led = module.add_port(Port {
        name: "led".into(),
        direction: PortDirection::Output,
        r#type: ValueType {
            width: BitWidth::new(8)?,
            signed: false,
            state: StateDomain::TwoState,
        },
    });
    let counter = module.add_signal(
        "counter",
        ValueType {
            width: BitWidth::new(24)?,
            signed: false,
            state: StateDomain::TwoState,
        },
    );
    let counter_value = module.read(counter)?;
    let one = module.constant(Constant::from_u64(BitWidth::new(24)?, 1));
    let next = module.binary(BinaryOp::Add, counter_value, one)?;
    let zero = module.constant(Constant::from_u64(BitWidth::new(24)?, 0));
    module.add_register(Register {
        name: "counter".into(),
        target: counter,
        next,
        clock,
        edge: ClockEdge::Rising,
        enable: None,
        reset: Some(Reset {
            signal: reset_signal,
            mode: ResetMode::Asynchronous,
            polarity: Polarity::ActiveLow,
            value: zero,
        }),
    })?;
    let visible = module.expression_slice(counter_value, 16, BitWidth::new(8)?)?;
    module.assign(module.whole(led)?, visible)?;
    let mut design = Design::new("ecp5_evn_blinky");
    design.add_module(module);

    let synthesized = synthesize(&design)?;
    println!("design: {}", synthesized.netlist.name());
    for report in &synthesized.reports {
        println!("{}: {}", report.pass, report.message);
    }
    let mapped = map_to_ecp5(&synthesized.netlist)?;
    println!("mapped cells: {}", mapped.cells().len());
    if let Some(path) = nextpnr_path {
        fs::write(path, mapped.to_nextpnr_json()?)?;
        println!("nextpnr JSON: {path}");
    }
    let celox = ecp5_frontend_artifact(&mapped)?;
    println!(
        "Celox artifact: {} signals, {} expressions, {} registers",
        celox.signals().len(),
        celox.expressions().len(),
        celox.registers().len()
    );
    if let Some(path) = celox_path {
        fs::write(path, celox.to_json()?)?;
        println!("Celox JSON: {path}");
    }
    let flow = Ecp5Flow::evaluation_board("ecp5_evn_blinky", "build/ecp5_evn_blinky");
    println!("target: {} ({})", flow.board.board, flow.board.device);
    println!("clock setup: {}", flow.board.clock_setup);
    println!(
        "release gates: {} required",
        VerificationPolicy::safety_critical().required().len()
    );
    Ok(())
}
