//! Command-line entry point for Struo.

use std::env;
use std::error::Error;
use std::process::ExitCode;

use std::fs;

use struo_celox::ecp5_simulator;
use struo_rtl::{
    BinaryOp, BitWidth, ClockEdge, Constant, Design, Module, Polarity, Port, PortDirection,
    Register, Reset, ResetMode, StateDomain, ValueType,
};
use struo_sample_axi4::axi4_crossbar_2x2;
use struo_sim::VerificationPolicy;
use struo_synth::synthesize;
use struo_target_ecp5::{Ecp5Flow, map_to_ecp5};

const USAGE: &str = "\
Struo FPGA synthesis playground

Usage:
  struo demo [NEXTPNR_JSON]
                       synthesize and simulate the ECP5 EVN blinky
  struo axi4-demo [MAPPED_JSON]
                       analyze, synthesize, and compile the 2x2 AXI4 crossbar
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
        Some("demo") => run_demo(env::args().nth(2).as_deref()),
        Some("axi4-demo") => run_axi4_demo(env::args().nth(2).as_deref()),
        None | Some("help" | "-h" | "--help") => {
            print!("{USAGE}");
            Ok(())
        }
        Some(command) => Err(format!("unknown command `{command}`\n\n{USAGE}").into()),
    }
}

fn run_axi4_demo(mapped_path: Option<&str>) -> Result<(), Box<dyn Error>> {
    let design = axi4_crossbar_2x2()?;
    let synthesized = synthesize(&design)?;
    for report in &synthesized.reports {
        println!("{}: {}", report.pass, report.message);
    }
    let mapped = map_to_ecp5(&synthesized.netlist)?;
    println!(
        "Veryl AXI4 crossbar: {} Boolean nodes, {} registers, {} ECP5 cells",
        synthesized.netlist.nodes().len(),
        synthesized.netlist.registers().len(),
        mapped.cells().len()
    );
    ecp5_simulator(&mapped)?.build_cranelift()?;
    println!("Celox post-map compile: passed without JSON serialization");
    if let Some(path) = mapped_path {
        fs::write(path, mapped.to_nextpnr_json()?)?;
        println!("mapped JSON: {path}");
    }
    Ok(())
}

fn run_demo(nextpnr_path: Option<&str>) -> Result<(), Box<dyn Error>> {
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
    let mut simulator = ecp5_simulator(&mapped)?.build_cranelift()?;
    let clock = simulator.event("clk");
    let reset = simulator.signal("btn");
    let led = simulator.signal("led");
    simulator.modify(|io| io.set(reset, 0u8))?;
    simulator.tick(clock)?;
    simulator.modify(|io| io.set(reset, 1u8))?;
    for _ in 0..65_536 {
        simulator.tick(clock)?;
    }
    if simulator.get(led) != 1u8.into() {
        return Err("post-map Celox simulation produced an unexpected LED value".into());
    }
    println!("Celox post-map simulation: passed without JSON serialization");
    let flow = Ecp5Flow::evaluation_board("ecp5_evn_blinky", "build/ecp5_evn_blinky");
    println!("target: {} ({})", flow.board.board, flow.board.device);
    println!("clock setup: {}", flow.board.clock_setup);
    println!(
        "release gates: {} required",
        VerificationPolicy::safety_critical().required().len()
    );
    Ok(())
}
