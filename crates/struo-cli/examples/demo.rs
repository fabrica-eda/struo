//! Example: synthesize and simulate the ECP5 EVN blinky end to end.
//!
//! Usage: cargo run -p struo-cli --example demo -- [nextpnr-json-path]

use std::error::Error;

use struo_celox::ecp5_simulator;
use struo_rtl::{
    BinaryOp, BitWidth, ClockEdge, Constant, Design, Module, Polarity, Port, PortDirection,
    Register, Reset, ResetMode, StateDomain, ValueType,
};
use struo_sim::VerificationPolicy;
use struo_synth::synthesize;
use struo_target_ecp5::{Ecp5Flow, map_to_ecp5};

fn main() -> Result<(), Box<dyn Error>> {
    let nextpnr_path = std::env::args().nth(1);
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
        std::fs::write(&path, mapped.to_nextpnr_json()?)?;
        println!("nextpnr JSON: {path}");
    }
    let mut simulator = ecp5_simulator(&mapped)?.build_native()?;
    let clock = simulator.event("clk");
    let reset = simulator.signal("btn");
    let sim_led = simulator.signal("led");
    simulator.modify(|io| io.set(reset, 0u8))?;
    simulator.tick(clock)?;
    simulator.modify(|io| io.set(reset, 1u8))?;
    for _ in 0..65_536 {
        simulator.tick(clock)?;
    }
    if simulator.get(sim_led) != 1u8.into() {
        return Err("post-map Celox simulation produced an unexpected LED value".into());
    }
    println!("Celox native post-map simulation: passed without JSON serialization");
    let flow = Ecp5Flow::evaluation_board("ecp5_evn_blinky", "build/ecp5_evn_blinky");
    println!("target: {} ({})", flow.board.board, flow.board.device);
    println!("clock setup: {}", flow.board.clock_setup);
    println!(
        "release gates: {} required",
        VerificationPolicy::safety_critical().required().len()
    );
    Ok(())
}
