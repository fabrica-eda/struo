//! Command-line entry point for Struo.

use std::env;
use std::error::Error;
use std::process::ExitCode;

use std::fs;
use std::path::Path;

use struo_celox::ecp5_simulator;
use struo_example_axi4_smartconnect::{axi4_crossbar_2x2, axi4_crossbar_self_test};
use struo_frontend_veryl::analyze_and_lower;
use struo_rtl::{
    BinaryOp, BitWidth, ClockEdge, Constant, Design, Module, Polarity, Port, PortDirection,
    Register, Reset, ResetMode, StateDomain, ValueType,
};
use struo_sim::VerificationPolicy;
use struo_synth::synthesize;
use struo_target_ecp5::{
    ArithmeticMapping, ECP5_QOR_TARGET_MHZ, Ecp5Cell, Ecp5Flow, MappingOptions, PhysicalFeedback,
    map_to_ecp5, map_to_ecp5_with_options,
};

const USAGE: &str = "\
Struo FPGA synthesis playground

Usage:
  struo demo [NEXTPNR_JSON]
                       synthesize and simulate the ECP5 EVN blinky
  struo axi4-demo [MAPPED_JSON]
                       analyze, synthesize, and compile the 2x2 AXI4 crossbar
  struo axi4-self-test [MAPPED_JSON] [TIMING_GOAL_MHZ] [DRAFT_REPORT] [DRAFT_PLACED_JSON]
                       synthesize the closed-system AXI4 board wrapper
  struo carry-benchmark [DIRECTORY]
                        emit 32-bit CCU2C and LUT ripple comparison designs
  struo qor <VERYL_FILE> <TOP> [NEXTPNR_JSON] [TIMING_GOAL_MHZ]
                        synthesize an arbitrary self-contained Veryl source
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
        Some("axi4-self-test") => run_axi4_self_test(
            env::args().nth(2).as_deref(),
            env::args().nth(3).as_deref(),
            env::args().nth(4).as_deref(),
            env::args().nth(5).as_deref(),
        ),
        Some("carry-benchmark") => run_carry_benchmark(
            env::args()
                .nth(2)
                .as_deref()
                .unwrap_or("build/carry-benchmark"),
        ),
        Some("qor") => run_qor(
            env::args().nth(2).as_deref(),
            env::args().nth(3).as_deref(),
            env::args().nth(4).as_deref(),
            env::args().nth(5).as_deref(),
        ),
        None | Some("help" | "-h" | "--help") => {
            print!("{USAGE}");
            Ok(())
        }
        Some(command) => Err(format!("unknown command `{command}`\n\n{USAGE}").into()),
    }
}

fn run_carry_benchmark(directory: &str) -> Result<(), Box<dyn Error>> {
    println!("ECP5 QoR timing target: {ECP5_QOR_TARGET_MHZ} MHz");
    let width = BitWidth::new(32)?;
    let bit = ValueType {
        width: BitWidth::new(1)?,
        signed: false,
        state: StateDomain::TwoState,
    };
    let word = ValueType {
        width,
        signed: false,
        state: StateDomain::TwoState,
    };
    let mut module = Module::new("carry_benchmark");
    let clock = module.add_port(Port {
        name: "clk".into(),
        direction: PortDirection::Input,
        r#type: bit,
    });
    let count_output = module.add_port(Port {
        name: "count".into(),
        direction: PortDirection::Output,
        r#type: word,
    });
    let count = module.add_signal("count_state", word);
    let current = module.read(count)?;
    let one = module.constant(Constant::from_u64(width, 1));
    let next = module.binary(BinaryOp::Add, current, one)?;
    module.add_register(Register {
        name: "count_state".into(),
        target: count,
        next,
        clock,
        edge: ClockEdge::Rising,
        enable: None,
        reset: None,
    })?;
    module.assign(module.whole(count_output)?, current)?;
    let mut design = Design::new("carry_benchmark");
    design.add_module(module);
    let synthesized = synthesize(&design)?;

    fs::create_dir_all(directory)?;
    for (label, arithmetic) in [
        ("carry", ArithmeticMapping::CarryChain),
        ("lut", ArithmeticMapping::Lut4),
    ] {
        let mapped = map_to_ecp5_with_options(
            &synthesized.netlist,
            MappingOptions {
                arithmetic,
                ..MappingOptions::default()
            },
        )?;
        ecp5_simulator(&mapped)?.build_native()?;
        let lut_count = mapped
            .cells()
            .iter()
            .filter(|cell| matches!(cell, Ecp5Cell::Lut4 { .. }))
            .count();
        let carry_count = mapped
            .cells()
            .iter()
            .filter(|cell| matches!(cell, Ecp5Cell::Ccu2c { .. }))
            .count();
        let path = Path::new(directory).join(format!("{label}.json"));
        fs::write(&path, mapped.to_nextpnr_json()?)?;
        println!(
            "{label}: {carry_count} CCU2C, {lut_count} LUT4, {} total cells -> {}",
            mapped.cells().len(),
            path.display()
        );
    }
    Ok(())
}

fn run_qor(
    veryl_path: Option<&str>,
    top: Option<&str>,
    mapped_path: Option<&str>,
    timing_goal_mhz: Option<&str>,
) -> Result<(), Box<dyn Error>> {
    let Some(veryl_path) = veryl_path else {
        return Err("qor requires a Veryl source path".into());
    };
    let Some(top) = top else {
        return Err("qor requires a top module name".into());
    };
    let timing_goal_mhz = timing_goal_mhz
        .map(str::parse)
        .transpose()?
        .unwrap_or(ECP5_QOR_TARGET_MHZ);
    if timing_goal_mhz == 0 {
        return Err("timing goal must be greater than zero".into());
    }
    let source = fs::read_to_string(veryl_path)?;
    let project = Path::new(veryl_path)
        .parent()
        .and_then(|parent| parent.file_name())
        .and_then(|name| name.to_str())
        .map_or_else(|| "bench".to_owned(), str::to_owned);
    let design = analyze_and_lower(&source, &project, top)?;
    run_qor_design(&design, top, timing_goal_mhz, mapped_path)
}

fn run_qor_design(
    design: &struo_rtl::Design,
    label: &str,
    timing_goal_mhz: u32,
    mapped_path: Option<&str>,
) -> Result<(), Box<dyn Error>> {
    let synthesized = synthesize(design)?;
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
    print_retiming_decision(&mapped);
    println!(
        "{label}: {} Boolean nodes, {} registers, {} ECP5 cells, goal {timing_goal_mhz} MHz",
        synthesized.netlist.nodes().len(),
        synthesized.netlist.registers().len(),
        mapped.cells().len()
    );
    ecp5_simulator(&mapped)?.build_native()?;
    if let Some(path) = mapped_path {
        fs::write(path, mapped.to_nextpnr_json()?)?;
        println!("nextpnr JSON: {path}");
    }
    Ok(())
}

fn run_axi4_self_test(
    mapped_path: Option<&str>,
    timing_goal_mhz: Option<&str>,
    draft_report: Option<&str>,
    draft_placed_json: Option<&str>,
) -> Result<(), Box<dyn Error>> {
    let timing_goal_mhz = timing_goal_mhz
        .map(str::parse)
        .transpose()?
        .unwrap_or(ECP5_QOR_TARGET_MHZ);
    if timing_goal_mhz == 0 {
        return Err("timing goal must be greater than zero".into());
    }
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
                &fs::read_to_string(report)?,
                &fs::read_to_string(placed)?,
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
            return Err("draft report and placed JSON must be provided together".into());
        }
    };
    print_retiming_decision(&mapped);
    println!(
        "Veryl AXI4 self-test at {timing_goal_mhz} MHz: {} Boolean nodes, {} registers, {} ECP5 cells",
        synthesized.netlist.nodes().len(),
        synthesized.netlist.registers().len(),
        mapped.cells().len()
    );
    ecp5_simulator(&mapped)?.build_native()?;
    println!("Celox native post-map compile: passed without JSON serialization");
    if let Some(path) = mapped_path {
        fs::write(path, mapped.to_nextpnr_json()?)?;
        println!("mapped JSON: {path}");
        let path = Path::new(path);
        let stem = path
            .file_stem()
            .and_then(|stem| stem.to_str())
            .unwrap_or("design");
        for (index, candidate) in additional_candidates.iter().enumerate() {
            let candidate_path =
                path.with_file_name(format!("{stem}.candidate-{}.json", index + 1));
            fs::write(&candidate_path, candidate.to_nextpnr_json()?)?;
            println!("physical candidate JSON: {}", candidate_path.display());
        }
    }
    Ok(())
}

fn run_axi4_demo(mapped_path: Option<&str>) -> Result<(), Box<dyn Error>> {
    let design = axi4_crossbar_2x2()?;
    let synthesized = synthesize(&design)?;
    for report in &synthesized.reports {
        println!("{}: {}", report.pass, report.message);
    }
    let mapped = map_to_ecp5(&synthesized.netlist)?;
    print_retiming_decision(&mapped);
    println!(
        "Veryl AXI4 crossbar: {} Boolean nodes, {} registers, {} ECP5 cells",
        synthesized.netlist.nodes().len(),
        synthesized.netlist.registers().len(),
        mapped.cells().len()
    );
    ecp5_simulator(&mapped)?.build_native()?;
    println!("Celox native post-map compile: passed without JSON serialization");
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
    print_retiming_decision(&mapped);
    println!("mapped cells: {}", mapped.cells().len());
    if let Some(path) = nextpnr_path {
        fs::write(path, mapped.to_nextpnr_json()?)?;
        println!("nextpnr JSON: {path}");
    }
    let mut simulator = ecp5_simulator(&mapped)?.build_native()?;
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

fn print_retiming_decision(mapped: &struo_target_ecp5::Ecp5Netlist) {
    let decision = mapped.retiming();
    let action = if decision.applied {
        "selected"
    } else {
        "kept original"
    };
    println!(
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
