//! Example: emit two equivalent 32-bit registered counters as CCU2C carry and
//! LUT ripple implementations for nextpnr comparison.
//!
//! Usage: cargo run -p struo-cli --example carry-benchmark -- [directory]
//!
//! On nextpnr 0.6, LFE5UM5G-85F speed grade 8, seed 1, and a 250 MHz target,
//! the routed carry version reached 472.14 MHz with 38 `TRELLIS_COMB` sites
//! while the LUT-ripple baseline reached 60.22 MHz with 65 sites.

use std::error::Error;

use struo::rtl::{
    BinaryOp, BitWidth, ClockEdge, Constant, Design, Module, Port, PortDirection, Register,
    StateDomain, ValueType,
};
use struo::target::ecp5 as struo_target_ecp5;
use struo::target::ecp5::{ArithmeticMapping, ECP5_QOR_TARGET_MHZ};
use struo::{MappingOptions, ecp5_simulator, map_to_ecp5_with_options, synthesize};

fn main() -> Result<(), Box<dyn Error>> {
    let directory = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "build/carry-benchmark".into());
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

    std::fs::create_dir_all(&directory)?;
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
            .filter(|cell| matches!(cell, struo_target_ecp5::Ecp5Cell::Lut4 { .. }))
            .count();
        let carry_count = mapped
            .cells()
            .iter()
            .filter(|cell| matches!(cell, struo_target_ecp5::Ecp5Cell::Ccu2c { .. }))
            .count();
        let path = std::path::Path::new(&directory).join(format!("{label}.json"));
        std::fs::write(&path, mapped.to_nextpnr_json()?)?;
        println!(
            "{label}: {carry_count} CCU2C, {lut_count} LUT4, {} total cells -> {}",
            mapped.cells().len(),
            path.display()
        );
    }
    Ok(())
}
