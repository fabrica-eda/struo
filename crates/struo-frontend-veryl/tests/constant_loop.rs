//! End-to-end regression for constant arithmetic in procedural Veryl loops.

use struo_celox::ecp5_simulator;
use struo_frontend_veryl::analyze_and_lower;
use struo_synth::synthesize;
use struo_target_ecp5::{MappingOptions, map_to_ecp5_with_options};

#[test]
fn constant_loop_arithmetic_does_not_create_carry_cells() {
    let rtl = analyze_and_lower(
        include_str!("fixtures/constant_loop.veryl"),
        "constant_loop",
        "Procedural",
    )
    .unwrap();
    let netlist = synthesize(&rtl).unwrap().netlist;
    assert!(netlist.arithmetic().is_empty());
    let mapped = map_to_ecp5_with_options(
        &netlist,
        MappingOptions {
            timing_goal_mhz: 175,
            ..MappingOptions::default()
        },
    )
    .unwrap();
    assert_eq!(mapped.cells().len(), 8);
    let mut sim = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
    let sp = sim.signal("sp");
    let slots = sim.signal("slots");
    for pointer in 0..8_u8 {
        sim.modify(|io| io.set(sp, pointer)).unwrap();
        assert_eq!(sim.get(slots), (1_u8 << ((pointer + 7) % 8)).into());
    }
}
