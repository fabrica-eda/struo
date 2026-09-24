//! Reproduce the nested ALU-result hold from Rica through ECP5 mapping.

use celox::SimulatorBuilder;
use struo_celox::ecp5_simulator;
use struo_frontend_veryl::analyze_and_lower;
use struo_synth::synthesize;
use struo_target_ecp5::{MappingOptions, RegisterEnableFanoutConstraint, map_to_ecp5_with_options};

#[test]
fn nested_alu_result_hold_maps_to_clock_enables_without_rewriting_rtl() {
    let source = include_str!("fixtures/nested_enable.veryl");
    let rtl = analyze_and_lower(source, "nested_enable", "NestedEnable").unwrap();
    let netlist = synthesize(&rtl).unwrap().netlist;
    assert_eq!(netlist.registers().len(), 64);
    assert!(netlist.registers().iter().all(|r| r.enable().is_some()));
    let mut mapped = map_to_ecp5_with_options(&netlist, MappingOptions::default()).unwrap();
    let report = mapped
        .apply_register_enable_fanout_constraints(&[RegisterEnableFanoutConstraint::new(
            "ff_result[*]",
            8,
        )])
        .unwrap();
    assert_eq!(report.matched_registers, 64);
    assert_eq!(report.inserted_branches, 8);
    let mut native = SimulatorBuilder::new(source, "NestedEnable")
        .build_native()
        .unwrap();
    let mut physical = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
    let mut expected = 0;
    for cycle in 0..512_u64 {
        let controls = cycle % 16;
        let reset = cycle % 97 != 0;
        let product = 0x1234_5678_9abc_def0 ^ cycle.rotate_left(31);
        let csr = 0xfedc_ba98_7654_3210 ^ cycle;
        let arithmetic = 0x8000_0000_0000_0001_u64.wrapping_mul(cycle);
        if !reset {
            expected = 0;
        } else if controls & 1 != 0 {
            expected = product;
        } else if controls & 2 == 0 || controls & 4 == 0 {
            expected = if controls & 8 != 0 { csr } else { arithmetic };
        }
        for sim in [&mut native, &mut physical] {
            let pins = [
                "rst_n",
                "muldiv_wait",
                "pmp_wait",
                "backend_wait",
                "is_csr",
                "muldiv_result",
                "csr_data",
                "arithmetic",
            ]
            .map(|name| sim.signal(name));
            sim.modify(|io| {
                io.set(pins[0], u8::from(reset));
                for bit in 0..4 {
                    io.set(pins[bit + 1], ((controls >> bit) & 1) as u8);
                }
                io.set(pins[5], product);
                io.set(pins[6], csr);
                io.set(pins[7], arithmetic);
            })
            .unwrap();
            let clk = sim.event("clk");
            sim.tick(clk).unwrap();
            assert_eq!(
                sim.get(sim.signal("result")),
                expected.into(),
                "cycle={cycle}"
            );
        }
    }
}
