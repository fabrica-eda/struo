//! Veryl-authored AXI4 synthesis and simulation design.

use struo_frontend_veryl::{ImportError, analyze_and_lower};
use struo_rtl::Design;

/// Source text of the two-by-two AXI4 crossbar.
pub const AXI4_CROSSBAR_SOURCE: &str = include_str!("../veryl/Axi4Crossbar2x2.veryl");

/// Analyzes the committed Veryl source and lowers its AIR into Struo RTL.
///
/// # Errors
///
/// Returns an error if Veryl analysis or semantic lowering fails.
pub fn axi4_crossbar_2x2() -> Result<Design, ImportError> {
    analyze_and_lower(AXI4_CROSSBAR_SOURCE, "struo_axi4", "Axi4Crossbar2x2")
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use celox::{JitBackend, Simulator, SimulatorBuilder};
    use struo_celox::ecp5_simulator;
    use struo_synth::synthesize;
    use struo_target_ecp5::map_to_ecp5;

    use super::axi4_crossbar_2x2;

    static TEST_LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn lowers_synthesizes_and_maps_committed_veryl() {
        let _guard = TEST_LOCK.lock().unwrap();
        let design = axi4_crossbar_2x2().unwrap();
        assert_eq!(design, axi4_crossbar_2x2().unwrap());
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();

        assert!(synthesized.netlist.registers().len() >= 400);
        assert!(mapped.cells().len() > synthesized.netlist.registers().len());
        ecp5_simulator(&mapped).unwrap().build_cranelift().unwrap();
    }

    #[test]
    fn preserves_axi4_bursts_ids_sidebands_and_backpressure() {
        let _guard = TEST_LOCK.lock().unwrap();
        for mut simulator in [reference_simulator(), crossbar_simulator()] {
            reset(&mut simulator);
            exercise_write_burst(&mut simulator);
            exercise_read_burst(&mut simulator);
            exercise_decode_errors(&mut simulator);
        }
    }

    #[test]
    fn alternates_contended_read_grants() {
        let _guard = TEST_LOCK.lock().unwrap();
        for mut simulator in [reference_simulator(), crossbar_simulator()] {
            reset(&mut simulator);
            set_u16(&mut simulator, "s0_araddr", 0x0020);
            set_u16(&mut simulator, "s1_araddr", 0x0024);
            set_u8(&mut simulator, "s0_arvalid", 1);
            set_u8(&mut simulator, "s1_arvalid", 1);
            assert_value(&mut simulator, "s0_arready", 1);
            assert_value(&mut simulator, "s1_arready", 0);
            tick(&mut simulator);
            set_u8(&mut simulator, "s0_arvalid", 0);
            assert_value(&mut simulator, "m0_araddr", 0x0020);
            set_u8(&mut simulator, "m0_arready", 1);
            tick(&mut simulator);
            set_u8(&mut simulator, "m0_arready", 0);
            set_u8(&mut simulator, "m0_rvalid", 1);
            set_u8(&mut simulator, "m0_rlast", 1);
            set_u8(&mut simulator, "s0_rready", 1);
            set_u8(&mut simulator, "s0_arvalid", 1);
            assert_value(&mut simulator, "s0_arready", 0);
            assert_value(&mut simulator, "s1_arready", 1);
            tick(&mut simulator);
            set_u8(&mut simulator, "m0_rvalid", 0);
            set_u8(&mut simulator, "m0_rlast", 0);
            set_u8(&mut simulator, "s0_rready", 0);
            set_u8(&mut simulator, "s0_arvalid", 0);
            set_u8(&mut simulator, "s1_arvalid", 0);
            assert_value(&mut simulator, "m0_araddr", 0x0024);
        }
    }

    #[test]
    fn routes_multiple_outstanding_transactions_by_extended_id() {
        let _guard = TEST_LOCK.lock().unwrap();
        for mut simulator in [reference_simulator(), crossbar_simulator()] {
            reset(&mut simulator);
            exercise_out_of_order_reads(&mut simulator);
            exercise_out_of_order_writes(&mut simulator);
        }
    }

    fn crossbar_simulator() -> Simulator<JitBackend> {
        let design = axi4_crossbar_2x2().unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        ecp5_simulator(&mapped).unwrap().build_cranelift().unwrap()
    }

    fn reference_simulator() -> Simulator<JitBackend> {
        SimulatorBuilder::new(super::AXI4_CROSSBAR_SOURCE, "Axi4Crossbar2x2")
            .build_cranelift()
            .unwrap()
    }

    fn reset(simulator: &mut Simulator<JitBackend>) {
        set_u8(simulator, "rst_n", 0);
        tick(simulator);
        set_u8(simulator, "rst_n", 1);
    }

    fn exercise_write_burst(simulator: &mut Simulator<JitBackend>) {
        set_u8(simulator, "s0_awid", 9);
        set_u16(simulator, "s0_awaddr", 0x8020);
        set_u8(simulator, "s0_awlen", 2);
        set_u8(simulator, "s0_awsize", 2);
        set_u8(simulator, "s0_awburst", 1);
        set_u8(simulator, "s0_awlock", 1);
        set_u8(simulator, "s0_awcache", 0b1010);
        set_u8(simulator, "s0_awprot", 0b011);
        set_u8(simulator, "s0_awqos", 7);
        set_u8(simulator, "s0_awregion", 4);
        set_u8(simulator, "s0_awvalid", 1);
        assert_value(simulator, "s0_awready", 1);
        tick(simulator);
        set_u8(simulator, "s0_awvalid", 0);
        assert_value(simulator, "m1_awvalid", 0);

        set_write_beat(simulator, 0, 0x1111_0001, false);
        assert_value(simulator, "s0_wready", 1);
        tick(simulator);
        set_u8(simulator, "s0_wvalid", 0);
        tick(simulator);
        assert_value(simulator, "m1_awvalid", 1);
        assert_value(simulator, "m1_awid", 9);
        assert_value(simulator, "m1_awaddr", 0x8020);
        assert_value(simulator, "m1_awlen", 2);
        assert_value(simulator, "m1_awsize", 2);
        assert_value(simulator, "m1_awburst", 1);
        assert_value(simulator, "m1_awlock", 1);
        assert_value(simulator, "m1_awcache", 0b1010);
        assert_value(simulator, "m1_awprot", 0b011);
        assert_value(simulator, "m1_awqos", 7);
        assert_value(simulator, "m1_awregion", 4);
        assert_value(simulator, "m1_wvalid", 1);
        assert_value(simulator, "m1_wdata", 0x1111_0001);

        set_write_beat(simulator, 0, 0x2222_0002, false);
        set_u8(simulator, "m1_awready", 1);
        set_u8(simulator, "m1_wready", 1);
        assert_value(simulator, "s0_wready", 1);
        tick(simulator);
        set_u8(simulator, "m1_awready", 0);
        assert_value(simulator, "m1_awvalid", 0);
        assert_value(simulator, "m1_wdata", 0x2222_0002);

        set_write_beat(simulator, 0, 0x3333_0003, true);
        tick(simulator);
        assert_value(simulator, "m1_wdata", 0x3333_0003);
        assert_value(simulator, "m1_wlast", 1);
        set_u8(simulator, "s0_wvalid", 0);
        tick(simulator);
        set_u8(simulator, "m1_wready", 0);
        tick(simulator);

        set_u8(simulator, "m1_bid", 9);
        set_u8(simulator, "m1_bresp", 0b10);
        set_u8(simulator, "m1_bvalid", 1);
        assert_value(simulator, "s0_bvalid", 1);
        assert_value(simulator, "s0_bid", 9);
        assert_value(simulator, "s0_bresp", 0b10);
        assert_value(simulator, "m1_bready", 0);
        tick(simulator);
        assert_value(simulator, "s0_bvalid", 1);
        set_u8(simulator, "s0_bready", 1);
        assert_value(simulator, "m1_bready", 1);
        tick(simulator);
        set_u8(simulator, "s0_bready", 0);
        set_u8(simulator, "m1_bvalid", 0);
        assert_value(simulator, "s0_bvalid", 0);
        assert_value(simulator, "s0_awready", 1);
    }

    fn exercise_read_burst(simulator: &mut Simulator<JitBackend>) {
        set_u8(simulator, "s1_arid", 6);
        set_u16(simulator, "s1_araddr", 0x0040);
        set_u8(simulator, "s1_arlen", 1);
        set_u8(simulator, "s1_arsize", 2);
        set_u8(simulator, "s1_arburst", 1);
        set_u8(simulator, "s1_arlock", 1);
        set_u8(simulator, "s1_arcache", 0b0011);
        set_u8(simulator, "s1_arprot", 0b101);
        set_u8(simulator, "s1_arqos", 8);
        set_u8(simulator, "s1_arregion", 2);
        set_u8(simulator, "s1_arvalid", 1);
        assert_value(simulator, "s1_arready", 1);
        tick(simulator);
        set_u8(simulator, "s1_arvalid", 0);
        assert_value(simulator, "m0_arvalid", 1);
        assert_value(simulator, "m0_arid", 0x16);
        assert_value(simulator, "m0_araddr", 0x0040);
        assert_value(simulator, "m0_arlen", 1);
        assert_value(simulator, "m0_arsize", 2);
        assert_value(simulator, "m0_arburst", 1);
        assert_value(simulator, "m0_arlock", 1);
        assert_value(simulator, "m0_arcache", 0b0011);
        assert_value(simulator, "m0_arprot", 0b101);
        assert_value(simulator, "m0_arqos", 8);
        assert_value(simulator, "m0_arregion", 2);
        set_u8(simulator, "m0_arready", 1);
        tick(simulator);
        set_u8(simulator, "m0_arready", 0);

        set_u8(simulator, "m0_rid", 0x16);
        set_u32(simulator, "m0_rdata", 0xaaaa_0001);
        set_u8(simulator, "m0_rresp", 0);
        set_u8(simulator, "m0_rlast", 0);
        set_u8(simulator, "m0_rvalid", 1);
        assert_value(simulator, "s1_rvalid", 1);
        assert_value(simulator, "s1_rid", 6);
        assert_value(simulator, "s1_rdata", 0xaaaa_0001);
        assert_value(simulator, "m0_rready", 0);
        tick(simulator);
        assert_value(simulator, "s1_rdata", 0xaaaa_0001);
        set_u8(simulator, "s1_rready", 1);
        tick(simulator);

        set_u32(simulator, "m0_rdata", 0xbbbb_0002);
        set_u8(simulator, "m0_rlast", 1);
        assert_value(simulator, "s1_rvalid", 1);
        assert_value(simulator, "s1_rdata", 0xbbbb_0002);
        assert_value(simulator, "s1_rlast", 1);
        tick(simulator);
        set_u8(simulator, "s1_rready", 0);
        set_u8(simulator, "m0_rvalid", 0);
        set_u8(simulator, "m0_rlast", 0);
        assert_value(simulator, "s1_rvalid", 0);
    }

    fn exercise_decode_errors(simulator: &mut Simulator<JitBackend>) {
        set_u8(simulator, "s0_arid", 3);
        set_u16(simulator, "s0_araddr", 0x6000);
        set_u8(simulator, "s0_arlen", 2);
        set_u8(simulator, "s0_arvalid", 1);
        assert_value(simulator, "s0_arready", 1);
        tick(simulator);
        set_u8(simulator, "s0_arvalid", 0);
        assert_value(simulator, "s0_rvalid", 1);
        assert_value(simulator, "s0_rid", 3);
        assert_value(simulator, "s0_rresp", 0b11);
        assert_value(simulator, "s0_rlast", 0);
        tick(simulator);
        assert_value(simulator, "s0_rvalid", 1);

        set_u8(simulator, "s0_rready", 1);
        for beat in 0..3 {
            assert_value(simulator, "s0_rvalid", 1);
            assert_value(simulator, "s0_rid", 3);
            assert_value(simulator, "s0_rresp", 0b11);
            assert_value(simulator, "s0_rdata", 0);
            assert_value(simulator, "s0_rlast", u64::from(beat == 2));
            tick(simulator);
        }
        set_u8(simulator, "s0_rready", 0);
        assert_value(simulator, "s0_rvalid", 0);

        set_u8(simulator, "s1_awid", 5);
        set_u16(simulator, "s1_awaddr", 0x7000);
        set_u8(simulator, "s1_awlen", 1);
        set_u8(simulator, "s1_awvalid", 1);
        set_write_beat(simulator, 1, 0xdead_0001, false);
        assert_value(simulator, "s1_awready", 1);
        assert_value(simulator, "s1_wready", 1);
        tick(simulator);
        set_u8(simulator, "s1_awvalid", 0);
        set_u8(simulator, "s1_wvalid", 0);
        tick(simulator);

        set_write_beat(simulator, 1, 0xdead_0002, true);
        assert_value(simulator, "s1_wready", 1);
        tick(simulator);
        set_u8(simulator, "s1_wvalid", 0);
        tick(simulator);
        assert_value(simulator, "s1_bvalid", 1);
        assert_value(simulator, "s1_bid", 5);
        assert_value(simulator, "s1_bresp", 0b11);
        tick(simulator);
        assert_value(simulator, "s1_bvalid", 1);
        set_u8(simulator, "s1_bready", 1);
        tick(simulator);
        set_u8(simulator, "s1_bready", 0);
        assert_value(simulator, "s1_bvalid", 0);
    }

    fn exercise_out_of_order_reads(simulator: &mut Simulator<JitBackend>) {
        issue_single_read(simulator, 0, 1, 0x0100, 0);
        issue_single_read(simulator, 0, 2, 0x8100, 1);

        set_u8(simulator, "s0_arid", 3);
        set_u16(simulator, "s0_araddr", 0x0200);
        set_u8(simulator, "s0_arvalid", 1);
        assert_value(simulator, "s0_arready", 0);

        set_u8(simulator, "m1_rid", 2);
        set_u32(simulator, "m1_rdata", 0x2222_2222);
        set_u8(simulator, "m1_rlast", 1);
        set_u8(simulator, "m1_rvalid", 1);
        assert_value(simulator, "s0_rvalid", 1);
        assert_value(simulator, "s0_rid", 2);
        assert_value(simulator, "s0_rdata", 0x2222_2222);
        assert_value(simulator, "m1_rready", 0);
        tick(simulator);
        set_u8(simulator, "s0_rready", 1);
        assert_value(simulator, "m1_rready", 1);
        tick(simulator);
        set_u8(simulator, "m1_rvalid", 0);
        set_u8(simulator, "s0_rready", 0);

        assert_value(simulator, "s0_arready", 1);
        tick(simulator);
        set_u8(simulator, "s0_arvalid", 0);
        assert_value(simulator, "m0_arvalid", 1);
        assert_value(simulator, "m0_arid", 3);
        set_u8(simulator, "m0_arready", 1);
        tick(simulator);
        set_u8(simulator, "m0_arready", 0);

        set_u8(simulator, "s0_arid", 3);
        set_u16(simulator, "s0_araddr", 0x8200);
        set_u8(simulator, "s0_arvalid", 1);
        assert_value(simulator, "s0_arready", 0);
        set_u8(simulator, "s0_arvalid", 0);

        set_u8(simulator, "m0_rid", 1);
        set_u32(simulator, "m0_rdata", 0x1111_1111);
        set_u8(simulator, "m0_rlast", 1);
        set_u8(simulator, "m0_rvalid", 1);
        set_u8(simulator, "s0_rready", 1);
        assert_value(simulator, "s0_rid", 1);
        tick(simulator);
        set_u8(simulator, "m0_rvalid", 0);

        set_u8(simulator, "m0_rid", 3);
        set_u32(simulator, "m0_rdata", 0x3333_3333);
        set_u8(simulator, "m0_rvalid", 1);
        assert_value(simulator, "s0_rid", 3);
        tick(simulator);
        set_u8(simulator, "m0_rvalid", 0);
        set_u8(simulator, "m0_rlast", 0);
        set_u8(simulator, "s0_rready", 0);
    }

    fn exercise_out_of_order_writes(simulator: &mut Simulator<JitBackend>) {
        issue_single_write(simulator, 0, 3, 0x0300, 0xaaaa_0003, 0);
        issue_single_write(simulator, 0, 4, 0x8300, 0xbbbb_0004, 1);

        set_u8(simulator, "s0_awid", 5);
        set_u16(simulator, "s0_awaddr", 0x0400);
        set_u8(simulator, "s0_awvalid", 1);
        assert_value(simulator, "s0_awready", 0);

        set_u8(simulator, "m1_bid", 4);
        set_u8(simulator, "m1_bvalid", 1);
        assert_value(simulator, "s0_bvalid", 1);
        assert_value(simulator, "s0_bid", 4);
        assert_value(simulator, "m1_bready", 0);
        tick(simulator);
        set_u8(simulator, "s0_bready", 1);
        assert_value(simulator, "m1_bready", 1);
        tick(simulator);
        set_u8(simulator, "m1_bvalid", 0);
        set_u8(simulator, "s0_bready", 0);

        assert_value(simulator, "s0_awready", 1);
        set_u8(simulator, "s0_awvalid", 0);

        set_u8(simulator, "m0_bid", 3);
        set_u8(simulator, "m0_bvalid", 1);
        set_u8(simulator, "s0_bready", 1);
        assert_value(simulator, "s0_bid", 3);
        tick(simulator);
        set_u8(simulator, "m0_bvalid", 0);
        set_u8(simulator, "s0_bready", 0);
    }

    fn issue_single_read(
        simulator: &mut Simulator<JitBackend>,
        initiator: usize,
        id: u8,
        address: u16,
        target: usize,
    ) {
        set_u8(simulator, &format!("s{initiator}_arid"), id);
        set_u16(simulator, &format!("s{initiator}_araddr"), address);
        set_u8(simulator, &format!("s{initiator}_arvalid"), 1);
        assert_value(simulator, &format!("s{initiator}_arready"), 1);
        tick(simulator);
        set_u8(simulator, &format!("s{initiator}_arvalid"), 0);
        assert_value(simulator, &format!("m{target}_arvalid"), 1);
        assert_value(
            simulator,
            &format!("m{target}_arid"),
            u64::from(id) | ((initiator as u64) << 4),
        );
        set_u8(simulator, &format!("m{target}_arready"), 1);
        tick(simulator);
        set_u8(simulator, &format!("m{target}_arready"), 0);
    }

    fn issue_single_write(
        simulator: &mut Simulator<JitBackend>,
        initiator: usize,
        id: u8,
        address: u16,
        data: u32,
        target: usize,
    ) {
        set_u8(simulator, &format!("s{initiator}_awid"), id);
        set_u16(simulator, &format!("s{initiator}_awaddr"), address);
        set_u8(simulator, &format!("s{initiator}_awvalid"), 1);
        set_write_beat(simulator, initiator, data, true);
        assert_value(simulator, &format!("s{initiator}_awready"), 1);
        assert_value(simulator, &format!("s{initiator}_wready"), 1);
        tick(simulator);
        set_u8(simulator, &format!("s{initiator}_awvalid"), 0);
        set_u8(simulator, &format!("s{initiator}_wvalid"), 0);
        tick(simulator);
        assert_value(simulator, &format!("m{target}_awvalid"), 1);
        assert_value(simulator, &format!("m{target}_wvalid"), 1);
        assert_value(
            simulator,
            &format!("m{target}_awid"),
            u64::from(id) | ((initiator as u64) << 4),
        );
        set_u8(simulator, &format!("m{target}_awready"), 1);
        set_u8(simulator, &format!("m{target}_wready"), 1);
        tick(simulator);
        set_u8(simulator, &format!("m{target}_awready"), 0);
        set_u8(simulator, &format!("m{target}_wready"), 0);
    }

    fn set_write_beat(simulator: &mut Simulator<JitBackend>, index: usize, data: u32, last: bool) {
        set_u32(simulator, &format!("s{index}_wdata"), data);
        set_u8(simulator, &format!("s{index}_wstrb"), 0b1111);
        set_u8(simulator, &format!("s{index}_wlast"), u8::from(last));
        set_u8(simulator, &format!("s{index}_wvalid"), 1);
    }

    fn tick(simulator: &mut Simulator<JitBackend>) {
        simulator.tick(simulator.event("clk")).unwrap();
    }

    fn set_u8(simulator: &mut Simulator<JitBackend>, name: &str, value: u8) {
        let signal = simulator.signal(name);
        simulator.modify(|io| io.set(signal, value)).unwrap();
    }

    fn set_u16(simulator: &mut Simulator<JitBackend>, name: &str, value: u16) {
        let signal = simulator.signal(name);
        simulator.modify(|io| io.set(signal, value)).unwrap();
    }

    fn set_u32(simulator: &mut Simulator<JitBackend>, name: &str, value: u32) {
        let signal = simulator.signal(name);
        simulator.modify(|io| io.set(signal, value)).unwrap();
    }

    fn assert_value(simulator: &mut Simulator<JitBackend>, name: &str, expected: u64) {
        assert_eq!(
            simulator.get(simulator.signal(name)),
            expected.into(),
            "{name}"
        );
    }
}
