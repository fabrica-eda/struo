//! Veryl-authored AXI4 synthesis and simulation design.

use struo_frontend_veryl::{ImportError, analyze_and_lower};
use struo_rtl::Design;

/// Source text of the two-by-two AXI4 crossbar.
pub const AXI4_CROSSBAR_SOURCE: &str = include_str!("../veryl/axi4_crossbar_2x2.veryl");

/// Analyzes the committed Veryl source and lowers its AIR into Struo RTL.
///
/// # Errors
///
/// Returns an error if Veryl analysis or semantic lowering fails.
pub fn axi4_crossbar_2x2() -> Result<Design, ImportError> {
    analyze_and_lower(AXI4_CROSSBAR_SOURCE, "struo_axi4", "Axi4Crossbar2x2")
}

/// Analyzes and flattens the closed-system crossbar verification top.
///
/// # Errors
///
/// Returns an error if Veryl analysis, hierarchy flattening, or RTL validation fails.
pub fn axi4_crossbar_self_test() -> Result<Design, ImportError> {
    analyze_and_lower(AXI4_CROSSBAR_SOURCE, "struo_axi4", "Axi4CrossbarSelfTest")
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use celox::{NativeBackend, SignalRef, Simulator, SimulatorBuilder};
    use struo_celox::ecp5_simulator;
    use struo_synth::synthesize;
    use struo_target_ecp5::map_to_ecp5;

    use super::{axi4_crossbar_2x2, axi4_crossbar_self_test};

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
        ecp5_simulator(&mapped).unwrap().build_native().unwrap();
    }

    #[test]
    fn preserves_axi4_bursts_ids_sidebands_and_backpressure() {
        let _guard = TEST_LOCK.lock().unwrap();
        for mut simulator in [reference_simulator(), crossbar_simulator()] {
            reset(&mut simulator);
            exercise_write_burst(&mut simulator);
            exercise_read_burst(&mut simulator);
            exercise_decode_errors(&mut simulator);
            reset(&mut simulator);
            exercise_burst_footprint_validation(&mut simulator);
            reset(&mut simulator);
            exercise_qos_arbitration(&mut simulator);
        }
    }

    #[test]
    fn alternates_contended_read_grants() {
        let _guard = TEST_LOCK.lock().unwrap();
        for mut simulator in [reference_simulator(), crossbar_simulator()] {
            reset(&mut simulator);
            set_u16(&mut simulator, "s0.araddr", 0x0020);
            set_u16(&mut simulator, "s1.araddr", 0x0024);
            set_u8(&mut simulator, "s0.arvalid", 1);
            set_u8(&mut simulator, "s1.arvalid", 1);
            assert_value(&mut simulator, "s0.arready", 1);
            assert_value(&mut simulator, "s1.arready", 0);
            tick(&mut simulator);
            set_u8(&mut simulator, "s0.arvalid", 0);
            assert_value(&mut simulator, "m0.araddr", 0x0020);
            set_u8(&mut simulator, "m0.arready", 1);
            tick(&mut simulator);
            set_u8(&mut simulator, "m0.arready", 0);
            set_u8(&mut simulator, "m0.rvalid", 1);
            set_u8(&mut simulator, "m0.rlast", 1);
            set_u8(&mut simulator, "s0.rready", 1);
            set_u8(&mut simulator, "s0.arvalid", 1);
            assert_value(&mut simulator, "s0.arready", 0);
            assert_value(&mut simulator, "s1.arready", 1);
            tick(&mut simulator);
            set_u8(&mut simulator, "m0.rvalid", 0);
            set_u8(&mut simulator, "m0.rlast", 0);
            set_u8(&mut simulator, "s0.rready", 0);
            set_u8(&mut simulator, "s0.arvalid", 0);
            set_u8(&mut simulator, "s1.arvalid", 0);
            assert_value(&mut simulator, "m0.araddr", 0x0024);
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

    #[test]
    fn self_test_wrapper_passes_before_and_after_mapping() {
        let _guard = TEST_LOCK.lock().unwrap();
        let design = axi4_crossbar_self_test().unwrap();
        assert!(design.top_module().unwrap().instances().is_empty());
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mapped_simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        let reference_simulator =
            SimulatorBuilder::new(super::AXI4_CROSSBAR_SOURCE, "Axi4CrossbarSelfTest")
                .build_native()
                .unwrap();

        for mut simulator in [reference_simulator, mapped_simulator] {
            reset(&mut simulator);
            for _ in 0..12 {
                if simulator.get(signal(&simulator, "passed")) == 1u8.into() {
                    break;
                }
                tick(&mut simulator);
            }
            assert_value(&mut simulator, "passed", 1);
            assert_value(&mut simulator, "failed", 0);
        }
    }

    fn crossbar_simulator() -> Simulator<NativeBackend> {
        let design = axi4_crossbar_2x2().unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        ecp5_simulator(&mapped).unwrap().build_native().unwrap()
    }

    fn reference_simulator() -> Simulator<NativeBackend> {
        SimulatorBuilder::new(super::AXI4_CROSSBAR_SOURCE, "Axi4Crossbar2x2")
            .build_native()
            .unwrap()
    }

    fn reset(simulator: &mut Simulator<NativeBackend>) {
        set_u8(simulator, "rst_n", 0);
        tick(simulator);
        set_u8(simulator, "rst_n", 1);
    }

    fn exercise_write_burst(simulator: &mut Simulator<NativeBackend>) {
        set_u8(simulator, "s0.awid", 9);
        set_u16(simulator, "s0.awaddr", 0x8020);
        set_u8(simulator, "s0.awlen", 2);
        set_u8(simulator, "s0.awsize", 2);
        set_u8(simulator, "s0.awburst", 1);
        set_u8(simulator, "s0.awlock", 1);
        set_u8(simulator, "s0.awcache", 0b1010);
        set_u8(simulator, "s0.awprot", 0b011);
        set_u8(simulator, "s0.awqos", 7);
        set_u8(simulator, "s0.awregion", 4);
        set_u8(simulator, "s0.awvalid", 1);
        assert_value(simulator, "s0.awready", 1);
        tick(simulator);
        set_u8(simulator, "s0.awvalid", 0);
        assert_value(simulator, "m1.awvalid", 0);

        set_write_beat(simulator, 0, 0x1111_0001, false);
        assert_value(simulator, "s0.wready", 1);
        tick(simulator);
        set_u8(simulator, "s0.wvalid", 0);
        tick(simulator);
        assert_value(simulator, "m1.awvalid", 1);
        assert_value(simulator, "m1.awid", 9);
        assert_value(simulator, "m1.awaddr", 0x8020);
        assert_value(simulator, "m1.awlen", 2);
        assert_value(simulator, "m1.awsize", 2);
        assert_value(simulator, "m1.awburst", 1);
        assert_value(simulator, "m1.awlock", 1);
        assert_value(simulator, "m1.awcache", 0b1010);
        assert_value(simulator, "m1.awprot", 0b011);
        assert_value(simulator, "m1.awqos", 7);
        assert_value(simulator, "m1.awregion", 4);
        assert_value(simulator, "m1.wvalid", 1);
        assert_value(simulator, "m1.wdata", 0x1111_0001);

        set_write_beat(simulator, 0, 0x2222_0002, false);
        set_u8(simulator, "m1.awready", 1);
        set_u8(simulator, "m1.wready", 1);
        assert_value(simulator, "s0.wready", 1);
        tick(simulator);
        set_u8(simulator, "m1.awready", 0);
        assert_value(simulator, "m1.awvalid", 0);
        assert_value(simulator, "m1.wdata", 0x2222_0002);

        set_write_beat(simulator, 0, 0x3333_0003, true);
        tick(simulator);
        assert_value(simulator, "m1.wdata", 0x3333_0003);
        assert_value(simulator, "m1.wlast", 1);
        set_u8(simulator, "s0.wvalid", 0);
        tick(simulator);
        set_u8(simulator, "m1.wready", 0);
        tick(simulator);

        set_u8(simulator, "m1.bid", 9);
        set_u8(simulator, "m1.bresp", 0b10);
        set_u8(simulator, "m1.bvalid", 1);
        assert_value(simulator, "s0.bvalid", 1);
        assert_value(simulator, "s0.bid", 9);
        assert_value(simulator, "s0.bresp", 0b10);
        assert_value(simulator, "m1.bready", 0);
        tick(simulator);
        assert_value(simulator, "s0.bvalid", 1);
        set_u8(simulator, "s0.bready", 1);
        assert_value(simulator, "m1.bready", 1);
        tick(simulator);
        set_u8(simulator, "s0.bready", 0);
        set_u8(simulator, "m1.bvalid", 0);
        assert_value(simulator, "s0.bvalid", 0);
        assert_value(simulator, "s0.awready", 1);
    }

    fn exercise_read_burst(simulator: &mut Simulator<NativeBackend>) {
        set_u8(simulator, "s1.arid", 6);
        set_u16(simulator, "s1.araddr", 0x0040);
        set_u8(simulator, "s1.arlen", 1);
        set_u8(simulator, "s1.arsize", 2);
        set_u8(simulator, "s1.arburst", 1);
        set_u8(simulator, "s1.arlock", 1);
        set_u8(simulator, "s1.arcache", 0b0011);
        set_u8(simulator, "s1.arprot", 0b101);
        set_u8(simulator, "s1.arqos", 8);
        set_u8(simulator, "s1.arregion", 2);
        set_u8(simulator, "s1.arvalid", 1);
        assert_value(simulator, "s1.arready", 1);
        tick(simulator);
        set_u8(simulator, "s1.arvalid", 0);
        assert_value(simulator, "m0.arvalid", 1);
        assert_value(simulator, "m0.arid", 0x16);
        assert_value(simulator, "m0.araddr", 0x0040);
        assert_value(simulator, "m0.arlen", 1);
        assert_value(simulator, "m0.arsize", 2);
        assert_value(simulator, "m0.arburst", 1);
        assert_value(simulator, "m0.arlock", 1);
        assert_value(simulator, "m0.arcache", 0b0011);
        assert_value(simulator, "m0.arprot", 0b101);
        assert_value(simulator, "m0.arqos", 8);
        assert_value(simulator, "m0.arregion", 2);
        set_u8(simulator, "m0.arready", 1);
        tick(simulator);
        set_u8(simulator, "m0.arready", 0);

        set_u8(simulator, "m0.rid", 0x16);
        set_u32(simulator, "m0.rdata", 0xaaaa_0001);
        set_u8(simulator, "m0.rresp", 0);
        set_u8(simulator, "m0.rlast", 0);
        set_u8(simulator, "m0.rvalid", 1);
        assert_value(simulator, "s1.rvalid", 1);
        assert_value(simulator, "s1.rid", 6);
        assert_value(simulator, "s1.rdata", 0xaaaa_0001);
        assert_value(simulator, "m0.rready", 0);
        tick(simulator);
        assert_value(simulator, "s1.rdata", 0xaaaa_0001);
        set_u8(simulator, "s1.rready", 1);
        tick(simulator);

        set_u32(simulator, "m0.rdata", 0xbbbb_0002);
        set_u8(simulator, "m0.rlast", 1);
        assert_value(simulator, "s1.rvalid", 1);
        assert_value(simulator, "s1.rdata", 0xbbbb_0002);
        assert_value(simulator, "s1.rlast", 1);
        tick(simulator);
        set_u8(simulator, "s1.rready", 0);
        set_u8(simulator, "m0.rvalid", 0);
        set_u8(simulator, "m0.rlast", 0);
        assert_value(simulator, "s1.rvalid", 0);
    }

    fn exercise_decode_errors(simulator: &mut Simulator<NativeBackend>) {
        set_u8(simulator, "s0.arid", 3);
        set_u16(simulator, "s0.araddr", 0x6000);
        set_u8(simulator, "s0.arlen", 2);
        set_u8(simulator, "s0.arvalid", 1);
        assert_value(simulator, "s0.arready", 1);
        tick(simulator);
        set_u8(simulator, "s0.arvalid", 0);
        assert_value(simulator, "s0.rvalid", 1);
        assert_value(simulator, "s0.rid", 3);
        assert_value(simulator, "s0.rresp", 0b11);
        assert_value(simulator, "s0.rlast", 0);
        tick(simulator);
        assert_value(simulator, "s0.rvalid", 1);

        set_u8(simulator, "s0.rready", 1);
        for beat in 0..3 {
            assert_value(simulator, "s0.rvalid", 1);
            assert_value(simulator, "s0.rid", 3);
            assert_value(simulator, "s0.rresp", 0b11);
            assert_value(simulator, "s0.rdata", 0);
            assert_value(simulator, "s0.rlast", u64::from(beat == 2));
            tick(simulator);
        }
        set_u8(simulator, "s0.rready", 0);
        assert_value(simulator, "s0.rvalid", 0);

        set_u8(simulator, "s1.awid", 5);
        set_u16(simulator, "s1.awaddr", 0x7000);
        set_u8(simulator, "s1.awlen", 1);
        set_u8(simulator, "s1.awvalid", 1);
        set_write_beat(simulator, 1, 0xdead_0001, false);
        assert_value(simulator, "s1.awready", 1);
        assert_value(simulator, "s1.wready", 1);
        tick(simulator);
        set_u8(simulator, "s1.awvalid", 0);
        set_u8(simulator, "s1.wvalid", 0);
        tick(simulator);

        set_write_beat(simulator, 1, 0xdead_0002, true);
        assert_value(simulator, "s1.wready", 1);
        tick(simulator);
        set_u8(simulator, "s1.wvalid", 0);
        tick(simulator);
        assert_value(simulator, "s1.bvalid", 1);
        assert_value(simulator, "s1.bid", 5);
        assert_value(simulator, "s1.bresp", 0b11);
        tick(simulator);
        assert_value(simulator, "s1.bvalid", 1);
        set_u8(simulator, "s1.bready", 1);
        tick(simulator);
        set_u8(simulator, "s1.bready", 0);
        assert_value(simulator, "s1.bvalid", 0);
    }

    fn exercise_burst_footprint_validation(simulator: &mut Simulator<NativeBackend>) {
        // A legal four-beat WRAP burst remains entirely in target 0.
        set_u8(simulator, "s0.arid", 7);
        set_u16(simulator, "s0.araddr", 0x3ffc);
        set_u8(simulator, "s0.arlen", 3);
        set_u8(simulator, "s0.arsize", 2);
        set_u8(simulator, "s0.arburst", 2);
        set_u8(simulator, "s0.arvalid", 1);
        assert_value(simulator, "s0.arready", 1);
        tick(simulator);
        set_u8(simulator, "s0.arvalid", 0);
        assert_value(simulator, "m0.arvalid", 1);
        assert_value(simulator, "m0.araddr", 0x3ffc);
        assert_value(simulator, "m0.arlen", 3);
        assert_value(simulator, "m0.arburst", 2);
        set_u8(simulator, "m0.arready", 1);
        tick(simulator);
        set_u8(simulator, "m0.arready", 0);

        set_u8(simulator, "m0.rid", 7);
        set_u8(simulator, "m0.rvalid", 1);
        set_u8(simulator, "s0.rready", 1);
        for beat in 0..4 {
            let data = 0x7000_0000 | u32::try_from(beat).unwrap();
            set_u32(simulator, "m0.rdata", data);
            set_u8(simulator, "m0.rlast", u8::from(beat == 3));
            assert_value(simulator, "s0.rid", 7);
            assert_value(simulator, "s0.rdata", u64::from(data));
            tick(simulator);
        }
        set_u8(simulator, "m0.rvalid", 0);
        set_u8(simulator, "m0.rlast", 0);
        set_u8(simulator, "s0.rready", 0);

        // The same start address with INCR would cross the target and 4 KiB
        // boundaries, so it must complete locally instead of reaching m0.
        set_u8(simulator, "s0.arid", 8);
        set_u16(simulator, "s0.araddr", 0x3ffc);
        set_u8(simulator, "s0.arlen", 1);
        set_u8(simulator, "s0.arburst", 1);
        set_u8(simulator, "s0.arvalid", 1);
        assert_value(simulator, "s0.arready", 1);
        tick(simulator);
        set_u8(simulator, "s0.arvalid", 0);
        assert_value(simulator, "m0.arvalid", 0);
        assert_value(simulator, "m1.arvalid", 0);
        consume_decode_error_read(simulator, 8, 2);

        // WRAP lengths other than 2, 4, 8, or 16 beats are illegal.
        set_u8(simulator, "s0.arid", 9);
        set_u16(simulator, "s0.araddr", 0x0100);
        set_u8(simulator, "s0.arlen", 2);
        set_u8(simulator, "s0.arburst", 2);
        set_u8(simulator, "s0.arvalid", 1);
        assert_value(simulator, "s0.arready", 1);
        tick(simulator);
        set_u8(simulator, "s0.arvalid", 0);
        assert_value(simulator, "m0.arvalid", 0);
        consume_decode_error_read(simulator, 9, 3);

        // AXI4 limits FIXED bursts to 16 beats.
        set_u8(simulator, "s0.arid", 10);
        set_u16(simulator, "s0.araddr", 0x0100);
        set_u8(simulator, "s0.arlen", 16);
        set_u8(simulator, "s0.arburst", 0);
        set_u8(simulator, "s0.arvalid", 1);
        assert_value(simulator, "s0.arready", 1);
        tick(simulator);
        set_u8(simulator, "s0.arvalid", 0);
        assert_value(simulator, "m0.arvalid", 0);
        consume_decode_error_read(simulator, 10, 17);
    }

    fn consume_decode_error_read(simulator: &mut Simulator<NativeBackend>, id: u8, beats: usize) {
        set_u8(simulator, "s0.rready", 1);
        for beat in 0..beats {
            assert_value(simulator, "s0.rvalid", 1);
            assert_value(simulator, "s0.rid", u64::from(id));
            assert_value(simulator, "s0.rresp", 0b11);
            assert_value(simulator, "s0.rlast", u64::from(beat + 1 == beats));
            tick(simulator);
        }
        set_u8(simulator, "s0.rready", 0);
        assert_value(simulator, "s0.rvalid", 0);
    }

    fn exercise_qos_arbitration(simulator: &mut Simulator<NativeBackend>) {
        // A higher ARQOS request wins before the round-robin tie breaker.
        set_u8(simulator, "s0.arid", 1);
        set_u16(simulator, "s0.araddr", 0x0100);
        set_u8(simulator, "s0.arsize", 2);
        set_u8(simulator, "s0.arburst", 1);
        set_u8(simulator, "s0.arqos", 1);
        set_u8(simulator, "s0.arvalid", 1);
        set_u8(simulator, "s1.arid", 2);
        set_u16(simulator, "s1.araddr", 0x0104);
        set_u8(simulator, "s1.arsize", 2);
        set_u8(simulator, "s1.arburst", 1);
        set_u8(simulator, "s1.arqos", 9);
        set_u8(simulator, "s1.arvalid", 1);
        assert_value(simulator, "s0.arready", 0);
        assert_value(simulator, "s1.arready", 1);
        tick(simulator);
        set_u8(simulator, "s0.arvalid", 0);
        set_u8(simulator, "s1.arvalid", 0);
        assert_value(simulator, "m0.arvalid", 1);
        assert_value(simulator, "m0.arid", 0x12);
        assert_value(simulator, "m0.araddr", 0x0104);
        set_u8(simulator, "m0.arready", 1);
        tick(simulator);
        set_u8(simulator, "m0.arready", 0);

        set_u8(simulator, "m0.rid", 0x12);
        set_u8(simulator, "m0.rlast", 1);
        set_u8(simulator, "m0.rvalid", 1);
        set_u8(simulator, "s1.rready", 1);
        tick(simulator);
        set_u8(simulator, "m0.rvalid", 0);
        set_u8(simulator, "m0.rlast", 0);
        set_u8(simulator, "s1.rready", 0);

        // AWQOS applies the same policy to contended write streams.
        set_u8(simulator, "s0.awid", 3);
        set_u16(simulator, "s0.awaddr", 0x8100);
        set_u8(simulator, "s0.awsize", 2);
        set_u8(simulator, "s0.awburst", 1);
        set_u8(simulator, "s0.awqos", 2);
        set_u8(simulator, "s0.awvalid", 1);
        set_write_beat(simulator, 0, 0x3000_0003, true);
        set_u8(simulator, "s1.awid", 4);
        set_u16(simulator, "s1.awaddr", 0x8104);
        set_u8(simulator, "s1.awsize", 2);
        set_u8(simulator, "s1.awburst", 1);
        set_u8(simulator, "s1.awqos", 10);
        set_u8(simulator, "s1.awvalid", 1);
        set_write_beat(simulator, 1, 0x4000_0004, true);
        assert_value(simulator, "s0.awready", 1);
        assert_value(simulator, "s1.awready", 1);
        tick(simulator);
        set_u8(simulator, "s0.awvalid", 0);
        set_u8(simulator, "s0.wvalid", 0);
        set_u8(simulator, "s1.awvalid", 0);
        set_u8(simulator, "s1.wvalid", 0);
        tick(simulator);
        assert_value(simulator, "m1.awvalid", 1);
        assert_value(simulator, "m1.awid", 0x14);
        assert_value(simulator, "m1.awaddr", 0x8104);
        assert_value(simulator, "m1.wdata", 0x4000_0004);
    }

    fn exercise_out_of_order_reads(simulator: &mut Simulator<NativeBackend>) {
        issue_single_read(simulator, 0, 1, 0x0100, 0);
        issue_single_read(simulator, 0, 2, 0x8100, 1);

        set_u8(simulator, "s0.arid", 3);
        set_u16(simulator, "s0.araddr", 0x0200);
        set_u8(simulator, "s0.arvalid", 1);
        assert_value(simulator, "s0.arready", 0);

        set_u8(simulator, "m1.rid", 2);
        set_u32(simulator, "m1.rdata", 0x2222_2222);
        set_u8(simulator, "m1.rlast", 1);
        set_u8(simulator, "m1.rvalid", 1);
        assert_value(simulator, "s0.rvalid", 1);
        assert_value(simulator, "s0.rid", 2);
        assert_value(simulator, "s0.rdata", 0x2222_2222);
        assert_value(simulator, "m1.rready", 0);
        tick(simulator);
        set_u8(simulator, "s0.rready", 1);
        assert_value(simulator, "m1.rready", 1);
        tick(simulator);
        set_u8(simulator, "m1.rvalid", 0);
        set_u8(simulator, "s0.rready", 0);

        assert_value(simulator, "s0.arready", 1);
        tick(simulator);
        set_u8(simulator, "s0.arvalid", 0);
        assert_value(simulator, "m0.arvalid", 1);
        assert_value(simulator, "m0.arid", 3);
        set_u8(simulator, "m0.arready", 1);
        tick(simulator);
        set_u8(simulator, "m0.arready", 0);

        set_u8(simulator, "s0.arid", 3);
        set_u16(simulator, "s0.araddr", 0x8200);
        set_u8(simulator, "s0.arvalid", 1);
        assert_value(simulator, "s0.arready", 0);
        set_u8(simulator, "s0.arvalid", 0);

        set_u8(simulator, "m0.rid", 1);
        set_u32(simulator, "m0.rdata", 0x1111_1111);
        set_u8(simulator, "m0.rlast", 1);
        set_u8(simulator, "m0.rvalid", 1);
        set_u8(simulator, "s0.rready", 1);
        assert_value(simulator, "s0.rid", 1);
        tick(simulator);
        set_u8(simulator, "m0.rvalid", 0);

        set_u8(simulator, "m0.rid", 3);
        set_u32(simulator, "m0.rdata", 0x3333_3333);
        set_u8(simulator, "m0.rvalid", 1);
        assert_value(simulator, "s0.rid", 3);
        tick(simulator);
        set_u8(simulator, "m0.rvalid", 0);
        set_u8(simulator, "m0.rlast", 0);
        set_u8(simulator, "s0.rready", 0);
    }

    fn exercise_out_of_order_writes(simulator: &mut Simulator<NativeBackend>) {
        issue_single_write(simulator, 0, 3, 0x0300, 0xaaaa_0003, 0);
        issue_single_write(simulator, 0, 4, 0x8300, 0xbbbb_0004, 1);

        set_u8(simulator, "s0.awid", 5);
        set_u16(simulator, "s0.awaddr", 0x0400);
        set_u8(simulator, "s0.awvalid", 1);
        assert_value(simulator, "s0.awready", 0);

        set_u8(simulator, "m1.bid", 4);
        set_u8(simulator, "m1.bvalid", 1);
        assert_value(simulator, "s0.bvalid", 1);
        assert_value(simulator, "s0.bid", 4);
        assert_value(simulator, "m1.bready", 0);
        tick(simulator);
        set_u8(simulator, "s0.bready", 1);
        assert_value(simulator, "m1.bready", 1);
        tick(simulator);
        set_u8(simulator, "m1.bvalid", 0);
        set_u8(simulator, "s0.bready", 0);

        assert_value(simulator, "s0.awready", 1);
        set_u8(simulator, "s0.awvalid", 0);

        set_u8(simulator, "m0.bid", 3);
        set_u8(simulator, "m0.bvalid", 1);
        set_u8(simulator, "s0.bready", 1);
        assert_value(simulator, "s0.bid", 3);
        tick(simulator);
        set_u8(simulator, "m0.bvalid", 0);
        set_u8(simulator, "s0.bready", 0);
    }

    fn issue_single_read(
        simulator: &mut Simulator<NativeBackend>,
        initiator: usize,
        id: u8,
        address: u16,
        target: usize,
    ) {
        set_u8(simulator, &format!("s{initiator}.arid"), id);
        set_u16(simulator, &format!("s{initiator}.araddr"), address);
        set_u8(simulator, &format!("s{initiator}.arvalid"), 1);
        assert_value(simulator, &format!("s{initiator}.arready"), 1);
        tick(simulator);
        set_u8(simulator, &format!("s{initiator}.arvalid"), 0);
        assert_value(simulator, &format!("m{target}.arvalid"), 1);
        assert_value(
            simulator,
            &format!("m{target}.arid"),
            u64::from(id) | ((initiator as u64) << 4),
        );
        set_u8(simulator, &format!("m{target}.arready"), 1);
        tick(simulator);
        set_u8(simulator, &format!("m{target}.arready"), 0);
    }

    fn issue_single_write(
        simulator: &mut Simulator<NativeBackend>,
        initiator: usize,
        id: u8,
        address: u16,
        data: u32,
        target: usize,
    ) {
        set_u8(simulator, &format!("s{initiator}.awid"), id);
        set_u16(simulator, &format!("s{initiator}.awaddr"), address);
        set_u8(simulator, &format!("s{initiator}.awvalid"), 1);
        set_write_beat(simulator, initiator, data, true);
        assert_value(simulator, &format!("s{initiator}.awready"), 1);
        assert_value(simulator, &format!("s{initiator}.wready"), 1);
        tick(simulator);
        set_u8(simulator, &format!("s{initiator}.awvalid"), 0);
        set_u8(simulator, &format!("s{initiator}.wvalid"), 0);
        tick(simulator);
        assert_value(simulator, &format!("m{target}.awvalid"), 1);
        assert_value(simulator, &format!("m{target}.wvalid"), 1);
        assert_value(
            simulator,
            &format!("m{target}.awid"),
            u64::from(id) | ((initiator as u64) << 4),
        );
        set_u8(simulator, &format!("m{target}.awready"), 1);
        set_u8(simulator, &format!("m{target}.wready"), 1);
        tick(simulator);
        set_u8(simulator, &format!("m{target}.awready"), 0);
        set_u8(simulator, &format!("m{target}.wready"), 0);
    }

    fn set_write_beat(
        simulator: &mut Simulator<NativeBackend>,
        index: usize,
        data: u32,
        last: bool,
    ) {
        set_u32(simulator, &format!("s{index}.wdata"), data);
        set_u8(simulator, &format!("s{index}.wstrb"), 0b1111);
        set_u8(simulator, &format!("s{index}.wlast"), u8::from(last));
        set_u8(simulator, &format!("s{index}.wvalid"), 1);
    }

    fn tick(simulator: &mut Simulator<NativeBackend>) {
        simulator.tick(simulator.event("clk")).unwrap();
    }

    fn set_u8(simulator: &mut Simulator<NativeBackend>, name: &str, value: u8) {
        let signal = signal(simulator, name);
        simulator.modify(|io| io.set(signal, value)).unwrap();
    }

    fn set_u16(simulator: &mut Simulator<NativeBackend>, name: &str, value: u16) {
        let signal = signal(simulator, name);
        simulator.modify(|io| io.set(signal, value)).unwrap();
    }

    fn set_u32(simulator: &mut Simulator<NativeBackend>, name: &str, value: u32) {
        let signal = signal(simulator, name);
        simulator.modify(|io| io.set(signal, value)).unwrap();
    }

    fn assert_value(simulator: &mut Simulator<NativeBackend>, name: &str, expected: u64) {
        assert_eq!(
            simulator.get(signal(simulator, name)),
            expected.into(),
            "{name}"
        );
    }

    fn signal(simulator: &Simulator<NativeBackend>, name: &str) -> SignalRef {
        simulator
            .named_signals()
            .into_iter()
            .find(|signal| signal.name == name)
            .unwrap_or_else(|| panic!("signal `{name}` not found"))
            .signal
    }
}
