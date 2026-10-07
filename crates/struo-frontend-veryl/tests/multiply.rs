//! End-to-end multiplication width, sign and partial-product regressions.

use struo_celox::ecp5_simulator;
use struo_frontend_veryl::analyze_and_lower;
use struo_synth::synthesize;
use struo_target_ecp5::{Ecp5Cell, map_to_ecp5};

fn check(source: &str, values: &[(u64, u64)], expected: impl Fn(u64, u64) -> u128) -> usize {
    let rtl = analyze_and_lower(source, "dsp_test", "Multiply").unwrap();
    let synthesized = synthesize(&rtl).unwrap();
    let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
    let count = mapped
        .cells()
        .iter()
        .filter(|c| matches!(c, Ecp5Cell::Multiplier { .. }))
        .count();
    let json: serde_json::Value = serde_json::from_str(&mapped.to_nextpnr_json().unwrap()).unwrap();
    let json_dsp_count = json["modules"]["Multiply"]["cells"]
        .as_object()
        .unwrap()
        .values()
        .filter(|c| c["type"] == "MULT18X18D")
        .count();
    assert_eq!(count, json_dsp_count);
    let mut sim = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
    let a = sim.signal("a");
    let b = sim.signal("b");
    let p = sim.signal("p");
    for &(x, y) in values {
        sim.modify(|io| {
            io.set_wide(a, x.into());
            io.set_wide(b, y.into());
        })
        .unwrap();
        let words = sim.get(p).to_u64_digits();
        let actual = u128::from(words.first().copied().unwrap_or(0))
            | (u128::from(words.get(1).copied().unwrap_or(0)) << 64);
        assert_eq!(actual, expected(x, y), "{source}; a={x:x}, b={y:x}");
    }
    count
}

#[test]
fn dsp_products_preserve_context_width_and_wide_partial_products() {
    for bits in [1_u32, 5, 16, 18, 19, 32, 36, 64] {
        let mask = u64::MAX >> (64 - bits);
        let edge = [0, 1, 2, mask / 2, mask / 2 + 1, mask - 1, mask].map(|value| value & mask);
        let mut values = edge
            .iter()
            .flat_map(|a| edge.iter().map(move |b| (*a, *b)))
            .collect::<Vec<_>>();
        let mut rng = 0xf31d_abca_5381_1925_u64;
        for _ in 0..256 {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            let a = rng & mask;
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            values.push((a, rng & mask));
        }
        let source = format!(
            "module Multiply (a: input logic<{bits}>, b: input logic<{bits}>, p: output logic<{}>) {{ assign p = a * b; }}",
            bits * 2
        );
        let count = check(&source, &values, |a, b| u128::from(a) * u128::from(b));
        if bits <= 18 {
            assert_eq!(count, 1);
        }
        let truncated = format!(
            "module Multiply (a: input logic<{bits}>, b: input logic<{bits}>, p: output logic<{bits}>) {{ assign p = a * b; }}"
        );
        check(&truncated, &values, |a, b| {
            u128::from(a.wrapping_mul(b) & mask)
        });
    }
}

#[test]
fn signed_and_mixed_products_preserve_extension_and_truncation() {
    let edge = [0, 1, 2, 0x7fff, 0x8000, 0xfffe, 0xffff];
    let values = edge
        .iter()
        .flat_map(|a| edge.iter().map(move |b| (*a, *b)))
        .collect::<Vec<_>>();
    check(
        "module Multiply (a: input i16, b: input i16, p: output i32) { assign p = a * b; }",
        &values,
        |a, b| {
            let a = i16::from_le_bytes(u16::try_from(a).unwrap().to_le_bytes());
            let b = i16::from_le_bytes(u16::try_from(b).unwrap().to_le_bytes());
            u128::from(u32::from_le_bytes(
                (i32::from(a) * i32::from(b)).to_le_bytes(),
            ))
        },
    );
    check(
        "module Multiply (a: input i16, b: input u16, p: output u32) { assign p = a * b; }",
        &values,
        |a, b| u128::from(a) * u128::from(b),
    );
}

#[test]
fn multiplier_pipeline_reads_pre_edge_operands() {
    let source = "module Multiply (clk: input clock, a: input logic<16>, b: input logic<16>, p: output logic<32>) { var a_q: logic<16>; var b_q: logic<16>; always_ff (clk) { a_q = a; b_q = b; p = a_q * b_q; } }";
    let rtl = analyze_and_lower(source, "dsp_test", "Multiply").unwrap();
    let mapped = map_to_ecp5(&synthesize(&rtl).unwrap().netlist).unwrap();
    let mut sim = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
    let a = sim.signal("a");
    let b = sim.signal("b");
    let p = sim.signal("p");
    let clk = sim.event("clk");
    let mut previous = None;
    for (x, y) in [
        (0_u16, 0_u16),
        (65535, 65535),
        (12345, 54321),
        (32768, 2),
        (1, 65535),
    ] {
        sim.modify(|io| {
            io.set(a, x);
            io.set(b, y);
        })
        .unwrap();
        sim.tick(clk).unwrap();
        if let Some(expected) = previous {
            assert_eq!(sim.get(p), expected);
        }
        previous = Some((u32::from(x) * u32::from(y)).into());
    }
}

// This reduction pipeline caught a Celox SplitCoalescedStores ordering bug:
// a store fragment must not cross a preceding load of the same FF state.
fn wide_pipeline() -> struo_target_ecp5::Ecp5Netlist {
    let rtl = analyze_and_lower(
        include_str!("fixtures/multiply_pipeline.veryl"),
        "dsp_test",
        "MultiplyPipeline",
    )
    .unwrap();
    map_to_ecp5(&synthesize(&rtl).unwrap().netlist).unwrap()
}

fn check_wide_pipeline<B: celox::SimBackend>(mut sim: celox::Simulator<B>) {
    use std::collections::VecDeque;
    let a = sim.signal("a");
    let b = sim.signal("b");
    let product = sim.signal("product");
    let clk = sim.event("clk");
    let edge = [
        0,
        1,
        u64::MAX,
        u64::MAX - 1,
        1 << 63,
        (1 << 32) - 1,
        0x1234_5678_9abc_def0,
    ];
    let mut vectors = edge
        .iter()
        .flat_map(|&x| edge.map(|y| (x, y)))
        .collect::<Vec<_>>();
    let mut random = 0xf053_4321_8834_934a_u64;
    for _ in 0..128 {
        random ^= random << 13;
        random ^= random >> 7;
        random ^= random << 17;
        let x = random;
        random ^= random << 13;
        random ^= random >> 7;
        random ^= random << 17;
        vectors.push((x, random));
    }
    vectors.extend([(0, 0); 6]);
    let mut pending = VecDeque::new();
    for (cycle, (x, y)) in vectors.into_iter().enumerate() {
        sim.modify(|io| {
            io.set(a, x);
            io.set(b, y);
        })
        .unwrap();
        sim.tick(clk).unwrap();
        pending.push_back(u128::from(x) * u128::from(y));
        if pending.len() == 7 {
            let words = sim.get(product).to_u64_digits();
            let got = u128::from(words.first().copied().unwrap_or(0))
                | (u128::from(words.get(1).copied().unwrap_or(0)) << 64);
            assert_eq!(
                got,
                pending.pop_front().unwrap(),
                "cycle={cycle}, current a={x:016x}, b={y:016x}, got={got:032x}"
            );
        }
    }
}

#[test]
fn wide_pipeline_preserves_cycle_alignment_native() {
    let mapped = wide_pipeline();
    check_wide_pipeline(ecp5_simulator(&mapped).unwrap().build_native().unwrap());
}

#[test]
fn wide_pipeline_preserves_cycle_alignment_cranelift() {
    let mapped = wide_pipeline();
    check_wide_pipeline(ecp5_simulator(&mapped).unwrap().build_cranelift().unwrap());
}

#[test]
fn wide_pipeline_preserves_cycle_alignment_wasm() {
    let mapped = wide_pipeline();
    check_wide_pipeline(ecp5_simulator(&mapped).unwrap().build_wasm().unwrap());
}
