//! Run the upstream corpus through Struo lowering, synthesis and ECP5 mapping.
use std::cell::RefCell;
use std::rc::Rc;

use celox::{NativeBackend, Simulator};
use celox_test_suite_veryl::{Backend, BigUint, CompilationRejected, Design, Result, SignalPath};
use struo_frontend_veryl::{ImportError, analyze_and_lower};

type Stage = Rc<RefCell<String>>;

struct MappedBackend {
    sim: Simulator<NativeBackend>,
    stage: Stage,
    ports: Option<std::collections::BTreeMap<String, usize>>,
    eliminated_events: std::collections::BTreeSet<String>,
}

impl MappedBackend {
    fn signals(&self, path: &SignalPath) -> Result<Vec<(celox::SignalRef, usize)>> {
        if path.instances.is_empty() {
            if let Some(ports) = &self.ports {
                if let Some(width) = ports.get(&path.name) {
                    return Ok(vec![(self.sim.signal(&path.name), *width)]);
                }
                let prefix = format!("{}[", path.name);
                let mut elements = ports
                    .iter()
                    .filter_map(|(name, width)| {
                        let suffix = name.strip_prefix(&prefix)?;
                        let indices = suffix
                            .trim_end_matches(']')
                            .split("][")
                            .map(str::parse::<usize>)
                            .collect::<std::result::Result<Vec<_>, _>>()
                            .ok()?;
                        Some((indices, self.sim.signal(name), *width))
                    })
                    .collect::<Vec<_>>();
                elements.sort_by(|a, b| a.0.cmp(&b.0));
                if !elements.is_empty() {
                    return Ok(elements
                        .into_iter()
                        .map(|(_, signal, width)| (signal, width))
                        .collect());
                }
            } else {
                // The source simulator already exposes packed array handles.
                return Ok(vec![(self.sim.signal(&path.name), 0)]);
            }
        }
        *self.stage.borrow_mut() = "adapter_unsupported".into();
        Err(
            format!("internal/hierarchical observation is not preserved by synthesis: {path:?}")
                .into(),
        )
    }
}

impl Backend for MappedBackend {
    fn write(&mut self, path: &SignalPath, payload: BigUint, mask: BigUint) -> Result<()> {
        *self.stage.borrow_mut() = "runtime_error".into();
        if mask != BigUint::default() {
            *self.stage.borrow_mut() = "adapter_unsupported".into();
            return Err("ECP5 simulation is two-state".into());
        }
        let mut offset = 0;
        for (signal, width) in self.signals(path)? {
            let value = if width == 0 {
                payload.clone()
            } else {
                (&payload >> offset) & ((BigUint::from(1u8) << width) - 1u8)
            };
            self.sim.set_wide(signal, value);
            offset += width;
        }
        *self.stage.borrow_mut() = "assertion_failure".into();
        Ok(())
    }

    fn read(&mut self, path: &SignalPath) -> Result<(BigUint, BigUint)> {
        *self.stage.borrow_mut() = "runtime_error".into();
        let signals = self.signals(path)?;
        self.sim.eval_comb().map_err(|e| format!("{e:?}"))?;
        let mut result = BigUint::default();
        let mut offset = 0;
        for (signal, width) in signals {
            result |= self.sim.get(signal) << offset;
            offset += width;
        }
        *self.stage.borrow_mut() = "assertion_failure".into();
        Ok((result, BigUint::default()))
    }

    fn eval_comb(&mut self) -> Result<()> {
        *self.stage.borrow_mut() = "runtime_error".into();
        self.sim.eval_comb().map_err(|e| format!("{e:?}"))?;
        *self.stage.borrow_mut() = "assertion_failure".into();
        Ok(())
    }

    fn tick(&mut self, event: &str) -> Result<()> {
        *self.stage.borrow_mut() = "runtime_error".into();
        if self
            .ports
            .as_ref()
            .is_some_and(|ports| !ports.contains_key(event))
        {
            return Err(format!("unknown top-level event: {event}").into());
        }
        self.sim.eval_comb().map_err(|e| format!("{e:?}"))?;
        // A fully combinational mapped design has no event handlers. Only
        // source clocks proven to have lost all state may become no-op ticks.
        if !self.eliminated_events.contains(event) {
            self.sim
                .tick(self.sim.event(event))
                .map_err(|e| format!("{e:?}"))?;
        }
        *self.stage.borrow_mut() = "assertion_failure".into();
        Ok(())
    }
}

fn report_elapsed(label: &str, timer: &mut std::time::Instant) {
    if std::env::var_os("STRUO_VERYL_TIMING").is_some() {
        eprintln!("STRUO_TIMING {label} {:.6}", timer.elapsed().as_secs_f64());
    }
    *timer = std::time::Instant::now();
}

fn compile(design: &Design, stage: &Stage) -> Result<Box<dyn Backend>> {
    if design.four_state {
        *stage.borrow_mut() = "adapter_unsupported".into();
        return Err("ECP5 simulation does not implement four-state storage".into());
    }
    *stage.borrow_mut() = "lowering_error".into();
    let source = design
        .sources
        .iter()
        .map(|s| s.text.as_str())
        .collect::<Vec<_>>()
        .join("\n");
    if std::env::var_os("STRUO_VERYL_REFERENCE").is_some() {
        *stage.borrow_mut() = "source_simulator_build_error".into();
        let sim = celox::SimulatorBuilder::new(&source, &design.top).build_native()?;
        *stage.borrow_mut() = "assertion_failure".into();
        return Ok(Box::new(MappedBackend {
            sim,
            stage: stage.clone(),
            ports: None,
            eliminated_events: std::collections::BTreeSet::default(),
        }));
    }
    let mut timer = std::time::Instant::now();
    let rtl = analyze_and_lower(&source, "suite", &design.top).map_err(|error| {
        if let ImportError::AnalysisFailed(diagnostic) = error {
            *stage.borrow_mut() = "analysis_error".into();
            Box::new(CompilationRejected(diagnostic)) as celox_test_suite_veryl::Error
        } else {
            error.to_string().into()
        }
    })?;
    report_elapsed("lowering", &mut timer);
    *stage.borrow_mut() = "synthesis_error".into();
    let netlist = struo_synth::synthesize(&rtl)?.netlist;
    report_elapsed("synthesis", &mut timer);
    *stage.borrow_mut() = "mapping_error".into();
    let mapped = struo_target_ecp5::map_to_ecp5(&netlist)?;
    report_elapsed("mapping", &mut timer);
    *stage.borrow_mut() = "simulator_build_error".into();
    let ports = mapped
        .ports()
        .iter()
        .map(|p| (p.name.clone(), p.bits.len()))
        .collect();
    let builder = struo_celox::ecp5_simulator(&mapped)?;
    report_elapsed("simulation_ir", &mut timer);
    let sim = builder.build_native()?;
    report_elapsed("native_build", &mut timer);
    let eliminated_events = if sim.named_events().is_empty()
        && mapped.cells().iter().all(|cell| {
            !matches!(
                cell,
                struo_target_ecp5::Ecp5Cell::FlipFlop { .. }
                    | struo_target_ecp5::Ecp5Cell::BlockRam { .. }
            )
        }) {
        let top = rtl.top_module().expect("lowered top module");
        top.registers()
            .iter()
            .map(|register| {
                top.signals()[register.clock.index() as usize]
                    .name()
                    .to_owned()
            })
            .collect()
    } else {
        std::collections::BTreeSet::default()
    };
    *stage.borrow_mut() = "assertion_failure".into();
    Ok(Box::new(MappedBackend {
        sim,
        stage: stage.clone(),
        ports: Some(ports),
        eliminated_events,
    }))
}

/// Isolated worker invoked by scripts/check-veryl-suite.py.
#[test]
#[ignore = "invoked per case by scripts/check-veryl-suite.py"]
fn corpus_case() {
    if std::env::var_os("STRUO_VERYL_TIMING").is_some() {
        let _ = tracing_subscriber::fmt()
            .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
            .with_writer(std::io::stderr)
            .try_init();
    }
    let name = std::env::var("STRUO_VERYL_CASE").expect("STRUO_VERYL_CASE");
    let case = celox_test_suite_veryl::case(&name).expect("unknown corpus case");
    let stage = Rc::new(RefCell::new("setup_error".into()));
    let mut timer = std::time::Instant::now();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        case.run(&mut |design| compile(design, &stage));
    }));
    report_elapsed("case_total", &mut timer);
    let status = if result.is_ok() {
        if case.expectation == celox_test_suite_veryl::Expectation::CompilationError {
            "rejected".to_owned()
        } else {
            "passed".to_owned()
        }
    } else {
        stage.borrow().clone()
    };
    println!(
        "STRUO_RESULT {}",
        serde_json::json!({"name": name, "category": format!("{:?}", case.category), "status": status})
    );
    assert!(result.is_ok(), "{name}: {status}");
}

#[test]
#[ignore = "catalogue for scripts/check-veryl-suite.py"]
fn corpus_list() {
    for case in celox_test_suite_veryl::cases() {
        println!(
            "STRUO_CASE {}",
            serde_json::json!({
                "name": case.name,
                "tags": case.tags.iter().map(|tag| tag.as_str()).collect::<Vec<_>>(),
                "tag_reasons": case.tags.iter().map(|tag| tag.reason()).collect::<Vec<_>>(),
                "stronger_than_sv": case.has_stronger_than_sv_expectations(),
            })
        );
    }
}

#[test]
fn corpus_smoke() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "operators::test_bitwise_operations",
        "concatenation::test_rhs_concatenation_execution",
        "counter::test_counter_n4_basic",
        "data_access::test_dynamic_index_read",
        "data_access::test_dynamic_index_write",
        "flip_flop::test_ff_swap_correctness",
        "reset_edge_cases::test_reset_async_high",
        "reset_edge_cases::test_reset_sync_low",
        "multi_clock::test_separate_resets_per_domain",
        "wide_operators::test_wide_ff_accumulator",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn corpus_width_and_signedness_regressions() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "context_width::test_runtime_variable_width3_subtraction",
        "context_width::test_ff_width_propagation",
        "expression_semantics::folded_and_runtime_builtin_selects_are_unsigned",
        "expression_semantics::signed_type_cast_keeps_comparison_operands_signed",
        "self_determination::test_concatenation_constant_self_determination",
        "std_edge_detector::test_edge_detector_multibit",
        "system_function::test_direct_comb_bits_system_function",
        "system_function::test_direct_comb_bits_type_system_function",
        "wide_context_width::test_wide_context_addition_carry",
        "wide_context_width::test_wide_context_subtraction_underflow",
        "wide_context_width::test_wide_context_constant_folding",
        "wide_context_width::test_wide_context_constant_folding_128bit",
        "wide_context_width::test_wide_context_addition_mixed_boundary",
        "flip_flop::test_ff_function_call_part_select_of_signed_formal_is_unsigned",
        "hierarchy::test_instance_input_port_assignment_width_context",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn corpus_unpacked_array_output_slice() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case("hierarchy::test_instance_unpacked_array_slice_output")
        .unwrap()
        .run(&mut |design| compile(design, &stage));
}

#[test]
fn array_output_slices_preserve_element_order_and_neighbors() {
    let stage = Rc::new(RefCell::new(String::new()));
    for (selection, count) in [("1+:2", 2), ("2-:2", 2), ("1:2", 2), ("1+:1", 1)] {
        let neighbor = if count == 1 {
            "assign data[2] = 8'h7e;"
        } else {
            ""
        };
        let code = format!(
            r"
            module Child #(param N: u32 = 2) (
                i_data: input logic<8>, o_data: output logic<8>[N]
            ) {{
                always_comb {{
                    for j in 0..N {{ o_data[j] = i_data + j as 8; }}
                }}
            }}
            module Packed(i: input logic<8>, o: output logic<16>) {{
                assign o = {{i + 8'd1, i}};
            }}
            module Top(i: input logic<8>, o: output logic<32>, o_packed: output logic<16>) {{
                var data: logic<8>[4];
                assign data[0] = 8'h5a;
                assign data[3] = 8'ha5;
                {neighbor}
                inst child: Child #(N: {count}) (i_data: i, o_data: data[{selection}]);
                inst packed_child: Packed(i, o: {{o_packed[7:0], o_packed[15:8]}});
                assign o = {{data[3], data[2], data[1], data[0]}};
            }}
            "
        );
        let design = Design::new(&code, "Top");
        let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
        let i = sim.signal("i");
        let o = sim.signal("o");
        let packed = sim.signal("o_packed");
        for value in 0..=255u8 {
            sim.modify(|io| io.set(i, value)).unwrap();
            let second = if count == 1 {
                0x7e
            } else {
                value.wrapping_add(1)
            };
            let expected = 0xa500_005a_u32 | (u32::from(second) << 16) | (u32::from(value) << 8);
            assert_eq!(
                sim.get(o),
                expected.into(),
                "slice {selection}, input {value}"
            );
            let expected_packed = (u16::from(value) << 8) | u16::from(value.wrapping_add(1));
            assert_eq!(sim.get(packed), expected_packed.into());
        }
    }
}

#[test]
fn array_output_slices_reject_element_count_and_width_mismatches() {
    for (width, count) in [(8, 3), (4, 2), (4, 4)] {
        let source = format!(
            "module Child(o: output logic<8>[2]) {{
                assign o[0] = 8'h12; assign o[1] = 8'h34;
            }}
            module Top(o: output logic<{width}>[{count}]) {{
                inst child: Child(o: o[0+:{count}]);
            }}"
        );
        let error = analyze_and_lower(&source, "array_output_mismatch", "Top").unwrap_err();
        assert!(
            matches!(&error, ImportError::UnsupportedBehavior(message)
                if message.contains("array instance output"))
                || matches!(&error, ImportError::AnalysisFailed(message)
                    if message.contains("MismatchAssignment")),
            "unexpected rejection for {count} elements of width {width}: {error}"
        );
    }
}

#[test]
fn corpus_open_output_ports() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "hierarchy::test_unconnected_child_output_needs_no_parent_glue",
        "std_binary_codec::test_binary_encoder",
        "std_binary_codec::test_binary_encoder_disabled",
        "std_binary_codec::test_binary_codec_roundtrip",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn open_output_ports_preserve_child_internal_reads() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Child(i: input logic<4>, unused: output logic<4>,
                     unused_array: output logic<4>[2], o: output logic<4>) {
            assign unused = ~i;
            assign unused_array[0] = i;
            assign unused_array[1] = unused;
            assign o = unused_array[1] ^ 4'h5;
        }
        module Top(i: input logic<4>, explicit: output logic<4>,
                   omitted: output logic<4>) {
            inst explicit_open: Child(i, unused: _, unused_array: _, o: explicit);
            inst omitted_open: Child(i, o: omitted);
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let i = sim.signal("i");
    let explicit = sim.signal("explicit");
    let omitted = sim.signal("omitted");
    for value in 0..16u8 {
        sim.modify(|io| io.set(i, value)).unwrap();
        let expected = ((!value & 0xf) ^ 5).into();
        assert_eq!(sim.get(explicit), expected);
        assert_eq!(sim.get(omitted), expected);
    }
}

#[test]
fn corpus_disjoint_combinational_drivers() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "basic::test_mixed_selects_execution",
        "data_access::test_partial_write_merging",
        "false_loop::test_read_then_overwrite_convergence",
        "std_lfsr::test_lfsr_basic_shift",
        "std_lfsr::test_lfsr_deterministic_cycle",
        "std_lfsr::test_lfsr_enable_hold",
        "hierarchy::test_hierarchical_concat_feedback_runtime",
        "hierarchy::test_hierarchical_concat_feedback_runtime_multi_observe",
        "hierarchy::test_hierarchical_concat_feedback_with_constant_middle_bit",
        "hierarchy::test_hierarchical_concat_then_overlap_dynamic_index_runtime",
        "hierarchy::test_hierarchical_dynamic_index_feedback_runtime",
        "hierarchy::test_hierarchical_dual_dynamic_readers_feedback_runtime",
        "hierarchy::test_hierarchical_overlapping_partial_write_dynamic_index_runtime",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn corpus_runtime_loops_with_proven_breaks() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "basic::test_comb_effectful_if_condition_after_dynamic_break_stays_inactive",
        "basic::test_comb_effectful_case_after_dynamic_break_stays_inactive",
        "basic::test_comb_value_system_function_after_dynamic_break_stays_inactive",
        "synth_dynamic_loop::test_runtime_break_in_synth_comb_loop",
        "synth_dynamic_loop::test_runtime_break_after_assign_in_synth_comb_loop",
        "flip_flop::test_ff_runtime_for_break",
        "flip_flop::test_ff_runtime_for_unsigned_slice_bound_zero_extends_signed_source",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn bounded_runtime_loops_preserve_guards_steps_and_mutable_bounds() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(count: input logic<3>, bound: input logic<6>, stop: input logic,
                   signed_end: input signed logic<4>,
                   sum: output logic<16>, inclusive: output logic<8>,
                   stepped: output logic<16>, hits: output logic<8>,
                   signed_hits: output logic<8>, mutable_hits: output logic<8>) {
            always_comb {
                sum = 0;
                for i in 0..count { sum += i as 16; }
                inclusive = 0;
                for i in 0..=bound { inclusive = (i + 1) as 8; }
                stepped = 0;
                for i in 0..count step += 3 { stepped += i as 16; }
                hits = 0;
                for i in 0..count {
                    hits += 1;
                    if stop { break; }
                }
                signed_hits = 0;
                for i in 0..signed_end { signed_hits += 1; }
                var limit: logic<4>;
                limit = 5;
                mutable_hits = 0;
                for i in 0..limit {
                    mutable_hits += 1;
                    limit = 1;
                }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let count = sim.signal("count");
    let bound = sim.signal("bound");
    let stop = sim.signal("stop");
    let signed_end = sim.signal("signed_end");
    let sum = sim.signal("sum");
    let inclusive = sim.signal("inclusive");
    let stepped = sim.signal("stepped");
    let hits = sim.signal("hits");
    let signed_hits = sim.signal("signed_hits");
    let mutable_hits = sim.signal("mutable_hits");
    for n in 0..64u16 {
        for halted in [0u8, 1] {
            for end in [0u8, 7, 8, 15] {
                sim.modify(|io| {
                    io.set(count, n % 8);
                    io.set(bound, n);
                    io.set(stop, halted);
                    io.set(signed_end, end);
                })
                .unwrap();
                assert_eq!(sim.get(sum), (0..n % 8).sum::<u16>().into());
                assert_eq!(sim.get(inclusive), (n + 1).into());
                assert_eq!(sim.get(stepped), (0..n % 8).step_by(3).sum::<u16>().into());
                assert_eq!(
                    sim.get(hits),
                    (if halted == 0 { n % 8 } else { (n % 8).min(1) }).into()
                );
                assert_eq!(sim.get(signed_hits), (if end < 8 { end } else { 0 }).into());
                assert_eq!(sim.get(mutable_hits), 1u8.into());
            }
        }
    }
}

#[test]
fn packed_array_literals_reject_invalid_shapes() {
    for literal in [
        "'{1'b0}",
        "'{1'b0 repeat 3}",
        "'{default: 1'b0, default: 1'b1}",
    ] {
        let source = format!(
            "module Top(q: output logic<2>) {{
                function identity(x: input logic<2>) -> logic<2> {{ return x; }}
                always_comb {{ q = identity({literal}); }}
            }}"
        );
        let error = analyze_and_lower(&source, "invalid_pattern", "Top").unwrap_err();
        assert!(
            matches!(
                error,
                ImportError::AnalysisFailed(_) | ImportError::UnsupportedBehavior(_)
            ),
            "{error}"
        );
    }
}

#[test]
fn corpus_array_literal_effects() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case(
        "comb_observer::test_comb_function_packed_array_literal_preserves_source_order",
    )
    .unwrap()
    .run(&mut |design| compile(design, &stage));
}

fn array_literal_effects_design() -> Design {
    Design::new(
        r"
        module Top(d: input logic<8>, gate: input logic,
                   values: output logic<24>, side: output logic<8>,
                   repeated: output logic<24>, repeat_side: output logic<8>,
                   returned: output logic<24>, return_side: output logic<8>,
                   p: output logic<8>, packed_side: output logic<8>,
                   packed_nested: output logic<8>, packed_wide: output logic<16>,
                   packed_repeat: output logic<4>, packed_repeat_side: output logic<8>,
                   nested: output logic<32>, nested_side: output logic<8>,
                   loop_value: output logic<16>, loop_side: output logic<8>,
                   guarded: output logic<24>, guard_side: output logic<8>) {
            type row_t = logic<8>[2];
            function mark(x: input logic<8>, y: output logic<8>) -> logic<8> {
                y = x; return x;
            }
            function bump(x: input logic<8>, y: output logic<8>) -> logic<8> {
                y = x + 8'd1; return y;
            }
            function pack3(x: input logic<8>[3]) -> logic<24> {
                return {x[2], x[1], x[0]};
            }
            function pack2(x: input logic<8>[2]) -> logic<16> {
                return {x[1], x[0]};
            }
            function pack_grid(x: input logic<8>[2,2]) -> logic<32> {
                return {x[1][1], x[1][0], x[0][1], x[0][0]};
            }
            function make(x: input logic<8>, y: output logic<8>) -> row_t {
                y = x + 8'd1;
                return '{x, x + 8'd2};
            }
            function consume(x: input logic<8>[2], later: input logic<8>) -> logic<24> {
                return {later, x[1], x[0]};
            }
            function packed_identity(x: input logic<2,4>) -> logic<8> { return x; }
            function packed_grid(x: input logic<2,2,2>) -> logic<8> { return x; }
            function packed_bytes(x: input logic<2,8>) -> logic<16> { return x; }
            function packed_bits(x: input logic<4>) -> logic<4> { return x; }
            always_comb {
                side = 0;
                values = pack3('{default: mark(d, side), mark(d + 8'd1, side)});
                repeat_side = 0;
                repeated = pack3('{bump(repeat_side, repeat_side) repeat 3});
                return_side = 0;
                returned = consume(make(d, return_side), return_side);
                packed_side = 0;
                p = packed_identity('{default: mark(d, packed_side), 4'ha});
                packed_nested = packed_grid('{'{d as 2, 2'b01}, '{2'b10, d as 2}});
                packed_wide = packed_bytes('{4'sh8, d});
                packed_repeat_side = 0;
                packed_repeat = packed_bits('{bump(packed_repeat_side, packed_repeat_side) repeat 4});
                nested_side = 0;
                nested = pack_grid('{'{mark(d, nested_side), bump(nested_side, nested_side)},
                                     '{default: bump(nested_side, nested_side)}});
                loop_side = 0;
                loop_value = 0;
                for i in 0..3 {
                    loop_value = pack2('{bump(i as 8, loop_side), d});
                }
                guard_side = 8'd7;
                guarded = if gate ? pack3('{default: bump(d, guard_side)}) : 24'habcdef;
            }
        }
    ",
        "Top",
    )
}

#[test]
fn array_literal_effects_preserve_values_order_and_guards() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = array_literal_effects_design();
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    let gate = sim.signal("gate");
    for value in 0..=255u8 {
        for enabled in [0u8, 1] {
            sim.modify(|io| {
                io.set(d, value);
                io.set(gate, enabled);
            })
            .unwrap();
            let first = u32::from(value);
            let second = u32::from(value.wrapping_add(1));
            let third = u32::from(value.wrapping_add(2));
            for (name, expected) in [
                ("values", (first << 16) | (first << 8) | second),
                ("side", second),
                ("repeated", 0x0001_0101),
                ("repeat_side", 1),
                ("returned", (second << 16) | (third << 8) | first),
                ("return_side", second),
                ("p", 0xa0 | (first & 0xf)),
                ("packed_side", first),
                ("packed_nested", ((first & 3) << 6) | 0x18 | (first & 3)),
                ("packed_wide", 0xf800 | first),
                ("packed_repeat", 0xf),
                ("packed_repeat_side", 1),
                (
                    "nested",
                    (third << 24) | (third << 16) | (second << 8) | first,
                ),
                ("nested_side", third),
                ("loop_value", (first << 8) | 3),
                ("loop_side", 3),
                (
                    "guarded",
                    if enabled == 1 {
                        second * 0x0001_0101
                    } else {
                        0x00ab_cdef
                    },
                ),
                ("guard_side", if enabled == 1 { second } else { 7 }),
            ] {
                assert_eq!(
                    sim.get(sim.signal(name)),
                    expected.into(),
                    "{name}: input {value}, gate {enabled}"
                );
            }
        }
    }
}

#[test]
fn corpus_function_input_effects() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "basic::test_statement_call_inputs_follow_output_writeback_order",
        "comb_observer::test_named_function_inputs_evaluate_in_source_order",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

fn function_input_effects_design() -> Design {
    Design::new(
        r"
        module Top(d: input logic<8>, gate: input logic,
                   frozen: output logic<16>, named: output logic<16>,
                   nested: output logic<16>, guarded: output logic<16>,
                   side: output logic<8>, named_side: output logic<8>,
                   nested_side: output logic<8>, guarded_side: output logic<8>,
                   overwritten: output logic<8>, global_side: output logic<8>,
                   global_read: output logic<8>, returned: output logic<8>,
                   return_side: output logic<8>, and_side: output logic<8>,
                   or_side: output logic<8>, and_result: output logic, or_result: output logic) {
            function bump(x: input logic<8>, y: output logic<8>) -> logic<8> {
                y = x + 8'd1;
                return x + 8'd2;
            }
            function pair(first: input logic<8>, second: input logic<8>) -> logic<16> {
                return {first, second};
            }
            function finish(x: input logic<8>, dst: output logic<8>) {
                dst = x ^ 8'ha5;
            }
            function read_global(x: input logic<8>) -> logic<8> {
                return x ^ global_side;
            }
            function identity(x: input logic<8>) -> logic<8> { return x; }
            function early(stop: input logic, x: input logic<8>, dst: output logic<8>) -> logic<8> {
                dst = 8'd7;
                if stop { return 8'd0; }
                return identity(bump(x, dst));
            }
            always_comb {
                side = 8'h5a;
                frozen = pair(side, bump(d, side));
                named_side = 0;
                named = pair(second: bump(d, named_side), first: named_side);
                nested_side = 0;
                nested = pair(bump(d, nested_side), pair(nested_side, d) as 8);
                guarded_side = 8'd3;
                guarded = if gate ? pair(bump(d, guarded_side), guarded_side) : 16'habcd;
                overwritten = 0;
                finish(bump(d, overwritten), overwritten);
                global_side = 0;
                global_read = read_global(bump(d, global_side));
                returned = early(gate, d, return_side);
                and_side = 8'd4;
                and_result = gate && identity(bump(d, and_side));
                or_side = 8'd4;
                or_result = gate || identity(bump(d, or_side));
            }
        }
        ",
        "Top",
    )
}

#[test]
fn function_input_effects_preserve_snapshots_frames_and_guards() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = function_input_effects_design();
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    let gate = sim.signal("gate");
    for value in 0..=255u8 {
        for stop in [0u8, 1] {
            sim.modify(|io| {
                io.set(d, value);
                io.set(gate, stop);
            })
            .unwrap();
            let written = value.wrapping_add(1);
            let result = value.wrapping_add(2);
            for (name, expected) in [
                ("frozen", 0x5a00 | u16::from(result)),
                ("named", (u16::from(written) << 8) | u16::from(result)),
                ("nested", (u16::from(result) << 8) | u16::from(value)),
                (
                    "guarded",
                    if stop == 1 {
                        (u16::from(result) << 8) | u16::from(written)
                    } else {
                        0xabcd
                    },
                ),
                ("side", u16::from(written)),
                ("named_side", u16::from(written)),
                ("nested_side", u16::from(written)),
                (
                    "guarded_side",
                    u16::from(if stop == 1 { written } else { 3 }),
                ),
                ("and_side", u16::from(if stop == 1 { written } else { 4 })),
                ("or_side", u16::from(if stop == 0 { written } else { 4 })),
                ("and_result", u16::from(stop == 1 && result != 0)),
                ("or_result", u16::from(stop == 1 || result != 0)),
                ("overwritten", u16::from(result ^ 0xa5)),
                ("global_side", u16::from(written)),
                ("global_read", u16::from(result ^ written)),
                ("returned", u16::from(if stop == 1 { 0 } else { result })),
                (
                    "return_side",
                    u16::from(if stop == 1 { 7 } else { written }),
                ),
            ] {
                assert_eq!(
                    sim.get(sim.signal(name)),
                    expected.into(),
                    "{name}: input {value}, gate {stop}"
                );
            }
        }
    }
}

#[test]
fn corpus_functions_and_static_loops() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "basic::test_comb_function_call_early_return",
        "basic::test_comb_function_call_return_indexed_local_temp",
        "basic::test_comb_function_call_local_and_return_width_coercion",
        "function_arguments::test_comb_output_copyout_freezes_aliased_inputs_and_return",
        "function_arguments::test_comb_statement_output_copyout_obeys_named_argument_order",
        "function_arguments::test_comb_nested_output_copyout_stops_at_early_return",
        "function_arguments::test_comb_expression_output_copyout_uses_unsigned_formal_for_signed_body",
        "function_arguments::test_comb_output_copyout_to_concat_preserves_unselected_bits_and_elements",
        "for_loop_unroll::test_for_loop_unroll_shift_register",
        "for_loop_unroll::test_for_loop_unroll_with_default_zero_reset",
        "for_loop_unroll::test_for_loop_unroll_break_in_always_ff",
        "nba_dynamic_array::test_always_ff_let_bindings_are_visible_immediately",
        "nba_dynamic_array::test_dynamic_ff_array_partial_squash_preserves_head_and_branch",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn corpus_comb_function_effects_and_language_rejections() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "basic::test_comb_function_call_expression_output_is_visible_to_later_operand",
        "basic::test_comb_function_call_expression_output_is_guarded_by_ternary",
        "basic::test_comb_function_call_expression_output_respects_short_circuit",
        "basic::test_comb_function_call_with_output_argument_in_if_condition",
        "basic::test_comb_function_call_with_output_argument_in_case_target",
        "basic::test_comb_function_condition_output_is_guarded_after_early_return",
        "basic::test_comb_nested_function_output_call_in_function_condition",
        "concat_operators::test_shift_in_concat",
        "hierarchy::test_instance_output_dynamic_index_function_output_writeback",
        "hierarchy::test_instance_output_concat_advances_each_destination",
        "hierarchy::test_dynamic_output_port_rmw_preserves_unselected_bits",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn corpus_system_function_output_effects() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "basic::test_comb_function_call_expression_output_survives_system_function_wrapper",
        "basic::test_comb_value_system_function_statement_applies_argument_outputs",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn system_function_effects_preserve_values_and_execution_guards() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(d: input logic<8>, stop: input logic,
                   signed_q: output logic<16>, unsigned_q: output logic<16>,
                   side: output logic<8>,
                   loop_side: output logic<8>, return_side: output logic<8>) {
            function f(x: input logic<8>, y: output logic<8>) -> logic<8> {
                y = x + 8'd1;
                return x;
            }
            function guarded(x: input logic<8>, stop: input logic,
                             y: output logic<8>) -> logic<8> {
                y = 0;
                if stop { return 0; }
                $unsigned(f(x, y));
                return y;
            }
            always_comb {
                signed_q = $signed(f(d, side));
                unsigned_q = $unsigned($signed(f(d, side)));
                loop_side = 0;
                for i in 0..2 {
                    if stop { break; }
                    $unsigned(f(d + i, loop_side));
                }
                var ignored: logic<8>;
                ignored = guarded(d, stop, return_side);
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    let stop = sim.signal("stop");
    let signed_q = sim.signal("signed_q");
    let unsigned_q = sim.signal("unsigned_q");
    let side = sim.signal("side");
    let loop_side = sim.signal("loop_side");
    let return_side = sim.signal("return_side");
    for x in [0u8, 1, 3, 8, 128, 255] {
        for halted in [1u8, 0, 1] {
            sim.modify(|io| {
                io.set(d, x);
                io.set(stop, halted);
            })
            .unwrap();
            assert_eq!(
                sim.get(signed_q),
                i16::from(x.cast_signed()).cast_unsigned().into()
            );
            assert_eq!(sim.get(unsigned_q), x.into());
            assert_eq!(sim.get(side), x.wrapping_add(1).into());
            let guarded = if halted == 0 { x.wrapping_add(1) } else { 0 };
            assert_eq!(
                sim.get(loop_side),
                if halted == 0 { x.wrapping_add(2) } else { 0 }.into()
            );
            assert_eq!(sim.get(return_side), guarded.into());
        }
    }
}

#[test]
fn corpus_function_array_arguments() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "comb_observer::test_comb_function_direct_array_argument_converts_each_element",
        "comb_observer::test_comb_function_direct_array_return_preserves_all_elements",
        "comb_observer::test_comb_function_array_literal_accepts_array_returning_items",
        "comb_observer::test_comb_statement_function_direct_array_argument_converts_each_element",
        "comb_observer::test_comb_function_array_literal_array_item_preserves_element_type",
        "comb_observer::test_comb_function_nested_array_scalar_default_converts_each_element",
        "flip_flop::test_ff_function_call_array_literal_element_uses_formal_context_width",
        "flip_flop::test_ff_function_call_array_literal_supports_dynamic_multidim_indexing",
        "flip_flop::test_ff_function_call_restores_nearest_array_view_after_deep_reentrant_call",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn array_returns_preserve_runtime_elements_and_signedness() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(a: input signed logic<4>, b: input signed logic<4>,
                   index: input logic, q: output signed logic<8>, copied: output signed logic<8>) {
            type row_t = signed logic<4> [2];
            function make_row(x: input signed logic<4>, y: input signed logic<4>) -> row_t {
                var row: row_t;
                row[0] = x;
                row[1] = y;
                return row;
            }
            function pick(row: input signed logic<8> [2], i: input logic) -> signed logic<8> {
                return row[i];
            }
            always_comb {
                var row: row_t;
                row = make_row(a, b);
                copied = pick(row, index);
                q = pick(make_row(a, b), index);
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let a = sim.signal("a");
    let b = sim.signal("b");
    let index = sim.signal("index");
    let q = sim.signal("q");
    let copied = sim.signal("copied");
    for (x, y) in [(8u8, 7u8), (1, 15), (6, 9)] {
        for i in 0..2u8 {
            sim.modify(|io| {
                io.set(a, x);
                io.set(b, y);
                io.set(index, i);
            })
            .unwrap();
            let value = if i == 0 { x } else { y };
            let expected = if value & 8 == 0 { value } else { value | 0xf0 };
            assert_eq!(sim.get(q), expected.into());
            assert_eq!(sim.get(copied), expected.into());
        }
    }
}

#[test]
fn corpus_constant_power() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "operators::test_pow_operator_constant_exponent",
        "operators::test_pow_operator_constant_exponent_ff",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn powers_preserve_context_and_self_determined_exponents() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(clk: input clock, base: input logic<4>,
                   signed_base: input signed logic<4>, exponent: input logic<4>,
                   signed_exponent: input signed logic<4>, wide_exponent: input logic<8>,
                   unsigned_o: output logic<8>, signed_o: output signed logic<8>,
                   unsigned_negative_o: output logic<8>, ff_o: output signed logic<8>,
                   zero_o: output logic<8>, cube_o: output logic<8>,
                   high_o: output logic<8>, narrow_o: output logic<4>,
                   negative_odd_o: output signed logic<8>, negative_even_o: output signed logic<8>,
                   mixed_o: output logic<8>, wide_o: output signed logic<65>,
                   small_o: output logic<4>, signed_unsigned_o: output signed logic<8>) {
            assign unsigned_o = base ** exponent;
            assign signed_o = signed_base ** signed_exponent;
            assign unsigned_negative_o = base ** signed_exponent;
            always_ff (clk) { ff_o = signed_base ** signed_exponent; }
            assign zero_o = base ** 0;
            assign cube_o = base ** 3;
            assign high_o = base ** 8'd128;
            assign narrow_o = {signed_base ** exponent};
            assign negative_odd_o = signed_base ** -3;
            assign negative_even_o = signed_base ** -2;
            assign mixed_o = (signed_base ** 3) + 8'd0;
            assign wide_o = signed_base ** 3;
            assign small_o = base ** wide_exponent;
            assign signed_unsigned_o = signed_base ** exponent;
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let base = sim.signal("base");
    let signed_base = sim.signal("signed_base");
    let exponent = sim.signal("exponent");
    let signed_exponent = sim.signal("signed_exponent");
    let wide_exponent = sim.signal("wide_exponent");
    let clk = sim.event("clk");
    for b in 0u8..16 {
        for e in 0u8..16 {
            sim.modify(|io| {
                io.set(base, b);
                io.set(signed_base, b);
                io.set(exponent, e);
                io.set(signed_exponent, e);
                io.set(wide_exponent, 128 | e);
            })
            .unwrap();
            sim.tick(clk).unwrap();
            let signed_b = if b & 8 == 0 { b } else { b | 0xf0 };
            let signed_expected = if e < 8 {
                signed_b.wrapping_pow(u32::from(e))
            } else {
                match signed_b {
                    255 if e & 1 == 1 => 255,
                    1 | 255 => 1,
                    _ => 0,
                }
            };
            let unsigned_negative = if e < 8 {
                b.wrapping_pow(u32::from(e))
            } else {
                u8::from(b == 1)
            };
            for (name, expected) in [
                ("unsigned_o", b.wrapping_pow(u32::from(e))),
                ("signed_o", signed_expected),
                ("ff_o", signed_expected),
                ("unsigned_negative_o", unsigned_negative),
                ("zero_o", 1),
                ("cube_o", b.wrapping_pow(3)),
                ("high_o", b.wrapping_pow(128)),
                ("narrow_o", b.wrapping_pow(u32::from(e)) & 15),
                (
                    "negative_odd_o",
                    if b == 15 { 255 } else { u8::from(b == 1) },
                ),
                ("negative_even_o", u8::from(b == 1 || b == 15)),
                ("mixed_o", b.wrapping_pow(3)),
                ("small_o", b.wrapping_pow(u32::from(128 | e)) & 15),
                ("signed_unsigned_o", signed_b.wrapping_pow(u32::from(e))),
            ] {
                assert_eq!(
                    sim.get(sim.signal(name)),
                    expected.into(),
                    "{name}: base={b}, exponent={e}"
                );
            }
            let integer_base = i128::from(b & 7) - i128::from(b & 8);
            let expected =
                u128::from_le_bytes(integer_base.pow(3).to_le_bytes()) & ((1u128 << 65) - 1);
            assert_eq!(
                sim.get(sim.signal("wide_o")),
                expected.into(),
                "wide base={b}"
            );
        }
    }
}

#[test]
fn dynamic_part_selects_preserve_only_in_range_bits() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(index: input signed logic<6>, data: input logic<16>, value: input logic<4>,
                   plus_read: output logic<4>, minus_read: output logic<4>,
                   plus_write: output logic<16>, minus_write: output logic<16>,
                   step_read: output logic<4>, step_write: output logic<16>) {
            assign plus_read = data[index +: 4];
            assign minus_read = data[index -: 4];
            assign step_read = data[index step 4];
            always_comb {
                plus_write = data;
                plus_write[index +: 4] = value;
                minus_write = data;
                minus_write[index -: 4] = value;
                step_write = data;
                step_write[index step 4] = value;
            }
        }
    ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let index = sim.signal("index");
    let data = sim.signal("data");
    let value = sim.signal("value");
    for base in -32i32..32 {
        sim.modify(|io| {
            io.set(index, u8::try_from(base.rem_euclid(64)).unwrap());
            io.set(data, 0xa53cu16);
            io.set(value, 0xbu8);
        })
        .unwrap();
        for (prefix, low) in [("plus", base), ("minus", base - 3), ("step", base * 4)] {
            let mut read = 0u16;
            let mut written = 0xa53cu16;
            for bit in 0..4 {
                let target = low + bit;
                if (0..16).contains(&target) {
                    read |= ((0xa53cu16 >> target) & 1) << bit;
                    written = (written & !(1 << target)) | (((0xbu16 >> bit) & 1) << target);
                }
            }
            assert_eq!(
                sim.get(sim.signal(&format!("{prefix}_read"))),
                read.into(),
                "{prefix} index={base}"
            );
            assert_eq!(
                sim.get(sim.signal(&format!("{prefix}_write"))),
                written.into(),
                "{prefix} index={base}"
            );
        }
    }
}

#[test]
fn mixed_width_comparison_zero_extends_when_either_operand_is_unsigned() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(a: input signed logic<8>, b: input logic<16>,
                   less: output logic, equal: output logic) {
            assign less = a <: b;
            assign equal = a == b;
        }
    ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let a = sim.signal("a");
    let b = sim.signal("b");
    let less = sim.signal("less");
    let equal = sim.signal("equal");
    for av in [0u8, 127, 128, 255] {
        for bv in [0u16, 128, 255, 256, 65535] {
            sim.modify(|io| {
                io.set(a, av);
                io.set(b, bv);
            })
            .unwrap();
            assert_eq!(sim.get(less), u8::from(u16::from(av) < bv).into());
            assert_eq!(sim.get(equal), u8::from(u16::from(av) == bv).into());
        }
    }
}

#[test]
fn eliminated_clock_only_accepts_known_source_events() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        "module Top(clk: input clock, q: output logic) { always_ff (clk) { q = 1; } }",
        "Top",
    );
    let mut sim = compile(&design, &stage).unwrap();
    sim.tick("clk").unwrap();
    sim.tick("clk").unwrap();
    assert!(sim.tick("missing").is_err());
    let path = SignalPath {
        instances: vec![],
        name: "q".into(),
    };
    assert_eq!(sim.read(&path).unwrap(), (1u8.into(), 0u8.into()));
}

#[test]
fn corpus_division_system_functions_and_eliminated_clocks() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "operators::test_comb_div",
        "operators::test_ff_div",
        "signed_divrem::signed_divrem_i8",
        "expression_semantics::system_function_results_obey_ternary_width_contexts",
        "expression_semantics::cast_binary_semantics_match_between_comb_and_ff",
        "expression_semantics::aggregate_results_consume_the_unary_parent_context",
        "system_function::test_direct_ff_bits_type_system_function",
        "system_function::test_direct_comb_size_system_function",
        "system_function::test_comb_function_body_clog2_system_function",
        "system_function::test_ff_function_body_onehot_system_function",
        "system_function::test_direct_comb_signed_system_function_sign_extends_to_context",
        "system_function::test_direct_comb_unsigned_system_function_zero_extends_to_context",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn unsigned_parent_reaches_ternary_but_not_concatenation_operands() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(a: input signed logic<8>, b: input signed logic<8>,
                   u: input logic<16>, sel: input logic,
                   branch: output logic<16>, joined: output logic<16>) {
            assign branch = (if sel ? a : b) + u;
            assign joined = {8'b0, ((a as 8) / (b as 8))};
        }
        ",
        "Top",
    );
    let mut sim = compile(&design, &stage).unwrap();
    for (name, value) in [("a", 249u16), ("b", 2), ("u", 0), ("sel", 1)] {
        sim.write(
            &SignalPath {
                instances: vec![],
                name: name.into(),
            },
            value.into(),
            0u8.into(),
        )
        .unwrap();
    }
    for (name, expected) in [("branch", 249u16), ("joined", 253)] {
        assert_eq!(
            sim.read(&SignalPath {
                instances: vec![],
                name: name.into()
            })
            .unwrap()
            .0,
            expected.into(),
            "{name}"
        );
    }
}

#[test]
fn upstream_constant_size_cast_regression() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case(
        "expression_semantics::constant_and_runtime_casts_use_the_same_resize_rule",
    )
    .unwrap()
    .run(&mut |design| compile(design, &stage));
}

#[test]
fn upstream_constant_function_actual_regression() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case("flip_flop::test_ff_function_call_nonvariable_argument_preserves_self_sized_overflow_before_coercion")
        .unwrap().run(&mut |design| compile(design, &stage));
}

#[test]
fn corpus_index_read_effects() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case(
        "basic::test_comb_function_call_with_output_argument_in_index_expression",
    )
    .unwrap()
    .run(&mut |design| compile(design, &stage));
}

fn index_read_effects_design() -> Design {
    Design::new(
        r"
        module Top(d: input logic<8>, sel: input logic<3>, gate: input logic,
                   bit_q: output logic, slice_q: output logic<3>,
                   array_q: output logic<8>, nested_q: output logic,
                   guarded_q: output logic, side: output logic<8>,
                   slice_side: output logic<8>, array_side: output logic<8>,
                   nested_side: output logic<8>, guard_side: output logic<8>,
                   changed_q: output logic, changed: output logic<8>) {
            function index(x: input logic<3>, count: input logic<8>,
                           next: output logic<8>) -> logic<3> {
                next = count + 8'd1;
                return x;
            }
            function replace(x: input logic<3>, data: output logic<8>) -> logic<3> {
                data = 8'hff;
                return x;
            }
            var words: logic<8>[4];
            always_comb {
                words[0] = d;
                words[1] = ~d;
                words[2] = 8'h35;
                words[3] = 8'hca;
                side = 0;
                bit_q = d[index(sel, side, side)];
                slice_side = 0;
                slice_q = d[index(sel, slice_side, slice_side)+:3];
                array_side = 0;
                array_q = words[index(sel, array_side, array_side)];
                nested_side = 0;
                nested_q = words[index(sel, nested_side, nested_side)]
                                [index(sel, nested_side, nested_side)];
                guard_side = 0;
                guarded_q = if gate ? d[index(sel, guard_side, guard_side)] : 1'b0;
                changed = 0;
                changed_q = changed[replace(sel, changed)];
            }
        }
        ",
        "Top",
    )
}

#[test]
fn index_reads_evaluate_once_and_preserve_guards() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = index_read_effects_design();
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    let sel = sim.signal("sel");
    let gate = sim.signal("gate");
    for value in 0..=255u8 {
        for index in 0..8u8 {
            for enabled in [0u8, 1] {
                sim.modify(|io| {
                    io.set(d, value);
                    io.set(sel, index);
                    io.set(gate, enabled);
                })
                .unwrap();
                let selected = match index {
                    0 => value,
                    1 => !value,
                    2 => 0x35,
                    3 => 0xca,
                    _ => 0,
                };
                for (name, expected) in [
                    ("bit_q", (value >> index) & 1),
                    ("slice_q", (value >> index) & 7),
                    ("array_q", selected),
                    ("nested_q", (selected >> index) & 1),
                    ("guarded_q", enabled & (value >> index) & 1),
                    ("side", 1),
                    ("slice_side", 1),
                    ("array_side", 1),
                    ("nested_side", 2),
                    ("guard_side", enabled),
                    ("changed_q", 1),
                    ("changed", 255),
                ] {
                    assert_eq!(
                        sim.get(sim.signal(name)),
                        u32::from(expected).into(),
                        "{name}: data {value}, index {index}, gate {enabled}"
                    );
                }
            }
        }
    }
}

#[test]
fn corpus_destination_index_effects() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case(
        "basic::test_comb_function_call_with_output_argument_in_destination_index",
    )
    .unwrap()
    .run(&mut |design| compile(design, &stage));
}

fn destination_index_effects_design() -> Design {
    Design::new(
        r"
        module Top(sel: input logic<3>, gate: input logic,
                   words_q: output logic<32>, count: output logic<8>,
                   slice_q: output logic<8>, slice_count: output logic<8>,
                   concat_q: output logic<8>, concat_count: output logic<8>,
                   changed: output logic<8>, guarded: output logic<8>,
                   guard_count: output logic<8>, rhs_q: output logic<8>,
                   frozen_q: output logic<8>, address: output logic<3>) {
            function index(x: input logic<3>, old: input logic<8>,
                           next: output logic<8>) -> logic<3> {
                next = old + 8'd1;
                return x;
            }
            function replace(x: input logic<3>, data: output logic<8>) -> logic<3> {
                data = 8'hff;
                return x;
            }
            function rhs(x: input logic<3>, dest: output logic<3>) -> logic {
                dest = x;
                return 1'b1;
            }
            var words: logic<8>[4];
            var rhs_address: logic<3>;
            always_comb {
                words[0] = 0;
                words[1] = 0;
                words[2] = 0;
                words[3] = 0;
                count = 0;
                words[index(sel, count, count)][index(sel, count, count)] = 1'b1;
                words_q = {words[3], words[2], words[1], words[0]};
                slice_q = 0;
                slice_count = 0;
                slice_q[index(sel, slice_count, slice_count)+:3] = 3'b111;
                concat_q = 0;
                concat_count = 0;
                {concat_q[index(sel, concat_count, concat_count)],
                 concat_q[index(sel + 3'd1, concat_count, concat_count)]} = 2'b10;
                changed = 0;
                changed[replace(sel, changed)] = 1'b0;
                guarded = 0;
                guard_count = 0;
                if gate {
                    guarded[index(sel, guard_count, guard_count)] = 1'b1;
                }
                rhs_q = 0;
                rhs_address = 0;
                rhs_q[rhs_address] = rhs(sel, rhs_address);
                address = sel;
                frozen_q = 0;
                {address, frozen_q[address]} = {3'd7, 1'b1};
            }
        }
        ",
        "Top",
    )
}

#[test]
fn destination_indices_freeze_once_before_writes() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = destination_index_effects_design();
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let sel = sim.signal("sel");
    let gate = sim.signal("gate");
    for index in 0..8u32 {
        for enabled in [0u32, 1] {
            sim.modify(|io| {
                io.set(sel, index);
                io.set(gate, enabled);
            })
            .unwrap();
            for (name, expected) in [
                ("words_q", if index < 4 { 1 << (9 * index) } else { 0 }),
                ("count", 2),
                ("slice_q", (7 << index) & 255),
                ("slice_count", 1),
                ("concat_q", 1 << index),
                ("concat_count", 2),
                ("changed", 255 ^ (1 << index)),
                ("guarded", enabled << index),
                ("guard_count", enabled),
                ("rhs_q", 1 << index),
                ("frozen_q", 1 << index),
                ("address", 7),
            ] {
                assert_eq!(
                    sim.get(sim.signal(name)),
                    expected.into(),
                    "{name}: index {index}, gate {enabled}"
                );
            }
        }
    }
}

#[test]
fn corpus_expression_type_regressions() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "veryl_context_regressions::part_select_of_signed_is_unsigned",
        "veryl_context_regressions::signed_struct_member_sign_extends",
        "veryl_context_regressions::wide_logical_operand_keeps_result_type",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

fn expression_type_design() -> Design {
    Design::new(
        r"
        module Top(a: input logic<8>, b: input logic<2>,
                   member: output logic<16>, selected: output logic<16>,
                   numeric: output logic<16>, mixed: output logic<16>,
                   signed_sum: output logic<16>, joined: output logic<16>,
                   repeated: output logic<24>) {
            struct Inner { m: signed logic<8>, n: logic<8>, }
            struct Outer { inner: Inner, tail: logic<4>, }
            var s: Outer;
            var i: i32;
            always_comb {
                s.inner.m = a;
                s.inner.n = a;
                s.tail = b;
                i = $signed(a);
                member = s.inner.m;
                selected = s.inner.m[7:0] + 16'sd0;
                numeric = i[7:0] + 16'sd0;
                mixed = $signed(a[3:0]) + b;
                signed_sum = $signed(a[3:0]) + 16'sd0;
                joined = {(if 1 ? a : b), 4'h5};
                repeated = {(if 1 ? a : b) repeat 2, 4'h5};
            }
        }
        ",
        "Top",
    )
}

#[test]
fn expression_types_preserve_members_selections_and_context() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = expression_type_design();
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let a = sim.signal("a");
    let b = sim.signal("b");
    for value in 0..=255u32 {
        for other in 0..4u32 {
            sim.modify(|io| {
                io.set(a, value);
                io.set(b, other);
            })
            .unwrap();
            let member = if value & 128 != 0 {
                value | 0xff00
            } else {
                value
            };
            let low = value & 15;
            let signed = if low & 8 != 0 { low | 0xfff0 } else { low };
            for (name, expected) in [
                ("member", member),
                ("selected", value),
                ("numeric", value),
                ("mixed", low + other),
                ("signed_sum", signed),
                ("joined", (value << 4) | 5),
                ("repeated", (value << 12) | (value << 4) | 5),
            ] {
                assert_eq!(
                    sim.get(sim.signal(name)),
                    expected.into(),
                    "{name}: a={value}, b={other}"
                );
            }
        }
    }
}

#[test]
fn packed_member_dynamic_access_stays_in_its_domain() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(d: input logic<8>, idx: input signed logic<8>,
                   read: output logic<3>, written: output logic<16>,
                   row_read: output logic, row_written: output logic<16>, expr_read: output logic, calls: output logic<8>) {
            struct S { high: logic<4>, data: logic<8>, low: logic<4>, }
            struct Rows { high: logic<4>, data: logic<2,4>, low: logic<4>, }
            function address(x: input signed logic<8>, old: input logic<8>,
                             next: output logic<8>) -> signed logic<8> {
                next = old + 1;
                return x;
            }
            var s: S;
            var rows: Rows;
            always_comb {
                calls = 0;
                s.high = 4'ha;
                s.data = d;
                s.low = 4'h5;
                read = s.data[address(idx, calls, calls)+:3];
                s.data[address(idx, calls, calls)+:3] = 3'b101;
                written = {s.high, s.data, s.low};
                rows.high = 4'ha;
                rows.data = d;
                rows.low = 4'h5;
                row_read = rows.data[1][idx];
                expr_read = rows.data[1][((idx as u8) + 8'hff)];
                rows.data[1][idx] = 1'b1;
                row_written = {rows.high, rows.data, rows.low};
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    let idx = sim.signal("idx");
    for value in 0..256u32 {
        for index in -12..20i32 {
            sim.modify(|io| {
                io.set(d, value);
                io.set(idx, index.to_le_bytes()[0]);
            })
            .unwrap();
            let mut read = 0u32;
            let mut written = value;
            for bit in 0..3 {
                let target = index + bit;
                if (0..8).contains(&target) {
                    read |= ((value >> target) & 1) << bit;
                    written = (written & !(1 << target)) | (((5 >> bit) & 1) << target);
                }
            }
            let row_read = if (0..4).contains(&index) {
                (value >> (index + 4)) & 1
            } else {
                0
            };
            let row_written = if (0..4).contains(&index) {
                value | (1 << (index + 4))
            } else {
                value
            };
            for (name, expected) in [
                ("calls", 2),
                ("read", read),
                ("written", 0xa005 | (written << 4)),
                ("row_read", row_read),
                (
                    "expr_read",
                    if (1..=4).contains(&index) {
                        (value >> (index + 3)) & 1
                    } else {
                        0
                    },
                ),
                ("row_written", 0xa005 | (row_written << 4)),
            ] {
                assert_eq!(
                    sim.get(sim.signal(name)),
                    expected.into(),
                    "{name}: d={value}, idx={index}"
                );
            }
        }
    }
}

#[test]
fn packed_member_stride_does_not_wrap_large_indices() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(d: input logic<8>, idx: input logic<32>,
                   read: output logic<4>, written: output logic<16>) {
            struct S { high: logic<4>, data: logic<2,4>, low: logic<4>, }
            var s: S;
            always_comb {
                s.high = 4'ha;
                s.data = d;
                s.low = 4'h5;
                read = s.data[idx];
                s.data[idx] = 4'hc;
                written = {s.high, s.data, s.low};
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    let idx = sim.signal("idx");
    for value in 0..256u32 {
        for index in [0, 1, 2, 0x4000_0000, 0x8000_0000, u32::MAX] {
            sim.modify(|io| {
                io.set(d, value);
                io.set(idx, index);
            })
            .unwrap();
            let (read, written) = if index < 2 {
                let shift = index * 4;
                (
                    (value >> shift) & 15,
                    (value & !(15 << shift)) | (12 << shift),
                )
            } else {
                (0, value)
            };
            assert_eq!(sim.get(sim.signal("read")), read.into(), "idx={index}");
            assert_eq!(
                sim.get(sim.signal("written")),
                (0xa005 | (written << 4)).into(),
                "idx={index}"
            );
        }
    }
}

#[test]
fn corpus_veryl_022_regressions() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "expression_semantics::short_circuit_operators_skip_effectful_operands",
        "expression_semantics::constant_and_runtime_casts_use_the_same_resize_rule",
        "flip_flop::test_ff_function_call_nonvariable_argument_preserves_self_sized_overflow_before_coercion",
        "system_function::test_direct_ff_size_packed_multidimensional_system_function",
        "system_function::test_direct_ff_size_packed_multidimensional_type_system_function",
        "veryl_context_regressions::constant_ternary_keeps_both_arm_types",
        "veryl_context_regressions::signed_cast_of_folded_constant_sign_extends",
        "veryl_context_regressions::constant_case_on_signed_target",
        "veryl_regressions::wide_struct_bit_field_rhs_no_spill",
        "hierarchy::test_inactive_instance_input_output_call_adds_no_parent_driver",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn function_inout_variable_formals_copy_in_before_body() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(d: input logic<8>, q: output logic<8>, original: output logic<8>) {
            function update(value: inout logic<8>, snapshot: input logic<8>,
                            observed: output logic<8>) {
                value += 8'd3;
                observed = snapshot;
                value += snapshot;
            }
            always_comb {
                q = d;
                update(q, q, original);
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    for value in 0..=255u8 {
        sim.modify(|io| io.set(d, value)).unwrap();
        assert_eq!(
            sim.get(sim.signal("q")),
            value.wrapping_mul(2).wrapping_add(3).into()
        );
        assert_eq!(sim.get(sim.signal("original")), value.into());
    }
}

#[test]
fn ff_inout_to_module_state_remains_rejected() {
    let error = analyze_and_lower(
        r"
        module Top(clk: input clock, q: output logic<8>) {
            function update(value: inout logic<8>) { value += 8'd1; }
            always_ff(clk) { update(q); }
        }
        ",
        "ff_inout",
        "Top",
    )
    .unwrap_err();
    assert!(matches!(
        error,
        ImportError::AnalysisFailed(_) | ImportError::UnsupportedBehavior(_)
    ));
}

#[test]
fn corpus_wildcard_comparisons() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "veryl_context_regressions::runtime_case_target_uses_comparison_context",
        "expression_semantics::wildcard_predicates_remain_one_bit_in_ternaries_and_concats",
        "veryl_context_regressions::case_compares_each_label_as_an_if_does",
        "veryl_language::inside_outside_range_endpoints",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn wildcard_constant_masks_preserve_width_and_signedness() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(a: input logic<8>, s: input signed logic<8>, wide: input logic<128>,
                   eq: output logic, ne: output logic, sx: output logic, ux: output logic,
                   sign: output logic, all: output logic, calls: output logic<8>, wide_eq: output logic,
                   bits: output logic<3>, registered: output logic<3>, clk: input clock) {
            function observe(x: input logic<8>, old: input logic<8>, next: output logic<8>) -> logic<8> {
                next = old + 1;
                return x;
            }
            const P: signed logic<4> = 4'sbx101;
            assign eq = a ==? 8'b10xz01xz;
            assign ne = a !=? 8'b10xz01xz;
            assign sx = s ==? P;
            assign ux = a ==? P;
            assign sign = s ==? 4'sb1x01;
            always_comb {
                calls = 0;
                all = observe(a, calls, calls) ==? 'x;
            }
            assign wide_eq = wide ==? {48'hxxxxxxxxxxxx, 8'b10xz01xz, 8'hzz,
                                       48'hzzzzzzzzzzzz, 8'b01xz10xz, 8'hxx};
            assign bits = {1'b1, (a ==? 8'b10xz01xz), (a !=? 8'b10xz01xz)};
            always_ff (clk) {
                registered = {1'b1, (a ==? 8'b10xz01xz), (a !=? 8'b10xz01xz)};
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let a = sim.signal("a");
    let s = sim.signal("s");
    let wide = sim.signal("wide");
    let clk = sim.event("clk");
    for value in 0..256u32 {
        sim.modify(|io| {
            io.set(a, value);
            io.set(s, value);
            io.set(
                wide,
                (u128::from(value) << 72) | (u128::from(value ^ 255) << 8),
            );
        })
        .unwrap();
        sim.tick(clk).unwrap();
        let eq = u32::from(value & 0xcc == 0x84);
        for (name, expected) in [
            ("eq", eq),
            ("ne", 1 - eq),
            ("sx", u32::from(value & 7 == 5)),
            ("ux", u32::from(value & 0xf7 == 5)),
            ("sign", u32::from(value & 0xfb == 0xf9)),
            ("all", 1),
            ("calls", 1),
            ("wide_eq", eq),
            ("bits", 4 | (eq << 1) | (1 - eq)),
            ("registered", 4 | (eq << 1) | (1 - eq)),
        ] {
            assert_eq!(
                sim.get(sim.signal(name)),
                expected.into(),
                "{name}: a={value}"
            );
        }
    }
}

#[test]
fn case_context_preserves_priority_and_evaluates_target_once() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(a: input logic<8>, clk: input clock,
                   calls: output logic<8>, y: output logic<3>, f: output logic<3>,
                   function_y: output logic<3>, loop_y: output logic<3>) {
            function observe(x: input logic<8>, old: input logic<8>, next: output logic<8>) -> logic<8> {
                next = old + 1;
                return x;
            }
            function select_value(x: input logic<8>) -> logic<3> {
                var result: logic<3>;
                case x + 8'h10 {
                    9'h105: result = 1;
                    default: result = 0;
                }
                return result;
            }
            always_comb {
                calls = 0;
                case observe(a, calls, calls) + 8'h10 {
                    9'h105: y = 1;
                    9'h106: y = 2;
                    9'bx0000xxxx: y = 3;
                    9'bxxxxx0000: y = 4;
                    default: y = 0;
                }
                function_y = select_value(a);
                loop_y = 0;
                for i in 0..2 {
                    case a + 8'h10 {
                        9'h105: loop_y += 1;
                        default: loop_y += 0;
                    }
                }
            }
            always_ff (clk) {
                case a + 8'h10 {
                    9'h105: f = 1;
                    9'h106: f = 2;
                    9'bx0000xxxx: f = 3;
                    9'bxxxxx0000: f = 4;
                    default: f = 0;
                }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let a = sim.signal("a");
    let clk = sim.event("clk");
    for value in 0..256u32 {
        sim.modify(|io| io.set(a, value)).unwrap();
        sim.tick(clk).unwrap();
        let selected = match value {
            245 => 1u32,
            246 => 2,
            240..=255 => 3,
            _ if value % 16 == 0 => 4,
            _ => 0,
        };
        for (name, expected) in [
            ("calls", 1),
            ("y", selected),
            ("f", selected),
            ("function_y", u32::from(value == 245)),
            ("loop_y", 2 * u32::from(value == 245)),
        ] {
            assert_eq!(
                sim.get(sim.signal(name)),
                expected.into(),
                "{name}: a={value}"
            );
        }
    }
}

#[test]
fn corpus_constant_array_reads() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "veryl_regressions::nested_array_index_const_array",
        "veryl_context_regressions::folded_const_select_keeps_its_sign",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

fn constant_array_read_design() -> Design {
    Design::new(
        r"
        package pkg {
            const W: logic<80> [2] = '{80'h123456789abcdef01234, 80'hfedcba9876543210abcd};
        }
        module Top #(
            param TABLE: i8 [2,3] = '{'{-3, 7, -5}, '{9, -11, 13}},
        ) (
            row: input logic<8>, col: input logic<8>,
            selected: output logic<16>, nibble: output logic<16>, calls: output logic<8>,
            row_sum: output logic<16>, fixed_row: output logic<16>,
            defaults: output logic<8>, repeated: output logic<8>, wide: output logic<80>,
        ) {
            const D: logic<8> [5] = '{default: 8'ha5};
            const R: logic<8> [5] = '{8'h12, 8'h34 repeat 3, 8'h56};
            function observe(x: input logic<8>, old: input logic<8>, next: output logic<8>) -> logic<8> {
                next = old + 1;
                return x;
            }
            function sum(values: input i8 [3]) -> i16 {
                return values[0] + values[1] + values[2];
            }
            always_comb {
                calls = 0;
                selected = TABLE[observe(row, calls, calls)][observe(col, calls, calls)];
                nibble = TABLE[row][col][3:0];
                row_sum = sum(TABLE[row]);
                fixed_row = TABLE[1][col];
                defaults = D[col];
                repeated = R[col];
                wide = pkg::W[row];
            }
        }
        ",
        "Top",
    )
}

#[test]
fn constant_arrays_preserve_shape_sign_and_index_effects() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = constant_array_read_design();
    let rtl = analyze_and_lower(&design.sources[0].text, "constant_rom", "Top").unwrap();
    let top = rtl.top_module().unwrap();
    assert!(top.registers().is_empty());
    assert!(top.memories().is_empty());
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let row = sim.signal("row");
    let col = sim.signal("col");
    let table = [[-3i16, 7, -5], [9, -11, 13]];
    for r in [0u8, 1, 2, 3, 255] {
        for c in [0u8, 1, 2, 3, 4, 5, 255] {
            sim.modify(|io| {
                io.set(row, r);
                io.set(col, c);
            })
            .unwrap();
            let value = table
                .get(usize::from(r))
                .and_then(|row| row.get(usize::from(c)))
                .copied()
                .unwrap_or(0);
            let fixed = table[1].get(usize::from(c)).copied().unwrap_or(0);
            let sum = table
                .get(usize::from(r))
                .map_or(0, |row| row.iter().sum::<i16>());
            for (name, expected) in [
                ("selected", u16::from_le_bytes(value.to_le_bytes())),
                ("nibble", u16::from_le_bytes(value.to_le_bytes()) & 15),
                ("calls", 2),
                ("fixed_row", u16::from_le_bytes(fixed.to_le_bytes())),
                ("row_sum", u16::from_le_bytes(sum.to_le_bytes())),
                ("defaults", if c < 5 { 0xa5 } else { 0 }),
                (
                    "repeated",
                    match c {
                        0 => 0x12,
                        1..=3 => 0x34,
                        4 => 0x56,
                        _ => 0,
                    },
                ),
            ] {
                assert_eq!(
                    sim.get(sim.signal(name)),
                    expected.into(),
                    "{name}: row={r}, col={c}"
                );
            }
            let wide = match r {
                0 => 0x1234_5678_9abc_def0_1234u128,
                1 => 0xfedc_ba98_7654_3210_abcdu128,
                _ => 0,
            };
            assert_eq!(sim.get(sim.signal("wide")), wide.into(), "row={r}");
        }
    }
}

#[test]
fn signed_array_indices_do_not_alias_negative_values() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(idx: input signed logic<2>, rom: output logic<8>,
                   read: output logic<8>, written: output logic<32>) {
            const TABLE: logic<8> [4] = '{11, 22, 33, 44};
            var data: logic<8> [4];
            always_comb {
                data = '{11, 22, 33, 44};
                rom = TABLE[idx];
                read = data[idx];
                data[idx] = 8'hcc;
                written = {data[3], data[2], data[1], data[0]};
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let idx = sim.signal("idx");
    for (bits, expected, written) in [
        (0u8, 11u8, 0x2c21_16ccu32),
        (1, 22, 0x2c21_cc0b),
        (2, 0, 0x2c21_160b),
        (3, 0, 0x2c21_160b),
    ] {
        sim.modify(|io| io.set(idx, bits)).unwrap();
        assert_eq!(sim.get(sim.signal("rom")), expected.into());
        assert_eq!(sim.get(sim.signal("read")), expected.into());
        assert_eq!(sim.get(sim.signal("written")), written.into());
    }
}

#[test]
fn runtime_values_do_not_prove_unconditional_loop_breaks() {
    for condition in ["A[index]", "early(index)", "truncated()"] {
        let source = format!(
            r"
            module Top(index: input logic, count: input logic<32>, q: output logic<8>) {{
                const A: logic [2] = '{{1'b1, 1'b0}};
                function early(x: input logic) -> logic {{
                    if x {{ return 1'b0; }}
                    return 1'b1;
                }}
                function truncated() -> logic {{ return 2'd2; }}
                always_comb {{
                    q = 0;
                    for i in 0..count {{
                        if {condition} {{ break; }}
                        q += i as 8;
                    }}
                }}
            }}
            "
        );
        let error = analyze_and_lower(&source, "runtime_break_proof", "Top").unwrap_err();
        assert!(
            matches!(&error, ImportError::UnsupportedBehavior(message)
                if message.contains("termination is not proven")),
            "{condition}: {error}"
        );
    }
}

#[test]
fn corpus_scoped_local_variables() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case("duplicate_varpath::test_duplicate_scoped_var_in_always_comb")
        .unwrap()
        .run(&mut |design| compile(design, &stage));
}

#[test]
fn corpus_generate_constant_mux_dependencies() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case("duplicate_varpath::test_duplicate_scoped_var_with_generate_for")
        .unwrap()
        .run(&mut |design| compile(design, &stage));
}

#[test]
fn scoped_signal_names_are_unique_and_stable() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(d: input logic<4>, clk: input clock,
                   c0: output logic<4>, c1: output logic<4>,
                   q0: output logic<4>, q1: output logic<4>) {
            always_comb {
                for i in 0..2 {
                    var tmp: logic<4>;
                    tmp = d + i;
                    c0 = tmp;
                }
                for j in 0..3 {
                    var tmp: logic<4>;
                    tmp = d + j;
                    c1 = tmp;
                }
            }
            always_ff (clk) {
                var tmp: logic<4>;
                tmp = d + 4'd1;
                q0 = tmp;
            }
            always_ff (clk) {
                var tmp: logic<4>;
                tmp = d + 4'd2;
                q1 = tmp;
            }
        }
        ",
        "Top",
    );
    let names = || {
        let rtl = analyze_and_lower(&design.sources[0].text, "scoped_locals", "Top").unwrap();
        rtl.top_module()
            .unwrap()
            .signals()
            .iter()
            .map(|signal| signal.name().to_owned())
            .collect::<Vec<_>>()
    };
    let first = names();
    assert_eq!(
        first.iter().collect::<std::collections::HashSet<_>>().len(),
        first.len()
    );
    assert!(first.iter().any(|name| name.contains("$scope")));
    analyze_and_lower(
        "module Other(a: input logic, q: output logic) { assign q = a; }",
        "other",
        "Other",
    )
    .unwrap();
    assert_eq!(first, names());
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    let clk = sim.event("clk");
    for value in 0..16u8 {
        sim.modify(|io| io.set(d, value)).unwrap();
        sim.tick(clk).unwrap();
        for (name, increment) in [("c0", 1), ("c1", 2), ("q0", 1), ("q1", 2)] {
            assert_eq!(
                sim.get(sim.signal(name)),
                ((value + increment) & 15).into(),
                "{name}: d={value}"
            );
        }
    }
}

#[test]
fn ff_local_blocking_updates_preserve_state_and_global_nba_reads() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(clk: input clock, clear: input logic, d: input logic<8>, idx: input logic<2>,
                   current: output logic<8>, prior: output logic<8>, local_read: output logic<8>) {
            var delayed: logic<8>;
            always_ff (clk) {
                var state: logic<8>;
                if clear {
                    state = 0;
                    delayed = 0;
                    current = 0;
                    prior = 0;
                } else {
                    state += d;
                    delayed = state;
                    current = state;
                    prior = delayed;
                }
            }
            always_ff (clk) {
                var words: logic<8> [4];
                words[idx] = d;
                local_read = words[idx];
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let clk = sim.event("clk");
    let clear = sim.signal("clear");
    let d = sim.signal("d");
    let idx = sim.signal("idx");
    sim.modify(|io| io.set(clear, 1u8)).unwrap();
    sim.tick(clk).unwrap();
    sim.modify(|io| io.set(clear, 0u8)).unwrap();
    let mut previous = 0u8;
    for value in 0..=255u8 {
        sim.modify(|io| {
            io.set(d, value);
            io.set(idx, value & 3);
        })
        .unwrap();
        sim.tick(clk).unwrap();
        let current = previous.wrapping_add(value);
        assert_eq!(sim.get(sim.signal("current")), current.into());
        assert_eq!(sim.get(sim.signal("prior")), previous.into());
        assert_eq!(sim.get(sim.signal("local_read")), value.into());
        previous = current;
    }
}

#[test]
fn ff_local_blocking_memory_requirement_is_not_silently_read_first() {
    for policy in ["required", "block", "distributed"] {
        let source = format!(
            r#"
            module Top(clk: input clock, idx: input logic<2>, d: input logic<8>, q: output logic<8>) {{
                always_ff (clk) {{
                    #[sv("struo_memory = \"{policy}\"")]
                    var words: logic<8> [4];
                    words[idx] = d;
                    q = words[idx];
                }}
            }}
            "#
        );
        let error = analyze_and_lower(&source, "blocking_memory", "Top").unwrap_err();
        assert!(
            matches!(&error, ImportError::RequiredMemoryInferenceFailed { reason, .. }
            if reason.contains("blocking always_ff-local")),
            "{policy}: {error}"
        );
    }
}

#[test]
fn corpus_instance_input_defaults() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case("veryl_regressions::inst_port_default_value_connected_not_folded")
        .unwrap()
        .run(&mut |design| compile(design, &stage));
}

#[test]
fn instance_input_defaults_preserve_connected_runtime_values() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Child(i: input logic<80> = 80'hfedc_ba98_7654_3210_abcd,
                     o: output logic<80>) { assign o = i; }
        module Top(d: input logic<80>, a: output logic<80>, b: output logic<80>) {
            inst omitted: Child(o: a);
            inst connected: Child(i: d, o: b);
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    for value in [
        0u128,
        1,
        0xffff_ffff_ffff_ffff_ffff,
        0x1234_5678_9abc_def0_1234,
    ] {
        sim.modify(|io| io.set(d, value)).unwrap();
        assert_eq!(
            sim.get(sim.signal("a")),
            0xfedc_ba98_7654_3210_abcdu128.into()
        );
        assert_eq!(sim.get(sim.signal("b")), value.into());
    }
}

#[test]
fn instance_input_defaults_do_not_zero_unknowns() {
    let source = "module Child(i: input logic<8> = 8'hxx, o: output logic<8>) { assign o = i; }
                  module Top(q: output logic<8>) { inst u: Child(o: q); }";
    let design = analyze_and_lower(source, "unknown_input_default", "Top").unwrap();
    let error = struo_synth::synthesize(&design).unwrap_err();
    assert!(
        matches!(error, struo_synth::SynthesisError::UndrivenSignalBit { .. }),
        "{error}"
    );
}

#[test]
fn instance_input_defaults_do_not_bypass_anonymous_input_rejection() {
    let source = "module Child(i: input logic<8> = 8'h5a, o: output logic<8>) { assign o = i; }
                  module Top(q: output logic<8>) { inst u: Child(i: _, o: q); }";
    let error = analyze_and_lower(source, "anonymous_input_default", "Top").unwrap_err();
    assert!(
        matches!(&error, ImportError::AnalysisFailed(message) if message.contains("AnonymousIdentifierUsage")),
        "{error}"
    );
}

#[test]
fn instance_input_defaults_preserve_signed_extension() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Child (
            i: input signed logic<12> = -12'sd3,
            o: output signed logic<16>
        ) { assign o = i; }
        module Top(a: output logic<16>, b: output logic<16>, c: output logic<16>) {
            inst first: Child(o: a);
            inst second: Child(o: b);
            inst explicit: Child(i: 12'sd2, o: c);
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    for (name, expected) in [("a", 0xfffdu16), ("b", 0xfffd), ("c", 2)] {
        assert_eq!(sim.get(sim.signal(name)), expected.into());
    }
}

#[test]
fn corpus_preferred_memory_falls_back_to_registers() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "flip_flop::test_ff_static_and_dynamic_writes_share_sparse_state",
        "nba_dynamic_array::test_dynamic_array_write_is_deferred_across_ff_blocks",
        "nba_dynamic_array::test_unaligned_309_bit_dynamic_ff_round_trip",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn preferred_memory_fallback_retains_other_memories_and_required_policy() {
    let source = r"
        module Top(clk: input clock, idx: input logic<2>, d: input logic<8>,
                   good_q: output logic<8>, bad_q: output logic<8>) {
            var a_good: logic<8> [4];
            BAD_POLICY
            var z_bad: logic<8> [4];
            always_ff (clk) {
                a_good[idx] = d;
                good_q = a_good[idx];
                z_bad[idx] = d;
                z_bad[0] = d + 8'd1;
                bad_q = z_bad[idx];
            }
        }
    ";
    let ordinary = source.replace("BAD_POLICY", "");
    let lowered = analyze_and_lower(&ordinary, "mixed_memory_fallback", "Top").unwrap();
    let memories = lowered.top_module().unwrap().memories();
    assert_eq!(memories.len(), 1);
    assert_eq!(memories[0].name, "a_good");
    let stage = Rc::new(RefCell::new(String::new()));
    let mut sim = celox_test_suite_veryl::Simulator::new(
        compile(&Design::new(&ordinary, "Top"), &stage).unwrap(),
    );
    let clk = sim.event("clk");
    let idx = sim.signal("idx");
    let d = sim.signal("d");
    for index in 0..4u8 {
        sim.modify(|io| {
            io.set(idx, index);
            io.set(d, 20u8 + index);
        })
        .unwrap();
        sim.tick(clk).unwrap();
    }
    let mut good = [20u8, 21, 22, 23];
    let mut bad = [24u8, 21, 22, 23];
    for value in 0..64u8 {
        let index = value & 3;
        sim.modify(|io| {
            io.set(idx, index);
            io.set(d, value);
        })
        .unwrap();
        sim.tick(clk).unwrap();
        assert_eq!(sim.get(sim.signal("good_q")), good[index as usize].into());
        assert_eq!(sim.get(sim.signal("bad_q")), bad[index as usize].into());
        good[index as usize] = value;
        bad[index as usize] = value;
        bad[0] = value + 1;
    }
    for policy in ["required", "block"] {
        let required = source.replace(
            "BAD_POLICY",
            &format!(r#"#[sv("struo_memory = \"{policy}\"")]"#),
        );
        let error = analyze_and_lower(&required, "required_memory_fallback", "Top").unwrap_err();
        assert!(
            matches!(&error, ImportError::RequiredMemoryInferenceFailed { memory, .. } if memory == "z_bad"),
            "{error}"
        );
    }
}

#[test]
fn corpus_bounded_runtime_loop_initializers() {
    let stage = Rc::new(RefCell::new(String::new()));
    for name in [
        "basic::test_comb_loop_bound_output_call_writes_back_once",
        "comb_observer::test_comb_function_loop_bounds_apply_output_effects_left_to_right",
    ] {
        celox_test_suite_veryl::case(name)
            .unwrap()
            .run(&mut |design| compile(design, &stage));
    }
}

#[test]
fn bounded_runtime_starts_preserve_steps_effects_breaks_and_empty_ranges() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(clk: input clock, start: input logic<4>, limit: input logic<4>,
                   stop: input logic<4>, sum: output logic<8>, q: output logic<8>,
                   init_calls: output logic<8>, body_calls: output logic<8>) {
            function begin_loop(x: input logic<4>, calls: inout logic<8>) -> logic<4> {
                calls += 8'd1;
                return x;
            }
            function mark(x: input logic<4>, calls: inout logic<8>) -> logic<4> {
                calls += 8'd1;
                return x;
            }
            always_comb {
                init_calls = 0;
                body_calls = 0;
                sum = 0;
                for i in begin_loop(start, init_calls)..limit step += 3 {
                    if i == stop { break; }
                    sum += mark(i as 4, body_calls) as 8;
                }
                for unused in begin_loop(start, init_calls)..0 {
                    body_calls += 8'd10;
                }
            }
            always_ff (clk) {
                var tmp: logic<8>;
                tmp = 0;
                for i in start..=limit step += 2 { tmp += i as 8; }
                q = tmp;
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let clk = sim.event("clk");
    let start = sim.signal("start");
    let limit = sim.signal("limit");
    let stop = sim.signal("stop");
    for first in 0..16u8 {
        for end in 0..16u8 {
            for stop_at in 0..16u8 {
                sim.modify(|io| {
                    io.set(start, first);
                    io.set(limit, end);
                    io.set(stop, stop_at);
                })
                .unwrap();
                let values = (first..end)
                    .step_by(3)
                    .take_while(|i| *i != stop_at)
                    .collect::<Vec<_>>();
                assert_eq!(
                    sim.get(sim.signal("sum")),
                    values.iter().copied().sum::<u8>().into(),
                    "start={first}, end={end}, stop={stop_at}"
                );
                assert_eq!(sim.get(sim.signal("init_calls")), 2u8.into());
                assert_eq!(sim.get(sim.signal("body_calls")), values.len().into());
                sim.tick(clk).unwrap();
                assert_eq!(
                    sim.get(sim.signal("q")),
                    (first..=end).step_by(2).sum::<u8>().into()
                );
            }
        }
    }
}

#[test]
fn runtime_starts_reject_unproven_ranges_and_counter_overflow() {
    for (start_type, end_type, step, reason) in [
        ("signed logic<8>", "logic<4>", 1u32, "non-negative"),
        ("logic<32>", "logic<4>", 1, "non-negative"),
        ("logic<4>", "logic<32>", 1, "termination is not proven"),
        ("logic<4>", "logic<4>", 2_147_483_647, "overflow"),
    ] {
        let source = format!(
            "module Top(start: input {start_type}, limit: input {end_type}, q: output logic<8>) {{
                always_comb {{ q = 0; for i in start..limit step += {step} {{ q += 1; }} }}
             }}"
        );
        let error = analyze_and_lower(&source, "unproven_runtime_start", "Top").unwrap_err();
        assert!(
            matches!(&error, ImportError::UnsupportedBehavior(message) if message.contains(reason)),
            "{error}"
        );
    }
    let source = "module Top(start: input logic<4>, q: output logic<8>) {
        always_comb { q = 0; for i in $signed(start)..8 { q += 1; } }
    }";
    let error = analyze_and_lower(source, "signed_runtime_start", "Top").unwrap_err();
    assert!(
        matches!(&error, ImportError::UnsupportedBehavior(message) if message.contains("non-negative")),
        "{error}"
    );
}

#[test]
fn runtime_start_packed_select_remains_unsigned() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(d: input signed logic<8>, q: output logic<8>) {
            always_comb {
                q = 0;
                for i in d[3:0]..8 { q += i as 8; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    for start in 0..16u8 {
        sim.modify(|io| io.set(d, 0xf0u8 | start)).unwrap();
        assert_eq!(sim.get(sim.signal("q")), (start..8).sum::<u8>().into());
    }
}

#[test]
fn corpus_display_argument_function_effects() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case(
        "basic::test_comb_function_call_with_output_argument_in_display_argument",
    )
    .unwrap()
    .run(&mut |design| compile(design, &stage));
}

#[test]
fn output_task_arguments_preserve_short_circuit_and_break_effects() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r#"
        module Top(d: input logic<8>, gate: input logic, stop: input logic<3>, q: output logic<8>) {
            function bump(x: input logic<8>, calls: inout logic<8>) -> logic<8> {
                calls += x + 8'd1;
                return x;
            }
            always_comb {
                q = 0;
                $display("", 8'hxx);
                $display("%d", gate && bump(d, q));
                if !gate { $write("%d", bump(d, q)); }
                for i in 0..4 {
                    if i == stop { break; }
                    $write("%d", bump(i as 8, q));
                }
            }
        }
        "#,
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let d = sim.signal("d");
    let gate = sim.signal("gate");
    let stop = sim.signal("stop");
    for value in [0u8, 1, 255] {
        for enabled in [0u8, 1] {
            for stopped_at in 0..8u8 {
                sim.modify(|io| {
                    io.set(d, value);
                    io.set(gate, enabled);
                    io.set(stop, stopped_at);
                })
                .unwrap();
                assert_eq!(
                    sim.get(sim.signal("q")),
                    value
                        .wrapping_add(1)
                        .wrapping_add((0..stopped_at.min(4)).map(|i| i + 1).sum::<u8>())
                        .into()
                );
            }
        }
    }
}

#[test]
fn output_tasks_do_not_bypass_ff_function_write_restrictions() {
    for task in ["display", "write"] {
        let source = format!(
            r#"
            module Top(clk: input clock, q: output logic<8>) {{
                function mark(x: output logic<8>) -> logic {{ x = 8'd1; return 1'b1; }}
                always_ff(clk) {{ ${task}("%d", mark(q)); }}
            }}
        "#
        );
        let error = analyze_and_lower(&source, "ff_output_task_effects", "Top").unwrap_err();
        assert!(
            matches!(
                error,
                ImportError::AnalysisFailed(_) | ImportError::UnsupportedBehavior(_)
            ),
            "{error}"
        );
    }
}

#[test]
fn instance_input_function_output_effects_remain_rejected() {
    let source = r"
        module Child(i: input logic, o: output logic) { assign o = i; }
        module Top(d: input logic, q: output logic, side: output logic) {
            function mark(x: input logic, seen: output logic) -> logic { seen = x; return x; }
            inst u: Child(i: mark(d, side), o: q);
        }
    ";
    let error = analyze_and_lower(source, "nonprocedural_function_outputs", "Top").unwrap_err();
    assert!(
        matches!(&error, ImportError::UnsupportedBehavior(message) if message.contains("read-only expression")),
        "{error}"
    );
}

#[test]
fn case_loop_termination_preserves_target_effects_and_each_branch() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(count: input logic<32>, sel: input logic<2>, gate: input logic,
                   q: output logic<8>, calls: output logic<8>) {
            function target(x: input logic<2>, n: inout logic<8>) -> logic<2> {
                n += 1;
                return x;
            }
            always_comb {
                q = 0;
                calls = 0;
                for i in 0..count {
                    case target(sel, calls) {
                        0: { q = 10; break; }
                        1: {
                            if gate { q = 20; break; }
                            else { q = 30; break; }
                        }
                        default: { q = 40; break; }
                    }
                    q = 99;
                }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let count = sim.signal("count");
    let sel = sim.signal("sel");
    let gate = sim.signal("gate");
    for bound in [0u32, 1, 64, 255, u32::MAX] {
        for selector in 0..4u8 {
            for enabled in [0u8, 1] {
                sim.modify(|io| {
                    io.set(count, bound);
                    io.set(sel, selector);
                    io.set(gate, enabled);
                })
                .unwrap();
                let expected = if bound == 0 {
                    0u8
                } else {
                    match selector {
                        0 => 10,
                        1 if enabled == 1 => 20,
                        1 => 30,
                        _ => 40,
                    }
                };
                assert_eq!(sim.get(sim.signal("q")), expected.into());
                assert_eq!(sim.get(sim.signal("calls")), u8::from(bound != 0).into());
            }
        }
    }
}

#[test]
fn case_loop_termination_requires_every_path_to_break_this_loop() {
    for body in [
        "case sel { 0: break; }",
        "case sel { 0: break; default: { q += 1; } }",
        "case sel { 0: { q += 1; } default: break; }",
        "case sel { 0: break; default: { if gate { break; } } }",
        "case sel { 0: break; default: { for j in 0..2 { break; } } }",
    ] {
        let source = format!(
            "module Top(count: input logic<32>, sel: input logic<2>, gate: input logic,
                        q: output logic<8>) {{
                always_comb {{ q = 0; for i in 0..count {{ {body} }} }}
            }}"
        );
        let error = analyze_and_lower(&source, "case_termination", "Top").unwrap_err();
        assert!(
            matches!(error, ImportError::UnsupportedBehavior(_)),
            "{error}"
        );
        assert!(error.to_string().contains("termination"), "{error}");
    }
}

#[test]
fn byte_bound_loops_cover_the_full_range_and_break_boundary() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(count: input logic<8>, start: input logic<8>, stop: input logic<8>,
                   hits: output logic<9>, sum: output logic<16>, tail: output logic<9>) {
            always_comb {
                hits = 0;
                for i in 0..=count { hits = (i + 1) as 9; }
                sum = 0;
                for i in 0..count step += 3 {
                    if i == stop { break; }
                    sum ^= i as 16;
                }
                tail = 0;
                for i in start..=count { tail = (i + 1) as 9; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let count = sim.signal("count");
    let start = sim.signal("start");
    let stop = sim.signal("stop");
    for n in 0..256u16 {
        for stop_at in [0u16, 63, 64, 252, 255] {
            sim.modify(|io| {
                io.set(count, n);
                io.set(start, n ^ 128);
                io.set(stop, stop_at);
            })
            .unwrap();
            assert_eq!(sim.get(sim.signal("hits")), (n + 1).into());
            assert_eq!(
                sim.get(sim.signal("tail")),
                (if (n ^ 128) <= n { n + 1 } else { 0 }).into()
            );
            assert_eq!(
                sim.get(sim.signal("sum")),
                (0..n)
                    .step_by(3)
                    .take_while(|i| *i != stop_at)
                    .fold(0u16, |value, i| value ^ i)
                    .into()
            );
        }
    }
}

#[test]
fn static_loop_ranges_do_not_hide_counter_wrap_or_unsigned_reverse_sentinels() {
    for range in [
        "rev 8'd0..4",
        "rev 8'd1..3 step += 3",
        "rev 8'd0..0",
        "64'd4294967296..64'd4294967298",
        "2147483646..=2147483647",
    ] {
        let source = format!(
            "module Top(stop: input logic, q: output logic<8>) {{
                always_comb {{ q = 0; for i in {range} {{ if stop {{ break; }} q += 1; }} }}
            }}"
        );
        let error = analyze_and_lower(&source, "static_loop_range", "Top").unwrap_err();
        assert!(
            matches!(error, ImportError::UnsupportedBehavior(_)),
            "{range}: {error}"
        );
    }
}

#[test]
fn static_loop_proofs_preserve_signed_sentinels_and_guaranteed_breaks() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(stop: input logic, digits: output logic<16>, singleton: output logic<8>,
                   broken: output logic<8>, empty: output logic<8>) {
            always_comb {
                digits = 0;
                for i in rev 8'sd0..4 {
                    digits = digits * 10 + i as 16;
                    if stop { break; }
                }
                singleton = 0;
                for i in rev 8'd1..4 step += 3 {
                    singleton = i as 8;
                    if stop { break; }
                }
                broken = 0;
                for i in rev 8'd0..1 { broken += 1; break; }
                empty = 0;
                for i in rev 0..0 step += 2 { empty += 1; if stop { break; } }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let stop = sim.signal("stop");
    for halted in [0u8, 1] {
        sim.modify(|io| io.set(stop, halted)).unwrap();
        assert_eq!(
            sim.get(sim.signal("digits")),
            (if halted == 0 { 3210u16 } else { 3 }).into()
        );
        assert_eq!(sim.get(sim.signal("singleton")), 3u8.into());
        assert_eq!(sim.get(sim.signal("broken")), 1u8.into());
        assert_eq!(sim.get(sim.signal("empty")), 0u8.into());
    }
}

#[test]
fn corpus_reverse_loop_with_guaranteed_first_iteration_break() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case(
        "flip_flop::test_ff_runtime_reverse_min_i32_end_wraps_before_range_check",
    )
    .unwrap()
    .run(&mut |design| compile(design, &stage));
}

#[test]
fn single_iteration_loops_use_truncated_counters_and_preserve_nested_effects() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(start: input signed logic<64>, limit: input signed logic<64>, gate: input logic,
                   fwd: output logic<32>, reverse_value: output logic<32>, nested: output logic<32>,
                   calls: output logic<8>, branch: output logic<8>) {
            function seed(x: input signed logic<64>, n: inout logic<8>) -> signed logic<64> {
                n += 1;
                return x;
            }
            always_comb {
                calls = 0;
                fwd = 32'heeeeeeee;
                reverse_value = 32'hdddddddd;
                nested = 32'hcccccccc;
                branch = 0;
                for i in seed(start, calls)..limit step *= 2 {
                    fwd = i as 32;
                    for j in rev start..limit { nested = (i + j) as 32; break; }
                    if gate { branch = 1; break; } else { branch = 2; break; }
                }
                for i in rev start..seed(limit, calls) { reverse_value = i as 32; break; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let start = sim.signal("start");
    let limit = sim.signal("limit");
    let gate = sim.signal("gate");
    let values = [
        i64::MIN,
        -4_294_967_296,
        -2_147_483_648,
        -1,
        0,
        1,
        2_147_483_647,
        4_294_967_299,
        i64::MAX,
    ];
    for first in values {
        for end in values {
            for enabled in [0u8, 1] {
                sim.modify(|io| {
                    io.set(start, first.cast_unsigned());
                    io.set(limit, end.cast_unsigned());
                    io.set(gate, enabled);
                })
                .unwrap();
                let forward_counter =
                    i32::from_le_bytes(first.to_le_bytes()[..4].try_into().unwrap());
                let reverse_counter =
                    i32::from_le_bytes(end.wrapping_sub(1).to_le_bytes()[..4].try_into().unwrap());
                let forward_active = i64::from(forward_counter) < end;
                let reverse_active = i64::from(reverse_counter) >= first;
                assert_eq!(
                    sim.get(sim.signal("fwd")),
                    (if forward_active {
                        forward_counter.cast_unsigned()
                    } else {
                        0xeeee_eeee
                    })
                    .into()
                );
                assert_eq!(
                    sim.get(sim.signal("reverse_value")),
                    (if reverse_active {
                        reverse_counter.cast_unsigned()
                    } else {
                        0xdddd_dddd
                    })
                    .into()
                );
                assert_eq!(
                    sim.get(sim.signal("nested")),
                    (if forward_active && reverse_active {
                        forward_counter
                            .wrapping_add(reverse_counter)
                            .cast_unsigned()
                    } else {
                        0xcccc_cccc
                    })
                    .into()
                );
                assert_eq!(sim.get(sim.signal("calls")), 2u8.into());
                assert_eq!(
                    sim.get(sim.signal("branch")),
                    (if forward_active {
                        if enabled == 1 { 1u8 } else { 2 }
                    } else {
                        0
                    })
                    .into()
                );
            }
        }
    }
}

#[test]
fn single_iteration_ff_reverse_comparison_keeps_unsigned_bound_context() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(clk: input clock, start: input logic<64>, limit: input signed logic<64>,
                   q: output logic<32>) {
            always_ff (clk) {
                q = 32'heeeeeeee;
                for i in rev start..limit { q = i as 32; break; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let clk = sim.event("clk");
    let start = sim.signal("start");
    let limit = sim.signal("limit");
    for first in [
        0u64,
        1,
        2_147_483_647,
        4_294_967_295,
        4_294_967_296,
        u64::MAX,
    ] {
        for end in [
            i64::MIN,
            -2_147_483_648,
            -1,
            0,
            1,
            2_147_483_647,
            4_294_967_296,
            i64::MAX,
        ] {
            sim.modify(|io| {
                io.set(start, first);
                io.set(limit, end.cast_unsigned());
            })
            .unwrap();
            sim.tick(clk).unwrap();
            let counter =
                u32::from_le_bytes(end.wrapping_sub(1).to_le_bytes()[..4].try_into().unwrap());
            let expected = if u64::from(counter) >= first {
                counter
            } else {
                0xeeee_eeee
            };
            assert_eq!(
                sim.get(sim.signal("q")),
                expected.into(),
                "start={first}, end={end}"
            );
        }
    }
}

#[test]
fn single_iteration_counter_drives_dynamic_selects_and_constant_array_reads() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(start: input signed logic<64>, bits: output logic<16>, data: output logic<8>) {
            const A: logic<8> [4] = '{11, 22, 33, 44};
            always_comb {
                bits = 0;
                data = 0;
                for i in start..64'sh7fff_ffff_ffff_ffff step *= 2 {
                    bits[i] = 1;
                    data = A[i];
                    break;
                }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let start = sim.signal("start");
    for first in [-1i64, 0, 1, 3, 4, 15, 16, 2_147_483_648, 4_294_967_299] {
        sim.modify(|io| io.set(start, first.cast_unsigned()))
            .unwrap();
        let counter = i32::from_le_bytes(first.to_le_bytes()[..4].try_into().unwrap());
        let bits = if (0..16).contains(&counter) {
            1u16 << counter
        } else {
            0
        };
        let data = usize::try_from(counter)
            .ok()
            .and_then(|i| [11u8, 22, 33, 44].get(i).copied())
            .unwrap_or(0);
        assert_eq!(sim.get(sim.signal("bits")), bits.into());
        assert_eq!(sim.get(sim.signal("data")), data.into());
    }
}

#[test]
fn single_iteration_loops_reject_lost_constant_initializer_bits() {
    for start in ["128'h1_0000_0000_0000_0003", "128'sh1_0000_0000_0000_0003"] {
        let source = format!(
            "module Top(limit: input logic<128>, q: output logic<32>) {{
                always_comb {{ q = 0; for i in {start}..limit {{ q = i as 32; break; }} }}
            }}"
        );
        let error = analyze_and_lower(&source, "saturated_loop_initializer", "Top").unwrap_err();
        assert!(
            matches!(&error, ImportError::UnsupportedBehavior(message) if message.contains("saturated")),
            "{error}"
        );
    }
}

#[test]
fn corpus_proven_signed_bitwise_loop() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case(
        "synth_dynamic_loop::test_signed_xor_step_uses_loop_counter_width",
    )
    .unwrap()
    .run(&mut |design| compile(design, &stage));
}

#[test]
fn known_bitwise_initializers_preserve_sign_step_width_and_condition_order() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(limit: input signed logic<128>, xor_hits: output logic<8>, xor_last: output logic<32>,
                   or_hits: output logic<8>, or_last: output logic<32>,
                   falling_hits: output logic<8>, falling_last: output logic<32>) {
            always_comb {
                var seed: signed logic<8>;
                seed = 8'shf8;
                xor_hits = 0;
                xor_last = 32'heeeeeeee;
                for i in seed..=limit step ^= 2147483648 {
                    xor_hits += 1;
                    xor_last = i as 32;
                    seed = 0;
                    if i == 2147483640 { break; }
                }
                var other: signed logic<32>;
                other = (0 - 8) as 32;
                or_hits = 0;
                or_last = 32'heeeeeeee;
                for i in other..=limit step |= 4294967300 {
                    or_hits += 1;
                    or_last = i as 32;
                    if i == (0 - 4) { break; }
                }
                other = (0 - 5) as 32;
                falling_hits = 0;
                falling_last = 32'heeeeeeee;
                for i in other..=limit step ^= 4294967299 {
                    falling_hits += 1;
                    falling_last = i as 32;
                    if i == (0 - 8) { break; }
                }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let limit = sim.signal("limit");
    for bound in [
        i128::MIN,
        -9,
        -8,
        -7,
        -5,
        -4,
        -3,
        0,
        2_147_483_639,
        2_147_483_640,
        i128::MAX,
    ] {
        sim.modify(|io| io.set(limit, bound.cast_unsigned()))
            .unwrap();
        for (hits, last, trace) in [
            ("xor_hits", "xor_last", [-8i128, 2_147_483_640]),
            ("or_hits", "or_last", [-8i128, -4]),
            ("falling_hits", "falling_last", [-5i128, -8]),
        ] {
            let executed = trace
                .into_iter()
                .take_while(|i| *i <= bound)
                .collect::<Vec<_>>();
            let expected = executed.last().map_or(0xeeee_eeee, |n| {
                u32::from_le_bytes(n.to_le_bytes()[..4].try_into().unwrap())
            });
            assert_eq!(sim.get(sim.signal(hits)), executed.len().into());
            assert_eq!(sim.get(sim.signal(last)), expected.into());
        }
    }
}

#[test]
fn bitwise_loop_proofs_reject_cycles_and_unknown_initializers() {
    for (setup, range, condition) in [
        ("seed = 3;", "seed..=limit step |= 6", "i == limit"),
        ("seed = 3;", "seed..=limit step ^= 6", "i == 7"),
        ("seed = value;", "seed..=limit step |= 4", "i == 7"),
        ("seed = 0;", "seed_fn(calls)..=limit step |= 4", "i == 7"),
    ] {
        let source = format!(
            "module Top(value: input logic<32>, limit: input logic<32>, q: output logic<32>, calls: output logic<8>) {{
                function seed_fn(n: inout logic<8>) -> logic<32> {{ n += 1; return 3; }}
                always_comb {{ var seed: logic<32>; calls = 0; {setup} q = 0;
                    for i in {range} {{ q = i as 32; if {condition} {{ break; }} }}
                }}
            }}"
        );
        let error = analyze_and_lower(&source, "unproven_bitwise_loop", "Top").unwrap_err();
        assert!(
            matches!(error, ImportError::UnsupportedBehavior(_)),
            "{error}"
        );
    }
    let source = "module Top(clk: input clock, limit: input logic<32>, q: output logic<32>) {
        var seed: logic<32>;
        always_ff (clk) {
            seed = 3;
            q = 0;
            for i in seed..limit step |= 4 { q = i as 32; if i == 7 { break; } }
        }
    }";
    let error = analyze_and_lower(source, "nba_bitwise_initializer", "Top").unwrap_err();
    assert!(
        matches!(error, ImportError::UnsupportedBehavior(_)),
        "{error}"
    );
}

#[test]
fn bitwise_loop_proof_observes_blocking_ff_local_initialization() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(clk: input clock, count: input logic<8>, q: output logic<8>) {
            always_ff (clk) {
                var seed: logic<32>;
                var hits: logic<8>;
                seed = 3;
                hits = 0;
                for i in seed..count step |= 4 {
                    hits += 1;
                    if i == 7 { break; }
                }
                q = hits;
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let clk = sim.event("clk");
    let count = sim.signal("count");
    for n in 0..256u16 {
        sim.modify(|io| io.set(count, n)).unwrap();
        sim.tick(clk).unwrap();
        let expected = [3u16, 7].into_iter().take_while(|i| *i < n).count();
        assert_eq!(sim.get(sim.signal("q")), expected.into());
    }
}

#[test]
fn corpus_known_negative_additive_loop() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case("veryl_context_regressions::runtime_for_with_negative_bound")
        .unwrap()
        .run(&mut |design| compile(design, &stage));
}

#[test]
fn known_additive_loops_preserve_typed_bounds_and_capture_initializers() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(a: input logic<8>, signed_hits: output logic<8>, unsigned_hits: output logic<8>,
                   wrap_hits: output logic<8>, total: output logic<32>) {
            always_comb {
                var seed: i32;
                seed = -2;
                signed_hits = 0;
                total = 0;
                for i in (seed - 1)..=2 {
                    signed_hits += 1;
                    total += a;
                    seed = 100;
                }
                seed = -2;
                unsigned_hits = 0;
                for i in seed..32'd2 { unsigned_hits += 1; }
                seed = 2147483646;
                wrap_hits = 0;
                for i in seed..=32'd2147483647 { wrap_hits += 1; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let a = sim.signal("a");
    for value in 0..256u16 {
        sim.modify(|io| io.set(a, value)).unwrap();
        assert_eq!(sim.get(sim.signal("signed_hits")), 6u8.into());
        assert_eq!(sim.get(sim.signal("unsigned_hits")), 0u8.into());
        assert_eq!(sim.get(sim.signal("wrap_hits")), 2u8.into());
        assert_eq!(sim.get(sim.signal("total")), (6 * value).into());
    }
}

#[test]
fn known_additive_loops_reject_mutable_bounds_and_signed_wraparound() {
    for (setup, range, body) in [
        ("seed = 2147483646;", "seed..=2147483647", "q += 1;"),
        (
            "seed = -2; limit = 2;",
            "seed..limit",
            "limit += 1; q += 1;",
        ),
    ] {
        let source = format!(
            "module Top(value: input i32, q: output logic<32>) {{
                always_comb {{ var seed: i32; var limit: i32; limit = 0;
                    {setup} q = 0; for i in {range} {{ {body} }}
                }}
            }}"
        );
        let error = analyze_and_lower(&source, "unproven_additive_loop", "Top").unwrap_err();
        assert!(
            matches!(error, ImportError::UnsupportedBehavior(_)),
            "{error}"
        );
    }
}

#[test]
fn bounded_reverse_loops_capture_starts_and_preserve_steps_and_breaks() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(b: input logic<4>, stop: input logic<4>, exclusive: output logic<16>,
                   inclusive: output logic<16>, stepped: output logic<16>, forward: output logic<8>) {
            always_comb {
                var upper: logic<4>;
                upper = b;
                exclusive = 0;
                for i in rev 0..upper {
                    exclusive += (i + 1) as 16;
                    upper = 0;
                    if i == stop { break; }
                }
                inclusive = 0;
                for i in rev 0..=(b + 4'd1) { inclusive += (i + 1) as 16; }
                stepped = 0;
                for i in rev 0..b step += 3 { stepped += (i + 1) as 16; }
                forward = 0;
                for i in 0..=(b + 4'd1) { forward += 1; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let b = sim.signal("b");
    let stop = sim.signal("stop");
    for bound in 0..16u16 {
        for stop_value in 0..16u16 {
            sim.modify(|io| {
                io.set(b, bound);
                io.set(stop, stop_value);
            })
            .unwrap();
            let mut exclusive = 0u16;
            for i in (0..bound).rev() {
                exclusive += i + 1;
                if i == stop_value {
                    break;
                }
            }
            let stepped: u16 = (0..bound).rev().step_by(3).map(|i| i + 1).sum();
            assert_eq!(sim.get(sim.signal("exclusive")), exclusive.into());
            assert_eq!(
                sim.get(sim.signal("inclusive")),
                ((bound + 2) * (bound + 3) / 2).into()
            );
            assert_eq!(sim.get(sim.signal("stepped")), stepped.into());
            assert_eq!(sim.get(sim.signal("forward")), (bound + 2).into());
        }
    }
}

#[test]
fn bounded_reverse_loops_reject_unsigned_conditions_and_unproven_initializers() {
    for range in ["rev 8'd0..b", "rev 0..wide", "rev 0..signed_bound"] {
        let source = format!(
            "module Top(b: input logic<4>, wide: input logic<32>, signed_bound: input i32, q: output logic<32>) {{
                always_comb {{ q = 0; for i in {range} {{ q += 1; }} }}
            }}"
        );
        let error = analyze_and_lower(&source, "unproven_reverse_loop", "Top").unwrap_err();
        assert!(
            matches!(error, ImportError::UnsupportedBehavior(_)),
            "{error}"
        );
    }
}

#[test]
fn bounded_reverse_initialization_effects_and_ff_local_writes() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(clk: input clock, b: input logic<4>, q: output logic<16>,
                   calls: output logic<8>, comb: output logic<16>) {
            function upper(v: input logic<4>, n: inout logic<8>) -> logic<4> {
                n += 1; return v;
            }
            always_comb {
                calls = 0;
                comb = 0;
                for i in rev 2..upper(b, calls) step += 3 { comb += (i + 1) as 16; }
            }
            always_ff (clk) {
                var bound: logic<4>;
                var total: logic<16>;
                bound = b;
                total = 0;
                for i in rev 0..=bound { total += (i + 1) as 16; bound = 0; }
                q = total;
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let b = sim.signal("b");
    let clk = sim.event("clk");
    for bound in 0..16u16 {
        sim.modify(|io| io.set(b, bound)).unwrap();
        sim.tick(clk).unwrap();
        let comb: u16 = (2..bound).rev().step_by(3).map(|i| i + 1).sum();
        assert_eq!(
            sim.get(sim.signal("q")),
            ((bound + 1) * (bound + 2) / 2).into()
        );
        assert_eq!(sim.get(sim.signal("comb")), comb.into());
        assert_eq!(sim.get(sim.signal("calls")), 1u8.into());
    }
}

#[test]
fn arithmetic_loop_ranges_enforce_the_expansion_budget() {
    for (range, accepted) in [
        ("0..(b + 9'd1)", true),
        ("0..=(b + 9'd1)", false),
        ("rev 0..(b + 9'd1)", true),
        ("rev 0..=(b + 9'd1)", false),
    ] {
        let source = format!(
            "module Top(b: input logic<9>, q: output logic<32>) {{
                always_comb {{ q = 0; for i in {range} {{ q = i as 32; }} }}
            }}"
        );
        let result = analyze_and_lower(&source, "arithmetic_loop_budget", "Top");
        if accepted {
            result.unwrap();
        } else {
            assert!(matches!(result, Err(ImportError::UnsupportedBehavior(_))));
        }
    }
}

#[test]
fn constant_driven_reverse_bounds_preserve_negative_counters_and_ff_reads() {
    let stage = Rc::new(RefCell::new(String::new()));
    for (floor, kind) in [(-2i32, "i32"), (2, "i32"), (-2, "signed logic<64>")] {
        let source = r"
        module Top(clk: input clock, b: input logic<4>, hits: output logic<8>,
                   sum: output logic<32>, stepped: output logic<32>, q: output logic<32>) {
            var floor: i32;
            always_comb { floor = -2; }
            always_comb {
                hits = 0;
                sum = 0;
                for i in rev floor..b { hits += 1; sum += i as 32; }
                stepped = 0;
                for i in rev floor..b step += 3 { stepped += i as 32; }
            }
            always_ff (clk) {
                var total: logic<32>;
                total = 0;
                for i in rev floor..=b { total += i as 32; }
                q = total;
            }
        }
        "
        .replace("floor = -2;", &format!("floor = {floor};"))
        .replace("var floor: i32;", &format!("var floor: {kind};"));
        let design = Design::new(&source, "Top");
        let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
        let b = sim.signal("b");
        let clk = sim.event("clk");
        for bound in 0..16i32 {
            sim.modify(|io| io.set(b, bound.cast_unsigned())).unwrap();
            sim.tick(clk).unwrap();
            let sum: i32 = (floor..bound).sum();
            let stepped: i32 = (floor..bound).rev().step_by(3).sum();
            assert_eq!(
                sim.get(sim.signal("hits")),
                (bound - floor).max(0).cast_unsigned().into()
            );
            assert_eq!(sim.get(sim.signal("sum")), sum.cast_unsigned().into());
            assert_eq!(
                sim.get(sim.signal("q")),
                (floor..=bound).sum::<i32>().cast_unsigned().into()
            );
            assert_eq!(
                sim.get(sim.signal("stepped")),
                stepped.cast_unsigned().into()
            );
        }
    }
}

#[test]
fn constant_driven_reverse_bounds_do_not_assume_mutable_or_registered_values() {
    for (provider, body, suffix) in [
        ("always_comb { floor = input_floor; }", "q += 1;", ""),
        ("always_ff (clk) { floor = -2; }", "q += 1;", ""),
        ("always_comb { floor = -2147483648; }", "q += 1;", ""),
        ("always_comb { floor = -2; }", "floor -= 1; q += 1;", ""),
        (
            "always_comb { floor = -2; }",
            "q += 1;",
            "always_comb { floor = 0; }",
        ),
    ] {
        let source = format!(
            "module Top(clk: input clock, b: input logic<4>, input_floor: input i32, q: output logic<8>) {{
                var floor: i32; {provider}
                always_comb {{ q = 0; for i in rev floor..b {{ {body} }} }}
                {suffix}
            }}"
        );
        assert!(
            analyze_and_lower(&source, "unproven_constant_driver", "Top").is_err(),
            "{source}"
        );
    }
    let source = "module Top(b: input logic<4>, q: output logic<8>) {
        always_comb { var floor: i32; floor = -2; q = 0;
            for i in rev floor..b { floor -= 1; q += 1; }
        }
    }";
    assert!(analyze_and_lower(source, "mutable_local_bound", "Top").is_err());
}

#[test]
fn additive_reductions_preserve_modular_widths_and_signed_increments() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(seed: input logic<32>, count: input logic<32>,
                   sum: output logic<32>, narrow: output logic<8>,
                   signed_sum: output signed logic<64>, mixed_sum: output logic<64>) {
            always_comb {
                sum = seed;
                for i in 0..count { sum += 3; }
                narrow = seed as 8;
                for i in 0..count { narrow = 5 + narrow; }
                signed_sum = 0;
                for i in 0..count { signed_sum += 8'shff; }
                mixed_sum = 0;
                for i in 0..count { mixed_sum += 8'shff; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    for count in [
        0u32,
        1,
        2,
        255,
        256,
        512,
        65_535,
        0x7fff_ffff,
        0x8000_0000,
        u32::MAX,
    ] {
        for seed in [0u32, 1, 255, 0x8000_0000, u32::MAX] {
            let count_signal = sim.signal("count");
            let seed_signal = sim.signal("seed");
            sim.modify(|io| {
                io.set(count_signal, count);
                io.set(seed_signal, seed);
            })
            .unwrap();
            assert_eq!(
                sim.get(sim.signal("sum")),
                seed.wrapping_add(count.wrapping_mul(3)).into()
            );
            assert_eq!(
                sim.get(sim.signal("narrow")),
                (seed.wrapping_add(count.wrapping_mul(5)) & 255).into()
            );
            assert_eq!(
                sim.get(sim.signal("signed_sum")),
                0u64.wrapping_sub(u64::from(count)).into()
            );
            assert_eq!(
                sim.get(sim.signal("mixed_sum")),
                (u64::from(count) * 255).into()
            );
        }
    }
}

#[test]
fn additive_reductions_reject_unproven_bounds_and_dependent_bodies() {
    for (bound_type, range, body) in [
        ("logic<64>", "0..count", "out += 1;"),
        ("signed logic<32>", "0..count", "out += 1;"),
        ("logic<32>", "0..=count", "out += 1;"),
        ("logic<32>", "0..count step += 2", "out += 1;"),
        ("logic<32>", "0..count", "out += i as 32;"),
        ("logic<32>", "0..count", "out += seed;"),
        ("logic<32>", "0..out", "out += 1;"),
    ] {
        let source = format!(
            "module Top(count: input {bound_type}, seed: input logic<32>, out: output logic<32>) {{
            always_comb {{ out = seed; for i in {range} {{ {body} }} }}
        }}"
        );
        assert!(
            analyze_and_lower(&source, "unproven_reduction", "Top").is_err(),
            "{source}"
        );
    }
}

#[test]
fn invariant_additive_reductions_preserve_empty_ranges_and_boolean_guards() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(seed: input logic<8>, count: input logic<32>, gate: input logic<8>,
                   enabled: output logic<8>, stopped: output logic<8>) {
            always_comb {
                enabled = seed;
                for i in 0..count { if gate { enabled += 3; } }
                stopped = seed;
                for i in 0..count { stopped += 5; if gate { break; } }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let seed_signal = sim.signal("seed");
    let count_signal = sim.signal("count");
    let gate_signal = sim.signal("gate");
    for count in [0u32, 1, 255, 256, 512, 0x8000_0000, u32::MAX] {
        for gate in [0u8, 1, 2, 128] {
            for seed in [0u8, 17, 255] {
                sim.modify(|io| {
                    io.set(seed_signal, seed);
                    io.set(count_signal, count);
                    io.set(gate_signal, gate);
                })
                .unwrap();
                let enabled_count = if gate == 0 { 0 } else { count };
                let stopped_count = if gate == 0 {
                    count
                } else {
                    u32::from(count != 0)
                };
                assert_eq!(
                    sim.get(sim.signal("enabled")),
                    ((u32::from(seed).wrapping_add(enabled_count.wrapping_mul(3))) & 255).into()
                );
                assert_eq!(
                    sim.get(sim.signal("stopped")),
                    ((u32::from(seed).wrapping_add(stopped_count.wrapping_mul(5))) & 255).into()
                );
            }
        }
    }
}

#[test]
fn invariant_additive_reductions_reject_changing_or_effectful_guards() {
    for body in [
        "if out { out += 1; }",
        "if i { out += 1; }",
        "out += 1; if out { break; }",
        "out += 1; if i == 2147483646 { break; }",
        "if gate { out += 1; } else { out += 2; }",
        "if effect(gate, calls) { out += 1; }",
        "out += 1; if effect(gate, calls) { break; }",
    ] {
        let source = format!("module Top(count: input logic<32>, gate: input logic, out: output logic<32>, calls: output logic<32>) {{
            function effect(x: input logic, n: inout logic<32>) -> logic {{ n += 1; return x; }}
            always_comb {{ out = 0; calls = 0; for i in 0..count {{ {body} }} }}
        }}");
        assert!(
            analyze_and_lower(&source, "dependent_reduction_guard", "Top").is_err(),
            "{source}"
        );
    }
}

#[test]
fn idempotent_loops_preserve_partial_writes_order_and_empty_ranges() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(seed: input logic<3>, count: input logic<32>, idx: input logic<32>,
                   copied: output logic<3>, selected: output logic) {
            var x: logic<2>;
            always_comb {
                copied = seed;
                for i in 0..count { copied[0] = copied[1]; }
                x = seed as 2;
                selected = 1;
                for i in 0..count { x[0] = 0; selected = x[idx]; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let seed_signal = sim.signal("seed");
    let count_signal = sim.signal("count");
    let index_signal = sim.signal("idx");
    for count in [0u32, 1, 2, 256, 0x8000_0000, u32::MAX] {
        for seed in 0..8u8 {
            for index in [0u32, 1, 2, 3, u32::MAX] {
                sim.modify(|io| {
                    io.set(seed_signal, seed);
                    io.set(count_signal, count);
                    io.set(index_signal, index);
                })
                .unwrap();
                let copied = if count == 0 {
                    seed
                } else {
                    (seed & 6) | ((seed >> 1) & 1)
                };
                let selected = if count == 0 {
                    1
                } else if index == 1 {
                    (seed >> 1) & 1
                } else {
                    0
                };
                assert_eq!(sim.get(sim.signal("copied")), copied.into());
                assert_eq!(sim.get(sim.signal("selected")), selected.into());
            }
        }
    }
}

#[test]
fn idempotent_loops_reject_unproven_state_and_counter_dependencies() {
    for body in [
        "x[i as 8] = 0;",
        "x[0] = i as 1;",
        "x[0] = x[index];",
        "y = x[index]; x[0] = 0;",
        "x[0] = effect(calls);",
        "x[effect(calls)] = 0;",
        "x[0] = x[effect(calls)];",
        "limit = 0;",
    ] {
        let source = format!("module Top(count: input logic<32>, seed: input logic<2>, index: input logic<32>, x: output logic<2>, y: output logic, calls: output logic<32>) {{
            function effect(n: inout logic<32>) -> logic {{ n += 1; return 0; }}
            always_comb {{ var limit: logic<32>; limit = count; x = seed; y = 0; calls = 0;
                for i in 0..limit {{ {body} }}
            }}
        }}");
        assert!(
            analyze_and_lower(&source, "unproven_idempotence", "Top").is_err(),
            "{source}"
        );
    }
}

#[test]
fn small_state_loops_preserve_cycles_transients_and_full_counts() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(seed: input logic<3>, count: input logic<32>,
                   partial: output logic<2>, rot: output logic<3>, mix: output logic<3>,
                   down: output logic<3>) {
            always_comb {
                partial = seed as 2;
                for i in 0..count { partial[0] = partial == 2'b10; }
                rot = seed;
                for i in 0..count { rot = {rot[1:0], ~rot[2]}; }
                mix = seed;
                for i in 0..count { mix = (mix * 3) as 3; mix += 1; }
                down = seed;
                for i in 0..count { down = if down == 0 ? 3 : down - 1; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let seed_signal = sim.signal("seed");
    let count_signal = sim.signal("count");
    for count in [0u32, 1, 2, 3, 4, 5, 6, 255, 256, 0x8000_0000, u32::MAX] {
        for seed in 0..8u8 {
            sim.modify(|io| {
                io.set(seed_signal, seed);
                io.set(count_signal, count);
            })
            .unwrap();
            let partial = if count == 0 {
                seed & 3
            } else if seed & 2 == 0 {
                0
            } else {
                (seed & 3) ^ u8::from(count & 1 != 0)
            };
            let mut rot = seed;
            for _ in 0..count % 6 {
                rot = ((rot << 1) | ((!rot >> 2) & 1)) & 7;
            }
            let mut mix = seed;
            for _ in 0..count % 4 {
                mix = (mix * 3 + 1) & 7;
            }
            assert_eq!(sim.get(sim.signal("partial")), partial.into());
            assert_eq!(sim.get(sim.signal("rot")), rot.into());
            assert_eq!(sim.get(sim.signal("mix")), mix.into());
            let down = if count <= u32::from(seed) {
                u32::from(seed) - count
            } else {
                (4 - (count - u32::from(seed)) % 4) % 4
            };
            assert_eq!(sim.get(sim.signal("down")), down.into());
        }
    }
}

#[test]
fn small_state_loops_reject_external_dependencies_and_large_tables() {
    for (width, body) in [
        (5, "x[0] = ~x[0];"),
        (3, "x = x ^ seed;"),
        (3, "x = x ^ count as 3;"),
        (3, "x = x ^ i as 3;"),
    ] {
        let source = format!("module Top(count: input logic<32>, seed: input logic<{width}>, x: output logic<{width}>) {{
            always_comb {{ x = seed; for i in 0..count {{ {body} }} }}
        }}");
        assert!(
            analyze_and_lower(&source, "unproven_state_transition", "Top").is_err(),
            "{source}"
        );
    }
}

#[test]
fn sparse_index_loops_preserve_order_and_late_wrapped_writes() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(seed: input logic<4>, count: input logic<32>,
                   chain: output logic<4>, reverse: output logic<4>, wrapped: output logic<4>,
                   middle: output logic<4>) {
            always_comb {
                chain = seed;
                for i in 0..count { chain[i + 1] = chain[i]; chain[i] = ~chain[i]; }
                reverse = 0;
                for i in 0..count { reverse[3 - i] = seed[i]; }
                wrapped = seed;
                for i in 0..count { wrapped[i + 3] = 1; }
                middle = seed;
                for i in 0..count { middle[i + 32'h8000_0000] = 1; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let seed_signal = sim.signal("seed");
    let count_signal = sim.signal("count");
    for count in [
        0u32,
        1,
        2,
        3,
        4,
        5,
        256,
        0x7fff_ffff,
        0x8000_0000,
        0x8000_0001,
        0x8000_0004,
        u32::MAX - 2,
        u32::MAX - 1,
        u32::MAX,
    ] {
        for seed in 0..16u8 {
            sim.modify(|io| {
                io.set(seed_signal, seed);
                io.set(count_signal, count);
            })
            .unwrap();
            let mut chain = seed;
            let mut reverse = 0u8;
            for i in 0..count.min(4) {
                if i < 3 {
                    chain = (chain & !(1 << (i + 1))) | (((chain >> i) & 1) << (i + 1));
                }
                chain ^= 1 << i;
                reverse |= ((seed >> i) & 1) << (3 - i);
            }
            let wrapped = seed
                | if count > 0 { 8 } else { 0 }
                | u8::from(count >= u32::MAX - 1)
                | if count == u32::MAX { 2 } else { 0 };
            assert_eq!(
                sim.get(sim.signal("chain")),
                chain.into(),
                "chain: seed={seed}, count={count}"
            );
            assert_eq!(
                sim.get(sim.signal("reverse")),
                reverse.into(),
                "reverse: seed={seed}, count={count}"
            );
            assert_eq!(
                sim.get(sim.signal("wrapped")),
                wrapped.into(),
                "wrapped: seed={seed}, count={count}"
            );
            let middle = seed | ((1 << count.saturating_sub(0x8000_0000).min(4)) - 1);
            assert_eq!(
                sim.get(sim.signal("middle")),
                middle.into(),
                "middle: seed={seed}, count={count}"
            );
        }
    }
}

#[test]
fn sparse_index_loops_reject_unproven_indices_and_skipped_effects() {
    for body in [
        "x[i as 8] = 0;",
        "x[(i as 64) + 1] = 0;",
        "x[i + gate] = 0;",
        "x[i * 2] = 0;",
        "x[i] = effect(calls);",
        "x[i] = 0; calls += 1;",
        "x[i] = 0; limit -= 1;",
        "x[i] = 0; if gate { break; }",
    ] {
        let source = format!("module Top(count: input logic<32>, gate: input logic<32>, x: output logic<4>, calls: output logic<32>) {{
            function effect(n: inout logic<32>) -> logic {{ n += 1; return 1; }}
            always_comb {{ var limit: logic<32>; limit = count; x = 0; calls = 0;
                for i in 0..limit {{ {body} }}
            }}
        }}");
        assert!(
            analyze_and_lower(&source, "unproven_sparse_loop", "Top").is_err(),
            "{source}"
        );
    }
}

#[test]
fn periodic_reductions_preserve_offsets_signed_casts_and_modular_counts() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(seed: input logic<8>, count: input logic<32>,
                   hits: output logic<32>, masked: output logic<8>,
                   negative: output signed logic<64>, mixed: output logic<64>) {
            always_comb {
                hits = seed;
                for i in 254..count { if (i as u8) <: 8'd4 { hits += 3; } }
                masked = seed;
                for i in 5..count { if ((i as u8) & 8'd3) == 8'd1 { masked += 5; } }
                negative = 0;
                for i in 128..count { if (i as i8) <: 0 { negative += 8'shff; } }
                mixed = 0;
                for i in 254..count { if (i as u8) <: 8'd4 { mixed += 8'shff; } }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let seed_signal = sim.signal("seed");
    let count_signal = sim.signal("count");
    for count in [
        0u32,
        1,
        4,
        5,
        6,
        127,
        128,
        129,
        254,
        255,
        256,
        257,
        260,
        511,
        512,
        0x8000_0000,
        u32::MAX,
    ] {
        for seed in [0u8, 17, 255] {
            sim.modify(|io| {
                io.set(seed_signal, seed);
                io.set(count_signal, count);
            })
            .unwrap();
            let n = u64::from(count);
            let hits = (n / 256 * 4 + (n % 256).min(4)).saturating_sub(4);
            let masked = ((n + 2) / 4).saturating_sub(1);
            let negative = n / 256 * 128 + (n % 256).saturating_sub(128);
            assert_eq!(
                sim.get(sim.signal("hits")),
                ((u64::from(seed) + hits * 3) & u64::from(u32::MAX)).into(),
                "count={count}"
            );
            assert_eq!(
                sim.get(sim.signal("masked")),
                ((u64::from(seed) + masked * 5) & 255).into(),
                "count={count}"
            );
            assert_eq!(
                sim.get(sim.signal("negative")),
                0u64.wrapping_sub(negative).into(),
                "count={count}"
            );
            assert_eq!(
                sim.get(sim.signal("mixed")),
                (hits * 255).into(),
                "count={count}"
            );
        }
    }
}

#[test]
fn periodic_reductions_reject_nonperiodic_or_effectful_conditions() {
    for body in [
        "if i <: 4 { out += 1; }",
        "if (i as u16) <: 4 { out += 1; }",
        "if (i as u8) <: count { out += 1; }",
        "if (i as u8) <: out { out += 1; }",
        "if (i as u8) <: 4 { out += i as 32; }",
        "if (i as u8) <: 4 { out += seed; }",
        "if effect(calls) { out += 1; }",
        "if (i as u8) <: 4 { out += 1; calls += 1; }",
        "if (i as u8) <: 4 { out += 1; } else { out += 2; }",
    ] {
        let source = format!("module Top(count: input logic<32>, seed: input logic<32>, out: output logic<32>, calls: output logic<32>) {{
            function effect(n: inout logic<32>) -> logic {{ n += 1; return 1; }}
            always_comb {{ out = seed; calls = 0; for i in 254..count {{ {body} }} }}
        }}");
        assert!(
            analyze_and_lower(&source, "unproven_periodic_reduction", "Top").is_err(),
            "{source}"
        );
    }
}

#[test]
fn signed_periodic_starts_preserve_conversion_and_capture() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(start: input logic<64>, short_start: input signed logic<8>, seed: input logic<8>,
                   hits: output logic<32>, narrow: output logic<8>,
                   mixed: output logic<64>, captured: output logic<32>) {
            always_comb {
                hits = seed;
                for i in start..260 { if (i as u8) <: 8'd4 { hits += 3; } }
                narrow = seed;
                for i in short_start..260 { if (i as u8) <: 8'd4 { narrow += 5; } }
                mixed = 0;
                for i in start..260 { if (i as i8) <: 0 { mixed += 8'shff; } }
                captured = start as 32;
                for i in captured..260 { if (i as u8) <: 8'd4 { captured += 1; } }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let start_signal = sim.signal("start");
    let small_signal = sim.signal("short_start");
    let seed_signal = sim.signal("seed");
    let prefix = |n: i64| n.div_euclid(256) * 4 + n.rem_euclid(256).min(4);
    let signed_prefix = |n: i64| n.div_euclid(256) * 128 + (n.rem_euclid(256) - 128).max(0);
    for start in [
        0u64,
        1,
        3,
        4,
        127,
        128,
        254,
        255,
        256,
        259,
        260,
        261,
        0x7fff_ffff,
        0x8000_0000,
        0xffff_ff00,
        0xffff_ff80,
        0xffff_fffe,
        0xffff_ffff,
        0x1_0000_00fe,
        0xffff_ffff_8000_0000,
        u64::MAX,
    ] {
        let bytes = start.to_le_bytes();
        let bits = u32::from_le_bytes(bytes[..4].try_into().unwrap());
        let signed_start = i64::from(i32::from_le_bytes(bits.to_le_bytes()));
        let small = bytes[0];
        let small_start = i64::from(i8::from_le_bytes([small]));
        let hits = u64::try_from((prefix(260) - prefix(signed_start)).max(0)).unwrap();
        let narrow = u64::try_from((prefix(260) - prefix(small_start)).max(0)).unwrap();
        let mixed =
            u64::try_from((signed_prefix(260) - signed_prefix(signed_start)).max(0)).unwrap();
        for seed in [0u8, 17, 255] {
            sim.modify(|io| {
                io.set(start_signal, start);
                io.set(small_signal, small);
                io.set(seed_signal, seed);
            })
            .unwrap();
            assert_eq!(
                sim.get(sim.signal("hits")),
                ((u64::from(seed) + hits * 3) & u64::from(u32::MAX)).into(),
                "start={start}"
            );
            assert_eq!(
                sim.get(sim.signal("narrow")),
                ((u64::from(seed) + narrow * 5) & 255).into(),
                "start={start}"
            );
            assert_eq!(
                sim.get(sim.signal("mixed")),
                (mixed * 255).into(),
                "start={start}"
            );
            assert_eq!(
                sim.get(sim.signal("captured")),
                ((u64::from(bits) + hits) & u64::from(u32::MAX)).into(),
                "start={start}"
            );
        }
    }
}

#[test]
fn signed_periodic_starts_reject_unproven_ranges_and_effects() {
    for range in [
        "start..32'd260",
        "start..64'sd2147483648",
        "start..=260",
        "start..260 step += 2",
        "start..finish",
        "effect(start, calls)..260",
    ] {
        let source = format!("module Top(start: input logic<32>, finish: input logic<32>, out: output logic<32>, calls: output logic<32>) {{
            function effect(x: input logic<32>, n: inout logic<32>) -> logic<32> {{ n += 1; return x; }}
            always_comb {{ out = 0; calls = 0; for i in {range} {{ if (i as u8) <: 4 {{ out += 1; }} }} }}
        }}");
        assert!(
            analyze_and_lower(&source, "unproven_signed_periodic_range", "Top").is_err(),
            "{source}"
        );
    }
}

#[test]
fn linear_reductions_preserve_steps_last_values_and_captured_starts() {
    let stage = Rc::new(RefCell::new(String::new()));
    let design = Design::new(
        r"
        module Top(start: input logic<64>, seed: input logic<8>,
                   hits10: output logic<32>, last10: output logic<8>, narrow: output logic<8>,
                   hits300: output signed logic<64>, last300: output signed logic<64>, captured: output logic<32>,
                   full: output logic<64>) {
            always_comb {
                var local_start: i32;
                local_start = start as i32;
                hits10 = seed; last10 = 8'hee; narrow = seed;
                for i in start..255 step += 10 { hits10 += 3; last10 = i; narrow += 5; }
                hits300 = seed; last300 = -99; captured = start as 32;
                for i in captured..255 step += 300 { captured += 1; hits300 += 8'shff; last300 = i; }
                full = 0;
                for i in local_start..2147483647 { full += 1; }
            }
        }
        ",
        "Top",
    );
    let mut sim = celox_test_suite_veryl::Simulator::new(compile(&design, &stage).unwrap());
    let start_signal = sim.signal("start");
    let seed_signal = sim.signal("seed");
    for start in [
        0u64,
        1,
        9,
        10,
        127,
        245,
        249,
        250,
        254,
        255,
        256,
        0x7fff_ffff,
        0x8000_0000,
        0x8000_0001,
        0xffff_ff00,
        0xffff_ffd2,
        0xffff_ffd3,
        0xffff_fffe,
        0xffff_ffff,
        0x1_0000_00fa,
        u64::MAX,
    ] {
        let bytes = start.to_le_bytes();
        let initial_bits = u32::from_le_bytes(bytes[..4].try_into().unwrap());
        let initial = i64::from(i32::from_le_bytes(initial_bits.to_le_bytes()));
        let trip_count = |step| {
            if initial >= 255 {
                0
            } else {
                (254 - initial) / step + 1
            }
        };
        let n10 = trip_count(10);
        let n300 = trip_count(300);
        let last10 = if n10 == 0 {
            238
        } else {
            initial + (n10 - 1) * 10
        };
        let last300 = if n300 == 0 {
            -99
        } else {
            initial + (n300 - 1) * 300
        };
        let n10 = u64::try_from(n10).unwrap();
        let n300 = u64::try_from(n300).unwrap();
        for seed in [0u8, 17, 255] {
            sim.modify(|io| {
                io.set(start_signal, start);
                io.set(seed_signal, seed);
            })
            .unwrap();
            for (name, expected) in [
                (
                    "full",
                    u64::try_from(i64::from(i32::MAX) - initial).unwrap(),
                ),
                ("hits10", (u64::from(seed) + n10 * 3) & u64::from(u32::MAX)),
                ("last10", u64::from_le_bytes(last10.to_le_bytes()) & 255),
                ("narrow", (u64::from(seed) + n10 * 5) & 255),
                ("hits300", u64::from(seed).wrapping_sub(n300)),
                ("last300", u64::from_le_bytes(last300.to_le_bytes())),
                (
                    "captured",
                    (u64::from(initial_bits) + n300) & u64::from(u32::MAX),
                ),
            ] {
                assert_eq!(
                    sim.get(sim.signal(name)),
                    expected.into(),
                    "{name}: start={start}"
                );
            }
        }
    }
}

#[test]
fn linear_reductions_reject_dependent_updates_and_wrapping_exit_steps() {
    for (range, body) in [
        ("start..255 step += 10", "hits += i as 32;"),
        ("start..255 step += 10", "last = i; hits += last;"),
        ("start..255 step += 10", "hits += 1; hits += 2;"),
        ("start..255 step += 10", "hits += effect(calls);"),
        ("start..255 step += 10", "if hits { break; } hits += 1;"),
        ("start..2147483647 step += 2", "hits += 1;"),
        ("start..=2147483647", "hits += 1;"),
        ("start..64'sd2147483648", "hits += 1;"),
        ("start..32'd255 step += 10", "hits += 1;"),
        ("start..255 step += 2147483648", "hits += 1;"),
        ("start..255 step *= 2", "hits += 1;"),
    ] {
        let source = format!("module Top(start: input logic<32>, hits: output logic<32>, last: output logic<32>, calls: output logic<32>) {{
            function effect(n: inout logic<32>) -> logic<32> {{ n += 1; return 1; }}
            always_comb {{ hits = 0; last = 0; calls = 0; for i in {range} {{ {body} }} }}
        }}");
        assert!(
            analyze_and_lower(&source, "unproven_linear_loop", "Top").is_err(),
            "{source}"
        );
    }
}
