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
        "synth_dynamic_loop::test_runtime_break_in_synth_comb_loop",
        "synth_dynamic_loop::test_runtime_break_after_assign_in_synth_comb_loop",
        "flip_flop::test_ff_runtime_for_break",
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
#[ignore = "Veryl 0.21.0 folds signed size-cast division with incorrect signedness"]
fn upstream_constant_size_cast_regression() {
    let stage = Rc::new(RefCell::new(String::new()));
    celox_test_suite_veryl::case(
        "expression_semantics::constant_and_runtime_casts_use_the_same_resize_rule",
    )
    .unwrap()
    .run(&mut |design| compile(design, &stage));
}

#[test]
#[ignore = "Veryl 0.21.0 folds the actual before applying the function formal width"]
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
