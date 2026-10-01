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
        println!("STRUO_CASE {}", case.name);
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
