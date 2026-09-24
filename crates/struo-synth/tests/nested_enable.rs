//! Regression tests for clock enables inferred from nested priority updates.

use std::collections::HashMap;
use struo_ir::{ActiveLevel, ClockEdge, ComparisonOp, NetId, Netlist, NodeKind, RegisterCell};
use struo_synth::{InferRegisterEnables, Pass};

fn values(design: &Netlist, inputs: &HashMap<String, bool>) -> Vec<bool> {
    let mut values = vec![false; design.nodes().len()];
    for node in design.nodes() {
        let pins = node.inputs();
        values[node.output().index() as usize] = match node.kind() {
            NodeKind::Input(name) | NodeKind::RegisterOutput(name) => inputs[name],
            NodeKind::Constant(value) => *value,
            NodeKind::Not => !values[pins[0].index() as usize],
            NodeKind::And => values[pins[0].index() as usize] & values[pins[1].index() as usize],
            NodeKind::Or => values[pins[0].index() as usize] | values[pins[1].index() as usize],
            NodeKind::Xor => values[pins[0].index() as usize] ^ values[pins[1].index() as usize],
            NodeKind::Mux => {
                values[pins[usize::from(!values[pins[0].index() as usize]) + 1].index() as usize]
            }
            other => panic!("unexpected node: {other:?}"),
        };
    }
    values
}

fn add_register(design: &mut Netlist, state: NetId, next_data: NetId, clk: NetId) {
    design.add_register(RegisterCell::new(
        format!("state_{}", state.index()),
        state,
        next_data,
        clk,
        ClockEdge::Rising,
        None,
        None,
    ));
}

#[test]
fn priority_updates_infer_enable_and_preserve_all_transitions() {
    let mut design = Netlist::new("priority");
    let clk = design.add_input("clk");
    let select_a = design.add_input("select_a");
    let select_b = design.add_input("select_b");
    let input_data = design.add_input("input_data");
    let other_data = design.add_input("other_data");
    let state = design.add_register_output("state");
    // if select_a { state=input_data; } else if select_b { state=other_data; } else { hold; }
    let update_predicate = design.add_or(select_a, select_b);
    let inner = design.add_mux(select_b, other_data, state);
    let next_data = design.add_mux(select_a, input_data, inner);
    add_register(&mut design, state, next_data, clk);
    InferRegisterEnables.run(&mut design).unwrap();
    let register = &design.registers()[0];
    let enable = register.enable().expect("nested self-hold must infer CE");
    assert_eq!(enable.signal, update_predicate);
    for pattern in 0..32 {
        let mut inputs = HashMap::from([("clk".into(), false)]);
        for (bit, name) in ["select_a", "select_b", "input_data", "other_data", "state"]
            .into_iter()
            .enumerate()
        {
            inputs.insert(name.into(), pattern & (1 << bit) != 0);
        }
        let nets = values(&design, &inputs);
        let active = nets[enable.signal.index() as usize] == (enable.active == ActiveLevel::High);
        assert_eq!(active, inputs["select_a"] || inputs["select_b"]);
        let next = if active {
            nets[register.data().index() as usize]
        } else {
            inputs["state"]
        };
        assert_eq!(next, nets[next_data.index() as usize], "pattern={pattern}");
    }
    design.validate().unwrap();
}

#[test]
fn branching_shared_holds_preserve_priority_and_real_feedback_updates() {
    let mut design = Netlist::new("branching");
    let clk = design.add_input("clk");
    let select_a = design.add_input("select_a");
    let select_b = design.add_input("select_b");
    let select_c = design.add_input("select_c");
    let input_data = design.add_input("input_data");
    let state = design.add_register_output("state");
    let other = design.add_register_output("other");
    // Q ^ input_data is select_a real update, even though it depends on the old Q.
    let toggle = design.add_xor(state, input_data);
    let shared = design.add_mux(select_c, state, toggle);
    let left = design.add_mux(select_b, shared, other);
    let right = design.add_mux(select_b, input_data, shared);
    let next_data = design.add_mux(select_a, left, right);
    add_register(&mut design, state, next_data, clk);
    // Another register consumes the original, shared mux DAG. It must not
    // accidentally acquire the first register's self-hold predicate.
    add_register(&mut design, other, next_data, clk);
    InferRegisterEnables.run(&mut design).unwrap();
    for pattern in 0..64 {
        let mut inputs = HashMap::from([("clk".into(), false)]);
        for (bit, name) in [
            "select_a",
            "select_b",
            "select_c",
            "input_data",
            "state",
            "other",
        ]
        .into_iter()
        .enumerate()
        {
            inputs.insert(name.into(), pattern & (1 << bit) != 0);
        }
        let nets = values(&design, &inputs);
        for register in design.registers() {
            let enable = register
                .enable()
                .expect("both registers have select_a conditional hold");
            let active =
                nets[enable.signal.index() as usize] == (enable.active == ActiveLevel::High);
            let next = nets[if active {
                register.data()
            } else {
                register.output()
            }
            .index() as usize];
            assert_eq!(next, nets[next_data.index() as usize], "pattern={pattern}");
        }
        let enable = design.registers()[0].enable().unwrap();
        let active = nets[enable.signal.index() as usize] == (enable.active == ActiveLevel::High);
        assert_eq!(
            active,
            (inputs["select_a"] != inputs["select_b"]) || !inputs["select_c"]
        );
    }
    design.validate().unwrap();
}

#[test]
fn feedback_in_data_or_selector_without_a_hold_is_not_an_enable() {
    let mut design = Netlist::new("updates");
    let clk = design.add_input("clk");
    let select_a = design.add_input("select_a");
    let input_data = design.add_input("input_data");
    let state = design.add_register_output("state");
    let not_q = design.add_not(state);
    let toggle = design.add_xor(state, input_data);
    let branch = design.add_mux(select_a, toggle, not_q);
    let next_data = design.add_mux(state, branch, input_data);
    add_register(&mut design, state, next_data, clk);
    let before = design.clone();
    InferRegisterEnables.run(&mut design).unwrap();
    assert_eq!(design, before);
}

#[test]
fn deep_shared_mux_dags_are_processed_without_recursive_or_exponential_growth() {
    let mut design = Netlist::new("deep");
    let clk = design.add_input("clk");
    let input_data = design.add_input("input_data");
    let state = design.add_register_output("state");
    let mut next_data = state;
    for index in 0..10_000 {
        let select_a = design.add_input(format!("select_a{index}"));
        let select_b = design.add_input(format!("select_b{index}"));
        let right = design.add_mux(select_b, next_data, input_data);
        next_data = design.add_mux(select_a, next_data, right);
    }
    add_register(&mut design, state, next_data, clk);
    let before = design.nodes().len();
    InferRegisterEnables.run(&mut design).unwrap();
    assert!(design.registers()[0].enable().is_some());
    assert!(design.nodes().len() < before * 3);
    design.validate().unwrap();
}

#[test]
fn direct_hold_preserves_the_existing_outer_enable_and_inner_data_mux() {
    let mut design = Netlist::new("direct");
    let clk = design.add_input("clk");
    let outer = design.add_input("outer");
    let inner = design.add_input("inner");
    let data = design.add_input("data");
    let state = design.add_register_output("state");
    let inner_mux = design.add_mux(inner, data, state);
    let next_data = design.add_mux(outer, inner_mux, state);
    add_register(&mut design, state, next_data, clk);
    InferRegisterEnables.run(&mut design).unwrap();
    let register = &design.registers()[0];
    let enable = register.enable().unwrap();
    assert_eq!(enable.signal, outer);
    assert_eq!(enable.active, ActiveLevel::High);
    assert_eq!(register.data(), inner_mux);
}

#[test]
fn nested_set_reset_preserves_the_compact_feedback_function() {
    let mut design = Netlist::new("set_reset");
    let clk = design.add_input("clk");
    let set = design.add_input("set");
    let reset = design.add_input("reset");
    let state = design.add_register_output("state");
    let zero = design.add_constant(false);
    let one = design.add_constant(true);
    let set_mux = design.add_mux(set, one, state);
    let next = design.add_mux(reset, zero, set_mux);
    add_register(&mut design, state, next, clk);
    let before = design.clone();
    InferRegisterEnables.run(&mut design).unwrap();
    assert_eq!(design, before);
}

#[test]
fn multiple_writers_of_the_same_constant_still_infer_an_enable() {
    let mut design = Netlist::new("constant_update");
    let clk = design.add_input("clk");
    let first = design.add_input("first");
    let second = design.add_input("second");
    let state = design.add_register_output("state");
    let one = design.add_constant(true);
    let inner = design.add_mux(second, one, state);
    let next = design.add_mux(first, one, inner);
    add_register(&mut design, state, next, clk);
    InferRegisterEnables.run(&mut design).unwrap();
    assert!(design.registers()[0].enable().is_some());
    assert_eq!(design.registers()[0].data(), one);
}

#[test]
fn nested_state_dependent_guards_keep_feedback_in_the_data_cone() {
    for retained in 0..3 {
        let mut design = Netlist::new("state_guard");
        let clk = design.add_input("clk");
        let select = design.add_input("select");
        let payload = design.add_input("payload");
        let other = design.add_input("other");
        let state = design.add_register_output("state");
        let guard = match retained {
            0 => design.add_xor(state, other),
            1 => design
                .add_comparison(ComparisonOp::LessThanUnsigned, &[state], &[other])
                .unwrap(),
            _ => design
                .add_arithmetic_with_carry(&[payload], &[other], state)
                .unwrap()[0],
        };
        let inner = design.add_mux(guard, payload, state);
        let next = design.add_mux(select, other, inner);
        add_register(&mut design, state, next, clk);
        let original = design.registers()[0].clone();
        InferRegisterEnables.run(&mut design).unwrap();
        assert_eq!(design.registers()[0], original);
        design.validate().unwrap();
    }
}
