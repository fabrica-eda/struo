//! Cycle checks for explicit register replication, including active-low controls.
use struo_ir::{ActiveLevel, ClockEdge, EnableControl, Netlist, RegisterCell, ResetControl};
use struo_target_ecp5::{
    Bit, Ecp5Cell, MappingOptions, RegisterBranchReplication, map_to_ecp5_with_options,
};

#[test]
#[allow(clippy::too_many_lines)]
fn explicit_register_branches_preserve_reset_enable_and_cycle_outputs() {
    let mut source = Netlist::new("register_branches");
    let clock = source.add_input("clock");
    let data = source.add_input("data");
    let enable = source.add_input("enable_n");
    let reset = source.add_input("rst_n");
    let state = source.add_register_output("state");
    source.add_register(RegisterCell::new(
        "state",
        state,
        data,
        clock,
        ClockEdge::Rising,
        Some(EnableControl {
            signal: enable,
            active: ActiveLevel::Low,
        }),
        Some(ResetControl {
            signal: reset,
            active: ActiveLevel::Low,
            asynchronous: true,
            value: false,
        }),
    ));
    for i in 0..3 {
        let input = source.add_input(format!("in{i}"));
        let result = source.add_xor(state, input);
        source.add_output(format!("out{i}"), result);
    }
    let mapped = map_to_ecp5_with_options(
        &source,
        MappingOptions {
            retiming: false,
            ..MappingOptions::default()
        },
    )
    .unwrap();
    let (driver, wire) = mapped
        .cells()
        .iter()
        .find_map(|cell| match cell {
            Ecp5Cell::FlipFlop { name, output, .. } => Some((name.clone(), *output)),
            _ => None,
        })
        .unwrap();
    let sinks = mapped
        .cells()
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::Lut4 { name, inputs, .. } if inputs.contains(&Bit::Wire(wire)) => {
                Some(name.clone())
            }
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(sinks.len(), 3);
    let mut cloned = mapped.clone();
    for sink in &sinks[..2] {
        cloned
            .replicate_register_branches(&[RegisterBranchReplication {
                driver: driver.clone(),
                sinks: vec![sink.clone()],
            }])
            .unwrap();
    }
    let mut sims = [mapped, cloned].map(|net| {
        struo_celox::ecp5_simulator(&net)
            .unwrap()
            .build_native()
            .unwrap()
    });
    let mut expected_state = false;
    for cycle in 0..512_u32 {
        let reset_n = cycle % 17 != 0;
        let enable_n = cycle % 3 == 0;
        let data = cycle & 4 != 0;
        if !reset_n {
            expected_state = false;
        } else if !enable_n {
            expected_state = data;
        }
        for sim in &mut sims {
            for (name, value) in [
                ("rst_n", reset_n),
                ("enable_n", enable_n),
                ("data", data),
                ("in0", cycle & 1 != 0),
                ("in1", cycle & 2 != 0),
                ("in2", cycle & 8 != 0),
            ] {
                let signal = sim.signal(name);
                sim.modify(|io| io.set(signal, u8::from(value))).unwrap();
            }
            sim.tick(sim.event("clock")).unwrap();
            for (i, bit) in [1, 2, 8].into_iter().enumerate() {
                let actual = sim
                    .get(sim.signal(&format!("out{i}")))
                    .to_u64_digits()
                    .first()
                    .copied()
                    .unwrap_or(0);
                assert_eq!(
                    actual,
                    u64::from(expected_state ^ (cycle & bit != 0)),
                    "cycle={cycle}, output={i}"
                );
            }
        }
    }
}
