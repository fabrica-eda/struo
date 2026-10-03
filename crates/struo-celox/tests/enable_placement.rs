//! Placement-guided CE branches must preserve independent register behavior.
use std::collections::BTreeMap;
use struo_ir::{ActiveLevel, ClockEdge, EnableControl, Netlist, RegisterCell, ResetControl};
use struo_target_ecp5::{
    Bit, Ecp5Cell, Ecp5Netlist, MappingOptions, RegisterEnableFanoutConstraint,
    RegisterEnablePlacement, map_to_ecp5_with_options,
};

fn fixture() -> Ecp5Netlist {
    let mut source = Netlist::new("placed_enables");
    let clock = source.add_input("clock");
    let reset = source.add_input("rst_n");
    let lhs = source.add_input("lhs");
    let rhs = source.add_input("rhs");
    let a = source.add_and(lhs, rhs);
    let b = source.add_xor(lhs, rhs);
    for i in 0..16 {
        let data = source.add_input(format!("data{i}"));
        let name = format!("value[{i}]");
        let q = source.add_register_output(&name);
        source.add_register(RegisterCell::new(
            name,
            q,
            data,
            clock,
            ClockEdge::Rising,
            Some(EnableControl {
                signal: if i < 12 { a } else { b },
                active: if i % 3 == 0 {
                    ActiveLevel::Low
                } else {
                    ActiveLevel::High
                },
            }),
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::Low,
                asynchronous: true,
                value: i % 2 != 0,
            }),
        ));
        source.add_output(format!("out{i}"), q);
    }
    map_to_ecp5_with_options(
        &source,
        MappingOptions {
            retiming: false,
            ..Default::default()
        },
    )
    .unwrap()
}

fn hints() -> BTreeMap<String, RegisterEnablePlacement> {
    // Logical neighbors are far apart. Nonadjacent bits share an enable site.
    (0..8_u32)
        .chain(12..16)
        .map(|i| {
            (
                format!("ff_value[{i}]"),
                RegisterEnablePlacement {
                    x: (i % 2) * 100,
                    y: (i % 4) / 2,
                    shared_enable: Some(format!("CE{}", i % 4)),
                },
            )
        })
        .collect()
}

fn enables(net: &Ecp5Netlist) -> BTreeMap<&str, Bit> {
    net.cells()
        .iter()
        .filter_map(|cell| match cell {
            Ecp5Cell::FlipFlop {
                name,
                enable: Some(enable),
                ..
            } => Some((name.as_str(), enable.signal)),
            _ => None,
        })
        .collect()
}

#[test]
fn nearby_groups_preserve_limits_shared_sites_and_control_functions() {
    let mut net = fixture();
    let report = net
        .apply_register_enable_fanout_with_placement(
            &[RegisterEnableFanoutConstraint::new("ff_value[*]", 4)],
            &hints(),
        )
        .unwrap();
    assert_eq!(report.matched_registers, 16);
    assert_eq!(report.rewired_registers, 16);
    assert_eq!(report.inserted_branches, 4);
    assert!(net.retiming().equivalence_signed_off);
    let ce = enables(&net);
    for group in [[0, 2, 4, 6], [1, 3, 5, 7], [8, 9, 10, 11], [12, 13, 14, 15]] {
        let expected = ce[format!("ff_value[{}]", group[0]).as_str()];
        for i in group {
            assert_eq!(ce[format!("ff_value[{i}]").as_str()], expected);
        }
        assert_eq!(ce.values().filter(|&&wire| wire == expected).count(), 4);
    }
    assert_ne!(ce["ff_value[0]"], ce["ff_value[12]"]);
}

#[test]
fn missing_hints_are_identical_to_legacy_and_invalid_constraints_are_atomic() {
    let original = fixture();
    let limits = [RegisterEnableFanoutConstraint::new("ff_value[*]", 4)];
    let mut legacy = original.clone();
    legacy
        .apply_register_enable_fanout_constraints(&limits)
        .unwrap();
    for positions in [
        BTreeMap::new(),
        BTreeMap::from([(
            "absent".into(),
            RegisterEnablePlacement {
                x: 1,
                y: 2,
                shared_enable: None,
            },
        )]),
    ] {
        let mut candidate = original.clone();
        candidate
            .apply_register_enable_fanout_with_placement(&limits, &positions)
            .unwrap();
        assert_eq!(candidate, legacy);
    }
    for bad in [
        vec![RegisterEnableFanoutConstraint::new("ff_value[*]", 0)],
        vec![RegisterEnableFanoutConstraint::new("absent", 4)],
        vec![limits[0].clone(), limits[0].clone()],
    ] {
        let mut candidate = original.clone();
        assert!(
            candidate
                .apply_register_enable_fanout_with_placement(&bad, &hints())
                .is_err()
        );
        assert_eq!(candidate, original);
    }
}

#[test]
fn placed_enable_branches_match_independent_cycle_oracle() {
    let original = fixture();
    let mut grouped = original.clone();
    grouped
        .apply_register_enable_fanout_with_placement(
            &[RegisterEnableFanoutConstraint::new("ff_value[*]", 4)],
            &hints(),
        )
        .unwrap();
    let mut sims = [original, grouped].map(|net| {
        struo_celox::ecp5_simulator(&net)
            .unwrap()
            .build_native()
            .unwrap()
    });
    let mut expected = [false; 16];
    for cycle in 0..1024_u32 {
        let reset_n = cycle % 23 != 0;
        let lhs = cycle & 1 != 0;
        let rhs = cycle & 2 != 0;
        for (i, value) in expected.iter_mut().enumerate() {
            let enable = if i < 12 { lhs && rhs } else { lhs ^ rhs };
            let enabled = if i % 3 == 0 { !enable } else { enable };
            if !reset_n {
                *value = i % 2 != 0;
            } else if enabled {
                *value = cycle & (4 << (i % 6)) != 0;
            }
        }
        for sim in &mut sims {
            for (name, value) in [("rst_n", reset_n), ("lhs", lhs), ("rhs", rhs)] {
                let signal = sim.signal(name);
                sim.modify(|io| io.set(signal, u8::from(value))).unwrap();
            }
            for i in 0..16 {
                let signal = sim.signal(&format!("data{i}"));
                sim.modify(|io| io.set(signal, u8::from(cycle & (4 << (i % 6)) != 0)))
                    .unwrap();
            }
            sim.tick(sim.event("clock")).unwrap();
            for (i, &value) in expected.iter().enumerate() {
                assert_eq!(
                    sim.get(sim.signal(&format!("out{i}")))
                        .to_u64_digits()
                        .first()
                        .copied()
                        .unwrap_or(0),
                    u64::from(value),
                    "cycle={cycle} register={i}"
                );
            }
        }
    }
}
