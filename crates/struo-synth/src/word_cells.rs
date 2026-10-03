//! Share pure word operations after their complete input nets are resolved.
//!
//! This cache lives only during lowering, before netlist rewrites. Keys retain
//! operand order, width, comparison signedness, and the optional carry input.
use std::collections::HashMap;

use struo_ir::{ArithmeticOp, ComparisonOp, NetId, Netlist};

#[derive(Hash, PartialEq, Eq)]
struct ArithmeticKey {
    operation: ArithmeticOp,
    lhs: Vec<NetId>,
    rhs: Vec<NetId>,
    carry: Option<NetId>,
}

#[derive(Hash, PartialEq, Eq)]
struct ComparisonKey {
    operation: ComparisonOp,
    lhs: Vec<NetId>,
    rhs: Vec<NetId>,
}

#[derive(Default)]
pub(super) struct WordCells {
    arithmetic: HashMap<ArithmeticKey, Vec<NetId>>,
    comparisons: HashMap<ComparisonKey, NetId>,
}

impl WordCells {
    pub(super) fn arithmetic(
        &mut self,
        netlist: &mut Netlist,
        operation: ArithmeticOp,
        lhs: &[NetId],
        rhs: &[NetId],
        carry: Option<NetId>,
    ) -> Vec<NetId> {
        let key = ArithmeticKey {
            operation,
            lhs: lhs.to_vec(),
            rhs: rhs.to_vec(),
            carry,
        };
        self.arithmetic
            .entry(key)
            .or_insert_with(|| {
                if let Some(carry) = carry {
                    assert_eq!(operation, ArithmeticOp::Add);
                    netlist.add_arithmetic_with_carry(lhs, rhs, carry)
                } else {
                    netlist.add_arithmetic(operation, lhs, rhs)
                }
                .expect("validated RTL arithmetic has equal, non-zero widths")
            })
            .clone()
    }

    pub(super) fn comparison(
        &mut self,
        netlist: &mut Netlist,
        operation: ComparisonOp,
        lhs: &[NetId],
        rhs: &[NetId],
    ) -> NetId {
        let key = ComparisonKey {
            operation,
            lhs: lhs.to_vec(),
            rhs: rhs.to_vec(),
        };
        *self.comparisons.entry(key).or_insert_with(|| {
            netlist
                .add_comparison(operation, lhs, rhs)
                .expect("validated RTL comparisons have equal, non-zero widths")
        })
    }
}
