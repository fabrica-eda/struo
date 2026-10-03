//! Cofactor LUT truth tables before allocating simulation expressions.
use celox::frontend_sdk::{BuildError, ExprId};

use super::BitOptimizer;

impl BitOptimizer<'_> {
    pub fn lut(&mut self, inputs: &[ExprId], table: u16) -> Result<ExprId, BuildError> {
        assert_eq!(inputs.len(), 4);
        self.lut_cofactor(inputs, table)
    }

    fn lut_cofactor(&mut self, inputs: &[ExprId], table: u16) -> Result<ExprId, BuildError> {
        let mask = u16::MAX >> (16 - (1 << inputs.len()));
        if table == 0 || table == mask {
            return Ok(if table == 0 {
                self.constants.zero
            } else {
                self.constants.one
            });
        }
        let (&select, remaining) = inputs.split_first().expect("nonconstant truth table");
        let (low, high) = cofactors(table, remaining.len());
        if select == self.constants.zero || select == self.constants.one {
            let table = if select == self.constants.zero {
                low
            } else {
                high
            };
            return self.lut_cofactor(remaining, table);
        }
        // Equal cofactors do not depend on this input. Complementary ones
        // toggle exactly when the input does, including parity LUTs.
        let low_expr = self.lut_cofactor(remaining, low)?;
        if low == high {
            return Ok(low_expr);
        }
        let remaining_mask = u16::MAX >> (16 - (1 << remaining.len()));
        if low ^ high == remaining_mask {
            return self.xor(select, low_expr);
        }
        let high_expr = self.lut_cofactor(remaining, high)?;
        self.mux(select, high_expr, low_expr)
    }
}

fn cofactors(table: u16, remaining: usize) -> (u16, u16) {
    let mut low = 0;
    let mut high = 0;
    for bit in 0..(1 << remaining) {
        low |= ((table >> (2 * bit)) & 1) << bit;
        high |= ((table >> (2 * bit + 1)) & 1) << bit;
    }
    (low, high)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optimize::BitConstants;
    use celox::frontend_sdk::{BinaryOp, Constant, ExprNode, ModuleBuilder, ValueType};

    #[test]
    fn parity_uses_three_xors_and_fixed_luts_allocate_no_logic() {
        let mut builder = ModuleBuilder::new("compact_luts").unwrap();
        let zero = builder.constant(Constant::two_state(0u8, 1).unwrap());
        let one = builder.constant(Constant::two_state(1u8, 1).unwrap());
        let constants = BitConstants { zero, one };
        for address in 0..16u16 {
            let inputs = (0..4)
                .map(|bit| if address & (1 << bit) == 0 { zero } else { one })
                .collect::<Vec<_>>();
            for table in 0..=u16::MAX {
                let value = BitOptimizer::new(&mut builder, constants)
                    .lut(&inputs, table)
                    .unwrap();
                assert_eq!(
                    value,
                    if table & (1 << address) == 0 {
                        zero
                    } else {
                        one
                    }
                );
            }
        }
        let mut inputs = Vec::new();
        for index in 0..4 {
            let signal = builder
                .input(format!("i{index}"), ValueType::bits(1).unwrap())
                .unwrap();
            inputs.push(builder.read_slice(builder.whole(signal).unwrap()).unwrap());
        }
        BitOptimizer::new(&mut builder, constants)
            .lut(&inputs, 0x6996)
            .unwrap();
        let artifact = builder.finish();
        assert_eq!(artifact.expressions().len(), 9);
        assert_eq!(
            artifact
                .expressions()
                .iter()
                .filter(|expression| matches!(
                    expression.node(),
                    ExprNode::Binary {
                        op: BinaryOp::Xor,
                        ..
                    }
                ))
                .count(),
            3
        );
        assert!(
            !artifact
                .expressions()
                .iter()
                .any(|expression| matches!(expression.node(), ExprNode::Mux { .. }))
        );
    }
}
