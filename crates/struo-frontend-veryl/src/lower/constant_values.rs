//! Read-only constant proofs on typed RTL, without changing the emitted graph.
use veryl_analyzer::ir::ForBound;

use super::{BinaryOp, Constant, Env, ExprId, ExprKind, ImportError, ModuleLowerer, UnaryOp};

impl ModuleLowerer<'_> {
    pub(super) fn known_rtl_constant(&self, id: ExprId) -> Option<Constant> {
        let expressions = self.rtl.expressions();
        let target = &expressions[id.index() as usize];
        if let ExprKind::Constant(value) = target.kind() {
            return Some(value.clone());
        }
        // Conservative scalar evaluator: unknown signals and unsupported
        // operations cannot provide a termination proof. RTL dependencies are
        // earlier entries, so shared subexpressions are evaluated only once.
        let mut values: Vec<Option<u64>> = Vec::new();
        for expression in &expressions[..=id.index() as usize] {
            let width = expression.r#type().width.get();
            let value = (|| {
                if width > u64::BITS {
                    return None;
                }
                let get = |id: ExprId| values[id.index() as usize];
                Some(
                    match expression.kind() {
                        ExprKind::Constant(value) => (0..width)
                            .fold(0, |bits, bit| bits | (u64::from(value.bit(bit)) << bit)),
                        ExprKind::Binary { op, lhs, rhs } => {
                            let operand_width =
                                expressions[lhs.index() as usize].r#type().width.get();
                            constant_binary(*op, get(*lhs)?, get(*rhs)?, operand_width)?
                        }
                        ExprKind::Unary {
                            op: UnaryOp::BitNot,
                            input,
                        } => !get(*input)?,
                        ExprKind::Mux {
                            condition,
                            then_expr,
                            else_expr,
                        } => {
                            if get(*condition)? == 0 {
                                get(*else_expr)?
                            } else {
                                get(*then_expr)?
                            }
                        }
                        ExprKind::Slice { input, lsb } => get(*input)?.checked_shr(*lsb)?,
                        ExprKind::Concat(parts) => {
                            let mut bits = 0u64;
                            for part in parts {
                                let width = expressions[part.index() as usize].r#type().width.get();
                                bits = bits.checked_shl(width).unwrap_or(0) | get(*part)?;
                            }
                            bits
                        }
                        _ => return None,
                    } & (u64::MAX >> (u64::BITS - width)),
                )
            })();
            values.push(value);
        }
        Some(Constant::from_u64(
            target.r#type().width,
            values[id.index() as usize]?,
        ))
    }

    pub(super) fn known_loop_initializer(
        &mut self,
        start: &ForBound,
        reads: &Env,
        writes: &Env,
        sequential: bool,
        width: u32,
    ) -> Result<Option<usize>, ImportError> {
        match start {
            ForBound::Const(value, signed) => {
                let saturation = if *signed {
                    usize::try_from(i64::MAX).unwrap_or(usize::MAX)
                } else {
                    usize::MAX
                };
                Ok((*value != saturation).then_some(*value))
            }
            ForBound::Expression(expression) => {
                let snapshot = if sequential {
                    self.sequential_reads(reads, writes)
                } else {
                    writes.clone()
                };
                // Read-only lowering rejects initializer writes; skipping its
                // later evaluation is safe only for an effect-free constant.
                let value = self.lower_expression(expression, &snapshot)?;
                let Some(constant) = self.known_rtl_constant(value.id) else {
                    return Ok(None);
                };
                let mut bits = 0usize;
                for bit in 0..width {
                    let set = if bit < value.width {
                        constant.bit(bit)
                    } else {
                        value.signed && constant.bit(value.width - 1)
                    };
                    if set {
                        bits |= 1usize << bit;
                    }
                }
                Ok(Some(bits))
            }
        }
    }
}

fn constant_binary(op: BinaryOp, lhs: u64, rhs: u64, width: u32) -> Option<u64> {
    // Flipping the sign bit orders two's-complement bit patterns as unsigned keys.
    let sign = 1u64 << (width - 1);
    let (signed_lhs, signed_rhs) = (lhs ^ sign, rhs ^ sign);
    Some(match op {
        BinaryOp::Add => lhs.wrapping_add(rhs),
        BinaryOp::Sub => lhs.wrapping_sub(rhs),
        BinaryOp::Mul => lhs.wrapping_mul(rhs),
        BinaryOp::Equal => u64::from(lhs == rhs),
        BinaryOp::NotEqual => u64::from(lhs != rhs),
        BinaryOp::LessThanUnsigned => u64::from(lhs < rhs),
        BinaryOp::LessOrEqualUnsigned => u64::from(lhs <= rhs),
        BinaryOp::GreaterThanUnsigned => u64::from(lhs > rhs),
        BinaryOp::GreaterOrEqualUnsigned => u64::from(lhs >= rhs),
        BinaryOp::LessThanSigned => u64::from(signed_lhs < signed_rhs),
        BinaryOp::LessOrEqualSigned => u64::from(signed_lhs <= signed_rhs),
        BinaryOp::GreaterThanSigned => u64::from(signed_lhs > signed_rhs),
        BinaryOp::GreaterOrEqualSigned => u64::from(signed_lhs >= signed_rhs),
        BinaryOp::And => lhs & rhs,
        BinaryOp::Or => lhs | rhs,
        BinaryOp::Xor => lhs ^ rhs,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::{BinaryOp, constant_binary};

    #[test]
    fn constant_comparisons_preserve_operand_width_and_sign() {
        for width in 1..=8 {
            let values = (0..(1u64 << width)).collect::<Vec<_>>();
            check_comparisons(width, &values);
        }
        check_comparisons(
            64,
            &[
                0,
                1,
                i64::MAX.unsigned_abs(),
                1u64 << 63,
                u64::MAX - 1,
                u64::MAX,
            ],
        );
    }

    fn check_comparisons(width: u32, values: &[u64]) {
        let signed = |bits: u64| {
            if bits & (1 << (width - 1)) == 0 {
                i128::from(bits)
            } else {
                i128::from(bits) - (1i128 << width)
            }
        };
        for &lhs in values {
            for &rhs in values {
                for (op, expected) in [
                    (BinaryOp::Equal, lhs == rhs),
                    (BinaryOp::NotEqual, lhs != rhs),
                    (BinaryOp::LessThanUnsigned, lhs < rhs),
                    (BinaryOp::LessOrEqualUnsigned, lhs <= rhs),
                    (BinaryOp::GreaterThanUnsigned, lhs > rhs),
                    (BinaryOp::GreaterOrEqualUnsigned, lhs >= rhs),
                    (BinaryOp::LessThanSigned, signed(lhs) < signed(rhs)),
                    (BinaryOp::LessOrEqualSigned, signed(lhs) <= signed(rhs)),
                    (BinaryOp::GreaterThanSigned, signed(lhs) > signed(rhs)),
                    (BinaryOp::GreaterOrEqualSigned, signed(lhs) >= signed(rhs)),
                ] {
                    assert_eq!(
                        constant_binary(op, lhs, rhs, width),
                        Some(u64::from(expected)),
                        "{op:?}: width={width}, lhs={lhs}, rhs={rhs}"
                    );
                }
            }
        }
    }
}
