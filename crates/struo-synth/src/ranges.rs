//! Conservative ranges of concrete bit patterns before word-cell lowering.
//!
//! Signals retain their full domain: neither drivers nor register initial
//! values are inferred. Unsupported operations and uncertain wraparound retain
//! the full domain. The result only permits zero-extending narrower adds.
use struo_rtl::{BinaryOp, ExprId, ExprKind, Module};

#[derive(Clone, Copy)]
struct Range {
    min: u64,
    max: u64,
}

pub(super) fn adder_widths(module: &Module) -> Vec<Option<usize>> {
    let mut ranges: Vec<Option<Range>> = Vec::with_capacity(module.expressions().len());
    let mut widths = Vec::with_capacity(module.expressions().len());
    for expression in module.expressions() {
        let width = expression.r#type().width.get();
        if width > u64::BITS {
            ranges.push(None);
            widths.push(None);
            continue;
        }
        let mask = u64::MAX >> (u64::BITS - width);
        let full = Range { min: 0, max: mask };
        let get = |id: ExprId| ranges[id.index() as usize];
        let range = (|| {
            Some(match expression.kind() {
                ExprKind::Constant(value) => {
                    let value =
                        (0..width).fold(0, |bits, bit| bits | (u64::from(value.bit(bit)) << bit));
                    Range {
                        min: value,
                        max: value,
                    }
                }
                ExprKind::Binary { op, lhs, rhs } => {
                    binary_range(*op, get(*lhs)?, get(*rhs)?, mask)?
                }
                ExprKind::Mux {
                    condition,
                    then_expr,
                    else_expr,
                } => {
                    let (yes, no) = (get(*then_expr)?, get(*else_expr)?);
                    match get(*condition) {
                        Some(Range { min: 0, max: 0 }) => no,
                        Some(Range { min: 1, max: 1 }) => yes,
                        _ => Range {
                            min: yes.min.min(no.min),
                            max: yes.max.max(no.max),
                        },
                    }
                }
                ExprKind::Slice { input, lsb } => {
                    let input = get(*input)?;
                    if input.min == input.max {
                        let value = input.max.checked_shr(*lsb)? & mask;
                        Range {
                            min: value,
                            max: value,
                        }
                    } else {
                        Range {
                            min: 0,
                            max: input.max.checked_shr(*lsb)?.min(mask),
                        }
                    }
                }
                ExprKind::Concat(parts) => {
                    let mut result = Range { min: 0, max: 0 };
                    for part in parts {
                        let part_width = module.expressions()[part.index() as usize]
                            .r#type()
                            .width
                            .get();
                        let part = get(*part)?;
                        result.min = result.min.checked_shl(part_width).unwrap_or(0) | part.min;
                        result.max = result.max.checked_shl(part_width).unwrap_or(0) | part.max;
                    }
                    result
                }
                _ => full,
            })
        })()
        .unwrap_or(full);
        let needed = (u64::BITS - range.max.leading_zeros()).max(1);
        widths.push(
            (matches!(
                expression.kind(),
                ExprKind::Binary {
                    op: BinaryOp::Add,
                    ..
                }
            ) && needed < width)
                .then_some(needed as usize),
        );
        ranges.push(Some(range));
    }
    widths
}

fn binary_range(op: BinaryOp, lhs: Range, rhs: Range, mask: u64) -> Option<Range> {
    Some(match op {
        BinaryOp::Add if lhs.min == lhs.max && rhs.min == rhs.max => {
            let value = lhs.min.wrapping_add(rhs.min) & mask;
            Range {
                min: value,
                max: value,
            }
        }
        BinaryOp::Add => {
            let max = lhs.max.checked_add(rhs.max).filter(|max| *max <= mask)?;
            Range {
                min: lhs.min + rhs.min,
                max,
            }
        }
        BinaryOp::Sub if lhs.min == lhs.max && rhs.min == rhs.max => {
            let value = lhs.min.wrapping_sub(rhs.min) & mask;
            Range {
                min: value,
                max: value,
            }
        }
        BinaryOp::Sub if lhs.min >= rhs.max => Range {
            min: lhs.min - rhs.max,
            max: lhs.max - rhs.min,
        },
        BinaryOp::And => Range {
            min: 0,
            max: lhs.max.min(rhs.max),
        },
        _ => Range { min: 0, max: mask },
    })
}

#[cfg(test)]
mod tests {
    use super::{Range, binary_range};
    use struo_rtl::BinaryOp;

    #[test]
    fn arithmetic_ranges_contain_every_small_domain_result() {
        for width in 1..=3 {
            let mask = (1u64 << width) - 1;
            for lhs_min in 0..=mask {
                for lhs_max in lhs_min..=mask {
                    for rhs_min in 0..=mask {
                        for rhs_max in rhs_min..=mask {
                            let lhs = Range {
                                min: lhs_min,
                                max: lhs_max,
                            };
                            let rhs = Range {
                                min: rhs_min,
                                max: rhs_max,
                            };
                            for op in [BinaryOp::Add, BinaryOp::Sub, BinaryOp::And] {
                                let range = binary_range(op, lhs, rhs, mask)
                                    .unwrap_or(Range { min: 0, max: mask });
                                for a in lhs_min..=lhs_max {
                                    for b in rhs_min..=rhs_max {
                                        let value = match op {
                                            BinaryOp::Add => a.wrapping_add(b) & mask,
                                            BinaryOp::Sub => a.wrapping_sub(b) & mask,
                                            BinaryOp::And => a & b,
                                            _ => unreachable!(),
                                        };
                                        assert!((range.min..=range.max).contains(&value));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
