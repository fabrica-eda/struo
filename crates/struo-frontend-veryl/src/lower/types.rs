use veryl_analyzer::ir::{Comptime, Expression, Factor, Op, ValueVariant, VarSelect};

use super::is_boolean_operator;

// AIR keeps intrinsic types separate from the propagated expression context.
// IEEE 1800-2023 11.8.2 requires widening operands before applying the operator.
// Numeric size casts preserve source signedness, which is not reliably
// reflected in AIR's propagated expression context. Recompute the common
// type before lowering, including unsigned context inherited from a parent.
pub(super) fn expression_signedness(expression: &Expression) -> bool {
    match expression {
        Expression::Binary(lhs, Op::As, rhs, comptime) => {
            if matches!(rhs.comptime().value, ValueVariant::Type(_)) {
                comptime.r#type.signed
            } else {
                expression_signedness(lhs)
            }
        }
        Expression::Binary(_, op, _, _) if is_boolean_operator(*op) => false,
        Expression::Binary(lhs, op, rhs, _) => {
            expression_signedness(lhs)
                && (matches!(
                    op,
                    Op::LogicShiftL | Op::LogicShiftR | Op::ArithShiftL | Op::ArithShiftR | Op::Pow
                ) || expression_signedness(rhs))
        }
        Expression::Unary(op, input, _) => {
            matches!(op, Op::Add | Op::Sub | Op::BitNot) && expression_signedness(input)
        }
        Expression::Ternary(_, yes, no, _) => {
            expression_signedness(yes) && expression_signedness(no)
        }
        Expression::Concatenation(..) => false,
        Expression::Term(factor) => match factor.as_ref() {
            Factor::SystemFunctionCall(call) => {
                use veryl_analyzer::ir::SystemFunctionKind;
                match call.kind {
                    SystemFunctionKind::Bits(_)
                    | SystemFunctionKind::Size(..)
                    | SystemFunctionKind::Clog2(_)
                    | SystemFunctionKind::Signed(_) => true,
                    SystemFunctionKind::Unsigned(_) | SystemFunctionKind::Onehot(_) => false,
                    _ => expression.comptime().r#type.signed,
                }
            }
            Factor::Variable(_, _, select, ct) => variable_signedness(select, ct),
            Factor::Value(ct) => match &ct.value {
                ValueVariant::Numeric(value) => value.signed(),
                _ => ct.r#type.signed,
            },
            _ => expression.comptime().r#type.signed,
        },
        _ => expression.comptime().r#type.signed,
    }
}

pub(super) fn repair_expression_signedness(expression: &mut Expression, inherited: Option<bool>) {
    let signed = inherited.unwrap_or_else(|| expression_signedness(expression));
    expression.comptime_mut().expr_context.signed = signed;
    match expression {
        Expression::Binary(lhs, Op::As, _, _) => repair_expression_signedness(lhs, None),
        Expression::Binary(lhs, op, rhs, _) => {
            if matches!(op, Op::LogicAnd | Op::LogicOr) {
                repair_expression_signedness(lhs, None);
                repair_expression_signedness(rhs, None);
            } else if is_boolean_operator(*op) {
                let common = expression_signedness(lhs) && expression_signedness(rhs);
                repair_expression_signedness(lhs, Some(common));
                repair_expression_signedness(rhs, Some(common));
            } else {
                repair_expression_signedness(lhs, Some(signed));
                let shift = matches!(
                    op,
                    Op::LogicShiftL | Op::LogicShiftR | Op::ArithShiftL | Op::ArithShiftR | Op::Pow
                );
                repair_expression_signedness(rhs, if shift { None } else { Some(signed) });
            }
        }
        Expression::Unary(op, input, _) => repair_expression_signedness(
            input,
            if matches!(op, Op::Add | Op::Sub | Op::BitNot) {
                Some(signed)
            } else {
                None
            },
        ),
        Expression::Concatenation(parts, _) => {
            for (part, _) in parts {
                repair_expression_signedness(part, None);
            }
        }
        Expression::Ternary(cond, yes, no, _) => {
            repair_expression_signedness(cond, None);
            repair_expression_signedness(yes, Some(signed));
            repair_expression_signedness(no, Some(signed));
        }
        _ => {}
    }
}

// AIR flattens packed struct members to slices of the containing variable.
// A whole-member reference retains its declaration's sign; an explicit bit or
// part selection (including a full-width one) is unsigned for every base type.
pub(super) fn variable_signedness(select: &VarSelect, ct: &Comptime) -> bool {
    if select.is_empty() {
        return ct.r#type.signed;
    }
    ct.member_signed.unwrap_or(false)
}
