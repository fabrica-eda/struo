//! Conservative expression ranges independent of mutable procedural values.
use super::{Expression, Factor, ImportError, Op, concrete_width, context_width, evaluated_u64};

pub(super) fn nonnegative_maximum(expression: &Expression) -> Result<Option<usize>, ImportError> {
    match expression {
        Expression::Term(factor) => {
            let width = concrete_width(&expression.comptime().r#type, "loop bound")?;
            if let Factor::Value(_) = factor.as_ref() {
                let Some(value) = evaluated_u64(expression) else {
                    return Ok(None);
                };
                if width > u64::BITS
                    || (expression.comptime().r#type.signed && value & (1u64 << (width - 1)) != 0)
                {
                    return Ok(None);
                }
                return Ok(usize::try_from(value).ok());
            }
            if super::super::types::expression_signedness(expression) || width >= usize::BITS {
                Ok(None)
            } else {
                Ok(Some((1usize << width) - 1))
            }
        }
        Expression::Binary(left, Op::Add, right, comptime) => {
            let (Some(left), Some(right)) =
                (nonnegative_maximum(left)?, nonnegative_maximum(right)?)
            else {
                return Ok(None);
            };
            let maximum = left.checked_add(right);
            let width = context_width(comptime)?
                - u32::from(super::super::types::expression_signedness(expression));
            Ok(maximum.filter(|maximum| width >= usize::BITS || *maximum < (1usize << width)))
        }
        _ => Ok(None),
    }
}
