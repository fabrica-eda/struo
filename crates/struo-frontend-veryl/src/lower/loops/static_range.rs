//! Check host-sized range enumeration against the emitted counter semantics.
use veryl_analyzer::ir::{ForBound, ForRange, ForStatement, Module};

use super::{ImportError, always_breaks, concrete_width, substitute_statements, unsupported};

pub(super) fn validate(
    statement: &ForStatement,
    source: &Module,
    iterations: &mut Vec<usize>,
) -> Result<(), ImportError> {
    let value = |bound: &ForBound| {
        bound
            .eval_value(&mut veryl_analyzer::Context::default())
            .map(|x| x as i128)
            .ok_or_else(|| unsupported("static loop bound is not constant"))
    };
    let width = concrete_width(&statement.var_type, "loop induction variable")?;
    let magnitude = width - u32::from(statement.var_type.signed);
    let maximum = if magnitude >= 127 {
        i128::MAX
    } else {
        (1i128 << magnitude) - 1
    };
    let minimum = if statement.var_type.signed {
        -maximum - 1
    } else {
        0
    };
    let (initial, reverse_unsigned, next) = match &statement.range {
        ForRange::Forward { start, step, .. } => (
            value(start)?,
            false,
            iterations.last().map(|last| *last as i128 + *step as i128),
        ),
        ForRange::Reverse {
            start,
            end,
            inclusive,
            step,
        } => {
            let signed = match start {
                ForBound::Const(_, signed) => *signed,
                ForBound::Expression(expression) => {
                    super::super::types::expression_signedness(expression)
                }
            };
            (
                value(end)? - i128::from(!inclusive),
                !signed || !statement.var_type.signed,
                iterations.last().map(|last| *last as i128 - *step as i128),
            )
        }
        ForRange::Stepped {
            start, step, op, ..
        } => (
            value(start)?,
            false,
            iterations
                .last()
                .map(|last| {
                    op.eval(*last, *step)
                        .map(|x| x as i128)
                        .ok_or_else(|| unsupported("static loop step overflows host range"))
                })
                .transpose()?,
        ),
    };
    let fits = |x| minimum <= x && x <= maximum;
    if !fits(initial) || iterations.iter().any(|value| !fits(*value as i128)) {
        return Err(unsupported(
            "static loop counter requires truncation or overflow",
        ));
    }
    // A negative counter converts to a large unsigned value in the comparison
    // (IEEE 1800-2023 11.8.1). An empty host range can therefore execute in SV.
    if reverse_unsigned && initial < 0 {
        return Err(unsupported(
            "static reverse loop has an unsigned negative sentinel",
        ));
    }
    if let ForRange::Reverse { start, .. } = &statement.range
        && initial < value(start)?
    {
        // The host enumerator saturates an exclusive zero upper bound when
        // step > 1; SV instead initializes the signed counter to -1.
        iterations.clear();
        return Ok(());
    }
    if iterations
        .first()
        .is_some_and(|first| *first as i128 != initial)
    {
        return Err(unsupported(
            "static loop enumeration disagrees with counter initialization",
        ));
    }
    if let (Some(last), Some(next)) = (iterations.last(), next) {
        let mut body = statement.body.clone();
        substitute_statements(&mut body, statement.var_id, *last)?;
        if !always_breaks(&body, source) && (!fits(next) || (reverse_unsigned && next < 0)) {
            return Err(unsupported(
                "static loop termination would require counter wrap or an unsigned negative sentinel",
            ));
        }
    }
    Ok(())
}
