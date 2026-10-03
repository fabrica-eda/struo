//! Descending candidate expansion with a captured runtime initializer.
use super::{
    ForBound, ForRange, ForStatement, GUARDED_ITERATION_BUDGET, ImportError, LoopPlan,
    concrete_width, unsupported,
};

pub(super) fn plan(statement: &ForStatement) -> Result<LoopPlan, ImportError> {
    // Unsigned lower comparisons can turn negative counter values into large
    // positives after subtraction, so they cannot use this signed proof.
    let ForRange::Reverse {
        start: ForBound::Const(lower, true),
        ..
    } = &statement.range
    else {
        return Err(unsupported(
            "reverse loops require a constant signed lower bound",
        ));
    };
    if *lower == usize::try_from(i64::MAX).unwrap_or(usize::MAX) {
        return Err(unsupported(
            "reverse lower bound was saturated by the analyzer",
        ));
    }
    plan_with_lower(statement, *lower as i128)
}

pub(super) fn plan_with_lower(
    statement: &ForStatement,
    lower: i128,
) -> Result<LoopPlan, ImportError> {
    let ForRange::Reverse {
        start,
        end,
        inclusive,
        step,
    } = &statement.range
    else {
        unreachable!()
    };
    let width = concrete_width(&statement.var_type, "loop induction variable")?;
    if !statement.var_type.signed || width > usize::BITS {
        return Err(unsupported(
            "reverse loop counter must be signed and host-sized",
        ));
    }
    let counter_max = (1i128 << (width - 1)) - 1;
    let counter_min = -(1i128 << (width - 1));
    if *step == 0 || lower > counter_max || lower - (*step as i128) < counter_min {
        return Err(unsupported(
            "reverse loop step or lower bound can wrap the counter",
        ));
    }
    let ForBound::Expression(initializer) = end else {
        return Err(unsupported(
            "reverse loop initializer is not a runtime expression",
        ));
    };
    let maximum = super::bounds::nonnegative_maximum(initializer)?
        .filter(|maximum| (*maximum as i128) <= counter_max)
        .ok_or_else(|| {
            unsupported("reverse initializer is not proven non-negative without truncation")
        })?;
    let end_exclusive = maximum as i128 + i128::from(*inclusive);
    let count = (end_exclusive - lower).max(0);
    if count > GUARDED_ITERATION_BUDGET as i128 {
        return Err(unsupported(
            "reverse loop exceeds the 512-iteration synthesis budget",
        ));
    }
    let mask = usize::MAX >> (usize::BITS - width);
    Ok(LoopPlan {
        iterations: (lower..(lower + count))
            .rev()
            .map(|value| {
                usize::try_from(value & (mask as i128)).expect("masked counter fits usize")
            })
            .collect(),
        guard: Some((start.clone(), true)),
        runtime_start: Some((end.clone(), *step)),
    })
}
