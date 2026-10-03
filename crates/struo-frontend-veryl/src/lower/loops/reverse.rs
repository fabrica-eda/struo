//! Descending candidate expansion with a captured runtime initializer.
use super::{
    ForBound, ForRange, ForStatement, GUARDED_ITERATION_BUDGET, ImportError, LoopPlan,
    concrete_width, unsupported,
};

pub(super) fn plan(statement: &ForStatement) -> Result<LoopPlan, ImportError> {
    let ForRange::Reverse {
        start,
        end,
        inclusive,
        step,
    } = &statement.range
    else {
        unreachable!()
    };
    // An unsigned lower bound makes negative counter values compare as large
    // positives, so wrapping through zero cannot establish termination.
    let ForBound::Const(lower, true) = start else {
        return Err(unsupported(
            "reverse loops require a constant signed lower bound",
        ));
    };
    let width = concrete_width(&statement.var_type, "loop induction variable")?;
    if !statement.var_type.signed || width > usize::BITS {
        return Err(unsupported(
            "reverse loop counter must be signed and host-sized",
        ));
    }
    let counter_max = (1usize << (width - 1)) - 1;
    if *step == 0
        || *step > counter_max
        || *lower > counter_max
        || *lower == usize::try_from(i64::MAX).unwrap_or(usize::MAX)
    {
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
        .filter(|maximum| *maximum <= counter_max)
        .ok_or_else(|| {
            unsupported("reverse initializer is not proven non-negative without truncation")
        })?;
    let count = maximum
        .checked_add(usize::from(*inclusive))
        .and_then(|end| end.checked_sub(*lower))
        .unwrap_or(0);
    if count > GUARDED_ITERATION_BUDGET {
        return Err(unsupported(
            "reverse loop exceeds the 512-iteration synthesis budget",
        ));
    }
    Ok(LoopPlan {
        iterations: (*lower..(*lower + count)).rev().collect(),
        guard: Some((start.clone(), true)),
        runtime_start: Some((end.clone(), *step)),
    })
}
