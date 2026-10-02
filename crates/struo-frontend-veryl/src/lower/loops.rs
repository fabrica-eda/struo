//! Plans finite hardware expansion without using simulation input samples.
use veryl_analyzer::ir::{ForBound, ForRange, ForStatement, Op, Statement};

use super::{ImportError, concrete_width, context_width, evaluated_u64, substitute_statements};

/// Resource budget for newly supported runtime-bound loops, not a runtime cap.
const GUARDED_ITERATION_BUDGET: usize = 64;

pub(super) struct LoopPlan {
    pub iterations: Vec<usize>,
    pub guard: Option<(ForBound, bool)>,
}

pub(super) fn plan(statement: &ForStatement) -> Result<LoopPlan, ImportError> {
    let mut context = veryl_analyzer::Context::default();
    if let Some(iterations) = statement.range.eval_iter(&mut context) {
        return Ok(LoopPlan {
            iterations,
            guard: None,
        });
    }
    let (ForRange::Forward {
        start,
        end,
        inclusive,
        step,
    }
    | ForRange::Stepped {
        start,
        end,
        inclusive,
        step,
        op: Op::Add,
    }) = &statement.range
    else {
        return Err(unsupported(
            "reverse or non-additive runtime loops need a termination proof",
        ));
    };
    let start = start
        .eval_value(&mut context)
        .ok_or_else(|| unsupported("runtime loop start is not a non-negative constant"))?;
    if *step == 0 {
        return Err(unsupported("runtime loop step does not advance"));
    }
    let end_max = maximum_bound(end, statement.var_type.signed)?;
    let variable_width = concrete_width(&statement.var_type, "loop induction variable")?;
    let magnitude_bits = variable_width - u32::from(statement.var_type.signed);
    let variable_max = if magnitude_bits >= usize::BITS {
        usize::MAX
    } else {
        (1usize << magnitude_bits) - 1
    };
    if start > variable_max {
        return Err(unsupported(
            "runtime loop start does not fit the induction variable without truncation",
        ));
    }
    let mut iterations = Vec::new();
    let mut value = start;
    for _ in 0..GUARDED_ITERATION_BUDGET {
        if past_end(value, end_max, *inclusive) {
            return Ok(LoopPlan {
                iterations,
                guard: Some((end.clone(), *inclusive)),
            });
        }
        if value > variable_max {
            return Err(unsupported(
                "runtime loop induction could overflow before termination",
            ));
        }
        iterations.push(value);
        let mut body = statement.body.clone();
        substitute_statements(&mut body, statement.var_id, value)?;
        if always_breaks(&body) {
            return Ok(LoopPlan {
                iterations,
                guard: Some((end.clone(), *inclusive)),
            });
        }
        value = value
            .checked_add(*step)
            .filter(|next| *next <= variable_max)
            .ok_or_else(|| {
                unsupported("runtime loop induction could overflow before termination")
            })?;
    }
    if past_end(value, end_max, *inclusive) {
        return Ok(LoopPlan {
            iterations,
            guard: Some((end.clone(), *inclusive)),
        });
    }
    Err(unsupported(
        "runtime loop termination is not proven within the 64-iteration synthesis budget",
    ))
}

fn unsupported(message: &str) -> ImportError {
    ImportError::UnsupportedBehavior(message.into())
}

fn maximum_bound(bound: &ForBound, induction_signed: bool) -> Result<Option<usize>, ImportError> {
    if let Some(value) = bound.eval_value(&mut veryl_analyzer::Context::default()) {
        return Ok(Some(value));
    }
    let ForBound::Expression(expression) = bound else {
        unreachable!()
    };
    // A value-producing leaf keeps its declared result width even when the
    // for comparison widens it to int. Unsigned leaves zero-extend; signed
    // leaves compared with a signed induction cannot gain a larger positive value.
    let width = if matches!(expression.as_ref(), veryl_analyzer::ir::Expression::Term(_))
        && (!expression.comptime().r#type.signed || induction_signed)
    {
        concrete_width(&expression.comptime().r#type, "runtime loop bound")?
    } else {
        context_width(expression.comptime())?
    };
    // Treating signed operands as unsigned here is conservative: this is only
    // an upper-bound proof. The actual comparison retains operand signedness.
    Ok(if width >= usize::BITS {
        None
    } else {
        Some((1usize << width) - 1)
    })
}

fn past_end(value: usize, maximum: Option<usize>, inclusive: bool) -> bool {
    maximum.is_some_and(|maximum| {
        if inclusive {
            value > maximum
        } else {
            value >= maximum
        }
    })
}

fn always_breaks(statements: &[Statement]) -> bool {
    statements.iter().any(|statement| match statement {
        Statement::Break => true,
        Statement::If(branch) => match evaluated_u64(&branch.cond) {
            Some(0) => always_breaks(&branch.false_side),
            Some(_) => always_breaks(&branch.true_side),
            None => always_breaks(&branch.true_side) && always_breaks(&branch.false_side),
        },
        // A break in a nested loop does not terminate this loop. Case-based
        // termination and other proofs can be added independently later.
        _ => false,
    })
}
