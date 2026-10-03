//! Plans finite hardware expansion without using simulation input samples.
mod bounds;
mod reverse;
mod static_range;
use veryl_analyzer::ir::{
    Expression, Factor, ForBound, ForRange, ForStatement, Module, Op, Statement,
};

use super::{ImportError, concrete_width, context_width, evaluated_u64, substitute_statements};

/// Resource budget for newly supported runtime-bound loops, not a runtime cap.
pub(super) const GUARDED_ITERATION_BUDGET: usize = 512;

pub(super) struct LoopPlan {
    pub iterations: Vec<usize>,
    pub guard: Option<(ForBound, bool)>,
    pub runtime_start: Option<(ForBound, usize)>,
}

pub(super) fn plan_reverse_with_lower(
    statement: &ForStatement,
    lower: i128,
) -> Result<LoopPlan, ImportError> {
    reverse::plan_with_lower(statement, lower)
}

pub(super) fn plan(statement: &ForStatement, source: &Module) -> Result<LoopPlan, ImportError> {
    let mut context = veryl_analyzer::Context::default();
    if let Some(mut iterations) = statement.range.eval_iter(&mut context) {
        static_range::validate(statement, source, &mut iterations)?;
        return Ok(LoopPlan {
            iterations,
            guard: None,
            runtime_start: None,
        });
    }
    if matches!(statement.range, ForRange::Reverse { .. }) {
        return reverse::plan(statement);
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
    let Some(start_value) = start.eval_value(&mut context) else {
        return plan_runtime_start(statement, start, end, *inclusive, *step);
    };
    let start = start_value;
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
                runtime_start: None,
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
        if always_breaks(&body, source) {
            return Ok(LoopPlan {
                iterations,
                guard: Some((end.clone(), *inclusive)),
                runtime_start: None,
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
            runtime_start: None,
        });
    }
    Err(unsupported(
        "runtime loop termination is not proven within the 512-iteration synthesis budget",
    ))
}

// Enumerate every possible non-negative counter value, not sampled input
// values. Lowering tracks which candidates the initialized counter reaches.
fn plan_runtime_start(
    statement: &ForStatement,
    start: &ForBound,
    end: &ForBound,
    inclusive: bool,
    step: usize,
) -> Result<LoopPlan, ImportError> {
    let ForBound::Expression(expression) = start else {
        return Err(unsupported(
            "runtime loop start is not a non-negative constant",
        ));
    };
    let width = concrete_width(&statement.var_type, "loop induction variable")?;
    let magnitude = width - u32::from(statement.var_type.signed);
    // A narrow unsigned leaf zero-extends into the counter. Do not infer a
    // range from a context-sized arithmetic expression or a truncated value.
    if !matches!(expression.as_ref(), Expression::Term(_))
        || super::types::expression_signedness(expression)
        || concrete_width(&expression.comptime().r#type, "loop start")? > magnitude
    {
        return Err(unsupported(
            "runtime loop start is not proven non-negative without truncation",
        ));
    }
    if step == 0 {
        return Err(unsupported("runtime loop step does not advance"));
    }
    let count = maximum_bound(end, statement.var_type.signed)?
        .and_then(|maximum| maximum.checked_add(usize::from(inclusive)))
        .filter(|count| *count <= GUARDED_ITERATION_BUDGET)
        .ok_or_else(|| {
            unsupported(
                "runtime loop termination is not proven within the 512-iteration synthesis budget",
            )
        })?;
    let counter_max = if magnitude >= usize::BITS {
        usize::MAX
    } else {
        (1usize << magnitude) - 1
    };
    if count != 0
        && count
            .checked_sub(1)
            .and_then(|last| last.checked_add(step))
            .is_none_or(|next| next > counter_max)
    {
        return Err(unsupported(
            "runtime loop induction could overflow before termination",
        ));
    }
    Ok(LoopPlan {
        iterations: (0..count).collect(),
        guard: Some((end.clone(), inclusive)),
        runtime_start: Some((start.clone(), step)),
    })
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
    if let Some(maximum) = bounds::nonnegative_maximum(expression)? {
        return Ok(Some(maximum));
    }
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

pub(super) fn always_breaks(statements: &[Statement], source: &Module) -> bool {
    statements.iter().any(|statement| match statement {
        Statement::Break => true,
        Statement::If(branch) => match known_condition(&branch.cond, source) {
            Some(0) => always_breaks(&branch.false_side, source),
            Some(_) => always_breaks(&branch.true_side, source),
            None => {
                always_breaks(&branch.true_side, source)
                    && always_breaks(&branch.false_side, source)
            }
        },
        Statement::Case(case) => {
            always_breaks(&case.default, source)
                && case.arms.iter().all(|arm| always_breaks(&arm.body, source))
        }
        // A break in a nested loop does not terminate this loop. Require a
        // terminating default above instead of assuming case patterns exhaustive.
        _ => false,
    })
}

fn known_condition(expression: &Expression, source: &Module) -> Option<u64> {
    if let Some(value) = evaluated_u64(expression) {
        return Some(value);
    }
    // An effectful function can still have a fixed return value. Prove that
    // from its body rather than trusting the AIR numeric cache of a non-const
    // expression (which can contain a representative array element).
    let Expression::Term(factor) = expression else {
        return None;
    };
    let Factor::FunctionCall(call) = factor.as_ref() else {
        return None;
    };
    let body = source
        .functions
        .get(&call.id)?
        .get_function(call.index.as_deref().unwrap_or(&[]))?;
    let ret = body.ret?;
    let (last, prefix) = body.statements.split_last()?;
    let Statement::Assign(assign) = last else {
        return None;
    };
    if super::contains_destination(prefix, ret) || assign.dst.len() != 1 {
        return None;
    }
    let destination = &assign.dst[0];
    if destination.id != ret || !destination.index.0.is_empty() || !destination.select.is_empty() {
        return None;
    }
    let value = evaluated_u64(&assign.expr)?;
    let width = source.variables.get(&ret)?.r#type.total_width()?;
    Some(if width < 64 {
        value & ((1u64 << width) - 1)
    } else {
        value
    })
}

pub(super) fn unsigned_unit_range<'a>(
    statement: &'a ForStatement,
    source: &Module,
) -> Option<&'a Expression> {
    let ForRange::Forward {
        start: ForBound::Const(0, _),
        end: ForBound::Expression(bound),
        inclusive: false,
        step: 1,
    } = &statement.range
    else {
        return None;
    };
    let Expression::Term(factor) = bound.as_ref() else {
        return None;
    };
    let Factor::Variable(id, index, select, _) = factor.as_ref() else {
        return None;
    };
    let ty = &source.variables.get(id)?.r#type;
    if !index.0.is_empty()
        || !select.is_empty()
        || ty.signed
        || !ty.array.is_empty()
        || concrete_width(ty, "unsigned loop bound").ok()?
            > concrete_width(&statement.var_type, "unit-step loop counter").ok()?
    {
        return None;
    }
    Some(bound)
}
