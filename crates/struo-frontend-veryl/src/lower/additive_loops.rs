//! Prove short typed counter traces against a constant additive-loop bound.
use std::cmp::Ordering;

use veryl_analyzer::ir::{ForBound, ForRange, ForStatement};

use super::{
    Constant, Env, HashSet, ImportError, ModuleLowerer, Op, concrete_width, loops,
    substitute_statements,
};

impl ModuleLowerer<'_> {
    pub(super) fn plan_known_additive_loop(
        &mut self,
        statement: &ForStatement,
        reads: &Env,
        writes: &Env,
        sequential: bool,
    ) -> Result<Option<loops::LoopPlan>, ImportError> {
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
            return Ok(None);
        };
        let width = concrete_width(&statement.var_type, "loop induction variable")?;
        if width > usize::BITS || *step == usize::MAX {
            return Ok(None);
        }
        // The condition must stay constant throughout the body. A variable
        // currently holding a constant can be changed by the loop itself.
        let limit = match end {
            ForBound::Const(value, signed) => {
                let saturation = if *signed {
                    usize::try_from(i64::MAX).unwrap_or(usize::MAX)
                } else {
                    usize::MAX
                };
                if *value == saturation {
                    return Ok(None);
                }
                super::LoweredExpr {
                    signed: *signed,
                    ..self.constant(64, *value as u64)
                }
            }
            ForBound::Expression(expression) if expression.comptime().is_const => {
                self.lower_expression(expression, writes)?
            }
            ForBound::Expression(_) => return Ok(None),
        };
        let Some(bound) = self.known_rtl_constant(limit.id) else {
            return Ok(None);
        };
        let Some(initial) = self.known_loop_initializer(start, reads, writes, sequential, width)?
        else {
            return Ok(None);
        };
        let mask = usize::MAX >> (usize::BITS - width);
        let mut counter = initial & mask;
        let mut seen = HashSet::new();
        let mut iterations = Vec::new();
        loop {
            let order = compare_counter(
                counter,
                width,
                statement.var_type.signed,
                &bound,
                limit.signed,
            );
            if order == Ordering::Greater || (!inclusive && order == Ordering::Equal) {
                return Ok(Some(loops::LoopPlan {
                    iterations,
                    guard: Some((end.clone(), *inclusive)),
                    runtime_start: None,
                }));
            }
            if iterations.len() == loops::GUARDED_ITERATION_BUDGET || !seen.insert(counter) {
                return Ok(None);
            }
            iterations.push(counter);
            let mut body = statement.body.clone();
            substitute_statements(&mut body, statement.var_id, counter)?;
            if loops::always_breaks(&body, self.source) {
                return Ok(Some(loops::LoopPlan {
                    iterations,
                    guard: Some((end.clone(), *inclusive)),
                    runtime_start: None,
                }));
            }
            counter = counter.wrapping_add(*step) & mask;
        }
    }
}

// Common signedness is determined before extending comparison operands
// (IEEE 1800-2023 11.8.1/11.8.2), including negative counter bit patterns.
fn compare_counter(
    counter: usize,
    width: u32,
    signed: bool,
    bound: &Constant,
    bound_signed: bool,
) -> Ordering {
    let common_signed = signed && bound_signed;
    let counter_negative = common_signed && (counter & (1usize << (width - 1))) != 0;
    let bound_negative = common_signed && bound.bit(bound.width().get() - 1);
    if counter_negative != bound_negative {
        return if counter_negative {
            Ordering::Less
        } else {
            Ordering::Greater
        };
    }
    for bit in (0..width.max(bound.width().get())).rev() {
        let left = if bit < width {
            counter & (1usize << bit) != 0
        } else {
            counter_negative
        };
        let right = if bit < bound.width().get() {
            bound.bit(bit)
        } else {
            bound_negative
        };
        match left.cmp(&right) {
            Ordering::Equal => (),
            order => return order,
        }
    }
    Ordering::Equal
}
