//! Expand only iterations that can write an in-range packed bit.
use std::collections::BTreeMap;

use veryl_analyzer::ir::{ForStatement, Module};

use super::{
    DrivenBits, Env, Expression, Factor, ImportError, LoweredExpr, ModuleLowerer, Op, Statement,
    VarId, concrete_width, context_width, evaluated_u64, loops, substitute_statements, types,
};

type Events = BTreeMap<usize, Vec<usize>>;

impl ModuleLowerer<'_> {
    pub(super) fn lower_sparse_index_loop(
        &mut self,
        statement: &ForStatement,
        writes: &mut Env,
    ) -> Result<Option<DrivenBits>, ImportError> {
        let Some(bound) = loops::unsigned_unit_range(statement, self.source) else {
            return Ok(None);
        };
        let Some(events) = write_events(statement, self.source, bound) else {
            return Ok(None);
        };
        let count = self.lower_expression(bound, writes)?;
        let width = concrete_width(&statement.var_type, "sparse loop counter")?;
        let mut changed = DrivenBits::default();
        for (iteration, assignments) in events {
            let index = LoweredExpr {
                signed: statement.var_type.signed,
                ..self.constant(width, iteration as u64)
            };
            let skip = self.lower_binary(Op::GreaterEq, index, count, 1, false)?;
            let before = writes.clone();
            let mut body = assignments
                .into_iter()
                .map(|index| statement.body[index].clone())
                .collect::<Vec<_>>();
            substitute_statements(&mut body, statement.var_id, iteration)?;
            for assignment in body {
                let Statement::Assign(assignment) = assignment else {
                    unreachable!()
                };
                changed.extend(self.lower_assignment(&assignment, &before, writes, false)?);
            }
            *writes = self.merge_values(skip, &before, writes)?;
        }
        Ok(Some(changed))
    }
}

fn write_events(statement: &ForStatement, source: &Module, bound: &Expression) -> Option<Events> {
    let Expression::Term(factor) = bound else {
        return None;
    };
    let Factor::Variable(bound_id, ..) = factor.as_ref() else {
        return None;
    };
    let width = concrete_width(&statement.var_type, "sparse loop counter").ok()?;
    let bound_width = concrete_width(&source.variables.get(bound_id)?.r#type, "bound").ok()?;
    if !(1..=u64::BITS).contains(&width) || bound_width == 0 || statement.body.len() > 32 {
        return None;
    }
    let mask = u64::MAX >> (u64::BITS - width);
    // The largest possible exclusive bound itself is never an iteration.
    let maximum = u64::MAX >> (u64::BITS - bound_width);
    let mut events = Events::new();
    for (position, body) in statement.body.iter().enumerate() {
        let Statement::Assign(assign) = body else {
            return None;
        };
        let [destination] = assign.dst.as_slice() else {
            return None;
        };
        let [index] = destination.select.0.as_slice() else {
            return None;
        };
        let ty = &source.variables.get(&destination.id)?.r#type;
        let total = concrete_width(ty, "sparse loop destination").ok()?;
        if assign.hier_dst.is_some()
            || destination.id == statement.var_id
            || destination.id == *bound_id
            || !destination.index.0.is_empty()
            || destination.select.1.is_some()
            || destination.comptime.member_select_domain.is_some()
            || !ty.array.is_empty()
            || total as usize > loops::GUARDED_ITERATION_BUDGET
            || !pure_expression(&assign.expr)
        {
            return None;
        }
        let (reverse, offset) = affine_index(index, statement.var_id, width, mask)?;
        let signed = types::expression_signedness(index);
        for bit in 0..u64::from(total) {
            if bit > mask || (signed && bit & (1 << (width - 1)) != 0) {
                continue;
            }
            // Solve +/-i + offset == bit modulo the actual index width. This
            // includes late writes after wrap, rather than assuming a prefix.
            let iteration = if reverse {
                offset.wrapping_sub(bit)
            } else {
                bit.wrapping_sub(offset)
            } & mask;
            if iteration < maximum {
                events
                    .entry(usize::try_from(iteration).ok()?)
                    .or_default()
                    .push(position);
                if events.len() > loops::GUARDED_ITERATION_BUDGET {
                    return None;
                }
            }
        }
    }
    Some(events)
}

// Each node keeps exactly the counter width. Narrow casts and widened
// arithmetic need a different inverse proof and are deliberately excluded.
fn affine_index(expr: &Expression, counter: VarId, width: u32, mask: u64) -> Option<(bool, u64)> {
    if concrete_width(&expr.comptime().r#type, "sparse index").ok()? != width {
        return None;
    }
    match expr {
        Expression::Term(factor) => match factor.as_ref() {
            Factor::Variable(id, index, select, ct)
                if *id == counter
                    && index.0.is_empty()
                    && select.is_empty()
                    && ct.member_select_domain.is_none() =>
            {
                Some((false, 0))
            }
            _ => None,
        },
        Expression::Binary(lhs, op @ (Op::Add | Op::Sub), rhs, ct)
            if context_width(ct).ok()? == width =>
        {
            if let Some(constant) = literal(rhs, width) {
                let (reverse, offset) = affine_index(lhs, counter, width, mask)?;
                let offset = if *op == Op::Add {
                    offset.wrapping_add(constant)
                } else {
                    offset.wrapping_sub(constant)
                };
                Some((reverse, offset & mask))
            } else {
                let constant = literal(lhs, width)?;
                let (reverse, offset) = affine_index(rhs, counter, width, mask)?;
                Some(if *op == Op::Add {
                    (reverse, constant.wrapping_add(offset) & mask)
                } else {
                    (!reverse, constant.wrapping_sub(offset) & mask)
                })
            }
        }
        _ => None,
    }
}

fn literal(expr: &Expression, width: u32) -> Option<u64> {
    if matches!(expr, Expression::Term(factor) if matches!(factor.as_ref(), Factor::Value(_)))
        && concrete_width(&expr.comptime().r#type, "index offset").ok()? == width
    {
        evaluated_u64(expr)
    } else {
        None
    }
}

fn pure_expression(expr: &Expression) -> bool {
    match expr {
        Expression::Term(factor) => match factor.as_ref() {
            Factor::Value(_) => true,
            Factor::Variable(_, index, select, _) => {
                index.0.iter().chain(&select.0).all(pure_expression)
                    && select
                        .1
                        .as_ref()
                        .is_none_or(|(_, expr)| pure_expression(expr))
            }
            _ => false,
        },
        Expression::Unary(_, value, _) => pure_expression(value),
        Expression::Binary(lhs, _, rhs, _) => pure_expression(lhs) && pure_expression(rhs),
        Expression::Ternary(cond, yes, no, _) => {
            pure_expression(cond) && pure_expression(yes) && pure_expression(no)
        }
        Expression::Concatenation(parts, _) => parts.iter().all(|(value, repeat)| {
            pure_expression(value) && repeat.as_ref().is_none_or(pure_expression)
        }),
        _ => false,
    }
}
