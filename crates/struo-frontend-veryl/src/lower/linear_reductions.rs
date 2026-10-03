//! Closed forms for independent scalar updates in proven signed forward ranges.
use std::collections::HashSet;

use veryl_analyzer::ir::{ForBound, ForRange, ForStatement, Module};

use super::{
    BinaryOp, DrivenBits, Env, Expression, Factor, ImportError, LoweredExpr, ModuleLowerer, Op,
    SignalKey, Statement, VarId, concrete_width,
    reductions::{constant_addition, whole_variable},
    types,
};

struct LinearRange<'a> {
    start: &'a Expression,
    end: u64,
    step: u64,
    width: u32,
}

impl ModuleLowerer<'_> {
    pub(super) fn lower_linear_reduction(
        &mut self,
        statement: &ForStatement,
        writes: &mut Env,
    ) -> Result<Option<DrivenBits>, ImportError> {
        let Some(range) = linear_range(statement, self.source) else {
            return Ok(None);
        };
        if !independent_updates(statement, self.source) {
            return Ok(None);
        }
        let before = writes.clone();
        let initial = self.lower_expression(range.start, &before)?;
        let initial = self.resize(initial, range.width, true)?;
        let sign = 1u64 << (range.width - 1);
        let sign_mask = self.constant(range.width, sign);
        let ordinal = LoweredExpr {
            id: self.rtl.binary(BinaryOp::Xor, initial.id, sign_mask.id)?,
            signed: false,
            ..initial
        };
        let end = self.constant(range.width, range.end ^ sign);
        let empty = self.lower_binary(Op::GreaterEq, ordinal, end, 1, false)?;
        let distance = self.lower_binary(Op::Sub, end, ordinal, range.width, false)?;
        let one = self.constant(range.width, 1);
        let step = self.constant(range.width, range.step);
        // ceil(distance / step), avoiding overflow in distance + step - 1.
        let raw_count = if range.step == 1 {
            distance
        } else {
            let adjusted = self.lower_binary(Op::Sub, distance, one, range.width, false)?;
            let quotient = self.lower_binary(Op::Div, adjusted, step, range.width, false)?;
            self.lower_binary(Op::Add, quotient, one, range.width, false)?
        };
        let zero = self.constant(range.width, 0);
        let count = LoweredExpr {
            id: self.rtl.mux(empty.id, zero.id, raw_count.id)?,
            ..raw_count
        };
        let preceding = self.lower_binary(Op::Sub, count, one, range.width, false)?;
        let offset = self.lower_binary(Op::Mul, preceding, step, range.width, false)?;
        let last = self.lower_binary(Op::Add, initial, offset, range.width, true)?;
        let mut final_reads = before.clone();
        final_reads.insert(
            SignalKey {
                id: statement.var_id,
                index: Vec::new(),
            },
            last,
        );
        let mut changed = DrivenBits::default();
        for body in &statement.body {
            let Statement::Assign(assign) = body else {
                unreachable!()
            };
            let value = if let Some((accumulator, increment)) = constant_addition(assign) {
                let width = concrete_width(
                    &self.source.variables[&assign.dst[0].id].r#type,
                    "linear accumulator",
                )?;
                let initial = self.lower_expression(accumulator, &before)?;
                let mut increment = self.lower_expression(increment, &before)?;
                increment.signed = types::expression_signedness(&assign.expr);
                let increment = self.resize(increment, width, false)?;
                let count = self.resize(count, width, false)?;
                let delta = self.lower_binary(Op::Mul, count, increment, width, false)?;
                let initial = self.resize(initial, width, false)?;
                self.lower_binary(Op::Add, initial, delta, width, false)?
            } else {
                // AIR classifies induction variables as constants. This use
                // reads the computed runtime value, as in single-iteration lowering.
                let inserted = self.runtime_loop_variables.insert(statement.var_id);
                let value = self.lower_expression(&assign.expr, &final_reads);
                if inserted {
                    self.runtime_loop_variables.remove(&statement.var_id);
                }
                value?
            };
            changed.extend(self.assign_destinations_effects(&assign.dst, value, writes)?);
        }
        *writes = self.merge_values(empty, &before, writes)?;
        Ok(Some(changed))
    }
}

fn linear_range<'a>(statement: &'a ForStatement, source: &Module) -> Option<LinearRange<'a>> {
    let (ForRange::Forward {
        start: ForBound::Expression(start),
        end: ForBound::Const(end, true),
        inclusive: false,
        step,
    }
    | ForRange::Stepped {
        start: ForBound::Expression(start),
        end: ForBound::Const(end, true),
        inclusive: false,
        step,
        op: Op::Add,
    }) = &statement.range
    else {
        return None;
    };
    let width = concrete_width(&statement.var_type, "linear counter").ok()?;
    if !statement.var_type.signed || !(2..=64).contains(&width) {
        return None;
    }
    let maximum = u64::MAX >> (65 - width);
    let end = *end as u64;
    let step = *step as u64;
    // Every active value is below end. The exit step must also fit the signed
    // counter, including starts whose assignment conversion makes them negative.
    if end > maximum
        || step == 0
        || step > maximum
        || end == i64::MAX.unsigned_abs()
        || step == i64::MAX.unsigned_abs()
        || end.checked_add(step - 1)? > maximum
    {
        return None;
    }
    let start_id = whole_variable(start)?;
    if !source.variables.get(&start_id)?.r#type.array.is_empty() {
        return None;
    }
    Some(LinearRange {
        start,
        end,
        step,
        width,
    })
}

fn independent_updates(statement: &ForStatement, source: &Module) -> bool {
    if statement.body.len() > 32 {
        return false;
    }
    let mut destinations = HashSet::new();
    statement.body.iter().all(|body| {
        let Statement::Assign(assign) = body else {
            return false;
        };
        let [destination] = assign.dst.as_slice() else {
            return false;
        };
        assign.hier_dst.is_none()
            && destination.index.0.is_empty()
            && destination.select.is_empty()
            && destination.comptime.member_select_domain.is_none()
            && destination.id != statement.var_id
            && source.variables[&destination.id].r#type.array.is_empty()
            && destinations.insert(destination.id)
            && (constant_addition(assign).is_some() || final_value(&assign.expr, statement.var_id))
    })
}

fn final_value(expression: &Expression, counter: VarId) -> bool {
    match expression {
        Expression::Term(factor) => {
            matches!(factor.as_ref(), Factor::Value(_))
                || whole_variable(expression) == Some(counter)
        }
        Expression::Binary(lhs, Op::As, rhs, _) => {
            final_value(lhs, counter)
                && matches!(rhs.as_ref(), Expression::Term(factor) if matches!(factor.as_ref(), Factor::Value(_)))
        }
        _ => false,
    }
}
