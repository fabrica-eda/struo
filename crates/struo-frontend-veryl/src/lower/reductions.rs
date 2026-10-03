//! Replace proven modular additive reductions with a constant-size circuit.
use veryl_analyzer::ir::{AssignStatement, ForBound, ForRange, ForStatement};

use super::{
    DrivenBits, Env, Expression, Factor, ImportError, LoweredExpr, ModuleLowerer, Op, Statement,
    VarId, concrete_width, types,
};

impl ModuleLowerer<'_> {
    pub(super) fn lower_reduction(
        &mut self,
        statement: &ForStatement,
        writes: &mut Env,
    ) -> Result<Option<DrivenBits>, ImportError> {
        if let Some(changed) = self.lower_additive_reduction(statement, writes)? {
            Ok(Some(changed))
        } else if let Some(changed) = self.lower_idempotent_loop(statement, writes)? {
            Ok(Some(changed))
        } else if let Some(changed) = self.lower_small_state_loop(statement, writes)? {
            Ok(Some(changed))
        } else if let Some(changed) = self.lower_sparse_index_loop(statement, writes)? {
            Ok(Some(changed))
        } else if let Some(changed) = self.lower_periodic_reduction(statement, writes)? {
            Ok(Some(changed))
        } else {
            self.lower_linear_reduction(statement, writes)
        }
    }

    pub(super) fn lower_additive_reduction(
        &mut self,
        statement: &ForStatement,
        writes: &mut Env,
    ) -> Result<Option<DrivenBits>, ImportError> {
        let Some(Reduction {
            bound,
            assign,
            accumulator,
            increment,
            guard,
        }) = reduction(statement)
        else {
            return Ok(None);
        };
        let bound_id = whole_variable(bound).expect("validated reduction bound");
        let bound_type = &self.source.variables[&bound_id].r#type;
        let accumulator_type = &self.source.variables[&assign.dst[0].id].r#type;
        let counter_width = concrete_width(&statement.var_type, "reduction counter")?;
        let bound_width = concrete_width(bound_type, "reduction bound")?;
        if bound_type.signed
            || !bound_type.array.is_empty()
            || !accumulator_type.array.is_empty()
            || bound_width > counter_width
        {
            return Ok(None);
        }
        // IEEE 1800-2023 12.7.1 and 11.8.1: starting at zero, an unsigned
        // exclusive bound no wider than the counter is reached before wrap.
        // Without a break the body executes exactly bound times. An invariant
        // guard can adjust the number of accumulator updates below.
        let width = concrete_width(accumulator_type, "reduction accumulator")?;
        let initial = self.lower_expression(accumulator, writes)?;
        let mut increment = self.lower_expression(increment, writes)?;
        // Preserve the original addition's common type before extending its
        // increment, then compute in the accumulator's modular bit width.
        increment.signed = types::expression_signedness(&assign.expr);
        let increment = self.resize(increment, width, false)?;
        let count = self.lower_expression(bound, writes)?;
        let count = self.guard_reduction_count(count, guard, writes)?;
        let count = self.resize(count, width, false)?;
        let delta = self.lower_binary(Op::Mul, count, increment, width, false)?;
        let initial = self.resize(initial, width, false)?;
        let value = self.lower_binary(Op::Add, initial, delta, width, false)?;
        Ok(Some(self.assign_destinations_effects(
            &assign.dst,
            value,
            writes,
        )?))
    }

    fn guard_reduction_count(
        &mut self,
        count: LoweredExpr,
        guard: Option<(&Expression, Guard)>,
        writes: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let Some((condition, kind)) = guard else {
            return Ok(count);
        };
        let condition = self.lower_expression(condition, writes)?;
        let condition = self.boolean(condition)?;
        let (yes, no) = match kind {
            Guard::Enable => (count, self.constant(count.width, 0)),
            Guard::Break => {
                // Test the original bound before narrowing to the accumulator:
                // count=256 still executes once when an 8-bit result is used.
                let nonempty = self.boolean(count)?;
                (self.resize(nonempty, count.width, false)?, count)
            }
        };
        Ok(LoweredExpr {
            id: self.rtl.mux(condition.id, yes.id, no.id)?,
            ..count
        })
    }
}

#[derive(Clone, Copy)]
enum Guard {
    Enable,
    Break,
}

struct Reduction<'a> {
    bound: &'a Expression,
    assign: &'a AssignStatement,
    accumulator: &'a Expression,
    increment: &'a Expression,
    guard: Option<(&'a Expression, Guard)>,
}

fn reduction(statement: &ForStatement) -> Option<Reduction<'_>> {
    let ForRange::Forward {
        start: ForBound::Const(0, _),
        end: ForBound::Expression(bound),
        inclusive: false,
        step: 1,
    } = &statement.range
    else {
        return None;
    };
    let bound_id = whole_variable(bound)?;
    let (assign, guard) = reduction_body(&statement.body)?;
    let [destination] = assign.dst.as_slice() else {
        return None;
    };
    if assign.hier_dst.is_some()
        || !destination.index.0.is_empty()
        || !destination.select.is_empty()
        || destination.id == bound_id
        || destination.id == statement.var_id
    {
        return None;
    }
    if let Some((condition, _)) = guard {
        let condition_id = whole_variable(condition)?;
        if condition_id == destination.id || condition_id == statement.var_id {
            return None;
        }
    }
    let (accumulator, increment) = constant_addition(assign)?;
    Some(Reduction {
        bound,
        assign,
        accumulator,
        increment,
        guard,
    })
}

pub(super) fn constant_addition(assign: &AssignStatement) -> Option<(&Expression, &Expression)> {
    let [destination] = assign.dst.as_slice() else {
        return None;
    };
    let Expression::Binary(lhs, Op::Add, rhs, _) = &assign.expr else {
        return None;
    };
    let (accumulator, increment) = if whole_variable(lhs) == Some(destination.id) {
        (lhs.as_ref(), rhs.as_ref())
    } else if whole_variable(rhs) == Some(destination.id) {
        (rhs.as_ref(), lhs.as_ref())
    } else {
        return None;
    };
    if !matches!(increment, Expression::Term(factor) if matches!(factor.as_ref(), Factor::Value(_)))
    {
        return None;
    }
    Some((accumulator, increment))
}

fn reduction_body(body: &[Statement]) -> Option<(&AssignStatement, Option<(&Expression, Guard)>)> {
    match body {
        [Statement::Assign(assign)] => Some((assign, None)),
        [Statement::If(branch)] if branch.false_side.is_empty() => {
            let [Statement::Assign(assign)] = branch.true_side.as_slice() else {
                return None;
            };
            Some((assign, Some((&branch.cond, Guard::Enable))))
        }
        [Statement::Assign(assign), Statement::If(branch)]
            if branch.false_side.is_empty()
                && matches!(branch.true_side.as_slice(), [Statement::Break]) =>
        {
            Some((assign, Some((&branch.cond, Guard::Break))))
        }
        _ => None,
    }
}

pub(super) fn whole_variable(expression: &Expression) -> Option<VarId> {
    let Expression::Term(factor) = expression else {
        return None;
    };
    match factor.as_ref() {
        Factor::Variable(id, index, select, _) if index.0.is_empty() && select.is_empty() => {
            Some(*id)
        }
        _ => None,
    }
}
