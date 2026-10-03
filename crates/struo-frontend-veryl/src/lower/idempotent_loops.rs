//! Prove that a straight-line loop body forgets every bit it overwrites.
use std::collections::HashMap;

use veryl_analyzer::ir::{AssignStatement, ForBound, ForRange, ForStatement, Module};

use super::{
    DrivenBits, Env, Expression, Factor, ImportError, ModuleLowerer, Op, Statement, VarId,
    VarSelect, concrete_width, static_select,
};

type Dependencies = HashMap<VarId, Vec<bool>>;

impl ModuleLowerer<'_> {
    pub(super) fn lower_idempotent_loop(
        &mut self,
        statement: &ForStatement,
        writes: &mut Env,
    ) -> Result<Option<DrivenBits>, ImportError> {
        let Some(bound) = idempotent_bound(statement, self.source) else {
            return Ok(None);
        };
        let Expression::Term(factor) = bound else {
            unreachable!()
        };
        let Factor::Variable(bound_id, ..) = factor.as_ref() else {
            unreachable!()
        };
        if !proves_idempotence(statement, self.source, *bound_id) {
            return Ok(None);
        }
        let count = self.lower_expression(bound, writes)?;
        let zero = self.constant(count.width, 0);
        let empty = self.lower_binary(Op::Eq, count, zero, 1, false)?;
        let before = writes.clone();
        let mut changed = DrivenBits::default();
        for statement in &statement.body {
            let Statement::Assign(assign) = statement else {
                unreachable!()
            };
            changed.extend(self.lower_assignment(assign, &before, writes, false)?);
        }
        *writes = self.merge_values(empty, &before, writes)?;
        Ok(Some(changed))
    }
}

fn idempotent_bound<'a>(statement: &'a ForStatement, source: &Module) -> Option<&'a Expression> {
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
        || concrete_width(ty, "idempotent bound").ok()?
            > concrete_width(&statement.var_type, "idempotent counter").ok()?
    {
        return None;
    }
    Some(bound)
}

fn destination(assign: &AssignStatement, source: &Module) -> Option<(VarId, usize, usize, usize)> {
    let [dst] = assign.dst.as_slice() else {
        return None;
    };
    let ty = &source.variables.get(&dst.id)?.r#type;
    if assign.hier_dst.is_some()
        || !dst.index.0.is_empty()
        || !ty.array.is_empty()
        || dst.comptime.member_select_domain.is_some()
        || !literal_select(&dst.select)
    {
        return None;
    }
    let total = concrete_width(ty, "idempotent destination").ok()?;
    // Bound the analysis itself; this is not a semantic loop iteration cap.
    if total > 4096 {
        return None;
    }
    let (lsb, width) = static_select(&dst.select, total).ok()?;
    Some((dst.id, lsb as usize, width as usize, total as usize))
}

fn proves_idempotence(statement: &ForStatement, source: &Module, bound: VarId) -> bool {
    let mut dependencies = Dependencies::new();
    for body in &statement.body {
        let Statement::Assign(assign) = body else {
            return false;
        };
        let Some((id, lsb, width, total)) = destination(assign, source) else {
            return false;
        };
        if id == bound || id == statement.var_id {
            return false;
        }
        dependencies.entry(id).or_insert_with(|| vec![false; total])[lsb..lsb + width].fill(true);
    }
    // True marks dependency on any pre-iteration bit written by the body.
    // Straight-line assignment order propagates these dependencies, including
    // reads after earlier writes. False on all final written bits proves F(F(x)) = F(x).
    for body in &statement.body {
        let Statement::Assign(assign) = body else {
            unreachable!()
        };
        let Some(depends) = expression_dependency(&assign.expr, &dependencies, statement.var_id)
        else {
            return false;
        };
        let (id, lsb, width, _) = destination(assign, source).expect("validated destination");
        dependencies.get_mut(&id).expect("written variable")[lsb..lsb + width].fill(depends);
    }
    dependencies
        .values()
        .all(|bits| !bits.iter().any(|bit| *bit))
}

fn expression_dependency(expr: &Expression, deps: &Dependencies, counter: VarId) -> Option<bool> {
    let check = |expr| expression_dependency(expr, deps, counter);
    match expr {
        Expression::Term(factor) => match factor.as_ref() {
            Factor::Value(_) => Some(false),
            Factor::Variable(id, index, select, ct) => {
                if *id == counter || !index.0.is_empty() || ct.member_select_domain.is_some() {
                    return None;
                }
                let mut dependent = false;
                for expression in &select.0 {
                    dependent |= check(expression)?;
                }
                if let Some((_, expression)) = &select.1 {
                    dependent |= check(expression)?;
                }
                if let Some(bits) = deps.get(id) {
                    let selected = literal_select(select)
                        .then_some(bits.len())
                        .and_then(|width| u32::try_from(width).ok())
                        .and_then(|width| static_select(select, width).ok());
                    let bits = selected.map_or(bits.as_slice(), |(lsb, width)| {
                        &bits[lsb as usize..(lsb + width) as usize]
                    });
                    dependent |= bits.iter().any(|bit| *bit);
                }
                Some(dependent)
            }
            _ => None,
        },
        Expression::Unary(_, value, _) => check(value),
        Expression::Binary(lhs, _, rhs, _) => Some(check(lhs)? | check(rhs)?),
        Expression::Ternary(cond, yes, no, _) => Some(check(cond)? | check(yes)? | check(no)?),
        Expression::Concatenation(parts, _) => {
            let mut dependent = false;
            for (value, repeat) in parts {
                dependent |= check(value)?;
                if let Some(repeat) = repeat {
                    dependent |= check(repeat)?;
                }
            }
            Some(dependent)
        }
        _ => None,
    }
}

fn literal_select(select: &VarSelect) -> bool {
    select
        .0
        .iter()
        .chain(select.1.iter().map(|(_, expr)| expr))
        .all(literal_expression)
}

fn literal_expression(expr: &Expression) -> bool {
    match expr {
        Expression::Term(factor) => matches!(factor.as_ref(), Factor::Value(_)),
        Expression::Unary(_, value, _) => literal_expression(value),
        Expression::Binary(lhs, _, rhs, _) => literal_expression(lhs) && literal_expression(rhs),
        Expression::Ternary(cond, yes, no, _) => {
            literal_expression(cond) && literal_expression(yes) && literal_expression(no)
        }
        _ => false,
    }
}
