//! Compose fully enumerated small-state transitions using binary powers.
use veryl_analyzer::ir::{AssignStatement, ForStatement};

use super::{
    BitWidth, DrivenBits, Env, Expression, Factor, ImportError, LoweredExpr, ModuleLowerer,
    SignalKey, Statement, VarId, idempotent_loops::literal_select, loops, static_select,
};

impl ModuleLowerer<'_> {
    pub(super) fn lower_small_state_loop(
        &mut self,
        statement: &ForStatement,
        writes: &mut Env,
    ) -> Result<Option<DrivenBits>, ImportError> {
        let Some(bound) = loops::unsigned_unit_range(statement, self.source) else {
            return Ok(None);
        };
        let [Statement::Assign(first), ..] = statement.body.as_slice() else {
            return Ok(None);
        };
        let [destination] = first.dst.as_slice() else {
            return Ok(None);
        };
        let key = SignalKey {
            id: destination.id,
            index: Vec::new(),
        };
        let Some(mut current) = writes.get(&key).copied() else {
            return Ok(None);
        };
        // This bounds analysis and circuit size, not the runtime trip count.
        if current.width > 4
            || statement.body.len() > 32
            || key.id == statement.var_id
            || !self.source.variables[&key.id].r#type.array.is_empty()
            || !statement
                .body
                .iter()
                .all(|body| valid_assignment(body, key.id, current.width))
            || references_state(bound, key.id)
        {
            return Ok(None);
        }
        let count = self.lower_expression(bound, writes)?;
        if count.width > 64 {
            return Ok(None);
        }
        let Some((mut table, changed)) =
            self.state_transition_table(statement, &key, current, writes)?
        else {
            return Ok(None);
        };
        // At bit k, table represents F^(2^k). Apply that power only when
        // the corresponding trip-count bit is set, then square for bit k+1.
        for bit in 0..count.width {
            if table.iter().enumerate().all(|(state, next)| state == *next) {
                break;
            }
            let advanced = self.lookup_state_transition(&table, current)?;
            let selected = self
                .rtl
                .expression_slice(count.id, bit, BitWidth::new(1)?)?;
            current.id = self.rtl.mux(selected, advanced.id, current.id)?;
            table = table.iter().map(|next| table[*next]).collect();
        }
        writes.insert(key, current);
        Ok(Some(changed))
    }

    fn state_transition_table(
        &mut self,
        statement: &ForStatement,
        key: &SignalKey,
        initial: LoweredExpr,
        writes: &Env,
    ) -> Result<Option<(Vec<usize>, DrivenBits)>, ImportError> {
        let mut table = Vec::new();
        let mut changed = DrivenBits::default();
        for state in 0..(1usize << initial.width) {
            let mut snapshot = writes.clone();
            let value = LoweredExpr {
                signed: initial.signed,
                ..self.constant(initial.width, state as u64)
            };
            snapshot.insert(key.clone(), value);
            for body in &statement.body {
                let Statement::Assign(assign) = body else {
                    unreachable!()
                };
                changed.extend(self.lower_assignment(assign, writes, &mut snapshot, false)?);
            }
            let Some(value) = self.known_rtl_constant(snapshot[key].id) else {
                return Ok(None);
            };
            let next =
                (0..initial.width).fold(0, |bits, bit| bits | (usize::from(value.bit(bit)) << bit));
            table.push(next);
        }
        Ok(Some((table, changed)))
    }

    fn lookup_state_transition(
        &mut self,
        table: &[usize],
        state: LoweredExpr,
    ) -> Result<LoweredExpr, ImportError> {
        let mut values = table
            .iter()
            .map(|value| self.constant(state.width, *value as u64).id)
            .collect::<Vec<_>>();
        for bit in 0..state.width {
            let selected = self
                .rtl
                .expression_slice(state.id, bit, BitWidth::new(1)?)?;
            values = values
                .as_chunks::<2>()
                .0
                .iter()
                .map(|pair| self.rtl.mux(selected, pair[1], pair[0]))
                .collect::<Result<_, _>>()?;
        }
        Ok(LoweredExpr {
            id: values[0],
            ..state
        })
    }
}

fn valid_assignment(statement: &Statement, state: VarId, width: u32) -> bool {
    let Statement::Assign(AssignStatement {
        dst,
        hier_dst: None,
        expr,
        ..
    }) = statement
    else {
        return false;
    };
    let [destination] = dst.as_slice() else {
        return false;
    };
    destination.id == state
        && destination.index.0.is_empty()
        && destination.comptime.member_select_domain.is_none()
        && literal_select(&destination.select)
        && static_select(&destination.select, width).is_ok()
        && state_expression(expr, state)
}

fn references_state(expr: &Expression, state: VarId) -> bool {
    matches!(expr, Expression::Term(factor) if matches!(factor.as_ref(), Factor::Variable(id, ..) if *id == state))
}

fn state_expression(expr: &Expression, state: VarId) -> bool {
    let check = |expr| state_expression(expr, state);
    match expr {
        Expression::Term(factor) => match factor.as_ref() {
            Factor::Value(_) => true,
            Factor::Variable(id, index, select, ct) => {
                *id == state
                    && index.0.is_empty()
                    && ct.member_select_domain.is_none()
                    && select.0.iter().all(check)
                    && select.1.as_ref().is_none_or(|(_, expr)| check(expr))
            }
            _ => false,
        },
        Expression::Unary(_, value, _) => check(value),
        Expression::Binary(lhs, _, rhs, _) => check(lhs) && check(rhs),
        Expression::Ternary(cond, yes, no, _) => check(cond) && check(yes) && check(no),
        Expression::Concatenation(parts, _) => parts
            .iter()
            .all(|(value, repeat)| check(value) && repeat.as_ref().is_none_or(check)),
        _ => false,
    }
}
