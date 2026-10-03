//! Count a proven periodic predicate before applying a modular scalar reduction.
use veryl_analyzer::ir::{AssignStatement, ForBound, ForRange, ForStatement, Module};

use super::{
    BinaryOp, BitWidth, DrivenBits, Env, Expression, Factor, ImportError, LoweredExpr,
    ModuleLowerer, Op, Statement, VarId, concrete_width, loops,
    reductions::{constant_addition, whole_variable},
    substitute_induction, types,
};

impl ModuleLowerer<'_> {
    pub(super) fn lower_periodic_reduction(
        &mut self,
        statement: &ForStatement,
        writes: &mut Env,
    ) -> Result<Option<DrivenBits>, ImportError> {
        let counter_width = concrete_width(&statement.var_type, "periodic counter")?;
        let Some(range) = periodic_range(statement, self.source, counter_width) else {
            return Ok(None);
        };
        let Some((condition, assign)) = periodic_body(statement) else {
            return Ok(None);
        };
        let Some(bits @ 0..=8) = period_bits(condition, statement.var_id, counter_width) else {
            return Ok(None);
        };
        if matches!(range, PeriodicRange::RuntimeStart { .. }) && bits >= counter_width {
            return Ok(None);
        }
        let Some((accumulator, increment)) = constant_addition(assign) else {
            return Ok(None);
        };
        let [destination] = assign.dst.as_slice() else {
            return Ok(None);
        };
        let ty = &self.source.variables[&destination.id].r#type;
        if assign.hier_dst.is_some()
            || !destination.index.0.is_empty()
            || !destination.select.is_empty()
            || destination.comptime.member_select_domain.is_some()
            || !ty.array.is_empty()
            || Some(destination.id) == range.mutable_bound()
            || destination.id == statement.var_id
        {
            return Ok(None);
        }
        let Some(prefix) = self.predicate_prefix(condition, statement.var_id, bits, writes)? else {
            return Ok(None);
        };
        let width = concrete_width(ty, "periodic accumulator")?;
        let initial = self.lower_expression(accumulator, writes)?;
        let mut increment = self.lower_expression(increment, writes)?;
        increment.signed = types::expression_signedness(&assign.expr);
        let increment = self.resize(increment, width, false)?;
        let hits = self.periodic_hits(range, counter_width, bits, &prefix, width, writes)?;
        let delta = self.lower_binary(Op::Mul, hits, increment, width, false)?;
        let initial = self.resize(initial, width, false)?;
        let value = self.lower_binary(Op::Add, initial, delta, width, false)?;
        Ok(Some(self.assign_destinations_effects(
            &assign.dst,
            value,
            writes,
        )?))
    }

    fn periodic_hits(
        &mut self,
        range: PeriodicRange<'_>,
        counter_width: u32,
        bits: u32,
        prefix: &[u64],
        width: u32,
        writes: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let (hits, empty) = match range {
            PeriodicRange::RuntimeEnd { start, end } => {
                let count = self.lower_expression(end, writes)?;
                let total = self.periodic_prefix(count, bits, prefix, width)?;
                let start_total = self.constant(width, constant_prefix(start, bits, prefix));
                let hits = self.lower_binary(Op::Sub, total, start_total, width, false)?;
                let start = self.constant(counter_width, start);
                let empty = self.lower_binary(Op::LessEq, count, start, 1, false)?;
                (hits, empty)
            }
            PeriodicRange::RuntimeStart { start, end } => {
                // Capture the initializer with the counter's assignment conversion.
                let captured = self.lower_expression(start, writes)?;
                let captured = self.resize(captured, counter_width, true)?;
                // Bias signed order into unsigned ordinals. The predicate uses
                // only lower bits, so flipping the sign bit preserves its phase.
                let sign = 1u64 << (counter_width - 1);
                let sign_mask = self.constant(counter_width, sign);
                let ordinal = LoweredExpr {
                    id: self.rtl.binary(BinaryOp::Xor, captured.id, sign_mask.id)?,
                    signed: false,
                    ..captured
                };
                let start_total = self.periodic_prefix(ordinal, bits, prefix, width)?;
                let end_ordinal = end ^ sign;
                let end_total = self.constant(width, constant_prefix(end_ordinal, bits, prefix));
                let hits = self.lower_binary(Op::Sub, end_total, start_total, width, false)?;
                let end = self.constant(counter_width, end_ordinal);
                let empty = self.lower_binary(Op::GreaterEq, ordinal, end, 1, false)?;
                (hits, empty)
            }
        };
        let zero = self.constant(width, 0);
        Ok(LoweredExpr {
            id: self.rtl.mux(empty.id, zero.id, hits.id)?,
            ..hits
        })
    }

    fn predicate_prefix(
        &mut self,
        condition: &Expression,
        counter: VarId,
        bits: u32,
        writes: &Env,
    ) -> Result<Option<Vec<u64>>, ImportError> {
        let mut prefix = vec![0];
        for residue in 0..(1usize << bits) {
            let mut condition = condition.clone();
            substitute_induction(&mut condition, counter, residue)?;
            let value = self.lower_expression(&condition, writes)?;
            let Some(constant) = self.known_rtl_constant(value.id) else {
                return Ok(None);
            };
            let enabled = (0..value.width).any(|bit| constant.bit(bit));
            prefix.push(prefix[residue] + u64::from(enabled));
        }
        Ok(Some(prefix))
    }

    fn periodic_prefix(
        &mut self,
        count: LoweredExpr,
        bits: u32,
        prefix: &[u64],
        width: u32,
    ) -> Result<LoweredExpr, ImportError> {
        // Divide the original count before narrowing to the accumulator width.
        // Otherwise a narrow accumulator would lose whole predicate periods.
        let quotient = if bits == 0 {
            count
        } else if count.width > bits {
            LoweredExpr {
                id: self.rtl.expression_slice(
                    count.id,
                    bits,
                    BitWidth::new(count.width - bits)?,
                )?,
                width: count.width - bits,
                signed: false,
            }
        } else {
            self.constant(1, 0)
        };
        let quotient = self.resize(quotient, width, false)?;
        let per_period = self.constant(width, *prefix.last().expect("nonempty prefix"));
        let full = self.lower_binary(Op::Mul, quotient, per_period, width, false)?;
        let mut values = prefix[..prefix.len() - 1]
            .iter()
            .map(|value| self.constant(width, *value).id)
            .collect::<Vec<_>>();
        let remainder = self.resize(count, bits.max(1), false)?;
        for bit in 0..bits {
            let selected = self
                .rtl
                .expression_slice(remainder.id, bit, BitWidth::new(1)?)?;
            values = values
                .as_chunks::<2>()
                .0
                .iter()
                .map(|pair| self.rtl.mux(selected, pair[1], pair[0]))
                .collect::<Result<_, _>>()?;
        }
        let tail = LoweredExpr {
            id: values[0],
            width,
            signed: false,
        };
        self.lower_binary(Op::Add, full, tail, width, false)
    }
}

#[derive(Clone, Copy)]
enum PeriodicRange<'a> {
    RuntimeEnd { start: u64, end: &'a Expression },
    RuntimeStart { start: &'a Expression, end: u64 },
}

impl PeriodicRange<'_> {
    fn mutable_bound(&self) -> Option<VarId> {
        match self {
            Self::RuntimeEnd { end, .. } => whole_variable(end),
            Self::RuntimeStart { .. } => None,
        }
    }
}

fn periodic_range<'a>(
    statement: &'a ForStatement,
    source: &Module,
    width: u32,
) -> Option<PeriodicRange<'a>> {
    if !(1..=64).contains(&width) {
        return None;
    }
    let maximum = (u64::MAX >> (64 - width)) >> u32::from(statement.var_type.signed);
    match &statement.range {
        ForRange::Forward {
            start: ForBound::Const(start, _),
            ..
        } if concrete_endpoint(*start, maximum) => {
            let end = loops::unsigned_unit_bound(statement, source)?;
            Some(PeriodicRange::RuntimeEnd {
                start: *start as u64,
                end,
            })
        }
        ForRange::Forward {
            start: ForBound::Expression(start),
            end: ForBound::Const(end, true),
            inclusive: false,
            step: 1,
        } if statement.var_type.signed && concrete_endpoint(*end, maximum) => {
            let id = whole_variable(start)?;
            if !source.variables.get(&id)?.r#type.array.is_empty() {
                return None;
            }
            Some(PeriodicRange::RuntimeStart {
                start,
                end: *end as u64,
            })
        }
        _ => None,
    }
}

fn concrete_endpoint(value: usize, maximum: u64) -> bool {
    value as u64 <= maximum && value != usize::MAX && value as u64 != i64::MAX.unsigned_abs()
}

fn constant_prefix(point: u64, bits: u32, prefix: &[u64]) -> u64 {
    let residue = usize::try_from(point & ((1u64 << bits) - 1)).expect("at most eight bits");
    (point >> bits) * prefix[prefix.len() - 1] + prefix[residue]
}

fn periodic_body(statement: &ForStatement) -> Option<(&Expression, &AssignStatement)> {
    let [Statement::If(branch)] = statement.body.as_slice() else {
        return None;
    };
    let [Statement::Assign(assign)] = branch.true_side.as_slice() else {
        return None;
    };
    branch
        .false_side
        .is_empty()
        .then_some((&branch.cond, assign))
}

// Upper bound on the number of low counter bits that can influence a pure
// expression. A cast directly on the counter removes its higher bits; an
// arbitrary operation followed by a cast does not receive that assumption.
fn period_bits(expr: &Expression, counter: VarId, counter_width: u32) -> Option<u32> {
    let check = |expr| period_bits(expr, counter, counter_width);
    match expr {
        Expression::Term(factor) => match factor.as_ref() {
            Factor::Value(_) => Some(0),
            Factor::Variable(id, index, select, ct)
                if *id == counter
                    && index.0.is_empty()
                    && ct.member_select_domain.is_none()
                    && select.is_empty() =>
            {
                Some(counter_width)
            }
            _ => None,
        },
        Expression::Binary(lhs, Op::As, rhs, ct) if matches!(rhs.as_ref(), Expression::Term(factor) if matches!(factor.as_ref(), Factor::Value(_))) =>
        {
            let dependencies = check(lhs)?;
            if whole_variable(lhs) == Some(counter) {
                Some(dependencies.min(concrete_width(&ct.r#type, "periodic cast").ok()?))
            } else {
                Some(dependencies)
            }
        }
        Expression::Unary(_, input, _) => check(input),
        Expression::Binary(lhs, _, rhs, _) => Some(check(lhs)?.max(check(rhs)?)),
        Expression::Ternary(cond, yes, no, _) => {
            Some(check(cond)?.max(check(yes)?).max(check(no)?))
        }
        Expression::Concatenation(parts, _) => {
            let mut bits = 0;
            for (part, repeat) in parts {
                bits = bits.max(check(part)?);
                if let Some(repeat) = repeat {
                    bits = bits.max(check(repeat)?);
                }
            }
            Some(bits)
        }
        _ => None,
    }
}
