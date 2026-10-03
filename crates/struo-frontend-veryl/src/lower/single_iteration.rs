//! Lower a loop whose body always exits before its first counter update.
use veryl_analyzer::ir::{ForBound, ForRange, ForStatement};

use super::{
    DrivenBits, Env, ImportError, LoweredExpr, ModuleLowerer, Op, SignalKey, concrete_width,
};

impl ModuleLowerer<'_> {
    pub(super) fn lower_single_iteration(
        &mut self,
        statement: &ForStatement,
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<DrivenBits, ImportError> {
        let (initializer, bound, reverse, inclusive) = match &statement.range {
            ForRange::Forward {
                start,
                end,
                inclusive,
                ..
            }
            | ForRange::Stepped {
                start,
                end,
                inclusive,
                ..
            } => (start, end, false, *inclusive),
            ForRange::Reverse {
                start,
                end,
                inclusive,
                ..
            } => (end, start, true, *inclusive),
        };
        // Veryl saturates constant bounds to host usize / signed i64 maxima.
        // Comparison with an int counter tolerates that bound, but truncating
        // a saturated initializer cannot recover the original low bits.
        if let ForBound::Const(value, signed) = initializer
            && *value
                == if *signed {
                    usize::try_from(i64::MAX).unwrap_or(usize::MAX)
                } else {
                    usize::MAX
                }
        {
            return Err(ImportError::UnsupportedBehavior(
                "constant loop initializer may be saturated by the analyzer".into(),
            ));
        }
        let mut changed = DrivenBits::default();
        let mut initial = if sequential {
            let mut snapshot = self.sequential_reads(reads, writes);
            self.lower_single_loop_bound(initializer, &mut snapshot, &mut changed, false)?
        } else {
            self.lower_single_loop_bound(initializer, writes, &mut changed, true)?
        };
        let width = concrete_width(&statement.var_type, "loop induction variable")?;
        if reverse && !inclusive {
            let one = LoweredExpr {
                signed: true,
                ..self.constant(32, 1)
            };
            initial = self.lower_binary(
                Op::Sub,
                initial,
                one,
                initial.width.max(width).max(32),
                initial.signed,
            )?;
        }
        // The emitted SV assigns the initializer into the declared counter
        // before comparing it. Preserve truncation and signed interpretation.
        let counter = self.resize(initial, width, statement.var_type.signed)?;
        let mut snapshot = if sequential {
            self.sequential_reads(reads, writes)
        } else {
            writes.clone()
        };
        let bound = self.lower_single_loop_bound(bound, &mut snapshot, &mut changed, false)?;
        let op = if reverse {
            Op::GreaterEq
        } else if inclusive {
            Op::LessEq
        } else {
            Op::Less
        };
        let active = self.lower_binary(op, counter, bound, 1, false)?;
        let zero = self.constant(1, 0);
        let skip = self.lower_binary(Op::Eq, active, zero, 1, false)?;
        let before = writes.clone();
        let written = self.lower_single_body(statement, counter, reads, writes, sequential)?;
        *writes = self.merge_values(skip, &before, writes)?;
        changed.extend(written);
        Ok(changed)
    }

    fn lower_single_body(
        &mut self,
        statement: &ForStatement,
        counter: LoweredExpr,
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<DrivenBits, ImportError> {
        let key = SignalKey {
            id: statement.var_id,
            index: Vec::new(),
        };
        let previous = writes.insert(key.clone(), counter);
        let previous_width = self.widths.insert(key.clone(), counter.width);
        let previous_signed = self.signed.insert(key.clone(), statement.var_type.signed);
        let inserted = self.runtime_loop_variables.insert(statement.var_id);
        let mut body_reads = reads.clone();
        body_reads.insert(key.clone(), counter);
        let result = self.lower_loop_body(&statement.body, &body_reads, writes, sequential);
        if inserted {
            self.runtime_loop_variables.remove(&statement.var_id);
        }
        if let Some(value) = previous {
            writes.insert(key.clone(), value);
        } else {
            writes.remove(&key);
        }
        if let Some(value) = previous_width {
            self.widths.insert(key.clone(), value);
        } else {
            self.widths.remove(&key);
        }
        if let Some(value) = previous_signed {
            self.signed.insert(key.clone(), value);
        } else {
            self.signed.remove(&key);
        }
        let (written, _) = result?;
        Ok(written)
    }

    fn lower_single_loop_bound(
        &mut self,
        bound: &ForBound,
        env: &mut Env,
        effects: &mut DrivenBits,
        allow_effects: bool,
    ) -> Result<LoweredExpr, ImportError> {
        match bound {
            ForBound::Const(value, signed) => Ok(LoweredExpr {
                signed: *signed,
                ..self.constant(64, *value as u64)
            }),
            ForBound::Expression(expression) if allow_effects => {
                self.lower_comb_expression(expression, env, effects)
            }
            ForBound::Expression(expression) => self.lower_expression(expression, env),
        }
    }
}
