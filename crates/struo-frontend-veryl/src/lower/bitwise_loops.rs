//! Prove finite bitwise-counter traces from a known procedural initializer.
use veryl_analyzer::ir::{ForRange, ForStatement};

use super::{
    Env, HashSet, ImportError, ModuleLowerer, Op, concrete_width, loops, substitute_statements,
};

impl ModuleLowerer<'_> {
    pub(super) fn plan_constant_bitwise_loop(
        &mut self,
        statement: &ForStatement,
        reads: &Env,
        writes: &Env,
        sequential: bool,
    ) -> Result<Option<loops::LoopPlan>, ImportError> {
        let ForRange::Stepped {
            start,
            end,
            inclusive,
            step,
            op: op @ (Op::BitOr | Op::BitXor),
        } = &statement.range
        else {
            return Ok(None);
        };
        let width = concrete_width(&statement.var_type, "loop induction variable")?;
        // The analyzer also saturates step constants. A saturated host maximum
        // cannot establish the original low counter bits.
        if width > usize::BITS || *step == usize::MAX {
            return Ok(None);
        }
        let Some(initial) = self.known_loop_initializer(start, reads, writes, sequential, width)?
        else {
            return Ok(None);
        };
        let mask = usize::MAX >> (usize::BITS - width);
        let mut counter = initial & mask;
        let mut seen = HashSet::new();
        let mut iterations = Vec::new();
        for _ in 0..loops::GUARDED_ITERATION_BUDGET {
            if !seen.insert(counter) {
                // Repeating a counter value without an unconditional exit is
                // not a termination proof, even if sampled inputs stop there.
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
            counter = match op {
                Op::BitOr => counter | step,
                Op::BitXor => counter ^ step,
                _ => unreachable!(),
            } & mask;
        }
        Ok(None)
    }
}
