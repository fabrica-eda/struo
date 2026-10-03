//! Loop facts from already-lowered, whole-signal combinational drivers.
use veryl_analyzer::ir::{ForBound, ForRange, ForStatement};

use super::{Env, ExprKind, ImportError, ModuleLowerer, loops};

impl ModuleLowerer<'_> {
    pub(super) fn plan_constant_driven_reverse_loop(
        &mut self,
        statement: &ForStatement,
        reads: &Env,
        writes: &Env,
        sequential: bool,
    ) -> Result<Option<loops::LoopPlan>, ImportError> {
        let ForRange::Reverse {
            start: ForBound::Expression(bound),
            ..
        } = &statement.range
        else {
            return Ok(None);
        };
        let snapshot = if sequential {
            self.sequential_reads(reads, writes)
        } else {
            writes.clone()
        };
        let value = self.lower_expression(bound, &snapshot)?;
        if !value.signed || value.width > u64::BITS {
            return Ok(None);
        }
        // A procedural constant in this block is insufficient: its body could
        // change the lower bound. Require a wire driven by another completed
        // process instead. Overlapping drivers, including later body writes,
        // are rejected by the importer's existing ownership validation.
        let ExprKind::Signal(slice) = self.rtl.expressions()[value.id.index() as usize].kind()
        else {
            return Ok(None);
        };
        if slice.lsb != 0
            || slice.width
                != self.rtl.signals()[slice.signal.index() as usize]
                    .r#type()
                    .width
        {
            return Ok(None);
        }
        let mut drivers = self
            .rtl
            .assignments()
            .iter()
            .filter(|assignment| assignment.target.signal == slice.signal);
        let Some(driver) = drivers.next() else {
            return Ok(None);
        };
        if drivers.next().is_some() || driver.target != *slice {
            return Ok(None);
        }
        let Some(constant) = self.known_rtl_constant(driver.value) else {
            return Ok(None);
        };
        let bits = (0..value.width).fold(0u64, |bits, bit| {
            bits | (u64::from(constant.bit(bit)) << bit)
        });
        let lower = i128::from(bits)
            - if constant.bit(value.width - 1) {
                1i128 << value.width
            } else {
                0
            };
        loops::plan_reverse_with_lower(statement, lower).map(Some)
    }
}
