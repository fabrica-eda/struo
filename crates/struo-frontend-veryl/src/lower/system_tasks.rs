//! Preserve procedural argument effects of simulation-only output tasks.
use veryl_analyzer::ir::{Expression, Factor, SystemFunctionCall, SystemFunctionKind};

use super::{DrivenBits, Env, ImportError, ModuleLowerer};

impl ModuleLowerer<'_> {
    pub(super) fn lower_system_statement(
        &mut self,
        call: &SystemFunctionCall,
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<DrivenBits, ImportError> {
        let mut effects = DrivenBits::default();
        if let SystemFunctionKind::Display(args) | SystemFunctionKind::Write(args) = &call.kind {
            // Display/write produce no hardware output, but function calls in
            // their argument expressions can write observable signals. Keep
            // those effects in the surrounding procedure (IEEE 1800-2023 21.2.1).
            for arg in args {
                if matches!(&arg.0, Expression::Term(factor)
                    if matches!(factor.as_ref(), Factor::Value(_) | Factor::Unknown(_)))
                {
                    // Literal formats and values cannot have argument effects.
                    continue;
                }
                if sequential {
                    self.lower_expression(&arg.0, reads)?;
                } else {
                    self.lower_comb_expression(&arg.0, writes, &mut effects)?;
                }
            }
        } else if sequential {
            self.lower_system_function(call, reads)?;
        } else {
            self.lower_system_function_effects(call, writes, &mut effects)?;
        }
        Ok(effects)
    }
}
