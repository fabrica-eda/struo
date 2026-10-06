//! Comparison contexts and constant RHS wildcard masks.
use super::{
    BinaryOp, BitWidth, Comptime, Constant, ImportError, LoweredExpr, ModuleLowerer, Op, UnaryOp,
    ValueVariant, context_width,
};

impl ModuleLowerer<'_> {
    pub(super) fn lower_wildcard_pattern(
        &mut self,
        lhs: LoweredExpr,
        op: Op,
        comptime: &Comptime,
    ) -> Result<LoweredExpr, ImportError> {
        let ValueVariant::Numeric(value) = &comptime.value else {
            unreachable!()
        };
        // IEEE 1800-2023 11.4.6: only RHS X/Z bits are wildcards, and
        // operand conversion/extension follows logical equality. Extend the
        // four-state constant before deriving its mask (including an X/Z sign bit).
        let rhs_width = if value.width() == 0 {
            context_width(comptime)?
        } else {
            u32::try_from(value.width())
                .map_err(|_| ImportError::WidthTooLarge("wildcard pattern".into()))?
        };
        let width = lhs.width.max(rhs_width);
        let signed = lhs.signed && value.signed();
        let lhs = self.resize(LoweredExpr { signed, ..lhs }, width, signed)?;
        let value = value.expand(width as usize, signed);
        let mask = self.rtl.constant(Constant::new(
            BitWidth::new(width)?,
            value.mask_xz().to_u64_digits(),
        ));
        let care = self.rtl.unary(UnaryOp::BitNot, mask)?;
        let rhs = self.rtl.constant(Constant::new(
            BitWidth::new(width)?,
            value.payload().to_u64_digits(),
        ));
        let lhs = self.rtl.binary(BinaryOp::And, lhs.id, care)?;
        let rhs = self.rtl.binary(BinaryOp::And, rhs, care)?;
        let op = if op == Op::EqWildcard {
            BinaryOp::Equal
        } else {
            BinaryOp::NotEqual
        };
        Ok(LoweredExpr {
            id: self.rtl.binary(op, lhs, rhs)?,
            width: 1,
            signed: false,
        })
    }
}

// Only compile-time operands can provide a hardware comparison mask. Runtime
// four-state data remains unsupported; do not erase effects of runtime calls.
pub(super) fn wildcard_pattern(expression: &super::Expression) -> Option<Comptime> {
    if !expression.comptime().is_const {
        return None;
    }
    let value = expression.eval_value(&mut veryl_analyzer::Context::default())?;
    if !value.is_xz() {
        return None;
    }
    let mut comptime = expression.comptime().clone();
    comptime.value = ValueVariant::Numeric(value);
    Some(comptime)
}

pub(super) struct PreparedCaseTarget {
    expression: super::Expression,
    value: LoweredExpr,
    inputs: super::Env,
}

impl ModuleLowerer<'_> {
    pub(super) fn prepare_case_target(
        &mut self,
        expression: &super::Expression,
        reads: &super::Env,
        writes: &mut super::Env,
        sequential: bool,
        effects: &mut super::DrivenBits,
    ) -> Result<PreparedCaseTarget, ImportError> {
        let inputs = if sequential {
            reads.clone()
        } else {
            writes.clone()
        };
        let value = if sequential {
            self.lower_expression(expression, reads)?
        } else {
            self.lower_comb_expression(expression, writes, effects)?
        };
        Ok(PreparedCaseTarget {
            expression: expression.clone(),
            value,
            inputs,
        })
    }

    /// Whether the arms' constant equality labels match every value of the
    /// case target, making a `default` arm unreachable.
    ///
    /// Only labels compared in the target's own width and signedness are
    /// counted (the comparison is then on the target value itself); wildcard
    /// and range labels are ignored, so an uncertain case keeps its default.
    pub(super) fn case_arms_cover_target<'a>(
        target: &PreparedCaseTarget,
        arm_patterns: impl IntoIterator<Item = &'a [super::CasePattern]>,
    ) -> bool {
        const MAX_COVERED_WIDTH: u32 = 12;
        let width = target.value.width;
        if width > MAX_COVERED_WIDTH {
            return false;
        }
        let own = target.expression.comptime().expr_context;
        let mut covered = vec![false; 1 << width];
        for patterns in arm_patterns {
            for pattern in patterns {
                let super::CasePattern::Eq(label) = pattern else {
                    continue;
                };
                let pair = label.comptime().expr_context;
                if !(target.expression.is_self_determined()
                    || (pair.width == own.width && pair.signed == own.signed))
                    || wildcard_pattern(label.as_ref()).is_some()
                {
                    continue;
                }
                if let Some(index) = super::evaluated_u64(label.as_ref())
                    .filter(|value| *value >> width == 0)
                    .and_then(|value| usize::try_from(value).ok())
                {
                    covered[index] = true;
                }
            }
        }
        covered.iter().all(|covered| *covered)
    }

    pub(super) fn lower_case_label(
        &mut self,
        expression: &super::Expression,
        env: &super::Env,
    ) -> Result<LoweredExpr, ImportError> {
        let mut expression = expression.clone();
        let signed = expression.comptime().expr_context.signed;
        super::repair_expression_signedness(&mut expression, Some(signed));
        self.lower_prepared_expression(&expression, env)
    }

    pub(super) fn lower_case_operand(
        &mut self,
        target: &PreparedCaseTarget,
        operand: &super::Expression,
    ) -> Result<LoweredExpr, ImportError> {
        // Veryl gives each label/range endpoint its own comparison context.
        // Resize the target's computation, not its already-truncated result.
        let pair = operand.comptime().expr_context;
        let own = target.expression.comptime().expr_context;
        if target.expression.is_self_determined()
            || (pair.width == own.width && pair.signed == own.signed)
        {
            return Ok(target.value);
        }
        let mut expression = target.expression.clone();
        let mut context = veryl_analyzer::Context::default();
        expression.apply_context(&mut context, pair);
        let errors = context.drain_errors();
        if errors.iter().any(veryl_analyzer::AnalyzerError::is_error) {
            return Err(ImportError::UnsupportedBehavior(format!(
                "case target context: {errors:?}"
            )));
        }
        super::repair_expression_signedness(&mut expression, Some(pair.signed));
        // Target effects were committed once by prepare_case_target. Rebuild
        // only the context-dependent value from its captured input environment;
        // no writes from this reconstruction are committed to the caller.
        let mut inputs = target.inputs.clone();
        self.lower_expression_effects(&expression, &mut inputs, &mut super::DrivenBits::default())
    }
}
