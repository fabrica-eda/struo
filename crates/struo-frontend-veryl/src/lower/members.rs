//! Lower member-coordinate arithmetic and retain the selected storage domain.
use veryl_analyzer::ir::{Expression, Factor, Op, ValueVariant};

use super::{BitWidth, Constant, DrivenBits, Env, ExprId, ImportError, LoweredExpr, ModuleLowerer};

impl ModuleLowerer<'_> {
    pub(super) fn lower_member_index(
        &mut self,
        expression: &Expression,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<LoweredExpr, ImportError> {
        // Veryl 0.22 builds member offsets from synthetic add/multiply nodes.
        // They have a default span or reuse an operand's span. Source operators
        // have their own span and retain normal HDL sizing. Coordinate arithmetic
        // must extend each source index before adding offsets or multiplying strides.
        let token = expression.comptime().token;
        if let Expression::Term(factor) = expression
            && let Factor::Value(ct) = factor.as_ref()
            && let ValueVariant::Numeric(value) = &ct.value
            && let Some(number) = value.to_usize_saturating()
        {
            let bits = (usize::BITS - number.leading_zeros()).max(1) + 1;
            return Ok(LoweredExpr {
                signed: true,
                ..self.constant(bits, number as u64)
            });
        }
        if let Expression::Binary(lhs, op @ (Op::Add | Op::Sub | Op::Mul), rhs, _) = expression
            && (matches!(
                token.beg.source,
                veryl_parser::veryl_token::TokenSource::Generated(_)
            ) || token == lhs.comptime().token
                || token == rhs.comptime().token)
        {
            let lhs = self.lower_member_index(lhs, env, effects)?;
            let rhs = self.lower_member_index(rhs, env, effects)?;
            let width = if *op == Op::Mul {
                lhs.width
                    .checked_add(rhs.width)
                    .and_then(|width| width.checked_add(1))
            } else {
                lhs.width.max(rhs.width).checked_add(2)
            }
            .ok_or_else(|| ImportError::WidthTooLarge("member offset".into()))?;
            let lhs = self.resize(lhs, width, true)?;
            let rhs = self.resize(rhs, width, true)?;
            return self.lower_binary(*op, lhs, rhs, width, true);
        }
        if let Expression::Unary(Op::Sub, input, _) = expression
            && matches!(
                token.beg.source,
                veryl_parser::veryl_token::TokenSource::Generated(_)
            )
        {
            let value = self.lower_member_index(input, env, effects)?;
            let width = value
                .width
                .checked_add(1)
                .ok_or_else(|| ImportError::WidthTooLarge("member offset".into()))?;
            let value = self.resize(value, width, true)?;
            let zero = self.constant(width, 0);
            return self.lower_binary(Op::Sub, zero, value, width, true);
        }
        let mut source = expression.clone();
        let mut context = veryl_analyzer::Context::default();
        let own_context = source.gather_context(&mut context);
        source.apply_context(&mut context, own_context);
        let errors = context.drain_errors();
        if errors.iter().any(veryl_analyzer::AnalyzerError::is_error) {
            return Err(ImportError::UnsupportedBehavior(format!(
                "member index type reconstruction: {errors:?}"
            )));
        }
        self.lower_comb_expression(&source, env, effects)
    }

    pub(super) fn member_mask(
        &mut self,
        width: u32,
        domain: veryl_analyzer::ir::MemberSelectDomain,
    ) -> Result<ExprId, ImportError> {
        if domain.low > domain.high || domain.high >= width as usize {
            return Err(ImportError::UnsupportedBehavior(
                "invalid packed member domain".into(),
            ));
        }
        let mut words = vec![0u64; (width as usize).div_ceil(64)];
        for bit in domain.low..=domain.high {
            words[bit / 64] |= 1u64 << (bit % 64);
        }
        Ok(self
            .rtl
            .constant(Constant::new(BitWidth::new(width)?, words)))
    }
}
