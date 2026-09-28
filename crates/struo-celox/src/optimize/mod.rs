//! Local optimizations applied while constructing one-bit simulation expressions.
//!
//! Cell conversion describes the hardware operation; this layer selects a
//! reduced expression and emits it through the backend SDK. Rules live in
//! operation-specific modules and do not mutate the builder. Add new local
//! operations here rather than embedding algebra in individual cell emitters.
//!
//! This is construction-time simplification, not a whole-IR optimization pass.
//! It deliberately uses only expression identity and the two canonical constant
//! IDs supplied by the caller. It neither inspects arbitrary expressions nor
//! performs global constant propagation, and must not be used for word muxes.

mod mux;

use celox::frontend_sdk::{BinaryOp, BuildError, ExprId, ModuleBuilder, UnaryOp, ValueType};

/// Canonical one-bit constants already created in the destination builder.
#[derive(Clone, Copy)]
pub(super) struct BitConstants {
    pub zero: ExprId,
    pub one: ExprId,
}

/// The result of a local rewrite, independent of backend expression allocation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Rewrite {
    Value(ExprId),
    Not(ExprId),
    And(ExprId, ExprId),
    Or(ExprId, ExprId),
    AndNot(ExprId, ExprId),
    OrNot(ExprId, ExprId),
    Mux(ExprId, ExprId, ExprId),
}

/// Emits simplified one-bit expressions. Operands must all have width one.
pub(super) struct BitOptimizer<'a> {
    builder: &'a mut ModuleBuilder,
    constants: BitConstants,
}

impl<'a> BitOptimizer<'a> {
    pub fn new(builder: &'a mut ModuleBuilder, constants: BitConstants) -> Self {
        Self { builder, constants }
    }

    pub fn mux(
        &mut self,
        select: ExprId,
        when_true: ExprId,
        when_false: ExprId,
    ) -> Result<ExprId, BuildError> {
        let rewrite = mux::simplify(select, when_true, when_false, self.constants);
        self.emit(rewrite)
    }

    fn emit(&mut self, rewrite: Rewrite) -> Result<ExprId, BuildError> {
        let bit = ValueType::bits(1)?;
        match rewrite {
            Rewrite::Value(value) => Ok(value),
            Rewrite::Not(value) => self.builder.unary(UnaryOp::LogicNot, value, bit),
            Rewrite::And(lhs, rhs) => self.builder.binary(BinaryOp::LogicAnd, lhs, rhs, bit),
            Rewrite::Or(lhs, rhs) => self.builder.binary(BinaryOp::LogicOr, lhs, rhs, bit),
            Rewrite::AndNot(lhs, rhs) | Rewrite::OrNot(lhs, rhs) => {
                let inverted = self.builder.unary(UnaryOp::LogicNot, lhs, bit)?;
                let op = if matches!(rewrite, Rewrite::AndNot(..)) {
                    BinaryOp::LogicAnd
                } else {
                    BinaryOp::LogicOr
                };
                self.builder.binary(op, inverted, rhs, bit)
            }
            Rewrite::Mux(select, when_true, when_false) => {
                self.builder.mux(select, when_true, when_false)
            }
        }
    }
}
