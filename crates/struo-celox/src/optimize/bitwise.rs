//! Two-state bitwise identities independent of backend allocation.
use celox::frontend_sdk::ExprId;

use super::{BitConstants, Rewrite};

pub(super) fn simplify_xor(lhs: ExprId, rhs: ExprId, constants: BitConstants) -> Rewrite {
    if lhs == rhs {
        Rewrite::Value(constants.zero)
    } else if lhs == constants.zero {
        Rewrite::Value(rhs)
    } else if rhs == constants.zero {
        Rewrite::Value(lhs)
    } else if lhs == constants.one {
        Rewrite::Not(rhs)
    } else if rhs == constants.one {
        Rewrite::Not(lhs)
    } else {
        Rewrite::Xor(lhs, rhs)
    }
}
