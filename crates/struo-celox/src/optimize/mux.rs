//! Ordered one-bit MUX identities. More specific rules precede general ones.
use super::{BitConstants, Rewrite};
use celox::frontend_sdk::ExprId;

pub(super) fn simplify(
    select: ExprId,
    when_true: ExprId,
    when_false: ExprId,
    constants: BitConstants,
) -> Rewrite {
    let BitConstants { zero, one } = constants;
    if select == zero {
        Rewrite::Value(when_false)
    } else if select == one || when_true == when_false {
        Rewrite::Value(when_true)
    } else if when_false == zero && when_true == one {
        Rewrite::Value(select)
    } else if when_false == one && when_true == zero {
        Rewrite::Not(select)
    } else if when_false == zero {
        Rewrite::And(select, when_true)
    } else if when_true == one {
        Rewrite::Or(select, when_false)
    } else if when_true == zero {
        Rewrite::AndNot(select, when_false)
    } else if when_false == one {
        Rewrite::OrNot(select, when_true)
    } else {
        Rewrite::Mux(select, when_true, when_false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use celox::frontend_sdk::{Constant, ModuleBuilder, ValueType};

    #[test]
    fn selects_specific_rules_and_preserves_general_muxes() {
        let mut builder = ModuleBuilder::new("rules").unwrap();
        let zero = builder.constant(Constant::two_state(0u8, 1).unwrap());
        let one = builder.constant(Constant::two_state(1u8, 1).unwrap());
        let mut input = |name| {
            let signal = builder.input(name, ValueType::bits(1).unwrap()).unwrap();
            builder.read_slice(builder.whole(signal).unwrap()).unwrap()
        };
        let select = input("select");
        let a = input("a");
        let b = input("b");
        let constants = BitConstants { zero, one };
        assert_eq!(simplify(zero, a, b, constants), Rewrite::Value(b));
        assert_eq!(simplify(one, a, b, constants), Rewrite::Value(a));
        for (yes, no, expected) in [
            (a, a, Rewrite::Value(a)),
            (zero, zero, Rewrite::Value(zero)),
            (one, one, Rewrite::Value(one)),
            (one, zero, Rewrite::Value(select)),
            (zero, one, Rewrite::Not(select)),
            (a, zero, Rewrite::And(select, a)),
            (one, a, Rewrite::Or(select, a)),
            (zero, a, Rewrite::AndNot(select, a)),
            (a, one, Rewrite::OrNot(select, a)),
            (a, b, Rewrite::Mux(select, a, b)),
        ] {
            assert_eq!(simplify(select, yes, no, constants), expected);
        }
    }
}
