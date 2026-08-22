use std::collections::HashMap;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub(crate) struct Literal(pub(crate) usize);

impl Literal {
    pub(crate) const FALSE: Self = Self(0);
    pub(crate) const TRUE: Self = Self(1);

    pub(crate) const fn node(self) -> usize {
        self.0 >> 1
    }

    pub(crate) const fn inverted(self) -> bool {
        self.0 & 1 != 0
    }

    pub(crate) const fn negate(self) -> Self {
        Self(self.0 ^ 1)
    }
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub(crate) enum Variable {
    Input(String),
    State(String),
    Proof(String),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum Node {
    Constant,
    Variable(Variable),
    And(Literal, Literal),
}

#[derive(Clone, Debug)]
pub(crate) struct Aig {
    nodes: Vec<Node>,
    variables: HashMap<Variable, Literal>,
    ands: HashMap<(Literal, Literal), Literal>,
}

impl Default for Aig {
    fn default() -> Self {
        Self::new()
    }
}

impl Aig {
    pub(crate) fn new() -> Self {
        Self {
            nodes: vec![Node::Constant],
            variables: HashMap::new(),
            ands: HashMap::new(),
        }
    }

    pub(crate) fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    pub(crate) fn variable(&mut self, variable: Variable) -> Literal {
        if let Some(literal) = self.variables.get(&variable) {
            return *literal;
        }
        let literal = Literal(self.nodes.len() << 1);
        self.nodes.push(Node::Variable(variable.clone()));
        self.variables.insert(variable, literal);
        literal
    }

    pub(crate) fn and(&mut self, lhs: Literal, rhs: Literal) -> Literal {
        if lhs == Literal::FALSE || rhs == Literal::FALSE {
            return Literal::FALSE;
        }
        if lhs == Literal::TRUE {
            return rhs;
        }
        if rhs == Literal::TRUE || lhs == rhs {
            return lhs;
        }
        if lhs == rhs.negate() {
            return Literal::FALSE;
        }
        let inputs = if lhs <= rhs { (lhs, rhs) } else { (rhs, lhs) };
        if let Some(literal) = self.ands.get(&inputs) {
            return *literal;
        }
        let literal = Literal(self.nodes.len() << 1);
        self.nodes.push(Node::And(inputs.0, inputs.1));
        self.ands.insert(inputs, literal);
        literal
    }

    pub(crate) fn or(&mut self, lhs: Literal, rhs: Literal) -> Literal {
        self.and(lhs.negate(), rhs.negate()).negate()
    }

    pub(crate) fn xor(&mut self, lhs: Literal, rhs: Literal) -> Literal {
        if lhs == rhs {
            return Literal::FALSE;
        }
        if lhs == rhs.negate() {
            return Literal::TRUE;
        }
        let lhs_only = self.and(lhs, rhs.negate());
        let rhs_only = self.and(lhs.negate(), rhs);
        self.or(lhs_only, rhs_only)
    }

    pub(crate) fn mux(
        &mut self,
        condition: Literal,
        then_value: Literal,
        else_value: Literal,
    ) -> Literal {
        if condition == Literal::TRUE {
            return then_value;
        }
        if condition == Literal::FALSE {
            return else_value;
        }
        if then_value == else_value {
            return then_value;
        }
        if then_value == Literal::TRUE && else_value == Literal::FALSE {
            return condition;
        }
        if then_value == Literal::FALSE && else_value == Literal::TRUE {
            return condition.negate();
        }
        let selected_then = self.and(condition, then_value);
        let selected_else = self.and(condition.negate(), else_value);
        self.or(selected_then, selected_else)
    }

    pub(crate) fn evaluate(literal: Literal, values: &[bool]) -> bool {
        values[literal.node()] ^ literal.inverted()
    }
}

#[cfg(test)]
mod tests {
    use super::{Aig, Literal, Variable};

    #[test]
    fn structurally_hashes_and_simplifies_boolean_logic() {
        let mut aig = Aig::new();
        let a = aig.variable(Variable::Input("a".into()));
        let b = aig.variable(Variable::Input("b".into()));

        assert_eq!(aig.and(a, b), aig.and(b, a));
        assert_eq!(aig.and(a, Literal::TRUE), a);
        assert_eq!(aig.and(a, a.negate()), Literal::FALSE);
        assert_eq!(aig.xor(a, a), Literal::FALSE);
        assert_eq!(aig.mux(a, Literal::TRUE, Literal::FALSE), a);
    }
}
