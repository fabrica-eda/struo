use std::cmp::Ordering;

use crate::aig::{Aig, Literal, Node};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SatLiteral {
    variable: usize,
    positive: bool,
}

impl SatLiteral {
    const fn positive(variable: usize) -> Self {
        Self {
            variable,
            positive: true,
        }
    }

    const fn negate(self) -> Self {
        Self {
            variable: self.variable,
            positive: !self.positive,
        }
    }

    const fn watch_index(self) -> usize {
        self.variable * 2 + if self.positive { 0 } else { 1 }
    }
}

#[derive(Clone, Debug)]
struct Clause {
    literals: Vec<SatLiteral>,
    watched: [usize; 2],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Value {
    False,
    Unassigned,
    True,
}

#[derive(Debug)]
struct Solver {
    clauses: Vec<Clause>,
    watches: Vec<Vec<usize>>,
    assignments: Vec<Value>,
    levels: Vec<usize>,
    reasons: Vec<Option<usize>>,
    phases: Vec<bool>,
    trail: Vec<SatLiteral>,
    trail_limits: Vec<usize>,
    propagation_head: usize,
    activity: Vec<f64>,
    activity_increment: f64,
    inconsistent: bool,
}

impl Solver {
    fn new(variable_count: usize) -> Self {
        Self {
            clauses: Vec::new(),
            watches: vec![Vec::new(); (variable_count + 1) * 2],
            assignments: vec![Value::Unassigned; variable_count + 1],
            levels: vec![0; variable_count + 1],
            reasons: vec![None; variable_count + 1],
            phases: vec![false; variable_count + 1],
            trail: Vec::new(),
            trail_limits: Vec::new(),
            propagation_head: 0,
            activity: vec![0.0; variable_count + 1],
            activity_increment: 1.0,
            inconsistent: false,
        }
    }

    fn decision_level(&self) -> usize {
        self.trail_limits.len()
    }

    fn value(&self, literal: SatLiteral) -> Value {
        match self.assignments[literal.variable] {
            Value::Unassigned => Value::Unassigned,
            Value::True if literal.positive => Value::True,
            Value::False if !literal.positive => Value::True,
            Value::True | Value::False => Value::False,
        }
    }

    fn add_clause(&mut self, mut literals: Vec<SatLiteral>) -> Option<usize> {
        literals.sort_by_key(|literal| (literal.variable, literal.positive));
        literals.dedup();
        if literals.windows(2).any(|pair| {
            pair[0].variable == pair[1].variable && pair[0].positive != pair[1].positive
        }) {
            return None;
        }
        if literals.is_empty() {
            self.inconsistent = true;
            return None;
        }
        for literal in &literals {
            self.activity[literal.variable] += 1.0;
        }
        let second = usize::from(literals.len() > 1);
        let clause_id = self.clauses.len();
        self.clauses.push(Clause {
            literals,
            watched: [0, second],
        });
        let first_literal = self.clauses[clause_id].literals[0];
        self.watches[first_literal.watch_index()].push(clause_id);
        if second != 0 {
            let second_literal = self.clauses[clause_id].literals[second];
            self.watches[second_literal.watch_index()].push(clause_id);
        }
        Some(clause_id)
    }

    fn enqueue(&mut self, literal: SatLiteral, reason: Option<usize>) -> bool {
        match self.value(literal) {
            Value::True => true,
            Value::False => false,
            Value::Unassigned => {
                self.assignments[literal.variable] = if literal.positive {
                    Value::True
                } else {
                    Value::False
                };
                self.levels[literal.variable] = self.decision_level();
                self.reasons[literal.variable] = reason;
                self.phases[literal.variable] = literal.positive;
                self.trail.push(literal);
                true
            }
        }
    }

    fn initialize_units(&mut self) -> bool {
        let units = self
            .clauses
            .iter()
            .enumerate()
            .filter(|(_, clause)| clause.literals.len() == 1)
            .map(|(clause_id, clause)| (clause.literals[0], clause_id))
            .collect::<Vec<_>>();
        units
            .into_iter()
            .all(|(literal, clause_id)| self.enqueue(literal, Some(clause_id)))
    }

    fn propagate(&mut self) -> Option<usize> {
        while self.propagation_head < self.trail.len() {
            let assigned = self.trail[self.propagation_head];
            self.propagation_head += 1;
            let false_literal = assigned.negate();
            let watch_index = false_literal.watch_index();
            let pending = std::mem::take(&mut self.watches[watch_index]);

            for (pending_index, clause_id) in pending.iter().copied().enumerate() {
                let false_slot = {
                    let clause = &self.clauses[clause_id];
                    if clause.literals[clause.watched[0]] == false_literal {
                        0
                    } else if clause.literals[clause.watched[1]] == false_literal {
                        1
                    } else {
                        continue;
                    }
                };
                let other_slot = 1 - false_slot;
                let other_literal = {
                    let clause = &self.clauses[clause_id];
                    clause.literals[clause.watched[other_slot]]
                };
                if self.value(other_literal) == Value::True {
                    self.watches[watch_index].push(clause_id);
                    continue;
                }

                let replacement = {
                    let clause = &self.clauses[clause_id];
                    clause
                        .literals
                        .iter()
                        .enumerate()
                        .find(|(literal_index, literal)| {
                            *literal_index != clause.watched[0]
                                && *literal_index != clause.watched[1]
                                && self.value(**literal) != Value::False
                        })
                        .map(|(literal_index, _)| literal_index)
                };
                if let Some(replacement) = replacement {
                    self.clauses[clause_id].watched[false_slot] = replacement;
                    let replacement_literal = self.clauses[clause_id].literals[replacement];
                    self.watches[replacement_literal.watch_index()].push(clause_id);
                    continue;
                }

                self.watches[watch_index].push(clause_id);
                if self.value(other_literal) == Value::False
                    || !self.enqueue(other_literal, Some(clause_id))
                {
                    self.watches[watch_index]
                        .extend(pending.iter().skip(pending_index + 1).copied());
                    return Some(clause_id);
                }
            }
        }
        None
    }

    fn bump_activity(&mut self, variable: usize) {
        self.activity[variable] += self.activity_increment;
        if self.activity[variable] > 1.0e100 {
            for activity in &mut self.activity {
                *activity *= 1.0e-100;
            }
            self.activity_increment *= 1.0e-100;
        }
    }

    fn analyze(&mut self, conflict: usize) -> (Vec<SatLiteral>, usize) {
        let current_level = self.decision_level();
        let mut learned = vec![SatLiteral::positive(0)];
        let mut seen = vec![false; self.assignments.len()];
        let mut open_at_level = 0usize;
        let mut trail_index = self.trail.len();
        let mut clause_id = conflict;
        let mut resolved_variable = 0usize;

        loop {
            let literals = self.clauses[clause_id].literals.clone();
            for literal in literals {
                let variable = literal.variable;
                if variable == resolved_variable || seen[variable] || self.levels[variable] == 0 {
                    continue;
                }
                seen[variable] = true;
                self.bump_activity(variable);
                if self.levels[variable] == current_level {
                    open_at_level += 1;
                } else {
                    learned.push(literal);
                }
            }

            let pivot = loop {
                trail_index -= 1;
                let candidate = self.trail[trail_index];
                if seen[candidate.variable] {
                    break candidate;
                }
            };
            seen[pivot.variable] = false;
            open_at_level -= 1;
            if open_at_level == 0 {
                learned[0] = pivot.negate();
                break;
            }
            resolved_variable = pivot.variable;
            clause_id = self.reasons[pivot.variable]
                .expect("a non-UIP propagated literal has a reason clause");
        }

        let backtrack_level = learned
            .iter()
            .skip(1)
            .map(|literal| self.levels[literal.variable])
            .max()
            .unwrap_or(0);
        if learned.len() > 2 {
            let highest = (1..learned.len())
                .max_by_key(|index| self.levels[learned[*index].variable])
                .expect("learned clause has a non-asserting literal");
            learned.swap(1, highest);
        }
        self.activity_increment /= 0.95;
        (learned, backtrack_level)
    }

    fn backtrack(&mut self, level: usize) {
        if self.decision_level() <= level {
            return;
        }
        let trail_length = self.trail_limits[level];
        for literal in self.trail.drain(trail_length..) {
            self.assignments[literal.variable] = Value::Unassigned;
            self.levels[literal.variable] = 0;
            self.reasons[literal.variable] = None;
        }
        self.trail_limits.truncate(level);
        self.propagation_head = self.trail.len();
    }

    fn choose_variable(&self) -> Option<usize> {
        self.assignments
            .iter()
            .enumerate()
            .skip(1)
            .filter(|(_, value)| **value == Value::Unassigned)
            .max_by(|(lhs, _), (rhs, _)| {
                self.activity[*lhs]
                    .partial_cmp(&self.activity[*rhs])
                    .unwrap_or(Ordering::Equal)
                    .then_with(|| rhs.cmp(lhs))
            })
            .map(|(variable, _)| variable)
    }

    fn solve(mut self) -> Option<Vec<bool>> {
        if self.inconsistent || !self.initialize_units() {
            return None;
        }
        loop {
            if let Some(conflict) = self.propagate() {
                if self.decision_level() == 0 {
                    return None;
                }
                let (learned, backtrack_level) = self.analyze(conflict);
                self.backtrack(backtrack_level);
                let asserting = learned[0];
                let clause_id = self
                    .add_clause(learned)
                    .expect("a learned clause cannot be tautological or empty");
                if !self.enqueue(asserting, Some(clause_id)) {
                    return None;
                }
                continue;
            }

            let Some(variable) = self.choose_variable() else {
                return Some(
                    self.assignments
                        .iter()
                        .map(|value| *value == Value::True)
                        .collect(),
                );
            };
            self.trail_limits.push(self.trail.len());
            let decision = SatLiteral {
                variable,
                positive: self.phases[variable],
            };
            assert!(self.enqueue(decision, None));
        }
    }
}

enum ClauseAtom {
    Constant(bool),
    Literal(SatLiteral),
}

fn clause_atom(literal: Literal) -> ClauseAtom {
    if literal.node() == 0 {
        ClauseAtom::Constant(literal.inverted())
    } else {
        ClauseAtom::Literal(SatLiteral {
            variable: literal.node(),
            positive: !literal.inverted(),
        })
    }
}

fn add_aig_clause(solver: &mut Solver, literals: &[Literal]) {
    let mut clause = Vec::with_capacity(literals.len());
    for literal in literals {
        match clause_atom(*literal) {
            ClauseAtom::Constant(true) => return,
            ClauseAtom::Constant(false) => {}
            ClauseAtom::Literal(literal) => clause.push(literal),
        }
    }
    solver.add_clause(clause);
}

pub(crate) fn solve(aig: &Aig, assumptions: &[Literal]) -> Option<Vec<bool>> {
    let mut solver = Solver::new(aig.nodes().len().saturating_sub(1));
    for (node_index, node) in aig.nodes().iter().enumerate().skip(1) {
        if let Node::And(lhs, rhs) = node {
            let output = Literal(node_index << 1);
            add_aig_clause(&mut solver, &[output.negate(), *lhs]);
            add_aig_clause(&mut solver, &[output.negate(), *rhs]);
            add_aig_clause(&mut solver, &[output, lhs.negate(), rhs.negate()]);
        }
    }
    for assumption in assumptions {
        add_aig_clause(&mut solver, &[*assumption]);
    }
    solver.solve()
}

#[cfg(test)]
mod tests {
    use super::{SatLiteral, Solver, solve};
    use crate::aig::{Aig, Variable};

    #[test]
    fn solves_aig_sat_and_unsat_problems() {
        let mut aig = Aig::new();
        let a = aig.variable(Variable::Input("a".into()));
        let b = aig.variable(Variable::Input("b".into()));
        let different = aig.xor(a, b);

        let model = solve(&aig, &[different]).unwrap();
        assert_ne!(Aig::evaluate(a, &model), Aig::evaluate(b, &model));
        assert!(solve(&aig, &[different, different.negate()]).is_none());
    }

    #[test]
    fn learns_that_three_pigeons_do_not_fit_in_two_holes() {
        let mut solver = Solver::new(6);
        let variable = |pigeon: usize, hole: usize| SatLiteral::positive(pigeon * 2 + hole + 1);
        for pigeon in 0..3 {
            solver.add_clause(vec![variable(pigeon, 0), variable(pigeon, 1)]);
        }
        for hole in 0..2 {
            for lhs in 0..3 {
                for rhs in (lhs + 1)..3 {
                    solver.add_clause(vec![
                        variable(lhs, hole).negate(),
                        variable(rhs, hole).negate(),
                    ]);
                }
            }
        }

        assert!(solver.solve().is_none());
    }

    #[test]
    fn agrees_with_exhaustive_search_on_small_random_formulas() {
        let mut random = 0x6a09_e667_f3bc_c909_u64;
        for case in 0..2_000 {
            let variable_count = usize::try_from(next_random(&mut random) % 6).unwrap() + 1;
            let clause_count = usize::try_from(next_random(&mut random) % 16).unwrap();
            let mut clauses = Vec::with_capacity(clause_count);
            let mut solver = Solver::new(variable_count);
            for _ in 0..clause_count {
                let literal_count = usize::try_from(next_random(&mut random) % 4).unwrap();
                let clause = (0..literal_count)
                    .map(|_| SatLiteral {
                        variable: usize::try_from(
                            next_random(&mut random) % u64::try_from(variable_count).unwrap(),
                        )
                        .unwrap()
                            + 1,
                        positive: next_random(&mut random) & 1 != 0,
                    })
                    .collect::<Vec<_>>();
                clauses.push(clause.clone());
                solver.add_clause(clause);
            }

            let expected = (0..(1usize << variable_count)).find(|assignment| {
                clauses.iter().all(|clause| {
                    clause.iter().any(|literal| {
                        let value = assignment & (1 << (literal.variable - 1)) != 0;
                        value == literal.positive
                    })
                })
            });
            let actual = solver.solve();

            assert_eq!(
                actual.is_some(),
                expected.is_some(),
                "SAT mismatch in generated case {case}: {clauses:?}"
            );
            if let Some(model) = actual {
                assert!(clauses.iter().all(|clause| {
                    clause
                        .iter()
                        .any(|literal| model[literal.variable] == literal.positive)
                }));
            }
        }
    }

    fn next_random(state: &mut u64) -> u64 {
        *state ^= *state << 13;
        *state ^= *state >> 7;
        *state ^= *state << 17;
        *state
    }
}
