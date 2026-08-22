//! Native formal-equivalence infrastructure for Struo.
//!
//! The crate deliberately owns the transition-system and proof semantics. It
//! does not round-trip a design through another synthesis IR. Boolean proofs
//! use a structurally hashed AIG and an internal SAT solver, while
//! transformation-aware checks can use compact independently checked
//! certificates.

mod aig;
mod equivalence;
mod retiming;
mod sat;
mod transition;

pub use equivalence::{
    Counterexample, EquivalenceError, EquivalenceResult, EquivalenceStatus, InputFrame,
    prove_sequential_equivalence,
};
pub use retiming::{
    LogicFunction, RetimingCertificate, RetimingDomain, RetimingEdge, RetimingError, RetimingGraph,
    RetimingVertex, derive_retimed_graph, verify_retiming_certificate,
};
pub use transition::{StateBit, StateDomain, TransitionError, TransitionSystem};
