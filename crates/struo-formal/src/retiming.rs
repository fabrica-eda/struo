use std::error::Error;
use std::fmt::{self, Display, Formatter};

use struo_ir::{ActiveLevel, ClockEdge};

/// A small combinational truth table used by the retiming graph.
///
/// Bit zero is the output for all-zero inputs. Input zero is the least
/// significant truth-table index bit. Functions are limited to six inputs so
/// the checker remains simple and representation-independent.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LogicFunction {
    input_count: u8,
    truth_table: u64,
}

impl LogicFunction {
    /// Creates a truth-table function. Structural validation occurs when a
    /// certificate is checked.
    #[must_use]
    pub const fn new(input_count: u8, truth_table: u64) -> Self {
        Self {
            input_count,
            truth_table,
        }
    }

    /// Returns the number of inputs.
    #[must_use]
    pub const fn input_count(self) -> u8 {
        self.input_count
    }

    /// Returns the packed truth table.
    #[must_use]
    pub const fn truth_table(self) -> u64 {
        self.truth_table
    }

    const fn preserves_zero(self) -> bool {
        self.truth_table & 1 == 0
    }
}

/// One combinational vertex in a classical retiming graph.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RetimingVertex {
    name: String,
    function: LogicFunction,
    boundary: bool,
}

impl RetimingVertex {
    /// Creates an internal combinational vertex.
    #[must_use]
    pub fn logic(name: impl Into<String>, function: LogicFunction) -> Self {
        Self {
            name: name.into(),
            function,
            boundary: false,
        }
    }

    /// Creates a primary-input or primary-output boundary vertex.
    #[must_use]
    pub fn boundary(name: impl Into<String>, function: LogicFunction) -> Self {
        Self {
            name: name.into(),
            function,
            boundary: true,
        }
    }

    /// Returns the stable vertex name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }
}

/// One directed retiming edge and its ordered reset values.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RetimingEdge {
    source: usize,
    target: usize,
    reset_values: Vec<bool>,
}

impl RetimingEdge {
    /// Creates an edge. Reset values are ordered from source to target.
    #[must_use]
    pub fn new(source: usize, target: usize, reset_values: Vec<bool>) -> Self {
        Self {
            source,
            target,
            reset_values,
        }
    }

    /// Returns the source vertex index.
    #[must_use]
    pub const fn source(&self) -> usize {
        self.source
    }

    /// Returns the target vertex index.
    #[must_use]
    pub const fn target(&self) -> usize {
        self.target
    }

    /// Returns reset values for registers on the edge.
    #[must_use]
    pub fn reset_values(&self) -> &[bool] {
        &self.reset_values
    }
}

/// Single clock/reset domain covered by one retiming graph.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RetimingDomain {
    clock: String,
    edge: ClockEdge,
    reset: String,
    reset_active: ActiveLevel,
    reset_asynchronous: bool,
}

impl RetimingDomain {
    /// Creates a reset-to-zero retiming domain.
    #[must_use]
    pub fn new(
        clock: impl Into<String>,
        edge: ClockEdge,
        reset: impl Into<String>,
        reset_active: ActiveLevel,
        reset_asynchronous: bool,
    ) -> Self {
        Self {
            clock: clock.into(),
            edge,
            reset: reset.into(),
            reset_active,
            reset_asynchronous,
        }
    }

    /// Returns the clock signal name.
    #[must_use]
    pub fn clock(&self) -> &str {
        &self.clock
    }

    /// Returns the active clock edge.
    #[must_use]
    pub const fn edge(&self) -> ClockEdge {
        self.edge
    }

    /// Returns the reset signal name.
    #[must_use]
    pub fn reset(&self) -> &str {
        &self.reset
    }

    /// Returns the reset assertion level.
    #[must_use]
    pub const fn reset_active(&self) -> ActiveLevel {
        self.reset_active
    }

    /// Returns whether reset is asynchronous.
    #[must_use]
    pub const fn reset_asynchronous(&self) -> bool {
        self.reset_asynchronous
    }
}

/// A combinational topology with registers represented as edge weights.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RetimingGraph {
    domain: RetimingDomain,
    vertices: Vec<RetimingVertex>,
    edges: Vec<RetimingEdge>,
}

impl RetimingGraph {
    /// Creates a graph for certificate generation or checking.
    #[must_use]
    pub const fn new(
        domain: RetimingDomain,
        vertices: Vec<RetimingVertex>,
        edges: Vec<RetimingEdge>,
    ) -> Self {
        Self {
            domain,
            vertices,
            edges,
        }
    }

    /// Returns the clock/reset domain.
    #[must_use]
    pub const fn domain(&self) -> &RetimingDomain {
        &self.domain
    }

    /// Returns graph vertices.
    #[must_use]
    pub fn vertices(&self) -> &[RetimingVertex] {
        &self.vertices
    }

    /// Returns graph edges.
    #[must_use]
    pub fn edges(&self) -> &[RetimingEdge] {
        &self.edges
    }
}

/// Integer retiming labels for every graph vertex.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RetimingCertificate {
    labels: Vec<i32>,
}

impl RetimingCertificate {
    /// Creates a certificate in graph vertex order.
    #[must_use]
    pub const fn new(labels: Vec<i32>) -> Self {
        Self { labels }
    }

    /// Returns all retiming labels.
    #[must_use]
    pub fn labels(&self) -> &[i32] {
        &self.labels
    }
}

/// Checks that `after` is exactly the zero-reset, boundary-preserving retiming
/// described by `certificate`.
///
/// The initial implementation intentionally accepts only reset-to-zero edges
/// and zero-preserving functions crossed by retiming. This makes reset-state
/// correspondence independently checkable without sequential search.
///
/// # Errors
///
/// Returns the first malformed graph or violated retiming invariant.
pub fn verify_retiming_certificate(
    before: &RetimingGraph,
    after: &RetimingGraph,
    certificate: &RetimingCertificate,
) -> Result<(), RetimingError> {
    validate_graph(before)?;
    validate_graph(after)?;
    if before.domain != after.domain {
        return Err(RetimingError::ChangedDomain);
    }
    if before.vertices.len() != after.vertices.len() {
        return Err(RetimingError::VertexCount {
            before: before.vertices.len(),
            after: after.vertices.len(),
        });
    }
    if before.edges.len() != after.edges.len() {
        return Err(RetimingError::EdgeCount {
            before: before.edges.len(),
            after: after.edges.len(),
        });
    }
    if certificate.labels.len() != before.vertices.len() {
        return Err(RetimingError::LabelCount {
            expected: before.vertices.len(),
            actual: certificate.labels.len(),
        });
    }

    for (index, (before_vertex, after_vertex)) in
        before.vertices.iter().zip(&after.vertices).enumerate()
    {
        if before_vertex != after_vertex {
            return Err(RetimingError::ChangedVertex(index));
        }
        let label = certificate.labels[index];
        if before_vertex.boundary && label != 0 {
            return Err(RetimingError::MovedBoundary(index));
        }
        if label != 0 && !before_vertex.function.preserves_zero() {
            return Err(RetimingError::NonZeroPreservingVertex(index));
        }
    }

    for (index, (before_edge, after_edge)) in before.edges.iter().zip(&after.edges).enumerate() {
        if (before_edge.source, before_edge.target) != (after_edge.source, after_edge.target) {
            return Err(RetimingError::ChangedEdge(index));
        }
        if before_edge.reset_values.iter().any(|value| *value)
            || after_edge.reset_values.iter().any(|value| *value)
        {
            return Err(RetimingError::NonZeroReset(index));
        }
        let delta = i64::from(certificate.labels[before_edge.target])
            - i64::from(certificate.labels[before_edge.source]);
        let magnitude = usize::try_from(delta.unsigned_abs())
            .map_err(|_| RetimingError::WeightOverflow { edge: index })?;
        let expected = if delta >= 0 {
            before_edge.reset_values.len().checked_add(magnitude)
        } else {
            before_edge.reset_values.len().checked_sub(magnitude)
        }
        .ok_or(if delta >= 0 {
            RetimingError::WeightOverflow { edge: index }
        } else {
            RetimingError::NegativeWeight { edge: index }
        })?;
        if expected != after_edge.reset_values.len() {
            return Err(RetimingError::WrongWeight {
                edge: index,
                expected,
                actual: after_edge.reset_values.len(),
            });
        }
    }
    Ok(())
}

fn validate_graph(graph: &RetimingGraph) -> Result<(), RetimingError> {
    let vertex_count = graph.vertices.len();
    let mut incoming = vec![0usize; vertex_count];
    for (edge, connection) in graph.edges.iter().enumerate() {
        if connection.source >= vertex_count || connection.target >= vertex_count {
            return Err(RetimingError::InvalidEndpoint(edge));
        }
        incoming[connection.target] += 1;
    }
    for (index, vertex) in graph.vertices.iter().enumerate() {
        if vertex.function.input_count > 6 {
            return Err(RetimingError::WideFunction(index));
        }
        let truth_bits = 1usize << vertex.function.input_count;
        if truth_bits < 64 && vertex.function.truth_table >> truth_bits != 0 {
            return Err(RetimingError::InvalidTruthTable(index));
        }
        if !vertex.boundary && incoming[index] != usize::from(vertex.function.input_count) {
            return Err(RetimingError::FunctionArity {
                vertex: index,
                expected: usize::from(vertex.function.input_count),
                actual: incoming[index],
            });
        }
    }
    Ok(())
}

/// Rejected retiming graph or certificate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RetimingError {
    /// The clock, reset, edge, or reset mode changed.
    ChangedDomain,
    /// The two graphs have different vertex counts.
    VertexCount {
        /// Vertex count before retiming.
        before: usize,
        /// Vertex count after retiming.
        after: usize,
    },
    /// The two graphs have different edge counts.
    EdgeCount {
        /// Edge count before retiming.
        before: usize,
        /// Edge count after retiming.
        after: usize,
    },
    /// The certificate does not contain one label per vertex.
    LabelCount {
        /// Required label count.
        expected: usize,
        /// Provided label count.
        actual: usize,
    },
    /// A combinational function or vertex identity changed.
    ChangedVertex(usize),
    /// An edge endpoint changed.
    ChangedEdge(usize),
    /// An edge references a missing vertex.
    InvalidEndpoint(usize),
    /// Retiming attempted to cross a primary boundary.
    MovedBoundary(usize),
    /// Retiming crossed a function that does not map zero to zero.
    NonZeroPreservingVertex(usize),
    /// A register edge uses an unsupported reset-to-one value.
    NonZeroReset(usize),
    /// A retimed edge would contain a negative number of registers.
    NegativeWeight {
        /// Offending edge index.
        edge: usize,
    },
    /// The retimed edge weight cannot be represented.
    WeightOverflow {
        /// Offending edge index.
        edge: usize,
    },
    /// The output graph has a register count inconsistent with its labels.
    WrongWeight {
        /// Offending edge index.
        edge: usize,
        /// Weight implied by the certificate.
        expected: usize,
        /// Weight present in the output graph.
        actual: usize,
    },
    /// Truth tables wider than six inputs are unsupported.
    WideFunction(usize),
    /// A truth table sets bits outside its declared input width.
    InvalidTruthTable(usize),
    /// A vertex arity does not match its incoming edges.
    FunctionArity {
        /// Offending vertex index.
        vertex: usize,
        /// Declared function arity.
        expected: usize,
        /// Incoming graph edge count.
        actual: usize,
    },
}

impl Display for RetimingError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::ChangedDomain => formatter.write_str("retiming domain changed"),
            Self::VertexCount { before, after } => {
                write!(formatter, "vertex count changed from {before} to {after}")
            }
            Self::EdgeCount { before, after } => {
                write!(formatter, "edge count changed from {before} to {after}")
            }
            Self::LabelCount { expected, actual } => {
                write!(formatter, "expected {expected} labels, found {actual}")
            }
            Self::ChangedVertex(vertex) => write!(formatter, "vertex {vertex} changed"),
            Self::ChangedEdge(edge) => write!(formatter, "edge {edge} changed endpoints"),
            Self::InvalidEndpoint(edge) => write!(formatter, "edge {edge} has an invalid endpoint"),
            Self::MovedBoundary(vertex) => {
                write!(formatter, "boundary vertex {vertex} has a non-zero label")
            }
            Self::NonZeroPreservingVertex(vertex) => {
                write!(
                    formatter,
                    "retimed vertex {vertex} does not preserve reset zero"
                )
            }
            Self::NonZeroReset(edge) => {
                write!(formatter, "edge {edge} contains a reset-to-one register")
            }
            Self::NegativeWeight { edge } => {
                write!(formatter, "retiming makes edge {edge} weight negative")
            }
            Self::WeightOverflow { edge } => {
                write!(formatter, "retiming overflows edge {edge} weight")
            }
            Self::WrongWeight {
                edge,
                expected,
                actual,
            } => write!(
                formatter,
                "edge {edge} should contain {expected} registers, found {actual}"
            ),
            Self::WideFunction(vertex) => {
                write!(formatter, "vertex {vertex} has more than six inputs")
            }
            Self::InvalidTruthTable(vertex) => {
                write!(
                    formatter,
                    "vertex {vertex} has truth bits outside its width"
                )
            }
            Self::FunctionArity {
                vertex,
                expected,
                actual,
            } => write!(
                formatter,
                "vertex {vertex} expects {expected} inputs, found {actual} incoming edges"
            ),
        }
    }
}

impl Error for RetimingError {}

#[cfg(test)]
mod tests {
    use struo_ir::{ActiveLevel, ClockEdge};

    use super::{
        LogicFunction, RetimingCertificate, RetimingDomain, RetimingEdge, RetimingError,
        RetimingGraph, RetimingVertex, verify_retiming_certificate,
    };

    #[test]
    fn checks_forward_retiming_across_zero_preserving_logic() {
        let vertices = vec![
            RetimingVertex::boundary("a", LogicFunction::new(0, 0)),
            RetimingVertex::boundary("b", LogicFunction::new(0, 0)),
            RetimingVertex::logic("and", LogicFunction::new(2, 0b1000)),
            RetimingVertex::boundary("y", LogicFunction::new(1, 0b10)),
        ];
        let before = RetimingGraph::new(
            domain(),
            vertices.clone(),
            vec![
                RetimingEdge::new(0, 2, vec![false]),
                RetimingEdge::new(1, 2, vec![false]),
                RetimingEdge::new(2, 3, vec![]),
            ],
        );
        let after = RetimingGraph::new(
            domain(),
            vertices,
            vec![
                RetimingEdge::new(0, 2, vec![]),
                RetimingEdge::new(1, 2, vec![]),
                RetimingEdge::new(2, 3, vec![false]),
            ],
        );

        assert_eq!(
            verify_retiming_certificate(
                &before,
                &after,
                &RetimingCertificate::new(vec![0, 0, -1, 0]),
            ),
            Ok(())
        );
    }

    #[test]
    fn rejects_illegal_weight_and_reset_transformations() {
        let vertices = vec![
            RetimingVertex::boundary("a", LogicFunction::new(0, 0)),
            RetimingVertex::logic("not", LogicFunction::new(1, 0b01)),
            RetimingVertex::boundary("y", LogicFunction::new(1, 0b10)),
        ];
        let before = RetimingGraph::new(
            domain(),
            vertices.clone(),
            vec![
                RetimingEdge::new(0, 1, vec![false]),
                RetimingEdge::new(1, 2, vec![]),
            ],
        );
        let after = RetimingGraph::new(
            domain(),
            vertices,
            vec![
                RetimingEdge::new(0, 1, vec![]),
                RetimingEdge::new(1, 2, vec![false]),
            ],
        );

        assert_eq!(
            verify_retiming_certificate(&before, &after, &RetimingCertificate::new(vec![0, -1, 0]),),
            Err(RetimingError::NonZeroPreservingVertex(1))
        );
    }

    fn domain() -> RetimingDomain {
        RetimingDomain::new("clock", ClockEdge::Rising, "reset", ActiveLevel::High, true)
    }
}
