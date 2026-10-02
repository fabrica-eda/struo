mod arrays;
mod comparisons;
mod loops;
mod members;
mod types;

use comparisons::PreparedCaseTarget;
use types::{repair_expression_signedness, variable_signedness};

use std::collections::{BTreeMap, HashMap, HashSet};

use struo_rtl::{
    BinaryOp, BitWidth, ClockEdge, Constant, Design, Enable, ExprId, ExprKind, Memory, MemoryPort,
    MemoryStyle, Module as RtlModule, Polarity, Port, PortDirection, Register, Reset, ResetMode,
    SignalId, SignalSlice, StateDomain, UnaryOp, ValueType,
};
use veryl_analyzer::ir::{
    ArrayLiteralItem, AssignDestination, CasePattern, CaseStatement, Component, Comptime,
    Declaration, Expression, Factor, FfDeclaration, IfResetStatement, InstDeclaration, Ir, Module,
    Op, Statement, Type, TypeKind, ValueVariant, VarId, VarIndex, VarKind, VarSelect, VarSelectOp,
};
use veryl_analyzer::{attribute::Attribute as VerylAttribute, attribute_table};
use veryl_parser::resource_table::StrId;

use crate::{ImportError, MemoryInferencePolicy, resolve_name};

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
struct SignalKey {
    id: VarId,
    index: Vec<usize>,
}

struct PreparedSelect {
    select: VarSelect,
    offset: Option<(ExprId, ExprId)>,
    domain: Option<veryl_analyzer::ir::MemberSelectDomain>,
}

struct PreparedDestination {
    elements: Vec<(SignalKey, Option<LoweredExpr>)>,
    packed: PreparedSelect,
    width: u32,
}

type Env = HashMap<SignalKey, LoweredExpr>;
type FunctionOutputs = Vec<(Vec<AssignDestination>, LoweredExpr)>;
type FunctionResult = (Option<LoweredExpr>, FunctionOutputs);

#[derive(Clone, Debug, Default)]
struct DrivenBits {
    // Static packed writes are kept as normalized, non-overlapping ranges.
    // SignalKey intentionally continues to identify the unpacked element so
    // expression environments can compose partial writes over a whole value.
    ranges: BTreeMap<SignalKey, Vec<BitRange>>,
}

#[derive(Clone, Copy, Debug)]
struct BitRange {
    start: u32,
    end: u32,
}

impl DrivenBits {
    fn insert_range(&mut self, key: SignalKey, lsb: u32, width: u32) {
        let ranges = self.ranges.entry(key).or_default();
        let mut merged = BitRange {
            start: lsb,
            end: lsb + width,
        };
        let mut index = 0;
        while index < ranges.len() && ranges[index].end < merged.start {
            index += 1;
        }
        while index < ranges.len() && ranges[index].start <= merged.end {
            let range = ranges.remove(index);
            merged.start = merged.start.min(range.start);
            merged.end = merged.end.max(range.end);
        }
        ranges.insert(index, merged);
    }

    fn extend(&mut self, other: Self) {
        for (key, ranges) in other.ranges {
            for range in ranges {
                self.insert_range(key.clone(), range.start, range.end - range.start);
            }
        }
    }

    fn extend_from(&mut self, other: &Self) {
        for (key, ranges) in &other.ranges {
            for range in ranges {
                self.insert_range(key.clone(), range.start, range.end - range.start);
            }
        }
    }

    fn keys(&self) -> impl Iterator<Item = &SignalKey> {
        self.ranges.keys()
    }

    fn contains_key(&self, key: &SignalKey) -> bool {
        self.ranges.contains_key(key)
    }

    fn first_overlap(&self, other: &Self) -> Option<&SignalKey> {
        self.ranges.iter().find_map(|(key, ranges)| {
            let other_ranges = other.ranges.get(key)?;
            ranges
                .iter()
                .any(|range| {
                    other_ranges
                        .iter()
                        .any(|other| range.start < other.end && other.start < range.end)
                })
                .then_some(key)
        })
    }

    fn ranges(&self, key: &SignalKey) -> Vec<(u32, u32)> {
        self.ranges.get(key).map_or_else(Vec::new, |ranges| {
            ranges
                .iter()
                .map(|range| (range.start, range.end - range.start))
                .collect()
        })
    }
}

#[derive(Clone, Copy)]
struct LoweredExpr {
    id: ExprId,
    width: u32,
    signed: bool,
}

struct LoweredFf {
    clock: SignalId,
    edge: ClockEdge,
    initial: Env,
    next: Env,
    reset_values: Option<Env>,
    reset_control: Option<(SignalId, ResetMode, Polarity)>,
    changed: DrivenBits,
}

struct CaseMatchTree {
    any_match: LoweredExpr,
    kind: CaseMatchTreeKind,
}

enum CaseMatchTreeKind {
    Arm(usize),
    Branch {
        left: Box<CaseMatchTree>,
        right: Box<CaseMatchTree>,
    },
}

#[derive(Clone, Copy)]
enum LoweredArrayIndex {
    Static(usize),
    Dynamic(LoweredExpr),
}

struct ModuleLowerer<'a> {
    source: &'a Module,
    call_depth: usize,
    rtl: RtlModule,
    signals: HashMap<SignalKey, SignalId>,
    signal_order: Vec<SignalKey>,
    widths: HashMap<SignalKey, u32>,
    signed: HashMap<SignalKey, bool>,
    inferred_memories: HashSet<VarId>,
    memory_policies: HashMap<VarId, MemoryInferencePolicy>,
}

#[derive(Clone)]
struct MemoryWritePattern {
    clock: SignalKey,
    edge: ClockEdge,
    address: Expression,
    data: Expression,
    enable: Vec<Expression>,
}

#[derive(Clone)]
struct MemoryReadPattern {
    clock: SignalKey,
    edge: ClockEdge,
    address: Expression,
    data: VarId,
    enable: Vec<Expression>,
}

#[derive(Clone)]
struct AsyncMemoryReadPattern {
    address: Expression,
    data: VarId,
    enable: Vec<Expression>,
}

#[derive(Default)]
struct PartialMemoryPattern {
    writes: Vec<MemoryWritePattern>,
    reads: Vec<MemoryReadPattern>,
    async_reads: Vec<AsyncMemoryReadPattern>,
}

/// Lowers analyzed Veryl AIR into Struo RTL without generated Verilog.
///
/// The current semantic boundary supports scalar packed variables, statically
/// and dynamically indexed unpacked arrays, recursively flattened module instances,
/// analyzer-expanded interface/modport connections, combinational and
/// sequential assignments, compile-time constants, static packed selects,
/// dynamic packed bit selects, packed struct constructors and member accesses,
/// conditionals, case statements, concatenations, arithmetic, comparisons,
/// shifts, and reset branches. Unsupported AIR is rejected rather than
/// silently discarded.
///
/// # Errors
///
/// Returns an error for a missing top module, unsupported analyzer constructs,
/// unresolved widths, or invalid resulting RTL.
pub fn lower_analyzed_ir(ir: &Ir, top: &str) -> Result<Design, ImportError> {
    let top_id = veryl_parser::resource_table::insert_str(top);
    let source = ir
        .components
        .iter()
        .find_map(|component| match component {
            Component::Module(module) if module.name == top_id => Some(module),
            _ => None,
        })
        .ok_or_else(|| ImportError::MissingTop(top.into()))?;

    let mut lowerer = ModuleLowerer::new(source)?;
    lowerer.lower_declarations()?;
    lowerer.rtl.validate()?;
    let mut design = Design::new(top);
    design.add_module(lowerer.rtl);
    design.validate()?;
    Ok(design)
}

impl<'a> ModuleLowerer<'a> {
    fn new(source: &'a Module) -> Result<Self, ImportError> {
        let mut rtl = RtlModule::new(resolve_name(source.name)?);
        let mut signals = HashMap::new();
        let mut signal_order = Vec::new();
        let mut widths = HashMap::new();
        let mut signed = HashMap::new();
        let memory_policies = memory_inference_policies(source)?;
        let inferred_memories = memory_candidates(source, &memory_policies);

        let mut ports = source.ports.iter().collect::<Vec<_>>();
        ports.sort_by_key(|(path, _)| path.to_string());
        for (path, id) in ports {
            let variable = source
                .variables
                .get(id)
                .ok_or_else(|| ImportError::MissingVariable(path.to_string()))?;
            if variable.affiliation == veryl_analyzer::symbol::Affiliation::Function {
                continue;
            }
            let direction = match variable.kind {
                VarKind::Input => PortDirection::Input,
                VarKind::Output => PortDirection::Output,
                VarKind::Inout => PortDirection::Inout,
                _ => return Err(ImportError::NonPort(path.to_string())),
            };
            let base_name = path.to_string();
            let r#type = value_type(&variable.r#type, &base_name)?;
            for index in array_indices(&variable.r#type, &base_name)? {
                let key = SignalKey { id: *id, index };
                let signal = rtl.add_port(Port {
                    name: indexed_name(&base_name, &key.index),
                    direction,
                    r#type,
                });
                signals.insert(key.clone(), signal);
                signal_order.push(key.clone());
                widths.insert(key.clone(), r#type.width.get());
                signed.insert(key, r#type.signed);
            }
        }

        let mut internals = source
            .variables
            .values()
            .filter(|variable| {
                matches!(variable.kind, VarKind::Variable | VarKind::Let)
                    && variable.affiliation != veryl_analyzer::symbol::Affiliation::Function
                    && !inferred_memories.contains(&variable.id)
            })
            .collect::<Vec<_>>();
        internals.sort_by_key(|variable| (variable.path.to_string(), variable.id));
        let mut names = rtl
            .signals()
            .iter()
            .map(|signal| signal.name().to_owned())
            .collect::<HashSet<_>>();
        for variable in internals {
            let base_name = variable.path.to_string();
            let r#type = value_type(&variable.r#type, &base_name)?;
            for index in array_indices(&variable.r#type, &base_name)? {
                let key = SignalKey {
                    id: variable.id,
                    index,
                };
                let base = indexed_name(&base_name, &key.index);
                let mut name = base.clone();
                let mut suffix = 0usize;
                while !names.insert(name.clone()) {
                    suffix += 1;
                    name = format!("{base}$scope{suffix}");
                }
                let signal = rtl.add_signal(name, r#type);
                signals.insert(key.clone(), signal);
                signal_order.push(key.clone());
                widths.insert(key.clone(), r#type.width.get());
                signed.insert(key, r#type.signed);
            }
        }

        Ok(Self {
            source,
            call_depth: 0,
            rtl,
            signals,
            signal_order,
            widths,
            signed,
            inferred_memories,
            memory_policies,
        })
    }

    fn infer_memories(&mut self) -> Result<(), ImportError> {
        let mut candidates = self.inferred_memories.iter().copied().collect::<Vec<_>>();
        if candidates.is_empty() {
            return Ok(());
        }
        candidates.sort_by_key(|memory| self.variable_name(*memory));

        let mut patterns = self.collect_memory_patterns(&self.inferred_memories)?;
        for memory_id in candidates {
            if self.source.variables[&memory_id].affiliation
                == veryl_analyzer::symbol::Affiliation::AlwaysFf
            {
                return Err(self.memory_inference_failure(memory_id,
                    "blocking always_ff-local accesses cannot use the synchronous read-first memory template"));
            }
            let mut pattern = patterns.remove(&memory_id).unwrap_or_default();
            if self.memory_policy(memory_id) == MemoryInferencePolicy::Distributed {
                if let Err(error) = self.lower_distributed_memory(memory_id, pattern) {
                    return Err(self.requirement_failure(memory_id, error));
                }
                continue;
            }
            if pattern.writes.is_empty() || pattern.reads.is_empty() {
                let missing = if pattern.writes.is_empty() && pattern.reads.is_empty() {
                    "no supported synchronous read or write port was found"
                } else if pattern.writes.is_empty() {
                    "no supported synchronous write port was found"
                } else {
                    "no supported synchronous read port was found"
                };
                return Err(self.memory_inference_failure(memory_id, missing));
            }
            if pattern.writes.len() > 2 || pattern.reads.len() > 2 {
                return Err(self.memory_inference_failure(
                    memory_id,
                    "more than two read/write ports are not supported",
                ));
            }
            let mut ports = Vec::new();
            for write in pattern.writes {
                let Some(index) = pattern
                    .reads
                    .iter()
                    .position(|read| read.clock == write.clock && read.edge == write.edge)
                else {
                    return Err(self.memory_inference_failure(
                        memory_id,
                        "each write port requires a read port on the same clock edge",
                    ));
                };
                ports.push((write, pattern.reads.remove(index)));
            }
            if !pattern.reads.is_empty() {
                return Err(self.memory_inference_failure(
                    memory_id,
                    "each read port requires a write port on the same clock edge",
                ));
            }
            if let Err(error) = self.lower_inferred_memory(memory_id, ports) {
                return Err(self.requirement_failure(memory_id, error));
            }
        }
        Ok(())
    }

    fn collect_memory_patterns(
        &self,
        candidates: &HashSet<VarId>,
    ) -> Result<HashMap<VarId, PartialMemoryPattern>, ImportError> {
        let mut patterns = HashMap::<VarId, PartialMemoryPattern>::new();
        for declaration in &self.source.declarations {
            match declaration {
                Declaration::Ff(ff) => {
                    let (clock, edge) = self.source_clock(ff)?;
                    for statement in &ff.statements {
                        for pattern in memory_statement_patterns(statement, candidates) {
                            match pattern {
                                MemoryStatementPattern::Write {
                                    memory,
                                    address,
                                    data,
                                    enable,
                                } => {
                                    patterns.entry(memory).or_default().writes.push(
                                        MemoryWritePattern {
                                            clock: clock.clone(),
                                            edge,
                                            address,
                                            data,
                                            enable,
                                        },
                                    );
                                }
                                MemoryStatementPattern::Read {
                                    memory,
                                    address,
                                    data,
                                    enable,
                                } => {
                                    patterns.entry(memory).or_default().reads.push(
                                        MemoryReadPattern {
                                            clock: clock.clone(),
                                            edge,
                                            address,
                                            data,
                                            enable,
                                        },
                                    );
                                }
                            }
                        }
                    }
                }
                Declaration::Comb(comb) => {
                    for statement in &comb.statements {
                        for pattern in memory_statement_patterns(statement, candidates) {
                            let MemoryStatementPattern::Read {
                                memory,
                                address,
                                data,
                                enable,
                            } = pattern
                            else {
                                continue;
                            };
                            patterns.entry(memory).or_default().async_reads.push(
                                AsyncMemoryReadPattern {
                                    address,
                                    data,
                                    enable,
                                },
                            );
                        }
                    }
                }
                _ => {}
            }
        }
        Ok(patterns)
    }

    fn lower_inferred_memory(
        &mut self,
        memory_id: VarId,
        ports: Vec<(MemoryWritePattern, MemoryReadPattern)>,
    ) -> Result<(), ImportError> {
        let variable = &self.source.variables[&memory_id];
        if variable.r#type.array.dims() != 1 {
            return Err(self.memory_inference_failure(
                memory_id,
                "the array must have exactly one unpacked dimension",
            ));
        }
        let depth = variable
            .r#type
            .total_array()
            .ok_or_else(|| ImportError::NonConcreteWidth(self.variable_name(memory_id)))?;
        let depth = u32::try_from(depth)
            .map_err(|_| ImportError::WidthTooLarge(self.variable_name(memory_id)))?;
        if depth == 0 {
            return Err(self.memory_inference_failure(memory_id, "the array has zero depth"));
        }
        let word = value_type(&variable.r#type, &self.variable_name(memory_id))?;
        let address_width = (u32::BITS - (depth - 1).leading_zeros()).max(1);
        let mut ports = ports
            .into_iter()
            .enumerate()
            .map(|(index, (write, read))| {
                self.lower_memory_port(memory_id, index, word, address_width, write, read)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let primary = ports.remove(0);
        let second_port = ports.pop();
        self.rtl.add_memory(Memory {
            name: self.variable_name(memory_id),
            word,
            depth,
            style: if self.memory_policy(memory_id) == MemoryInferencePolicy::Block {
                MemoryStyle::Block
            } else {
                MemoryStyle::Auto
            },
            read_latency: 1,
            read_address: primary.read_address,
            read_data: primary.read_data,
            read_enable: primary.read_enable,
            write_address: primary.write_address,
            write_data: primary.write_data,
            write_enable: primary.write_enable,
            clock: primary.clock,
            edge: primary.edge,
            second_port,
        });
        Ok(())
    }

    fn lower_distributed_memory(
        &mut self,
        memory_id: VarId,
        mut pattern: PartialMemoryPattern,
    ) -> Result<(), ImportError> {
        if pattern.writes.len() != 1 || pattern.async_reads.len() != 1 || !pattern.reads.is_empty()
        {
            return Err(self.memory_inference_failure(
                memory_id,
                "distributed RAM requires exactly one synchronous write and one asynchronous read port",
            ));
        }
        let variable = &self.source.variables[&memory_id];
        if variable.r#type.array.dims() != 1 {
            return Err(self.memory_inference_failure(
                memory_id,
                "the array must have exactly one unpacked dimension",
            ));
        }
        let depth = variable
            .r#type
            .total_array()
            .ok_or_else(|| ImportError::NonConcreteWidth(self.variable_name(memory_id)))?;
        let depth = u32::try_from(depth)
            .map_err(|_| ImportError::WidthTooLarge(self.variable_name(memory_id)))?;
        if depth == 0 {
            return Err(self.memory_inference_failure(memory_id, "the array has zero depth"));
        }
        let word = value_type(&variable.r#type, &self.variable_name(memory_id))?;
        let address_width = (u32::BITS - (depth - 1).leading_zeros()).max(1);
        let write = pattern.writes.remove(0);
        let read = pattern.async_reads.remove(0);
        if !read.enable.is_empty() {
            return Err(self.memory_inference_failure(
                memory_id,
                "distributed RAM asynchronous reads cannot have an enable",
            ));
        }
        let env = self.read_env()?;
        let read_address = self.lower_expression(&read.address, &env)?;
        let read_address = self.resize(read_address, address_width, false)?;
        let write_address = self.lower_expression(&write.address, &env)?;
        let write_address = self.resize(write_address, address_width, false)?;
        let write_data = self.lower_expression(&write.data, &env)?;
        let write_data = self.resize(write_data, word.width.get(), word.signed)?;
        let write_enable = self.lower_memory_enable(write.enable, &env)?;
        let write_enable = self.materialize_memory_enable(memory_id, "write_a", write_enable)?;
        self.rtl.add_memory(Memory {
            name: self.variable_name(memory_id),
            word,
            depth,
            style: MemoryStyle::Distributed,
            read_latency: 0,
            read_address: read_address.id,
            read_data: self.signal(&SignalKey {
                id: read.data,
                index: Vec::new(),
            })?,
            read_enable: None,
            write_address: write_address.id,
            write_data: write_data.id,
            write_enable: Enable {
                signal: write_enable,
                polarity: Polarity::ActiveHigh,
            },
            clock: self.signal(&write.clock)?,
            edge: write.edge,
            second_port: None,
        });
        Ok(())
    }

    fn lower_memory_port(
        &mut self,
        memory_id: VarId,
        port_index: usize,
        word: ValueType,
        address_width: u32,
        write: MemoryWritePattern,
        read: MemoryReadPattern,
    ) -> Result<MemoryPort, ImportError> {
        let env = self.read_env()?;
        let read_address = self.lower_expression(&read.address, &env)?;
        let read_address = self.resize(read_address, address_width, false)?;
        let write_address = self.lower_expression(&write.address, &env)?;
        let write_address = self.resize(write_address, address_width, false)?;
        let write_data = self.lower_expression(&write.data, &env)?;
        let write_data = self.resize(write_data, word.width.get(), word.signed)?;
        let write_enable = self.lower_memory_enable(write.enable, &env)?;
        let read_enable = (!read.enable.is_empty())
            .then(|| self.lower_memory_enable(read.enable, &env))
            .transpose()?;
        let port_name = if port_index == 0 { "a" } else { "b" };
        let write_enable =
            self.materialize_memory_enable(memory_id, &format!("write_{port_name}"), write_enable)?;
        let read_enable = read_enable
            .map(|enable| {
                self.materialize_memory_enable(memory_id, &format!("read_{port_name}"), enable)
            })
            .transpose()?;
        Ok(MemoryPort {
            read_address: read_address.id,
            read_data: self.signal(&SignalKey {
                id: read.data,
                index: Vec::new(),
            })?,
            read_enable: read_enable.map(|signal| Enable {
                signal,
                polarity: Polarity::ActiveHigh,
            }),
            write_address: write_address.id,
            write_data: write_data.id,
            write_enable: Enable {
                signal: write_enable,
                polarity: Polarity::ActiveHigh,
            },
            clock: self.signal(&write.clock)?,
            edge: write.edge,
        })
    }

    fn memory_policy(&self, memory: VarId) -> MemoryInferencePolicy {
        self.memory_policies
            .get(&memory)
            .copied()
            .unwrap_or_default()
    }

    fn memory_inference_failure(&self, memory: VarId, reason: impl Into<String>) -> ImportError {
        let memory_name = self.variable_name(memory);
        let reason = reason.into();
        if matches!(
            self.memory_policy(memory),
            MemoryInferencePolicy::Required
                | MemoryInferencePolicy::Block
                | MemoryInferencePolicy::Distributed
        ) {
            ImportError::RequiredMemoryInferenceFailed {
                memory: memory_name,
                reason,
            }
        } else {
            ImportError::UnsupportedBehavior(format!(
                "unpacked array {memory_name} cannot be inferred as a block memory: {reason}"
            ))
        }
    }

    fn requirement_failure(&self, memory: VarId, error: ImportError) -> ImportError {
        if !matches!(
            self.memory_policy(memory),
            MemoryInferencePolicy::Required
                | MemoryInferencePolicy::Block
                | MemoryInferencePolicy::Distributed
        ) || matches!(error, ImportError::RequiredMemoryInferenceFailed { .. })
        {
            error
        } else {
            ImportError::RequiredMemoryInferenceFailed {
                memory: self.variable_name(memory),
                reason: error.to_string(),
            }
        }
    }

    fn lower_memory_enable(
        &mut self,
        enable: Vec<Expression>,
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let mut result = self.constant(1, 1);
        for enable in enable {
            let enable = self.lower_expression(&enable, env)?;
            let enable = self.boolean(enable)?;
            result = LoweredExpr {
                id: self.rtl.binary(BinaryOp::And, result.id, enable.id)?,
                width: 1,
                signed: false,
            };
        }
        Ok(result)
    }

    fn source_clock(&self, ff: &FfDeclaration) -> Result<(SignalKey, ClockEdge), ImportError> {
        if !ff.clock.select.is_empty() {
            return Err(ImportError::UnsupportedBehavior("selected clocks".into()));
        }
        let clock = self.key_from_index(ff.clock.id, &ff.clock.index)?;
        let edge = match self.variable_type(ff.clock.id)?.kind {
            TypeKind::ClockNegedge => ClockEdge::Falling,
            TypeKind::Clock | TypeKind::ClockPosedge => ClockEdge::Rising,
            _ => {
                return Err(ImportError::UnsupportedBehavior(
                    "always_ff clock is not a clock type".into(),
                ));
            }
        };
        Ok((clock, edge))
    }

    fn materialize_memory_enable(
        &mut self,
        memory: VarId,
        port: &str,
        value: LoweredExpr,
    ) -> Result<SignalId, ImportError> {
        let signal = self.rtl.add_signal(
            format!("__struo_{memory}_{port}_enable"),
            ValueType {
                width: BitWidth::new(1)?,
                signed: false,
                state: StateDomain::TwoState,
            },
        );
        self.rtl.assign(self.rtl.whole(signal)?, value.id)?;
        Ok(signal)
    }

    fn ff_owners(&self) -> Result<BTreeMap<SignalKey, usize>, ImportError> {
        let mut owners = BTreeMap::new();
        for declaration in &self.source.declarations {
            let Declaration::Ff(ff) = declaration else {
                continue;
            };
            let mut changed = DrivenBits::default();
            for statement in &ff.statements {
                if memory_statement_patterns(statement, &self.inferred_memories).is_empty() {
                    self.collect_statement_destinations(statement, &mut changed)?;
                }
            }
            for key in changed.keys() {
                *owners.entry(key.clone()).or_default() += 1;
            }
        }
        Ok(owners)
    }

    fn collect_statement_destinations(
        &self,
        statement: &Statement,
        changed: &mut DrivenBits,
    ) -> Result<(), ImportError> {
        match statement {
            Statement::Assign(assign) => {
                for destination in &assign.dst {
                    for key in self.destination_candidates(destination)? {
                        let (lsb, width) = driven_select(&destination.select, self.width(&key)?)?;
                        changed.insert_range(key, lsb, width);
                    }
                }
            }
            Statement::If(branch) => {
                for statement in branch.true_side.iter().chain(&branch.false_side) {
                    self.collect_statement_destinations(statement, changed)?;
                }
            }
            Statement::IfReset(branch) => {
                for statement in branch.true_side.iter().chain(&branch.false_side) {
                    self.collect_statement_destinations(statement, changed)?;
                }
            }
            Statement::Case(case_statement) => {
                for statement in &case_statement.default {
                    self.collect_statement_destinations(statement, changed)?;
                }
                for arm in &case_statement.arms {
                    for statement in &arm.body {
                        self.collect_statement_destinations(statement, changed)?;
                    }
                }
            }
            Statement::Null
            | Statement::For(_)
            | Statement::FunctionCall(_)
            | Statement::SystemFunctionCall(_)
            | Statement::TbMethodCall(_)
            | Statement::Break
            | Statement::Unsupported(_) => {}
        }
        Ok(())
    }

    fn destination_candidates(
        &self,
        destination: &AssignDestination,
    ) -> Result<Vec<SignalKey>, ImportError> {
        if has_dynamic_array_index(&destination.index) {
            self.array_candidate_keys(destination.id, &destination.index)
        } else {
            Ok(vec![self.destination_key(destination)?])
        }
    }

    fn lower_declarations(&mut self) -> Result<(), ImportError> {
        self.infer_memories()?;
        // Discover multi-owner aggregates without lowering expressions.  The
        // real pass can then emit every ordinary register at its original
        // declaration position, preserving stable IDs and mapped QoR.
        let owners = self.ff_owners()?;
        let mut driven_comb = DrivenBits::default();
        let mut driven_ff = DrivenBits::default();
        for declaration in &self.source.declarations {
            match declaration {
                Declaration::Comb(comb) => {
                    let initial = self.read_env()?;
                    let mut env = initial.clone();
                    let mut changed = DrivenBits::default();
                    for statement in &comb.statements {
                        if !memory_statement_patterns(statement, &self.inferred_memories).is_empty()
                        {
                            continue;
                        }
                        // Blocking assignments make each following statement
                        // observe the writes already made in this block.  Keep
                        // the memory-pattern filter local without bypassing
                        // the snapshot semantics used by `lower_statements`.
                        let snapshot = env.clone();
                        changed
                            .extend(self.lower_statement(statement, &snapshot, &mut env, false)?);
                    }
                    if let Some(key) = driven_comb.first_overlap(&changed) {
                        return Err(ImportError::UnsupportedBehavior(format!(
                            "multiple procedural drivers for {}",
                            self.signal_name(key)
                        )));
                    }
                    for key in changed.keys() {
                        if driven_ff.contains_key(key) {
                            return Err(ImportError::UnsupportedBehavior(format!(
                                "multiple procedural drivers for {}",
                                self.signal_name(key)
                            )));
                        }
                        let signal = self.signal(key)?;
                        let value = env[key];
                        // The environment holds a whole value so subsequent blocking
                        // reads see prior writes. Only the written ranges belong to
                        // this process; emitting the untouched bits would introduce
                        // extra drivers and artificial combinational feedback.
                        for (lsb, width) in changed.ranges(key) {
                            let width = BitWidth::new(width)?;
                            let target = self.rtl.slice(signal, lsb, width)?;
                            let value = if lsb == 0 && width.get() == value.width {
                                value.id
                            } else {
                                self.rtl.expression_slice(value.id, lsb, width)?
                            };
                            self.rtl.assign(target, value)?;
                        }
                    }
                    driven_comb.extend(changed);
                }
                Declaration::Ff(ff) => {
                    let block = self.lower_ff(ff)?;
                    for key in block.changed.keys() {
                        if driven_comb.contains_key(key) {
                            return Err(ImportError::UnsupportedBehavior(format!(
                                "multiple procedural drivers for {}",
                                self.signal_name(key)
                            )));
                        }
                    }
                    if let Some(key) = driven_ff.first_overlap(&block.changed) {
                        return Err(ImportError::UnsupportedBehavior(format!(
                            "multiple procedural drivers for {}",
                            self.signal_name(key)
                        )));
                    }
                    driven_ff.extend_from(&block.changed);
                    self.materialize_ff(&block, &owners)?;
                }
                Declaration::Null => {}
                Declaration::Inst(instance) => self.lower_instance(instance)?,
                Declaration::External(_) => {
                    return Err(ImportError::UnsupportedBehavior(
                        "external components are not synthesizable".into(),
                    ));
                }
                Declaration::Initial(_) | Declaration::Final(_) => {
                    return Err(ImportError::UnsupportedBehavior(
                        "initial and final blocks are simulation-only".into(),
                    ));
                }
                Declaration::Unsupported(_) => {
                    return Err(ImportError::UnsupportedBehavior(
                        "analyzer marked a declaration unsupported".into(),
                    ));
                }
            }
        }
        Ok(())
    }

    fn lower_instance(&mut self, instance: &InstDeclaration) -> Result<(), ImportError> {
        // Instance outputs are implicit continuous assignments (IEEE 1800
        // 10.2): every selected destination must have constant addressing.
        for destination in instance.outputs.iter().flat_map(|output| &output.dst) {
            let dynamic = destination
                .index
                .0
                .iter()
                .chain(&destination.select.0)
                .chain(destination.select.1.iter().map(|(_, bound)| bound))
                .any(|index| !index.comptime().is_const && evaluated_u64(index).is_none());
            if dynamic {
                return Err(ImportError::AnalysisFailed(
                    "instance output destination requires constant indices and bounds".into(),
                ));
            }
        }
        let Component::Module(child_source) = instance.component.as_ref() else {
            return Err(ImportError::UnsupportedBehavior(
                "only synthesizable module instances can be flattened".into(),
            ));
        };
        let mut child = ModuleLowerer::new(child_source)?;
        child.lower_declarations()?;
        child.rtl.validate()?;

        let prefix = instance
            .hierarchy
            .iter()
            .map(ToString::to_string)
            .chain(std::iter::once(resolve_name(instance.name)?))
            .collect::<Vec<_>>()
            .join(".");
        let inline_signals = self.inline_module(&child.rtl, &prefix)?;
        let parent_env = self.read_env()?;
        self.lower_instance_inputs(instance, &child, &inline_signals, &parent_env)?;
        self.lower_instance_outputs(instance, &child, &inline_signals)?;
        Ok(())
    }

    fn lower_instance_inputs(
        &mut self,
        instance: &InstDeclaration,
        child: &ModuleLowerer<'_>,
        inline_signals: &HashMap<SignalId, SignalId>,
        parent_env: &Env,
    ) -> Result<(), ImportError> {
        let mut input_elements = HashMap::new();

        for input in &instance.inputs {
            if let Some(parent_id) = input.single().and_then(whole_array_variable) {
                let child_keys = child.keys_for_id(input.id);
                let parent_keys = self.keys_for_id(parent_id);
                if child_keys.len() > 1 || parent_keys.len() > 1 {
                    if child_keys.len() != parent_keys.len() {
                        return Err(ImportError::UnsupportedBehavior(format!(
                            "array instance input {} has {} child elements and {} parent elements",
                            child.variable_name(input.id),
                            child_keys.len(),
                            parent_keys.len()
                        )));
                    }
                    for (child_key, parent_key) in child_keys.iter().zip(&parent_keys) {
                        let child_signal = child.signal(child_key)?;
                        let target = inline_signals[&child_signal];
                        let width = child.width(child_key)?;
                        let value = parent_env[parent_key];
                        let value = self.resize(value, width, child.is_signed(child_key))?;
                        self.rtl.assign(self.rtl.whole(target)?, value.id)?;
                    }
                    input_elements.insert(input.id, child_keys.len());
                    continue;
                }
            }
            for expression in &input.exprs {
                let element = input_elements.entry(input.id).or_insert(0);
                let child_key = child.port_element_key(input.id, *element)?;
                *element += 1;
                let child_signal = child.signal(&child_key)?;
                let target = inline_signals[&child_signal];
                let width = child.width(&child_key)?;
                let value = self.lower_assignment_value(
                    expression,
                    parent_env,
                    width,
                    child.is_signed(&child_key),
                )?;
                self.rtl.assign(self.rtl.whole(target)?, value.id)?;
            }
        }
        Ok(())
    }

    fn lower_instance_outputs(
        &mut self,
        instance: &InstDeclaration,
        child: &ModuleLowerer<'_>,
        inline_signals: &HashMap<SignalId, SignalId>,
    ) -> Result<(), ImportError> {
        let mut output_elements = HashMap::new();
        for output in &instance.outputs {
            // An explicit anonymous connection (`port: _`) has no destinations.
            // The child still drives its own signal; only parent wiring is absent
            // (IEEE 1800-2023 23.3.2.2, empty named port connections).
            if output.dst.is_empty() {
                continue;
            }
            if let [destination] = output.dst.as_slice()
                && destination.index.0.is_empty()
                && destination.select.is_empty()
            {
                let child_keys = child.keys_for_id(output.id);
                let parent_keys = self.keys_for_id(destination.id);
                if child_keys.len() > 1 || parent_keys.len() > 1 {
                    if child_keys.len() != parent_keys.len() {
                        return Err(ImportError::UnsupportedBehavior(format!(
                            "array instance output {} has {} child elements and {} parent elements",
                            child.variable_name(output.id),
                            child_keys.len(),
                            parent_keys.len()
                        )));
                    }
                    for (child_key, parent_key) in child_keys.iter().zip(&parent_keys) {
                        let child_signal = child.signal(child_key)?;
                        let source = inline_signals[&child_signal];
                        let source_width = child.width(child_key)?;
                        let target = self.signal(parent_key)?;
                        if source_width != self.width(parent_key)? {
                            return Err(ImportError::UnsupportedBehavior(format!(
                                "array instance output {} element width mismatch",
                                child.variable_name(output.id)
                            )));
                        }
                        let value = self.rtl.read(source)?;
                        self.rtl.assign(self.rtl.whole(target)?, value)?;
                    }
                    output_elements.insert(output.id, child_keys.len());
                    continue;
                }
            }
            let destinations = output
                .dst
                .iter()
                .map(|destination| self.destination_slice(destination))
                .collect::<Result<Vec<_>, _>>()?;
            if !child.variable_type(output.id)?.array.is_empty() {
                // AIR expands an unpacked slice into destinations in element
                // order. These are separate element assignments, not the
                // high-to-low pieces of a packed concatenation.
                let child_keys = child.keys_for_id(output.id);
                if child_keys.len() != destinations.len() {
                    return Err(ImportError::UnsupportedBehavior(format!(
                        "array instance output {} has {} child elements and {} destinations",
                        child.variable_name(output.id),
                        child_keys.len(),
                        destinations.len()
                    )));
                }
                for (child_key, destination) in child_keys.iter().zip(destinations) {
                    if child.width(child_key)? != destination.width.get() {
                        return Err(ImportError::UnsupportedBehavior(format!(
                            "array instance output {} element width mismatch",
                            child.variable_name(output.id)
                        )));
                    }
                    let source = inline_signals[&child.signal(child_key)?];
                    let value = self.rtl.read(source)?;
                    self.rtl.assign(destination, value)?;
                }
                output_elements.insert(output.id, child_keys.len());
                continue;
            }
            let element = output_elements.entry(output.id).or_insert(0);
            let child_key = child.port_element_key(output.id, *element)?;
            *element += 1;
            let child_signal = child.signal(&child_key)?;
            let source = inline_signals[&child_signal];
            let source_width = child.width(&child_key)?;
            let source_expr = self.rtl.read(source)?;
            let destination_width = destinations
                .iter()
                .map(|slice| slice.width.get())
                .sum::<u32>();
            if destination_width != source_width {
                return Err(ImportError::UnsupportedBehavior(format!(
                    "instance output {} connects {source_width} bits to {destination_width} bits",
                    child.variable_name(output.id)
                )));
            }
            let mut remaining = source_width;
            for destination in destinations {
                remaining -= destination.width.get();
                let value = if remaining == 0 && destination.width.get() == source_width {
                    source_expr
                } else {
                    self.rtl
                        .expression_slice(source_expr, remaining, destination.width)?
                };
                self.rtl.assign(destination, value)?;
            }
        }
        Ok(())
    }

    fn inline_module(
        &mut self,
        child: &RtlModule,
        prefix: &str,
    ) -> Result<HashMap<SignalId, SignalId>, ImportError> {
        reject_nested_instances(child)?;

        let mut signals = HashMap::new();
        for signal in child.signals() {
            let mapped = self
                .rtl
                .add_signal(format!("{prefix}.{}", signal.name()), signal.r#type());
            signals.insert(signal.id(), mapped);
        }

        let mut expressions = HashMap::new();
        for expression in child.expressions() {
            let mapped = match expression.kind() {
                ExprKind::Signal(slice) => self.rtl.read_slice(SignalSlice {
                    signal: signals[&slice.signal],
                    lsb: slice.lsb,
                    width: slice.width,
                })?,
                ExprKind::Constant(value) => self.rtl.constant(copy_constant(value)),
                ExprKind::Unary { op, input } => self.rtl.unary(*op, expressions[input])?,
                ExprKind::Binary { op, lhs, rhs } => {
                    self.rtl.binary(*op, expressions[lhs], expressions[rhs])?
                }
                ExprKind::Mux {
                    condition,
                    then_expr,
                    else_expr,
                } => self.rtl.mux(
                    expressions[condition],
                    expressions[then_expr],
                    expressions[else_expr],
                )?,
                ExprKind::Concat(parts) => self
                    .rtl
                    .concat(parts.iter().map(|part| expressions[part]).collect())?,
                ExprKind::Slice { input, lsb } => self.rtl.expression_slice(
                    expressions[input],
                    *lsb,
                    expression.r#type().width,
                )?,
            };
            expressions.insert(expression.id(), mapped);
        }

        for assignment in child.assignments() {
            self.rtl.assign(
                SignalSlice {
                    signal: signals[&assignment.target.signal],
                    lsb: assignment.target.lsb,
                    width: assignment.target.width,
                },
                expressions[&assignment.value],
            )?;
        }
        for register in child.registers() {
            self.rtl.add_register(Register {
                name: format!("{prefix}.{}", register.name),
                target: signals[&register.target],
                next: expressions[&register.next],
                clock: signals[&register.clock],
                edge: register.edge,
                enable: register.enable.map(|enable| Enable {
                    signal: signals[&enable.signal],
                    polarity: enable.polarity,
                }),
                reset: register.reset.map(|reset| Reset {
                    signal: signals[&reset.signal],
                    mode: reset.mode,
                    polarity: reset.polarity,
                    value: expressions[&reset.value],
                }),
            })?;
        }
        for memory in child.memories() {
            self.rtl.add_memory(Memory {
                name: format!("{prefix}.{}", memory.name),
                word: memory.word,
                depth: memory.depth,
                style: memory.style,
                read_latency: memory.read_latency,
                read_address: expressions[&memory.read_address],
                read_data: signals[&memory.read_data],
                read_enable: memory.read_enable.map(|enable| Enable {
                    signal: signals[&enable.signal],
                    polarity: enable.polarity,
                }),
                write_address: expressions[&memory.write_address],
                write_data: expressions[&memory.write_data],
                write_enable: Enable {
                    signal: signals[&memory.write_enable.signal],
                    polarity: memory.write_enable.polarity,
                },
                clock: signals[&memory.clock],
                edge: memory.edge,
                second_port: memory
                    .second_port
                    .as_ref()
                    .map(|port| remap_memory_port(port, &expressions, &signals)),
            });
        }
        Ok(signals)
    }

    fn destination_slice(
        &self,
        destination: &AssignDestination,
    ) -> Result<SignalSlice, ImportError> {
        let key = self.destination_key(destination)?;
        let signal = self.signal(&key)?;
        let (lsb, width) = static_select(&destination.select, self.width(&key)?)?;
        Ok(self.rtl.slice(signal, lsb, BitWidth::new(width)?)?)
    }

    fn lower_ff(&mut self, ff: &FfDeclaration) -> Result<LoweredFf, ImportError> {
        let (clock_key, edge) = self.source_clock(ff)?;
        let clock = self.signal(&clock_key)?;
        let initial = self.read_env()?;
        let mut next = initial.clone();
        let mut reset_values = None;
        let mut changed = DrivenBits::default();

        for statement in &ff.statements {
            if !memory_statement_patterns(statement, &self.inferred_memories).is_empty() {
                continue;
            }
            if let Statement::IfReset(branch) = statement {
                if reset_values.is_some() {
                    return Err(ImportError::UnsupportedBehavior(
                        "multiple if_reset statements in one always_ff".into(),
                    ));
                }
                let (reset_env, next_env, branch_changed) =
                    self.lower_if_reset(branch, &initial)?;
                reset_values = Some(reset_env);
                next = next_env;
                changed.extend(branch_changed);
            } else {
                changed.extend(self.lower_statement(statement, &initial, &mut next, true)?);
            }
        }

        let reset_control = if let Some(reset) = &ff.reset {
            if !reset.select.is_empty() {
                return Err(ImportError::UnsupportedBehavior("selected resets".into()));
            }
            let (mode, polarity) = match self.variable_type(reset.id)?.kind {
                TypeKind::ResetAsyncHigh => (ResetMode::Asynchronous, Polarity::ActiveHigh),
                TypeKind::ResetAsyncLow | TypeKind::Reset => {
                    (ResetMode::Asynchronous, Polarity::ActiveLow)
                }
                TypeKind::ResetSyncHigh => (ResetMode::Synchronous, Polarity::ActiveHigh),
                TypeKind::ResetSyncLow => (ResetMode::Synchronous, Polarity::ActiveLow),
                _ => {
                    return Err(ImportError::UnsupportedBehavior(
                        "always_ff reset is not a reset type".into(),
                    ));
                }
            };
            let reset_key = self.key_from_index(reset.id, &reset.index)?;
            Some((self.signal(&reset_key)?, mode, polarity))
        } else {
            None
        };

        changed
            .ranges
            .retain(|key, _| self.source.variables[&key.id].kind != VarKind::Let);
        Ok(LoweredFf {
            clock,
            edge,
            initial,
            next,
            reset_values,
            reset_control,
            changed,
        })
    }

    fn materialize_ff(
        &mut self,
        block: &LoweredFf,
        owners: &BTreeMap<SignalKey, usize>,
    ) -> Result<(), ImportError> {
        for key in block.changed.keys() {
            let signal = self.signal(key)?;
            let initial = block.initial[key];
            let next = block.next.get(key).copied().unwrap_or(initial);
            let reset_value = block
                .reset_values
                .as_ref()
                .map(|values| values.get(key).copied().unwrap_or(initial));

            // Preserve the established whole-register representation whenever
            // one block owns the signal.  Only split an aggregate when distinct
            // always_ff blocks need independent clock/reset semantics.
            if owners.get(key).copied().unwrap_or_default() == 1 {
                let reset = if let (Some(value), Some((reset_signal, mode, polarity))) =
                    (reset_value, block.reset_control)
                {
                    Some(Reset {
                        signal: reset_signal,
                        mode,
                        polarity,
                        value: value.id,
                    })
                } else {
                    None
                };
                self.rtl.add_register(Register {
                    name: self.signal_name(key),
                    target: signal,
                    next: next.id,
                    clock: block.clock,
                    edge: block.edge,
                    enable: None,
                    reset,
                })?;
                continue;
            }

            // Register targets in Struo RTL are whole signals.  Proxy signals
            // let each source block own exactly its packed ranges, which are
            // then projected back into the original aggregate with assignments.
            let signal_type = self.rtl.signals()[signal.index() as usize].r#type();
            for (lsb, width) in block.changed.ranges(key) {
                let width = BitWidth::new(width)?;
                let name = format!(
                    "__struo_ff_slice_{}_{}_{}",
                    signal.index(),
                    lsb,
                    width.get()
                );
                let slice_signal = self.rtl.add_signal(
                    name.clone(),
                    ValueType {
                        width,
                        signed: false,
                        state: signal_type.state,
                    },
                );
                let next = self.rtl.expression_slice(next.id, lsb, width)?;
                let reset = if let (Some(value), Some((reset_signal, mode, polarity))) =
                    (reset_value, block.reset_control)
                {
                    Some(Reset {
                        signal: reset_signal,
                        mode,
                        polarity,
                        value: self.rtl.expression_slice(value.id, lsb, width)?,
                    })
                } else {
                    None
                };
                self.rtl.add_register(Register {
                    name,
                    target: slice_signal,
                    next,
                    clock: block.clock,
                    edge: block.edge,
                    enable: None,
                    reset,
                })?;
                let value = self.rtl.read(slice_signal)?;
                let target = self.rtl.slice(signal, lsb, width)?;
                self.rtl.assign(target, value)?;
            }
        }
        Ok(())
    }

    fn lower_if_reset(
        &mut self,
        branch: &IfResetStatement,
        initial: &Env,
    ) -> Result<(Env, Env, DrivenBits), ImportError> {
        let mut reset = initial.clone();
        let mut next = initial.clone();
        let mut changed = self.lower_statements(&branch.true_side, initial, &mut reset, true)?;
        changed.extend(self.lower_statements(&branch.false_side, initial, &mut next, true)?);
        Ok((reset, next, changed))
    }

    fn lower_statements(
        &mut self,
        statements: &[Statement],
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<DrivenBits, ImportError> {
        let mut changed = DrivenBits::default();
        for statement in statements {
            if sequential {
                changed.extend(self.lower_statement(statement, reads, writes, true)?);
            } else {
                // Combinational blocks follow blocking semantics: each
                // statement observes every earlier write in the block.
                let snapshot = writes.clone();
                changed.extend(self.lower_statement(statement, &snapshot, writes, false)?);
            }
        }
        Ok(changed)
    }

    fn sequential_reads(&self, reads: &Env, writes: &Env) -> Env {
        let mut result = reads.clone();
        for (key, value) in writes {
            if self.source.variables.get(&key.id).is_some_and(|v| {
                v.kind == VarKind::Let
                    || v.affiliation == veryl_analyzer::symbol::Affiliation::AlwaysFf
            }) {
                result.insert(key.clone(), *value);
            }
        }
        result
    }

    fn lower_statement(
        &mut self,
        statement: &Statement,
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<DrivenBits, ImportError> {
        let effective_reads;
        let reads = if sequential {
            effective_reads = self.sequential_reads(reads, writes);
            &effective_reads
        } else {
            reads
        };
        match statement {
            Statement::Assign(assign) => self.lower_assignment(assign, reads, writes, sequential),
            Statement::If(branch) => {
                let mut effects = DrivenBits::default();
                let condition = if sequential {
                    self.lower_expression(&branch.cond, reads)?
                } else {
                    self.lower_comb_expression(&branch.cond, writes, &mut effects)?
                };
                let condition = self.boolean(condition)?;
                let base = writes.clone();
                let mut true_env = base.clone();
                let mut false_env = base;
                let mut changed =
                    self.lower_statements(&branch.true_side, reads, &mut true_env, sequential)?;
                changed.extend(self.lower_statements(
                    &branch.false_side,
                    reads,
                    &mut false_env,
                    sequential,
                )?);
                for key in changed.keys() {
                    let then_value = true_env[key];
                    let else_value = false_env[key];
                    let width = self.width(key)?;
                    let then_value = self.resize(then_value, width, self.is_signed(key))?;
                    let else_value = self.resize(else_value, width, self.is_signed(key))?;
                    let value = self.rtl.mux(condition.id, then_value.id, else_value.id)?;
                    writes.insert(
                        key.clone(),
                        LoweredExpr {
                            id: value,
                            width,
                            signed: self.is_signed(key),
                        },
                    );
                }
                changed.extend(effects);
                Ok(changed)
            }
            Statement::IfReset(_) if sequential => Err(ImportError::UnsupportedBehavior(
                "nested if_reset statements".into(),
            )),
            Statement::Null => Ok(DrivenBits::default()),
            Statement::Case(case_statement) => {
                self.lower_case(case_statement, reads, writes, sequential)
            }
            Statement::For(statement) => self.lower_for(statement, reads, writes, sequential),
            Statement::FunctionCall(call) => {
                if sequential {
                    let (_, outputs) = self.lower_function_call(call, reads)?;
                    self.validate_ff_function_outputs(&outputs)?;
                    self.copy_function_outputs(outputs, reads, writes)
                } else {
                    let mut effects = DrivenBits::default();
                    let (_, outputs) =
                        self.lower_function_call_effects(call, writes, &mut effects)?;
                    let reads = writes.clone();
                    effects.extend(self.copy_function_outputs(outputs, &reads, writes)?);
                    Ok(effects)
                }
            }
            Statement::SystemFunctionCall(call) => {
                let mut effects = DrivenBits::default();
                if sequential {
                    self.lower_system_function(call, reads)?;
                } else {
                    self.lower_system_function_effects(call, writes, &mut effects)?;
                }
                Ok(effects)
            }
            Statement::TbMethodCall(_) => Err(ImportError::UnsupportedBehavior(
                "testbench method calls are not synthesizable".into(),
            )),
            Statement::Break => Err(ImportError::UnsupportedBehavior(
                "break outside a lowered loop".into(),
            )),
            Statement::Unsupported(_) => Err(ImportError::UnsupportedBehavior(
                "analyzer marked a statement unsupported".into(),
            )),
            Statement::IfReset(_) => Err(ImportError::UnsupportedBehavior(
                "if_reset outside always_ff".into(),
            )),
        }
    }

    fn lower_assignment(
        &mut self,
        assign: &veryl_analyzer::ir::AssignStatement,
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<DrivenBits, ImportError> {
        if let [destination] = assign.dst.as_slice()
            && destination.index.0.len()
                < self.source.variables[&destination.id].r#type.array.dims()
        {
            return self.lower_array_assignment(destination, &assign.expr, reads, writes);
        }
        // Reads observe the pre-edge register value, but a partial
        // write composes over the value already scheduled for this
        // edge (later writes win per bit).
        let (value, mut changed) = if !sequential {
            let mut effects = DrivenBits::default();
            let value = self.lower_comb_expression(&assign.expr, writes, &mut effects)?;
            (value, effects)
        } else if let Expression::Term(factor) = &assign.expr
            && let Factor::FunctionCall(call) = factor.as_ref()
        {
            let (value, outputs) = self.lower_function_call(call, reads)?;
            self.validate_ff_function_outputs(&outputs)?;
            let changed = self.copy_function_outputs(outputs, reads, writes)?;
            (
                value.ok_or_else(|| {
                    ImportError::UnsupportedBehavior("void function assignment".into())
                })?,
                changed,
            )
        } else {
            (
                self.lower_expression(&assign.expr, reads)?,
                DrivenBits::default(),
            )
        };
        if sequential {
            changed.extend(self.assign_destinations(&assign.dst, value, reads, writes)?);
        } else {
            changed.extend(self.assign_destinations_effects(&assign.dst, value, writes)?);
        }
        Ok(changed)
    }

    fn lower_for(
        &mut self,
        statement: &veryl_analyzer::ir::ForStatement,
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<DrivenBits, ImportError> {
        let plan = loops::plan(statement, self.source)?;
        if plan.iterations.is_empty()
            && let Some((veryl_analyzer::ir::ForBound::Expression(end), _)) = &plan.guard
        {
            // Even an empty range evaluates its first condition. Until bound
            // effects are supported, reject them rather than dropping them.
            let snapshot = if sequential {
                self.sequential_reads(reads, writes)
            } else {
                writes.clone()
            };
            self.lower_expression(end, &snapshot)?;
        }
        let mut changed = DrivenBits::default();
        let mut stopped = self.constant(1, 0);
        for iteration in plan.iterations {
            if let Some((bound, inclusive)) = &plan.guard {
                let snapshot = if sequential {
                    self.sequential_reads(reads, writes)
                } else {
                    writes.clone()
                };
                let end = match bound {
                    veryl_analyzer::ir::ForBound::Const(value, signed) => LoweredExpr {
                        signed: *signed,
                        ..self.constant(64, *value as u64)
                    },
                    veryl_analyzer::ir::ForBound::Expression(expression) => {
                        self.lower_expression(expression, &snapshot)?
                    }
                };
                let width = concrete_width(&statement.var_type, "loop induction variable")?;
                let index = LoweredExpr {
                    signed: statement.var_type.signed,
                    ..self.constant(width, iteration as u64)
                };
                let op = if *inclusive {
                    Op::Greater
                } else {
                    Op::GreaterEq
                };
                let finished = self.lower_binary(op, index, end, 1, false)?;
                stopped = self.lower_binary(Op::LogicOr, stopped, finished, 1, false)?;
            }
            let mut body = statement.body.clone();
            substitute_statements(&mut body, statement.var_id, iteration)?;
            let before = writes.clone();
            let (written, stop) = self.lower_loop_body(&body, reads, writes, sequential)?;
            *writes = self.merge_values(stopped, &before, writes)?;
            stopped = self.lower_binary(Op::LogicOr, stopped, stop, 1, false)?;
            changed.extend(written);
        }
        Ok(changed)
    }

    fn lower_loop_body(
        &mut self,
        statements: &[Statement],
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<(DrivenBits, LoweredExpr), ImportError> {
        let mut changed = DrivenBits::default();
        let mut stopped = self.constant(1, 0);
        for statement in statements {
            let before = writes.clone();
            let snapshot = if sequential {
                self.sequential_reads(reads, writes)
            } else {
                before.clone()
            };
            let (written, stop) = match statement {
                Statement::Break => (DrivenBits::default(), self.constant(1, 1)),
                Statement::If(branch) => {
                    let mut effects = DrivenBits::default();
                    let condition = if sequential {
                        self.lower_expression(&branch.cond, &snapshot)?
                    } else {
                        self.lower_comb_expression(&branch.cond, writes, &mut effects)?
                    };
                    let condition = self.boolean(condition)?;
                    let mut yes = writes.clone();
                    let mut no = writes.clone();
                    let (mut written, yes_stop) =
                        self.lower_loop_body(&branch.true_side, reads, &mut yes, sequential)?;
                    let (no_written, no_stop) =
                        self.lower_loop_body(&branch.false_side, reads, &mut no, sequential)?;
                    written.extend(no_written);
                    written.extend(effects);
                    *writes = self.merge_values(condition, &yes, &no)?;
                    (
                        written,
                        LoweredExpr {
                            id: self.rtl.mux(condition.id, yes_stop.id, no_stop.id)?,
                            width: 1,
                            signed: false,
                        },
                    )
                }
                Statement::Case(case) => {
                    let mut effects = DrivenBits::default();
                    let target = self.prepare_case_target(
                        &case.case_target,
                        &snapshot,
                        writes,
                        sequential,
                        &mut effects,
                    )?;
                    let mut result = writes.clone();
                    let (mut written, mut stop) =
                        self.lower_loop_body(&case.default, reads, &mut result, sequential)?;
                    written.extend(effects);
                    for arm in case.arms.iter().rev() {
                        let condition =
                            self.lower_case_patterns(&target, &arm.patterns, &snapshot)?;
                        let mut yes = writes.clone();
                        let (yes_written, yes_stop) =
                            self.lower_loop_body(&arm.body, reads, &mut yes, sequential)?;
                        written.extend(yes_written);
                        result = self.merge_values(condition, &yes, &result)?;
                        stop = LoweredExpr {
                            id: self.rtl.mux(condition.id, yes_stop.id, stop.id)?,
                            width: 1,
                            signed: false,
                        };
                    }
                    *writes = result;
                    (written, stop)
                }
                _ => (
                    self.lower_statement(statement, &snapshot, writes, sequential)?,
                    self.constant(1, 0),
                ),
            };
            *writes = self.merge_values(stopped, &before, writes)?;
            stopped = self.lower_binary(Op::LogicOr, stopped, stop, 1, false)?;
            changed.extend(written);
        }
        Ok((changed, stopped))
    }

    fn lower_case(
        &mut self,
        statement: &CaseStatement,
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<DrivenBits, ImportError> {
        let mut effects = DrivenBits::default();
        let target = self.prepare_case_target(
            &statement.case_target,
            reads,
            writes,
            sequential,
            &mut effects,
        )?;
        let base = writes.clone();
        let mut else_env = base.clone();
        let mut changed =
            self.lower_statements(&statement.default, reads, &mut else_env, sequential)?;
        changed.extend(effects);

        // A balanced first-match tree does not reduce the mux depth below four
        // arms: the final default selection replaces the level saved in the
        // matched-value tree.  Keep the established priority chain for these
        // small cases so equivalent source does not needlessly perturb packing
        // and placement.
        if statement.arms.len() < 4 {
            for arm in statement.arms.iter().rev() {
                let mut then_env = base.clone();
                let arm_changed =
                    self.lower_statements(&arm.body, reads, &mut then_env, sequential)?;
                let condition = self.lower_case_patterns(&target, &arm.patterns, &base)?;
                let mut merged_changed = changed.clone();
                merged_changed.extend(arm_changed);
                let mut merged_env = base.clone();
                for key in merged_changed.keys() {
                    let width = self.width(key)?;
                    let signed = self.is_signed(key);
                    let then_value = self.resize(then_env[key], width, signed)?;
                    let else_value = self.resize(else_env[key], width, signed)?;
                    let value = self.rtl.mux(condition.id, then_value.id, else_value.id)?;
                    merged_env.insert(
                        key.clone(),
                        LoweredExpr {
                            id: value,
                            width,
                            signed,
                        },
                    );
                }
                else_env = merged_env;
                changed = merged_changed;
            }

            *writes = else_env;
            return Ok(changed);
        }

        // Preserve the existing body-lowering order and environment: each arm
        // starts from the pre-case writes, and the default is lowered first.
        // The vector is reversed afterwards so index zero is the highest-priority
        // source arm.
        let mut lowered_arms = Vec::with_capacity(statement.arms.len());

        for arm in statement.arms.iter().rev() {
            let mut then_env = base.clone();
            let arm_changed = self.lower_statements(&arm.body, reads, &mut then_env, sequential)?;
            let condition = self.lower_case_patterns(&target, &arm.patterns, &base)?;
            changed.extend(arm_changed);
            lowered_arms.push((condition, then_env));
        }
        lowered_arms.reverse();

        if lowered_arms.is_empty() {
            *writes = else_env;
            return Ok(changed);
        }

        let conditions = lowered_arms
            .iter()
            .map(|(condition, _)| *condition)
            .collect::<Vec<_>>();
        let match_tree = self.lower_case_match_tree(&conditions, 0)?;
        let mut merged_env = base;
        for key in changed.keys() {
            let width = self.width(key)?;
            let signed = self.is_signed(key);
            let matched_value =
                self.lower_case_value_tree(&match_tree, &lowered_arms, key, width, signed)?;
            let default_value = self.resize(else_env[key], width, signed)?;
            let value =
                self.rtl
                    .mux(match_tree.any_match.id, matched_value.id, default_value.id)?;
            merged_env.insert(
                key.clone(),
                LoweredExpr {
                    id: value,
                    width,
                    signed,
                },
            );
        }

        *writes = merged_env;
        Ok(changed)
    }

    fn lower_case_match_tree(
        &mut self,
        conditions: &[LoweredExpr],
        first_arm: usize,
    ) -> Result<CaseMatchTree, ImportError> {
        debug_assert!(!conditions.is_empty());
        if conditions.len() == 1 {
            return Ok(CaseMatchTree {
                any_match: conditions[0],
                kind: CaseMatchTreeKind::Arm(first_arm),
            });
        }

        let split = conditions.len() / 2;
        let left = self.lower_case_match_tree(&conditions[..split], first_arm)?;
        let right = self.lower_case_match_tree(&conditions[split..], first_arm + split)?;
        let any_match =
            self.lower_binary(Op::LogicOr, left.any_match, right.any_match, 1, false)?;
        Ok(CaseMatchTree {
            any_match,
            kind: CaseMatchTreeKind::Branch {
                left: Box::new(left),
                right: Box::new(right),
            },
        })
    }

    fn lower_case_value_tree(
        &mut self,
        tree: &CaseMatchTree,
        arms: &[(LoweredExpr, Env)],
        key: &SignalKey,
        width: u32,
        signed: bool,
    ) -> Result<LoweredExpr, ImportError> {
        match &tree.kind {
            CaseMatchTreeKind::Arm(index) => self.resize(arms[*index].1[key], width, signed),
            CaseMatchTreeKind::Branch { left, right } => {
                let left_value = self.lower_case_value_tree(left, arms, key, width, signed)?;
                let right_value = self.lower_case_value_tree(right, arms, key, width, signed)?;
                Ok(LoweredExpr {
                    id: self
                        .rtl
                        .mux(left.any_match.id, left_value.id, right_value.id)?,
                    width,
                    signed,
                })
            }
        }
    }

    fn lower_case_patterns(
        &mut self,
        target: &PreparedCaseTarget,
        patterns: &[CasePattern],
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let mut condition = None;
        for pattern in patterns {
            let matches = match pattern {
                CasePattern::Eq(value) => {
                    let target = self.lower_case_operand(target, value)?;
                    if let Some(ct) = comparisons::wildcard_pattern(value) {
                        self.lower_wildcard_pattern(target, Op::EqWildcard, &ct)?
                    } else {
                        let value = self.lower_case_label(value, env)?;
                        self.lower_binary(Op::Eq, target, value, 1, false)?
                    }
                }
                CasePattern::Range { lo, hi, inclusive } => {
                    let lo_target = self.lower_case_operand(target, lo)?;
                    let hi_target = self.lower_case_operand(target, hi)?;
                    let lo = self.lower_case_label(lo, env)?;
                    let hi = self.lower_case_label(hi, env)?;
                    let lower = self.lower_binary(Op::LessEq, lo, lo_target, 1, false)?;
                    let upper_op = if *inclusive { Op::LessEq } else { Op::Less };
                    let upper = self.lower_binary(upper_op, hi_target, hi, 1, false)?;
                    self.lower_binary(Op::LogicAnd, lower, upper, 1, false)?
                }
            };
            condition = Some(match condition {
                Some(previous) => self.lower_binary(Op::LogicOr, previous, matches, 1, false)?,
                None => matches,
            });
        }
        condition
            .ok_or_else(|| ImportError::UnsupportedBehavior("case arm without a pattern".into()))
    }

    fn lower_function_call(
        &mut self,
        call: &veryl_analyzer::ir::FunctionCall,
        reads: &Env,
    ) -> Result<FunctionResult, ImportError> {
        let mut caller = reads.clone();
        let mut effects = DrivenBits::default();
        let result = self.lower_function_call_effects(call, &mut caller, &mut effects)?;
        if !effects.ranges.is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "function input effects in a read-only call".into(),
            ));
        }
        Ok(result)
    }

    fn lower_function_call_effects(
        &mut self,
        call: &veryl_analyzer::ir::FunctionCall,
        caller: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<FunctionResult, ImportError> {
        if self.call_depth >= 64 {
            return Err(ImportError::UnsupportedBehavior(
                "recursive function expansion exceeds 64 calls".into(),
            ));
        }
        let function = self
            .source
            .functions
            .get(&call.id)
            .ok_or_else(|| ImportError::UnsupportedBehavior("missing function body".into()))?;
        let body = function
            .get_function(call.index.as_deref().unwrap_or(&[]))
            .ok_or_else(|| {
                ImportError::UnsupportedBehavior("unresolved function specialization".into())
            })?;
        // Choose AIR argument order (source order), without implying that SV
        // requires this order: IEEE 1800-2023 13.5 leaves it undefined. Freeze
        // each value after its evaluation, while exposing its writes to later
        // arguments. Only then create the callee's automatic frame (13.5.1).
        let mut inputs = Vec::new();
        for (path, expression) in &call.inputs {
            let id = *body.arg_map.get(path).ok_or_else(|| {
                ImportError::UnsupportedBehavior("missing function formal".into())
            })?;
            let mut ty = self.source.variables[&id].r#type.clone();
            let dimensions = ty
                .array
                .iter()
                .copied()
                .collect::<Option<Vec<_>>>()
                .ok_or_else(|| ImportError::NonConcreteWidth("function argument".into()))?;
            ty.array.clear();
            let values =
                self.lower_array_argument_effects(expression, &dimensions, &ty, caller, effects)?;
            for (index, value) in
                array_indices(&self.source.variables[&id].r#type, "function argument")?
                    .into_iter()
                    .zip(values)
            {
                inputs.push((SignalKey { id, index }, value));
            }
        }
        let mut env = caller.clone();
        // Function storage is an automatic frame, never a hardware register.
        let locals = self
            .source
            .variables
            .values()
            .filter(|v| {
                v.affiliation == veryl_analyzer::symbol::Affiliation::Function
                    && !matches!(v.kind, VarKind::Const | VarKind::Param)
            })
            .cloned()
            .collect::<Vec<_>>();
        for variable in locals {
            let width = concrete_width(&variable.r#type, "function local")?;
            for index in array_indices(&variable.r#type, "function local")? {
                let key = SignalKey {
                    id: variable.id,
                    index,
                };
                self.widths.insert(key.clone(), width);
                self.signed.insert(key.clone(), variable.r#type.signed);
                let mut value = self.constant(width, 0);
                value.signed = variable.r#type.signed;
                env.insert(key, value);
            }
        }
        env.extend(inputs);
        self.call_depth += 1;
        let result = self.lower_function_statements(&body.statements, &mut env, body.ret);
        self.call_depth -= 1;
        result?;
        // Explicit output formals are copied below. A write directly to module
        // storage must not disappear when this automatic frame is discarded.
        if caller.iter().any(|(key, before)| {
            self.source.variables.get(&key.id).is_some_and(|variable| {
                variable.affiliation != veryl_analyzer::symbol::Affiliation::Function
            }) && env.get(key).is_some_and(|after| after.id != before.id)
        }) {
            return Err(ImportError::UnsupportedBehavior(
                "function writes to non-local storage require caller writeback".into(),
            ));
        }
        let returned = body
            .ret
            .map(|id| self.pack_function_return(id, &env))
            .transpose()?;
        let mut outputs = Vec::new();
        for (path, destinations) in &call.outputs {
            let id = *body.arg_map.get(path).ok_or_else(|| {
                ImportError::UnsupportedBehavior("missing function output".into())
            })?;
            let key = self.key_from_index(id, &VarIndex::default())?;
            outputs.push((destinations.clone(), env[&key]));
        }
        Ok((returned, outputs))
    }

    fn pack_function_return(&mut self, id: VarId, env: &Env) -> Result<LoweredExpr, ImportError> {
        let ty = &self.source.variables[&id].r#type;
        let values = array_indices(ty, "function return")?
            .into_iter()
            .map(|index| {
                env.get(&SignalKey { id, index })
                    .copied()
                    .ok_or_else(|| ImportError::MissingVariable("function return".into()))
            })
            .collect::<Result<Vec<_>, _>>()?;
        if values.len() == 1 {
            return Ok(values[0]);
        }
        let width = values
            .iter()
            .try_fold(0u32, |sum, value| sum.checked_add(value.width))
            .ok_or_else(|| ImportError::NonConcreteWidth("function return".into()))?;
        // Element zero occupies the low bits of this internal transport value.
        Ok(LoweredExpr {
            id: self
                .rtl
                .concat(values.iter().rev().map(|v| v.id).collect())?,
            width,
            signed: false,
        })
    }

    fn merge_values(
        &mut self,
        condition: LoweredExpr,
        yes: &Env,
        no: &Env,
    ) -> Result<Env, ImportError> {
        let mut merged = no.clone();
        for (key, value) in yes {
            let previous = no.get(key).copied().unwrap_or(*value);
            if value.id != previous.id {
                merged.insert(
                    key.clone(),
                    LoweredExpr {
                        id: self.rtl.mux(condition.id, value.id, previous.id)?,
                        ..*value
                    },
                );
            }
        }
        Ok(merged)
    }

    fn lower_function_statements(
        &mut self,
        statements: &[Statement],
        env: &mut Env,
        ret: Option<VarId>,
    ) -> Result<LoweredExpr, ImportError> {
        let mut returned = self.constant(1, 0);
        for statement in statements {
            let before = env.clone();
            let this_return = match statement {
                Statement::If(branch) => {
                    let condition =
                        self.lower_comb_expression(&branch.cond, env, &mut DrivenBits::default())?;
                    let condition = self.boolean(condition)?;
                    let mut yes = env.clone();
                    let mut no = env.clone();
                    let yes_return =
                        self.lower_function_statements(&branch.true_side, &mut yes, ret)?;
                    let no_return =
                        self.lower_function_statements(&branch.false_side, &mut no, ret)?;
                    *env = self.merge_values(condition, &yes, &no)?;
                    LoweredExpr {
                        id: self.rtl.mux(condition.id, yes_return.id, no_return.id)?,
                        width: 1,
                        signed: false,
                    }
                }
                Statement::Case(case) => {
                    let target = self.prepare_case_target(
                        &case.case_target,
                        &before,
                        env,
                        false,
                        &mut DrivenBits::default(),
                    )?;
                    let mut result = env.clone();
                    let mut flag =
                        self.lower_function_statements(&case.default, &mut result, ret)?;
                    for arm in case.arms.iter().rev() {
                        let condition =
                            self.lower_case_patterns(&target, &arm.patterns, &before)?;
                        let mut yes = env.clone();
                        let yes_return =
                            self.lower_function_statements(&arm.body, &mut yes, ret)?;
                        result = self.merge_values(condition, &yes, &result)?;
                        flag = LoweredExpr {
                            id: self.rtl.mux(condition.id, yes_return.id, flag.id)?,
                            width: 1,
                            signed: false,
                        };
                    }
                    *env = result;
                    flag
                }
                Statement::For(loop_statement)
                    if ret.is_some_and(|id| contains_destination(&loop_statement.body, id)) =>
                {
                    return Err(ImportError::UnsupportedBehavior(
                        "return inside a retained function loop".into(),
                    ));
                }
                _ => {
                    self.lower_statement(statement, &before, env, false)?;
                    self.constant(1, u64::from(matches!(statement, Statement::Assign(a) if a.dst.iter().any(|d| Some(d.id) == ret))))
                }
            };
            // A return in either branch prevents all later side effects on that path.
            *env = self.merge_values(returned, &before, env)?;
            returned = self.lower_binary(Op::LogicOr, returned, this_return, 1, false)?;
        }
        Ok(returned)
    }

    fn validate_ff_function_outputs(&self, outputs: &FunctionOutputs) -> Result<(), ImportError> {
        // Veryl 0.22 now converts inout formals. Do not let that bypass the
        // retained restriction on function copy-out to module state in always_ff.
        if outputs.iter().any(|(destinations, _)| {
            destinations.iter().any(|dst| {
                self.source.variables[&dst.id].affiliation
                    != veryl_analyzer::symbol::Affiliation::AlwaysFf
            })
        }) {
            return Err(ImportError::UnsupportedBehavior(
                "function output/inout copy-out to module state in always_ff".into(),
            ));
        }
        Ok(())
    }

    fn copy_function_outputs(
        &mut self,
        outputs: FunctionOutputs,
        reads: &Env,
        writes: &mut Env,
    ) -> Result<DrivenBits, ImportError> {
        let mut changed = DrivenBits::default();
        for (destinations, value) in outputs {
            changed.extend(self.assign_destinations(&destinations, value, reads, writes)?);
        }
        Ok(changed)
    }

    fn assign_destinations(
        &mut self,
        destinations: &[AssignDestination],
        value: LoweredExpr,
        reads: &Env,
        writes: &mut Env,
    ) -> Result<DrivenBits, ImportError> {
        if destinations.len() == 1 {
            return self.assign_destination(&destinations[0], value, reads, writes);
        }
        let widths = destinations
            .iter()
            .map(|dst| {
                let width = concrete_width(self.variable_type(dst.id)?, "assignment destination")?;
                selected_width(&dst.select, width)
            })
            .collect::<Result<Vec<_>, ImportError>>()?;
        let total = widths
            .iter()
            .try_fold(0u32, |sum, width| sum.checked_add(*width))
            .ok_or_else(|| ImportError::WidthTooLarge("concatenated assignment".into()))?;
        let value = self.resize(value, total, value.signed)?;
        let mut remaining = total;
        let mut changed = DrivenBits::default();
        for (dst, width) in destinations.iter().zip(widths) {
            remaining -= width;
            let part = LoweredExpr {
                id: self
                    .rtl
                    .expression_slice(value.id, remaining, BitWidth::new(width)?)?,
                width,
                signed: false,
            };
            changed.extend(self.assign_destination(dst, part, reads, writes)?);
        }
        Ok(changed)
    }

    fn prepare_destination(
        &mut self,
        destination: &AssignDestination,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<PreparedDestination, ImportError> {
        let elements = if has_dynamic_array_index(&destination.index) {
            self.lower_array_elements_effects(destination.id, &destination.index, env, effects)?
                .into_iter()
                .map(|(key, condition)| (key, Some(condition)))
                .collect()
        } else {
            vec![(self.destination_key(destination)?, None)]
        };
        let width = selected_width(&destination.select, self.width(&elements[0].0)?)?;
        let offset = if dynamic_packed_select(&destination.select) {
            Some(self.lower_select_offset_effects(
                &destination.select,
                width,
                env,
                effects,
                destination.comptime.member_select_domain.is_some(),
            )?)
        } else {
            None
        };
        Ok(PreparedDestination {
            elements,
            packed: PreparedSelect {
                select: destination.select.clone(),
                offset,
                domain: destination.comptime.member_select_domain,
            },
            width,
        })
    }

    fn assign_destinations_effects(
        &mut self,
        destinations: &[AssignDestination],
        value: LoweredExpr,
        env: &mut Env,
    ) -> Result<DrivenBits, ImportError> {
        let mut changed = DrivenBits::default();
        // The RHS is already frozen. Evaluate all LHS addresses in AIR order
        // before storing any parts; IEEE 1800-2023 10.4.1 leaves RHS/LHS
        // evaluation order unspecified for blocking assignments without timing.
        let prepared = destinations
            .iter()
            .map(|dst| self.prepare_destination(dst, env, &mut changed))
            .collect::<Result<Vec<_>, _>>()?;
        let total = prepared
            .iter()
            .try_fold(0u32, |sum, dst| sum.checked_add(dst.width))
            .ok_or_else(|| ImportError::WidthTooLarge("concatenated assignment".into()))?;
        let value = if prepared.len() == 1 {
            value
        } else {
            self.resize(value, total, value.signed)?
        };
        let mut remaining = total;
        for dst in &prepared {
            remaining -= dst.width;
            let part = if prepared.len() == 1 {
                value
            } else {
                LoweredExpr {
                    id: self.rtl.expression_slice(
                        value.id,
                        remaining,
                        BitWidth::new(dst.width)?,
                    )?,
                    width: dst.width,
                    signed: false,
                }
            };
            for (key, condition) in &dst.elements {
                let (lsb, width) = driven_select(&dst.packed.select, self.width(key)?)?;
                let current = env[key];
                self.assign_key_prepared(key, &dst.packed, part, env)?;
                if let Some(condition) = condition {
                    let assigned = env[key];
                    env.insert(
                        key.clone(),
                        LoweredExpr {
                            id: self.rtl.mux(condition.id, assigned.id, current.id)?,
                            ..current
                        },
                    );
                }
                changed.insert_range(key.clone(), lsb, width);
            }
        }
        Ok(changed)
    }

    fn lower_array_assignment(
        &mut self,
        destination: &AssignDestination,
        expression: &Expression,
        reads: &Env,
        writes: &mut Env,
    ) -> Result<DrivenBits, ImportError> {
        if !destination.select.is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "packed select on array assignment".into(),
            ));
        }
        let mut ty = self.source.variables[&destination.id].r#type.clone();
        let shape = ty
            .array
            .iter()
            .skip(destination.index.0.len())
            .copied()
            .collect::<Vec<_>>();
        let dimensions = shape
            .iter()
            .copied()
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| ImportError::NonConcreteWidth("array assignment".into()))?;
        ty.array.clear();
        let values = self.lower_array_argument(expression, &dimensions, &ty, reads)?;
        let mut changed = DrivenBits::default();
        for (flat, value) in values.into_iter().enumerate() {
            let mut element = destination.clone();
            element
                .index
                .0
                .extend(VarIndex::from_index(flat, veryl_analyzer::ir::ShapeRef::new(&shape)).0);
            changed.extend(self.assign_destination(&element, value, reads, writes)?);
        }
        Ok(changed)
    }

    // Array arguments are copied into the automatic frame element by element.
    // Packed conversion applies to each leaf, not to the aggregate bit stream.
    fn lower_array_argument(
        &mut self,
        expression: &Expression,
        dimensions: &[usize],
        element_type: &Type,
        env: &Env,
    ) -> Result<Vec<LoweredExpr>, ImportError> {
        let mut temporary = env.clone();
        let mut effects = DrivenBits::default();
        let values = self.lower_array_argument_effects(
            expression,
            dimensions,
            element_type,
            &mut temporary,
            &mut effects,
        )?;
        if !effects.ranges.is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "array argument effects in a read-only expression".into(),
            ));
        }
        Ok(values)
    }

    fn lower_array_argument_effects(
        &mut self,
        expression: &Expression,
        dimensions: &[usize],
        element_type: &Type,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<Vec<LoweredExpr>, ImportError> {
        let width = concrete_width(element_type, "array element")?;
        let signed = element_type.signed;
        let Some((&length, tail)) = dimensions.split_first() else {
            if let Expression::ArrayLiteral(items, _) = expression {
                return Ok(vec![self.lower_packed_array_literal(
                    items,
                    element_type,
                    env,
                    effects,
                )?]);
            }
            return Ok(vec![self.lower_assignment_value_effects(
                expression, env, width, signed, effects,
            )?]);
        };
        if let Expression::ArrayLiteral(items, _) = expression {
            let repetitions = arrays::literal_repetitions(items, length)?;
            let mut values = Vec::new();
            let mut default = None;
            for (item, repeat) in items.iter().zip(repetitions) {
                if repeat == 0 {
                    continue;
                }
                let value = match item {
                    ArrayLiteralItem::Value(value, _) | ArrayLiteralItem::Defaul(value) => value,
                };
                let elements = if matches!(item, ArrayLiteralItem::Defaul(_))
                    && !matches!(value.as_ref(), Expression::ArrayLiteral(_, _))
                    && value.comptime().r#type.array.is_empty()
                {
                    let leaf =
                        self.lower_assignment_value_effects(value, env, width, signed, effects)?;
                    vec![leaf; tail.iter().product()]
                } else {
                    self.lower_array_argument_effects(value, tail, element_type, env, effects)?
                };
                if matches!(item, ArrayLiteralItem::Defaul(_)) {
                    default = Some((elements, repeat));
                } else {
                    for _ in 0..repeat {
                        values.extend_from_slice(&elements);
                    }
                }
            }
            if let Some((elements, repeat)) = default {
                for _ in 0..repeat {
                    values.extend_from_slice(&elements);
                }
            }
            return Ok(values);
        }
        if let Expression::Term(factor) = expression
            && let Factor::FunctionCall(call) = factor.as_ref()
        {
            return self.lower_array_call(
                call,
                &expression.comptime().r#type,
                dimensions,
                element_type,
                env,
                effects,
            );
        }
        if let Expression::Term(factor) = expression
            && let Factor::Variable(id, prefix, select, comptime) = factor.as_ref()
        {
            let ty = &self.source.variables[id].r#type;
            let shape = dimensions.iter().copied().map(Some).collect::<Vec<_>>();
            if !select.is_empty()
                || ty
                    .array
                    .iter()
                    .skip(prefix.0.len())
                    .copied()
                    .collect::<Vec<_>>()
                    != shape
            {
                return Err(ImportError::UnsupportedBehavior(
                    "array argument shape mismatch".into(),
                ));
            }
            let mut values = Vec::new();
            for flat in 0..dimensions.iter().product() {
                let mut index = prefix.clone();
                index.0.extend(
                    VarIndex::from_index(flat, veryl_analyzer::ir::ShapeRef::new(&shape)).0,
                );
                let value = self.lower_factor(
                    &Factor::Variable(*id, index, select.clone(), comptime.clone()),
                    env,
                )?;
                values.push(self.resize(value, width, signed)?);
            }
            return Ok(values);
        }
        Err(ImportError::UnsupportedBehavior(
            "array-valued function argument".into(),
        ))
    }

    fn lower_array_call(
        &mut self,
        call: &veryl_analyzer::ir::FunctionCall,
        ty: &Type,
        dimensions: &[usize],
        target: &Type,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<Vec<LoweredExpr>, ImportError> {
        let width = concrete_width(target, "array return target")?;
        let signed = target.signed;
        let shape = dimensions.iter().copied().map(Some).collect::<Vec<_>>();
        if ty.array.iter().copied().collect::<Vec<_>>() != shape {
            return Err(ImportError::UnsupportedBehavior(
                "array return shape mismatch".into(),
            ));
        }
        let element_width = concrete_width(ty, "array return")?;
        let element_signed = ty.signed;
        let (value, outputs) = self.lower_function_call_effects(call, env, effects)?;
        let reads = env.clone();
        effects.extend(self.copy_function_outputs(outputs, &reads, env)?);
        let value =
            value.ok_or_else(|| ImportError::UnsupportedBehavior("void array call".into()))?;
        let mut values = Vec::new();
        for flat in 0..dimensions.iter().product::<usize>() {
            let element = LoweredExpr {
                id: self.rtl.expression_slice(
                    value.id,
                    u32::try_from(flat)
                        .ok()
                        .and_then(|index| index.checked_mul(element_width))
                        .ok_or_else(|| {
                            ImportError::NonConcreteWidth("array return offset".into())
                        })?,
                    BitWidth::new(element_width)?,
                )?,
                width: element_width,
                signed: element_signed,
            };
            values.push(self.resize(element, width, signed)?);
        }
        Ok(values)
    }

    fn lower_packed_array_literal(
        &mut self,
        items: &[ArrayLiteralItem],
        ty: &Type,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<LoweredExpr, ImportError> {
        if matches!(ty.kind, TypeKind::Unknown) {
            return Err(ImportError::NonConcreteWidth(
                "packed array literal type".into(),
            ));
        }
        if !ty.array.is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "unpacked array literal in a scalar expression".into(),
            ));
        }
        let width = concrete_width(ty, "packed array literal")?;
        let length = if ty.width().is_empty() {
            width as usize
        } else {
            ty.width()[0]
                .ok_or_else(|| ImportError::NonConcreteWidth("packed array literal".into()))?
        };
        if length == 0 || !(width as usize).is_multiple_of(length) {
            return Err(ImportError::UnsupportedBehavior(
                "packed array literal shape mismatch".into(),
            ));
        }
        let element_width = width
            / u32::try_from(length)
                .map_err(|_| ImportError::WidthTooLarge("packed array literal dimension".into()))?;
        let mut element_type = ty.clone();
        element_type.signed = false;
        if element_type.width().is_empty() {
            element_type.kind = TypeKind::Logic;
        } else {
            element_type.width_mut().drain(0..1);
        }
        let repetitions = arrays::literal_repetitions(items, length)?;
        let mut parts = Vec::new();
        let mut default = None;
        // Evaluate active items once in AIR order, then replicate their values.
        // IEEE 1800-2023 10.9.1 leaves evaluation counts for defaults and
        // repetitions with side effects undefined; this is our chosen policy.
        for (item, repeat) in items.iter().zip(repetitions) {
            if repeat == 0 {
                continue;
            }
            let expression = match item {
                ArrayLiteralItem::Value(value, _) | ArrayLiteralItem::Defaul(value) => value,
            };
            let value = if let Expression::ArrayLiteral(nested, _) = expression.as_ref() {
                self.lower_packed_array_literal(nested, &element_type, env, effects)?
            } else {
                self.lower_assignment_value_effects(expression, env, element_width, false, effects)?
            };
            if matches!(item, ArrayLiteralItem::Defaul(_)) {
                default = Some((value.id, repeat));
            } else {
                parts.extend(std::iter::repeat_n(value.id, repeat));
            }
        }
        if let Some((value, repeat)) = default {
            parts.extend(std::iter::repeat_n(value, repeat));
        }
        Ok(LoweredExpr {
            id: self.rtl.concat(parts)?,
            width,
            signed: ty.signed,
        })
    }

    fn lower_assignment_value(
        &mut self,
        expression: &Expression,
        env: &Env,
        width: u32,
        signed: bool,
    ) -> Result<LoweredExpr, ImportError> {
        let mut contextual = expression.clone();
        contextual.comptime_mut().expr_context.width =
            contextual.comptime().expr_context.width.max(width as usize);
        let expression = &contextual;
        let value = self.lower_expression(expression, env)?;
        self.resize(value, width, signed)
    }

    fn lower_assignment_value_effects(
        &mut self,
        expression: &Expression,
        env: &mut Env,
        width: u32,
        signed: bool,
        effects: &mut DrivenBits,
    ) -> Result<LoweredExpr, ImportError> {
        let mut contextual = expression.clone();
        contextual.comptime_mut().expr_context.width =
            contextual.comptime().expr_context.width.max(width as usize);
        let value = self.lower_comb_expression(&contextual, env, effects)?;
        self.resize(value, width, signed)
    }

    fn lower_expression(
        &mut self,
        expression: &Expression,
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let mut expression = expression.clone();
        repair_expression_signedness(&mut expression, None);
        self.lower_prepared_expression(&expression, env)
    }

    fn lower_prepared_expression(
        &mut self,
        expression: &Expression,
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let mut temporary = env.clone();
        let mut effects = DrivenBits::default();
        let value = self.lower_expression_effects(expression, &mut temporary, &mut effects)?;
        if !effects.ranges.is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "function output effects in a read-only expression".into(),
            ));
        }
        Ok(value)
    }

    fn lower_comb_expression(
        &mut self,
        expression: &Expression,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<LoweredExpr, ImportError> {
        let mut expression = expression.clone();
        repair_expression_signedness(&mut expression, None);
        self.lower_expression_effects(&expression, env, effects)
    }

    fn lower_expression_effects(
        &mut self,
        expression: &Expression,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<LoweredExpr, ImportError> {
        match expression {
            Expression::Term(factor) => self.lower_factor_effects(factor, env, effects),
            Expression::Unary(op, input, comptime) => {
                let input = self.lower_expression_effects(input, env, effects)?;
                self.lower_unary(*op, input, comptime)
            }
            Expression::Binary(lhs, op, rhs, comptime) => {
                if matches!(op, Op::EqWildcard | Op::NeWildcard)
                    && let Some(ct) = comparisons::wildcard_pattern(rhs)
                {
                    let lhs = self.lower_expression_effects(lhs, env, effects)?;
                    return self.lower_wildcard_pattern(lhs, *op, &ct);
                }
                if *op == Op::As {
                    let lhs = self.lower_expression_effects(lhs, env, effects)?;
                    let width = concrete_width(&comptime.r#type, "cast expression")?;
                    let signed = if matches!(rhs.comptime().value, ValueVariant::Type(_)) {
                        comptime.r#type.signed
                    } else {
                        lhs.signed
                    };
                    return self.resize(lhs, width, signed);
                }
                let skip_rhs = lhs.comptime().is_const
                    && evaluated_u64(lhs).is_some_and(|value| {
                        (*op == Op::LogicAnd && value == 0) || (*op == Op::LogicOr && value != 0)
                    });
                let lhs = self.lower_expression_effects(lhs, env, effects)?;
                if skip_rhs {
                    return self.boolean(lhs);
                }
                let before_rhs = env.clone();
                let rhs = self.lower_expression_effects(rhs, env, effects)?;
                if matches!(op, Op::LogicAnd | Op::LogicOr) {
                    let condition = self.boolean(lhs)?;
                    *env = if *op == Op::LogicAnd {
                        self.merge_values(condition, env, &before_rhs)?
                    } else {
                        self.merge_values(condition, &before_rhs, env)?
                    };
                }
                let result_width = binary_width(*op, comptime)?;
                let signed = binary_signed(*op, comptime);
                self.lower_binary(*op, lhs, rhs, result_width, signed)
            }
            Expression::Ternary(condition, then_expr, else_expr, comptime) => {
                self.lower_ternary_effects(condition, then_expr, else_expr, comptime, env, effects)
            }
            Expression::Concatenation(parts, _) => {
                let mut lowered = Vec::new();
                let mut width = 0u32;
                for (part, repeat) in parts {
                    let part = self.lower_expression_effects(part, env, effects)?;
                    let count = if let Some(repeat) = repeat {
                        constant_value(repeat)?
                    } else {
                        1
                    };
                    width = count
                        .checked_mul(u64::from(part.width))
                        .and_then(|bits| bits.checked_add(u64::from(width)))
                        .and_then(|bits| u32::try_from(bits).ok())
                        .ok_or_else(|| ImportError::WidthTooLarge("concatenation".into()))?;
                    for _ in 0..count {
                        lowered.push(part.id);
                    }
                }
                Ok(LoweredExpr {
                    id: self.rtl.concat(lowered)?,
                    width,
                    signed: false,
                })
            }
            Expression::StructConstructor(r#type, fields, _) => {
                self.lower_struct_constructor(r#type, fields, env)
            }
            Expression::ArrayLiteral(items, comptime) => {
                self.lower_packed_array_literal(items, &comptime.r#type, env, effects)
            }
        }
    }

    fn lower_ternary_effects(
        &mut self,
        condition: &Expression,
        then_expr: &Expression,
        else_expr: &Expression,
        comptime: &Comptime,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<LoweredExpr, ImportError> {
        let selected = condition
            .comptime()
            .is_const
            .then(|| evaluated_u64(condition))
            .flatten();
        let condition = self.lower_expression_effects(condition, env, effects)?;
        if let Some(selected) = selected {
            let value = self.lower_expression_effects(
                if selected != 0 { then_expr } else { else_expr },
                env,
                effects,
            )?;
            let width = context_width(comptime)?.max(value.width);
            let signed = comptime.expr_context.signed;
            return self.resize(LoweredExpr { signed, ..value }, width, signed);
        }
        let condition = self.boolean(condition)?;
        let mut yes = env.clone();
        let mut no = env.clone();
        let then_expr = self.lower_expression_effects(then_expr, &mut yes, effects)?;
        let else_expr = self.lower_expression_effects(else_expr, &mut no, effects)?;
        *env = self.merge_values(condition, &yes, &no)?;
        let width = context_width(comptime)?
            .max(then_expr.width)
            .max(else_expr.width);
        let signed = comptime.expr_context.signed;
        let then_expr = self.resize(
            LoweredExpr {
                signed,
                ..then_expr
            },
            width,
            signed,
        )?;
        let else_expr = self.resize(
            LoweredExpr {
                signed,
                ..else_expr
            },
            width,
            signed,
        )?;
        Ok(LoweredExpr {
            id: self.rtl.mux(condition.id, then_expr.id, else_expr.id)?,
            width,
            signed,
        })
    }

    fn lower_factor_effects(
        &mut self,
        factor: &Factor,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<LoweredExpr, ImportError> {
        if let Factor::FunctionCall(call) = factor {
            let (value, outputs) = self.lower_function_call_effects(call, env, effects)?;
            let reads = env.clone();
            effects.extend(self.copy_function_outputs(outputs, &reads, env)?);
            value.ok_or_else(|| ImportError::UnsupportedBehavior("void function expression".into()))
        } else if let Factor::Variable(id, index, select, comptime) = factor {
            self.lower_variable_effects(*id, index, select, comptime, env, effects)
        } else if let Factor::SystemFunctionCall(call) = factor {
            self.lower_system_function_effects(call, env, effects)
        } else {
            self.lower_factor(factor, env)
        }
    }

    fn lower_unary(
        &mut self,
        op: Op,
        input: LoweredExpr,
        comptime: &Comptime,
    ) -> Result<LoweredExpr, ImportError> {
        let sized = matches!(op, Op::Add | Op::Sub | Op::BitNot);
        let width = if sized { context_width(comptime)? } else { 1 };
        let signed = sized && comptime.expr_context.signed;
        let input = if sized {
            self.resize(
                LoweredExpr { signed, ..input },
                input.width.max(width),
                signed,
            )?
        } else {
            input
        };
        let (id, result_width, signed) = match op {
            Op::Add => (input.id, input.width, input.signed),
            Op::Sub => {
                let zero = self.constant(input.width, 0);
                (
                    self.rtl.binary(BinaryOp::Sub, zero.id, input.id)?,
                    input.width,
                    input.signed,
                )
            }
            Op::LogicNot => (self.rtl.unary(UnaryOp::LogicNot, input.id)?, 1, false),
            Op::BitNot => (
                self.rtl.unary(UnaryOp::BitNot, input.id)?,
                input.width,
                input.signed,
            ),
            Op::BitAnd => (self.rtl.unary(UnaryOp::ReduceAnd, input.id)?, 1, false),
            Op::BitOr => (self.rtl.unary(UnaryOp::ReduceOr, input.id)?, 1, false),
            Op::BitXor => (self.rtl.unary(UnaryOp::ReduceXor, input.id)?, 1, false),
            Op::BitNand | Op::BitNor | Op::BitXnor => {
                let reduction = match op {
                    Op::BitNand => UnaryOp::ReduceAnd,
                    Op::BitNor => UnaryOp::ReduceOr,
                    _ => UnaryOp::ReduceXor,
                };
                let reduced = self.rtl.unary(reduction, input.id)?;
                (self.rtl.unary(UnaryOp::BitNot, reduced)?, 1, false)
            }
            _ => return Err(Self::unsupported_expression(op)),
        };
        self.resize(
            LoweredExpr {
                id,
                width: result_width,
                signed,
            },
            width,
            signed,
        )
    }

    fn lower_struct_constructor(
        &mut self,
        r#type: &Type,
        fields: &[(StrId, Expression)],
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let TypeKind::Struct(definition) = &r#type.kind else {
            return Err(ImportError::UnsupportedBehavior(
                "non-struct aggregate constructor".into(),
            ));
        };
        let mut lowered = Vec::with_capacity(definition.members.len());
        for member in &definition.members {
            let field = fields
                .iter()
                .find_map(|(name, expression)| (*name == member.name).then_some(expression))
                .ok_or_else(|| {
                    ImportError::UnsupportedBehavior(format!(
                        "struct constructor is missing field `{}`",
                        member.name
                    ))
                })?;
            let value = self.lower_expression(field, env)?;
            let width = concrete_width(&member.r#type, "struct member")?;
            lowered.push(self.resize(value, width, member.r#type.signed)?.id);
        }
        Ok(LoweredExpr {
            id: self.rtl.concat(lowered)?,
            width: concrete_width(r#type, "struct constructor")?,
            signed: r#type.signed,
        })
    }

    fn lower_binary(
        &mut self,
        op: Op,
        lhs: LoweredExpr,
        rhs: LoweredExpr,
        result_width: u32,
        result_signed: bool,
    ) -> Result<LoweredExpr, ImportError> {
        if op == Op::Pow {
            return self.lower_power(lhs, rhs, result_width, result_signed);
        }
        if matches!(op, Op::LogicAnd | Op::LogicOr) {
            let lhs = self.boolean(lhs)?;
            let rhs = self.boolean(rhs)?;
            let op = if op == Op::LogicAnd {
                BinaryOp::And
            } else {
                BinaryOp::Or
            };
            return Ok(LoweredExpr {
                id: self.rtl.binary(op, lhs.id, rhs.id)?,
                width: 1,
                signed: false,
            });
        }
        if matches!(
            op,
            Op::LogicShiftL | Op::ArithShiftL | Op::LogicShiftR | Op::ArithShiftR
        ) {
            let operation = match op {
                Op::LogicShiftL | Op::ArithShiftL => BinaryOp::ShiftLeft,
                Op::ArithShiftR if lhs.signed && result_signed => BinaryOp::ShiftRightArithmetic,
                Op::LogicShiftR | Op::ArithShiftR => BinaryOp::ShiftRightLogical,
                _ => unreachable!(),
            };
            let lhs = self.resize(lhs, result_width, result_signed)?;
            return Ok(LoweredExpr {
                id: self.rtl.binary(operation, lhs.id, rhs.id)?,
                width: result_width,
                signed: result_signed,
            });
        }
        let comparison = matches!(
            op,
            Op::Eq
                | Op::Ne
                | Op::EqWildcard
                | Op::NeWildcard
                | Op::Less
                | Op::LessEq
                | Op::Greater
                | Op::GreaterEq
        );
        // Widen arithmetic operands before evaluation; widening the result
        // afterwards cannot recover lost carry, borrow or product bits.
        let operand_width = if comparison {
            lhs.width.max(rhs.width)
        } else {
            lhs.width.max(rhs.width).max(result_width)
        };
        let signed_compare = if comparison {
            lhs.signed && rhs.signed
        } else {
            result_signed
        };
        // Extension follows the common operand type, including comparisons:
        // an unsigned operand makes both sides unsigned before widening.
        let lhs = LoweredExpr {
            signed: signed_compare,
            ..lhs
        };
        let rhs = LoweredExpr {
            signed: signed_compare,
            ..rhs
        };
        let lhs = self.resize(lhs, operand_width, signed_compare)?;
        let rhs = self.resize(rhs, operand_width, signed_compare)?;
        if matches!(op, Op::Div | Op::Rem) {
            let value = self.lower_div_rem(lhs, rhs, op == Op::Rem, result_signed)?;
            return self.resize(value, result_width, result_signed);
        }
        let operation = match op {
            Op::Add => BinaryOp::Add,
            Op::Sub => BinaryOp::Sub,
            Op::Mul => BinaryOp::Mul,
            Op::BitAnd => BinaryOp::And,
            Op::BitOr => BinaryOp::Or,
            Op::BitXor | Op::BitXnor => BinaryOp::Xor,
            Op::Eq | Op::EqWildcard => BinaryOp::Equal,
            Op::Ne | Op::NeWildcard => BinaryOp::NotEqual,
            Op::Less if signed_compare => BinaryOp::LessThanSigned,
            Op::Less => BinaryOp::LessThanUnsigned,
            Op::LessEq if signed_compare => BinaryOp::LessOrEqualSigned,
            Op::LessEq => BinaryOp::LessOrEqualUnsigned,
            Op::Greater if signed_compare => BinaryOp::GreaterThanSigned,
            Op::Greater => BinaryOp::GreaterThanUnsigned,
            Op::GreaterEq if signed_compare => BinaryOp::GreaterOrEqualSigned,
            Op::GreaterEq => BinaryOp::GreaterOrEqualUnsigned,
            _ => return Err(Self::unsupported_expression(op)),
        };
        let mut id = self.rtl.binary(operation, lhs.id, rhs.id)?;
        if op == Op::BitXnor {
            id = self.rtl.unary(UnaryOp::BitNot, id)?;
        }
        let value = LoweredExpr {
            id,
            width: if comparison { 1 } else { operand_width },
            signed: !comparison && result_signed,
        };
        self.resize(value, result_width, result_signed)
    }

    // IEEE 1800-2023 11.4.3 and 11.6.1: the exponent is self-determined,
    // while the base is widened to the result context before multiplication.
    // Repeated squaring keeps the circuit size linear in the exponent width.
    fn lower_power(
        &mut self,
        base: LoweredExpr,
        exponent: LoweredExpr,
        width: u32,
        signed: bool,
    ) -> Result<LoweredExpr, ImportError> {
        let base = self.resize(LoweredExpr { signed, ..base }, width, signed)?;
        let known = match self.rtl.expressions()[exponent.id.index() as usize].kind() {
            ExprKind::Constant(value) => Some(value.clone()),
            _ => None,
        };
        let negative = if exponent.signed {
            Some(
                self.rtl
                    .expression_slice(exponent.id, exponent.width - 1, BitWidth::new(1)?)?,
            )
        } else {
            None
        };
        let one = self.constant(width, 1).id;
        let mut result = one;
        let mut power = base.id;
        let bits = known.as_ref().map_or(exponent.width, |value| {
            if exponent.signed && value.bit(exponent.width - 1) {
                0
            } else {
                (0..exponent.width)
                    .rev()
                    .find(|bit| value.bit(*bit))
                    .map_or(0, |bit| bit + 1)
            }
        });
        for bit in 0..bits {
            if known.as_ref().is_none_or(|value| value.bit(bit)) {
                let product = self.rtl.binary(BinaryOp::Mul, result, power)?;
                result = if known.is_some() {
                    product
                } else {
                    let enabled = self
                        .rtl
                        .expression_slice(exponent.id, bit, BitWidth::new(1)?)?;
                    self.rtl.mux(enabled, product, result)?
                };
            }
            if bit + 1 < bits {
                power = self.rtl.binary(BinaryOp::Mul, power, power)?;
            }
        }
        if let Some(negative) = negative {
            let zero = self.constant(width, 0).id;
            let is_one = self.rtl.binary(BinaryOp::Equal, base.id, one)?;
            let mut reciprocal = self.rtl.mux(is_one, one, zero)?;
            if signed {
                let minus_one = self.rtl.unary(UnaryOp::BitNot, zero)?;
                let is_minus_one = self.rtl.binary(BinaryOp::Equal, base.id, minus_one)?;
                let odd = self
                    .rtl
                    .expression_slice(exponent.id, 0, BitWidth::new(1)?)?;
                let unit = self.rtl.mux(odd, minus_one, one)?;
                reciprocal = self.rtl.mux(is_minus_one, unit, reciprocal)?;
            }
            // Zero raised to a negative exponent is X in four-state SV.
            // This two-state RTL follows the adapter's zero convention.
            result = self.rtl.mux(negative, reciprocal, result)?;
        }
        Ok(LoweredExpr {
            id: result,
            width,
            signed,
        })
    }

    // Restoring division uses an extra remainder bit so the trial shift cannot
    // overflow. Signed division truncates toward zero; remainder follows lhs.
    fn lower_div_rem(
        &mut self,
        lhs: LoweredExpr,
        rhs: LoweredExpr,
        remainder: bool,
        signed: bool,
    ) -> Result<LoweredExpr, ImportError> {
        let width = lhs.width;
        let zero = self.constant(width, 0).id;
        let lhs_sign = self
            .rtl
            .expression_slice(lhs.id, width - 1, BitWidth::new(1)?)?;
        let rhs_sign = self
            .rtl
            .expression_slice(rhs.id, width - 1, BitWidth::new(1)?)?;
        let (numerator, denominator) = if signed {
            let neg_lhs = self.rtl.binary(BinaryOp::Sub, zero, lhs.id)?;
            let neg_rhs = self.rtl.binary(BinaryOp::Sub, zero, rhs.id)?;
            (
                self.rtl.mux(lhs_sign, neg_lhs, lhs.id)?,
                self.rtl.mux(rhs_sign, neg_rhs, rhs.id)?,
            )
        } else {
            (lhs.id, rhs.id)
        };
        let denominator = self
            .resize(
                LoweredExpr {
                    id: denominator,
                    width,
                    signed: false,
                },
                width + 1,
                false,
            )?
            .id;
        let mut rem = self.constant(width + 1, 0).id;
        let mut quotient = Vec::with_capacity(width as usize);
        for bit in (0..width).rev() {
            let low = self.rtl.expression_slice(rem, 0, BitWidth::new(width)?)?;
            let next_bit = self
                .rtl
                .expression_slice(numerator, bit, BitWidth::new(1)?)?;
            let shifted = self.rtl.concat(vec![low, next_bit])?;
            let fits = self
                .rtl
                .binary(BinaryOp::GreaterOrEqualUnsigned, shifted, denominator)?;
            let difference = self.rtl.binary(BinaryOp::Sub, shifted, denominator)?;
            rem = self.rtl.mux(fits, difference, shifted)?;
            quotient.push(fits);
        }
        let mut id = if remainder {
            self.rtl.expression_slice(rem, 0, BitWidth::new(width)?)?
        } else {
            self.rtl.concat(quotient)?
        };
        if signed {
            let negative = if remainder {
                lhs_sign
            } else {
                self.rtl.binary(BinaryOp::Xor, lhs_sign, rhs_sign)?
            };
            let negated = self.rtl.binary(BinaryOp::Sub, zero, id)?;
            id = self.rtl.mux(negative, negated, id)?;
        }
        // Two-state simulation convention: map the all-X zero-divisor result
        // to zero. This adapter does not implement four-state arithmetic.
        let divisor_zero = self.rtl.binary(BinaryOp::Equal, rhs.id, zero)?;
        id = self.rtl.mux(divisor_zero, zero, id)?;
        Ok(LoweredExpr { id, width, signed })
    }

    fn lower_factor(&mut self, factor: &Factor, env: &Env) -> Result<LoweredExpr, ImportError> {
        match factor {
            Factor::Variable(id, index, select, comptime) => {
                let mut temporary = env.clone();
                let mut effects = DrivenBits::default();
                let value = self.lower_variable_effects(
                    *id,
                    index,
                    select,
                    comptime,
                    &mut temporary,
                    &mut effects,
                )?;
                if !effects.ranges.is_empty() {
                    return Err(ImportError::UnsupportedBehavior(
                        "index effects in a read-only expression".into(),
                    ));
                }
                Ok(value)
            }
            Factor::Value(comptime) | Factor::Anonymous(comptime) => {
                self.lower_comptime(comptime, "literal")
            }
            Factor::Unknown(_) => Err(ImportError::UnsupportedBehavior(
                "unknown or four-state X/Z literal".into(),
            )),
            Factor::HierVariable(_) => Err(ImportError::UnsupportedBehavior(
                "hierarchical variable reference".into(),
            )),
            Factor::FunctionCall(call) => {
                let (value, outputs) = self.lower_function_call(call, env)?;
                if !outputs.is_empty() {
                    return Err(ImportError::UnsupportedBehavior(
                        "function output inside compound expression".into(),
                    ));
                }
                value.ok_or_else(|| {
                    ImportError::UnsupportedBehavior("void function expression".into())
                })
            }
            Factor::SystemFunctionCall(call) => self.lower_system_function(call, env),
        }
    }

    fn lower_variable_effects(
        &mut self,
        id: VarId,
        index: &VarIndex,
        select: &VarSelect,
        comptime: &Comptime,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<LoweredExpr, ImportError> {
        let constant = if self.is_constant_variable(id) {
            Some(self.lower_constant_variable_read(id, index, env, effects)?)
        } else {
            None
        };
        // Freeze each address component once, in source order, before reading data.
        let elements = if constant.is_none() && has_dynamic_array_index(index) {
            Some(self.lower_array_elements_effects(id, index, env, effects)?)
        } else {
            None
        };
        let offset = if dynamic_packed_select(select) {
            let width = selected_width(
                select,
                concrete_width(self.variable_type(id)?, "indexed variable")?,
            )?;
            Some(self.lower_select_offset_effects(
                select,
                width,
                env,
                effects,
                comptime.member_select_domain.is_some(),
            )?)
        } else {
            None
        };
        let source = if let Some(value) = constant {
            value
        } else if let Some(elements) = elements {
            self.lower_dynamic_array_read(elements, env)?
        } else {
            let key = self.key_from_index(id, index)?;
            if let Some(source) = env.get(&key).copied() {
                source
            } else if comptime.is_const {
                self.lower_comptime(comptime, "constant variable")?
            } else {
                return Err(ImportError::UnsupportedBehavior(format!(
                    "reference to non-runtime variable {}",
                    self.signal_name(&key)
                )));
            }
        };
        let source = if let Some(domain) = comptime.member_select_domain {
            let mask = self.member_mask(source.width, domain)?;
            LoweredExpr {
                id: self.rtl.binary(BinaryOp::And, source.id, mask)?,
                ..source
            }
        } else {
            source
        };
        if let Some((offset, negative)) = offset {
            let width = selected_width(select, source.width)?;
            let shifted = self.shift_selected(source.id, offset, negative, false)?;
            return Ok(LoweredExpr {
                id: self
                    .rtl
                    .expression_slice(shifted, 0, BitWidth::new(width)?)?,
                width,
                signed: false,
            });
        }
        let (lsb, width) = static_select(select, source.width)?;
        let signed = variable_signedness(select, comptime);
        if lsb == 0 && width == source.width {
            Ok(LoweredExpr { signed, ..source })
        } else {
            Ok(LoweredExpr {
                id: self
                    .rtl
                    .expression_slice(source.id, lsb, BitWidth::new(width)?)?,
                width,
                signed,
            })
        }
    }

    fn lower_system_function(
        &mut self,
        call: &veryl_analyzer::ir::SystemFunctionCall,
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let mut temporary = env.clone();
        let mut effects = DrivenBits::default();
        let value = self.lower_system_function_effects(call, &mut temporary, &mut effects)?;
        if !effects.ranges.is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "function output effects in a read-only system function".into(),
            ));
        }
        Ok(value)
    }

    fn lower_system_function_effects(
        &mut self,
        call: &veryl_analyzer::ir::SystemFunctionCall,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<LoweredExpr, ImportError> {
        use veryl_analyzer::ir::SystemFunctionKind;
        let value = match &call.kind {
            SystemFunctionKind::Signed(input) | SystemFunctionKind::Unsigned(input) => {
                let value = self.lower_comb_expression(&input.0, env, effects)?;
                LoweredExpr {
                    signed: matches!(call.kind, SystemFunctionKind::Signed(_)),
                    ..value
                }
            }
            SystemFunctionKind::Bits(input) | SystemFunctionKind::Size(input, _) => {
                let comptime = input.0.comptime();
                let ty = if let ValueVariant::Type(ty) = &comptime.value {
                    ty
                } else {
                    &comptime.r#type
                };
                let size = if matches!(call.kind, SystemFunctionKind::Bits(_)) {
                    ty.total_width().and_then(|width| {
                        ty.array
                            .iter()
                            .try_fold(width, |total, dim| total.checked_mul((*dim)?))
                    })
                } else {
                    let dimension = match &call.kind {
                        SystemFunctionKind::Size(_, Some(dimension)) => {
                            constant_value(&dimension.0)?
                        }
                        _ => 1,
                    };
                    let dimension = usize::try_from(dimension)
                        .ok()
                        .and_then(|dimension| dimension.checked_sub(1))
                        .ok_or_else(|| {
                            ImportError::UnsupportedBehavior(
                                "$size dimension must be a positive constant".into(),
                            )
                        })?;
                    ty.array
                        .iter()
                        .chain(ty.width().iter())
                        .nth(dimension)
                        .copied()
                        .flatten()
                }
                .ok_or_else(|| ImportError::NonConcreteWidth("system function argument".into()))?;
                let value = self.constant(32, size as u64);
                LoweredExpr {
                    signed: true,
                    ..value
                }
            }
            SystemFunctionKind::Onehot(input) => {
                let value = self.lower_comb_expression(&input.0, env, effects)?;
                let one = self.constant(value.width, 1);
                let zero = self.constant(value.width, 0);
                let less = self.rtl.binary(BinaryOp::Sub, value.id, one.id)?;
                let masked = self.rtl.binary(BinaryOp::And, value.id, less)?;
                let single = self.rtl.binary(BinaryOp::Equal, masked, zero.id)?;
                let nonzero = self.rtl.binary(BinaryOp::NotEqual, value.id, zero.id)?;
                LoweredExpr {
                    id: self.rtl.binary(BinaryOp::And, single, nonzero)?,
                    width: 1,
                    signed: false,
                }
            }
            SystemFunctionKind::Clog2(input) => {
                let value = self.lower_comb_expression(&input.0, env, effects)?;
                let one = self.constant(value.width, 1);
                let zero = self.constant(value.width, 0);
                let less = self.rtl.binary(BinaryOp::Sub, value.id, one.id)?;
                let mut result = self.constant(32, 0);
                for bit in 0..value.width {
                    let set = self.rtl.expression_slice(less, bit, BitWidth::new(1)?)?;
                    let count = self.constant(32, u64::from(bit + 1));
                    result.id = self.rtl.mux(set, count.id, result.id)?;
                }
                let is_zero = self.rtl.binary(BinaryOp::Equal, value.id, zero.id)?;
                let zero_result = self.constant(32, 0);
                result.id = self.rtl.mux(is_zero, zero_result.id, result.id)?;
                result
            }
            _ => {
                return Err(ImportError::UnsupportedBehavior(
                    "system function expression".into(),
                ));
            }
        };
        let signed = call.comptime.expr_context.signed;
        self.resize(
            LoweredExpr { signed, ..value },
            context_width(&call.comptime)?,
            signed,
        )
    }

    fn lower_comptime(
        &mut self,
        comptime: &Comptime,
        context: &str,
    ) -> Result<LoweredExpr, ImportError> {
        let ValueVariant::Numeric(value) = &comptime.value else {
            return Err(ImportError::UnsupportedBehavior(format!(
                "non-numeric compile-time {context}"
            )));
        };
        // Folded system functions can carry TypeKind::Unknown with a valid
        // numeric width. Interpreting that placeholder type as one bit loses data.
        let width = if value.width() == 0 {
            context_width(comptime)?.max(1)
        } else {
            u32::try_from(value.width()).map_err(|_| ImportError::WidthTooLarge(context.into()))?
        };
        if value.is_xz() {
            return Err(ImportError::UnsupportedBehavior(format!(
                "unknown or four-state compile-time {context}"
            )));
        }
        let value = value.expand(width as usize, value.signed());
        let words = value.payload().to_u64_digits();
        Ok(LoweredExpr {
            id: self
                .rtl
                .constant(Constant::new(BitWidth::new(width)?, words)),
            width,
            signed: if comptime.expr_context.width == 0 {
                value.signed()
            } else {
                comptime.r#type.signed
            },
        })
    }

    fn lower_dynamic_array_read(
        &mut self,
        elements: Vec<(SignalKey, LoweredExpr)>,
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let first = &elements[0].0;
        let width = self.width(first)?;
        let signed = self.is_signed(first);
        let mut result = self.constant(width, 0);
        result.signed = signed;
        if elements.len() >= 4 {
            let mut entries = Vec::with_capacity(elements.len());
            for (key, condition) in elements {
                let value = env.get(&key).copied().ok_or_else(|| {
                    ImportError::UnsupportedBehavior(format!(
                        "reference to non-runtime variable {}",
                        self.signal_name(&key)
                    ))
                })?;
                entries.push((condition, value));
            }
            let (matched, value) = self.lower_array_read_tree(&entries)?;
            result.id = self.rtl.mux(matched.id, value.id, result.id)?;
            return Ok(result);
        }
        for (key, condition) in elements.into_iter().rev() {
            let value = env.get(&key).copied().ok_or_else(|| {
                ImportError::UnsupportedBehavior(format!(
                    "reference to non-runtime variable {}",
                    self.signal_name(&key)
                ))
            })?;
            result = LoweredExpr {
                id: self.rtl.mux(condition.id, value.id, result.id)?,
                width,
                signed,
            };
        }
        Ok(result)
    }

    /// Balance the selected value and its match predicate together. Retaining
    /// first-match priority also preserves behavior if index predicates overlap;
    /// the caller supplies the existing zero result when no element matches.
    fn lower_array_read_tree(
        &mut self,
        entries: &[(LoweredExpr, LoweredExpr)],
    ) -> Result<(LoweredExpr, LoweredExpr), ImportError> {
        if entries.len() == 1 {
            return Ok(entries[0]);
        }
        let split = entries.len() / 2;
        let (left_match, left_value) = self.lower_array_read_tree(&entries[..split])?;
        let (right_match, right_value) = self.lower_array_read_tree(&entries[split..])?;
        let matched = self.lower_binary(Op::LogicOr, left_match, right_match, 1, false)?;
        let value = LoweredExpr {
            id: self.rtl.mux(left_match.id, left_value.id, right_value.id)?,
            ..left_value
        };
        Ok((matched, value))
    }

    fn assign_destination(
        &mut self,
        destination: &AssignDestination,
        value: LoweredExpr,
        reads: &Env,
        writes: &mut Env,
    ) -> Result<DrivenBits, ImportError> {
        if !has_dynamic_array_index(&destination.index) {
            let key = self.destination_key(destination)?;
            let (lsb, width) = driven_select(&destination.select, self.width(&key)?)?;
            self.assign_key(&key, destination, value, reads, writes)?;
            let mut changed = DrivenBits::default();
            changed.insert_range(key, lsb, width);
            return Ok(changed);
        }

        let elements = self.lower_array_elements(destination.id, &destination.index, reads)?;
        let mut changed = DrivenBits::default();
        for (key, condition) in elements {
            let (lsb, width) = driven_select(&destination.select, self.width(&key)?)?;
            let current = writes[&key];
            self.assign_key(&key, destination, value, reads, writes)?;
            let assigned = writes[&key];
            writes.insert(
                key.clone(),
                LoweredExpr {
                    id: self.rtl.mux(condition.id, assigned.id, current.id)?,
                    width: current.width,
                    signed: current.signed,
                },
            );
            changed.insert_range(key, lsb, width);
        }
        Ok(changed)
    }

    // Keep offset arithmetic wider than the source index so an out-of-range
    // index cannot wrap back into the vector. Negative offsets reverse the
    // shift direction, preserving the valid bits of partially overlapping slices.
    fn lower_select_offset(
        &mut self,
        select: &VarSelect,
        width: u32,
        env: &Env,
        member: bool,
    ) -> Result<(ExprId, ExprId), ImportError> {
        let mut temporary = env.clone();
        let mut effects = DrivenBits::default();
        let offset =
            self.lower_select_offset_effects(select, width, &mut temporary, &mut effects, member)?;
        if !effects.ranges.is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "index effects in a read-only select".into(),
            ));
        }
        Ok(offset)
    }

    fn lower_select_offset_effects(
        &mut self,
        select: &VarSelect,
        width: u32,
        env: &mut Env,
        effects: &mut DrivenBits,
        member: bool,
    ) -> Result<(ExprId, ExprId), ImportError> {
        let index = if member {
            self.lower_member_index(&select.0[0], env, effects)?
        } else {
            self.lower_comb_expression(&select.0[0], env, effects)?
        };
        let offset_width = index
            .width
            .checked_add(u32::BITS - width.leading_zeros() + 1)
            .ok_or_else(|| ImportError::WidthTooLarge("packed select offset".into()))?;
        let index = self.resize(index, offset_width, true)?;
        let offset = match select.1 {
            Some((VarSelectOp::Step, _)) => {
                let stride = self.constant(offset_width, u64::from(width));
                self.rtl.binary(BinaryOp::Mul, index.id, stride.id)?
            }
            Some((VarSelectOp::MinusColon, _)) => {
                let adjustment = self.constant(offset_width, u64::from(width - 1));
                self.rtl.binary(BinaryOp::Sub, index.id, adjustment.id)?
            }
            _ => index.id,
        };
        let negative = self
            .rtl
            .expression_slice(offset, offset_width - 1, BitWidth::new(1)?)?;
        let zero = self.constant(offset_width, 0);
        let magnitude = self.rtl.binary(BinaryOp::Sub, zero.id, offset)?;
        let magnitude = self.rtl.mux(negative, magnitude, offset)?;
        Ok((magnitude, negative))
    }

    fn shift_selected(
        &mut self,
        value: ExprId,
        offset: ExprId,
        negative: ExprId,
        write: bool,
    ) -> Result<ExprId, ImportError> {
        let left = self.rtl.binary(BinaryOp::ShiftLeft, value, offset)?;
        let right = self
            .rtl
            .binary(BinaryOp::ShiftRightLogical, value, offset)?;
        Ok(if write {
            self.rtl.mux(negative, right, left)?
        } else {
            self.rtl.mux(negative, left, right)?
        })
    }

    fn assign_key(
        &mut self,
        key: &SignalKey,
        destination: &AssignDestination,
        value: LoweredExpr,
        reads: &Env,
        env: &mut Env,
    ) -> Result<(), ImportError> {
        let select = &destination.select;
        let offset = if dynamic_packed_select(select) {
            let width = selected_width(select, self.width(key)?)?;
            Some(self.lower_select_offset(
                select,
                width,
                reads,
                destination.comptime.member_select_domain.is_some(),
            )?)
        } else {
            None
        };
        self.assign_key_prepared(
            key,
            &PreparedSelect {
                select: select.clone(),
                offset,
                domain: destination.comptime.member_select_domain,
            },
            value,
            env,
        )
    }

    fn assign_key_prepared(
        &mut self,
        key: &SignalKey,
        packed: &PreparedSelect,
        value: LoweredExpr,
        env: &mut Env,
    ) -> Result<(), ImportError> {
        let previous = env[key];
        self.assign_key_bits(key, &packed.select, packed.offset, value, env)?;
        if let Some(domain) = packed.domain {
            let assigned = env[key];
            let mask = self.member_mask(previous.width, domain)?;
            let inverse = self.rtl.unary(UnaryOp::BitNot, mask)?;
            let held = self.rtl.binary(BinaryOp::And, previous.id, inverse)?;
            let written = self.rtl.binary(BinaryOp::And, assigned.id, mask)?;
            env.insert(
                key.clone(),
                LoweredExpr {
                    id: self.rtl.binary(BinaryOp::Or, held, written)?,
                    ..assigned
                },
            );
        }
        Ok(())
    }

    fn assign_key_bits(
        &mut self,
        key: &SignalKey,
        select: &VarSelect,
        offset: Option<(ExprId, ExprId)>,
        value: LoweredExpr,
        env: &mut Env,
    ) -> Result<(), ImportError> {
        let total_width = self.width(key)?;
        if let Some((offset, negative)) = offset {
            let width = selected_width(select, total_width)?;
            let value = self.resize(value, width, false)?;
            let value = self.resize(
                LoweredExpr {
                    signed: false,
                    ..value
                },
                total_width,
                false,
            )?;
            let ones = self.constant(width, 0);
            let ones = LoweredExpr {
                id: self.rtl.unary(UnaryOp::BitNot, ones.id)?,
                width,
                signed: false,
            };
            let ones = self.resize(ones, total_width, false)?;
            let mask = self.shift_selected(ones.id, offset, negative, true)?;
            let replacement = self.shift_selected(value.id, offset, negative, true)?;
            let inverse = self.rtl.unary(UnaryOp::BitNot, mask)?;
            let held = self.rtl.binary(BinaryOp::And, env[key].id, inverse)?;
            let value = self.rtl.binary(BinaryOp::Or, held, replacement)?;
            env.insert(
                key.clone(),
                LoweredExpr {
                    id: value,
                    width: total_width,
                    signed: self.is_signed(key),
                },
            );
            return Ok(());
        }
        let (lsb, width) = static_select(select, total_width)?;
        let value = self.resize(value, width, self.is_signed(key))?;
        if lsb == 0 && width == total_width {
            env.insert(key.clone(), value);
            return Ok(());
        }
        let current = env[key];
        let mut parts = Vec::new();
        let high_lsb = lsb + width;
        if high_lsb < total_width {
            parts.push(self.rtl.expression_slice(
                current.id,
                high_lsb,
                BitWidth::new(total_width - high_lsb)?,
            )?);
        }
        parts.push(value.id);
        if lsb != 0 {
            parts.push(
                self.rtl
                    .expression_slice(current.id, 0, BitWidth::new(lsb)?)?,
            );
        }
        env.insert(
            key.clone(),
            LoweredExpr {
                id: self.rtl.concat(parts)?,
                width: total_width,
                signed: self.is_signed(key),
            },
        );
        Ok(())
    }

    fn resize(
        &mut self,
        value: LoweredExpr,
        width: u32,
        signed: bool,
    ) -> Result<LoweredExpr, ImportError> {
        if value.width == width {
            return Ok(LoweredExpr { signed, ..value });
        }
        if value.width > width {
            return Ok(LoweredExpr {
                id: self
                    .rtl
                    .expression_slice(value.id, 0, BitWidth::new(width)?)?,
                width,
                signed,
            });
        }
        let extension_width = width - value.width;
        let extension = if value.signed {
            let sign = self
                .rtl
                .expression_slice(value.id, value.width - 1, BitWidth::new(1)?)?;
            let mut bits = Vec::with_capacity(extension_width as usize);
            bits.resize(extension_width as usize, sign);
            self.rtl.concat(bits)?
        } else {
            self.constant(extension_width, 0).id
        };
        Ok(LoweredExpr {
            id: self.rtl.concat(vec![extension, value.id])?,
            width,
            signed,
        })
    }

    fn boolean(&mut self, value: LoweredExpr) -> Result<LoweredExpr, ImportError> {
        if value.width == 1 {
            Ok(LoweredExpr {
                signed: false,
                ..value
            })
        } else {
            Ok(LoweredExpr {
                id: self.rtl.unary(UnaryOp::ReduceOr, value.id)?,
                width: 1,
                signed: false,
            })
        }
    }

    fn constant(&mut self, width: u32, value: u64) -> LoweredExpr {
        LoweredExpr {
            id: self.rtl.constant(Constant::from_u64(
                BitWidth::new(width).expect("lowered expression widths are non-zero"),
                value,
            )),
            width,
            signed: false,
        }
    }

    fn read_env(&mut self) -> Result<Env, ImportError> {
        let entries = self
            .signal_order
            .iter()
            .map(|key| {
                (
                    key.clone(),
                    self.signals[key],
                    self.widths[key],
                    self.signed[key],
                )
            })
            .collect::<Vec<_>>();
        entries
            .into_iter()
            .map(|(key, signal, width, signed)| {
                Ok((
                    key,
                    LoweredExpr {
                        id: self.rtl.read(signal)?,
                        width,
                        signed,
                    },
                ))
            })
            .collect()
    }

    fn destination_key(&self, destination: &AssignDestination) -> Result<SignalKey, ImportError> {
        let key = self.key_from_index(destination.id, &destination.index)?;
        if !self.signals.contains_key(&key) && !self.widths.contains_key(&key) {
            return Err(ImportError::UnsupportedBehavior(format!(
                "assignment to non-runtime variable {}",
                self.signal_name(&key)
            )));
        }
        Ok(key)
    }

    fn signal(&self, key: &SignalKey) -> Result<SignalId, ImportError> {
        self.signals.get(key).copied().ok_or_else(|| {
            ImportError::UnsupportedBehavior(format!(
                "variable {} has no RTL signal",
                self.signal_name(key)
            ))
        })
    }

    fn width(&self, key: &SignalKey) -> Result<u32, ImportError> {
        self.widths.get(key).copied().ok_or_else(|| {
            ImportError::UnsupportedBehavior(format!(
                "variable {} has no width",
                self.signal_name(key)
            ))
        })
    }

    fn is_signed(&self, key: &SignalKey) -> bool {
        self.signed.get(key).copied().unwrap_or(false)
    }

    fn port_element_key(&self, id: VarId, element: usize) -> Result<SignalKey, ImportError> {
        self.keys_for_id(id).get(element).cloned().ok_or_else(|| {
            ImportError::UnsupportedBehavior(format!(
                "module instance has too many connections for {}",
                self.variable_name(id)
            ))
        })
    }

    fn keys_for_id(&self, id: VarId) -> Vec<SignalKey> {
        let mut keys = self
            .widths
            .keys()
            .filter(|key| key.id == id)
            .cloned()
            .collect::<Vec<_>>();
        keys.sort();
        keys
    }

    fn array_candidate_keys(
        &self,
        id: VarId,
        index: &VarIndex,
    ) -> Result<Vec<SignalKey>, ImportError> {
        let variable = self
            .source
            .variables
            .get(&id)
            .ok_or_else(|| ImportError::MissingVariable(self.variable_name(id)))?;
        let dimensions = variable
            .r#type
            .array
            .iter()
            .map(|dimension| {
                dimension.ok_or_else(|| ImportError::NonConcreteWidth(self.variable_name(id)))
            })
            .collect::<Result<Vec<_>, _>>()?;
        if index.0.len() != dimensions.len() {
            return Err(ImportError::UnsupportedBehavior(format!(
                "whole or partially indexed unpacked array {}",
                self.variable_name(id)
            )));
        }

        let mut static_indices = Vec::with_capacity(index.0.len());
        for (expression, dimension) in index.0.iter().zip(dimensions) {
            let Some(value) = evaluated_u64(expression) else {
                static_indices.push(None);
                continue;
            };
            let value = usize::try_from(value).map_err(|_| {
                ImportError::UnsupportedBehavior("unpacked array index overflow".into())
            })?;
            if value >= dimension {
                return Err(ImportError::UnsupportedBehavior(format!(
                    "unpacked array index {value} exceeds dimension {dimension} of {}",
                    self.variable_name(id)
                )));
            }
            static_indices.push(Some(value));
        }

        let candidates = self
            .keys_for_id(id)
            .into_iter()
            .filter(|key| {
                static_indices
                    .iter()
                    .zip(&key.index)
                    .all(|(expected, actual)| expected.is_none_or(|expected| expected == *actual))
            })
            .collect::<Vec<_>>();
        if candidates.is_empty() {
            return Err(ImportError::UnsupportedBehavior(format!(
                "unpacked array {} has no runtime elements selectable by its index",
                self.variable_name(id)
            )));
        }
        Ok(candidates)
    }

    fn lower_array_elements(
        &mut self,
        id: VarId,
        index: &VarIndex,
        env: &Env,
    ) -> Result<Vec<(SignalKey, LoweredExpr)>, ImportError> {
        let mut temporary = env.clone();
        let mut effects = DrivenBits::default();
        let elements =
            self.lower_array_elements_effects(id, index, &mut temporary, &mut effects)?;
        if !effects.ranges.is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "array index effects in a read-only expression".into(),
            ));
        }
        Ok(elements)
    }

    fn lower_array_elements_effects(
        &mut self,
        id: VarId,
        index: &VarIndex,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<Vec<(SignalKey, LoweredExpr)>, ImportError> {
        self.lower_array_candidates_effects(id, index, self.keys_for_id(id), env, effects)
    }

    fn lower_array_candidates_effects(
        &mut self,
        id: VarId,
        index: &VarIndex,
        candidates: Vec<SignalKey>,
        env: &mut Env,
        effects: &mut DrivenBits,
    ) -> Result<Vec<(SignalKey, LoweredExpr)>, ImportError> {
        let variable = self
            .source
            .variables
            .get(&id)
            .ok_or_else(|| ImportError::MissingVariable(self.variable_name(id)))?;
        let dimensions = variable
            .r#type
            .array
            .iter()
            .map(|dimension| {
                dimension.ok_or_else(|| ImportError::NonConcreteWidth(self.variable_name(id)))
            })
            .collect::<Result<Vec<_>, _>>()?;
        if index.0.len() != dimensions.len() {
            return Err(ImportError::UnsupportedBehavior(format!(
                "whole or partially indexed unpacked array {}",
                self.variable_name(id)
            )));
        }

        let mut indices = Vec::with_capacity(index.0.len());
        for (expression, dimension) in index.0.iter().zip(&dimensions) {
            if let Some(value) = evaluated_u64(expression) {
                let value = usize::try_from(value).map_err(|_| {
                    ImportError::UnsupportedBehavior("unpacked array index overflow".into())
                })?;
                if value >= *dimension {
                    return Err(ImportError::UnsupportedBehavior(format!(
                        "unpacked array index {value} exceeds dimension {dimension} of {}",
                        self.variable_name(id)
                    )));
                }
                indices.push(LoweredArrayIndex::Static(value));
            } else {
                indices.push(LoweredArrayIndex::Dynamic(
                    self.lower_comb_expression(expression, env, effects)?,
                ));
            }
        }

        let mut elements = Vec::new();
        for key in candidates {
            let mut condition = None;
            let mut matches = true;
            for (index, candidate) in indices.iter().zip(&key.index) {
                match index {
                    LoweredArrayIndex::Static(value) => matches &= value == candidate,
                    LoweredArrayIndex::Dynamic(value) => {
                        let candidate = u64::try_from(*candidate).map_err(|_| {
                            ImportError::UnsupportedBehavior("unpacked array index overflow".into())
                        })?;
                        // A signed address has one fewer non-negative magnitude bit.
                        // Do not alias a negative index to an upper array element.
                        let magnitude = value.width - u32::from(value.signed);
                        if magnitude < u64::BITS && candidate >= (1_u64 << magnitude) {
                            matches = false;
                            break;
                        }
                        let candidate = self.constant(value.width, candidate);
                        let equals = self.lower_binary(Op::Eq, *value, candidate, 1, false)?;
                        condition = Some(match condition {
                            Some(previous) => {
                                self.lower_binary(Op::LogicAnd, previous, equals, 1, false)?
                            }
                            None => equals,
                        });
                    }
                }
                if !matches {
                    break;
                }
            }
            if matches {
                elements.push((
                    key,
                    condition.expect("dynamic array access has a dynamic index"),
                ));
            }
        }
        if elements.is_empty() {
            return Err(ImportError::UnsupportedBehavior(format!(
                "unpacked array {} has no runtime elements selectable by its index",
                self.variable_name(id)
            )));
        }
        Ok(elements)
    }

    fn key_from_index(&self, id: VarId, index: &VarIndex) -> Result<SignalKey, ImportError> {
        let variable = self
            .source
            .variables
            .get(&id)
            .ok_or_else(|| ImportError::MissingVariable(self.variable_name(id)))?;
        let indices = index
            .0
            .iter()
            .map(|value| {
                let index = static_array_index(value).map_err(|error| {
                    if self.memory_policy(id) == MemoryInferencePolicy::Forbidden {
                        ImportError::UnsupportedBehavior(format!(
                            "dynamic access to unpacked array {} cannot be lowered because block-memory inference is forbidden",
                            self.variable_name(id)
                        ))
                    } else {
                        error
                    }
                })?;
                usize::try_from(index).map_err(|_| {
                    ImportError::UnsupportedBehavior("unpacked array index overflow".into())
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        if indices.len() != variable.r#type.array.dims() {
            return Err(ImportError::UnsupportedBehavior(format!(
                "whole or partially indexed unpacked array {}",
                self.variable_name(id)
            )));
        }
        for (index, dimension) in indices.iter().zip(variable.r#type.array.iter()) {
            let Some(dimension) = dimension else {
                return Err(ImportError::NonConcreteWidth(self.variable_name(id)));
            };
            if index >= dimension {
                return Err(ImportError::UnsupportedBehavior(format!(
                    "unpacked array index {index} exceeds dimension {dimension} of {}",
                    self.variable_name(id)
                )));
            }
        }
        Ok(SignalKey { id, index: indices })
    }

    fn variable_type(&self, id: VarId) -> Result<&Type, ImportError> {
        self.source
            .variables
            .get(&id)
            .map(|variable| &variable.r#type)
            .ok_or_else(|| ImportError::MissingVariable(self.variable_name(id)))
    }

    fn variable_name(&self, id: VarId) -> String {
        self.source
            .variables
            .get(&id)
            .map_or_else(|| id.to_string(), |variable| variable.path.to_string())
    }

    fn signal_name(&self, key: &SignalKey) -> String {
        self.signals.get(key).map_or_else(
            || indexed_name(&self.variable_name(key.id), &key.index),
            |signal| {
                self.rtl.signals()[signal.index() as usize]
                    .name()
                    .to_owned()
            },
        )
    }

    fn unsupported_expression(op: Op) -> ImportError {
        ImportError::UnsupportedBehavior(format!("expression operator `{op}`"))
    }
}

fn memory_candidates(
    source: &Module,
    policies: &HashMap<VarId, MemoryInferencePolicy>,
) -> HashSet<VarId> {
    let arrays = source
        .variables
        .values()
        .filter(|variable| {
            variable.kind == VarKind::Variable
                && variable.affiliation != veryl_analyzer::symbol::Affiliation::AlwaysFf
                && !variable.r#type.array.is_empty()
        })
        .map(|variable| variable.id)
        .collect::<HashSet<_>>();
    let mut candidates = policies
        .iter()
        .filter_map(|(id, policy)| {
            matches!(
                policy,
                MemoryInferencePolicy::Required
                    | MemoryInferencePolicy::Block
                    | MemoryInferencePolicy::Distributed
            )
            .then_some(*id)
        })
        .collect::<HashSet<_>>();
    let mut accesses = HashMap::<VarId, (bool, bool, bool)>::new();
    for declaration in &source.declarations {
        let Declaration::Ff(ff) = declaration else {
            continue;
        };
        for statement in &ff.statements {
            for pattern in memory_statement_patterns(statement, &arrays) {
                let (memory, address, write) = match &pattern {
                    MemoryStatementPattern::Write {
                        memory, address, ..
                    } => (*memory, address, true),
                    MemoryStatementPattern::Read {
                        memory, address, ..
                    } => (*memory, address, false),
                };
                let access = accesses.entry(memory).or_default();
                access.0 |= write;
                access.1 |= !write;
                access.2 |= static_array_index(address).is_err();
            }
        }
    }
    candidates.extend(accesses.into_iter().filter_map(
        |(memory, (has_write, has_read, has_dynamic_address))| {
            (has_write
                && has_read
                && has_dynamic_address
                && policies.get(&memory).copied().unwrap_or_default()
                    != MemoryInferencePolicy::Forbidden)
                .then_some(memory)
        },
    ));
    candidates
}

fn memory_inference_policies(
    source: &Module,
) -> Result<HashMap<VarId, MemoryInferencePolicy>, ImportError> {
    let mut policies = HashMap::new();
    for variable in source
        .variables
        .values()
        .filter(|variable| variable.kind == VarKind::Variable)
    {
        for attribute in attribute_table::get(&variable.token.beg) {
            let VerylAttribute::Sv(value) = attribute else {
                continue;
            };
            let raw = veryl_parser::resource_table::get_str_value(value)
                .ok_or(ImportError::MissingResourceString)?;
            let text = veryl_analyzer::value::unescape_string_literal_to_string(&raw);
            let Some(value) = memory_policy_value(&text) else {
                continue;
            };
            let policy = match value {
                "preferred" => MemoryInferencePolicy::Preferred,
                "required" => MemoryInferencePolicy::Required,
                "forbidden" => MemoryInferencePolicy::Forbidden,
                "block" => MemoryInferencePolicy::Block,
                "distributed" => MemoryInferencePolicy::Distributed,
                _ => {
                    return Err(ImportError::InvalidMemoryInferencePolicy {
                        memory: variable.path.to_string(),
                        value: value.into(),
                    });
                }
            };
            if variable.r#type.array.is_empty() {
                return Err(ImportError::UnsupportedBehavior(format!(
                    "`struo_memory` policy on non-array variable {}",
                    variable.path
                )));
            }
            if let Some(previous) = policies.insert(variable.id, policy)
                && previous != policy
            {
                return Err(ImportError::ConflictingMemoryInferencePolicies(
                    variable.path.to_string(),
                ));
            }
        }
    }
    Ok(policies)
}

fn memory_policy_value(text: &str) -> Option<&str> {
    let (key, value) = text.split_once('=')?;
    if key.trim() != "struo_memory" {
        return None;
    }
    let value = value.trim();
    Some(
        value
            .strip_prefix('"')
            .and_then(|value| value.strip_suffix('"'))
            .unwrap_or(value)
            .trim(),
    )
}

enum MemoryStatementPattern {
    Write {
        memory: VarId,
        address: Expression,
        data: Expression,
        enable: Vec<Expression>,
    },
    Read {
        memory: VarId,
        address: Expression,
        data: VarId,
        enable: Vec<Expression>,
    },
}

fn memory_statement_patterns(
    statement: &Statement,
    memories: &HashSet<VarId>,
) -> Vec<MemoryStatementPattern> {
    memory_statement_patterns_with_enable(statement, memories, &[])
}

fn memory_statement_patterns_with_enable(
    statement: &Statement,
    memories: &HashSet<VarId>,
    inherited_enable: &[Expression],
) -> Vec<MemoryStatementPattern> {
    if let Statement::If(branch) = statement {
        if branch.false_side.is_empty() {
            let mut enable = inherited_enable.to_vec();
            enable.push(branch.cond.clone());
            return branch
                .true_side
                .iter()
                .flat_map(|statement| {
                    memory_statement_patterns_with_enable(statement, memories, &enable)
                })
                .collect();
        }
        if let ([true_statement], [false_statement]) =
            (branch.true_side.as_slice(), branch.false_side.as_slice())
        {
            let mut patterns = Vec::new();
            let mut true_enable = inherited_enable.to_vec();
            true_enable.push(branch.cond.clone());
            if let Some(pattern) = direct_memory_statement(true_statement, memories, true_enable) {
                patterns.push(pattern);
            }
            // A block-RAM port remains physically readable while writing. The
            // false-side read therefore needs no separate enable; consumers
            // observe it only on cycles where the source selected that arm.
            if let Some(pattern) =
                direct_memory_statement(false_statement, memories, inherited_enable.to_vec())
            {
                patterns.push(pattern);
            }
            return patterns;
        }
    }
    direct_memory_statement(statement, memories, inherited_enable.to_vec())
        .into_iter()
        .collect()
}

fn direct_memory_statement(
    statement: &Statement,
    memories: &HashSet<VarId>,
    enable: Vec<Expression>,
) -> Option<MemoryStatementPattern> {
    let Statement::Assign(assign) = statement else {
        return None;
    };
    let [destination] = assign.dst.as_slice() else {
        return None;
    };
    if memories.contains(&destination.id)
        && destination.index.0.len() == 1
        && destination.select.is_empty()
    {
        return Some(MemoryStatementPattern::Write {
            memory: destination.id,
            address: destination.index.0[0].clone(),
            data: assign.expr.clone(),
            enable,
        });
    }
    if !destination.index.0.is_empty() || !destination.select.is_empty() {
        return None;
    }
    let Expression::Term(factor) = &assign.expr else {
        return None;
    };
    let Factor::Variable(memory, index, select, _) = factor.as_ref() else {
        return None;
    };
    if !memories.contains(memory) || index.0.len() != 1 || !select.is_empty() {
        return None;
    }
    Some(MemoryStatementPattern::Read {
        memory: *memory,
        address: index.0[0].clone(),
        data: destination.id,
        enable,
    })
}

fn copy_constant(value: &Constant) -> Constant {
    let mut words = vec![0; value.width().get().div_ceil(64) as usize];
    for bit in 0..value.width().get() {
        if value.bit(bit) {
            words[bit as usize / 64] |= 1 << (bit % 64);
        }
    }
    Constant::new(value.width(), words)
}

fn remap_memory_port(
    port: &MemoryPort,
    expressions: &HashMap<ExprId, ExprId>,
    signals: &HashMap<SignalId, SignalId>,
) -> MemoryPort {
    MemoryPort {
        read_address: expressions[&port.read_address],
        read_data: signals[&port.read_data],
        read_enable: port.read_enable.map(|enable| Enable {
            signal: signals[&enable.signal],
            polarity: enable.polarity,
        }),
        write_address: expressions[&port.write_address],
        write_data: expressions[&port.write_data],
        write_enable: Enable {
            signal: signals[&port.write_enable.signal],
            polarity: port.write_enable.polarity,
        },
        clock: signals[&port.clock],
        edge: port.edge,
    }
}

fn reject_nested_instances(module: &RtlModule) -> Result<(), ImportError> {
    if module.instances().is_empty() {
        Ok(())
    } else {
        Err(ImportError::UnsupportedBehavior(
            "unflattened nested module instance".into(),
        ))
    }
}

fn array_indices(r#type: &Type, name: &str) -> Result<Vec<Vec<usize>>, ImportError> {
    let dimensions = r#type
        .array
        .iter()
        .map(|dimension| dimension.ok_or_else(|| ImportError::NonConcreteWidth(name.into())))
        .collect::<Result<Vec<_>, _>>()?;
    let mut indices = vec![Vec::new()];
    for dimension in dimensions {
        let mut expanded = Vec::with_capacity(indices.len().saturating_mul(dimension));
        for prefix in indices {
            for index in 0..dimension {
                let mut element = prefix.clone();
                element.push(index);
                expanded.push(element);
            }
        }
        indices = expanded;
    }
    Ok(indices)
}

fn indexed_name(name: &str, indices: &[usize]) -> String {
    indices
        .iter()
        .fold(name.to_owned(), |name, index| format!("{name}[{index}]"))
}

fn whole_array_variable(expression: &Expression) -> Option<VarId> {
    let Expression::Term(factor) = expression else {
        return None;
    };
    let Factor::Variable(id, index, select, _) = factor.as_ref() else {
        return None;
    };
    (index.0.is_empty() && select.is_empty()).then_some(*id)
}

fn value_type(r#type: &Type, name: &str) -> Result<ValueType, ImportError> {
    Ok(ValueType {
        width: BitWidth::new(concrete_width(r#type, name)?)?,
        signed: r#type.signed,
        state: if r#type.is_4state() {
            StateDomain::FourState
        } else {
            StateDomain::TwoState
        },
    })
}

fn concrete_width(r#type: &Type, name: &str) -> Result<u32, ImportError> {
    let width = r#type
        .total_width()
        .ok_or_else(|| ImportError::NonConcreteWidth(name.into()))?;
    u32::try_from(width).map_err(|_| ImportError::WidthTooLarge(name.into()))
}

fn binary_signed(op: Op, comptime: &Comptime) -> bool {
    !is_boolean_operator(op) && comptime.expr_context.signed
}

fn is_boolean_operator(op: Op) -> bool {
    matches!(
        op,
        Op::Eq
            | Op::Ne
            | Op::EqWildcard
            | Op::NeWildcard
            | Op::Less
            | Op::LessEq
            | Op::Greater
            | Op::GreaterEq
            | Op::LogicAnd
            | Op::LogicOr
    )
}

fn context_width(comptime: &Comptime) -> Result<u32, ImportError> {
    let width = comptime
        .r#type
        .total_width()
        .ok_or_else(|| ImportError::NonConcreteWidth("context-sized expression".into()))?
        .max(comptime.expr_context.width);
    u32::try_from(width).map_err(|_| ImportError::WidthTooLarge("context-sized expression".into()))
}

fn binary_width(op: Op, comptime: &Comptime) -> Result<u32, ImportError> {
    if is_boolean_operator(op) {
        Ok(1)
    } else {
        context_width(comptime)
    }
}

fn contains_destination(statements: &[Statement], id: VarId) -> bool {
    statements.iter().any(|statement| match statement {
        Statement::Assign(assign) => assign.dst.iter().any(|dst| dst.id == id),
        Statement::If(branch) => {
            contains_destination(&branch.true_side, id)
                || contains_destination(&branch.false_side, id)
        }
        Statement::Case(case) => {
            contains_destination(&case.default, id)
                || case
                    .arms
                    .iter()
                    .any(|arm| contains_destination(&arm.body, id))
        }
        Statement::For(statement) => contains_destination(&statement.body, id),
        _ => false,
    })
}

// Substitute a static induction value before lowering each iteration. Recompute
// index expressions instead of leaving AIR's pre-substitution value cache stale.
fn substitute_induction(
    expression: &mut Expression,
    id: VarId,
    value: usize,
) -> Result<(), ImportError> {
    match expression {
        Expression::Term(factor) => match factor.as_mut() {
            Factor::Variable(var, index, select, comptime) => {
                if *var == id {
                    if !index.0.is_empty() || !select.is_empty() {
                        return Err(ImportError::UnsupportedBehavior(
                            "selected loop induction variable".into(),
                        ));
                    }
                    let mut replacement = comptime.clone();
                    replacement.value = ValueVariant::Numeric(veryl_analyzer::value::Value::new(
                        value as u64,
                        concrete_width(&comptime.r#type, "loop variable")? as usize,
                        comptime.r#type.signed,
                    ));
                    replacement.is_const = true;
                    **factor = Factor::Value(replacement);
                } else {
                    substitute_access(index, select, id, value)?;
                }
            }
            Factor::FunctionCall(call) => substitute_call(call, id, value)?,
            Factor::SystemFunctionCall(call) => substitute_system_call(call, id, value)?,
            _ => (),
        },
        Expression::Unary(_, a, _) => substitute_induction(a, id, value)?,
        Expression::Binary(a, _, b, _) => {
            substitute_induction(a, id, value)?;
            substitute_induction(b, id, value)?;
        }
        Expression::Ternary(a, b, c, _) => {
            substitute_induction(a, id, value)?;
            substitute_induction(b, id, value)?;
            substitute_induction(c, id, value)?;
        }
        Expression::Concatenation(parts, _) => {
            for (part, repeat) in parts {
                substitute_induction(part, id, value)?;
                if let Some(repeat) = repeat {
                    substitute_induction(repeat, id, value)?;
                }
            }
        }
        Expression::StructConstructor(_, fields, _) => {
            for (_, field) in fields {
                substitute_induction(field, id, value)?;
            }
        }
        Expression::ArrayLiteral(items, _) => {
            for item in items {
                match item {
                    ArrayLiteralItem::Value(expression, repeat) => {
                        substitute_induction(expression, id, value)?;
                        if let Some(repeat) = repeat {
                            substitute_induction(repeat, id, value)?;
                        }
                    }
                    ArrayLiteralItem::Defaul(expression) => {
                        substitute_induction(expression, id, value)?;
                    }
                }
            }
        }
    }
    // eval_value never invents values for unresolved runtime variables. Only
    // use fully known numbers; keep the original expression otherwise.
    if let Some(number) = expression.eval_value(&mut veryl_analyzer::Context::default())
        && !number.is_xz()
    {
        expression.comptime_mut().value = ValueVariant::Numeric(number);
        expression.comptime_mut().is_const = true;
    }
    Ok(())
}

fn substitute_access(
    index: &mut VarIndex,
    select: &mut VarSelect,
    id: VarId,
    value: usize,
) -> Result<(), ImportError> {
    for expression in index.0.iter_mut().chain(&mut select.0) {
        substitute_induction(expression, id, value)?;
    }
    if let Some((_, expression)) = &mut select.1 {
        substitute_induction(expression, id, value)?;
    }
    Ok(())
}

fn substitute_system_call(
    call: &mut veryl_analyzer::ir::SystemFunctionCall,
    id: VarId,
    value: usize,
) -> Result<(), ImportError> {
    use veryl_analyzer::ir::SystemFunctionKind;
    match &mut call.kind {
        SystemFunctionKind::Signed(input)
        | SystemFunctionKind::Unsigned(input)
        | SystemFunctionKind::Onehot(input)
        | SystemFunctionKind::Clog2(input)
        | SystemFunctionKind::Bits(input) => substitute_induction(&mut input.0, id, value),
        SystemFunctionKind::Size(input, dimension) => {
            substitute_induction(&mut input.0, id, value)?;
            if let Some(dimension) = dimension {
                substitute_induction(&mut dimension.0, id, value)?;
            }
            Ok(())
        }
        _ => Err(ImportError::UnsupportedBehavior(
            "system task in unrolled loop".into(),
        )),
    }
}

fn substitute_call(
    call: &mut veryl_analyzer::ir::FunctionCall,
    id: VarId,
    value: usize,
) -> Result<(), ImportError> {
    for expression in call.inputs.values_mut() {
        substitute_induction(expression, id, value)?;
    }
    for destinations in call.outputs.values_mut() {
        for dst in destinations {
            substitute_access(&mut dst.index, &mut dst.select, id, value)?;
        }
    }
    Ok(())
}

fn substitute_statements(
    statements: &mut [Statement],
    id: VarId,
    value: usize,
) -> Result<(), ImportError> {
    for statement in statements {
        match statement {
            Statement::Assign(assign) => {
                substitute_induction(&mut assign.expr, id, value)?;
                for dst in &mut assign.dst {
                    substitute_access(&mut dst.index, &mut dst.select, id, value)?;
                }
            }
            Statement::If(branch) => {
                substitute_induction(&mut branch.cond, id, value)?;
                substitute_statements(&mut branch.true_side, id, value)?;
                substitute_statements(&mut branch.false_side, id, value)?;
            }
            Statement::Case(case) => {
                substitute_induction(&mut case.case_target, id, value)?;
                for arm in &mut case.arms {
                    for pattern in &mut arm.patterns {
                        match pattern {
                            CasePattern::Eq(e) => substitute_induction(e, id, value)?,
                            CasePattern::Range { lo, hi, .. } => {
                                substitute_induction(lo, id, value)?;
                                substitute_induction(hi, id, value)?;
                            }
                        }
                    }
                    substitute_statements(&mut arm.body, id, value)?;
                }
                substitute_statements(&mut case.default, id, value)?;
            }
            Statement::For(nested) => {
                use veryl_analyzer::ir::{ForBound, ForRange};
                let (ForRange::Forward { start, end, .. }
                | ForRange::Reverse { start, end, .. }
                | ForRange::Stepped { start, end, .. }) = &mut nested.range;
                for bound in [start, end] {
                    if let ForBound::Expression(e) = bound {
                        substitute_induction(e, id, value)?;
                    }
                }
                substitute_statements(&mut nested.body, id, value)?;
            }
            Statement::FunctionCall(call) => substitute_call(call, id, value)?,
            Statement::SystemFunctionCall(call) => substitute_system_call(call, id, value)?,
            Statement::Null | Statement::Break => (),
            _ => {
                return Err(ImportError::UnsupportedBehavior(
                    "unsupported statement in unrolled loop".into(),
                ));
            }
        }
    }
    Ok(())
}

fn constant_value(expression: &Expression) -> Result<u64, ImportError> {
    evaluated_u64(expression)
        .ok_or_else(|| ImportError::UnsupportedBehavior("non-constant replication count".into()))
}

fn static_array_index(expression: &Expression) -> Result<u64, ImportError> {
    evaluated_u64(expression)
        .ok_or_else(|| ImportError::UnsupportedBehavior("dynamic unpacked array index".into()))
}

fn has_dynamic_array_index(index: &VarIndex) -> bool {
    index.0.iter().any(|index| evaluated_u64(index).is_none())
}

fn evaluated_u64(expression: &Expression) -> Option<u64> {
    // AIR can retain a representative numeric value for a runtime expression,
    // notably A[index] with a constant array. It is not a compile-time address.
    if !expression.comptime().is_const {
        return None;
    }
    expression
        .comptime()
        .get_value()
        .ok()
        .and_then(veryl_analyzer::value::Value::to_u64)
}

fn dynamic_packed_select(select: &VarSelect) -> bool {
    select.0.len() == 1 && evaluated_u64(&select.0[0]).is_none()
}

fn selected_width(select: &VarSelect, total: u32) -> Result<u32, ImportError> {
    if !dynamic_packed_select(select) {
        return static_select(select, total).map(|(_, width)| width);
    }
    match &select.1 {
        None => Ok(1),
        Some((VarSelectOp::PlusColon | VarSelectOp::MinusColon | VarSelectOp::Step, count)) => {
            evaluated_u64(count)
                .and_then(|n| u32::try_from(n).ok())
                .filter(|n| *n > 0 && *n <= total)
                .ok_or_else(|| {
                    ImportError::UnsupportedBehavior(
                        "dynamic packed select requires a fixed width".into(),
                    )
                })
        }
        _ => Err(ImportError::UnsupportedBehavior(
            "dynamic packed select direction is not supported".into(),
        )),
    }
}

fn driven_select(select: &VarSelect, total: u32) -> Result<(u32, u32), ImportError> {
    if dynamic_packed_select(select) {
        selected_width(select, total)?;
        Ok((0, total))
    } else {
        static_select(select, total)
    }
}

fn static_select(select: &VarSelect, source_width: u32) -> Result<(u32, u32), ImportError> {
    if select.0.is_empty() {
        return Ok((0, source_width));
    }
    if select.0.len() != 1 {
        return Err(ImportError::UnsupportedBehavior(
            "multi-dimensional packed select".into(),
        ));
    }
    let first = u32::try_from(evaluated_u64(&select.0[0]).ok_or_else(|| {
        ImportError::UnsupportedBehavior("dynamic packed select is unsupported here".into())
    })?)
    .map_err(|_| ImportError::UnsupportedBehavior("packed select index overflow".into()))?;
    let (lsb, width) = if let Some((operation, end)) = &select.1 {
        let end = u32::try_from(evaluated_u64(end).ok_or_else(|| {
            ImportError::UnsupportedBehavior("dynamic packed select width is unsupported".into())
        })?)
        .map_err(|_| ImportError::UnsupportedBehavior("packed select bound overflow".into()))?;
        match operation {
            VarSelectOp::Colon => (first.min(end), first.abs_diff(end) + 1),
            VarSelectOp::PlusColon => (first, end),
            VarSelectOp::MinusColon => (
                first
                    .checked_add(1)
                    .and_then(|value| value.checked_sub(end))
                    .ok_or_else(|| {
                        ImportError::UnsupportedBehavior(
                            "packed minus-colon select underflow".into(),
                        )
                    })?,
                end,
            ),
            VarSelectOp::Step => (
                first.checked_mul(end).ok_or_else(|| {
                    ImportError::UnsupportedBehavior("packed step select overflow".into())
                })?,
                end,
            ),
        }
    } else {
        (first, 1)
    };
    if width == 0 || lsb.checked_add(width).is_none_or(|end| end > source_width) {
        return Err(ImportError::UnsupportedBehavior(format!(
            "packed select [{lsb} +: {width}] exceeds width {source_width}"
        )));
    }
    Ok((lsb, width))
}

#[cfg(test)]
mod tests {
    use celox::{NativeBackend, Simulator};
    use struo_celox::ecp5_simulator;
    use struo_rtl::{ExprId, ExprKind, Module as RtlModule};
    use struo_synth::synthesize;
    use struo_target_ecp5::{
        Ecp5Cell, Ecp5MemoryImplementation, JtaggBinding, map_to_ecp5, map_to_ecp5_with_jtagg,
    };

    use crate::{ImportError, analyze_and_lower};

    const SOURCE: &str = r"
module Top (
    clk: input clock_posedge,
    rst_n: input reset_async_low,
    a: input logic<8>,
    b: input logic<8>,
    select: input logic,
    q: output logic<8>,
    flag: output logic,
) {
    var state: logic<8>;

    always_ff (clk, rst_n) {
        if_reset {
            state = 8'h00;
        } else {
            if select {
                state = a + b;
            } else {
                state = a - b;
            }
        }
    }

    always_comb {
        q = state;
        flag = (state >= 8'h80) || (state == 8'h00);
    }
}
";

    const ADD_WITH_CARRY_SOURCE: &str = r"
module AddWithCarry (
    a    : input  logic<8>,
    b    : input  logic<8>,
    carry: input  logic,
    sum  : output logic<8>,
) {
    always_comb {
        sum = a + b + carry;
    }
}
";

    const SHIFT_CONTEXT_SOURCE: &str = r"
module ShiftContext (
    value           : input  logic<8>,
    signed_value    : input  signed logic<8>,
    amount          : input  logic<4>,
    left            : output logic<16>,
    logical_right   : output signed logic<16>,
    arithmetic_right: output signed logic<16>,
) {
    always_comb {
        left             = value << amount;
        logical_right    = signed_value >> amount;
        arithmetic_right = signed_value >>> amount;
    }
}
";

    const HIERARCHY_SOURCE: &str = r"
interface ByteBus {
    var request : logic<8>;
    var response: logic<8>;

    modport initiator {
        request : output,
        response: input ,
    }
    modport target {
        request : input ,
        response: output,
    }
}

module Increment (
    bus: modport ByteBus::target,
) {
    always_comb {
        bus.response = bus.request + 8'h01;
    }
}

module HierarchyTop (
    value : input  logic<8>,
    result: output logic<8>,
) {
    inst bus: ByteBus;
    inst increment: Increment (
        bus: bus,
    );

    always_comb {
        bus.request = value;
        result      = bus.response;
    }
}
";

    const CASE_SOURCE: &str = r"
module CaseTop (
    select : input  logic<3>,
    value  : input  logic<8>,
    decoded: output logic<8>,
) {
    always_comb {
        case select {
            3'd0, 3'd2: decoded = value;
            3'd3..=3'd5: decoded = value + 8'h01;
            default: decoded = 8'hff;
        }
    }
}
";

    const CASE_FIRST_MATCH_SOURCE: &str = r"
module CaseFirstMatchTop (
    select : input  logic<4>,
    base   : input  logic<8>,
    decoded: output logic<8>,
    side   : output logic,
) {
    always_comb {
        decoded = base;
        side    = 1'b0;
        case select {
            4'd3, 4'd7: decoded = 8'h11;
            4'd3: {
                decoded = 8'h22;
                side    = 1'b1;
            }
            4'd4..=4'd8: {
                decoded = 8'h33;
                side    = 1'b1;
            }
            4'd5: {
                decoded = 8'h44;
                side    = 1'b0;
            }
            default: {
                decoded = 8'hee;
                side    = 1'b1;
            }
        }
    }
}
";

    const BALANCED_CASE_SOURCE: &str = r"
module BalancedCaseTop (
    select : input  logic<5>,
    values : input  logic<17>,
    decoded: output logic,
) {
    always_comb {
        case select {
            5'd0 : decoded = values[0];
            5'd1 : decoded = values[1];
            5'd2 : decoded = values[2];
            5'd3 : decoded = values[3];
            5'd4 : decoded = values[4];
            5'd5 : decoded = values[5];
            5'd6 : decoded = values[6];
            5'd7 : decoded = values[7];
            5'd8 : decoded = values[8];
            5'd9 : decoded = values[9];
            5'd10: decoded = values[10];
            5'd11: decoded = values[11];
            5'd12: decoded = values[12];
            5'd13: decoded = values[13];
            5'd14: decoded = values[14];
            5'd15: decoded = values[15];
            default: decoded = values[16];
        }
    }
}
";

    const GENERATE_FOR_SOURCE: &str = r"
module Increment (
    value : input  logic<8>,
    result: output logic<8>,
) {
    always_comb {
        result = value + 8'h01;
    }
}

module GenerateForBank::<PORTS: u32 = 2> (
    values : input  logic<PORTS * 8>,
    results: output logic<PORTS * 8>,
) {
    for i in 0..PORTS :lane {
        inst increment: Increment (
            value : values[i * 8+: 8] ,
            result: results[i * 8+: 8],
        );
    }
}

module GenerateForTop (
    values : input  logic<32>,
    results: output logic<32>,
) {
    inst bank: GenerateForBank::<4> (
        values : values ,
        results: results,
    );
}
";

    const UNPACKED_ARRAY_SOURCE: &str = r"
interface ByteLane {
    var request : logic<8>;
    var response: logic<8>;

    modport target {
        request : input ,
        response: output,
    }
}

module UnpackedArrayTop::<PORTS: u32 = 4> (
    clk   : input   clock_posedge            ,
    rst_n : input   reset_async_low          ,
    enable: input   logic [PORTS]            ,
    lanes : modport ByteLane::target [PORTS],
) {
    var state: logic<8> [PORTS];

    always_ff (clk, rst_n) {
        if_reset {
            for i in 0..PORTS {
                state[i] = 8'h00;
            }
        } else {
            for i in 0..PORTS {
                if enable[i] {
                    state[i] = lanes[i].request + 8'h01;
                }
            }
        }
    }

    always_comb {
        for i in 0..PORTS {
            lanes[i].response = state[i];
        }
    }
}

module UnpackedInterfaceArrayWrapper (
    clk      : input  clock_posedge  ,
    rst_n    : input  reset_async_low,
    requests : input  logic<32>      ,
    responses: output logic<32>      ,
) {
    inst lanes: ByteLane [4];
    var enable: logic [4];

    inst dut: UnpackedArrayTop::<4> (
        clk   : clk   ,
        rst_n : rst_n ,
        enable: enable,
        lanes : lanes ,
    );

    always_comb {
        for i in 0..4 {
            enable[i]             = 1'b1;
            lanes[i].request      = requests[i * 8+:8];
            responses[i * 8+:8] = lanes[i].response;
        }
    }
}
";

    const MEMORY_SOURCE: &str = r"
module MemoryTop (
    clk          : input  clock_posedge,
    write_enable : input  logic,
    read_address : input  logic<4>,
    write_address: input  logic<4>,
    write_data   : input  logic<8>,
    read_data    : output logic<8>,
) {
    var words: logic<8> [16];

    always_ff (clk) {
        if write_enable {
            words[write_address] = write_data;
        }
    }

    always_ff (clk) {
        read_data = words[read_address];
    }
}
";

    const TRUE_DUAL_PORT_MEMORY_SOURCE: &str = r#"
module TrueDualPortMemoryTop (
    clk_a : input  'a clock,
    clk_b : input  'a clock_negedge,
    addr_a: input  'a logic<4>,
    addr_b: input  'a logic<4>,
    we_a  : input  'a logic,
    we_b  : input  'a logic,
    ce_a  : input  'a logic,
    ce_b  : input  'a logic,
    data_a: input  'a logic<8>,
    data_b: input  'a logic<8>,
    read_a: output 'a logic<8>,
    read_b: output 'a logic<8>,
) {
    #[allow(multiple_assign)]
    #[sv("struo_memory = \"required\"")]
    var words: 'a logic<8> [16];

    always_ff (clk_a) {
        if ce_a {
            if we_a { words[addr_a] = data_a; }
            else { read_a = words[addr_a]; }
        }
    }
    always_ff (clk_b) {
        if ce_b {
            if we_b { words[addr_b] = data_b; }
            else { read_b = words[addr_b]; }
        }
    }
}
"#;

    const REQUIRED_ASYNC_MEMORY_SOURCE: &str = r#"
module RequiredAsyncMemoryTop (
    clk          : input  clock_posedge,
    write_enable : input  logic,
    read_address : input  logic<4>,
    write_address: input  logic<4>,
    write_data   : input  logic<8>,
    read_data    : output logic<8>,
) {
    #[sv("struo_memory = \"required\"")]
    var words: logic<8> [16];

    always_ff (clk) {
        if write_enable {
            words[write_address] = write_data;
        }
    }

    always_comb {
        read_data = words[read_address];
    }
}
"#;

    const DISTRIBUTED_MEMORY_SOURCE: &str = r#"
module DistributedMemoryTop (
    clk          : input  clock_posedge,
    write_enable : input  logic,
    read_address : input  logic<7>,
    write_address: input  logic<7>,
    write_data   : input  logic,
    read_data    : output logic,
) {
    #[sv("struo_memory = \"distributed\"")]
    var words: logic [128];

    always_ff (clk) {
        if write_enable {
            words[write_address] = write_data;
        }
    }

    always_comb {
        read_data = words[read_address];
    }
}
"#;

    const FORBIDDEN_MEMORY_SOURCE: &str = r#"
module ForbiddenMemoryTop (
    clk          : input  clock_posedge,
    write_enable : input  logic,
    read_address : input  logic<4>,
    write_address: input  logic<4>,
    write_data   : input  logic<8>,
    read_data    : output logic<8>,
) {
    #[sv("struo_memory = \"forbidden\"")]
    var words: logic<8> [16];

    always_ff (clk) {
        if write_enable {
            words[write_address] = write_data;
        }
        read_data = words[read_address];
    }
}
"#;

    const I2C_EXPRESSION_SOURCE: &str = r"
module I2cExpressionTop (
    read_data: input  logic<8>,
    bit_index: input  logic<3>,
    state    : output logic<4>,
    drive_low: output logic,
) {
    const STATE_IDLE: logic<4> = 4'h0;
    const STATE_READ: logic<4> = 4'h7;

    always_comb {
        state = STATE_IDLE;
        if read_data[bit_index] {
            state = STATE_READ;
        }
        drive_low = !read_data[bit_index];
    }
}
";

    const UNPACKED_ARRAY_INSTANCE_SOURCE: &str = r"
module ArrayIncrement::<PORTS: u32 = 2> (
    values : input  logic<8> [PORTS],
    results: output logic<8> [PORTS],
) {
    always_comb {
        for i in 0..PORTS {
            results[i] = values[i] + 8'h01;
        }
    }
}

module UnpackedArrayInstanceTop (
    values : input  logic<8> [4],
    results: output logic<8> [4],
) {
    inst increment: ArrayIncrement::<4> (
        values : values ,
        results: results,
    );
}
";

    const NBA_SOURCE: &str = r"
module NbaTop (
    clk    : input  clock_posedge  ,
    rst_n  : input  reset_async_low,
    din    : input  logic<8>       ,
    use_alt: input  logic          ,
    alt    : input  logic<2>       ,
    stage2 : output logic<8>       ,
    flags  : output logic<2>       ,
) {
    var stage1 : logic<8>;
    var echoed : logic<8>;
    var flags_q: logic<2>;

    always_ff (clk, rst_n) {
        if_reset {
            stage1 = 8'h00;
            echoed = 8'h00;
            flags_q = 2'b00;
        } else {
            stage1 = din;
            echoed = stage1 + 8'h01;
            flags_q = 2'b01;
            if use_alt {
                flags_q[1] = alt[0];
                flags_q[0] = alt[1];
            }
        }
    }

    always_comb {
        stage2 = echoed;
        flags  = flags_q;
    }
}
";

    const PARTIAL_MUX_SOURCE: &str = r"
module PartialMuxTop (
    clk    : input  clock_posedge,
    reverse: input  logic,
    value  : input  logic<8>,
    result : output logic<8>,
) {
    var mem_shift_result_q: logic<8>;
    var mem_shift_result  : logic<8>;

    always_comb {
        for i in 0..8 {
            if reverse {
                mem_shift_result[i] = mem_shift_result_q[7 - i];
            } else {
                mem_shift_result[i] = mem_shift_result_q[i];
            }
        }
        result = mem_shift_result;
    }

    always_ff (clk) {
        mem_shift_result_q = value;
    }
}
";

    const BLOCKING_COMB_SOURCE: &str = r"
module BlockingCombTop (
    seed  : input  logic<8>,
    enable: input  logic,
    result: output logic<8>,
) {
    var value     : logic<8>;
    var set_upper : logic;

    always_comb {
        value = seed;
        set_upper = enable && !value[7];
        if set_upper {
            value[7] = 1'b1;
        }
        result = value;
    }
}
";

    const STRUCT_SOURCE: &str = r"
module StructTop (
    header          : input  logic<3>,
    nibble          : input  logic<4>,
    flag            : input  logic,
    data            : input  logic<8>,
    override_nibble : input  logic,
    replacement     : input  logic<4>,
    packed_value    : output logic<16>,
    selected_nibble : output logic<4>,
    selected_flag   : output logic,
) {
    struct Inner {
        nibble: logic<4>,
        flag  : logic,
    }

    struct Payload {
        header: logic<3>,
        inner : Inner,
        data  : logic<8>,
    }

    var payload: Payload;

    always_comb {
        payload = Payload'{
            header: header,
            inner : Inner'{
                nibble: nibble,
                flag  : flag,
            },
            data: data,
        };
        if override_nibble {
            payload.inner.nibble = replacement;
        }
        packed_value    = payload;
        selected_nibble = payload.inner.nibble;
        selected_flag   = payload.inner.flag;
    }
}
";

    const STRUCT_INSTANCE_SOURCE: &str = r"
package StructTypes {
    struct Payload {
        upper: logic<4>,
        lower: logic<8>,
    }
}

module StructPass (
    value : input  StructTypes::Payload,
    result: output StructTypes::Payload,
) {
    always_comb {
        result = value;
    }
}

module StructInstanceTop (
    upper_in : input  logic<4>,
    lower_in : input  logic<8>,
    upper_out: output logic<4>,
    lower_out: output logic<8>,
) {
    var result: StructTypes::Payload;

    inst pass: StructPass (
        value: StructTypes::Payload'{
            upper: upper_in,
            lower: lower_in,
        },
        result: result,
    );

    always_comb {
        upper_out = result.upper;
        lower_out = result.lower;
    }
}
";

    const DISJOINT_STRUCT_FF_SOURCE: &str = r"
module DisjointStructFfTop (
    clk      : input  clock_posedge,
    rst_n    : input  reset_async_low,
    load     : input  logic,
    set_valid: input  logic,
    data     : input  logic<8>,
    payload  : output logic<8>,
    valid    : output logic,
) {
    struct Packet {
        payload: logic<8>,
        valid  : logic,
    }

    var packet_q: Packet;

    always_ff (clk) {
        if load {
            packet_q.payload = data;
        }
    }

    always_ff (clk, rst_n) {
        if_reset {
            packet_q.valid = 1'b0;
        } else if set_valid {
            packet_q.valid = 1'b1;
        }
    }

    always_comb {
        payload = packet_q.payload;
        valid = packet_q.valid;
    }
}
";

    const OVERLAPPING_STRUCT_FF_SOURCE: &str = r"
module OverlappingStructFfTop (
    clk : input clock_posedge,
    data: input logic<8>,
) {
    struct Packet {
        payload: logic<8>,
        valid  : logic,
    }

    var packet_q: Packet;

    always_ff (clk) {
        packet_q.payload = data;
    }

    always_ff (clk) {
        packet_q.payload = data + 8'h01;
    }
}
";

    const WIDE_LITERAL_SOURCE: &str = r"
module WideLiteralTop (
    zero108: output logic<108>,
    zero128: output logic<128>,
    value108: output logic<108>,
    value128: output logic<128>,
    value192: output logic<192>,
) {
    always_comb {
        zero108 = 108'd0;
        zero128 = 128'd0;
        value108 = 108'h800000000000000000000000001;
        value128 = 128'hfedcba98765432100123456789abcdef;
        value192 = 192'h0123456789abcdef_fedcba9876543210_8000000000000001;
    }
}
";

    #[test]
    fn lowers_wide_literals() {
        let design = analyze_and_lower(
            WIDE_LITERAL_SOURCE,
            "wide_literal_lowering",
            "WideLiteralTop",
        )
        .unwrap();
        let top = design.top_module().unwrap();

        for expected_width in [108, 128] {
            assert!(top.expressions().iter().any(|expression| {
                matches!(
                    expression.kind(),
                    ExprKind::Constant(value)
                        if value.width().get() == expected_width
                            && (0..expected_width).all(|bit| !value.bit(bit))
                )
            }));
        }

        let expected_values: &[(u32, &[u64])] = &[
            (108, &[1, 1 << 43]),
            (128, &[0x0123_4567_89ab_cdef, 0xfedc_ba98_7654_3210]),
            (
                192,
                &[
                    0x8000_0000_0000_0001,
                    0xfedc_ba98_7654_3210,
                    0x0123_4567_89ab_cdef,
                ],
            ),
        ];
        for &(expected_width, expected_words) in expected_values {
            assert!(top.expressions().iter().any(|expression| {
                matches!(
                    expression.kind(),
                    ExprKind::Constant(value)
                        if value.width().get() == expected_width
                            && (0..expected_width).all(|bit| {
                                value.bit(bit)
                                    == (((expected_words[bit as usize / 64] >> (bit % 64)) & 1)
                                        != 0)
                            })
                )
            }));
        }
        synthesize(&design).unwrap();
    }

    #[test]
    fn lowers_analyzed_comb_and_ff_through_ecp5_and_celox() {
        let design = analyze_and_lower(SOURCE, "air_lowering", "Top").unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        reset(&mut simulator);
        set(&mut simulator, "a", 100);
        set(&mut simulator, "b", 44);
        set(&mut simulator, "select", 1);
        tick(&mut simulator);
        assert_value(&mut simulator, "q", 144);
        assert_value(&mut simulator, "flag", 1);
        set(&mut simulator, "select", 0);
        tick(&mut simulator);
        assert_value(&mut simulator, "q", 56);
        assert_value(&mut simulator, "flag", 0);
    }

    #[test]
    fn retains_add_with_carry_as_one_arithmetic_cell() {
        let design = analyze_and_lower(
            ADD_WITH_CARRY_SOURCE,
            "add_with_carry_lowering",
            "AddWithCarry",
        )
        .unwrap();
        let synthesized = synthesize(&design).unwrap();

        assert_eq!(synthesized.netlist.arithmetic().len(), 1);
        assert!(synthesized.netlist.arithmetic()[0].carry_in().is_some());
    }

    #[test]
    fn shifts_at_the_assignment_context_width() {
        let design = analyze_and_lower(
            SHIFT_CONTEXT_SOURCE,
            "shift_context_lowering",
            "ShiftContext",
        )
        .unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        set(&mut simulator, "value", 1);
        set(&mut simulator, "signed_value", 0x80);
        set(&mut simulator, "amount", 8);
        assert_value(&mut simulator, "left", 0x0100);
        assert_value(&mut simulator, "logical_right", 0x00ff);
        assert_value(&mut simulator, "arithmetic_right", 0xffff);

        set(&mut simulator, "amount", 1);
        assert_value(&mut simulator, "left", 0x0002);
        assert_value(&mut simulator, "logical_right", 0x7fc0);
        assert_value(&mut simulator, "arithmetic_right", 0xffc0);
    }

    #[test]
    fn flattens_analyzer_expanded_interface_instances() {
        let design =
            analyze_and_lower(HIERARCHY_SOURCE, "interface_lowering", "HierarchyTop").unwrap();
        let top = design.top_module().unwrap();
        assert!(top.instances().is_empty());
        assert!(
            top.signals()
                .iter()
                .any(|signal| signal.name() == "increment.bus.request")
        );

        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        set(&mut simulator, "value", 0x7e);
        assert_value(&mut simulator, "result", 0x7f);
        set(&mut simulator, "value", 0xff);
        assert_value(&mut simulator, "result", 0x00);
    }

    #[test]
    fn lowers_case_patterns_to_synthesizable_priority_muxes() {
        let design = analyze_and_lower(CASE_SOURCE, "case_lowering", "CaseTop").unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        set(&mut simulator, "value", 0x20);
        for (select, expected) in [
            (0, 0x20),
            (1, 0xff),
            (2, 0x20),
            (3, 0x21),
            (5, 0x21),
            (6, 0xff),
        ] {
            set(&mut simulator, "select", select);
            assert_value(&mut simulator, "decoded", expected);
        }
    }

    #[test]
    fn case_duplicate_patterns_keep_the_first_matching_arm() {
        let design = analyze_and_lower(
            CASE_FIRST_MATCH_SOURCE,
            "case_first_match_lowering",
            "CaseFirstMatchTop",
        )
        .unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        set(&mut simulator, "base", 0x5a);
        for (select, decoded, side) in [(3, 0x11, 0), (7, 0x11, 0), (5, 0x33, 1), (2, 0xee, 1)] {
            set(&mut simulator, "select", select);
            assert_value(&mut simulator, "decoded", decoded);
            assert_value(&mut simulator, "side", side);
        }
    }

    #[test]
    fn sixteen_arm_case_has_logarithmic_match_and_data_depth() {
        let design = analyze_and_lower(
            BALANCED_CASE_SOURCE,
            "balanced_case_lowering",
            "BalancedCaseTop",
        )
        .unwrap();
        let top = design.top_module().unwrap();
        let decoded = top
            .signals()
            .iter()
            .find(|signal| signal.name() == "decoded")
            .unwrap()
            .id();
        let value = top
            .assignments()
            .iter()
            .find(|assignment| assignment.target.signal == decoded)
            .unwrap()
            .value;

        let ExprKind::Mux { condition, .. } = top.expressions()[value.index() as usize].kind()
        else {
            panic!("case result is not selected against its default")
        };

        // The match side is one equality plus four balanced OR levels. The
        // value side has four balanced first-match levels plus the final mux
        // selecting the default. A linear priority chain has sixteen data mux
        // levels (seventeen operations including its leaf equality) here.
        assert_eq!(expression_depth(top, *condition), 5);
        assert_eq!(case_data_mux_depth(top, value), 5);
    }

    #[test]
    fn dynamic_array_reads_have_logarithmic_depth_and_preserve_bounds() {
        for count in [3_u32, 5, 16, 17] {
            let source = format!(
                "module ArrayReadTop (values: input logic<8> [{count}], \
                 index: input logic<6>, result: output logic<8>) {{ \
                 assign result = values[index]; }}"
            );
            let design = analyze_and_lower(&source, "balanced_array", "ArrayReadTop").unwrap();
            let top = design.top_module().unwrap();
            let output = top
                .signals()
                .iter()
                .find(|s| s.name() == "result")
                .unwrap()
                .id();
            let value = top
                .assignments()
                .iter()
                .find(|a| a.target.signal == output)
                .unwrap()
                .value;
            let bound = (count.next_power_of_two().ilog2() + 1) as usize;
            assert!(
                case_data_mux_depth(top, value) <= bound,
                "{count} entries exceed logarithmic mux depth {bound}"
            );

            let synthesized = synthesize(&design).unwrap();
            let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
            let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
            for pattern in [0_u8, 0x31, 0x80, 0xff] {
                let bytes = (0..count)
                    .map(|i| pattern.wrapping_add(u8::try_from(i).unwrap().wrapping_mul(19)))
                    .collect::<Vec<_>>();
                for (i, &byte) in bytes.iter().enumerate() {
                    set(&mut simulator, &format!("values[{i}]"), byte);
                }
                for index in 0_u8..64 {
                    set(&mut simulator, "index", index);
                    assert_value(
                        &mut simulator,
                        "result",
                        u64::from(bytes.get(usize::from(index)).copied().unwrap_or(0)),
                    );
                }
            }
        }
    }

    #[test]
    fn sequential_reads_observe_previous_register_values() {
        let design = analyze_and_lower(NBA_SOURCE, "nba_lowering", "NbaTop").unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        // Read-after-write in one always_ff must observe the pre-edge
        // register value: `echoed` captures `stage1 + 1` from the *previous*
        // cycle, not the value assigned earlier in the same block.
        reset(&mut simulator);
        set(&mut simulator, "din", 100);
        tick(&mut simulator);
        assert_value(&mut simulator, "stage2", 1);
        tick(&mut simulator);
        assert_value(&mut simulator, "stage2", 101);

        // A default assignment followed by partial overrides composes over
        // the scheduled value, never over stale pre-edge bits.
        set(&mut simulator, "use_alt", 0);
        tick(&mut simulator);
        assert_value(&mut simulator, "flags", 0b01);
        set(&mut simulator, "use_alt", 1);
        set(&mut simulator, "alt", 0b10);
        tick(&mut simulator);
        // flags = {alt[0], alt[1]} = {0, 1}; a stale-bit composition would
        // keep the default 0b01 bit pattern instead.
        assert_value(&mut simulator, "flags", 0b01);
        set(&mut simulator, "alt", 0b11);
        tick(&mut simulator);
        assert_value(&mut simulator, "flags", 0b11);
    }

    #[test]
    fn lowers_exhaustive_partial_mux_assignments_without_a_false_loop() {
        let design =
            analyze_and_lower(PARTIAL_MUX_SOURCE, "partial_mux_lowering", "PartialMuxTop").unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        set(&mut simulator, "value", 0b1101_0010);
        tick(&mut simulator);
        set(&mut simulator, "reverse", 0);
        assert_value(&mut simulator, "result", 0b1101_0010);
        set(&mut simulator, "reverse", 1);
        assert_value(&mut simulator, "result", 0b0100_1011);
    }

    #[test]
    fn combinational_top_level_statements_observe_previous_blocking_writes() {
        let design = analyze_and_lower(
            BLOCKING_COMB_SOURCE,
            "blocking_comb_lowering",
            "BlockingCombTop",
        )
        .unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        set(&mut simulator, "enable", 0);
        set(&mut simulator, "seed", 0x25);
        assert_value(&mut simulator, "result", 0x25);
        set(&mut simulator, "enable", 1);
        assert_value(&mut simulator, "result", 0xa5);
        set(&mut simulator, "seed", 0xc3);
        assert_value(&mut simulator, "result", 0xc3);
    }

    #[test]
    fn lowers_nested_struct_constructors_and_member_accesses() {
        let design = analyze_and_lower(STRUCT_SOURCE, "struct_lowering", "StructTop").unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        set(&mut simulator, "header", 0b101);
        set(&mut simulator, "nibble", 0b1010);
        set(&mut simulator, "flag", 1);
        set(&mut simulator, "data", 0xc3);
        set(&mut simulator, "override_nibble", 0);
        assert_value(&mut simulator, "packed_value", 0xb5c3);
        assert_value(&mut simulator, "selected_nibble", 0b1010);
        assert_value(&mut simulator, "selected_flag", 1);

        set(&mut simulator, "replacement", 0b0011);
        set(&mut simulator, "override_nibble", 1);
        assert_value(&mut simulator, "packed_value", 0xa7c3);
        assert_value(&mut simulator, "selected_nibble", 0b0011);
        assert_value(&mut simulator, "selected_flag", 1);
    }

    #[test]
    fn lowers_struct_ports_across_flattened_instances() {
        let design = analyze_and_lower(
            STRUCT_INSTANCE_SOURCE,
            "struct_instance_lowering",
            "StructInstanceTop",
        )
        .unwrap();
        assert!(design.top_module().unwrap().instances().is_empty());

        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        set(&mut simulator, "upper_in", 0xa);
        set(&mut simulator, "lower_in", 0x5c);
        assert_value(&mut simulator, "upper_out", 0xa);
        assert_value(&mut simulator, "lower_out", 0x5c);
    }

    #[test]
    fn lowers_disjoint_struct_fields_from_separate_ff_blocks() {
        let design = analyze_and_lower(
            DISJOINT_STRUCT_FF_SOURCE,
            "disjoint_struct_ff_lowering",
            "DisjointStructFfTop",
        )
        .unwrap();
        let top = design.top_module().unwrap();
        let mut register_widths = top
            .registers()
            .iter()
            .map(|register| {
                top.signals()[register.target.index() as usize]
                    .r#type()
                    .width
                    .get()
            })
            .collect::<Vec<_>>();
        register_widths.sort_unstable();
        assert_eq!(register_widths, [1, 8]);

        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        reset(&mut simulator);
        set(&mut simulator, "load", 1);
        set(&mut simulator, "data", 0xa5);
        tick(&mut simulator);
        set(&mut simulator, "load", 0);
        set(&mut simulator, "set_valid", 1);
        tick(&mut simulator);
        assert_value(&mut simulator, "payload", 0xa5);
        assert_value(&mut simulator, "valid", 1);

        set(&mut simulator, "rst_n", 0);
        tick(&mut simulator);
        set(&mut simulator, "rst_n", 1);
        assert_value(&mut simulator, "payload", 0xa5);
        assert_value(&mut simulator, "valid", 0);
    }

    #[test]
    fn rejects_overlapping_struct_fields_from_separate_ff_blocks() {
        let error = analyze_and_lower(
            OVERLAPPING_STRUCT_FF_SOURCE,
            "overlapping_struct_ff_lowering",
            "OverlappingStructFfTop",
        )
        .unwrap_err();
        assert!(
            matches!(
                &error,
                ImportError::AnalysisFailed(message)
                    if message.contains("MultipleAssignment") && message.contains("packet_q")
            ),
            "{error:?}"
        );
    }

    #[test]
    fn flattens_parameter_bounded_generate_for_instances() {
        let design = analyze_and_lower(
            GENERATE_FOR_SOURCE,
            "generate_for_lowering",
            "GenerateForTop",
        )
        .unwrap();
        let top = design.top_module().unwrap();
        assert!(top.instances().is_empty());
        for lane in 0_u8..4 {
            assert!(
                top.signals().iter().any(|signal| {
                    signal.name() == format!("bank.lane[{lane}].increment.value")
                })
            );
        }

        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        let values = simulator.signal("values");
        simulator
            .modify(|io| io.set(values, 0xff7f_0100u32))
            .unwrap();
        assert_value(&mut simulator, "results", 0x0080_0201);
    }

    #[test]
    fn flattens_statically_indexed_unpacked_and_interface_arrays() {
        let design = analyze_and_lower(
            UNPACKED_ARRAY_SOURCE,
            "unpacked_array_lowering",
            "UnpackedArrayTop",
        )
        .unwrap();
        let top = design.top_module().unwrap();
        for lane in 0_u8..4 {
            for name in [
                format!("enable[{lane}]"),
                format!("lanes.request[{lane}]"),
                format!("lanes.response[{lane}]"),
                format!("state[{lane}]"),
            ] {
                assert!(
                    top.signals().iter().any(|signal| signal.name() == name),
                    "missing flattened signal {name}"
                );
            }
        }

        let synthesized = synthesize(&design).unwrap();
        assert_eq!(synthesized.netlist.registers().len(), 32);
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        reset(&mut simulator);
        for lane in 0_u8..4 {
            set(&mut simulator, &format!("enable[{lane}]"), 1);
            set(
                &mut simulator,
                &format!("lanes.request[{lane}]"),
                0x10 + lane,
            );
        }
        tick(&mut simulator);
        for lane in 0_u8..4 {
            assert_value(
                &mut simulator,
                &format!("lanes.response[{lane}]"),
                0x11 + u64::from(lane),
            );
        }
    }

    #[test]
    fn flattens_interface_arrays_across_module_instances() {
        let design = analyze_and_lower(
            UNPACKED_ARRAY_SOURCE,
            "interface_array_instance_lowering",
            "UnpackedInterfaceArrayWrapper",
        )
        .unwrap();
        let top = design.top_module().unwrap();
        assert!(top.instances().is_empty());
        for lane in 0_u8..4 {
            assert!(
                top.signals()
                    .iter()
                    .any(|signal| { signal.name() == format!("dut.lanes.request[{lane}]") })
            );
        }

        let synthesized = synthesize(&design).unwrap();
        assert_eq!(synthesized.netlist.registers().len(), 32);
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        reset(&mut simulator);
        let requests = simulator.signal("requests");
        simulator
            .modify(|io| io.set(requests, 0x4030_2010u32))
            .unwrap();
        tick(&mut simulator);
        assert_value(&mut simulator, "responses", 0x4131_2111);
    }

    #[test]
    fn flattens_unpacked_array_ports_across_module_instances() {
        let design = analyze_and_lower(
            UNPACKED_ARRAY_INSTANCE_SOURCE,
            "unpacked_array_instance_lowering",
            "UnpackedArrayInstanceTop",
        )
        .unwrap();
        let top = design.top_module().unwrap();
        assert!(top.instances().is_empty());
        for lane in 0_u8..4 {
            assert!(
                top.signals()
                    .iter()
                    .any(|signal| { signal.name() == format!("increment.values[{lane}]") })
            );
        }

        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        for lane in 0_u8..4 {
            set(&mut simulator, &format!("values[{lane}]"), 0x20 + lane);
            assert_value(
                &mut simulator,
                &format!("results[{lane}]"),
                0x21 + u64::from(lane),
            );
        }
    }

    #[test]
    fn flattens_unpacked_array_slices_in_instance_inputs() {
        let source = UNPACKED_ARRAY_INSTANCE_SOURCE
            .replace(
                "values : input  logic<8> [4]",
                "values : input  logic<8> [6]",
            )
            .replace("values : values ,", "values : values[1+:4],");
        let design = analyze_and_lower(
            &source,
            "unpacked_array_slice_instance_lowering",
            "UnpackedArrayInstanceTop",
        )
        .unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        for lane in 0_u8..6 {
            set(&mut simulator, &format!("values[{lane}]"), 0x20 + lane);
        }
        for lane in 0_u8..4 {
            assert_value(
                &mut simulator,
                &format!("results[{lane}]"),
                0x22 + u64::from(lane),
            );
        }
    }

    #[test]
    fn lowers_dynamic_register_array_indices() {
        let source = REQUIRED_ASYNC_MEMORY_SOURCE
            .replace("    #[sv(\"struo_memory = \\\"required\\\"\")]\n", "");
        let design = analyze_and_lower(
            &source,
            "dynamic_register_array_index_lowering",
            "RequiredAsyncMemoryTop",
        )
        .unwrap();
        let synthesized = synthesize(&design).unwrap();
        assert!(synthesized.netlist.memories().is_empty());
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        set(&mut simulator, "write_enable", 1);
        for address in 0_u8..16 {
            set(&mut simulator, "write_address", address);
            set(&mut simulator, "write_data", 0x40 + address);
            tick(&mut simulator);
        }
        set(&mut simulator, "write_enable", 0);
        for address in 0_u8..16 {
            set(&mut simulator, "read_address", address);
            assert_value(&mut simulator, "read_data", u64::from(0x40 + address));
        }
    }

    #[test]
    fn infers_veryl_array_as_mapped_block_ram() {
        let design = analyze_and_lower(MEMORY_SOURCE, "memory_lowering", "MemoryTop").unwrap();
        let top = design.top_module().unwrap();
        assert_eq!(top.memories().len(), 1);
        assert_eq!(top.memories()[0].name, "words");
        assert_eq!(top.memories()[0].depth, 16);

        let synthesized = synthesize(&design).unwrap();
        assert_eq!(synthesized.netlist.memories().len(), 1);
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let json = mapped.to_nextpnr_json().unwrap();
        assert!(json.contains("\"type\": \"DP16KD\""));

        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        set(&mut simulator, "write_enable", 1);
        set(&mut simulator, "write_address", 3);
        set(&mut simulator, "write_data", 0x5a);
        set(&mut simulator, "read_address", 0);
        tick(&mut simulator);
        set(&mut simulator, "write_enable", 0);
        set(&mut simulator, "read_address", 3);
        tick(&mut simulator);
        assert_value(&mut simulator, "read_data", 0x5a);
    }

    #[test]
    fn infers_independently_clocked_true_dual_port_block_ram() {
        let design = analyze_and_lower(
            TRUE_DUAL_PORT_MEMORY_SOURCE,
            "true_dual_port_memory_lowering",
            "TrueDualPortMemoryTop",
        )
        .unwrap();
        let memory = &design.top_module().unwrap().memories()[0];
        assert!(memory.second_port.is_some());
        assert!(memory.read_enable.is_some());
        assert!(memory.second_port.as_ref().unwrap().read_enable.is_some());

        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let json = mapped.to_nextpnr_json().unwrap();
        assert!(json.contains("\"WEAMUX\": \"WEA\""));
        assert!(json.contains("\"WEBMUX\": \"WEB\""));
        assert!(json.contains("\"CLKAMUX\""));
        assert!(json.contains("\"CLKBMUX\""));
        assert!(json.contains("\"INV\""));
        assert!(json.contains("\"DOA0\""));
        assert!(json.contains("\"DOB0\""));
        assert!(json.contains("\"CEAMUX\": \"CEA\""));
        assert!(json.contains("\"CEBMUX\": \"CEB\""));
    }

    #[test]
    fn true_dual_port_memory_requires_multiple_assign_opt_in() {
        let source = TRUE_DUAL_PORT_MEMORY_SOURCE.replace("#[allow(multiple_assign)]", "");
        let error = analyze_and_lower(
            &source,
            "true_dual_port_memory_without_opt_in",
            "TrueDualPortMemoryTop",
        )
        .unwrap_err();

        assert!(
            matches!(&error, ImportError::AnalysisFailed(message)
                if message.contains("MultipleAssignment")),
            "{error}"
        );
    }

    #[test]
    fn multiple_assign_opt_in_does_not_allow_multiple_drivers_in_logic() {
        let source = TRUE_DUAL_PORT_MEMORY_SOURCE.replace("required", "forbidden");
        let error = analyze_and_lower(
            &source,
            "multiple_assign_without_memory_inference",
            "TrueDualPortMemoryTop",
        )
        .unwrap_err();

        assert!(
            matches!(&error, ImportError::UnsupportedBehavior(message)
                if message.contains("multiple procedural drivers for words")),
            "{error}"
        );
    }

    #[test]
    fn required_memory_policy_accepts_a_supported_memory() {
        let source = MEMORY_SOURCE.replace(
            "    var words: logic<8> [16];",
            "    #[sv(\"struo_memory = \\\"required\\\"\")]\n    var words: logic<8> [16];",
        );
        let design = analyze_and_lower(&source, "required_memory", "MemoryTop").unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();

        assert_eq!(design.top_module().unwrap().memories().len(), 1);
        assert!(
            mapped
                .to_nextpnr_json()
                .unwrap()
                .contains("\"type\": \"DP16KD\"")
        );
    }

    #[test]
    fn block_memory_policy_selects_block_ram() {
        let source = MEMORY_SOURCE.replace(
            "    var words: logic<8> [16];",
            "    #[sv(\"struo_memory = \\\"block\\\"\")]\n    var words: logic<8> [16];",
        );
        let design = analyze_and_lower(&source, "block_memory", "MemoryTop").unwrap();
        let memory = &design.top_module().unwrap().memories()[0];
        assert_eq!(memory.style, struo_rtl::MemoryStyle::Block);
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        assert!(
            mapped
                .to_nextpnr_json()
                .unwrap()
                .contains("\"type\": \"DP16KD\"")
        );
    }

    #[test]
    fn required_memory_policy_reports_why_inference_failed() {
        let error = analyze_and_lower(
            REQUIRED_ASYNC_MEMORY_SOURCE,
            "required_async_memory",
            "RequiredAsyncMemoryTop",
        )
        .unwrap_err();

        assert!(matches!(
            &error,
            ImportError::RequiredMemoryInferenceFailed { memory, reason }
                if memory == "words"
                    && reason.contains("no supported synchronous read port")
        ));
        assert_eq!(
            error.to_string(),
            "memory inference was required for `words`, but failed: no supported synchronous read port was found"
        );
    }

    #[test]
    fn maps_one_bit_by_128_distributed_ram() {
        let design = analyze_and_lower(
            DISTRIBUTED_MEMORY_SOURCE,
            "distributed_memory",
            "DistributedMemoryTop",
        )
        .unwrap();
        let memory = &design.top_module().unwrap().memories()[0];
        assert_eq!(memory.style, struo_rtl::MemoryStyle::Distributed);
        assert_eq!(memory.read_latency, 0);

        let synthesized = synthesize(&design).unwrap();
        let memory = &synthesized.netlist.memories()[0];
        assert_eq!(memory.read_latency(), 0);
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(
                    cell,
                    Ecp5Cell::BlockRam {
                        implementation: Ecp5MemoryImplementation::Distributed,
                        ..
                    }
                ))
                .count(),
            8
        );
        let json = mapped.to_nextpnr_json().unwrap();
        assert_eq!(json.matches("\"type\": \"TRELLIS_DPR16X4\"").count(), 8);
        assert!(!json.contains("\"type\": \"DP16KD\""));

        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        set(&mut simulator, "write_enable", 1);
        for address in [0_u8, 15, 16, 31, 64, 127] {
            set(&mut simulator, "write_address", address);
            set(&mut simulator, "write_data", 1);
            tick(&mut simulator);
        }
        set(&mut simulator, "write_enable", 0);
        for address in [0_u8, 15, 16, 31, 64, 127] {
            set(&mut simulator, "read_address", address);
            assert_value(&mut simulator, "read_data", 1);
        }
        for address in [1_u8, 14, 17, 63, 65, 126] {
            set(&mut simulator, "read_address", address);
            assert_value(&mut simulator, "read_data", 0);
        }
    }

    #[test]
    fn forbidden_memory_policy_disables_inference() {
        let design = analyze_and_lower(
            FORBIDDEN_MEMORY_SOURCE,
            "forbidden_memory",
            "ForbiddenMemoryTop",
        )
        .unwrap();
        assert!(design.top_module().unwrap().memories().is_empty());
        let synthesized = synthesize(&design).unwrap();
        assert!(synthesized.netlist.memories().is_empty());
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        set(&mut simulator, "write_enable", 1);
        for address in 0_u8..16 {
            set(&mut simulator, "write_address", address);
            set(&mut simulator, "write_data", 0x60 + address);
            tick(&mut simulator);
        }
        set(&mut simulator, "write_enable", 0);
        for address in 0_u8..16 {
            set(&mut simulator, "read_address", address);
            tick(&mut simulator);
            assert_value(&mut simulator, "read_data", u64::from(0x60 + address));
        }
    }

    #[test]
    fn lowers_module_constants_and_dynamic_packed_bit_selects() {
        let design = analyze_and_lower(
            I2C_EXPRESSION_SOURCE,
            "i2c_expression_lowering",
            "I2cExpressionTop",
        )
        .unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();

        set(&mut simulator, "read_data", 0b1010_0100);
        for bit_index in 0..8 {
            set(&mut simulator, "bit_index", bit_index);
            let selected = (0b1010_0100 >> bit_index) & 1;
            assert_value(&mut simulator, "state", if selected == 0 { 0 } else { 7 });
            assert_value(&mut simulator, "drive_low", 1 - selected);
        }
    }

    #[test]
    fn maps_a_veryl_top_interface_to_jtagg() {
        let source = r"
module DebugTop (
    jtag_tdi   : input logic,
    jtag_tck   : input clock,
    jtag_rti1  : input logic,
    jtag_rti2  : input logic,
    jtag_shift : input logic,
    jtag_update: input logic,
    jtag_rst_n : input reset_async_low,
    jtag_ce1   : input logic,
    jtag_ce2   : input logic,
    jtag_tdo1  : output logic,
    jtag_tdo2  : output logic,
) {
    always_comb {
        jtag_tdo1 = 0;
        jtag_tdo2 = 0;
    }
}
";
        let design = analyze_and_lower(source, "jtagg_lowering", "DebugTop").unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped =
            map_to_ecp5_with_jtagg(&synthesized.netlist, &JtaggBinding::with_prefix("jtag"))
                .unwrap();

        assert!(mapped.ports().is_empty());
        assert!(
            mapped
                .to_nextpnr_json()
                .unwrap()
                .contains("\"type\": \"JTAGG\"")
        );
    }

    fn case_data_mux_depth(module: &RtlModule, id: ExprId) -> usize {
        match module.expressions()[id.index() as usize].kind() {
            ExprKind::Signal(_) | ExprKind::Constant(_) => 0,
            ExprKind::Unary { input, .. } | ExprKind::Slice { input, .. } => {
                case_data_mux_depth(module, *input)
            }
            ExprKind::Binary { lhs, rhs, .. } => {
                case_data_mux_depth(module, *lhs).max(case_data_mux_depth(module, *rhs))
            }
            ExprKind::Mux {
                then_expr,
                else_expr,
                ..
            } => {
                1 + case_data_mux_depth(module, *then_expr)
                    .max(case_data_mux_depth(module, *else_expr))
            }
            ExprKind::Concat(parts) => parts
                .iter()
                .map(|part| case_data_mux_depth(module, *part))
                .max()
                .unwrap_or_default(),
        }
    }

    fn expression_depth(module: &RtlModule, id: ExprId) -> usize {
        match module.expressions()[id.index() as usize].kind() {
            ExprKind::Signal(_) | ExprKind::Constant(_) => 0,
            ExprKind::Unary { input, .. } | ExprKind::Slice { input, .. } => {
                1 + expression_depth(module, *input)
            }
            ExprKind::Binary { lhs, rhs, .. } => {
                1 + expression_depth(module, *lhs).max(expression_depth(module, *rhs))
            }
            ExprKind::Mux {
                condition,
                then_expr,
                else_expr,
            } => {
                1 + expression_depth(module, *condition)
                    .max(expression_depth(module, *then_expr))
                    .max(expression_depth(module, *else_expr))
            }
            ExprKind::Concat(parts) => {
                1 + parts
                    .iter()
                    .map(|part| expression_depth(module, *part))
                    .max()
                    .unwrap_or_default()
            }
        }
    }

    fn reset(simulator: &mut Simulator<NativeBackend>) {
        set(simulator, "rst_n", 0);
        tick(simulator);
        set(simulator, "rst_n", 1);
    }

    fn tick(simulator: &mut Simulator<NativeBackend>) {
        simulator.tick(simulator.event("clk")).unwrap();
    }

    fn set(simulator: &mut Simulator<NativeBackend>, name: &str, value: u8) {
        let signal = simulator.signal(name);
        simulator.modify(|io| io.set(signal, value)).unwrap();
    }

    fn assert_value(simulator: &mut Simulator<NativeBackend>, name: &str, expected: u64) {
        assert_eq!(
            simulator.get(simulator.signal(name)),
            expected.into(),
            "{name}"
        );
    }
}
