use std::cmp::Reverse;
use std::collections::{BTreeSet, BinaryHeap, HashMap, HashSet};

use struo_rtl::{
    BinaryOp, BitWidth, ClockEdge, Constant, Design, Enable, ExprId, ExprKind, Memory, MemoryPort,
    MemoryStyle, Module as RtlModule, Polarity, Port, PortDirection, Register, Reset, ResetMode,
    SignalId, SignalSlice, StateDomain, UnaryOp, ValueType,
};
use veryl_analyzer::ir::{
    AssignDestination, CasePattern, CaseStatement, Component, Comptime, Declaration, Expression,
    Factor, FfDeclaration, IfResetStatement, InstDeclaration, Ir, Module, Op, Statement, Type,
    TypeKind, ValueVariant, VarId, VarIndex, VarKind, VarSelect, VarSelectOp,
};
use veryl_analyzer::{attribute::Attribute as VerylAttribute, attribute_table};
use veryl_parser::resource_table::StrId;

use crate::{ImportError, MemoryInferencePolicy, resolve_name};

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
struct SignalKey {
    id: VarId,
    index: Vec<usize>,
}

type Env = HashMap<SignalKey, LoweredExpr>;

#[derive(Clone, Copy)]
struct LoweredExpr {
    id: ExprId,
    width: u32,
    signed: bool,
}

#[derive(Clone, Copy)]
enum AssociativePlan {
    Leaf(ExprId),
    Combine { lhs: usize, rhs: usize },
}

#[derive(Clone, Copy)]
enum LoweredArrayIndex {
    Static(usize),
    Dynamic(LoweredExpr),
}

struct ModuleLowerer<'a> {
    source: &'a Module,
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
                variable.kind == VarKind::Variable && !inferred_memories.contains(&variable.id)
            })
            .collect::<Vec<_>>();
        internals.sort_by_key(|variable| variable.path.to_string());
        for variable in internals {
            let base_name = variable.path.to_string();
            let r#type = value_type(&variable.r#type, &base_name)?;
            for index in array_indices(&variable.r#type, &base_name)? {
                let key = SignalKey {
                    id: variable.id,
                    index,
                };
                let signal = rtl.add_signal(indexed_name(&base_name, &key.index), r#type);
                signals.insert(key.clone(), signal);
                signal_order.push(key.clone());
                widths.insert(key.clone(), r#type.width.get());
                signed.insert(key, r#type.signed);
            }
        }

        Ok(Self {
            source,
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

    fn lower_declarations(&mut self) -> Result<(), ImportError> {
        self.infer_memories()?;
        let mut driven_comb = BTreeSet::new();
        let mut driven_ff = BTreeSet::new();
        for declaration in &self.source.declarations {
            match declaration {
                Declaration::Comb(comb) => {
                    let initial = self.read_env()?;
                    let mut env = initial.clone();
                    let mut changed = BTreeSet::new();
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
                    for key in changed {
                        if !driven_comb.insert(key.clone()) || driven_ff.contains(&key) {
                            return Err(ImportError::UnsupportedBehavior(format!(
                                "multiple procedural drivers for {}",
                                self.signal_name(&key)
                            )));
                        }
                        let signal = self.signal(&key)?;
                        let value = env[&key];
                        self.rtl.assign(self.rtl.whole(signal)?, value.id)?;
                    }
                }
                Declaration::Ff(ff) => {
                    let changed = self.lower_ff(ff)?;
                    for key in changed {
                        if !driven_ff.insert(key.clone()) || driven_comb.contains(&key) {
                            return Err(ImportError::UnsupportedBehavior(format!(
                                "multiple procedural drivers for {}",
                                self.signal_name(&key)
                            )));
                        }
                    }
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
            if let Some(parent_id) = whole_array_variable(&input.expr) {
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
            let element = input_elements.entry(input.id).or_insert(0);
            let child_key = child.port_element_key(input.id, *element)?;
            *element += 1;
            let child_signal = child.signal(&child_key)?;
            let target = inline_signals[&child_signal];
            let width = child.width(&child_key)?;
            let value = self.lower_expression(&input.expr, parent_env)?;
            let value = self.resize(value, width, child.is_signed(&child_key))?;
            self.rtl.assign(self.rtl.whole(target)?, value.id)?;
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
            let element = output_elements.entry(output.id).or_insert(0);
            let child_key = child.port_element_key(output.id, *element)?;
            *element += 1;
            let child_signal = child.signal(&child_key)?;
            let source = inline_signals[&child_signal];
            let source_width = child.width(&child_key)?;
            let source_expr = self.rtl.read(source)?;
            let destinations = output
                .dst
                .iter()
                .map(|destination| {
                    self.destination_slice(destination)
                        .map(|slice| (destination, slice))
                })
                .collect::<Result<Vec<_>, _>>()?;
            let destination_width = destinations
                .iter()
                .map(|(_, slice)| slice.width.get())
                .sum::<u32>();
            if destination_width != source_width {
                return Err(ImportError::UnsupportedBehavior(format!(
                    "instance output {} connects {source_width} bits to {destination_width} bits",
                    child.variable_name(output.id)
                )));
            }
            let mut remaining = source_width;
            for (_, destination) in destinations {
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

    fn lower_ff(&mut self, ff: &FfDeclaration) -> Result<BTreeSet<SignalKey>, ImportError> {
        let (clock_key, edge) = self.source_clock(ff)?;
        let clock = self.signal(&clock_key)?;
        let initial = self.read_env()?;
        let mut next = initial.clone();
        let mut reset_values = None;
        let mut changed = BTreeSet::new();

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

        for key in &changed {
            let signal = self.signal(key)?;
            let reset = if let (Some(values), Some((reset_signal, mode, polarity))) =
                (&reset_values, reset_control)
            {
                let value = values.get(key).copied().unwrap_or(initial[key]);
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
                next: next.get(key).copied().unwrap_or(initial[key]).id,
                clock,
                edge,
                enable: None,
                reset,
            })?;
        }
        Ok(changed)
    }

    fn lower_if_reset(
        &mut self,
        branch: &IfResetStatement,
        initial: &Env,
    ) -> Result<(Env, Env, BTreeSet<SignalKey>), ImportError> {
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
    ) -> Result<BTreeSet<SignalKey>, ImportError> {
        let mut changed = BTreeSet::new();
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

    fn lower_statement(
        &mut self,
        statement: &Statement,
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<BTreeSet<SignalKey>, ImportError> {
        match statement {
            Statement::Assign(assign) => {
                if assign.dst.len() != 1 {
                    return Err(ImportError::UnsupportedBehavior(
                        "concatenated assignment destinations".into(),
                    ));
                }
                let destination = &assign.dst[0];
                // Reads observe the pre-edge register value, but a partial
                // write composes over the value already scheduled for this
                // edge (later writes win per bit).
                let value = self.lower_expression(&assign.expr, reads)?;
                self.assign_destination(destination, value, reads, writes)
            }
            Statement::If(branch) => {
                let condition = self.lower_expression(&branch.cond, reads)?;
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
                for key in &changed {
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
                Ok(changed)
            }
            Statement::IfReset(_) if sequential => Err(ImportError::UnsupportedBehavior(
                "nested if_reset statements".into(),
            )),
            Statement::Null => Ok(BTreeSet::new()),
            Statement::Case(case_statement) => {
                self.lower_case(case_statement, reads, writes, sequential)
            }
            Statement::For(_) => Err(ImportError::UnsupportedBehavior(
                "runtime and unrolled for statements are not lowered yet".into(),
            )),
            Statement::FunctionCall(_) | Statement::SystemFunctionCall(_) => Err(
                ImportError::UnsupportedBehavior("statement-level function calls".into()),
            ),
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

    fn lower_case(
        &mut self,
        statement: &CaseStatement,
        reads: &Env,
        writes: &mut Env,
        sequential: bool,
    ) -> Result<BTreeSet<SignalKey>, ImportError> {
        let target = self.lower_expression(&statement.case_target, reads)?;
        let base = writes.clone();
        let mut else_env = base.clone();
        let mut changed =
            self.lower_statements(&statement.default, reads, &mut else_env, sequential)?;

        for arm in statement.arms.iter().rev() {
            let mut then_env = base.clone();
            let arm_changed = self.lower_statements(&arm.body, reads, &mut then_env, sequential)?;
            let condition = self.lower_case_patterns(target, &arm.patterns, &base)?;
            let mut merged_changed = changed.clone();
            merged_changed.extend(arm_changed);
            let mut merged_env = base.clone();
            for key in &merged_changed {
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
        Ok(changed)
    }

    fn lower_case_patterns(
        &mut self,
        target: LoweredExpr,
        patterns: &[CasePattern],
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let mut condition = None;
        for pattern in patterns {
            let matches = match pattern {
                CasePattern::Eq(value) => {
                    let value = self.lower_expression(value, env)?;
                    self.lower_binary(Op::Eq, target, value, 1, false)?
                }
                CasePattern::Range { lo, hi, inclusive } => {
                    let lo = self.lower_expression(lo, env)?;
                    let hi = self.lower_expression(hi, env)?;
                    let lower = self.lower_binary(Op::LessEq, lo, target, 1, false)?;
                    let upper_op = if *inclusive { Op::LessEq } else { Op::Less };
                    let upper = self.lower_binary(upper_op, target, hi, 1, false)?;
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

    fn lower_expression(
        &mut self,
        expression: &Expression,
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        match expression {
            Expression::Term(factor) => self.lower_factor(factor, env),
            Expression::Unary(op, input, comptime) => {
                let input = self.lower_expression(input, env)?;
                let width = concrete_width(&comptime.r#type, "unary expression")?;
                let input = self.resize(input, input.width.max(width), input.signed)?;
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
                    _ => return Err(Self::unsupported_expression(*op)),
                };
                self.resize(
                    LoweredExpr {
                        id,
                        width: result_width,
                        signed,
                    },
                    width,
                    comptime.r#type.signed,
                )
            }
            Expression::Binary(..) => self.lower_binary_expression(expression, env),
            Expression::Ternary(condition, then_expr, else_expr, comptime) => {
                let condition = self.lower_expression(condition, env)?;
                let condition = self.boolean(condition)?;
                let width = concrete_width(&comptime.r#type, "ternary expression")?;
                let signed = comptime.r#type.signed;
                let then_expr = self.lower_expression(then_expr, env)?;
                let then_expr = self.resize(then_expr, width, signed)?;
                let else_expr = self.lower_expression(else_expr, env)?;
                let else_expr = self.resize(else_expr, width, signed)?;
                Ok(LoweredExpr {
                    id: self.rtl.mux(condition.id, then_expr.id, else_expr.id)?,
                    width,
                    signed,
                })
            }
            Expression::Concatenation(parts, comptime) => {
                let mut lowered = Vec::new();
                for (part, repeat) in parts {
                    let part = self.lower_expression(part, env)?;
                    let count = if let Some(repeat) = repeat {
                        constant_value(repeat)?
                    } else {
                        1
                    };
                    for _ in 0..count {
                        lowered.push(part.id);
                    }
                }
                let width = concrete_width(&comptime.r#type, "concatenation")?;
                Ok(LoweredExpr {
                    id: self.rtl.concat(lowered)?,
                    width,
                    signed: false,
                })
            }
            Expression::StructConstructor(r#type, fields, _) => {
                self.lower_struct_constructor(r#type, fields, env)
            }
            Expression::ArrayLiteral(_, _) => Err(ImportError::UnsupportedBehavior(
                "array literal expression".into(),
            )),
        }
    }

    fn lower_binary_expression(
        &mut self,
        expression: &Expression,
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let Expression::Binary(lhs, op, rhs, comptime) = expression else {
            unreachable!("called with a non-binary expression")
        };
        if *op == Op::As {
            let lhs = self.lower_expression(lhs, env)?;
            let width = concrete_width(&comptime.r#type, "cast expression")?;
            return self.resize(lhs, width, comptime.r#type.signed);
        }
        if matches!(op, Op::LogicAnd | Op::LogicOr) {
            return self.lower_associative_logic(expression, *op, env);
        }
        let lhs = self.lower_expression(lhs, env)?;
        let rhs = self.lower_expression(rhs, env)?;
        let result_width = binary_width(*op, comptime)?;
        self.lower_binary(*op, lhs, rhs, result_width, comptime.r#type.signed)
    }

    fn lower_associative_logic(
        &mut self,
        expression: &Expression,
        op: Op,
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let mut operands = Vec::new();
        collect_associative_operands(expression, op, &mut operands);

        let mut depth_memo = HashMap::new();
        let mut leaves = Vec::with_capacity(operands.len());
        let mut leaf_depths = Vec::with_capacity(operands.len());
        for operand in operands {
            let lowered = self.lower_expression(operand, env)?;
            let id = self.boolean(lowered)?.id;
            leaf_depths.push(rtl_expression_depth(&self.rtl, id, &mut depth_memo));
            leaves.push(id);
        }

        let binary_op = if op == Op::LogicAnd {
            BinaryOp::And
        } else {
            BinaryOp::Or
        };
        let mut plans = leaves
            .iter()
            .copied()
            .map(AssociativePlan::Leaf)
            .collect::<Vec<_>>();
        let mut source_leaf = 0;
        let (source_plan, source_depth) = build_associative_source_plan(
            expression,
            op,
            &leaf_depths,
            &mut source_leaf,
            &mut plans,
        );

        // Pairing by source position still buries an already deep predicate under
        // every tree level. Merge the shallowest partial trees first so that
        // equal-depth leaves remain balanced while deep leaves stay near the root.
        let mut pending = leaf_depths
            .iter()
            .copied()
            .enumerate()
            .map(|(order, depth)| Reverse((depth, order, order)))
            .collect::<BinaryHeap<_>>();
        let mut order = pending.len();
        while pending.len() > 1 {
            let Reverse((lhs_depth, _, lhs_plan)) = pending
                .pop()
                .expect("a logical chain has at least two operands");
            let Reverse((rhs_depth, _, rhs_plan)) = pending
                .pop()
                .expect("a logical chain has a second pending operand");
            let plan = plans.len();
            plans.push(AssociativePlan::Combine {
                lhs: lhs_plan,
                rhs: rhs_plan,
            });
            pending.push(Reverse((lhs_depth.max(rhs_depth) + 1, order, plan)));
            order += 1;
        }
        let Reverse((optimized_depth, _, optimized_plan)) =
            pending.pop().expect("a logical chain produces one result");

        // Keep an already depth-optimal or marginally deeper source tree intact.
        // Reassociating for a single level perturbs mapper sharing and fanout
        // without enough structural improvement to justify the churn.
        let selected_plan = if source_depth.saturating_sub(optimized_depth) >= 2 {
            optimized_plan
        } else {
            source_plan
        };
        let id = self.materialize_associative_plan(&plans, selected_plan, binary_op)?;

        Ok(LoweredExpr {
            id,
            width: 1,
            signed: false,
        })
    }

    fn materialize_associative_plan(
        &mut self,
        plans: &[AssociativePlan],
        plan: usize,
        op: BinaryOp,
    ) -> Result<ExprId, ImportError> {
        match plans[plan] {
            AssociativePlan::Leaf(id) => Ok(id),
            AssociativePlan::Combine { lhs, rhs } => {
                let lhs = self.materialize_associative_plan(plans, lhs, op)?;
                let rhs = self.materialize_associative_plan(plans, rhs, op)?;
                Ok(self.rtl.binary(op, lhs, rhs)?)
            }
        }
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
                Op::LogicShiftR => BinaryOp::ShiftRightLogical,
                Op::ArithShiftR => BinaryOp::ShiftRightArithmetic,
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
            Op::Eq | Op::Ne | Op::Less | Op::LessEq | Op::Greater | Op::GreaterEq
        );
        let operand_width = lhs.width.max(rhs.width);
        let signed_compare = lhs.signed && rhs.signed;
        let lhs = self.resize(lhs, operand_width, signed_compare)?;
        let rhs = self.resize(rhs, operand_width, signed_compare)?;
        let operation = match op {
            Op::Add => BinaryOp::Add,
            Op::Sub => BinaryOp::Sub,
            Op::BitAnd => BinaryOp::And,
            Op::BitOr => BinaryOp::Or,
            Op::BitXor => BinaryOp::Xor,
            Op::Eq => BinaryOp::Equal,
            Op::Ne => BinaryOp::NotEqual,
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
        let id = self.rtl.binary(operation, lhs.id, rhs.id)?;
        let value = LoweredExpr {
            id,
            width: if comparison { 1 } else { operand_width },
            signed: !comparison && result_signed,
        };
        self.resize(value, result_width, result_signed)
    }

    fn lower_factor(&mut self, factor: &Factor, env: &Env) -> Result<LoweredExpr, ImportError> {
        match factor {
            Factor::Variable(id, index, select, comptime) => {
                let source = if has_dynamic_array_index(index) {
                    self.lower_dynamic_array_read(*id, index, env)?
                } else {
                    let key = self.key_from_index(*id, index)?;
                    let Some(source) = env.get(&key).copied() else {
                        if comptime.is_const {
                            return self.lower_comptime(comptime, "constant variable");
                        }
                        return Err(ImportError::UnsupportedBehavior(format!(
                            "reference to non-runtime variable {}",
                            self.signal_name(&key)
                        )));
                    };
                    source
                };
                if select.0.len() == 1
                    && select.1.is_none()
                    && evaluated_u64(&select.0[0]).is_none()
                {
                    let index = self.lower_expression(&select.0[0], env)?;
                    let shifted =
                        self.rtl
                            .binary(BinaryOp::ShiftRightLogical, source.id, index.id)?;
                    return Ok(LoweredExpr {
                        id: self.rtl.expression_slice(shifted, 0, BitWidth::new(1)?)?,
                        width: 1,
                        signed: comptime.r#type.signed,
                    });
                }
                let (lsb, width) = static_select(select, source.width)?;
                if lsb == 0 && width == source.width {
                    Ok(source)
                } else {
                    Ok(LoweredExpr {
                        id: self
                            .rtl
                            .expression_slice(source.id, lsb, BitWidth::new(width)?)?,
                        width,
                        signed: comptime.r#type.signed,
                    })
                }
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
            Factor::FunctionCall(_) | Factor::SystemFunctionCall(_) => Err(
                ImportError::UnsupportedBehavior("function call expression".into()),
            ),
        }
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
        let width = concrete_width(&comptime.r#type, context)?;
        if value.is_xz() {
            return Err(ImportError::UnsupportedBehavior(format!(
                "unknown or four-state compile-time {context}"
            )));
        }
        let words = value.payload().to_u64_digits();
        Ok(LoweredExpr {
            id: self
                .rtl
                .constant(Constant::new(BitWidth::new(width)?, words)),
            width,
            signed: comptime.r#type.signed,
        })
    }

    fn lower_dynamic_array_read(
        &mut self,
        id: VarId,
        index: &VarIndex,
        env: &Env,
    ) -> Result<LoweredExpr, ImportError> {
        let elements = self.lower_array_elements(id, index, env)?;
        let first = &elements[0].0;
        let width = self.width(first)?;
        let signed = self.is_signed(first);
        let mut result = self.constant(width, 0);
        result.signed = signed;
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

    fn assign_destination(
        &mut self,
        destination: &AssignDestination,
        value: LoweredExpr,
        reads: &Env,
        writes: &mut Env,
    ) -> Result<BTreeSet<SignalKey>, ImportError> {
        if !has_dynamic_array_index(&destination.index) {
            let key = self.destination_key(destination)?;
            self.assign_key(&key, &destination.select, value, writes)?;
            return Ok(BTreeSet::from([key]));
        }

        let elements = self.lower_array_elements(destination.id, &destination.index, reads)?;
        let mut changed = BTreeSet::new();
        for (key, condition) in elements {
            let current = writes[&key];
            self.assign_key(&key, &destination.select, value, writes)?;
            let assigned = writes[&key];
            writes.insert(
                key.clone(),
                LoweredExpr {
                    id: self.rtl.mux(condition.id, assigned.id, current.id)?,
                    width: current.width,
                    signed: current.signed,
                },
            );
            changed.insert(key);
        }
        Ok(changed)
    }

    fn assign_key(
        &mut self,
        key: &SignalKey,
        select: &VarSelect,
        value: LoweredExpr,
        env: &mut Env,
    ) -> Result<(), ImportError> {
        let total_width = self.width(key)?;
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
        if !self.signals.contains_key(&key) {
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
        self.signal_order
            .iter()
            .filter(|key| key.id == id)
            .cloned()
            .collect()
    }

    fn lower_array_elements(
        &mut self,
        id: VarId,
        index: &VarIndex,
        env: &Env,
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
                    self.lower_expression(expression, env)?,
                ));
            }
        }

        let mut elements = Vec::new();
        for key in self.keys_for_id(id) {
            let mut condition = None;
            let mut matches = true;
            for (index, candidate) in indices.iter().zip(&key.index) {
                match index {
                    LoweredArrayIndex::Static(value) => matches &= value == candidate,
                    LoweredArrayIndex::Dynamic(value) => {
                        let candidate = u64::try_from(*candidate).map_err(|_| {
                            ImportError::UnsupportedBehavior("unpacked array index overflow".into())
                        })?;
                        if value.width < u64::BITS && candidate >= (1_u64 << value.width) {
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
        indexed_name(&self.variable_name(key.id), &key.index)
    }

    fn unsupported_expression(op: Op) -> ImportError {
        ImportError::UnsupportedBehavior(format!("expression operator `{op}`"))
    }
}

fn collect_associative_operands<'a>(
    expression: &'a Expression,
    op: Op,
    operands: &mut Vec<&'a Expression>,
) {
    if let Expression::Binary(lhs, nested_op, rhs, _) = expression
        && *nested_op == op
    {
        collect_associative_operands(lhs, op, operands);
        collect_associative_operands(rhs, op, operands);
    } else {
        operands.push(expression);
    }
}

fn build_associative_source_plan(
    expression: &Expression,
    op: Op,
    leaf_depths: &[usize],
    leaf: &mut usize,
    plans: &mut Vec<AssociativePlan>,
) -> (usize, usize) {
    if let Expression::Binary(lhs, nested_op, rhs, _) = expression
        && *nested_op == op
    {
        let (lhs, lhs_depth) = build_associative_source_plan(lhs, op, leaf_depths, leaf, plans);
        let (rhs, rhs_depth) = build_associative_source_plan(rhs, op, leaf_depths, leaf, plans);
        let plan = plans.len();
        plans.push(AssociativePlan::Combine { lhs, rhs });
        (plan, lhs_depth.max(rhs_depth) + 1)
    } else {
        let plan = *leaf;
        *leaf += 1;
        (plan, leaf_depths[plan])
    }
}

fn rtl_expression_depth(
    module: &RtlModule,
    id: ExprId,
    memo: &mut HashMap<ExprId, usize>,
) -> usize {
    if let Some(depth) = memo.get(&id) {
        return *depth;
    }
    let depth = match module.expressions()[id.index() as usize].kind() {
        ExprKind::Signal(_) | ExprKind::Constant(_) => 0,
        ExprKind::Unary { input, .. } => rtl_expression_depth(module, *input, memo) + 1,
        ExprKind::Binary { lhs, rhs, .. } => {
            rtl_expression_depth(module, *lhs, memo).max(rtl_expression_depth(module, *rhs, memo))
                + 1
        }
        ExprKind::Mux {
            condition,
            then_expr,
            else_expr,
        } => {
            rtl_expression_depth(module, *condition, memo)
                .max(rtl_expression_depth(module, *then_expr, memo))
                .max(rtl_expression_depth(module, *else_expr, memo))
                + 1
        }
        ExprKind::Concat(parts) => parts
            .iter()
            .map(|part| rtl_expression_depth(module, *part, memo))
            .max()
            .unwrap_or_default(),
        ExprKind::Slice { input, .. } => rtl_expression_depth(module, *input, memo),
    };
    memo.insert(id, depth);
    depth
}

fn memory_candidates(
    source: &Module,
    policies: &HashMap<VarId, MemoryInferencePolicy>,
) -> HashSet<VarId> {
    let arrays = source
        .variables
        .values()
        .filter(|variable| variable.kind == VarKind::Variable && !variable.r#type.array.is_empty())
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

// Shift result types retain the left operand's intrinsic width in AIR, while
// the assignment or parent expression width is carried separately as context.
fn binary_width(op: Op, comptime: &Comptime) -> Result<u32, ImportError> {
    if !matches!(
        op,
        Op::LogicShiftL | Op::ArithShiftL | Op::LogicShiftR | Op::ArithShiftR
    ) {
        return concrete_width(&comptime.r#type, "binary expression");
    }
    let width = comptime
        .r#type
        .total_width()
        .ok_or_else(|| ImportError::NonConcreteWidth("shift expression".into()))?
        .max(comptime.expr_context.width);
    u32::try_from(width).map_err(|_| ImportError::WidthTooLarge("shift expression".into()))
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
    expression
        .comptime()
        .get_value()
        .ok()
        .and_then(veryl_analyzer::value::Value::to_u64)
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
    use struo_rtl::{BinaryOp, ExprId, ExprKind, Module as RtlModule};
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

    const ASSOCIATIVE_LOGIC_SOURCE: &str = r"
module AssociativeLogicTop (
    clk       : input  clock,
    data      : input  logic<16>,
    all_result: output logic,
    any_result: output logic,
    skewed_result: output logic,
    marginal_result: output logic,
) {
    var data_q: logic<16>;
    var all_q : logic;
    var any_q : logic;
    var skewed_q: logic;
    var marginal_q: logic;

    always_comb {
        all_result = all_q;
        any_result = any_q;
        skewed_result = skewed_q;
        marginal_result = marginal_q;
    }

    always_ff (clk) {
        data_q = data;
        all_q = data_q[0] && data_q[1] && data_q[2] && data_q[3]
            && data_q[4] && data_q[5] && data_q[6] && data_q[7]
            && data_q[8] && data_q[9] && data_q[10] && data_q[11]
            && data_q[12] && data_q[13] && data_q[14] && data_q[15];
        any_q = data_q[0] || data_q[1] || data_q[2] || data_q[3]
            || data_q[4] || data_q[5] || data_q[6] || data_q[7]
            || data_q[8] || data_q[9] || data_q[10] || data_q[11]
            || data_q[12] || data_q[13] || data_q[14] || data_q[15];
        skewed_q = data_q[0] && data_q[1] && data_q[2] && data_q[3]
            && data_q[4] && data_q[5] && data_q[6]
            && (data_q[8] || data_q[9] || data_q[10] || data_q[11]
                || data_q[12] || data_q[13] || data_q[14] || data_q[15]);
        marginal_q = data_q[0] && data_q[1] && data_q[2] && data_q[3];
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

    fn associative_depth(module: &RtlModule, id: ExprId, operation: BinaryOp) -> usize {
        let expression = &module.expressions()[id.index() as usize];
        match expression.kind() {
            ExprKind::Binary { op, lhs, rhs } if *op == operation => {
                1 + associative_depth(module, *lhs, operation)
                    .max(associative_depth(module, *rhs, operation))
            }
            _ => 0,
        }
    }

    fn logic_depth(module: &RtlModule, id: ExprId) -> usize {
        let expression = &module.expressions()[id.index() as usize];
        match expression.kind() {
            ExprKind::Binary {
                op: BinaryOp::And | BinaryOp::Or,
                lhs,
                rhs,
            } => 1 + logic_depth(module, *lhs).max(logic_depth(module, *rhs)),
            _ => 0,
        }
    }

    #[test]
    fn balances_associative_logical_chains() {
        let design = analyze_and_lower(
            ASSOCIATIVE_LOGIC_SOURCE,
            "associative_logic_lowering",
            "AssociativeLogicTop",
        )
        .unwrap();
        let top = design.top_module().unwrap();

        for (register_name, operation) in [("all_q", BinaryOp::And), ("any_q", BinaryOp::Or)] {
            let register = top
                .registers()
                .iter()
                .find(|register| register.name == register_name)
                .unwrap();
            assert_eq!(associative_depth(top, register.next, operation), 4);
        }
        let skewed = top
            .registers()
            .iter()
            .find(|register| register.name == "skewed_q")
            .unwrap();
        assert_eq!(logic_depth(top, skewed.next), 4);
        let marginal = top
            .registers()
            .iter()
            .find(|register| register.name == "marginal_q")
            .unwrap();
        assert_eq!(associative_depth(top, marginal.next, BinaryOp::And), 3);

        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let mut simulator = ecp5_simulator(&mapped).unwrap().build_native().unwrap();
        let data = simulator.signal("data");
        let mut cases = vec![(0_u16, false, false), (u16::MAX, true, true)];
        cases.extend((0..16).map(|bit| (1_u16 << bit, false, true)));
        cases.extend((0..16).map(|bit| (!(1_u16 << bit), false, true)));
        for (value, all, any) in cases {
            simulator.modify(|io| io.set(data, value)).unwrap();
            tick(&mut simulator);
            tick(&mut simulator);
            assert_value(&mut simulator, "all_result", u64::from(all));
            assert_value(&mut simulator, "any_result", u64::from(any));
        }
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
