use std::collections::{BTreeSet, HashMap};

use struo_rtl::{
    BinaryOp, BitWidth, ClockEdge, Constant, Design, Enable, ExprId, ExprKind, Module as RtlModule,
    Polarity, Port, PortDirection, Register, Reset, ResetMode, SignalId, SignalSlice, StateDomain,
    UnaryOp, ValueType,
};
use veryl_analyzer::ir::{
    AssignDestination, Component, Declaration, Expression, Factor, FfDeclaration, IfResetStatement,
    InstDeclaration, Ir, Module, Op, Statement, Type, TypeKind, ValueVariant, VarId, VarKind,
    VarSelect, VarSelectOp,
};

use crate::{ImportError, resolve_name};

type Env = HashMap<VarId, LoweredExpr>;

#[derive(Clone, Copy)]
struct LoweredExpr {
    id: ExprId,
    width: u32,
    signed: bool,
}

struct ModuleLowerer<'a> {
    source: &'a Module,
    rtl: RtlModule,
    signals: HashMap<VarId, SignalId>,
    signal_order: Vec<VarId>,
    widths: HashMap<VarId, u32>,
    signed: HashMap<VarId, bool>,
}

/// Lowers analyzed Veryl AIR into Struo RTL without generated Verilog.
///
/// The current semantic boundary supports scalar packed variables, recursively
/// flattened module instances, analyzer-expanded interface/modport connections,
/// combinational and sequential assignments, static packed selects,
/// conditionals, concatenations, arithmetic, comparisons, shifts, and reset
/// branches. Unsupported AIR is rejected rather than silently discarded.
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
            let r#type = value_type(&variable.r#type, &path.to_string())?;
            let signal = rtl.add_port(Port {
                name: path.to_string(),
                direction,
                r#type,
            });
            signals.insert(*id, signal);
            signal_order.push(*id);
            widths.insert(*id, r#type.width.get());
            signed.insert(*id, r#type.signed);
        }

        let mut internals = source
            .variables
            .values()
            .filter(|variable| variable.kind == VarKind::Variable)
            .collect::<Vec<_>>();
        internals.sort_by_key(|variable| variable.path.to_string());
        for variable in internals {
            let name = variable.path.to_string();
            let r#type = value_type(&variable.r#type, &name)?;
            let signal = rtl.add_signal(name, r#type);
            signals.insert(variable.id, signal);
            signal_order.push(variable.id);
            widths.insert(variable.id, r#type.width.get());
            signed.insert(variable.id, r#type.signed);
        }

        Ok(Self {
            source,
            rtl,
            signals,
            signal_order,
            widths,
            signed,
        })
    }

    fn lower_declarations(&mut self) -> Result<(), ImportError> {
        let mut driven_comb = BTreeSet::new();
        let mut driven_ff = BTreeSet::new();
        for declaration in &self.source.declarations {
            match declaration {
                Declaration::Comb(comb) => {
                    let initial = self.read_env()?;
                    let mut env = initial.clone();
                    let changed = self.lower_statements(&comb.statements, &mut env, false)?;
                    for id in changed {
                        if !driven_comb.insert(id) || driven_ff.contains(&id) {
                            return Err(ImportError::UnsupportedBehavior(format!(
                                "multiple procedural drivers for {}",
                                self.variable_name(id)
                            )));
                        }
                        let signal = self.signal(id)?;
                        let value = env[&id];
                        self.rtl.assign(self.rtl.whole(signal)?, value.id)?;
                    }
                }
                Declaration::Ff(ff) => {
                    let changed = self.lower_ff(ff)?;
                    for id in changed {
                        if !driven_ff.insert(id) || driven_comb.contains(&id) {
                            return Err(ImportError::UnsupportedBehavior(format!(
                                "multiple procedural drivers for {}",
                                self.variable_name(id)
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

        for input in &instance.inputs {
            let child_signal = child.signal(input.id)?;
            let target = inline_signals[&child_signal];
            let width = child.width(input.id)?;
            let value = self.lower_expression(&input.expr, &parent_env)?;
            let value = self.resize(value, width, child.is_signed(input.id))?;
            self.rtl.assign(self.rtl.whole(target)?, value.id)?;
        }

        for output in &instance.outputs {
            let child_signal = child.signal(output.id)?;
            let source = inline_signals[&child_signal];
            let source_width = child.width(output.id)?;
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
        if !child.memories().is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "memories in flattened module instances".into(),
            ));
        }
        if !child.instances().is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "unflattened nested module instance".into(),
            ));
        }

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
        Ok(signals)
    }

    fn destination_slice(
        &self,
        destination: &AssignDestination,
    ) -> Result<SignalSlice, ImportError> {
        self.ensure_scalar_destination(destination)?;
        let signal = self.signal(destination.id)?;
        let (lsb, width) = static_select(&destination.select, self.width(destination.id)?)?;
        Ok(self.rtl.slice(signal, lsb, BitWidth::new(width)?)?)
    }

    fn lower_ff(&mut self, ff: &FfDeclaration) -> Result<BTreeSet<VarId>, ImportError> {
        if !ff.clock.index.0.is_empty() || !ff.clock.select.is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "selected or indexed clocks".into(),
            ));
        }
        let clock = self.signal(ff.clock.id)?;
        let edge = match self.variable_type(ff.clock.id)?.kind {
            TypeKind::ClockNegedge => ClockEdge::Falling,
            TypeKind::Clock | TypeKind::ClockPosedge => ClockEdge::Rising,
            _ => {
                return Err(ImportError::UnsupportedBehavior(
                    "always_ff clock is not a clock type".into(),
                ));
            }
        };
        let initial = self.read_env()?;
        let mut next = initial.clone();
        let mut reset_values = None;
        let mut changed = BTreeSet::new();

        for statement in &ff.statements {
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
                changed.extend(self.lower_statement(statement, &mut next, true)?);
            }
        }

        let reset_control = if let Some(reset) = &ff.reset {
            if !reset.index.0.is_empty() || !reset.select.is_empty() {
                return Err(ImportError::UnsupportedBehavior(
                    "selected or indexed resets".into(),
                ));
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
            Some((self.signal(reset.id)?, mode, polarity))
        } else {
            None
        };

        for id in &changed {
            let signal = self.signal(*id)?;
            let reset = if let (Some(values), Some((reset_signal, mode, polarity))) =
                (&reset_values, reset_control)
            {
                let value = values.get(id).copied().unwrap_or(initial[id]);
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
                name: self.variable_name(*id),
                target: signal,
                next: next.get(id).copied().unwrap_or(initial[id]).id,
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
    ) -> Result<(Env, Env, BTreeSet<VarId>), ImportError> {
        let mut reset = initial.clone();
        let mut next = initial.clone();
        let mut changed = self.lower_statements(&branch.true_side, &mut reset, true)?;
        changed.extend(self.lower_statements(&branch.false_side, &mut next, true)?);
        Ok((reset, next, changed))
    }

    fn lower_statements(
        &mut self,
        statements: &[Statement],
        env: &mut Env,
        sequential: bool,
    ) -> Result<BTreeSet<VarId>, ImportError> {
        let mut changed = BTreeSet::new();
        for statement in statements {
            changed.extend(self.lower_statement(statement, env, sequential)?);
        }
        Ok(changed)
    }

    fn lower_statement(
        &mut self,
        statement: &Statement,
        env: &mut Env,
        sequential: bool,
    ) -> Result<BTreeSet<VarId>, ImportError> {
        match statement {
            Statement::Assign(assign) => {
                if assign.dst.len() != 1 {
                    return Err(ImportError::UnsupportedBehavior(
                        "concatenated assignment destinations".into(),
                    ));
                }
                let destination = &assign.dst[0];
                self.ensure_scalar_destination(destination)?;
                let value = self.lower_expression(&assign.expr, env)?;
                self.assign_env(destination, value, env)?;
                Ok(BTreeSet::from([destination.id]))
            }
            Statement::If(branch) => {
                let condition = self.lower_expression(&branch.cond, env)?;
                let condition = self.boolean(condition)?;
                let base = env.clone();
                let mut true_env = base.clone();
                let mut false_env = base;
                let mut changed =
                    self.lower_statements(&branch.true_side, &mut true_env, sequential)?;
                changed.extend(self.lower_statements(
                    &branch.false_side,
                    &mut false_env,
                    sequential,
                )?);
                for id in &changed {
                    let then_value = true_env[id];
                    let else_value = false_env[id];
                    let width = self.width(*id)?;
                    let then_value = self.resize(then_value, width, self.is_signed(*id))?;
                    let else_value = self.resize(else_value, width, self.is_signed(*id))?;
                    let value = self.rtl.mux(condition.id, then_value.id, else_value.id)?;
                    env.insert(
                        *id,
                        LoweredExpr {
                            id: value,
                            width,
                            signed: self.is_signed(*id),
                        },
                    );
                }
                Ok(changed)
            }
            Statement::IfReset(_) if sequential => Err(ImportError::UnsupportedBehavior(
                "nested if_reset statements".into(),
            )),
            Statement::Null => Ok(BTreeSet::new()),
            Statement::Case(_) => Err(ImportError::UnsupportedBehavior(
                "case statements are not lowered yet".into(),
            )),
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
            Expression::Binary(lhs, op, rhs, comptime) => {
                if *op == Op::As {
                    let lhs = self.lower_expression(lhs, env)?;
                    let width = concrete_width(&comptime.r#type, "cast expression")?;
                    return self.resize(lhs, width, comptime.r#type.signed);
                }
                let lhs = self.lower_expression(lhs, env)?;
                let rhs = self.lower_expression(rhs, env)?;
                let result_width = concrete_width(&comptime.r#type, "binary expression")?;
                self.lower_binary(*op, lhs, rhs, result_width, comptime.r#type.signed)
            }
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
            Expression::ArrayLiteral(_, _) | Expression::StructConstructor(_, _, _) => Err(
                ImportError::UnsupportedBehavior("aggregate expressions".into()),
            ),
        }
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
                if !index.0.is_empty() {
                    return Err(ImportError::UnsupportedBehavior(
                        "unpacked array indexing".into(),
                    ));
                }
                let source = env.get(id).copied().ok_or_else(|| {
                    ImportError::UnsupportedBehavior(format!(
                        "reference to non-runtime variable {}",
                        self.variable_name(*id)
                    ))
                })?;
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
                let ValueVariant::Numeric(value) = &comptime.value else {
                    return Err(ImportError::UnsupportedBehavior(
                        "non-numeric compile-time value".into(),
                    ));
                };
                let width = concrete_width(&comptime.r#type, "literal")?;
                let value = value.to_u64().ok_or_else(|| {
                    ImportError::UnsupportedBehavior("literal wider than 64 value bits".into())
                })?;
                Ok(LoweredExpr {
                    id: self
                        .rtl
                        .constant(Constant::from_u64(BitWidth::new(width)?, value)),
                    width,
                    signed: comptime.r#type.signed,
                })
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

    fn assign_env(
        &mut self,
        destination: &AssignDestination,
        value: LoweredExpr,
        env: &mut Env,
    ) -> Result<(), ImportError> {
        let total_width = self.width(destination.id)?;
        let (lsb, width) = static_select(&destination.select, total_width)?;
        let value = self.resize(value, width, self.is_signed(destination.id))?;
        if lsb == 0 && width == total_width {
            env.insert(destination.id, value);
            return Ok(());
        }
        let current = env[&destination.id];
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
            destination.id,
            LoweredExpr {
                id: self.rtl.concat(parts)?,
                width: total_width,
                signed: self.is_signed(destination.id),
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
            .map(|id| (*id, self.signals[id], self.widths[id], self.signed[id]))
            .collect::<Vec<_>>();
        entries
            .into_iter()
            .map(|(id, signal, width, signed)| {
                Ok((
                    id,
                    LoweredExpr {
                        id: self.rtl.read(signal)?,
                        width,
                        signed,
                    },
                ))
            })
            .collect()
    }

    fn ensure_scalar_destination(
        &self,
        destination: &AssignDestination,
    ) -> Result<(), ImportError> {
        if !destination.index.0.is_empty() {
            return Err(ImportError::UnsupportedBehavior(
                "unpacked array assignment".into(),
            ));
        }
        if !self.signals.contains_key(&destination.id) {
            return Err(ImportError::UnsupportedBehavior(format!(
                "assignment to non-runtime variable {}",
                self.variable_name(destination.id)
            )));
        }
        Ok(())
    }

    fn signal(&self, id: VarId) -> Result<SignalId, ImportError> {
        self.signals.get(&id).copied().ok_or_else(|| {
            ImportError::UnsupportedBehavior(format!(
                "variable {} has no RTL signal",
                self.variable_name(id)
            ))
        })
    }

    fn width(&self, id: VarId) -> Result<u32, ImportError> {
        self.widths.get(&id).copied().ok_or_else(|| {
            ImportError::UnsupportedBehavior(format!(
                "variable {} has no width",
                self.variable_name(id)
            ))
        })
    }

    fn is_signed(&self, id: VarId) -> bool {
        self.signed.get(&id).copied().unwrap_or(false)
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

    fn unsupported_expression(op: Op) -> ImportError {
        ImportError::UnsupportedBehavior(format!("expression operator `{op}`"))
    }
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

fn constant_value(expression: &Expression) -> Result<u64, ImportError> {
    expression
        .comptime()
        .get_value()
        .ok()
        .and_then(veryl_analyzer::value::Value::to_u64)
        .ok_or_else(|| ImportError::UnsupportedBehavior("non-constant replication count".into()))
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
    let first = u32::try_from(constant_value(&select.0[0])?)
        .map_err(|_| ImportError::UnsupportedBehavior("packed select index overflow".into()))?;
    let (lsb, width) = if let Some((operation, end)) = &select.1 {
        let end = u32::try_from(constant_value(end)?)
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
    use struo_synth::synthesize;
    use struo_target_ecp5::map_to_ecp5;

    use crate::analyze_and_lower;

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
