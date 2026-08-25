//! Debug helper: dump lowered RTL registers for a self-contained Veryl source.
//!
//! Usage: cargo run --release -p struo-cli --example dump-ir -- <VERYL> <TOP>

use std::fs;

use struo::analyze_and_lower;
use struo::rtl::{ExprId, ExprKind, Module, SignalId, SignalSlice};

fn signal_name(module: &Module, id: SignalId) -> String {
    module
        .signals()
        .iter()
        .find(|s| s.id().index() == id.index())
        .map_or_else(|| format!("sig{}", id.index()), |s| s.name().to_owned())
}

fn slice_signal_name(module: &Module, slice: &SignalSlice) -> String {
    signal_name(module, slice.signal)
}

fn walk(module: &Module, id: ExprId, depth: usize) {
    let Some(expr) = module.expressions().iter().find(|e| e.id() == id) else {
        println!("{}<missing expr {id:?}>", "  ".repeat(depth));
        return;
    };
    let pad = "  ".repeat(depth);
    match expr.kind() {
        ExprKind::Signal(slice) => println!(
            "{pad}sig:{}[{}+{}]",
            slice_signal_name(module, slice),
            slice.lsb,
            slice.width.get(),
        ),
        ExprKind::Constant(c) => println!("{pad}const({c:?})"),
        ExprKind::Unary { op, input } => {
            println!("{pad}{op:?}");
            walk(module, *input, depth + 1);
        }
        ExprKind::Binary { op, lhs, rhs } => {
            println!("{pad}{op:?}");
            walk(module, *lhs, depth + 1);
            walk(module, *rhs, depth + 1);
        }
        ExprKind::Mux {
            condition,
            then_expr,
            else_expr,
        } => {
            println!("{pad}mux");
            walk(module, *condition, depth + 1);
            walk(module, *then_expr, depth + 1);
            walk(module, *else_expr, depth + 1);
        }
        ExprKind::Concat(parts) => {
            println!("{pad}concat");
            for part in parts {
                walk(module, *part, depth + 1);
            }
        }
        ExprKind::Slice { input, lsb } => {
            println!("{pad}slice lsb={lsb}");
            walk(module, *input, depth + 1);
        }
    }
}

fn main() {
    let path = std::env::args().nth(1).expect("veryl path");
    let top = std::env::args().nth(2).expect("top");
    let source = fs::read_to_string(&path).expect("read veryl");
    let project = std::path::Path::new(&path)
        .parent()
        .and_then(|p| p.file_name())
        .and_then(|n| n.to_str())
        .unwrap_or("bench")
        .to_owned();
    let design = analyze_and_lower(&source, &project, &top).expect("lower");
    let module = design.top_module().expect("top module");
    for register in module.registers() {
        println!(
            "register {} target={}",
            register.name,
            signal_name(module, register.target),
        );
        walk(module, register.next, 1);
        println!();
    }
}
