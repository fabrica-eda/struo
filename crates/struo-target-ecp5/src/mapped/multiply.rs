//! Word multiplication decomposed into unsigned hard-DSP partial products.
use super::{ArithmeticCell, Bit, CCU2C_ARITH_INIT, Ecp5Cell, LutEmitter};

pub(super) fn map_multiply(cell: &ArithmeticCell, emitter: &mut LutEmitter<'_>) {
    let mut lhs = cell
        .lhs()
        .iter()
        .map(|net| emitter.map_net(*net))
        .collect::<Vec<_>>();
    let mut rhs = cell
        .rhs()
        .iter()
        .map(|net| emitter.map_net(*net))
        .collect::<Vec<_>>();
    while lhs.last() == Some(&Bit::Zero) {
        lhs.pop();
    }
    while rhs.last() == Some(&Bit::Zero) {
        rhs.pop();
    }
    let width = cell.outputs().len();
    let mut terms = Vec::new();
    for (a_index, a) in lhs.chunks(18).enumerate() {
        for (b_index, b) in rhs.chunks(18).enumerate() {
            let shift = 18 * (a_index + b_index);
            if shift >= width {
                continue;
            }
            let mut a_bits = Box::new([Bit::Zero; 18]);
            let mut b_bits = Box::new([Bit::Zero; 18]);
            a_bits[..a.len()].copy_from_slice(a);
            b_bits[..b.len()].copy_from_slice(b);
            let product = Box::new(std::array::from_fn(|_| emitter.fresh_wire()));
            let mut term = vec![Bit::Zero; width];
            for (index, wire) in product.iter().enumerate().take(width - shift) {
                term[shift + index] = Bit::Wire(*wire);
            }
            emitter.push_cell(Ecp5Cell::Multiplier {
                name: format!("dsp_{}_{}_{}", cell.name(), a_index, b_index),
                lhs: a_bits,
                rhs: b_bits,
                product,
            });
            terms.push(term);
        }
    }
    // Balanced reduction retains exact modulo-2^width arithmetic. Sign extension
    // is already explicit in RTL, so signed products need no separate semantics.
    let mut level = 0;
    while terms.len() > 1 {
        let mut reduced = Vec::new();
        for (index, pair) in terms.chunks(2).enumerate() {
            reduced.push(if pair.len() == 1 {
                pair[0].clone()
            } else {
                add_words(
                    &pair[0],
                    &pair[1],
                    emitter,
                    &format!("{}_{}_{}", cell.name(), level, index),
                )
            });
        }
        terms = reduced;
        level += 1;
    }
    let result = terms.pop().unwrap_or_else(|| vec![Bit::Zero; width]);
    for (net, bit) in cell.outputs().iter().zip(result) {
        emitter.alias_net(*net, bit);
    }
}

fn add_words(lhs: &[Bit], rhs: &[Bit], emitter: &mut LutEmitter<'_>, name: &str) -> Vec<Bit> {
    let mut output = Vec::with_capacity(lhs.len());
    let mut carry = Bit::Zero;
    for pair in (0..lhs.len()).step_by(2) {
        let mut inputs = [[Bit::Zero; 4]; 2];
        let sums = std::array::from_fn(|_| emitter.fresh_wire());
        for slice in 0..2 {
            let index = pair + slice;
            inputs[slice] = [
                lhs.get(index).copied().unwrap_or(Bit::Zero),
                rhs.get(index).copied().unwrap_or(Bit::Zero),
                Bit::Zero,
                Bit::One,
            ];
            if index < lhs.len() {
                output.push(Bit::Wire(sums[slice]));
            }
        }
        let carry_out = emitter.fresh_wire();
        emitter.push_cell(Ecp5Cell::Ccu2c {
            name: format!("ccu_mul_{name}_{pair}"),
            inputs,
            carry_in: carry,
            sums,
            carry_out,
            init: [CCU2C_ARITH_INIT; 2],
            inject: [false; 2],
        });
        carry = Bit::Wire(carry_out);
    }
    output
}
