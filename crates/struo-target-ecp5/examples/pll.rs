//! Emits a user-configured 12 MHz to 250 MHz ECP5 PLL netlist for nextpnr.
//!
//! The divider values are intentionally supplied by this user-owned boundary
//! binding rather than calculated or selected by Struo.

use std::error::Error;

use struo_ir::{ClockEdge, Netlist, RegisterCell};
use struo_target_ecp5::{PllBinding, PllOutput, map_to_ecp5_with_pll};

fn main() -> Result<(), Box<dyn Error>> {
    let mut core = Netlist::new("pll_top");
    core.add_input("clk");
    let clk_250 = core.add_input("clk_250");
    let pll_locked = core.add_input("pll_locked");
    let data = core.add_input("data");
    let data_q = core.add_register_output("data_q");
    core.add_register(RegisterCell::new(
        "data_q",
        data_q,
        data,
        clk_250,
        ClockEdge::Rising,
        None,
        None,
    ));
    core.add_output("data_q", data_q);
    core.add_output("locked", pll_locked);

    // Generated with: ecppll -i 12 -o 250 --highres
    let mut pll = PllBinding::new(
        "clk",
        "clk_250",
        "pll_locked",
        PllOutput::Clkos,
        PllOutput::Clkop,
    );
    pll.parameters.extend(
        [
            ("PLLRST_ENA", "DISABLED"),
            ("INTFB_WAKE", "DISABLED"),
            ("STDBY_ENABLE", "DISABLED"),
            ("DPHASE_SOURCE", "DISABLED"),
            ("OUTDIVIDER_MUXA", "DIVA"),
            ("OUTDIVIDER_MUXB", "DIVB"),
            ("OUTDIVIDER_MUXC", "DIVC"),
            ("OUTDIVIDER_MUXD", "DIVD"),
            ("CLKI_DIV", "3"),
            ("CLKOP_ENABLE", "ENABLED"),
            ("CLKOP_DIV", "25"),
            ("CLKOP_CPHASE", "9"),
            ("CLKOP_FPHASE", "0"),
            ("CLKOS_ENABLE", "ENABLED"),
            ("CLKOS_DIV", "2"),
            ("CLKOS_CPHASE", "0"),
            ("CLKOS_FPHASE", "0"),
            ("FEEDBK_PATH", "CLKOP"),
            ("CLKFB_DIV", "5"),
        ]
        .map(|(name, value)| (name.into(), value.into())),
    );
    pll.attributes.extend(
        [
            ("FREQUENCY_PIN_CLKI", "12"),
            ("FREQUENCY_PIN_CLKOS", "250"),
            ("ICP_CURRENT", "12"),
            ("LPF_RESISTOR", "8"),
            ("MFG_ENABLE_FILTEROPAMP", "1"),
            ("MFG_GMCREF_SEL", "2"),
        ]
        .map(|(name, value)| (name.into(), value.into())),
    );

    let mapped = map_to_ecp5_with_pll(&core, &pll)?;
    println!("{}", mapped.to_nextpnr_json()?);
    Ok(())
}
