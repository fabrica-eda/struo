//! Emits a minimal ECP5 dedicated-JTAG netlist for nextpnr.
//!
//! JTCK is only the JTAG transport clock. A user-owned top or wrapper must
//! separately bind and constrain any PLL-backed fabric execution clock.

use std::error::Error;

use struo_ir::Netlist;
use struo_target_ecp5::{JtaggBinding, map_to_ecp5_with_jtagg};

fn main() -> Result<(), Box<dyn Error>> {
    let mut core = Netlist::new("jtagg");
    for name in [
        "jtag_tdi",
        "jtag_tck",
        "jtag_rti1",
        "jtag_rti2",
        "jtag_shift",
        "jtag_update",
        "jtag_rst_n",
        "jtag_ce1",
        "jtag_ce2",
    ] {
        core.add_input(name);
    }
    let zero = core.add_constant(false);
    core.add_output("jtag_tdo1", zero);
    core.add_output("jtag_tdo2", zero);

    let mapped = map_to_ecp5_with_jtagg(&core, &JtaggBinding::with_prefix("jtag"))?;
    println!("{}", mapped.to_nextpnr_json()?);
    Ok(())
}
