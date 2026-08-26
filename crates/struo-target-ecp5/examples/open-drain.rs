//! Emits a minimal ECP5 open-drain I/O netlist for nextpnr.

use std::error::Error;

use struo_ir::Netlist;
use struo_target_ecp5::{OpenDrainIo, map_to_ecp5_with_open_drain_ios};

fn main() -> Result<(), Box<dyn Error>> {
    let mut core = Netlist::new("open_drain");
    let sda_i = core.add_input("sda_i");
    let drive_low = core.add_input("drive_low");
    core.add_output("sda_drive_low", drive_low);
    core.add_output("sda_sampled", sda_i);

    let mapped = map_to_ecp5_with_open_drain_ios(
        &core,
        &[OpenDrainIo::new("sda", "sda_i", "sda_drive_low")],
    )?;
    println!("{}", mapped.to_nextpnr_json()?);
    Ok(())
}
