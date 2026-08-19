# ECP5 EVN blinky

This is the first end-to-end smoke test for `LFE5UM5G-85F-EVN`. Short JP2 to
connect the FTDI 12 MHz clock to the FPGA. `btn` is used as an active-low
asynchronous reset.

Yosys, nextpnr-ecp5, and Project Trellis must be available on `PATH`. On Ubuntu
24.04, install the following packages; Veryl must be installed separately.

```sh
sudo apt install yosys nextpnr-ecp5 fpga-trellis fpga-trellis-database openfpgaloader
```

```sh
./build.sh
```

In addition to the bitstream, `build/` contains the post-technology-mapping
netlist, post-P&R SDF, utilization and timing report, and stage logs. Veryl
emits generated SystemVerilog into `build/veryl/`; source directories remain
generated-file-free.

| Artifact | Purpose |
| --- | --- |
| `Top.bit` | Bitstream to load into the FPGA |
| `Top.svf` | JTAG/SVF programming stream |
| `Top.synth.v` | Technology-mapped netlist |
| `Top.sdf` | Post-P&R delay annotation |
| `nextpnr-report.json` | Machine-readable utilization and timing report |
| `blackbox-check.log` | Unresolved-cell check for the mapped netlist |

To load the bitstream into SRAM:

```sh
openFPGALoader -b ecp5_evn build/Top.bit
```

This is a bitstream-generation smoke test, not a substitute for post-synthesis
functional verification. Gate-level simulation using `Top.synth.v` and
`Top.sdf` will be added as a separate release gate.
