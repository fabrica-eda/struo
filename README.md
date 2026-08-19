# Struo

A Rust workspace implementing logic synthesis from Veryl to Lattice ECP5.
Frontend-specific types do not leak into the synthesis core. Post-synthesis
simulation and equivalence checking are mandatory gates for bitstream release.

## Compiler pipeline

```text
Veryl source
    │
    ▼
veryl-analyzer 0.20.3
    │  struo-frontend-veryl
    ▼
struo-rtl              module / type / clock / reset / register / memory
    │  struo-synth
    ▼
struo-ir               bit-level, target-independent netlist
    │  technology mapping
    ▼
ECP5 netlist           LUT / FF / EBR / DSP / IO primitives
```

Crate responsibilities:

- `struo-frontend-veryl`: adapter pinned to an exact `veryl-analyzer` version
- `struo-rtl`: frontend-independent RTL that preserves hardware semantics
- `struo-ir`: low-level netlist manipulated by synthesis passes
- `struo-synth`: RTL validation, lowering, and optimization pipeline
- `struo-sim`: RTL/gate simulation, equivalence, and release gates
- `struo-target-ecp5`: ECP5 primitives, tool recipes, and board profiles
- `struo-cli`: compiler driver

`struo-frontend-veryl` currently converts only the module and port shell.
`require_fully_lowered` fails while unsupported combinational logic, flip-flops,
or instances remain, preventing synthesis from silently discarding behavior.

## First hardware target

| Item | Value |
|---|---|
| Board | Lattice ECP5 Evaluation Board `LFE5UM5G-85F-EVN` |
| FPGA | `LFE5UM5G-85F-8BG381` |
| nextpnr device | `--um5g-85k` |
| Package | `CABGA381` |
| Speed grade | `-8` |
| Default clock | FTDI 12 MHz, pin A10, JP2 short required |

The base pin constraints are in
[`boards/lfe5um5g-85f-evn/base.lpf`](boards/lfe5um5g-85f-evn/base.lpf)
and are based on the Project Trellis `ecp5_evn` example and the Lattice board
manual.

## Mandatory verification

Every stage below must pass before a bitstream can be released:

1. hardware-semantic RTL simulation
2. equivalence between RTL and the synthesized netlist
3. unresolved/black-box primitive check
4. post-synthesis simulation of the ECP5 technology-mapped netlist
5. place and route
6. timing closure

`struo-sim::VerificationReport::authorize_bitstream` rejects bitstream
packaging and programming if any stage is missing, skipped, or failed.
Post-synthesis simulation uses Yosys `+/ecp5/cells_sim.v`; black-box-only models
must not be used to make a simulation pass.

The intended open-source toolchain consists of Yosys, nextpnr-ecp5, Project
Trellis (`ecppack`), Icarus Verilog or Verilator, and a formal engine. A
bitstream-generation smoke test for a minimal Veryl circuit is available under
`examples/ecp5-evn-blinky`.

## Development

```sh
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo run -- demo
```

The next implementation unit is complete lowering of the Veryl analyzer's
`Comb`, `Ff`, and `Inst` nodes into `struo-rtl`, followed by running identical
test vectors in the RTL and ECP5 gate-level simulators.
