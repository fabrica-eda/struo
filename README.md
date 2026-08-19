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
ECP5 mapped netlist    LUT4 / TRELLIS_FF primitives
    ├── Celox FrontendArtifact for post-map simulation
    └── Yosys JSON-compatible serialization for nextpnr
```

Crate responsibilities:

- `struo-frontend-veryl`: adapter pinned to an exact `veryl-analyzer` version
- `struo-rtl`: frontend-independent RTL that preserves hardware semantics
- `struo-ir`: low-level netlist manipulated by synthesis passes
- `struo-synth`: RTL validation, lowering, and optimization pipeline
- `struo-sim`: equivalence policy and release gates
- `struo-celox`: Celox SDK adapter for technology-mapped netlists
- `struo-target-ecp5`: ECP5 primitives, nextpnr serialization, tool recipes,
  and board profiles
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
Post-synthesis simulation converts the same mapped Rust object that is
serialized for nextpnr into a Celox `FrontendArtifact`. Verilog is not an
intermediate representation in this path. Reference simulation continues to
use Celox's native Veryl frontend; Struo's internal synthesis IR is not exported
back to Celox.

The direct backend requires nextpnr-ecp5 and Project Trellis (`ecppack`) after
synthesis. The existing Veryl/Yosys bitstream smoke test remains under
`examples/ecp5-evn-blinky` while the Veryl AIR adapter is completed.

## Development

```sh
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo run -- demo /tmp/struo-blinky.nextpnr.json /tmp/struo-blinky.celox.json
```

The implemented synthesis subset includes bitwise logic, wrapping addition and
subtraction, equality, muxes, concatenation, slicing, registers, enables, and
synchronous or asynchronous constant resets. It performs constant folding and
structural hashing, then maps Boolean nodes to `LUT4` and registers to
`TRELLIS_FF`. Memories, hierarchy, and inout ports are rejected explicitly.

The next frontend unit is complete lowering of Veryl analyzer `Comb`, `Ff`, and
`Inst` nodes into `struo-rtl`; the synthesis and ECP5 mapping path no longer
depends on generated Verilog.
