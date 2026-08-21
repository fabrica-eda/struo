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
- `struo-sample-axi4`: Veryl-authored protocol-level synthesis stress design
- `struo-ir`: low-level netlist manipulated by synthesis passes
- `struo-synth`: RTL validation, lowering, and optimization pipeline
- `struo-sim`: equivalence policy and release gates
- `struo-celox`: Celox SDK adapter for technology-mapped netlists
- `struo-target-ecp5`: ECP5 primitives, nextpnr serialization, tool recipes,
  and board profiles
- `struo-cli`: compiler driver

`struo-frontend-veryl` lowers analyzed Veryl `Comb` and `Ff` declarations,
including procedural conditionals, static packed selects, arithmetic,
comparisons, shifts, concatenations, and synchronous or asynchronous resets.
Unsupported constructs such as hierarchy and memories fail explicitly. The
older shell inventory API remains available for compatibility and never drops
pending behavior silently.

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
Post-synthesis simulation converts the mapped Rust object directly into a Celox
`FrontendArtifact` and passes it to `Simulator::from_frontend` in memory. It
does not serialize or parse JSON. Only the independent nextpnr branch writes
Yosys-compatible JSON. Verilog is not an intermediate representation in this
path. Reference simulation continues to use Celox's native Veryl frontend;
Struo's internal synthesis IR is not exported back to Celox.

The direct backend requires nextpnr-ecp5 and Project Trellis (`ecppack`) after
synthesis. The existing Veryl/Yosys bitstream smoke test remains under
`examples/ecp5-evn-blinky` while the Veryl AIR adapter is completed.

## Development

```sh
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo run -- demo /tmp/struo-blinky.nextpnr.json
cargo run -- axi4-demo /tmp/struo-axi4.nextpnr.json
(cd crates/struo-sample-axi4 && veryl fmt --check && veryl check)
```

The implemented synthesis subset includes bitwise logic, reductions, wrapping
addition and subtraction, signed and unsigned comparisons, variable logical
and arithmetic shifts, muxes, concatenation, slicing, registers, enables, and
synchronous or asynchronous constant resets. It performs constant folding and
structural hashing, then maps Boolean nodes to `LUT4` and registers to
`TRELLIS_FF`. Memories, hierarchy, and inout ports are rejected explicitly.

The next frontend unit is hierarchy and `Inst` lowering. The implemented
single-module path already consumes analyzer AIR directly and does not depend
on generated Verilog.

## Veryl AXI4 synthesis stress design

`struo-sample-axi4` keeps the synthesizable crossbar in committed Veryl source.
Its Rust API invokes the Veryl analyzer and lowers AIR into Struo RTL. The
two-initiator, two-target fabric forwards AXI4 burst metadata and IDs, buffers
AW and W independently, streams W and R beats through backpressure, uses
separate read/write round-robin arbitration, and locally completes unmapped
bursts with `DECERR`. Tests take only the analyzed Veryl path through synthesis,
ECP5 technology mapping, and Celox post-map simulation, with Celox's native
Veryl frontend as the reference. Each initiator has two read and two write
outstanding slots. A fifth downstream ID bit records the initiator, allowing
responses for different IDs to return out of order; a repeated ID is held until
its earlier transaction completes to preserve AXI ordering. The simulation
tests exercise reverse-order B/R completion and full-slot backpressure through
both the reference and post-map paths.

The standalone crossbar exposes every AXI signal for simulation and therefore
exceeds the evaluation board's physical IO count. Place-and-route coverage will
use an internal self-test wrapper once hierarchy and instance-port lowering are
implemented; reducing the protocol widths merely to satisfy top-level IO would
not be a representative AXI4 design.
