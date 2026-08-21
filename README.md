# Struo

[![CI](https://github.com/fabrica-eda/struo/actions/workflows/ci.yml/badge.svg)](https://github.com/fabrica-eda/struo/actions/workflows/ci.yml)

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

`struo-frontend-veryl` lowers analyzed Veryl `Comb`, `Ff`, and `Inst`
declarations, including recursively flattened hierarchy, analyzer-expanded
interface/modport connections and parameter-bounded generate-for instances,
procedural conditionals, static packed selects, procedural case statements with
value or range arms, arithmetic, comparisons, shifts, concatenations, and
synchronous or asynchronous resets. Unsupported constructs such as memories
fail explicitly.

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
`FrontendArtifact`, passes it to `Simulator::from_frontend` in memory, and uses
Celox's native execution backend. It does not serialize or parse JSON. Only the
independent nextpnr branch writes Yosys-compatible JSON. Verilog is not an
intermediate representation in this path. Reference simulation continues to
use Celox's Veryl frontend and native execution backend; Struo's internal
synthesis IR is not exported back to Celox.

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
cargo run -- axi4-self-test /tmp/struo-axi4-self-test.nextpnr.json
(cd examples/axi4-smartconnect && veryl fmt --check && veryl check)
```

The implemented synthesis subset includes bitwise logic, reductions, wrapping
addition and subtraction, signed and unsigned comparisons, variable logical
and arithmetic shifts, muxes, concatenation, slicing, registers, enables, and
synchronous or asynchronous constant resets. It performs constant folding and
structural hashing, balances associative reductions, and uses parallel-prefix
comparison networks before mapping Boolean nodes to `LUT4` and registers to
`TRELLIS_FF`. Memories and inout ports are rejected explicitly.
Module instances are flattened before synthesis; the implemented path consumes
analyzer AIR directly and does not depend on generated Verilog.

## Veryl AXI4 synthesis stress design

`examples/axi4-smartconnect` keeps the synthesizable crossbar in committed
Veryl source. Its Rust harness invokes the Veryl analyzer and lowers AIR into
Struo RTL. The
two-initiator, two-target fabric uses a parameterized Veryl `Axi4Interface` and
modports, forwards AXI4 burst metadata and IDs, buffers
AW and W independently, streams W and R beats through backpressure, and uses
QoS-first read/write arbitration with round-robin fairness for ties. A reusable
parameterized decoder bank uses Veryl's parameter-bounded generate-for to
elaborate one burst decoder per input directly in analyzer AIR; no template
engine or generated HDL is involved. A parameterized QoS arbiter compares every
request pair after elaboration and accepts a caller-provided tie-break order, so
the same block supports round-robin policies beyond two inputs. Each decoder
validates FIXED, INCR, and WRAP footprints, transfer width, WRAP length and
alignment, address overflow, target-window containment, and the AXI4 4 KiB
rule. Invalid or unmapped bursts complete locally with `DECERR`. Tests
take only the analyzed Veryl path through synthesis, ECP5 technology mapping,
and Celox post-map simulation, with Celox's native Veryl frontend as the
reference. Each initiator has two read and two write outstanding slots. A fifth
downstream ID bit records the initiator, allowing responses for different IDs
to return out of order; a repeated ID is held until its earlier transaction
completes to preserve AXI ordering. The simulation tests exercise reverse-order
B/R completion, full-slot backpressure, QoS contention, legal WRAP traffic, and
local error completion through both the reference and post-map paths.

`Axi4CrossbarSelfTest` instantiates the interface-based crossbar, an internal
initiator, a target response model, and a result scoreboard. Its physical top
exposes only clock, reset, pass, and fail, avoiding the evaluation board's IO
limit without reducing the protocol widths. The corresponding constraints are
in
[`examples/axi4-smartconnect/constraints/lfe5um5g-85f-evn.lpf`](examples/axi4-smartconnect/constraints/lfe5um5g-85f-evn.lpf).

The complete open-source hardware smoke flow is:

```sh
mkdir -p build/axi4-self-test
cargo run -- axi4-self-test build/axi4-self-test/design.json
nextpnr-ecp5 --um5g-85k --package CABGA381 --speed 8 \
  --json build/axi4-self-test/design.json \
  --lpf examples/axi4-smartconnect/constraints/lfe5um5g-85f-evn.lpf \
  --textcfg build/axi4-self-test/design.config \
  --report build/axi4-self-test/nextpnr-report.json --freq 12
ecppack --svf build/axi4-self-test/design.svf \
  build/axi4-self-test/design.config build/axi4-self-test/design.bit
```
