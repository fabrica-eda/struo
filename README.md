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
interface/modport connections, statically indexed unpacked and interface arrays
across module boundaries, parameter-bounded generate-for instances, procedural
conditionals, static packed selects, procedural case statements with value or
range arms, arithmetic, comparisons, shifts, concatenations, and synchronous or
asynchronous resets. One-dimensional dynamically indexed arrays with one
conditional write port and one registered read port are inferred as synchronous
block memories. Other unsupported constructs fail explicitly.

## First hardware target

| Item | Value |
|---|---|
| Board | Lattice ECP5 Evaluation Board `LFE5UM5G-85F-EVN` |
| FPGA | `LFE5UM5G-85F-8BG381` |
| nextpnr device | `--um5g-85k` |
| Package | `CABGA381` |
| Speed grade | `-8` |
| On-board reference clock | FTDI 12 MHz, pin A10, JP2 short required |

The base pin constraints are in
[`boards/lfe5um5g-85f-evn/base.lpf`](boards/lfe5um5g-85f-evn/base.lpf)
and are based on the Project Trellis `ecp5_evn` example and the Lattice board
manual.

The 12 MHz value describes the no-PLL board smoke-test clock, not an ECP5
performance target. Struo targets 250 MHz for ECP5 implementation QoR; designs
that use this frequency in hardware still require an explicit PLL clock path.

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
cargo run -- carry-benchmark /tmp/struo-carry-benchmark
(cd examples/axi4-smartconnect && veryl fmt --check && veryl check)
```

The implemented synthesis subset includes bitwise logic, reductions, wrapping
addition and subtraction, signed and unsigned comparisons, variable logical
and arithmetic shifts, muxes, concatenation, slicing, registers, enables, and
synchronous or asynchronous constant resets. It performs constant folding and
structural hashing, balances associative reductions, and uses parallel-prefix
comparison networks. Addition and subtraction remain word-level cells until
technology mapping. ECP5 maps operations wider than four bits to `CCU2C` carry
chains by default; explicit carry-chain and LUT-ripple modes are also available
for regression tests and A/B measurements. ECP5 technology mapping enumerates
bounded four-input cuts and selects a cover using a 250 MHz required-time
model. The estimate includes LUT, routing, carry-chain, BRAM, and setup arcs;
fanout-weighted timing selection is enabled for cones that consume about half
the available period before exact referenced-area recovery. Unreachable
Boolean logic is omitted and the selected cover maps to `LUT4`; registers map
to `TRELLIS_FF`.
Synchronous 1R1W memories map directly to ECP5 `DP16KD`
primitives, including width tiling across multiple blocks; inout ports are
rejected explicitly.

`carry-benchmark` emits two equivalent 32-bit registered counters as
`carry.json` and `lut.json`. On nextpnr 0.6, LFE5UM5G-85F speed grade 8, seed 1,
and a 250 MHz timing target, the routed carry version used 38 `TRELLIS_COMB`
sites and reached 472.14 MHz; the LUT-ripple baseline used 65 sites and reached
60.22 MHz. These figures are a reproducible comparison point, not a guaranteed
device specification; rerun place-and-route for the installed nextpnr/chipdb.

```sh
for implementation in carry lut; do
  nextpnr-ecp5 --um5g-85k --package CABGA381 --speed 8 \
    --json "/tmp/struo-carry-benchmark/${implementation}.json" \
    --lpf-allow-unconstrained \
    --textcfg "/tmp/struo-carry-benchmark/${implementation}.config" \
    --report "/tmp/struo-carry-benchmark/${implementation}-report.json" \
    --freq 250 --timing-allow-fail --seed 1
done
```
Module instances are flattened before synthesis; the implemented path consumes
analyzer AIR directly and does not depend on generated Verilog.

The BRAM inference contract is intentionally explicit: read and write ports
share one clock edge, reads have one-cycle latency, writes cover the whole word,
and arrays have one unpacked dimension with at most 16,384 words. Memory reset,
initial contents, byte enables, asynchronous reads, multiple ports, and
defined same-address read/write collision behavior remain unsupported and fail
instead of being lowered to flip-flops silently.

## Veryl AXI4 synthesis stress design

[axi4_crossbar_2x2.veryl](examples/axi4-smartconnect/veryl/axi4_crossbar_2x2.veryl)
keeps the synthesizable crossbar in committed Veryl source. Its Rust harness
invokes the Veryl analyzer and lowers AIR into Struo RTL. The
two-initiator, two-target fabric uses a parameterized Veryl `Axi4Interface` and
modports, forwards AXI4 burst metadata and IDs, buffers AW, W, and AR
independently, streams W and R beats through backpressure, and uses QoS-first
read/write arbitration with round-robin fairness for ties. Registered burst
decode, arbitration, local read-error responses, and response-ID release break
the former crossbar-wide combinational paths. A reusable parameterized decoder
bank uses Veryl's parameter-bounded generate-for to elaborate one burst decoder
per input directly in analyzer AIR; no template engine or generated HDL is
involved. A parameterized QoS arbiter compares every request pair after
elaboration and accepts a caller-provided tie-break order, so the same block
supports round-robin policies beyond two inputs. Each decoder validates FIXED,
INCR, and WRAP footprints, transfer width, FIXED and WRAP length rules, WRAP
alignment, address overflow, target-window containment, and the AXI4 4 KiB
rule. Invalid or unmapped bursts complete locally with `DECERR`.
Tests
take only the analyzed Veryl path through synthesis, ECP5 technology mapping,
and Celox post-map simulation, with Celox's native Veryl frontend as the
reference. Each initiator has two read and two write outstanding slots. A fifth
downstream ID bit records the initiator, allowing responses for different IDs
to return out of order; a repeated ID is held until its earlier transaction
completes to preserve AXI ordering. The simulation tests exercise reverse-order
B/R completion, full-slot backpressure, QoS contention, legal WRAP traffic, and
local error completion through both the reference and post-map paths.

On nextpnr 0.6, LFE5UM5G-85F speed grade 8, seeds 1 through 10, and a 250 MHz
constraint, the routed self-test averages 269.11 MHz and passes timing in 9 of
10 seeds, with a 243.66--290.44 MHz range. Register-enable inference converts
1,090 self-hold muxes to dedicated FF clock enables and reduces the routed
design from 2,148 to 1,133 `TRELLIS_COMB` sites while retaining 1,411
`TRELLIS_FF` sites. Before enable inference, the same ten seeds averaged 268.81
MHz, passed in 9 of 10 seeds, and ranged from 247.10 to 282.89 MHz. Before the
registered crossbar boundaries it reached 68.77 MHz under the same constraint.
Retaining eight ordering comparisons through synthesis and mapping them to
`CCU2C` raises the ten-seed average to 273.34 MHz, passes all ten seeds, and
ranges from 256.15 to 300.75 MHz. This delay-first mapping increases
`TRELLIS_COMB` use from 1,133 to 1,181 sites. Required-depth-constrained LUT
area recovery then reduces that to 1,163 sites while raising the ten-seed
average to 274.89 MHz; all ten seeds pass and range from 260.35 to 288.02 MHz.
Replacing the global LUT-depth budget with the ECP5 required-time model keeps
the same 1,163-site area and passes all ten seeds; the observed distribution is
273.14 MHz average and 255.43--296.21 MHz range. nextpnr timing remains the
sign-off result because placement-dependent routing cannot be predicted by the
pre-route mapper.
The timing tradeoff is additional address/control latency. Address decode
admits one request at a time per address channel and initiator, while W and R
data still stream at one beat per cycle; place and route should be repeated for
a production top, floorplan, and seed.

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
