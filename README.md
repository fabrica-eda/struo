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
- `struo-formal`: native transition systems, AIG/SAT equivalence, and
  transformation-certificate checking
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
performance target. Struo targets 300 MHz for ECP5 implementation QoR; designs
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

`struo-formal` owns the equivalence semantics without a Verilog, Yosys, EQY,
or ABC round trip. It bit-blasts Struo Boolean, arithmetic, comparison,
register-enable, and reset semantics into a structurally hashed AIG, solves the
resulting miters with an internal CDCL SAT kernel, and combines complete base
checks with k-induction. Matching state names are used as correspondence hints
only after their reachable base cases are proved; invalid hints are discarded.
Non-equivalence returns a named input trace through the first mismatching
output cycle. The initial transition-system path requires one clock domain,
known reset-derived state, and no retained memory. A separate linear-time
certificate checker validates boundary-preserving retiming over truth-table
vertices. It derives reset values in both directions: forward moves evaluate
the crossed function, while backward moves solve a reset preimage.
`struo-synth::TimingDrivenRetiming` builds that register-weighted graph, searches
forward and backward placements, checks the selected labels, and rebuilds the
netlist. Clock enables are exposed as feedback muxes during the move and
inferred again afterwards; unrelated clock/reset domains and their fan-in are
fixed boundaries.

ECP5 mapping also performs automatic, certificate-checked backward retiming on
the final LUT4 network, where physical LUT depth and duplicated inputs are
known. It moves a critical sink FF to the unique inputs of its driving LUT,
copies clock-enable semantics, derives every new reset value, and rejects
generated-clock/control uses. The greedy selection reduces maximum-depth
register endpoints without increasing the overall LUT depth and caps cell and
FF growth. This is part of normal mapping; callers do not choose an optimization
mode. Unsupported proof cases fail construction instead of being accepted as
equivalent.

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
comparison networks. A conservative sequential don't-care pass removes a
payload register's clock enable when a same-clock valid register and structural
influence analysis prove that the payload is unobservable while invalid. This
lets source RTL retain natural conditional assignments without putting their
hold muxes on timing-critical paths. Addition and subtraction remain
word-level cells until technology mapping. ECP5 maps operations wider than
four bits to `CCU2C` carry chains by default; explicit carry-chain and
LUT-ripple modes are also available for regression tests and A/B measurements.
ECP5 technology mapping enumerates bounded four-input cuts and selects a cover
using a 300 MHz required-time model. The estimate includes LUT, routing,
carry-chain, BRAM, and setup arcs; fanout-weighted timing selection is enabled
for cones that consume about half the available period before exact
referenced-area recovery. Unreachable Boolean logic is omitted and the
selected cover maps to `LUT4`; registers map to `TRELLIS_FF`.
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
reference. Each initiator has two issued-read slots plus one queued AR, and two
write outstanding slots. A fifth downstream ID bit records the initiator,
allowing responses for different IDs to return out of order; a repeated ID may
enter the AR skid register but is not issued until its earlier transaction
completes, preserving AXI ordering. The simulation tests exercise reverse-order
B/R completion, queued and repeated read IDs, QoS contention, legal WRAP
traffic, and local error completion through both the reference and post-map
paths.

On nextpnr 0.6, LFE5UM5G-85F speed grade 8, and seeds 1 through 10, the 300 MHz
flow passes all ten seeds and reaches 302.94--319.59 MHz (309.80 MHz mean). The
post-LUT retimer reduces four-LUT register endpoints from 15 to 6 and moves the
former address-decoder critical path off the routed worst path, at a cost of 19
FFs. The mapped self-test uses 1,193 `TRELLIS_COMB` and 1,452 `TRELLIS_FF` sites.
Without post-LUT retiming the same seeds reached 303.03--316.66 MHz (307.62 MHz
mean) with 1,433 FFs. The qualified-payload pass removes 69 of 1,109 inferred
clock enables and makes the eight QoS comparison preregisters unnecessary. The
remaining result combines the mapper's 300 MHz required-time cover with
registered write-completion, read-slot reservation, scoreboard, and arbitration
boundaries in the RTL.
nextpnr uses timing budgets, a heap timing weight of 30,
and timing-driven routing rip-up for every seed; no successful seed is selected
after the fact. The CI timing gate repeats all ten fixed seeds and requires each
one to reach 300 MHz. nextpnr timing remains the sign-off result because
placement-dependent routing cannot be predicted by the pre-route mapper.
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
  --report build/axi4-self-test/nextpnr-report.json \
  --freq 300 --placer-budgets --placer-heap-timingweight 30 --tmg-ripup
ecppack --svf build/axi4-self-test/design.svf \
  build/axi4-self-test/design.config build/axi4-self-test/design.bit
```
