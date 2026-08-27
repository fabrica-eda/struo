# Struo

[![CI](https://github.com/fabrica-eda/struo/actions/workflows/ci.yml/badge.svg)](https://github.com/fabrica-eda/struo/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/struo.svg)](https://crates.io/crates/struo)

A Rust workspace implementing logic synthesis from Veryl to Lattice ECP5.
Frontend-specific types do not leak into the synthesis core. Post-synthesis
simulation and equivalence checking are mandatory gates for bitstream release.

## Install

Applications only need the `struo` facade crate:

```toml
[dependencies]
struo = "0.1.0"
```

Its default features include the complete Veryl-to-ECP5 flow and Celox
post-synthesis adapter. The implementation layers are available through stable
namespaces without adding each crate separately:

```rust,no_run
use struo::{analyze_project_and_lower, ecp5_simulator, map_to_ecp5, synthesize};

# fn run() -> Result<(), Box<dyn std::error::Error>> {
let design = analyze_project_and_lower(".", "Top")?;
let synthesized = synthesize(&design)?;
let mapped = map_to_ecp5(&synthesized.netlist)?;
let simulator = ecp5_simulator(&mapped)?.build_native()?;
# let _ = simulator;
# Ok(())
# }
```

For lower-level integrations, disable default features and opt into only the
adapters required: `veryl`, `ecp5`, and `celox` (`celox` implies `ecp5`). The
core `rtl`, `ir`, `synth`, `formal`, and `sim` modules are always available.

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

- `struo`: single-dependency facade for applications
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
manual. The matching nextpnr pre-pack clock constraint is
[`boards/lfe5um5g-85f-evn/clock-12.py`](boards/lfe5um5g-85f-evn/clock-12.py).

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

ECP5 mapping also performs automatic, certificate-checked retiming on the final
LUT4 and CCU2C network, where physical depth, carry hops, and duplicated inputs
are known. Backward moves split a critical sink FF across a LUT or carry slice;
forward moves score a complete registered carry chain as one candidate so an
unprofitable half-moved chain is never selected. Both directions preserve clock
enables and derive every new reset value from the primitive truth table.
Equivalent generated FFs are shared only up to fanout two, retaining useful
physical replication for routing. LUT-driven clock enables with moderate
fanout are replicated once into groups of at most 16 sinks; blanket replication
of larger control nets is avoided because it destabilizes placement. The
search reduces timing cutsets without increasing the worst mapped data,
clock-enable, or output period and caps cell and FF growth. Each accepted
primitive move contributes its checked certificate
to a proof ledger; exact equivalent-FF merges, truth-table-identical logic
replication, and unobservable-cell removal are recorded as constructive
equivalence steps. Final driver/connectivity validation signs off the composed
chain, and an unsigned candidate cannot replace the original mapped netlist.
This is part of normal mapping; callers do not choose an optimization mode.
Unsupported proof cases fail construction instead of being accepted as
equivalent.

For placement-dependent misses, the ECP5 backend also supports a deterministic
physical-feedback loop. A routed draft emits exact cell locations and detailed
per-sink timing budgets; synthesis then clones truth-table-identical LUTs and
moves only the branches that exceeded their routed budget. Existing cells keep
their draft BELs while the new replicas remain free for placement. The local
rewrite is attempted only when every clock is already within 98 percent of its
goal, where a small routing repair is appropriate. If no such branch exists and
the design is within 95 percent, the reported register-to-register critical
path guides a whole-LUT boundary move at its sink. The move is checked by the
retiming certificate machinery, including reset derivation, before it becomes
an implementation candidate. A bounded search also tries the opposite
registered-input cut and one additional critical-cone move, emitting at most
three proof-signed JSON candidates without another optimization switch. Clock,
reset, and enable nets are excluded from
generic data rewrites. LUT replication retains compatible draft BELs; retiming
changes packing boundaries and therefore receives a fresh placement with the
same fixed seed. `Ecp5Flow::draft_place_and_route_command` and
`Ecp5Flow::physical_candidate_place_and_route_commands` produce the draft and
same-seed candidate configurations. The best candidate is accepted only when
every reported clock is no worse and at least one is strictly faster;
`pack_physical_candidates_command` otherwise rolls back to the already-routed
draft. Designs that already meet timing are not rewritten. This is
feedback-directed physical synthesis rather than seed selection.

The direct backend requires nextpnr-ecp5 and Project Trellis (`ecppack`) after
synthesis. The existing Veryl/Yosys bitstream smoke test remains under
`examples/ecp5-evn-blinky` while the Veryl AIR adapter is completed.

## Development

```sh
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo run -p struo-cli --example demo -- /tmp/struo-blinky.nextpnr.json
cargo run -p struo-cli --example axi4-demo -- /tmp/struo-axi4.nextpnr.json
cargo run -p struo-cli --example axi4-self-test -- /tmp/struo-axi4-self-test.nextpnr.json
cargo run -p struo-cli --example carry-benchmark -- /tmp/struo-carry-benchmark
struo bench/designs/counter32 --top counter32 --output /tmp/struo-counter32.nextpnr.json
python3 bench/scripts/qor.py
(cd examples/axi4-smartconnect && veryl fmt --check && veryl check)
```

`bench/scripts/qor.py` runs the QoR benchmark suite comparing Struo against
a pinned Yosys `synth_ecp5` baseline on shared Veryl sources with identical
nextpnr seeds and timing goals; methodology and current results are in
[`bench/README.md`](bench/README.md).

The `struo` positional input is a Veryl project directory or its `Veryl.toml`.
Struo resolves every configured source, the standard library, and locked
dependencies before running the analyzer.

The implemented synthesis subset includes bitwise logic, reductions, wrapping
addition and subtraction, signed and unsigned comparisons, variable logical
and arithmetic shifts, muxes, concatenation, static slicing, dynamic packed
bit selection, module constants, registers, enables, and synchronous or
asynchronous constant resets. It performs constant folding and structural
hashing, balances associative reductions, and uses parallel-prefix comparison
networks. A conservative sequential don't-care pass removes a
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
The frequency goal applies to synchronous paths. Top-level I/O paths are
unconstrained unless their delays are named explicitly, so a combinational or
I/O-to-register path is not silently treated as a 300 MHz path. Pass a JSON
file with `--io-timing-constraints`, for example:

```json
{
  "input_delays_ps": { "request": 500 },
  "output_delays_ps": { "response": 750 }
}
```

An input delay is the maximum arrival time at the FPGA boundary after the
launching edge. An output delay reserves that much of the clock period beyond
the boundary. Constraints apply to every bit of the named port; omitted ports
remain unconstrained. Library callers can construct the same data with
`IoTimingConstraints` and call `map_to_ecp5_with_constraints`.
Synchronous 1R1W memories map directly to ECP5 `DP16KD`
primitives, including width tiling across multiple blocks. General four-state
RTL `inout` ports remain rejected explicitly. Split open-drain interfaces can
instead be bound at the ECP5 boundary to a physical `TRELLIS_IO`.

For I²C and similar wired-AND buses, keep the verified core interface split
into an input and an active-high drive-low output, then bind the pair after
synthesis:

```rust
use struo::{OpenDrainIo, map_to_ecp5_with_open_drain_ios};

let bindings = [OpenDrainIo::new("sda", "sda_i", "sda_drive_low")];
let mapped = map_to_ecp5_with_open_drain_ios(&synthesized.netlist, &bindings)?;
```

The compiler driver accepts the same binding as a repeatable option:

```sh
struo . --top Top --output Top.json \
  --open-drain sda:sda_i:sda_drive_low
```

The two logical ports are replaced by one scalar `inout sda` port. ECP5
mapping emits `TRELLIS_IO` with `DIR=BIDIR`, drives only zero, and uses the
inverted `sda_drive_low` signal for the active-high tristate input. The board
must provide the normal external I²C pull-up. Bind SCL the same way when clock
stretching or multi-controller arbitration is required. The mapped Celox model
treats the physical pad as an external input while released and forces its
readback low while the core is pulling it low.

### Dedicated ECP5 JTAG access

Model `JTAGG` in Veryl as an ordinary top-level fabric interface, rather than
as an SV attribute or a vendor-named source instance. This keeps the core
portable and makes JTAG transactions directly driveable in RTL simulation.
For a `jtag` prefix, declare these scalar ports on the selected top:

```veryl
module Top (
    // Signals driven by the ECP5 TAP into the core.
    jtag_tdi   : input logic,
    jtag_tck   : input clock,
    jtag_rti1  : input logic,
    jtag_rti2  : input logic,
    jtag_shift : input logic,
    jtag_update: input logic,
    jtag_rst_n : input reset_async_low,
    jtag_ce1   : input logic,
    jtag_ce2   : input logic,

    // Data returned from the core to extension registers one and two.
    jtag_tdo1  : output logic,
    jtag_tdo2  : output logic,
) {
    always_comb {
        // Replace these with extension-register shift logic.
        jtag_tdo1 = 0;
        jtag_tdo2 = 0;
    }
}
```

After synthesis, bind all eleven ports atomically at the ECP5 boundary:

```rust
use struo::{JtaggBinding, map_to_ecp5_with_jtagg};

let mut jtagg = JtaggBinding::with_prefix("jtag");
jtagg.extension_register_2 = false; // when only ER1 is used
let mapped = map_to_ecp5_with_jtagg(&synthesized.netlist, &jtagg)?;
```

The compiler driver provides the equivalent `--jtagg-prefix jtag` option. The
binding removes the logical ports from the package-pin list and emits one
`JTAGG` cell using the device's dedicated JTAG pins. Missing, repeated,
non-scalar, or incorrectly directed ports fail mapping instead of producing a
partially connected primitive. Post-map Celox simulation holds the inaccessible
physical TAP in its inactive state; simulate JTAG traffic before binding when
the protocol itself is under test.

`jtag_tck` is only the TAP transport clock; binding `JTAGG` does not create or
replace a fabric system clock. The evaluation board's direct FTDI clock remains
12 MHz. A user top or wrapper whose logic is intended to run at 250 MHz must
provide a supported PLL path and constrain both its 12 MHz reference and the
derived 250 MHz clock. That user-owned boundary is independent of
`--jtagg-prefix` and of the JTAG programming transport.

### User-configured ECP5 PLL

The current Veryl analyzer IR does not retain enough named-port, parameter, and
SV-attribute metadata for Struo to reproduce a vendor primitive instance.
Until that upstream boundary is extended, declare the reference clock,
generated clock, and lock signal as ordinary scalar inputs for RTL simulation,
then apply `PllBinding` after synthesis. The binding owns no frequency policy:
the user supplies the `EHXPLLL` parameters and attributes.

For example, the Veryl top-level boundary for the supplied board configuration
contains these ports; the core uses `clk_250`, while a testbench drives the
logical generated clock and lock inputs before target binding:

```veryl
module Top (
    clk:        input 'a clock,
    clk_250:    input 'b clock,
    pll_locked: input 'b logic,
) {
    // Core sequential logic uses clk_250.
}
```

The compiler driver removes `clk_250` and `pll_locked` from the physical pin
list and connects them to one `EHXPLLL` selected by the user-owned JSON:

```sh
struo . --top Top --output build/Top.json \
  --pll-binding boards/lfe5um5g-85f-evn/pll-12-to-250.json
```

[`crates/struo-target-ecp5/examples/pll.rs`](crates/struo-target-ecp5/examples/pll.rs)
contains a complete 12 MHz to 250 MHz high-resolution configuration generated
with `ecppll -i 12 -o 250 --highres`. It retains physical port `clk`, replaces
logical inputs `clk_250` and `pll_locked`, uses `CLKOS` for the fabric clock,
and feeds `CLKOP` back to `CLKFB`. The same configuration is available as
[`boards/lfe5um5g-85f-evn/pll-12-to-250.json`](boards/lfe5um5g-85f-evn/pll-12-to-250.json);
replace that file with another valid device configuration to choose a different
frequency or PLL topology.

```sh
cargo run -q -p struo-target-ecp5 --example pll > /tmp/struo-pll.json
nextpnr-ecp5 --um5g-85k --package CABGA381 --speed 8 \
  --json /tmp/struo-pll.json --lpf-allow-unconstrained \
  --pre-pack boards/lfe5um5g-85f-evn/clock-12.py \
  --textcfg /tmp/struo-pll.config --freq 250
```

The pre-pack constraint names the physical `clk` net at 12 MHz. nextpnr then
derives the generated constraint from the supplied PLL dividers; `--freq 250`
is the default for otherwise unconstrained clocks and the implementation target.
Use the normal complete LPF rather than `--lpf-allow-unconstrained` for a
programmable board build.

`carry-benchmark` emits two equivalent 32-bit registered counters as
`carry.json` and `lut.json`. On nextpnr 0.6, LFE5UM5G-85F speed grade 8, seed 1,
and a 250 MHz timing target, the routed carry version used 38 `TRELLIS_COMB`
sites and reached 472.14 MHz; the LUT-ripple baseline used 65 sites and reached
60.22 MHz. These figures are a reproducible comparison point, not a guaranteed
device specification; rerun place-and-route for the installed nextpnr/chipdb.
Here `--freq 250` is a timing constraint for the benchmark's logical clock, not
a board clock configuration or a PLL implementation.

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

Inference intent can be attached to an unpacked array with Veryl's portable
SystemVerilog-attribute escape. `preferred` is the default and preserves
automatic inference, `required` makes a failed match an explicit diagnostic,
and `forbidden` prevents the array from becoming a memory:

```veryl
#[sv("struo_memory = \"required\"")]
var words: logic<32> [1024];
```

Veryl 0.20.3 rejects tool-defined attribute names, which is why this uses
`sv(...)` instead of a Struo-specific attribute namespace. Struo consumes only
the `struo_memory` key and ignores unrelated `sv` attributes. A required array
that violates the contract reports the array name and the first unsupported
port property. Physical geometry is checked later by the target mapper, which
also fails rather than replacing an inferred memory with registers.

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
flow passes all ten seeds and reaches 302.66--313.77 MHz (306.78 MHz mean) with
1,189 `TRELLIS_FF` and 1,197 `TRELLIS_COMB` sites. The burst decoder is natural
three-cycle RTL: its registered operands feed the 17-bit address addition and
the output holding state directly, without a hand-written carry-result pipeline
stage. Its single-flight handshake keeps the address and protocol metadata
stable until the result is consumed. Removing that explicit boundary reduces
the RTL from 1,365 to 1,293 registers, and certified retiming trims another 104
registers through twenty accepted moves. The retiming score prices
routing-burden growth into every candidate, so moves that would trade placement
pressure for model period are rejected rather than degrading routed results;
moving the last-address addition into stage zero was evaluated exactly this way
and rejected after routed results collapsed to 239--256 MHz across all seeds.
The mapped timing model scores both CCU2C carry hops and ordinary LUT routing,
including a post-map routing guard calibrated against the routed AXI paths.
The qualified-payload pass removes 69 inferred clock enables and makes the
eight QoS comparison preregisters unnecessary. The remaining result combines
the mapper's 300 MHz required-time cover with registered write-completion,
read-slot reservation, scoreboard, and arbitration boundaries in the RTL.
nextpnr uses timing budgets, a heap timing weight of 40,
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
cargo run -p struo-cli --example axi4-self-test -- build/axi4-self-test/design.json
nextpnr-ecp5 --um5g-85k --package CABGA381 --speed 8 \
  --json build/axi4-self-test/design.json \
  --lpf examples/axi4-smartconnect/constraints/lfe5um5g-85f-evn.lpf \
  --textcfg build/axi4-self-test/design.config \
  --report build/axi4-self-test/nextpnr-report.json \
  --freq 300 --placer-budgets --placer-heap-timingweight 40 --tmg-ripup
ecppack --svf build/axi4-self-test/design.svf \
  build/axi4-self-test/design.config build/axi4-self-test/design.bit
```

## License

Licensed under either [Apache License 2.0](LICENSE-APACHE) or
[MIT](LICENSE-MIT), at your option.
