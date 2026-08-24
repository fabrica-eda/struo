# QoR benchmark suite

Compares Struo's Veryl-to-ECP5 implementation flow against the standard
open-source baseline (Yosys `synth_ecp5`) on identical committed sources,
then place-and-routes both with identical nextpnr arguments so routed
differences are attributable to synthesis and technology mapping only.

## Methodology

- Each design has a single committed `.veryl` source used by both flows.
  There are no hand-written reference twins, so the two flows cannot drift.
- **struo flow**: `analyze_and_lower` -> `synthesize` -> ECP5 technology
  mapping (`struo qor <file> <top> <out.json> [goal_mhz]`) -> nextpnr JSON.
- **baseline flow**: `veryl build` emits SystemVerilog from the same source;
  Yosys 0.33 (`bench/docker/yosys.Dockerfile`, Ubuntu 24.04 to match the
  locally installed nextpnr package provenance) runs `synth_ecp5`.
- Both mapped netlists run through the same local `nextpnr-ecp5`
  (um5g-85k / CABGA381 / speed 8) with fixed seeds, `--freq 300`,
  `--timing-allow-fail`, timing budgets, heap timing weight 30, and rip-up.
- Metrics per design: COMB sites (`TRELLIS_COMB`, including CCU2C), FFs
  (`TRELLIS_FF`), and worst-clock achieved Fmax from the nextpnr report,
  averaged over the seed list. Timing pass counts report every clock at or
  above its constraint.
- Cross-flow comparison reports geometric means of per-design ratios
  (Struo/baseline); designs without a runnable baseline (interface ports)
  are reported absolutely but excluded from ratio aggregation.

## Usage

```sh
docker build -f bench/docker/yosys.Dockerfile -t struo-qor-yosys:latest .
python3 bench/scripts/qor.py                     # seeds 1,2,3; goal 300 MHz
python3 bench/scripts/qor.py --designs counter32,shift32 --seeds 1,2,3,4,5
python3 bench/scripts/qor.py --summarize-only    # re-render results.json
python3 bench/scripts/qor.py compare             # delta versus the previous run
```

Every completed run appends one JSON line to `build/qor/history.jsonl`
(timestamp, git revision, goal, seeds, per-design metrics). `compare` prints
COMB/FF/Fmax deltas against that history and flags regressions beyond 15%
(area) or 8% (Fmax); its exit code makes it usable as a gate. Each flow record
also embeds the worst critical path broken into route versus logic
picoseconds plus the top delay contributors, so an agent reading
`results.json` gets both the numbers and the reason behind them without
parsing nextpnr reports.

`bench/scripts/render_netlist.py <design.json>` renders a mapped netlist
while preserving module structure: flattened cell names carry the original
hierarchy, so logic elements group under their owning module and every wire
attaches to the deepest common ancestor of its endpoints, with boundary
crossings labeled. `--format json` emits the same tree for agents; note that
mapper-created cells without hierarchical names (`lut1724`, `ccu_arith0_5`)
report at the top level.

The HTML view adds cone attribution: anonymous combinational cells are
assigned to the register module they feed (single-label reachability), so
the Schematic tab draws meaningful blocks — e.g. an AXI4 decoder appears as
one ~280-cell group instead of being scattered across 2,000 anonymous LUTs.
Unattributed cross-module glue lands in "(shared / unattributed)". Add
`--report <nextpnr-report.json>` to embed critical-path timing.

Results land in `build/qor/results.json` plus per-design nextpnr reports.

## Designs

| design        | purpose                                                      |
|---------------|--------------------------------------------------------------|
| counter32     | carry-chain adder shape, matches the carry-benchmark counter |
| shift32       | variable barrel shifter with a clock enable                  |
| maxtree16     | two-stage comparison tree                                    |
| blinky        | existing board smoke test (`examples/ecp5-evn-blinky`)       |
| axi4-crossbar | stress design via `axi4-self-test` + LPF (no Yosys baseline: interface ports exceed Yosys 0.33 SystemVerilog support; revisit with yosys-slang) |

Adding a design: create `bench/designs/<name>/` with `Veryl.toml` and
`<name>.veryl`, then append an entry to `DESIGNS` in `bench/scripts/qor.py`.

## Current snapshot

nextpnr-ecp5 0.6-3build5, Yosys 0.33, seeds 1-3, goal 300 MHz:

| design | flow | COMB sites | FF | Fmax mean (min..max) MHz |
|---|---|---|---|---|
| counter32 | struo | 38 | 32 | 472.14 (472.14..472.14) |
| counter32 | baseline | 38 | 32 | 472.14 (472.14..472.14) |
| shift32 | struo | 157 | 60 | 617.75 (603.14..630.91) |
| shift32 | baseline | 204 | 56 | 604.77 (578.03..657.89) |
| maxtree16 | struo | 110 | 48 | 346.69 (319.49..360.88) |
| maxtree16 | baseline | 110 | 48 | 357.35 (352.73..364.83) |
| blinky | struo | 46 | 25 | 518.13 (518.13..518.13) |
| blinky | baseline | 54 | 24 | 373.34 (360.88..388.50) |
| axi4-crossbar | struo | 1197 | 1217 | 305.38 (304.60..306.09) |

Geometric-mean Struo/baseline ratios: COMB sites 0.900, FF 1.028,
period (1/Fmax) 0.923.

Observations:

- The trivial counter converges to the same canonical CCU2C implementation
  in both flows; it is a sanity anchor, not a differentiator.
- Struo wins area on shift-heavy logic (fanout-weighted cut selection) and
  period on the registered feedback designs; Yosys currently optimizes away
  redundant shifter pipeline bits more aggressively on shift32 (fewer FFs).
- Constant-input flip-flop folding at mapping time (GSR/REGSET
  initialization semantics) removed 4 constant shifter bits on shift32 and
  144 constant registers on the axi4-crossbar.
