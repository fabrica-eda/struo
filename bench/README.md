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
```

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
| shift32 | struo | 157 | 64 | 602.99 (575.71..630.12) |
| shift32 | baseline | 204 | 56 | 604.77 (578.03..657.89) |
| maxtree16 | struo | 110 | 48 | 346.69 (319.49..360.88) |
| maxtree16 | baseline | 110 | 48 | 357.35 (352.73..364.83) |
| blinky | struo | 46 | 25 | 518.13 (518.13..518.13) |
| blinky | baseline | 54 | 24 | 373.34 (360.88..388.50) |
| axi4-crossbar | struo | 1205 | 1361 | 307.53 (305.62..311.24) |

Geometric-mean Struo/baseline ratios: COMB sites 0.900, FF 1.045,
period (1/Fmax) 0.929.

Observations:

- The trivial counter converges to the same canonical CCU2C implementation
  in both flows; it is a sanity anchor, not a differentiator.
- Struo wins area on shift-heavy logic (fanout-weighted cut selection) and
  period on the registered feedback designs; Yosys currently optimizes away
  redundant shifter pipeline bits more aggressively on shift32 (fewer FFs).
