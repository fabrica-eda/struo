# Veryl corpus tests

The pinned `celox-test-suite-veryl` 0.8.1 corpus exercises:

```text
Veryl -> Struo RTL -> synthesis -> ECP5 mapping -> Celox 0.8.1 native simulation
```

CI enumerates every corpus case in the `Veryl corpus (Celox 0.8.1)` job on
pull requests and pushes to main. `Required CI` depends on this job. Known
unsupported/failing cases are skipped using exact names and reasons in
[veryl-suite-ignores.toml](veryl-suite-ignores.toml). They are reported as
`ignored`, never as passes. New failures outside that list fail the job;
there is no `continue-on-error`.

Each executed case runs in a separate process with a 60-second limit. Ignore
entries must exist in the pinned catalogue and cannot be duplicated. Use
`--include-ignored` to execute excluded cases and expose their actual outcomes.

The Actions job summary shows outcome counts. Download the
`veryl-corpus-results` artifact for per-case diagnostics, including after a
failed run. Reports are generated under `target/` and retained by Actions for
14 days; generated results and historical snapshots are not committed.

## Run locally

```sh
python3 scripts/check-veryl-suite.py
python3 scripts/check-veryl-suite.py --filter context_width:: --jobs 4
python3 scripts/check-veryl-suite.py --include-ignored --timeout 180 --report target/veryl-suite.json
python3 scripts/check-veryl-suite.py --timing --filter wide_shift_mem::test_512bit_shift
cargo test --locked -p struo-frontend-veryl --test veryl_suite
```

`--timing` records seconds spent in lowering, synthesis, mapping, simulation IR
construction, native simulator construction, and the complete case. The native
build stage includes backend optimization, code generation and initialization.
For backend details, also set `CELOX_PASS_TIMING=1 RUST_LOG=debug`.

The ordinary Rust tests cover selected corpus regressions and focused boundary
checks. The ignored worker/catalogue tests are entry points for the full runner;
`cargo test` alone does not run the complete corpus. The two known upstream
constant-folding regressions have explicit Rust `#[ignore]` reasons and can be
rerun by name with `-- --ignored`. Other regular regressions remain enabled.

For diagnostic comparison with Celox's source frontend:

```sh
python3 scripts/check-veryl-suite.py --reference --filter context_width:: \
  --report target/veryl-suite-reference.json
```

Reference runs do not apply Struo's ignore list. Reference results do not determine whether Struo passes. Some corpus cases are
build-only tests; passing them does not establish runtime correctness.

## Current limitations

The complete corpus is not yet passing. The adapter does not implement
four-state storage or internal/hierarchical observation after synthesis.
Unsupported lowering, analyzer errors, shared-reset clock-domain limitations,
and expensive wide designs remain documented exclusions. Veryl's constant folding
is left enabled. A known size-cast constant-expression mismatch therefore also
is explicitly ignored until the upstream behavior is fixed; it is not worked
around by globally disabling AIR constant folding.

The pinned corpus also requires an uninitialized constant-driven FF to start at
zero before its first clock. This is the suite's explicit two-state contract;
synthesis treats the unspecified source initialization as a don't-care.
Celox 0.8.1 separates that check into
`operators::test_ff_constant_two_state_initialization`, which remains explicitly
ignored. The original constant-folding case runs in CI again.

Function inputs support unpacked arrays, nested array literals, repetition and
default filling. Each element is converted to the formal element width and
signedness before binding the automatic frame; dynamic multidimensional reads
and forwarding to nested function calls preserve the array shape. Array-valued
function returns can be assigned or passed to another function, including as
items of nested array literals. Calls with output side effects inside array
expressions remain unsupported.

A no-op tick is permitted only for a source RTL register clock when the mapped
design has no state cells or event handlers. Unknown clocks are not accepted as
eliminated events. All output assertions still execute.

Independent combinational processes may drive disjoint packed ranges of the
same variable (IEEE 1800-2023 9.2.2.2 and 11.5.3). Driver checks and emitted RTL
assignments use the actual written ranges, while blocking reads within each
process still see its earlier writes. Overlapping ranges, including possible
dynamic-index overlap, remain rejected. Synthesis resolves bit dependencies
through whole-vector connections so flattened ports do not create false loops;
actual combinational feedback remains an error. Ordinary acyclic designs keep
the established whole-expression construction order. Bitwise resolution retries
from a fresh state only when that path reports a loop, preserving existing
netlist sharing and placement behavior.

Dynamic packed `+:`, `-:`, and `step` selects support reads and writes.
Offset arithmetic preserves signed indices without wrapping into the vector;
partially overlapping writes affect only valid bits. Out-of-range read bits
follow the mapped adapter's two-state zero convention. Regression tests cover
negative indices, both vector boundaries, and step offsets beyond the vector.

Integral `**` expressions support constant and runtime exponents in combinational
and FF logic. The base is widened to the expression context before repeated
squaring, and the exponent retains its own width and signedness (IEEE 1800-2023
11.4.3 and 11.6.1). Zero exponents produce one, including `0 ** 0`. Negative
exponents produce zero except for bases one and signed minus one; minus one
preserves exponent parity. `0 ** negative` follows the adapter's two-state zero
convention rather than preserving the four-state X result. Effectful exponent
calls and four-state operands remain subject to the existing frontend/adapter
limitations. Boundary regressions cover all four-bit bases and exponents,
constant negative exponents, unsigned parent contexts, and 65-bit results.

The corpus runner copies its worker executable into a temporary directory for
each run, so concurrent Cargo builds cannot replace a worker mid-audit.

Analyzer diagnostics are classified by Veryl's severity: warnings (including
unused return values and unsigned arithmetic shifts) do not reject a design.
Actual errors, including FF function-output restrictions, remain fatal. The
ignore manifest distinguishes those restrictions, invalid signed loop ranges,
and compile-time system-function operand requirements.

Combinational function output effects are supported in arithmetic, concatenation,
short-circuit and conditional expressions, and in if/case conditions. Value-returning
system functions also preserve argument effects, including nested `$signed` /
`$unsigned` wrappers and calls whose return value is discarded in statement position
(IEEE 1800-2023 20.5). Type queries (`$bits` / `$size`) do not evaluate their operands. Effects
are merged with the same condition as the expression value; early returns and
static-loop break guards suppress subsequent writes. Non-local function writes
are explicitly rejected until caller writeback is implemented. Array-valued
expressions, nested argument effects and runtime loops still have limitations.

Dynamic addressing of an instance output is rejected as an invalid implicit
continuous assignment, independently of analyzer warnings (IEEE 1800-2023
Table 10-1). Procedural dynamic part-select assignments remain supported.
