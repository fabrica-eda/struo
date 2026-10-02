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

`$display` and `$write` statements preserve function output/inout effects in
argument expressions, including short-circuit and loop-break guards. Formatting
and literal arguments produce no hardware or console output. This statement
lowering lives in `src/lower/system_tasks.rs`; value-returning system functions
keep their existing expression behavior and other unsupported tasks still fail.
FF function-write restrictions are unchanged.

Function output arguments in an instance input connection remain rejected:
IEEE 1800-2023 13.4 prohibits those calls outside procedural statements. The
suite's packed/unpacked mux-port mismatches also remain explicit ignores rather
than being accepted through an implicit layout conversion (7.6 and 23.3.3.3).
Missing drivers in a supplied library, such as the onehot W=1 base case, are not
filled with invented constants. The manifest distinguishes these fixture issues
from missing lowering support; absence of a suite tag does not prove valid SV.

Dynamic addressing of an instance output is rejected as an invalid implicit
continuous assignment, independently of analyzer warnings (IEEE 1800-2023
Table 10-1). Procedural dynamic part-select assignments remain supported.

Runtime-loop synthesis uses a separate planner in `src/lower/loops.rs`. Existing
constant-range expansion of retained AIR loops is checked by
`src/lower/loops/static_range.rs`: initialization and updates must fit the
counter, and a negative reverse sentinel must not become a large unsigned
comparison operand (IEEE 1800-2023 11.8.1). Guaranteed breaks need no final
update. An exclusive zero upper bound with a larger reverse step is an empty
signed range, even when the analyzer's host enumerator saturates it to zero.
Runtime bounds are accepted for a
non-negative constant start that fits the induction variable and a positive
additive step, when either the bound's type or a guaranteed break proves that
at most 256 candidate iterations are needed. This budget includes the iteration
that executes a break; it is a compile-time resource policy, never a silent
runtime truncation. An input-dependent break alone is not a termination proof,
and a nested loop's break does not terminate its parent.
The proof also accepts a `case` whose default and every arm terminate this loop.
Case-target effects and guarded branch writes are still evaluated by ordinary
lowering. A missing/non-terminating default or any non-terminating arm does not
prove a bound, even if test inputs happen to select a terminating arm.

A runtime start is also accepted when its unsigned leaf type fits the counter
without truncation and the end range fits within 256 non-negative counter values.
The initializer is evaluated exactly once, including output-argument writes and
empty ranges. Lowering tracks the counter value reached by each positive additive
step, suppressing both writes and breaks from skipped candidates. Unit-stride
loops compare each candidate with the captured start directly, avoiding a chain
of counter increments and muxes. Signed starts
without a non-negative proof and possible counter overflow remain unsupported.

Each candidate iteration retains the actual bound comparison and break guard.
The bound is reevaluated against the current combinational environment (or the
pre-edge FF reads), matching for-loop condition evaluation in IEEE 1800-2023
12.7.1 and Veryl's emitted SV. No clock cycles are introduced. Tests cover the
256-iteration boundary, signed bounds, stepped loops, changing bounds, FF writes,
and proof rejection. Runtime-start tests also cover 4,096 start/end/break
combinations, initialization effects, empty ranges, and unsigned packed selects.
Output effects in the end condition remain unsupported and are rejected even
for an empty range.

Veryl 0.22 has an upstream static-unrolling limitation before this validation:
when no `break` retains the loop in AIR, it already expands
`for i in rev 8'd0..4 { q += 1; }` into four assignments. Its emitted SV instead
uses `for (int i = 4 - 1; i >= 8'd0; i--)`, whose unsigned comparison does not
terminate at -1. Likewise, a wide initializer can be truncated by the emitted
`int` counter while host enumeration uses the original wide value. The original
range is absent from this AIR, so the lowering check cannot repair these upstream
expansions. Constant optimization remains enabled; this is an unresolved analyzer
conformance issue, not a supported finite-loop interpretation.

The ignore manifest distinguishes unproven runtime starts, reverse/non-additive loops,
and loops without a proof inside the expansion budget. These are current Struo
synthesis limits, not claims that every such loop is inherently unsynthesizable.
For example, a 32-bit input trip count may require billions of expanded bodies;
a small sampled count in a simulator test does not justify truncating it. New
proofs or algebraic transformations can extend support independently of the
lowering path. The always_ff function-effect restrictions remain unchanged.
