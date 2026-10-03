# Veryl corpus tests

The pinned `celox-test-suite-veryl` 0.9.0 corpus exercises:

```text
Veryl -> Struo RTL -> synthesis -> ECP5 mapping -> Celox 0.9.0 native simulation
```

CI enumerates every corpus case in the `Veryl corpus (Celox 0.9.0)` job on
pull requests and pushes to main. `Required CI` depends on this job. Known
unsupported/failing cases are skipped using exact names and reasons in
[veryl-suite-ignores.toml](veryl-suite-ignores.toml). They are reported as
`ignored`, never as passes. New failures outside that list fail the job;
there is no `continue-on-error`.

Each executed case runs in a separate process with a default 60-second limit.
Verified slow cases have exact `[[timeout]]` entries with a reason and a bounded
`seconds` budget in `veryl-suite-ignores.toml`; they execute in CI and must pass.
`--timeout N` overrides all case budgets, including these exceptions. Executed
cases record their effective `timeout_seconds` and `timeout_reason` in the report. Ignore
entries must exist in the pinned catalogue and cannot be duplicated. Use
`--include-ignored` to execute excluded cases and expose their actual outcomes.

The runner defaults to one worker and a hard 4096 MiB address-space limit per
worker (`--memory-mib`). The limit is applied before executing the native worker,
inherited by its child processes, and cannot be raised by the worker. Allocation
failure is a failed case, never a pass. Core dumps are disabled for workers.
This limits virtual address space, not just resident memory; a large reservation
can fail even without equivalent physical allocation. Multiple workers or runner
invocations multiply the possible memory usage. CI uses one worker explicitly.
The limit does not apply to the initial Cargo build.

The Actions job summary shows outcome counts. Download the
`veryl-corpus-results` artifact for per-case diagnostics, including after a
failed run. Reports are generated under `target/` and retained by Actions for
14 days; generated results and historical snapshots are not committed.

## Remaining resource limitations

Timeout exceptions enable verified slow cases in CI without weakening their
assertions. The remaining performance ignores have not demonstrated a complete
passing run within the diagnostic budgets:

| Case family | Evidence and current limitation |
| --- | --- |
| Large sparse FF line-write array | A 600-second run completed lowering in about 201 seconds, then timed out during synthesis. |
| Wide dynamic FF checkpoint | A 600-second run completed mapping and simulation IR construction, then timed out during native simulator construction. |
| Constant-driven typed reverse bound | A 240-second run completed mapping in about 51 seconds, then timed out during native simulator construction. |
| Packed scatter store | The 60-second audit timed out during synthesis. |
| Signed 128-bit and wide division/remainder | The 60-second audit timed out. Longer diagnostics were explicitly stopped after excessive local memory use was reported; they provide no completed correctness result. |

These limits do not establish a semantic mismatch or an unsynthesizable source.
They remain separate from tagged expectation exclusions, frontend restrictions,
and loops with demonstrated nonterminating inputs. Further diagnostics must use
the bounded launcher; raising timeouts alone does not address memory growth.

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
Celox 0.9.0 separates that check into
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
at most 512 candidate iterations are needed. This budget includes the iteration
that executes a break; it is a compile-time resource policy, never a silent
runtime truncation. An input-dependent break alone is not a termination proof,
and a nested loop's break does not terminate its parent.
The proof also accepts a `case` whose default and every arm terminate this loop.
Case-target effects and guarded branch writes are still evaluated by ordinary
lowering. A missing/non-terminating default or any non-terminating arm does not
prove a bound, even if test inputs happen to select a terminating arm.

A runtime start is also accepted when its unsigned leaf type fits the counter
without truncation and the end range fits within 512 non-negative counter values.
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

When range expansion cannot prove a bound, `src/lower/single_iteration.rs` also
accepts a body that exits through `break` on every path. It evaluates the actual
initializer, assigns it into the counter's declared width, and then compares it
with the condition bound. This supports wide or signed initializers and reverse
ranges, including wrapped exclusive reverse initializers, without executing any
step. A scoped runtime binding supplies the counter to expressions and nested
loops; constant and parameter reads elsewhere keep their usual treatment.
Initializer effects happen once even when the first condition is false. Body
writes are guarded by that condition, and existing FF function-write restrictions
remain in force. Tests cover 162 signed-bound/gate combinations and 48 FF cases
with mixed signed/unsigned comparison contexts, dynamic packed selects, and
constant-array reads. Veryl saturates constant range values at host integer
limits; a constant initializer at that boundary is rejected because AIR cannot
distinguish the exact boundary from a larger original value with different low
bits. Runtime expressions retain their full initializer bits.

`src/lower/bitwise_loops.rs` also proves OR/XOR counter traces when the initializer
lowers to an effect-free constant in the current procedural environment. Each
update is masked to the counter width, preserving signed interpretation when
substituting the next value. A guaranteed break must be reached before a repeated
counter value or the expansion budget; every actual bound comparison remains in
the circuit. Initializer writes and unknown initializers are rejected by this
proof. FF-local blocking initialization is visible, while a scheduled module FF
write cannot supply the current initializer.

`src/lower/additive_loops.rs` proves short forward additive traces with known
initializers and immutable constant bounds, including negative initial values
and bounds computed by enclosing loop substitution. Comparisons use the actual
operand widths and common signedness; updates wrap at the counter width. The
proof rejects cycles and traces exceeding 512 iterations. It does not treat a
mutable bound's initial constant value as an invariant. The shared read-only
RTL evaluator in `constant_values.rs` recognizes scalar arithmetic and bit
operations without trusting cached AIR numeric values or rewriting the circuit.
Regression tests include nested negative bounds, initializer capture, mixed
signed/unsigned comparisons, finite wraparound, and rejection of signed cycles.

The bound proof also recognizes non-negative additions without overflow in the
actual expression context. Thus an 8-bit input plus `8'd1` has maximum 256 in a
32-bit loop comparison. The 512-candidate policy includes its inclusive endpoint.
`loops/reverse.rs` handles descending runtime loops with constant signed lower
bounds and proven non-negative initial bounds. It captures the initializer once,
subtracts one before counter conversion for exclusive ranges, and visits only
reachable descending candidates. Unsigned lower comparisons and unproven
initial truncation remain rejected. Regression tests cover all 256 narrow
bound/break combinations, non-unit steps, initializer effects even for empty
ranges, blocking FF-local writes, and the 512/513-candidate boundary.

`constant_driven_loops.rs` can also prove a signed reverse lower bound from an
already-lowered whole-signal constant combinational driver, including negative
bounds. It retains typed counter values and rejects final-update overflow.
The existing process-ownership validation rejects any later overlapping writer,
including a loop body that attempts to modify this bound. Register outputs,
partial drivers, variable drivers, and mutable local constants cannot provide
this proof. This is a fallback using emitted RTL; a producer not yet lowered is
not available to it. Tests exercise signed negative iterations, empty and
non-empty prefixes, non-unit steps, FF reads, and conflicting drivers.

The remaining nonterminating-loop corpus ignores have concrete non-terminating input
values. They are not excluded merely because their loops are dynamic:

| Fixture family | Counterexample to termination |
| --- | --- |
| OR/XOR loops starting at 3 with endpoint-dependent breaks | Endpoint 8 makes OR stall at 7 and XOR cycle between 3 and 5. |
| Bitwise step operands with bits above the i32 counter | Start 0 and endpoints 8 make OR stall at 6 and XOR cycle between 0 and 6, missing breaks at 7/5. |
| FF signed XOR with external initial value | Start 0 and a large positive endpoint cycle between 0 and i32 minimum, never reaching the break at 2147483640. |
| Signed inclusive dynamic endpoints | An endpoint equal to i32 maximum keeps the condition true even after the counter wraps. |
| Combined runtime-bounds fixtures (comb and FF) | `step_start = 0` and `count = 4` make their multiply-by-two loop stay at zero forever. |
| Multiplicative stalled step | Start 0, count 4, and `sel = 0` repeat 0 without breaking. |
| Unsigned reverse singleton fixtures | Start/count 0 let the counter wrap under an unsigned comparison with zero. |
| Signed wide reverse fixtures without guaranteed breaks | An i64-minimum lower bound is below every i32 counter value, including values after wrap. |

Finite test vectors do not constrain these full input domains for synthesis.

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
