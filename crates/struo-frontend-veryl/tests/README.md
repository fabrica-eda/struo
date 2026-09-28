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
cargo test --locked -p struo-frontend-veryl --test veryl_suite
```

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
