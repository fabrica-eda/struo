# Veryl corpus tests

The pinned `celox-test-suite-veryl` 0.8.0 corpus exercises:

```text
Veryl -> Struo RTL -> synthesis -> ECP5 mapping -> Celox 0.8.0 native simulation
```

CI runs every corpus case in the `Veryl corpus (Celox 0.8.0)` job on pull
requests and pushes to main. `Required CI` depends on this job. Each case runs
in a separate process with a 60-second limit. Only `passed` and expected
compilation `rejected` outcomes pass the job; unsupported cases, assertion
failures, crashes, and timeouts fail it. There is no failure allowlist or
`continue-on-error`.

The Actions job summary shows outcome counts. Download the
`veryl-corpus-results` artifact for per-case diagnostics, including after a
failed run. Reports are generated under `target/` and retained by Actions for
14 days; generated results and historical snapshots are not committed.

## Run locally

```sh
python3 scripts/check-veryl-suite.py
python3 scripts/check-veryl-suite.py --filter context_width:: --jobs 4
python3 scripts/check-veryl-suite.py --timeout 180 --report target/veryl-suite.json
cargo test --locked -p struo-frontend-veryl --test veryl_suite
```

The ordinary Rust tests cover selected corpus regressions and focused boundary
checks. The ignored worker/catalogue tests are entry points for the full runner;
`cargo test` alone does not run the complete corpus.

For diagnostic comparison with Celox's source frontend:

```sh
python3 scripts/check-veryl-suite.py --reference --filter context_width:: \
  --report target/veryl-suite-reference.json
```

Reference results do not determine whether Struo passes. Some corpus cases are
build-only tests; passing them does not establish runtime correctness.

## Current limitations

The complete corpus is not yet passing. The adapter does not implement
four-state storage or internal/hierarchical observation after synthesis.
Unsupported lowering, analyzer errors, shared-reset clock-domain limitations,
and expensive wide designs remain visible failures. Veryl's constant folding
is left enabled. A known size-cast constant-expression mismatch therefore also
remains a failing regression; it is not worked around by globally disabling AIR
constant folding.

The pinned corpus also requires an uninitialized constant-driven FF to start at
zero before its first clock. This is the suite's explicit two-state contract;
synthesis treats the unspecified source initialization as a don't-care.
Upstream https://github.com/celox-sim/celox/pull/917 separates initialization
coverage from constant-folding coverage. The pinned expectation is unchanged
here and still fails CI.

A no-op tick is permitted only for a source RTL register clock when the mapped
design has no state cells or event handlers. Unknown clocks are not accepted as
eliminated events. All output assertions still execute.
