#!/usr/bin/env python3
"""Struo QoR suite driver.

Runs each bench design through two ECP5 implementation flows and compares:

- struo:   committed Veryl source -> analyze_and_lower -> synthesize ->
           ECP5 technology mapping -> nextpnr-ecp5
- baseline: same committed Veryl source -> `veryl build` SystemVerilog ->
           Yosys synth_ecp5 (Docker image bench/docker/yosys.Dockerfile) ->
           nextpnr-ecp5

Both flows share identical nextpnr device/package/speed/seed/timing-goal
arguments so routed Fmax differences are attributable to synthesis and
technology mapping only.

Results are written to build/qor/results.json and rendered as a markdown
table with geometric means of per-design struo/baseline ratios.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
BENCH_ROOT = REPO_ROOT / "bench"
BUILD_ROOT = REPO_ROOT / "build" / "qor"
YOSYS_IMAGE = "struo-qor-yosys:latest"
DEVICE_ARGS = [
    "--um5g-85k",
    "--package",
    "CABGA381",
    "--speed",
    "8",
]


@dataclass
class Design:
    name: str
    source_dir: Path
    top: str
    # Veryl source file; defaults to <source_dir>/<top>.veryl.
    source_file: Path | None = None
    # Optional LPF pin constraints handed to nextpnr for both flows.
    lpf: Path | None = None

    def veryl_path(self) -> Path:
        return self.source_file or self.source_dir / f"{self.top}.veryl"
    # CLI subcommand producing the mapped nextpnr JSON; defaults to the
    # generic `qor <file> <top>` path.
    struo_cli: tuple[str, ...] = field(default=())
    # Designs whose baseline cannot be produced (e.g. interface ports) are
    # reported absolutely but excluded from ratio aggregation.
    has_baseline: bool = True


def sh(command: list[str], **kwargs) -> subprocess.CompletedProcess:
    result = subprocess.run(command, cwd=REPO_ROOT, **kwargs)
    if result.returncode != 0:
        raise RuntimeError(f"command failed ({result.returncode}): {' '.join(command)}")
    return result


def run_capture(command: list[str]) -> str:
    return subprocess.run(
        command, cwd=REPO_ROOT, capture_output=True, text=True, check=False
    ).stdout


DESIGNS = [
    Design("counter32", BENCH_ROOT / "designs" / "counter32", "counter32"),
    Design("shift32", BENCH_ROOT / "designs" / "shift32", "shift32"),
    Design("maxtree16", BENCH_ROOT / "designs" / "maxtree16", "maxtree16"),
    Design(
        "blinky",
        REPO_ROOT / "examples" / "ecp5-evn-blinky",
        "Top",
        REPO_ROOT / "examples" / "ecp5-evn-blinky" / "src" / "Top.veryl",
    ),
    Design(
        "axi4-crossbar",
        REPO_ROOT / "examples" / "axi4-smartconnect",
        "Axi4CrossbarSelfTest",
        struo_cli=("axi4-self-test",),
        has_baseline=False,
        lpf=REPO_ROOT
        / "examples"
        / "axi4-smartconnect"
        / "constraints"
        / "lfe5um5g-85f-evn.lpf",
    ),
]


def veryl_project_name(source_dir: Path) -> str:
    text = (source_dir / "Veryl.toml").read_text()
    match = re.search(r'^\s*name\s*=\s*"([^"]+)"', text, re.MULTILINE)
    if not match:
        raise RuntimeError(f"no [project] name in {source_dir}/Veryl.toml")
    return match.group(1)


def run_struo(design: Design, out_json: Path, goal_mhz: int) -> None:
    if design.struo_cli:
        command = ["cargo", "run", "--release", "--quiet", "--", *design.struo_cli]
        command.append(str(out_json))
    else:
        command = [
            "cargo",
            "run",
            "--release",
            "--quiet",
            "--",
            "qor",
            str(design.veryl_path()),
            design.top,
            str(out_json),
            str(goal_mhz),
        ]
    sh(command)


def run_baseline_yosys(design: Design, workdir: Path, out_json: Path) -> str:
    """Returns the synthesized top module name."""
    scratch = workdir / "veryl-project"
    if scratch.exists():
        shutil.rmtree(scratch)
    shutil.copytree(design.source_dir, scratch)
    subprocess.run(["veryl", "build", "--quiet"], cwd=scratch, check=True)
    sv_path = scratch / "build" / "veryl" / f"{design.top}.sv"
    if not sv_path.exists():
        raise RuntimeError(f"veryl build did not emit {sv_path}")
    project = veryl_project_name(design.source_dir)
    top_module = f"{project}_{design.top}"
    if shutil.which("yosys") is not None:
        sh(
            [
                "yosys",
                "-p",
                f"read_verilog -sv {sv_path}; "
                f"hierarchy -check -top {top_module}; "
                f"synth_ecp5 -top {top_module} -json {out_json}",
            ],
            stdout=subprocess.DEVNULL,
        )
        return top_module

    mounted = "/work"
    container_sv = f"{mounted}/{sv_path.relative_to(REPO_ROOT)}"
    container_out = f"{mounted}/{out_json.relative_to(REPO_ROOT)}"
    yosys_script = (
        f"read_verilog -sv {container_sv}; "
        f"hierarchy -check -top {top_module}; "
        f"synth_ecp5 -top {top_module} -json {container_out}"
    )
    sh(
        [
            "docker",
            "run",
            "--rm",
            "-v",
            f"{REPO_ROOT}:{mounted}",
            "-w",
            mounted,
            YOSYS_IMAGE,
            "-p",
            yosys_script,
        ]
    )
    return top_module


def run_nextpnr(
    mapped_json: Path,
    report: Path,
    config: Path,
    seed: int,
    goal_mhz: int,
    lpf: Path | None = None,
) -> dict:
    command = [
        "nextpnr-ecp5",
        *DEVICE_ARGS,
        "--json",
        str(mapped_json),
    ]
    if lpf is not None:
        command += ["--lpf", str(lpf)]
    command += [
        "--lpf-allow-unconstrained",
        "--textcfg",
        str(config),
        "--report",
        str(report),
        "--freq",
        str(goal_mhz),
        "--timing-allow-fail",
        "--seed",
        str(seed),
        "--placer-budgets",
        "--placer-heap-timingweight",
        "30",
        "--tmg-ripup",
    ]
    sh(command, stdout=subprocess.DEVNULL)
    data = json.loads(report.read_text())
    fmax_clocks = [clock["achieved"] for clock in data["fmax"].values()]
    util = data["utilization"]
    comb = util["TRELLIS_COMB"]["used"]
    carry = util.get("CCU2C", {"used": 0})["used"]
    ff = util["TRELLIS_FF"]["used"]
    passed = all(clock["achieved"] >= clock["constraint"] for clock in data["fmax"].values())
    return {
        "seed": seed,
        "lut": comb,
        "ccu2c": carry,
        "ff": ff,
        "fmax_mhz": min(fmax_clocks),
        "timing_pass": passed,
        "critical_path": summarize_critical_path(data),
    }


def summarize_critical_path(report_data: dict) -> dict | None:
    """Aggregates the slowest register-to-register path into route versus
    logic time plus the largest individual contributors."""

    def path_total(path: dict) -> float:
        return sum(step.get("delay", 0) for step in path.get("path", []))

    raw_paths = report_data.get("critical_paths")
    if isinstance(raw_paths, dict):
        raw_paths = list(raw_paths.values())
    paths = [p for p in (raw_paths or []) if isinstance(p, dict)]
    def is_register_to_register(path: dict) -> bool:
        steps = path.get("path", [])
        if not steps:
            return False
        endpoints = [steps[0].get("from", {}), steps[-1].get("to", {})]
        return all(
            isinstance(endpoint, dict)
            and not str(endpoint.get("cell", "")).endswith("$tr_io")
            for endpoint in endpoints
        )

    interior = [
        p
        for p in paths
        if isinstance(p.get("from"), dict)
        and p["from"].get("cell", "").startswith(("ff_", "retime_"))
    ]
    candidates = [p for p in interior if is_register_to_register(p)] or interior or paths
    if not candidates:
        return None
    worst = max(candidates, key=path_total)
    steps = worst.get("path", [])
    route_ps = round(1000 * sum(step.get("delay", 0) for step in steps if "net" in step))
    logic_ps = round(1000 * sum(step.get("delay", 0) for step in steps if "net" not in step))
    contributors = sorted(
        (
            {
                "cell": step.get("from", {}).get("cell", "?"),
                "port": step.get("from", {}).get("port", ""),
                "ps": round(1000 * step.get("delay", 0)),
            }
            for step in steps
            if step.get("delay")
        ),
        key=lambda item: item["ps"],
        reverse=True,
    )
    return {
        "total_ps": round(1000 * path_total(worst)),
        "route_ps": route_ps,
        "logic_ps": logic_ps,
        "top_contributors": contributors[:6],
    }


def mean(values) -> float:
    values = list(values)
    return sum(values) / len(values)


def geomean(values) -> float:
    return math.exp(mean(math.log(v) for v in values))


def collect_flow_metrics(runs: list[dict]) -> dict:
    worst_path = min(
        (r.get("critical_path") for r in runs),
        key=lambda cp: (cp is not None, -(cp["total_ps"] if cp else 0)),
        default=None,
    )
    metrics = {
        "lut": runs[0]["lut"],
        "ccu2c": runs[0]["ccu2c"],
        "ff": runs[0]["ff"],
        "fmax_mean_mhz": round(mean([r["fmax_mhz"] for r in runs]), 2),
        "fmax_min_mhz": min(r["fmax_mhz"] for r in runs),
        "fmax_max_mhz": max(r["fmax_mhz"] for r in runs),
        "seeds_passed": sum(1 for r in runs if r["timing_pass"]),
        "seeds_total": len(runs),
    }
    if worst_path is not None:
        metrics["critical_path"] = worst_path
    return metrics


def render_summary(results: dict) -> str:
    lines = []
    lines.append("| design | flow | COMB sites | FF | Fmax mean (min..max) MHz | seeds pass |")
    lines.append("|---|---|---|---|---|---|---|")
    ratios = []
    for entry in results["designs"]:
        for flow in ("struo", "baseline"):
            metrics = entry.get(flow)
            if not metrics:
                continue
            lines.append(
                f"| {entry['name']} | {flow} "
                f"| {metrics['lut']} | {metrics['ff']} "
                f"| {metrics['fmax_mean_mhz']} ({metrics['fmax_min_mhz']}..{metrics['fmax_max_mhz']}) "
                f"| {metrics['seeds_passed']}/{metrics['seeds_total']} |"
            )
        if entry.get("struo") and entry.get("baseline"):
            s, b = entry["struo"], entry["baseline"]
            lut_s = s["lut"]
            lut_b = b["lut"]
            ratios.append(
                {
                    "name": entry["name"],
                    "lut": lut_s / lut_b,
                    "ff": s["ff"] / b["ff"],
                    "fmax": b["fmax_mean_mhz"] / s["fmax_mean_mhz"],
                }
            )
    lines.append("")
    if ratios:
        lines.append("| design | COMB sites struo/baseline | FF struo/baseline | 1/Fmax struo/baseline |")
        lines.append("|---|---|---|---|")
        for r in ratios:
            lines.append(
                f"| {r['name']} | {r['lut']:.3f} | {r['ff']:.3f} | {r['fmax']:.3f} |"
            )
        lines.append(
            f"| **geomean** | **{geomean([r['lut'] for r in ratios]):.3f}** "
            f"| **{geomean([r['ff'] for r in ratios]):.3f}** "
            f"| **{geomean([r['fmax'] for r in ratios]):.3f}** |"
        )
    lines.append("")
    lines.append(
        "> Ratios below 1.0 favor struo. The 1/Fmax ratio compares achievable "
        "period (struo period / baseline period); below 1.0 favors struo."
    )
    return "\n".join(lines)


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "compare":
        rest = sys.argv[2:]
        parser = argparse.ArgumentParser(prog="qor.py compare")
        parser.add_argument("--against", type=int, default=1, help="how many runs back to compare against")
        parser.add_argument("--comb-percent", type=float, default=DEFAULT_COMB_REGRESSION_PERCENT)
        parser.add_argument("--fmax-percent", type=float, default=DEFAULT_FMAX_REGRESSION_PERCENT)
        parser.add_argument("--json", action="store_true", help="emit machine-readable run records")
        compare_args = parser.parse_args(rest)
        return run_compare(
            compare_args.against,
            compare_args.comb_percent,
            compare_args.fmax_percent,
            compare_args.json,
        )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", default="1,2,3", help="comma-separated nextpnr seeds")
    parser.add_argument("--goal-mhz", type=int, default=300, help="nextpnr --freq value")
    parser.add_argument("--designs", default="", help="comma-separated subset of designs")
    parser.add_argument("--skip-struo", action="store_true", help="reuse existing struo JSON")
    parser.add_argument("--summarize-only", action="store_true", help="render existing results")
    args = parser.parse_args()

    results_path = BUILD_ROOT / "results.json"
    if args.summarize_only:
        print(render_summary(json.loads(results_path.read_text())))
        return 0

    seeds = [int(s) for s in args.seeds.split(",")]
    selected = DESIGNS
    if args.designs:
        wanted = set(args.designs.split(","))
        selected = [d for d in DESIGNS if d.name in wanted]

    versions = {
        "nextpnr": run_capture(["nextpnr-ecp5", "--version"]).strip(),
        "yosys": run_capture(
            ["yosys", "-V"] if shutil.which("yosys") is not None else ["docker", "run", "--rm", YOSYS_IMAGE, "-V"]
        ).strip(),
    }

    results = {
        "goal_mhz": args.goal_mhz,
        "seeds": seeds,
        "versions": versions,
        "designs": [],
    }

    for design in selected:
        print(f"=== {design.name} ===", flush=True)
        workdir = BUILD_ROOT / design.name
        workdir.mkdir(parents=True, exist_ok=True)
        entry: dict = {"name": design.name}

        struo_json = workdir / "struo" / "design.json"
        struo_runs = []
        if not (args.skip_struo and struo_json.exists()):
            struo_json.parent.mkdir(parents=True, exist_ok=True)
            run_struo(design, struo_json, args.goal_mhz)
        for seed in seeds:
            report = workdir / "struo" / f"report-seed{seed}.json"
            struo_runs.append(
                run_nextpnr(
                struo_json,
                report,
                workdir / "struo" / f"design-seed{seed}.config",
                seed,
                args.goal_mhz,
                design.lpf,
            )
            )
        entry["struo"] = collect_flow_metrics(struo_runs)

        if design.has_baseline:
            baseline_json = workdir / "baseline" / "design.json"
            baseline_json.parent.mkdir(parents=True, exist_ok=True)
            top_module = run_baseline_yosys(design, workdir, baseline_json)
            print(f"baseline top: {top_module}", flush=True)
            baseline_runs = []
            for seed in seeds:
                report = workdir / "baseline" / f"report-seed{seed}.json"
                baseline_runs.append(
                    run_nextpnr(
                        baseline_json,
                        report,
                        workdir / "baseline" / f"design-seed{seed}.config",
                        seed,
                        args.goal_mhz,
                        design.lpf,
                    )
                )
            entry["baseline"] = collect_flow_metrics(baseline_runs)
        else:
            entry["baseline"] = None

        results["designs"].append(entry)
        results_path.parent.mkdir(parents=True, exist_ok=True)
        results_path.write_text(json.dumps(results, indent=1))

    print()
    print(render_summary(results))
    append_history(results)
    return 0


def git_revision() -> str | None:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.strip() or None if result.returncode == 0 else None


def append_history(results: dict) -> Path:
    record = {
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "revision": git_revision(),
        "goal_mhz": results["goal_mhz"],
        "seeds": results["seeds"],
        "designs": [
            {"name": d["name"], "struo": d.get("struo"), "baseline": d.get("baseline")}
            for d in results["designs"]
        ],
    }
    history = BUILD_ROOT / "history.jsonl"
    history.parent.mkdir(parents=True, exist_ok=True)
    with history.open("a") as handle:
        handle.write(json.dumps(record) + "\n")
    return history


FLOWS = ("struo", "baseline")
DEFAULT_COMB_REGRESSION_PERCENT = 15.0
DEFAULT_FMAX_REGRESSION_PERCENT = 8.0


def compare_runs(current: dict, previous: dict, comb_percent: float, fmax_percent: float) -> tuple[str, list[str]]:
    lines = ["| design | flow | COMB delta | FF delta | Fmax min delta | flags |"]
    lines.append("|---|---|---|---|---|---|")
    flags: list[str] = []
    previous_by_name = {d["name"]: d for d in previous["designs"]}
    for entry in current["designs"]:
        prior = previous_by_name.get(entry["name"])
        if not prior:
            continue
        for flow in FLOWS:
            now_metrics = entry.get(flow)
            was_metrics = prior.get(flow)
            if not (now_metrics and was_metrics):
                continue
            comb_delta = percent_delta(now_metrics["lut"], was_metrics["lut"])
            ff_delta = percent_delta(now_metrics["ff"], was_metrics["ff"])
            fmax_delta = -percent_delta(now_metrics["fmax_min_mhz"], was_metrics["fmax_min_mhz"])
            row_flags = []
            if comb_delta > comb_percent:
                row_flags.append(f"COMB +{comb_delta:.1f}%")
            if ff_delta > comb_percent:
                row_flags.append(f"FF +{ff_delta:.1f}%")
            if fmax_delta > fmax_percent:
                row_flags.append(f"Fmax -{fmax_delta:.1f}%")
            marker = ", ".join(row_flags) or "-"
            if row_flags:
                flags.append(f"{entry['name']}/{flow}: {marker}")
            lines.append(
                f"| {entry['name']} | {flow} "
                f"| {comb_delta:+.1f}% | {ff_delta:+.1f}% | {-fmax_delta:+.1f}% | {marker} |"
            )
    lines.append("")
    lines.append(f"flags ({len(flags)}):" if flags else "flags: none")
    for flag in flags:
        lines.append(f"- {flag}")
    return "\n".join(lines), flags


def percent_delta(now: float, then: float) -> float:
    if then == 0:
        return 0.0
    return (now - then) / abs(then) * 100.0


def load_history_entry(offset: int) -> dict | None:
    history = BUILD_ROOT / "history.jsonl"
    if not history.exists():
        return None
    lines = [line for line in history.read_text().splitlines() if line.strip()]
    if len(lines) < offset + 1:
        return None
    return json.loads(lines[-1 - offset])


def run_compare(against: int, comb_percent: float, fmax_percent: float, as_json: bool) -> int:
    current = load_history_entry(0)
    previous = load_history_entry(against)
    if current is None or previous is None:
        print("compare needs at least two recorded runs in build/qor/history.jsonl")
        return 2
    text, _ = compare_runs(current, previous, comb_percent, fmax_percent)
    if as_json:
        print(json.dumps({"current": current, "previous": previous}))
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
