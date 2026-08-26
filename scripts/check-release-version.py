#!/usr/bin/env python3
"""Validate Struo's lockstep crates.io release version."""

from __future__ import annotations

import json
import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
PUBLIC_CRATES = {
    "struo",
    "struo-celox",
    "struo-cli",
    "struo-formal",
    "struo-frontend-veryl",
    "struo-ir",
    "struo-rtl",
    "struo-sim",
    "struo-synth",
    "struo-target-ecp5",
}
SEMVER = re.compile(r"^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$")


def fail(message: str) -> None:
    raise SystemExit(message)


version = (ROOT / "VERSION").read_text(encoding="utf-8").strip()
if not SEMVER.fullmatch(version):
    fail(f"VERSION must contain a stable SemVer version, got {version!r}")

root_manifest = (ROOT / "Cargo.toml").read_text(encoding="utf-8")
version_lines = [
    line for line in root_manifest.splitlines() if "x-release-please-version" in line
]
if not version_lines:
    fail("Cargo.toml has no release version markers")
for line in version_lines:
    match = re.search(r'\bversion\s*=\s*"=?([^"]+)"', line)
    if match is None or match.group(1) != version:
        fail(f"Cargo.toml release version is not {version}: {line}")

metadata = json.loads(
    subprocess.check_output(
        ["cargo", "metadata", "--locked", "--no-deps", "--format-version", "1"],
        cwd=ROOT,
        text=True,
    )
)
packages = {package["name"]: package for package in metadata["packages"]}
published = {
    name for name, package in packages.items() if package.get("publish") == ["crates-io"]
}
if published != PUBLIC_CRATES:
    fail(
        "public crate set differs from the release policy: "
        f"missing={sorted(PUBLIC_CRATES - published)}, "
        f"unexpected={sorted(published - PUBLIC_CRATES)}"
    )

for name in sorted(PUBLIC_CRATES):
    package = packages[name]
    if package["version"] != version:
        fail(f"{name} has version {package['version']}; expected {version}")
    for dependency in package["dependencies"]:
        dependency_name = dependency["name"]
        if dependency_name in PUBLIC_CRATES and dependency["req"] != f"={version}":
            fail(
                f"{name} requires {dependency_name} {dependency['req']}; "
                f"expected ={version}"
            )

lockfile = (ROOT / "Cargo.lock").read_text(encoding="utf-8")
for block in lockfile.split("[[package]]")[1:]:
    name_match = re.search(r'^\s*name = "([^"]+)"', block, re.MULTILINE)
    version_match = re.search(r'^\s*version = "([^"]+)"', block, re.MULTILINE)
    if name_match and name_match.group(1) in PUBLIC_CRATES:
        if version_match is None or version_match.group(1) != version:
            candidate = version_match.group(1) if version_match else "missing"
            fail(f"Cargo.lock has {name_match.group(1)} {candidate}; expected {version}")

print(f"Validated stable Struo version {version} across {len(PUBLIC_CRATES)} crates")
