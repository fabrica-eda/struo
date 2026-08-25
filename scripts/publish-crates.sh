#!/usr/bin/env bash
set -euo pipefail

mode="${1:-package}"
if [[ "$mode" != "list" && "$mode" != "package" && "$mode" != "publish" ]]; then
  echo "usage: $0 [list|package|publish]" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/.." && pwd)"
cd "$repo_root"

version="$(tr -d '\r\n' < VERSION)"
if [[ ! "$version" =~ ^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$ ]]; then
  echo "VERSION must contain a stable SemVer version, got $version" >&2
  exit 1
fi

# crates.io resolves path dependencies from the registry during packaging, so
# every crate must follow all of its normal and development dependencies.
crates=(
  struo-ir
  struo-rtl
  struo-sim
  struo-formal
  struo-synth
  struo-target-ecp5
  struo-celox
  struo-frontend-veryl
  struo
  struo-cli
)

if [[ "$mode" == "list" ]]; then
  printf '%s\n' "${crates[@]}"
  exit 0
fi

"$script_dir/check-release-version.py"

if [[ "$mode" == "publish" ]]; then
  : "${CARGO_REGISTRY_TOKEN:?CARGO_REGISTRY_TOKEN is required for publication}"
fi

crate_exists() {
  local crate="$1"
  curl --fail --silent --show-error \
    --user-agent "struo-release-workflow/$version (https://github.com/fabrica-eda/struo)" \
    "https://crates.io/api/v1/crates/$crate/$version" \
    >/dev/null 2>&1
}

wait_for_crate() {
  local crate="$1"
  for _ in {1..12}; do
    if crate_exists "$crate"; then
      return 0
    fi
    sleep 5
  done
  echo "$crate@$version was published but did not become visible on crates.io" >&2
  return 1
}

for crate in "${crates[@]}"; do
  if [[ "$mode" == "package" ]]; then
    # Before the first release, crates.io cannot resolve unpublished internal
    # dependencies. File-list validation still catches manifest packaging
    # errors; the ordered publish pass verifies each normalized archive.
    echo "checking package file list for $crate@$version"
    cargo package --locked --allow-dirty --no-verify --list -p "$crate" >/dev/null
    continue
  fi

  if crate_exists "$crate"; then
    echo "$crate@$version is already published; skipping"
    continue
  fi

  echo "building and checking package archive for $crate@$version"
  cargo package --locked -p "$crate"
  package_dir="$repo_root/target/package/$crate-$version"
  if [[ ! -f "$package_dir/Cargo.toml" || -L "$package_dir" ]]; then
    echo "cargo did not create the expected package directory: $package_dir" >&2
    exit 1
  fi
  cargo check \
    --locked \
    --all-targets \
    --manifest-path "$package_dir/Cargo.toml" \
    --target-dir "$repo_root/target/package-checks"
  cargo publish --locked -p "$crate"
  wait_for_crate "$crate"
done
