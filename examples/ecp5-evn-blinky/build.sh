#!/usr/bin/env bash
set -euo pipefail

example_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${example_dir}/../.." && pwd)"
build_dir="${example_dir}/build"
top="ecp5_evn_blinky_Top"

for tool in veryl yosys nextpnr-ecp5 ecppack; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "error: required tool not found: ${tool}" >&2
        exit 1
    fi
done

mkdir -p -- "${build_dir}"
cd -- "${example_dir}"

veryl build

yosys -ql "${build_dir}/yosys.log" -p "
    read_verilog -sv ${build_dir}/veryl/Top.sv;
    synth_ecp5 -top ${top} -json ${build_dir}/Top.json;
    write_verilog -noattr ${build_dir}/Top.synth.v
"

# Fail before P&R if the technology-mapped netlist contains unresolved cells.
yosys -ql "${build_dir}/blackbox-check.log" -p "
    read_verilog +/ecp5/cells_sim.v;
    read_verilog ${build_dir}/Top.synth.v;
    hierarchy -check -simcheck -top ${top}
"

nextpnr-ecp5 \
    --um5g-85k \
    --package CABGA381 \
    --speed 8 \
    --json "${build_dir}/Top.json" \
    --lpf "${repo_root}/boards/lfe5um5g-85f-evn/base.lpf" \
    --textcfg "${build_dir}/Top.config" \
    --sdf "${build_dir}/Top.sdf" \
    --report "${build_dir}/nextpnr-report.json" \
    --log "${build_dir}/nextpnr.log" \
    --freq 12

ecppack \
    --db "${TRELLIS_DATABASE_DIR:-/usr/share/trellis/database}" \
    --svf "${build_dir}/Top.svf" \
    "${build_dir}/Top.config" \
    "${build_dir}/Top.bit"

echo "bitstream: ${build_dir}/Top.bit"
