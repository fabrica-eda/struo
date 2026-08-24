#!/usr/bin/env python3
"""Renders a mapped ECP5 nextpnr JSON while preserving module structure.

Cells carry flattened hierarchical names such as
``ff_crossbar.s0_aw_decoder.stage1_last_address[15]``, so the original
module tree is recoverable from naming alone. The renderer groups logic
elements (LUT4, CCU2C slices, and flip-flops) under their owning module,
attributes every wire to the deepest common ancestor of its endpoints,
and marks wires that cross module boundaries.

Usage:
  render_netlist.py <design.json> [--format text|json] [--cell-limit N]
"""

from __future__ import annotations

import argparse
import collections
import json
import re
import sys
from dataclasses import dataclass, field

PRIMITIVE_PREFIXES = (
    "retime_",
    "physical_replicate_",
    "ff_",
)

BIT_SUFFIX = re.compile(r"\[(\d+)\]$")


@dataclass
class Node:
    name: str
    children: dict = field(default_factory=dict)
    cells: list = field(default_factory=list)
    internal_wires: set = field(default_factory=set)
    boundary_wires: dict = field(default_factory=dict)


def cell_hierarchy(name: str) -> list[str]:
    """Splits a flattened cell name into its module path."""
    stripped = name
    changed = True
    while changed:
        changed = False
        for prefix in PRIMITIVE_PREFIXES:
            if stripped.startswith(prefix):
                stripped = stripped[len(prefix) :]
                changed = True
                break
    parts = [part for part in stripped.split(".") if part]
    return parts or ["<root>"]


def wire_bits(module: dict) -> list[tuple[int, str]]:
    """Returns (bit, net-name) for every declared net, flattening aliases."""

    named = []
    for name, net in module.get("netnames", {}).items():
        bits = net.get("bits") or []
        for bit in bits:
            if isinstance(bit, int):
                named.append((bit, name))
    # A bit may appear under several alias names; keep the shortest name.
    best: dict[int, str] = {}
    for bit, name in named:
        current = best.get(bit)
        if current is None or len(name) < len(current):
            best[bit] = name
    return sorted(best.items())


def lca(path_a: list[str], path_b: list[str]) -> list[str]:
    common: list[str] = []
    for a, b in zip(path_a, path_b):
        if a != b:
            break
        common.append(a)
    return common


def build_tree(module: dict, top_name: str) -> tuple[Node, dict]:
    root = Node(top_name)
    cell_paths: dict[str, list[str]] = {}

    for name, cell in module["cells"].items():
        path = cell_hierarchy(name)
        cell_paths[name] = path
        node = root
        for part in path[:-1]:
            node = node.children.setdefault(part, Node(part))
        node.cells.append(
            {
                "name": name,
                "type": cell["type"],
            }
        )

    drivers: dict[int, tuple[str, list[str]]] = {}
    for name, cell in module["cells"].items():
        for port, bits in cell["connections"].items():
            if cell["port_directions"].get(port) != "output":
                continue
            for bit in bits:
                if isinstance(bit, int):
                    drivers.setdefault(bit, (name, cell_paths[name]))

    input_bits = {
        bit
        for port in module.get("ports", {}).values()
        if port.get("direction") == "input"
        for bit in port.get("bits", [])
        if isinstance(bit, int)
    }
    output_bits = {
        bit
        for port in module.get("ports", {}).values()
        if port.get("direction") == "output"
        for bit in port.get("bits", [])
        if isinstance(bit, int)
    }

    for bit, net_name in wire_bits(module):
        if bit in output_bits:
            continue
        driver = drivers.get(bit)
        driver_path = driver[1] if driver else None
        consumer_paths = [
            cell_paths[cname]
            for cname, cell in module["cells"].items()
            for port, bits in cell["connections"].items()
            if cell["port_directions"].get(port) == "input"
            and bit in bits
            and isinstance(bit, int)
        ]
        if not consumer_paths:
            continue
        endpoints = ([driver_path] if driver_path else []) + consumer_paths
        common = endpoints[0]
        for other in endpoints[1:]:
            common = lca(common, other)
        node = root
        for part in common:
            node = node.children.setdefault(part, Node(part))
        fully_internal = (
            driver_path is not None
            and all(path == driver_path for path in consumer_paths)
            and len(driver_path) == len(common)
        )
        if fully_internal:
            node.internal_wires.add(net_name)
            continue
        record = {"net": net_name}
        if driver_path is None:
            record["from"] = "<external>"
        elif len(driver_path) > len(common):
            record["from"] = driver_path[len(common)]
        else:
            record["from"] = "."
        record["to"] = sorted(
            {
                path[len(common)] if len(path) > len(common) else "."
                for path in consumer_paths
            }
        )
        node.boundary_wires.setdefault(net_name, []).append(record)

    def prune_external(node: Node) -> None:
        for name in [n for n, c in node.children.items() if n == "<external>" and not c.cells]:
            del node.children[name]
        for child in node.children.values():
            prune_external(child)

    prune_external(root)
    return root, cell_paths


def compress_cell_names(names: list[str], limit: int) -> list[str]:
    indexed = collections.defaultdict(list)
    plain = []
    pattern = re.compile(r"^(.*)\[(\d+)\]$")
    for name in names:
        match = pattern.match(name)
        if match:
            indexed[match.group(1)].append(int(match.group(2)))
        else:
            plain.append(name)
    compressed = list(plain)
    for base, indices in sorted(indexed.items()):
        indices.sort()
        compressed.append(f"{base}[{len(indices)} bits: {indices[0]}..{indices[-1]}]")
    return compressed[:limit]


def count_types(cells: list[dict]) -> dict:
    counts = collections.Counter(cell["type"] for cell in cells)
    le = counts.get("TRELLIS_FF", 0) + counts.get("LUT4", 0)
    return {
        "LE": le,
        "LUT4": counts.get("LUT4", 0),
        "CCU2C": counts.get("CCU2C", 0),
        "FF": counts.get("TRELLIS_FF", 0),
    }


def render_text(node: Node, depth: int = 0, cell_limit: int = 6) -> str:
    pad = "  " * depth
    lines = []

    def emit_current() -> None:
        counts = count_types(node.cells)
        summary = ", ".join(
            f"{key} {value}" for key, value in counts.items() if value
        )
        header = f"{pad}{node.name}/  [{summary or 'no cells'}]"
        if node.internal_wires:
            header += f", {len(node.internal_wires)} internal wires"
        lines.append(header)
        shown = compress_cell_names(
            [cell["name"] for cell in node.cells], cell_limit
        )
        hidden = len(node.cells) - len(shown)
        for name in shown:
            lines.append(f"{pad}  LE {name}")
        if hidden > 0:
            lines.append(f"{pad}  … {hidden} more")
        wire_items = sorted(node.boundary_wires.items())
        for _, records in wire_items[:cell_limit]:
            record = records[0]
            targets = record["to"]
            shown_targets = ",".join(targets[:3])
            if len(targets) > 3:
                shown_targets += f",+{len(targets) - 3}"
            lines.append(
                f"{pad}  wire {record['net']} from {record['from']} -> {shown_targets or '(dangling)'}"
            )
        if len(wire_items) > cell_limit:
            lines.append(f"{pad}  … {len(wire_items) - cell_limit} more wires")

    emit_current()
    for child in sorted(node.children.values(), key=lambda n: n.name):
        lines.append(f"{pad}  \\")
        lines.extend(render_text(child, depth + 1, cell_limit).splitlines())
    return "\n".join(line for line in lines if line != "")


def to_json(node: Node) -> dict:
    data = {
        "module": node.name,
        "cells": count_types(node.cells),
        "internal_wires": len(node.internal_wires),
        "boundary_wires": [
            {"net": net, "from": records[0]["from"], "to": records[0]["to"]}
            for net, records in sorted(node.boundary_wires.items())
        ],
        "children": [to_json(child) for child in node.children.values()],
    }
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("design_json")
    parser.add_argument("--format", choices=["text", "json"], default="text")
    parser.add_argument("--cell-limit", type=int, default=6)
    parser.add_argument("--top", default="", help="top module name (default: the only module)")
    args = parser.parse_args()

    data = json.load(open(args.design_json))
    modules = data["modules"]
    top_name = args.top or next(iter(modules))
    module = modules[top_name]

    root, _ = build_tree(module, top_name)
    if args.format == "json":
        print(json.dumps(to_json(root), indent=1))
    else:
        print(render_text(root, cell_limit=args.cell_limit))
    return 0


if __name__ == "__main__":
    sys.exit(main())
