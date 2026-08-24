#!/usr/bin/env python3
"""Renders a mapped ECP5 nextpnr JSON while preserving module structure.

Cells carry flattened hierarchical names such as
``ff_crossbar.s0_aw_decoder.stage1_last_address[15]``, so the original
module tree is recoverable from naming alone. Logic elements group under
their owning module, wires attach to the deepest common ancestor of their
endpoints, and combinational cells are attributed to the register module
their outputs reach (cone attribution) for the HTML viewer.

Usage:
  render_netlist.py <design.json> [--format text|json|html]
                    [--report nextpnr-report.json] [--output out.html]
"""

from __future__ import annotations

import argparse
import collections
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

PRIMITIVE_PREFIXES = (
    "retime_",
    "physical_replicate_",
    "ff_",
)
BIT_SUFFIX = re.compile(r"\[(\d+)\]$")


def cell_hierarchy(name: str) -> list[str]:
    stripped = name
    changed = True
    while changed:
        changed = False
        for prefix in PRIMITIVE_PREFIXES:
            if stripped.startswith(prefix):
                stripped = stripped[len(prefix):]
                changed = True
                break
    parts = [part for part in stripped.split(".") if part]
    return parts or ["<root>"]


def module_dir_of(name: str) -> str:
    stripped = re.sub(r"^(?:retime_|physical_replicate_)+", "", name)
    parts = stripped.split(".")
    parts.pop()
    return "/".join(parts)


def wire_bits(module: dict) -> list[tuple[int, str]]:
    best: dict[int, str] = {}
    for name, net in module.get("netnames", {}).items():
        for bit in net.get("bits") or []:
            if isinstance(bit, int):
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


@dataclass
class Node:
    name: str
    children: dict = field(default_factory=dict)
    cells: list = field(default_factory=list)
    internal_wires: set = field(default_factory=set)
    boundary_wires: dict = field(default_factory=dict)


def build_tree(module: dict, top_name: str) -> tuple[Node, dict]:
    root = Node(top_name)
    cell_paths: dict[str, list[str]] = {}

    for name, cell in module["cells"].items():
        path = cell_hierarchy(name)[:-1]
        cell_paths[name] = path
        node = root
        for part in path:
            node = node.children.setdefault(part, Node(part))
        node.cells.append({"name": name, "type": cell["type"]})

    drivers: dict[int, tuple[str, list[str]]] = {}
    consumers: dict[int, list[tuple[str, list[str]]]] = {}
    input_bits: set[int] = set()
    output_bits: set[int] = set()
    for name, cell in module["cells"].items():
        for port, bits in cell["connections"].items():
            direction = cell["port_directions"].get(port)
            for bit in bits:
                if not isinstance(bit, int):
                    continue
                if direction == "output":
                    drivers.setdefault(bit, (name, cell_paths[name]))
                else:
                    consumers.setdefault(bit, []).append((name, cell_paths[name]))
    for port in module.get("ports", {}).values():
        bits = [b for b in port.get("bits", []) if isinstance(b, int)]
        if port.get("direction") == "input":
            input_bits.update(bits)
        else:
            output_bits.update(bits)

    net_labels: dict[int, str] = {}
    for bit, net_name in wire_bits(module):
        net_labels.setdefault(bit, net_name)

    for bit in sorted(set(drivers) | set(consumers) | input_bits | output_bits):
        driver = drivers.get(bit)
        driver_path = driver[1] if driver else None
        consumer_paths = [path for _, path in consumers.get(bit, [])]
        net_name = net_labels.get(bit) or f"w{bit}"

        endpoints = ([driver_path] if driver_path else []) + consumer_paths
        if not endpoints:
            continue
        common = endpoints[0]
        for other in endpoints[1:]:
            common = lca(common, other)
        node = root
        for part in common:
            node = node.children.setdefault(part, Node(part))

        if all(path == common for path in endpoints):
            node.internal_wires.add(net_name)
            continue

        record = {"net": net_name}
        record["from"] = (
            "<external>" if driver_path is None
            else driver_path[len(common)] if len(driver_path) > len(common)
            else "."
        )
        targets = {
            path[len(common)] if len(path) > len(common) else "."
            for path in consumer_paths
        }
        if bit in output_bits or not targets:
            targets.add("<external>")
        record["to"] = sorted(targets - {"."}) or ["."]
        node.boundary_wires.setdefault(net_name, []).append(record)

    return root, cell_paths


def count_types(cells: list[dict]) -> dict:
    counts = collections.Counter(cell["type"] for cell in cells)
    le = counts.get("TRELLIS_FF", 0) + counts.get("LUT4", 0)
    return {
        "LE": le,
        "LUT4": counts.get("LUT4", 0),
        "CCU2C": counts.get("CCU2C", 0),
        "FF": counts.get("TRELLIS_FF", 0),
    }


def compress_cell_names(names: list[str], limit: int) -> list[str]:
    indexed: dict[str, list[int]] = collections.defaultdict(list)
    plain: list[str] = []
    for name in names:
        match = BIT_SUFFIX.match(name)
        if match:
            indexed[match.group(1)].append(int(match.group(2)))
        else:
            plain.append(name)
    compressed = list(plain)
    for base, indices in sorted(indexed.items()):
        indices.sort()
        compressed.append(f"{base}[{len(indices)} bits: {indices[0]}..{indices[-1]}]")
    return compressed[:limit]


def to_json(node: Node) -> dict:
    return {
        "module": node.name,
        "cells": count_types(node.cells),
        "cell_names": compress_cell_names(
            [cell["name"] for cell in node.cells], 40
        ),
        "internal_wires": len(node.internal_wires),
        "boundary_wires": [
            {"net": net, "from": records[0]["from"], "to": records[0]["to"]}
            for net, records in sorted(node.boundary_wires.items())
        ],
        "children": [to_json(child) for child in node.children.values()],
    }


def attach_edges(node: Node) -> None:
    grouped: dict[tuple[str, tuple[str, ...]], int] = collections.defaultdict(int)
    for records in node.boundary_wires.values():
        record = records[0]
        grouped[(record["from"], tuple(sorted(record["to"])))] += 1
    node.edges = [
        {"from": src, "to": list(targets), "count": count}
        for (src, targets), count in sorted(grouped.items())
    ]
    for child in node.children.values():
        attach_edges(child)


def extract_connectivity(module: dict) -> dict:
    drivers: dict[int, dict] = {}
    consumers: dict[int, list] = {}
    port_bits: dict[int, str] = {}
    for name, cell in module["cells"].items():
        for port, bits in cell["connections"].items():
            direction = cell["port_directions"].get(port)
            for bit in bits:
                if not isinstance(bit, int):
                    continue
                if direction == "output":
                    drivers[bit] = {"cell": name, "port": port}
                else:
                    consumers.setdefault(bit, []).append(
                        {"cell": name, "port": port}
                    )
    for pname, port in module.get("ports", {}).items():
        direction = port.get("direction")
        suffix = "$in" if direction == "input" else "$out"
        for bit in port.get("bits", []):
            if isinstance(bit, int):
                port_bits[bit] = f"{pname}{suffix}"
    wires = {}
    for bit in sorted(set(drivers) | set(consumers) | set(port_bits)):
        entry: dict = {}
        if bit in drivers:
            entry["driver"] = drivers[bit]
        elif bit in port_bits and port_bits[bit].endswith("$in"):
            entry["driver"] = {"cell": port_bits[bit], "port": "PAD"}
        if bit in consumers:
            entry["consumers"] = consumers[bit]
        if bit in port_bits and port_bits[bit].endswith("$out"):
            entry.setdefault("consumers", []).append(
                {"cell": port_bits[bit], "port": "PAD"}
            )
        label = wire_label(module, bit)
        if label:
            entry["name"] = label
        wires[str(bit)] = entry
    return wires


def wire_label(module: dict, bit: int) -> str:
    best = None
    for name, net in module.get("netnames", {}).items():
        bits = [b for b in (net.get("bits") or []) if isinstance(b, int)]
        if bit in bits and (best is None or len(name) < len(best)):
            best = name
    return best or ""


def cone_groups(module: dict) -> dict:
    """Attributes every combinational cell to the register module it feeds.

    Each flip-flop seeds its owning-module label on its Q net; labels
    propagate forward through combinational cells toward the flip-flops that
    consume them, and every combinational cell inherits the union of the
    labels arriving at it. Exactly one distinct label means the cell belongs
    to that module; anything else stays unattributed.
    """
    drivers: dict[int, tuple[str, str]] = {}
    cell_inputs: dict[str, list[int]] = {}
    cell_outputs: dict[str, list[int]] = {}
    for name, cell in module["cells"].items():
        ins: list[int] = []
        outs: list[int] = []
        for port, bits in cell["connections"].items():
            direction = cell["port_directions"].get(port)
            for bit in bits:
                if not isinstance(bit, int):
                    continue
                if direction == "output":
                    outs.append(bit)
                    drivers[bit] = (name, "cell")
                else:
                    ins.append(bit)
        cell_inputs[name] = ins
        cell_outputs[name] = outs

    ff_label_by_output: dict[int, str] = {}
    ff_cells: dict[str, list[str]] = collections.defaultdict(list)
    comb_cells: list[str] = []
    for name in sorted(cell_outputs):
        stripped = re.sub(r"^(?:retime_|physical_replicate_)+", "", name)
        if stripped.startswith("ff_"):
            label = module_dir_of(name)
            ff_cells[label].append(name)
            for bit in cell_outputs[name]:
                ff_label_by_output[bit] = label
        else:
            comb_cells.append(name)

    port_in_bits = {
        bit
        for port in module.get("ports", {}).values()
        if port.get("direction") == "input"
        for bit in port.get("bits", [])
        if isinstance(bit, int)
    }

    bit_memo: dict[int, frozenset | None] = {}

    def labels_of_bit(bit: int) -> frozenset:
        if bit in bit_memo:
            return bit_memo[bit] or frozenset()
        if bit in ff_label_by_output:
            return frozenset({ff_label_by_output[bit]})
        if bit in port_in_bits:
            return frozenset({"<inputs>"})
        driver = drivers.get(bit)
        if driver is None:
            return frozenset()
        bit_memo[bit] = None  # cycle guard placeholder
        result: set = set()
        for in_bit in cell_inputs[driver[0]]:
            result |= labels_of_bit(in_bit)
        frozen = frozenset(result)
        bit_memo[bit] = frozen
        return frozen

    groups: dict[str, list[str]] = {
        label: sorted(cells) for label, cells in ff_cells.items()
    }
    unassigned: list[str] = []
    for name in comb_cells:
        labels: set = set()
        seen_bits: set[int] = set()
        stack = list(cell_inputs[name])
        while stack:
            bit = stack.pop()
            if bit in seen_bits:
                continue
            seen_bits.add(bit)
            if bit in ff_label_by_output:
                labels.add(ff_label_by_output[bit])
                continue
            if bit in port_in_bits:
                continue
            driver = drivers.get(bit)
            if driver:
                dcell = driver[0]
                for in_bit in cell_inputs[dcell]:
                    if in_bit not in seen_bits:
                        stack.append(in_bit)
        if len(labels) == 1:
            groups[next(iter(labels))].append(name)
        else:
            unassigned.append(name)
    return {"groups": dict(sorted(groups.items())), "unassigned": unassigned}


def embed_timing(report_path: str) -> dict:
    if not report_path:
        return {"paths": [], "clocks": {}}
    try:
        report = json.load(open(report_path))
    except (OSError, json.JSONDecodeError):
        return {"paths": [], "clocks": {}}

    def total(path: dict) -> float:
        return sum(step.get("delay", 0) for step in path.get("path", []))

    def reg_to_reg(path: dict) -> bool:
        start = path.get("from")
        if not isinstance(start, str) or start.startswith("<"):
            return False
        return "$tr_io" not in json.dumps(path.get("path", []))

    scored = sorted(
        [p for p in report.get("critical_paths", []) if reg_to_reg(p)],
        key=total,
        reverse=True,
    )[:5]
    paths = []
    for path in scored:
        steps = [
            {
                "cell": step.get("from", {}).get("cell", "?"),
                "port": step.get("from", {}).get("port", ""),
                "delay_ps": round(1000 * step.get("delay", 0)),
                **({"net": step["net"]} if step.get("net") else {}),
            }
            for step in path.get("path", [])
        ]
        paths.append({
            "total_ps": round(1000 * total(path)),
            "start": str(path.get("from")),
            "steps": steps,
        })
    clocks = {
        name: round(clock["achieved"], 2)
        for name, clock in (report.get("fmax") or {}).items()
    }
    return {"clocks": clocks, "paths": paths}


def render_text(node: Node, depth: int = 0, cell_limit: int = 6) -> str:
    pad = "  " * depth
    counts = count_types(node.cells)
    summary = ", ".join(f"{k} {v}" for k, v in counts.items() if v)
    header = f"{pad}{node.name}/  [{summary or 'no cells'}]"
    if node.internal_wires:
        header += f", {len(node.internal_wires)} internal wires"
    lines = [header]
    shown = compress_cell_names([c["name"] for c in node.cells], cell_limit)
    hidden = len(node.cells) - len(shown)
    for name in shown:
        lines.append(f"{pad}  LE {name}")
    if hidden > 0:
        lines.append(f"{pad}  … {hidden} more")
    wire_items = sorted(node.boundary_wires.items())
    for _, records in wire_items[:cell_limit]:
        record = records[0]
        target = ",".join(record["to"][:3])
        if len(record["to"]) > 3:
            target += f",+{len(record['to']) - 3}"
        lines.append(
            f"{pad}  wire {record['net']} from {record['from']} -> {target or '(dangling)'}"
        )
    if len(wire_items) > cell_limit:
        lines.append(f"{pad}  … {len(wire_items) - cell_limit} more wires")
    for child in sorted(node.children.values(), key=lambda n: n.name):
        lines.append(f"{pad}  \\")
        lines.extend(render_text(child, depth + 1, cell_limit).splitlines())
    return "\n".join(line for line in lines if line != "")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("design_json")
    parser.add_argument("--format", choices=["text", "json", "html"], default="text")
    parser.add_argument("--output", default="", help="write HTML here")
    parser.add_argument("--cell-limit", type=int, default=6)
    parser.add_argument("--top", default="", help="top module name (default: the only module)")
    parser.add_argument("--report", default="", help="nextpnr --report JSON; embeds critical paths into the HTML viewer")
    args = parser.parse_args()

    data = json.load(open(args.design_json))
    modules = data["modules"]
    top_name = args.top or next(iter(modules))
    module = modules[top_name]

    root, _ = build_tree(module, top_name)
    attach_edges(root)
    tree = to_json(root)
    if args.format == "json":
        print(json.dumps(tree, indent=1))
        return 0
    if args.format == "html":
        payload = {
            "tree": tree,
            # Bit-level wiring so the schematic traces across the design.
            "connectivity": extract_connectivity(module),
            # Cone attribution for meaningful schematic grouping.
            **cone_groups(module),
            "timing": embed_timing(args.report),
        }
        template = (Path(__file__).parent / "viewer_template.html").read_text()
        html = template.replace("__DATA__", json.dumps(payload, separators=(",", ":")))
        output = args.output or "netlist.html"
        Path(output).write_text(html)
        print(f"wrote {output}")
        return 0
    print(render_text(root, cell_limit=args.cell_limit))
    return 0


if __name__ == "__main__":
    sys.exit(main())
