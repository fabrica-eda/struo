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
    edges: list = field(default_factory=list)


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
        # The final path component is the cell itself; its parent chain is
        # the owning module.
        cell_paths[name] = cell_hierarchy(name)[:-1]
        node = root
        for part in cell_paths[name]:
            node = node.children.setdefault(part, Node(part))
        node.cells.append(
            {
                "name": name,
                "type": cell["type"],
            }
        )

    # Enumerate every wire from cell connections; nextpnr JSON leaves most
    # interior nets anonymous, so netnames serve only as an optional label.
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
        bits = [bit for bit in port.get("bits", []) if isinstance(bit, int)]
        if port.get("direction") == "input":
            input_bits.update(bits)
        else:
            output_bits.update(bits)

    net_labels: dict[int, str] = {}
    for bit, net_name in wire_bits(module):
        net_labels.setdefault(bit, net_name)

    all_wires = set(drivers) | set(consumers) | input_bits | output_bits
    for bit in sorted(all_wires):
        driver = drivers.get(bit)
        driver_path = driver[1] if driver else None
        rides = consumers.get(bit, [])
        consumer_paths = [path for _, path in rides]
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

        # Internal when every endpoint lives in this exact module; otherwise
        # the wire crosses at least one module boundary here.
        if all(path == common for path in endpoints):
            node.internal_wires.add(net_name)
            continue

        record = {"net": net_name}
        if bit in output_bits and driver is None:
            continue
        if driver_path is None:
            record["from"] = "<external>"
        elif len(driver_path) > len(common):
            record["from"] = driver_path[len(common)]
        else:
            record["from"] = "."
        targets = {
            path[len(common)] if len(path) > len(common) else "."
            for path in consumer_paths
        }
        if bit in output_bits or not targets:
            targets.add("<external>")
        record["to"] = sorted(targets - {"."}) or ["."]
        node.boundary_wires.setdefault(net_name, []).append(record)

    def prune_external(node: Node) -> None:
        for name in [
            n
            for n, c in node.children.items()
            if n == "<external>" and not c.cells and not c.boundary_wires
        ]:
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


def attach_edges(node: Node) -> None:
    """Aggregates boundary wires into from/to edge records for rendering."""
    grouped = collections.defaultdict(int)
    for records in node.boundary_wires.values():
        record = records[0]
        grouped[(record["from"], tuple(sorted(record["to"])))] += 1
    node.edges = [
        {"from": src, "to": list(targets), "count": count}
        for (src, targets), count in sorted(grouped.items())
    ]
    for child in node.children.values():
        attach_edges(child)


def to_json(node: Node) -> dict:
    data = {
        "module": node.name,
        "cells": count_types(node.cells),
        "cell_names": compress_cell_names(
            [cell["name"] for cell in node.cells], 40
        ),
        "internal_wires": len(node.internal_wires),
        "edges": [
            {"from": e["from"], "to": e["to"], "count": e["count"]}
            for e in node.edges
        ],
        "boundary_wires": [
            {"net": net, "from": records[0]["from"], "to": records[0]["to"]}
            for net, records in sorted(node.boundary_wires.items())
        ],
        "children": [to_json(child) for child in node.children.values()],
    }
    return data


HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Struo netlist viewer</title>
<style>
 :root { --bg:#14161a; --panel:#1d2026; --fg:#d8dbe2; --dim:#8b93a1; --acc:#7ab3ff; --line:#3a404b; }
 * { box-sizing:border-box; }
 body { margin:0; background:var(--bg); color:var(--fg); font:13px/1.45 ui-monospace,Menlo,Consolas,monospace; display:flex; height:100vh; }
 #tree { width:340px; overflow:auto; border-right:1px solid var(--line); padding:8px; }
 #main { flex:1; display:flex; flex-direction:column; }
 #svgwrap { flex:1; overflow:auto; }
 #info { height:32%; overflow:auto; border-top:1px solid var(--line); padding:8px; white-space:pre-wrap; }
 details { margin-left:12px; }
 summary { cursor:pointer; padding:1px 2px; border-radius:3px; }
 summary:hover { background:#262b33; }
 summary.selected { background:#2b3a55; outline:1px solid var(--acc); }
 .counts { color:var(--dim); }
 .badge { display:inline-block; min-width:34px; text-align:right; margin-right:6px; color:var(--acc); }
 svg text { fill:var(--fg); font:11px ui-monospace,monospace; }
 svg .edge { fill:none; stroke-opacity:.65; }
 h1 { font-size:14px; margin:4px 6px 10px; color:var(--acc); }
</style>
</head>
<body>
<div id="tree"><h1>struo netlist</h1></div>
<div id="main">
  <div id="svgwrap"><svg id="graph" width="1600" height="900"></svg></div>
  <div id="info">click a module</div>
</div>
<script>
const DATA = __DATA__;

const NS = "http://www.w3.org/2000/svg";
function svgEl(tag, attrs, parent) {
  const el = document.createElementNS(NS, tag);
  for (const k in attrs) el.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(el);
  return el;
}
function div(parent, cls, text) {
  const el = document.createElement("div");
  if (cls) el.className = cls;
  if (text !== undefined) el.textContent = text;
  parent.appendChild(el);
  return el;
}
function countText(node) {
  const c = node.cells;
  return `LE ${c.LE} | LUT ${c.LUT4} CCU ${c.CCU2C} FF ${c.FF}`;
}
function findByPath(node, path) {
  let cur = node;
  for (const part of path) {
    cur = (cur.children || []).find(c => c.module === part);
    if (!cur) return null;
  }
  return cur;
}

let selectedPath = [];
let selectedNode = null;

function buildTree(parentEl, node, path) {
  const hasKids = (node.children || []).length > 0;
  const row = document.createElement(hasKids ? "details" : "div");
  row.style.marginLeft = hasKids ? "0" : "12px";
  const summary = document.createElement(hasKids ? "summary" : "div");
  div(summary, null, node.module);
  div(summary, "counts", ` ${countText(node)} | wires ${node.internal_wires}`);
  summary.onclick = () => select(path);
  row.appendChild(summary);
  parentEl.appendChild(row);
  const inner = document.createElement("div");
  row.appendChild(inner);
  for (const child of node.children || []) buildTree(inner, child, [...path, child.module]);
  return row;
}

function select(path) {
  selectedPath = path;
  selectedNode = findByPath(DATA, path);
  document.querySelectorAll("#tree summary").forEach(s => s.classList.remove("selected"));
  event && event.target && event.target.closest && (() => {})();
  renderGraph();
  renderInfo();
}

function highlightTree(node, path) {
  // simple: re-render nothing; visual selection handled by browser default focus
}

function renderInfo() {
  const n = selectedNode || DATA;
  const info = document.getElementById("info");
  info.textContent = "";
  div(info, null, `module: ${(selectedPath.join("/") || DATA.module) || "/"}`);
  div(info, null, countText(n) + ` | internal wires ${n.internal_wires} | direct cells shown ${n.cell_names.length}`);
  for (const e of n.edges || []) {
    div(info, "counts", `edge ${e.from} -> ${e.to.join(",")} x${e.count}`);
  }
  div(info, "counts", "--- cells ---");
  for (const name of n.cell_names) div(info, null, name);
}

function renderGraph() {
  const svg = document.getElementById("graph");
  svg.innerHTML = "";
  const W = 1600, H = 900, CX = W/2, CY = H/2 - 20;

  // focus marker
  svgEl("text", {x: 16, y: 22, "font-size": 13}, svg)
    .textContent = "focus: " + (selectedPath.join("/") || DATA.module);

  const kids = (selectedNode ? (selectedNode.children || []) : []);
  const focusEdges = (selectedNode ? (selectedNode.edges || []) : []);
  const nodes = new Map();

  const zero = {LUT4:0, CCU2C:0, FF:0};
  if (!selectedNode || (selectedNode.cells && selectedNode.cells.LE > 0)) {
    const le = selectedNode ? selectedNode.cells.LE : DATA.cells.LE;
    nodes.set(".", {module: ". (own logic)", isSelf:true, cells: Object.assign({LE: le}, zero)});
  }
  for (const c of kids) nodes.set(c.module, Object.assign({isSelf:false}, c));
  const edgesHere = selectedNode ? (selectedNode.edges || []) : [];
  if (edgesHere.some(e => e.from === "<external>" || e.to.includes("<external>"))) {
    nodes.set("<external>", {module: "<external>", isExternal:true, cells: Object.assign({LE: 0}, zero)});
  }

  const list = [...nodes.entries()];
  const R = Math.max(240, list.length * 46);
  list.forEach(([key, node], i) => {
    const a = -Math.PI/2 + i * 2*Math.PI / list.length;
    node.x = CX + R * Math.cos(a);
    node.y = CY + R * Math.sin(a);
  });

  // edges grouped by (from,to)
  const groups = new Map();
  for (const e of focusEdges) {
    for (const t of e.to) {
      const key = `${e.from}|${t}`;
      groups.set(key, (groups.get(key) || 0) + e.count);
    }
  }
  const posOf = label => {
    if (nodes.has(label)) return nodes.get(label);
    if (label === "." && selectedNode) return {x:CX, y:CY};
    return null;
  };
  for (const [key, count] of groups) {
    const [src, dst] = key.split("|");
    const a = posOf(src) || posOf(dst) && src === ".";
    const b = posOf(dst) || posOf(src) && dst === ".";
    if (!posOf(src) && !posOf(dst)) continue;
    const p1 = posOf(src), p2 = posOf(dst);
    if (!p1 || !p2 || p1 === p2) continue;
    const mx = (p1.x + p2.x)/2, my = (p1.y + p2.y)/2 - 30;
    const path = svgEl("path", {
      class:"edge",
      d:`M ${p1.x} ${p1.y} Q ${mx} ${my} ${p2.x} ${p2.y}`,
      stroke: src === "<external>" ? "#c9746f" : "#7ab3ff",
      "stroke-width": Math.min(6, 1 + Math.log2(count + 1) * 1.3),
    }, svg);
    const title = svgEl("title", {}, path);
    title.textContent = `${src} -> ${dst}: ${count} nets`;
  }

  for (const [key, node] of list) {
    const g = svgEl("g", {}, svg);
    const w = Math.min(190, 90 + node.module.length * 5.2);
    const h = 40;
    svgEl("rect", {
      x: node.x - w/2, y: node.y - h/2, rx: 9,
      width: w, height: h,
      fill: node.isSelf ? "#23303f" : (node.isExternal ? "#3a2528" : "#22262e"),
      stroke: key === "." ? "#5b87c9" : "#454c59",
    }, g);
    svgEl("text", {x: node.x, y: node.y - 2, "text-anchor":"middle"}, g)
      .textContent = node.module.length > 26 ? node.module.slice(0,25)+"…" : node.module;
    svgEl("text", {x: node.x, y: node.y + 13, "text-anchor":"middle", fill:"#8b93a1"}, g)
      .textContent = countText(node).replace("LE ","");
    g.style.cursor = "pointer";
    g.addEventListener("click", () => {
      if (node.isSelf || node.isExternal) return;
      select([...selectedPath, key]);
    });
  }
}

const treeRoot = document.getElementById("tree");
buildTree(treeRoot, DATA, []);
select([]);
</script>
</body>
</html>
"""

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("design_json")
    parser.add_argument("--format", choices=["text", "json", "html"], default="text")
    parser.add_argument("--output", default="", help="write HTML here (default: stdout for text/json)")
    parser.add_argument("--cell-limit", type=int, default=6)
    parser.add_argument("--top", default="", help="top module name (default: the only module)")
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
    elif args.format == "html":
        html = HTML_TEMPLATE.replace("__DATA__", json.dumps(tree, separators=(",", ":")))
        output = args.output or "netlist.html"
        with open(output, "w") as handle:
            handle.write(html)
        print(f"wrote {output}")
    else:
        print(render_text(root, cell_limit=args.cell_limit))
    return 0


if __name__ == "__main__":
    sys.exit(main())
