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
 :root { --bg:#14161a; --panel:#1d2026; --fg:#d8dbe2; --dim:#8b93a1; --acc:#7ab3ff;
         --route:#e07a72; --logic:#7ad0b3; --ff:#c9a86a; --line:#3a404b; }
 * { box-sizing:border-box; }
 body { margin:0; background:var(--bg); color:var(--fg);
        font:13px/1.45 ui-monospace,Menlo,Consolas,monospace; height:100vh;
        display:flex; flex-direction:column; }
 header { display:flex; align-items:center; gap:14px; padding:6px 12px;
          border-bottom:1px solid var(--line); }
 header .tab { cursor:pointer; padding:4px 10px; border-radius:5px; color:var(--dim); }
 header .tab.active { background:#2b3a55; color:var(--acc); }
 header .stat { margin-left:auto; color:var(--dim); }
 #body { flex:1; display:flex; min-height:0; }
 #tree { width:300px; overflow:auto; border-right:1px solid var(--line); padding:6px; }
 #view { flex:1; overflow:auto; position:relative; }
 #info { width:340px; overflow:auto; border-left:1px solid var(--line); padding:8px;
         white-space:pre-wrap; font-size:12px; }
 details { margin-left:10px; }
 summary { cursor:pointer; padding:1px 3px; border-radius:3px; list-style:none; }
 summary:before { content:"+ "; color:var(--dim); }
 details[open] > summary:before { content:"- "; }
 summary:hover { background:#262b33; }
 .counts { color:var(--dim); }
 .hidden { display:none !important; }
 table.path { border-collapse:collapse; margin:8px 0; width:100%; }
 table.path td, table.path th { border:1px solid var(--line); padding:3px 7px;
        font-size:12px; text-align:left; }
 .bar-row { display:flex; align-items:center; gap:6px; margin:2px 0; }
 .bar-label { width:330px; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
 .bar { height:13px; border-radius:3px; min-width:1px; }
 .bar.route { background:var(--route); }
 .bar.logic { background:var(--logic); }
 .bar.ff { background:var(--ff); }
 svg text { fill:var(--fg); font:11px ui-monospace,monospace; }
 svg .edge { fill:none; stroke-opacity:.6; }
 g.cellbox rect { fill:#22262e; stroke:#454c59; }
 g.cellbox.CCU2C rect { stroke:#b58fd0; }
 g.cellbox.FF rect { stroke:#c9a86a; }
 g.cellbox.selected rect { stroke:var(--acc); stroke-width:2; }
 g.cellbox { cursor:pointer; }
 h3 { margin:8px 0 4px; color:var(--acc); font-size:13px; }
</style>
</head>
<body>
<header>
  <span style="color:var(--acc)">struo viewer</span>
  <span class="tab active" data-tab="schematic">Schematic</span>
  <span class="tab" data-tab="timing">Timing</span>
  <span class="tab" data-tab="modules">Modules</span>
  <span class="stat" id="clockstat"></span>
</header>
<div id="body">
  <div id="tree"></div>
  <div id="view">
    <div id="tab-schematic" style="padding:10px">
      <h3>Schematic — select a module in the tree (leaf modules render cell-by-cell)</h3>
      <div id="sch-info" class="counts"></div>
      <svg id="sch-svg" width="1400" height="900"></svg>
    </div>
    <div id="tab-timing" class="hidden" style="padding:10px">
      <div id="timing-body"></div>
    </div>
    <div id="tab-modules" class="hidden"><svg id="mod-svg" width="1600" height="900"></svg></div>
  </div>
  <div id="info">click a module or cell</div>
</div>
<script>
const DATA = __DATA__;
const WIRES = DATA.connectivity || {};
const NS = "http://www.w3.org/2000/svg";
const $ = id => document.getElementById(id);

function svgEl(tag, attrs, parent) {
  const el = document.createElementNS(NS, tag);
  for (const k in attrs) el.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(el);
  return el;
}
function div(parent, cls, text) {
  if (!parent) return;
  const el = document.createElement("div");
  if (cls) el.className = cls;
  if (text !== undefined) el.textContent = text;
  parent.appendChild(el);
  return el;
}
function countText(node) {
  const c = node.cells || {LE:0,LUT4:0,CCU2C:0,FF:0};
  return `LE ${c.LE} | LUT ${c.LUT4} CCU ${c.CCU2C} FF ${c.FF}`;
}

/* ---------- module tree ---------- */
function buildTree(parentEl, node, path) {
  const hasKids = (node.children || []).length > 0;
  const row = document.createElement(hasKids ? "details" : "div");
  row.style.marginLeft = hasKids ? "0" : "12px";
  const head = document.createElement(hasKids ? "summary" : "div");
  div(head, null, node.module);
  div(head, "counts", ` ${countText(node)} | wires ${node.internal_wires}`);
  head.onclick = () => { setTab("schematic"); showModule(path.join("/"), node); };
  row.appendChild(head);
  const inner = document.createElement("div");
  row.appendChild(inner);
  for (const child of node.children || [])
    buildTree(inner, child, [...path, child.module]);
  parentEl.appendChild(row);
}

/* ---------- schematic ---------- */
let schSelection = null;


function wireInfo(bit) {
  return WIRES[String(bit)] || null;
}

function traceNeighborhood(rootCell) {
  // BFS over connectivity from one cell through its pins.
  const cells = new Map([[rootCell, {depth:0}]]);
  let frontier = [rootCell];
  for (let depth = 1; depth <= 2 && frontier.length; depth++) {
    const next = [];
    for (const name of frontier) {
      for (const bit of Object.keys(WIRES)) {
        const w = WIRES[bit];
        const touchesDriver = w.driver && w.driver.cell === name;
        const consumerHit = (w.consumers || []).some(c => c.cell === name);
        if (!touchesDriver && !consumerHit) continue;
        if (w.driver) {
          if (!cells.has(w.driver.cell)) {
            cells.set(w.driver.cell, {depth});
            next.push(w.driver.cell);
          }
        }
        for (const c of w.consumers || []) {
          if (!cells.has(c.cell)) {
            cells.set(c.cell, {depth});
            next.push(c.cell);
          }
        }
      }
    }
    frontier = next;
    if (cells.size > 120) break;
  }
  return [...cells.keys()];
}

let cellIndexCache = null;
function ensureCellIndex() {
  if (cellIndexCache) return cellIndexCache;
  cellIndexCache = {};
  for (const bit of Object.keys(WIRES)) {
    const w = WIRES[bit];
    const cellsHere = [];
    if (w.driver) cellsHere.push(w.driver.cell);
    for (const c of w.consumers || []) cellsHere.push(c.cell);
    for (const n of cellsHere) {
      const entry = (cellIndexCache[n] ||= {pins: []});
      entry.pins.push(bit);
    }
  }
  return cellIndexCache;
}

function showModule(modulePath) {
  schSelection = null;
  const index = ensureCellIndex();
  const prefix = modulePath ? modulePath + "." : "";
  const own = [];
  for (const name in index) {
    if (prefix
        ? name.startsWith(prefix) && !name.slice(prefix.length).includes(".")
        : !name.includes(".")) {
      own.push(name);
    }
  }
  drawSchematic(own.sort(), modulePath, prefix);
}

function drawSchematic(names, modulePath, prefix) {
  const svg = $("sch-svg");
  svg.innerHTML = "";
  const info = $("sch-info");
  info.textContent = `module '${modulePath || "(top)"}': ${names.length} direct cells`;
  if (!names.length) {
    div(svg.parentNode, "counts", "no directly-named cells here (anonymous mapper logic) — use a cell search below");
    return;
  }
  const COL = 190, ROWH = 64, PERCOL = Math.ceil(Math.sqrt(names.length * 2.2));
  names.forEach((name, i) => {
    const col = Math.floor(i / PERCOL), row = i % PERCOL;
    const x = 16 + col * COL, y = 16 + row * ROWH;
    const kind = /ccu|arith/i.test(name) ? "CCU2C"
               : /^ff_|_ff/.test(name) ? "FF" : "LUT";
    const g = svgEl("g", {class:`cellbox ${kind}`, transform:`translate(${x},${y})`}, svg);
    svgEl("rect", {width: COL - 26, height: ROWH - 18, rx: 7}, g);
    const short = name.length > 24 ? name.slice(0, 23) + "…" : name;
    svgEl("text", {x: 8, y: 17}, g).textContent = short;
    svgEl("text", {x: 8, y: 33, fill:"#8b93a1"}, g).textContent = kind;
    g.addEventListener("click", () => {
      svg.querySelectorAll("g.cellbox").forEach(b => b.classList.remove("selected"));
      g.classList.add("selected");
      inspectCell(name);
    });
    void col; void row;
  });
}

function inspectCell(name) {
  schSelection = name;
  const info = $("info");
  info.textContent = "";
  div(info, null, `cell: ${name}`);
  const seen = new Set();
  for (const bit of Object.keys(WIRES)) {
    const w = WIRES[bit];
    const isDriver = w.driver && w.driver.cell === name;
    const used = (w.consumers || []).some(c => c.cell === name);
    if (!isDriver && !used) continue;
    const label = w.name || `w${bit}`;
    const line = div(info, null, "");
    const dirTxt = isDriver ? "OUT" : "IN ";
    div(line, null, `${dirTxt} ${label}`);
    if (isDriver)
      for (const c of w.consumers || []) div(line, "counts", `   -> ${c.cell} (${c.port})`);
    if (used && w.driver)
      div(line, "counts", `   <- ${w.driver.cell} (${w.driver.port})`);
  }
}

/* ---------- selection ---------- */
function findByPath(node, path) {
  let cur = node;
  for (const part of path) {
    const next = (cur.children || []).find(c => c.module === part);
    if (!next) return null;
    cur = next;
  }
  return cur;
}

function select(path) {
  selectedPath = path;
  selectedNode = findByPath(DATA.tree, path);
  showModule(path.join("/"));
  renderInfo();
}

function renderInfo() {
  const info = $("info");
  info.textContent = "";
  const node = selectedNode || DATA.tree;
  div(info, null, `module: ${selectedPath.join("/") || "(top)"}`);
  div(info, "counts", countText(node) + ` | internal wires ${node.internal_wires}`);
  for (const e of node.edges || []) {
    div(info, "counts", `edge ${e.from} -> ${e.to.join(",")} x${e.count}`);
  }
  div(info, "counts", "--- cells (compressed) ---");
  for (const name of node.cell_names || []) div(info, null, name);
}

function renderGraph() {
  /* the schematic pane is the primary graph; kept as a no-op hook */
}

/* ---------- timing ---------- */
function renderTiming() {
  const body = $("timing-body");
  body.textContent = "";
  const t = DATA.timing || {paths: [], clocks: {}};
  const clocks = Object.entries(t.clocks || {});
  if (clocks.length) {
    div(body, "counts", "achieved Fmax: " +
      clocks.map(([n, v]) => `${n}: ${v} MHz`).join(" | "));
  }
  if (!t.paths.length) {
    div(body, "counts", "no critical paths embedded (pass --report when generating)");
    return;
  }
  t.paths.forEach((p, pi) => {
    const wrap = div(body, null, "");
    div(wrap, "h3", `path ${pi + 1}: ${p.total_ps} ps — from ${p.start}`);
    p.steps.forEach((step, si) => {
      const isRoute = "net" in step;
      const cls = isRoute ? "route" : /FF/i.test(step.cell) && step.port === "Q" ? "ff" : "logic";
      const row = div(wrap, "bar-row", "");
      const label = div(row, "bar-label",
        `${si === 0 ? "" : ""}${step.cell}@${step.port}${isRoute ? ` [net ${step.net}]` : ""}`);
      void label;
      const bar = document.createElement("div");
      bar.className = `bar ${cls}`;
      bar.style.width = Math.max(2, step.delay_ps / 20) + "px";
      bar.title = `${step.delay_ps} ps`;
      row.appendChild(bar);
      div(row, "counts", `${step.delay_ps} ps`);
    });
  });
}

/* ---------- modules tab (aggregate graph) ---------- */
function renderModules() {
  const svg = $("mod-svg");
  svg.innerHTML = "";
  svgEl("text", {x: 16, y: 22}, svg)
    .textContent = "top-level structure — click tree nodes to drill down in Schematic";
  let y = 50;
  const walk = (node, depth) => {
    svgEl("text", {x: 20 + depth * 24, y}, svg)
      .textContent = `${node.module}/  ${countText(node)} | internal wires ${node.internal_wires}`;
    y += 22;
    for (const e of node.edges || []) {
      svgEl("text", {x: 40 + depth * 24, y, fill: "#8b93a1"}, svg)
        .textContent = `↳ ${e.from} → ${e.to.join(",")} ×${e.count}`;
      y += 20;
    }
    for (const c of node.children || []) walk(c, depth + 1);
  };
  walk(DATA.tree, 0);
}

/* ---------- tabs ---------- */
function setTab(name) {
  for (const tab of document.querySelectorAll("header .tab"))
    tab.classList.toggle("active", tab.dataset.tab === name);
  for (const pane of ["schematic", "timing", "modules"])
    $("tab-" + pane).classList.toggle("hidden", pane !== name);
  if (name === "timing") renderTiming();
  if (name === "modules") renderModules();
}
for (const tab of document.querySelectorAll("header .tab")) {
  tab.addEventListener("click", () => setTab(tab.dataset.tab));
}

buildTree($("tree"), DATA.tree, []);
showModule("");
renderModules();

/* clocks banner */
(function () {
  const t = DATA.timing || {};
  const el = $("clockstat");
  const parts = Object.entries(t.clocks || {}).map(([n, v]) => `${n}: ${v} MHz`);
  el.textContent = parts.join(" | ");
})();
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
            # Full connectivity so the schematic can trace wires across the
            # whole design: bit -> driver and consumers by cell/port.
            "connectivity": extract_connectivity(module),
            "timing": embed_timing(args.report),
        }
        html = HTML_TEMPLATE.replace("__DATA__", json.dumps(payload, separators=(",", ":")))
        output = args.output or "netlist.html"
        with open(output, "w") as handle:
            handle.write(html)
        print(f"wrote {output}")
        return 0
    print(render_text(root, cell_limit=args.cell_limit))
    return 0


def extract_connectivity(module: dict) -> dict:
    """Bit-level wiring: every driven bit maps to its driver cell/port and
    its consumer cells/ports, independent of net naming."""

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
                    consumers.setdefault(bit, []).append({"cell": name, "port": port})
    for pname, port in module.get("ports", {}).items():
        for bit in port.get("bits", []):
            if isinstance(bit, int):
                port_bits[bit] = (
                    f"{pname}$in" if port.get("direction") == "input" else f"{pname}$out"
                )
    wires = {}
    for bit in sorted(set(drivers) | set(consumers) | set(port_bits)):
        entry = {}
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


def embed_timing(report_path: str) -> dict:
    if not report_path:
        return {"paths": []}
    report = json.load(open(report_path))
    paths = []

    def total(path: dict) -> float:
        return sum(step.get("delay", 0) for step in path.get("path", []))

    def reg_to_reg(path: dict) -> bool:
        start = path.get("from")
        if not isinstance(start, str) or start.startswith("<"):
            return False  # async / cross-domain boundary marker
        return "$tr_io" not in json.dumps(path.get("path", []))

    scored = sorted(
        [p for p in report.get("critical_paths", []) if reg_to_reg(p)],
        key=total,
        reverse=True,
    )
    for path in scored[:5]:
        steps = [
            {
                "cell": step.get("from", {}).get("cell", "?"),
                "port": step.get("from", {}).get("port", ""),
                "delay_ps": round(1000 * step.get("delay", 0)),
                **({"net": step["net"]} if step.get("net") else {}),
            }
            for step in path.get("path", [])
        ]
        paths.append(
            {
                "total_ps": round(1000 * total(path)),
                "start": str(path.get("from")),
                "steps": steps,
            }
        )
    clocks = {
        name: round(clock["achieved"], 2)
        for name, clock in (report.get("fmax") or {}).items()
    }
    return {"clocks": clocks, "paths": paths}


if __name__ == "__main__":
    sys.exit(main())
