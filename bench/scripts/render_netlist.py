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
 :root { --bg:#14161a; --panel:#1d2026; --fg:#d8dbe2; --dim:#94a0b4;
         --acc:#7ab3ff; --route:#e0746c; --logic:#69d0a8; --ff:#d9b06a;
         --line:#3a404b; }
 * { box-sizing:border-box; }
 body { margin:0; background:var(--bg); color:var(--fg);
        font:13px/1.45 ui-monospace,Menlo,Consolas,monospace; height:100vh;
        display:flex; flex-direction:column; }
 header { display:flex; align-items:center; gap:12px; padding:7px 12px;
          border-bottom:1px solid var(--line); }
 .tab { cursor:pointer; padding:4px 11px; border-radius:5px; color:var(--dim); }
 .tab.active { background:#2b3a55; color:var(--acc); }
 #clockstat { margin-left:auto; color:var(--dim); }
 #body { flex:1; display:flex; min-height:0; }
 #tree { width:290px; overflow:auto; border-right:1px solid var(--line);
         padding:6px; }
 #view { flex:1; display:flex; flex-direction:column; min-width:0; }
 #toolbar { padding:6px 10px; border-bottom:1px solid var(--line);
            display:flex; gap:10px; align-items:center; }
 #toolbar input { background:#0e1013; color:var(--fg); border:1px solid var(--line);
                  border-radius:4px; padding:3px 8px; font:inherit; }
 #canvaswrap { flex:1; overflow:auto; background:
   linear-gradient(90deg,#181b20 1px,transparent 1px) 0 0/28px 28px,
   linear-gradient(#181b20 1px,transparent 1px) 0 0/28px 28px; }
 #info { width:360px; overflow:auto; border-left:1px solid var(--line);
         padding:8px; font-size:12px; white-space:pre-wrap; }
 details { margin-left:10px; }
 summary { cursor:pointer; padding:1px 3px; border-radius:3px; }
 summary:hover { background:#262b33; }
 .counts { color:var(--dim); }
 .hidden { display:none !important; }
 svg text { fill:var(--fg); font:11px ui-monospace,monospace; }
 g.cell rect { fill:#20242b; stroke:#4a5260; rx:8; }
 g.cell.CCU2C rect { stroke:#b58fd0; }
 g.cell.FF rect { stroke:var(--ff); }
 g.cell.sel rect { stroke:var(--acc); stroke-width:2.5; }
 g.cell { cursor:pointer; }
 .legend span { margin-right:14px; }
 .sw { display:inline-block; width:10px; height:10px; border-radius:2px;
       margin-right:4px; vertical-align:-1px; }
</style>
</head>
<body>
<header>
  <span style="color:var(--acc)">struo viewer</span>
  <span class="tab active" data-tab="timing">Timing</span>
  <span class="tab" data-tab="schematic">Schematic</span>
  <span class="tab" data-tab="modules">Modules</span>
  <span id="clockstat"></span>
</header>
<div id="body">
  <div id="tree"></div>
  <div id="view">
    <div id="toolbar">
      <span class="counts legend">
        <span><i class="sw" style="background:var(--route)"></i>route</span>
        <span><i class="sw" style="background:var(--logic)"></i>logic</span>
        <span><i class="sw" style="background:var(--ff)"></i>clk-q / FF</span>
      </span>
      <input id="cellsearch" placeholder="filter cells (substring)…"
             style="margin-left:auto; min-width:220px">
      <span class="counts" id="sch-count"></span>
    </div>
    <div id="canvaswrap"><svg id="svg" width="2400" height="1400"></svg></div>
  </div>
  <div id="info"></div>
</div>
<script>
const DATA = __DATA__;
const WIRES = DATA.connectivity || {};
const TIMING = DATA.timing || {paths: [], clocks: {}};
const NS = "http://www.w3.org/2000/svg";
const $ = id => document.getElementById(id);

function svgEl(tag, attrs, parent) {
  const el = document.createElementNS(NS, tag);
  for (const k in attrs) el.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(el);
  return el;
}
function div(parent, cls, text) {
  if (!parent) return null;
  const el = document.createElement("div");
  if (cls) el.className = cls;
  if (text !== undefined) el.textContent = text;
  parent.appendChild(el);
  return el;
}

/* ================= shared: hierarchy ================= */
let ROOT = DATA.tree;
function findNode(path) {
  let cur = ROOT;
  for (const part of path) {
    cur = (cur.children || []).find(c => c.module === part);
    if (!cur) return null;
  }
  return cur;
}
let selectedPath = [];
let selectedNode = ROOT;
let schSelection = null;
let schModulePath = [];
let schFilter = "";

/* ================= info pane ================= */
function renderInfo(lines) {
  const info = $("info");
  info.textContent = "";
  for (const [cls, text] of lines) div(info, cls, text);
}

/* ================= TIMING: critical-path chains ================= */
function hopClass(step) {
  if ("net" in step) return "route";
  if (/FF/i.test(step.cell) && step.port === "Q") return "ff";
  return "logic";
}
function renderTiming() {
  const svg = $("svg");
  svg.innerHTML = "";
  const t = TIMING;
  const clocks = Object.entries(t.clocks || {});
  svgEl("text", {x: 18, y: 26}, svg).textContent =
    "achieved Fmax: " + (clocks.map(([n,v]) => `${v} MHz`).join(", ") || "n/a");
  if (!t.paths.length) {
    svgEl("text", {x: 18, y: 56, fill:"#e0746c"}, svg)
      .textContent = "no critical paths embedded — regenerate with --report <nextpnr-report.json>";
    return;
  }

  let top = 56;
  let widest = 0;
  t.paths.forEach((p, pi) => {
    svgEl("text", {x: 18, y: top}, svg)
      .textContent = `path ${pi+1}: total ${p.total_ps} ps — ${p.start}`;
    top += 22;

    // group consecutive steps into boxes (logic) and wires (net hops)
    const items = [];
    for (const step of p.steps) {
      if ("net" in step) {
        if (items.length && items[items.length-1].kind === "wire") {
          items[items.length-1].ps += step.delay_ps;
          items[items.length-1].nets.push(step.net);
        } else {
          items.push({kind:"wire", ps:step.delay_ps, nets:[step.net]});
        }
      } else if (items.length && items[items.length-1].kind === "cell"
                 && items[items.length-1].cell === step.cell) {
        items[items.length-1].ps += step.delay_ps;
        items[items.length-1].ports.push(step.port);
      } else {
        items.push({kind:"cell", cell:step.cell, ps:step.delay_ps,
                    ports:[step.port]});
      }
    }
    // drop leading/trailing wires for cleanliness
    while (items.length && items[0].kind === "wire") items.shift();
    while (items.length && items[items.length-1].kind === "wire") items.pop();

    const BOXW = 168, BOXH = 54, WIREW = 118, rowH = BOXH + 46;
    const nBoxes = items.filter(i => i.kind === "cell").length;
    widest = Math.max(widest, 30 + nBoxes * (BOXW + WIREW) + 40);
    let x = 24;
    const centerY = () => top + BOXH/2 + 6;
    items.forEach((item, idx) => {
      if (item.kind === "cell") {
        const g = svgEl("g", {class:"cell"}, svg);
        svgEl("rect", {x, y: top + 6, width: BOXW, height: BOXH}, g);
        const short = item.cell.length > 21
          ? item.cell.slice(0,20)+"…" : item.cell;
        svgEl("text", {x: x+10, y: top+24}, g).textContent = short;
        svgEl("text", {x: x+10, y: top+41, fill:"#8b93a1"}, g)
          .textContent = `${item.ps} ps · ${item.ports.join("/")}`;
        const t2 = svgEl("title", {}, g);
        t2.textContent = `${item.cell}\n${item.ps} ps`;
        x += BOXW;
      } else {
        const wireColor = "var(--route)";
        const y0 = centerY();
        svgEl("path", {d:`M ${x} ${y0} L ${x+WIREW-16} ${y0}`,
                       stroke: wireColor, "stroke-width": 3,
                       "marker-end":"url(#arrow)"}, svg);
        svgEl("text", {x: x+4, y: y0 - 9, fill:"#e0746c",
                       "font-size":"11px"}, svg)
          .textContent = `${item.ps} ps`;
        svgEl("text", {x: x+4, y: y0 + 20, fill:"#8b93a1",
                       "font-size":"10px"}, svg)
          .textContent = item.nets[0] || "";
        x += WIREW;
      }
    });
    top += rowH + 34;
  });
  svg.setAttribute("width", widest + 60);
  svgEl("defs", {}, svg).innerHTML =
    '<marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4"' +
    ' orient="auto"><path d="M0,0 L8,4 L0,8 z" fill="#e0746c"/></marker>';
}

/* ================= SCHEMATIC: real wires between cells ================= */

function moduleContainerOf(name) {
  // owning module directory of a flattened cell name
  const stripped = name.replace(/^(retime_|physical_replicate_)+/, "");
  const parts = stripped.split(".");
  parts.pop();
  return parts.join("/");
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

function schematicCellSet(modulePath, filterText) {
  const index = ensureCellIndex();
  const prefix = modulePath ? modulePath + "/" : "";
  let own = Object.keys(index).filter(name =>
    prefix
      ? name.startsWith(prefix) && !name.slice(prefix.length).includes("/")
      : !name.includes("/")
  );
  if (own.length === 0) {
    // anonymous mapper logic lives at top level; fall back to it
    own = Object.keys(index).filter(name => !name.includes("/"));
  }
  if (filterText) own = own.filter(n => n.includes(filterText));
  return own;
}

function renderSchematic() {
  const svg = $("svg");
  svg.innerHTML = "";
  svgEl("defs", {}, svg).innerHTML =
    '<marker id="arrow-s" markerWidth="9" markerHeight="9" refX="8" refY="4.5"' +
    ' orient="auto"><path d="M0,0 L9,4.5 L0,9 z" fill="#94a0b4"/></marker>';

  const moduleLabel = schModulePath.join("/") || "(top)";
  const allNames = schematicCellSet(schModulePath, schFilter);

  svgEl("text", {x: 16, y: 24, "font-size":"13px"}, svg)
    .textContent = `schematic: ${moduleLabel} — ${allNames.length} cells` +
      (allNames.length > 130 ? " (showing first 130; refine the filter)" : "");

  const shown = allNames.slice(0, 130);
  $("sch-count").textContent = `${shown.length}/${allNames.length}`;

  if (!shown.length) {
    svgEl("text", {x: 16, y: 56, fill:"#e0746c"}, svg)
      .textContent = "no cells match";
    return;
  }

  // wires internal to the shown set
  const inSet = new Set(shown);
  const wires = [];
  for (const bit of Object.keys(WIRES)) {
    const w = WIRES[bit];
    if (!w.driver) continue;
    if (!inSet.has(w.driver.cell)) continue;
    const sinks = (w.consumers || []).filter(c => inSet.has(c.cell));
    if (!sinks.length) continue;
    wires.push({bit, driver: w.driver, sinks, name: w.name || `w${bit}`});
  }

  // layered layout: depth = longest distance from a driverless cell
  const depth = new Map();
  const byName = new Map(shown.map(n => [n, {name:n}]));
  let remaining = shown.slice();
  let d = 0;
  while (remaining.length) {
    const layer = remaining.filter(name => {
      const ws = wires.filter(w =>
        w.sinks.some(c => c.cell === name));
      return !ws.some(w => depth.has(w.driver.cell) && !depth.has(name));
    });
    if (!layer.length) break;
    for (const name of layer) { depth.set(name, d); }
    remaining = remaining.filter(n => !depth.has(n));
    d++;
    if (d > 64) break;
  }
  for (const name of remaining) depth.set(name, d);

  const columns = new Map();
  for (const name of shown) {
    const dd = depth.get(name);
    if (!columns.has(dd)) columns.set(dd, []);
    columns.get(dd).push(name);
  }
  const COLW = 210, ROWH = 62;
  const ordered = [...columns.keys()].sort((a,b)=>a-b);
  const maxRows = Math.max(...ordered.map(k => columns.get(k).length), 1);
  svg.setAttribute("height", Math.max(900, 40 + maxRows*ROWH + 30));
  svg.setAttribute("width", Math.max(1600, 60 + ordered.length*COLW));

  const pos = new Map();
  for (const dd of ordered) {
    const col = columns.get(dd);
    col.sort();
    col.forEach((name, i) => pos.set(name, {
      x: 30 + dd * COLW,
      y: 46 + i * ROWH,
    }));
  }

  // wires first (under boxes)
  for (const w of wires) {
    const p1 = pos.get(w.driver.cell), p2s = w.sinks.map(s => pos.get(s.cell));
    if (!p1) continue;
    w.sinks.forEach((sink, si) => {
      const p2 = p2s[si];
      if (!p2) return;
      const x1 = p1.x + 158, y1 = p1.y + 26;
      const x2 = p2.x + 2,   y2 = p2.y + 26;
      const mx = (x1 + x2)/2;
      const hot = schSelection === w.driver.cell ||
                  schSelection === sink.cell;
      svgEl("path", {
        d:`M ${x1} ${y1} C ${mx} ${y1}, ${mx} ${y2}, ${x2 - 4} ${y2}`,
        stroke: hot ? "#7ab3ff" : "#5c6675",
        "stroke-width": hot ? 2.4 : 1.4,
        fill:"none",
        "marker-end":"url(#arrow-s)",
      }, svg);
      if (hot) {
        svgEl("text", {x: mx - 20, y: (y1+y2)/2 - 5,
          fill:"#7ab3ff", "font-size":"10px"}, svg).textContent = w.name;
      }
    });
  }

  // cells
  for (const name of shown) {
    const p = pos.get(name);
    const kind = /ccu|arith/i.test(name) ? "CCU2C"
               : /^ff_|(^|_)ff/.test(name) ? "FF" : "LUT4";
    const g = svgEl("g", {class:`cell ${kind}`}, svg);
    svgEl("rect", {x: p.x, y: p.y, width: 160, height: 44, rx: 8}, g);
    const short = name.length > 22 ? name.slice(0,21)+"…" : name;
    svgEl("text", {x: p.x + 9, y: p.y + 18}, g).textContent = short;
    svgEl("text", {x: p.x + 9, y: p.y + 34, fill:"#8b93a1"}, g).textContent = kind;
    g.addEventListener("click", () => {
      schSelection = schSelection === name ? null : name;
      renderSchematic();
      inspectCellInfo(name);
    });
  }
}

function inspectCellInfo(name) {
  const lines = [["", `cell: ${name}`]];
  for (const bit of Object.keys(WIRES)) {
    const w = WIRES[bit];
    const drv = w.driver && w.driver.cell === name;
    const used = (w.consumers || []).some(c => c.cell === name);
    if (!drv && !used) continue;
    const label = w.name || `w${bit}`;
    lines.push(["counts", `${drv ? "OUT" : "IN "} ${label}`]);
    if (drv) for (const c of w.consumers || [])
      lines.push(["counts", `   -> ${c.cell} (${c.port})`]);
    if (used && w.driver)
      lines.push(["counts", `   <- ${w.driver.cell} (${w.driver.port})`]);
  }
  renderInfo(lines);
}

/* ================= MODULES: aggregate graph ================= */
function renderModules() {
  const svg = $("svg");
  svg.innerHTML = "";
  let y = 30;
  const walk = (node, depth) => {
    svgEl("text", {x: 20 + depth*26, y, "font-size":"13px"}, svg)
      .textContent = `${node.module}/  ${countText(node)} | internal ${node.internal_wires}`;
    y += 24;
    for (const e of node.edges || []) {
      svgEl("text", {x: 44 + depth*26, y, fill:"#94a0b4"}, svg)
        .textContent = `↳ ${e.from} → ${e.to.join(",")} ×${e.count}`;
      y += 20;
    }
    for (const c of node.children || []) walk(c, depth + 1);
  };
  walk(ROOT, 0);
}

/* ================= count helper ================= */
function countText(node) {
  const c = node.cells || {LE:0,LUT4:0,CCU2C:0,FF:0};
  return `LE ${c.LE} | LUT ${c.LUT4} CCU ${c.CCU2C} FF ${c.FF}`;
}

/* ================= module tree ================= */
function buildTree(parentEl, node, path) {
  const hasKids = (node.children || []).length > 0;
  const row = document.createElement(hasKids ? "details" : "div");
  row.style.marginLeft = hasKids ? "0" : "12px";
  const head = document.createElement(hasKids ? "summary" : "div");
  div(head, null, node.module);
  div(head, "counts", ` ${countText(node)} | wires ${node.internal_wires}`);
  head.addEventListener("click", () => {
    selectedPath = path;
    selectedNode = node;
    schModulePath = path;
    schSelection = null;
    setTab("schematic");
    renderSchematic();
    renderTreeSelection(path);
    renderInfoHead(node, path);
  });
  row.appendChild(head);
  const inner = document.createElement("div");
  row.appendChild(inner);
  for (const child of node.children || [])
    buildTree(inner, child, [...path, child.module]);
  parentEl.appendChild(row);
}
function renderTreeSelection(path) { void path; }
function renderInfoHead(node, path) {
  renderInfo([
    ["", `module: ${path.join("/") || "(top)"}`],
    ["counts", countText(node)],
    ["counts", `internal wires: ${node.internal_wires}`],
  ]);
}

/* ================= tabs & boot ================= */
function setTab(name) {
  for (const tab of document.querySelectorAll("header .tab"))
    tab.classList.toggle("active", tab.dataset.tab === name);
  $("tab-timing").classList.toggle("hidden", name !== "timing");
  $("tab-schematic").classList.toggle("hidden", name !== "schematic");
  $("tab-modules").classList.toggle("hidden", name !== "modules");
  if (name === "timing") renderTiming();
  if (name === "schematic") renderSchematic();
  if (name === "modules") renderModules();
}
for (const tab of document.querySelectorAll("header .tab"))
  tab.addEventListener("click", () => setTab(tab.dataset.tab));
$("cellsearch").addEventListener("input", e => {
  schFilter = e.target.value.trim();
  renderSchematic();
});

buildTree($("tree"), ROOT, []);
schModulePath = [];
setTab("timing");

(function () {
  const clocks = Object.entries(TIMING.clocks || {})
    .map(([n, v]) => `${v} MHz`).join(", ");
  $("clockstat").textContent = clocks ? `Fmax: ${clocks}` : "";
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
    try:
        report = json.load(open(report_path))
    except (OSError, json.JSONDecodeError):
        return {"paths": []}
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
