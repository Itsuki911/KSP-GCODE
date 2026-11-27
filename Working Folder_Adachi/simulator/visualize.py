"""Interactive 3D Three.js WebGL HTML animation & Matplotlib PNG telemetry generator.

Generates a single self-contained HTML file (open directly after download — no
local server or ES module imports required) featuring:
  - Real Machine Mode (rotating turntable + workpiece, nozzle moving along Y/Z)
    AND Workpiece-Fixed Mode (legacy static workpiece view), switchable live.
  - Multi-view synchronized layout (Main 3D Perspective + Top Orthographic +
    Side R-Z Orthographic) plus one-click camera presets (3D / Top / Side / Front / Nozzle).
  - Synchronized HiDPI 2D telemetry charts (Axis Positions & Speeds) with hover
    readout, legend toggles, and click-to-seek.
  - Clean, non-overlapping bottom playback controls and legend bar.
"""
from __future__ import annotations

import html as html_lib
import json
import os
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

from klipper_sim import SimulationResult
from workpiece_model import (
    DEFAULT_WORKPIECE,
    WorkpieceSpec,
    build_plotly_workpiece_meshes,
    build_workpiece_labels,
    build_workpiece_wireframe_lines,
    polar_to_wp_xy,
)

# Avoid matplotlib font-cache permission warnings inside sandboxes/non-writable HOME
os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "ksp_mpl_cache"))


def _select_export_indices(sim: SimulationResult, max_points: int = 2200) -> np.ndarray:
    """Select time indices preserving endpoints, valve edges, and segment transitions."""
    n = len(sim.t)
    if n <= max_points:
        return np.arange(n, dtype=int)

    must_keep: set[int] = {0, n - 1}
    if n > 1:
        valve_changes = np.flatnonzero(np.diff(sim.valve_open.astype(np.int8)) != 0)
        for idx in valve_changes:
            must_keep.add(int(idx))
            must_keep.add(int(idx + 1))

    # Preserve exact segment boundaries via searchsorted
    for seg in sim.segments:
        for bt in (seg.t_start, seg.t_start + seg.duration):
            pos = int(np.searchsorted(sim.t, bt))
            if 0 <= pos < n:
                must_keep.add(pos)
            if pos > 0:
                must_keep.add(pos - 1)

    base = np.linspace(0, n - 1, max_points).round().astype(int)
    merged = np.unique(np.concatenate([base, np.fromiter(must_keep, dtype=int)]))
    return merged


def _round_list(arr: np.ndarray, decimals: int = 3) -> list[float]:
    return np.round(arr.astype(float), decimals).tolist()


def _build_html_payload(
    sim: SimulationResult,
    mujoco_report: dict[str, Any] | None,
    show_workpiece: bool,
    workpiece_spec: WorkpieceSpec,
) -> dict[str, Any]:
    idxs = _select_export_indices(sim)
    r_max = float(sim.config.r_center_mm)

    meshes_payload: list[dict[str, Any]] = []
    wireframe_payload: list[list[float | None]] = []
    labels_payload: list[dict[str, Any]] = []

    if show_workpiece:
        raw_meshes = build_plotly_workpiece_meshes(workpiece_spec)
        # Slightly richer opacity for Three.js lit surfaces while preserving path clarity
        opacity_map = {
            "Workpiece 3D (Cast Body)": 0.62,
            "Workpiece 3D (Machined Faces)": 0.78,
            "Workpiece 3D (Blue Purge Pad)": 0.78,
        }
        for m in raw_meshes:
            v = np.round(m["verts"].astype(float), 2)
            f = m["faces"].astype(int)
            meshes_payload.append(
                {
                    "name": m["name"],
                    "color": m["color"],
                    "opacity": opacity_map.get(m["name"], float(m["opacity"])),
                    "verts": v.reshape(-1).tolist(),
                    "faces": f.reshape(-1).tolist(),
                }
            )

        wx, wy, wz = build_workpiece_wireframe_lines(workpiece_spec)
        # Convert None-separated polylines into flat segment pairs [x0,y0,z0, x1,y1,z1, ...]
        seg_pts: list[float] = []
        for i in range(len(wx) - 1):
            if wx[i] is not None and wx[i + 1] is not None:
                seg_pts.extend(
                    [
                        round(float(wx[i]), 2),
                        round(float(wy[i]), 2),
                        round(float(wz[i]), 2),
                        round(float(wx[i + 1]), 2),
                        round(float(wy[i + 1]), 2),
                        round(float(wz[i + 1]), 2),
                    ]
                )
        wireframe_payload = seg_pts  # type: ignore[assignment]

        for text, lx, ly, lz in build_workpiece_labels(workpiece_spec):
            labels_payload.append(
                {
                    "text": text,
                    "x": round(float(lx), 2),
                    "y": round(float(ly), 2),
                    "z": round(float(lz), 2),
                }
            )

    summary = sim.summary()
    return {
        "file_name": sim.file_name,
        "r_center_mm": round(r_max, 3),
        "z_max_mm": round(float(max(55.0, sim.config.safe_z_max + 8.0)), 2),
        "show_workpiece_default": bool(show_workpiece),
        "summary": summary,
        "mujoco": mujoco_report,
        "workpiece": {
            "meshes": meshes_payload,
            "wireframe_segments": wireframe_payload,
            "labels": labels_payload,
        },
        "series": {
            "t": _round_list(sim.t[idxs], 3),
            "x_deg": _round_list(sim.x_deg[idxs], 3),
            "y_mm": _round_list(sim.y_mm[idxs], 3),
            "z_mm": _round_list(sim.z_mm[idxs], 3),
            "r_mm": _round_list(sim.r_mm[idxs], 3),
            "x_wp_mm": _round_list(sim.x_wp_mm[idxs], 3),
            "y_wp_mm": _round_list(sim.y_wp_mm[idxs], 3),
            "valve_deg": _round_list(sim.valve_deg[idxs], 2),
            "valve_open": [1 if bool(v) else 0 for v in sim.valve_open[idxs]],
            "surface_speed": _round_list(sim.surface_speed_mm_s[idxs], 2),
            "planner_speed": _round_list(sim.planner_speed[idxs], 2),
            "abs_vz": _round_list(np.abs(sim.vz_mm_s[idxs]), 2),
        },
    }


_HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>__TITLE__</title>
  <style>
    :root {
      --bg-0: #090e1a;
      --bg-1: #0f172a;
      --bg-2: #162033;
      --bg-3: #1e293b;
      --border: #283548;
      --border-hi: #3b4f6b;
      --text: #e2e8f0;
      --muted: #94a3b8;
      --accent: #38bdf8;
      --orange: #ff6b00;
      --green: #34d399;
      --yellow: #fbbf24;
      --pink: #fb7185;
      --purple: #a78bfa;
      --red: #f43f5e;
    }
    * { box-sizing: border-box; }
    html, body {
      margin: 0;
      padding: 0;
      height: 100%;
      background: var(--bg-0);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      font-size: 13px;
      overflow: hidden;
    }
    .app {
      display: flex;
      flex-direction: column;
      height: 100vh;
      width: 100vw;
      overflow: hidden;
    }

    /* Header */
    .header {
      flex: 0 0 auto;
      background: var(--bg-1);
      border-bottom: 1px solid var(--border);
      padding: 8px 14px;
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      justify-content: space-between;
      gap: 8px 14px;
    }
    .title-block {
      display: flex;
      align-items: baseline;
      gap: 10px;
      min-width: 0;
    }
    .title-block h1 {
      margin: 0;
      font-size: 15px;
      font-weight: 700;
      color: #f8fafc;
      white-space: nowrap;
    }
    .title-file {
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      font-size: 12.5px;
      color: var(--accent);
      background: rgba(56, 189, 248, 0.12);
      border: 1px solid rgba(56, 189, 248, 0.28);
      padding: 2px 7px;
      border-radius: 5px;
      white-space: nowrap;
    }
    .stats-bar {
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 6px;
    }
    .stat-pill {
      background: var(--bg-2);
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 3px 8px;
      font-size: 11.5px;
      color: #cbd5e1;
      white-space: nowrap;
    }
    .stat-pill b { color: #f8fafc; font-weight: 600; }
    .stat-pill.ok { border-color: rgba(74, 222, 128, 0.4); color: #86efac; }
    .stat-pill.bad { border-color: rgba(248, 113, 113, 0.5); color: #fca5a5; background: rgba(127, 29, 29, 0.35); }
    .stat-pill.warn { border-color: rgba(251, 191, 36, 0.45); color: #fde68a; }

    /* Toolbar */
    .toolbar {
      flex: 0 0 auto;
      background: var(--bg-1);
      border-bottom: 1px solid var(--border);
      padding: 6px 14px;
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 8px 14px;
    }
    .tool-group {
      display: flex;
      align-items: center;
      gap: 4px;
      flex-wrap: wrap;
    }
    .tool-label {
      font-size: 11px;
      font-weight: 600;
      color: var(--muted);
      margin-right: 2px;
      white-space: nowrap;
    }
    .seg-group {
      display: inline-flex;
      background: var(--bg-0);
      border: 1px solid var(--border);
      border-radius: 7px;
      padding: 2px;
      gap: 2px;
    }
    .btn {
      appearance: none;
      border: 1px solid transparent;
      background: transparent;
      color: #cbd5e1;
      padding: 4px 9px;
      border-radius: 5px;
      font-size: 12px;
      font-weight: 500;
      cursor: pointer;
      transition: background 0.12s, color 0.12s, border-color 0.12s;
      white-space: nowrap;
      line-height: 1.25;
    }
    .btn:hover {
      background: var(--bg-3);
      color: #f8fafc;
    }
    .btn.active {
      background: rgba(56, 189, 248, 0.2);
      border-color: rgba(56, 189, 248, 0.55);
      color: #e0f2fe;
      font-weight: 600;
    }
    .btn-outline {
      background: var(--bg-2);
      border-color: var(--border);
    }
    .btn-outline.active {
      background: rgba(56, 189, 248, 0.2);
      border-color: rgba(56, 189, 248, 0.55);
      color: #e0f2fe;
    }
    .divider {
      width: 1px;
      height: 20px;
      background: var(--border);
    }

    /* Main Workspace */
    .main {
      flex: 1 1 auto;
      min-height: 0;
      display: grid;
      grid-template-columns: minmax(0, 1.55fr) minmax(300px, 1fr);
      gap: 8px;
      padding: 8px 10px;
      overflow: hidden;
    }

    /* Left: 3D viewport container */
    .viewport-card {
      position: relative;
      background: var(--bg-1);
      border: 1px solid var(--border);
      border-radius: 9px;
      overflow: hidden;
      min-height: 0;
      display: flex;
      flex-direction: column;
    }
    .canvas-host {
      position: relative;
      flex: 1 1 auto;
      min-height: 0;
      width: 100%;
      background: radial-gradient(circle at 40% 35%, #131f38 0%, #0b1120 75%);
      cursor: grab;
      user-select: none;
    }
    .canvas-host:active { cursor: grabbing; }
    .canvas-host canvas {
      display: block;
      width: 100%;
      height: 100%;
    }
    /* Subview divider lines & labels when Multi-View is active */
    .vp-overlay {
      position: absolute;
      inset: 0;
      pointer-events: none;
    }
    .vp-badge {
      position: absolute;
      background: rgba(9, 14, 26, 0.78);
      border: 1px solid rgba(148, 163, 184, 0.25);
      backdrop-filter: blur(4px);
      color: #e2e8f0;
      font-size: 11px;
      font-weight: 600;
      padding: 3px 8px;
      border-radius: 5px;
      letter-spacing: 0.01em;
    }
    .vp-badge.clickable {
      pointer-events: auto;
      cursor: pointer;
    }
    .vp-badge.clickable:hover {
      border-color: var(--accent);
      color: #ffffff;
    }
    .vp-badge .sub {
      color: var(--muted);
      font-weight: 400;
      margin-left: 4px;
    }
    .hud-readout {
      position: absolute;
      left: 10px;
      bottom: 10px;
      background: rgba(9, 14, 26, 0.84);
      border: 1px solid rgba(56, 189, 248, 0.28);
      backdrop-filter: blur(4px);
      border-radius: 7px;
      padding: 6px 10px;
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      font-size: 11.5px;
      line-height: 1.5;
      color: #e2e8f0;
      pointer-events: none;
    }
    .hud-row {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }
    .hud-valve-open {
      color: #fb923c;
      font-weight: 700;
    }
    .hud-valve-closed {
      color: #38bdf8;
      font-weight: 700;
    }
    .vp-split-v {
      position: absolute;
      top: 0;
      bottom: 0;
      width: 1px;
      background: rgba(148, 163, 184, 0.3);
      display: none;
    }
    .vp-split-h {
      position: absolute;
      right: 0;
      height: 1px;
      background: rgba(148, 163, 184, 0.3);
      display: none;
    }
    .fallback-msg {
      position: absolute;
      inset: 0;
      display: none;
      align-items: center;
      justify-content: center;
      padding: 24px;
      text-align: center;
      color: #cbd5e1;
      background: rgba(9, 14, 26, 0.92);
    }

    /* Right: Telemetry Charts */
    .telemetry-col {
      display: flex;
      flex-direction: column;
      gap: 8px;
      min-height: 0;
      overflow: hidden;
    }
    .chart-card {
      flex: 1 1 0;
      min-height: 0;
      background: var(--bg-1);
      border: 1px solid var(--border);
      border-radius: 9px;
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }
    .chart-head {
      flex: 0 0 auto;
      padding: 6px 10px 4px;
      border-bottom: 1px solid rgba(40, 53, 72, 0.7);
      display: flex;
      flex-direction: column;
      gap: 4px;
    }
    .chart-title {
      font-size: 12px;
      font-weight: 600;
      color: #f1f5f9;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .chart-hint {
      font-size: 10.5px;
      font-weight: 400;
      color: var(--muted);
    }
    .chart-legend {
      display: flex;
      flex-wrap: wrap;
      gap: 5px 10px;
    }
    .chart-body {
      position: relative;
      flex: 1 1 auto;
      min-height: 0;
      width: 100%;
    }
    .chart-body canvas {
      display: block;
      width: 100%;
      height: 100%;
      cursor: crosshair;
    }
    .chart-tooltip {
      position: absolute;
      display: none;
      pointer-events: none;
      background: rgba(9, 14, 26, 0.92);
      border: 1px solid var(--border-hi);
      border-radius: 6px;
      padding: 5px 8px;
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      font-size: 11px;
      line-height: 1.45;
      color: #f8fafc;
      white-space: nowrap;
      z-index: 10;
    }

    /* Footer: Dedicated Playback + Clean Non-Overlapping Legend */
    .footer {
      flex: 0 0 auto;
      background: var(--bg-1);
      border-top: 1px solid var(--border);
      padding: 8px 14px 10px;
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    .playback-row {
      display: flex;
      align-items: center;
      gap: 10px;
      flex-wrap: wrap;
    }
    .play-btn {
      background: #0284c7;
      border: 1px solid #38bdf8;
      color: #ffffff;
      font-weight: 700;
      padding: 5px 14px;
      border-radius: 6px;
      cursor: pointer;
      font-size: 12.5px;
      min-width: 84px;
    }
    .play-btn:hover { background: #0369a1; }
    .play-btn.playing { background: #ea580c; border-color: #fb923c; }
    .play-btn.playing:hover { background: #c2410c; }
    .slider-wrap {
      flex: 1 1 280px;
      display: flex;
      flex-direction: column;
      gap: 3px;
      min-width: 180px;
    }
    .slider-input {
      width: 100%;
      height: 6px;
      accent-color: #38bdf8;
      cursor: pointer;
      margin: 0;
    }
    .slider-ticks {
      display: flex;
      justify-content: space-between;
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      font-size: 10.5px;
      color: var(--muted);
      user-select: none;
      padding: 0 2px;
    }
    .time-readout {
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      font-size: 12.5px;
      font-weight: 600;
      color: #f8fafc;
      background: var(--bg-2);
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 4px 9px;
      white-space: nowrap;
      min-width: 126px;
      text-align: center;
    }
    .legend-bar {
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 6px 12px;
      padding-top: 6px;
      border-top: 1px solid rgba(40, 53, 72, 0.75);
    }
    .legend-item {
      display: inline-flex;
      align-items: center;
      gap: 5px;
      font-size: 11.5px;
      color: #cbd5e1;
      cursor: pointer;
      user-select: none;
      padding: 2px 6px;
      border-radius: 5px;
      border: 1px solid transparent;
      background: transparent;
      transition: opacity 0.12s, background 0.12s;
    }
    .legend-item:hover {
      background: rgba(148, 163, 184, 0.1);
      color: #ffffff;
    }
    .legend-item.off {
      opacity: 0.38;
      text-decoration: line-through;
    }
    .swatch-line {
      display: inline-block;
      width: 16px;
      height: 3px;
      border-radius: 2px;
    }
    .swatch-dash {
      display: inline-block;
      width: 16px;
      height: 0;
      border-top: 2px dashed;
    }
    .swatch-box {
      display: inline-block;
      width: 11px;
      height: 11px;
      border-radius: 2px;
      border: 1px solid rgba(255, 255, 255, 0.35);
    }
    .swatch-dot {
      display: inline-block;
      width: 9px;
      height: 9px;
      border-radius: 50%;
      border: 1.5px solid #ffffff;
    }

    @media (max-width: 980px) {
      html, body { overflow: auto; }
      .app { height: auto; min-height: 100vh; overflow: visible; }
      .main {
        grid-template-columns: 1fr;
        grid-template-rows: 540px 440px;
      }
    }
  </style>
</head>
<body>
<div class="app">
  <!-- Header -->
  <header class="header">
    <div class="title-block">
      <h1>KSP-GCODE 3D Simulator</h1>
      <span class="title-file" id="hdr-file"></span>
    </div>
    <div class="stats-bar" id="hdr-stats"></div>
  </header>

  <!-- Mode / Camera / Layer Toolbar -->
  <div class="toolbar">
    <div class="tool-group">
      <span class="tool-label">Mode:</span>
      <div class="seg-group">
        <button class="btn active" id="mode-machine" title="Matches physical machine: turntable and workpiece rotate in X while the nozzle moves only in radial Y and vertical Z">🔄 Real Machine (Rotating Workpiece)</button>
        <button class="btn" id="mode-fixed" title="Legacy view: workpiece stays stationary while the nozzle and active angle ray sweep around">📌 Workpiece Fixed (Legacy)</button>
      </div>
    </div>

    <div class="divider"></div>

    <div class="tool-group">
      <span class="tool-label">Main View:</span>
      <div class="seg-group">
        <button class="btn active" data-cam="iso">3D Iso</button>
        <button class="btn" data-cam="top">Top</button>
        <button class="btn" data-cam="side">Side</button>
        <button class="btn" data-cam="front">Front</button>
        <button class="btn" data-cam="nozzle">Nozzle</button>
      </div>
    </div>

    <div class="divider"></div>

    <div class="tool-group">
      <button class="btn btn-outline active" id="btn-multiview" title="Show Main 3D, Top, and Side views simultaneously">⊞ Multi-View (3D + Top + Side)</button>
      <button class="btn btn-outline active" id="btn-machine-rig" title="Toggle turntable base, Y rail, and Z nozzle head">⚙ Machine Rig</button>
      <button class="btn btn-outline active" id="btn-labels" title="Toggle 3D workpiece feature labels">🏷 Labels</button>
    </div>
  </div>

  <!-- Main Workspace -->
  <main class="main">
    <!-- Left: 3D Multi-View Canvas -->
    <section class="viewport-card">
      <div class="canvas-host" id="three-host">
        <div class="vp-overlay">
          <div class="vp-split-v" id="split-v"></div>
          <div class="vp-split-h" id="split-h"></div>
          <div class="vp-badge" id="badge-main" style="left: 10px; top: 10px;">
            Main 3D <span class="sub">Drag: Orbit / Right-Drag: Pan / Wheel: Zoom</span>
          </div>
          <div class="vp-badge clickable" id="badge-top" style="display:none;" title="Click to focus Top view in Main viewport">
            Top View (Orthographic)
          </div>
          <div class="vp-badge clickable" id="badge-side" style="display:none;" title="Click to focus Side view in Main viewport">
            Side View (R-Z / Orthographic)
          </div>
          <div class="hud-readout" id="hud-readout"></div>
        </div>
        <div class="fallback-msg" id="fallback-msg"></div>
      </div>
    </section>

    <!-- Right: Synchronized Telemetry Charts -->
    <aside class="telemetry-col">
      <div class="chart-card">
        <div class="chart-head">
          <div class="chart-title">
            <span>Axis Positions (X [deg], Y [mm], Z [mm]) &amp; Valve [deg]</span>
            <span class="chart-hint">Click to seek / Click legend to toggle</span>
          </div>
          <div class="chart-legend" id="legend-pos"></div>
        </div>
        <div class="chart-body" id="wrap-pos">
          <canvas id="canvas-pos"></canvas>
          <div class="chart-tooltip" id="tip-pos"></div>
        </div>
      </div>

      <div class="chart-card">
        <div class="chart-head">
          <div class="chart-title">
            <span>Workpiece Surface Speed vs. Planner Speed &amp; |Vz|</span>
            <span class="chart-hint">Click to seek / Click legend to toggle</span>
          </div>
          <div class="chart-legend" id="legend-spd"></div>
        </div>
        <div class="chart-body" id="wrap-spd">
          <canvas id="canvas-spd"></canvas>
          <div class="chart-tooltip" id="tip-spd"></div>
        </div>
      </div>
    </aside>
  </main>

  <!-- Dedicated Bottom Playback + Legend Bar (Zero Overlap) -->
  <footer class="footer">
    <div class="playback-row">
      <button class="play-btn" id="btn-play">▶ Play</button>
      <button class="btn btn-outline" id="btn-reset" title="Reset to start">⏮ 0.00s</button>
      <div class="seg-group" id="speed-group" title="Playback speed">
        <button class="btn" data-speed="0.25">0.25x</button>
        <button class="btn" data-speed="0.5">0.5x</button>
        <button class="btn active" data-speed="1">1x</button>
        <button class="btn" data-speed="2">2x</button>
        <button class="btn" data-speed="5">5x</button>
      </div>
      <button class="btn btn-outline active" id="btn-loop" title="Loop playback">🔁 Loop</button>

      <div class="slider-wrap">
        <input type="range" class="slider-input" id="time-slider" min="0" max="1000" step="1" value="0" />
        <div class="slider-ticks" id="slider-ticks"></div>
      </div>

      <div class="time-readout" id="time-readout">0.00s / 0.00s</div>
    </div>

    <div class="legend-bar" id="legend-3d"></div>
  </footer>
</div>

<script id="sim-data" type="application/json">__PAYLOAD_JSON__</script>

<!-- Self-contained CDN fallback loader for Three.js UMD (works on file:// double-click) -->
<script>
(function() {
  var SIM = JSON.parse(document.getElementById("sim-data").textContent);

  // ---------------------------------------------------------------------------
  // 1. Header Stats & Helper Utilities
  // ---------------------------------------------------------------------------
  function fmt(v, d) { return Number(v).toFixed(d); }
  function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

  document.getElementById("hdr-file").textContent = SIM.file_name;
  var statsEl = document.getElementById("hdr-stats");
  var S = SIM.summary || {};
  function addPill(html, cls) {
    var d = document.createElement("span");
    d.className = "stat-pill" + (cls ? " " + cls : "");
    d.innerHTML = html;
    statsEl.appendChild(d);
  }
  addPill("Total: <b>" + fmt(S.total_time_s || 0, 2) + "s</b> (Move " + fmt(S.move_time_s || 0, 1) + "s / Home " + fmt(S.home_time_s || 0, 1) + "s / Dwell " + fmt(S.dwell_time_s || 0, 1) + "s)");
  var zr = S.z_range_mm || [0, 0];
  addPill("Z: <b>[" + fmt(zr[0], 2) + ", " + fmt(zr[1], 2) + "] mm</b>");
  addPill("Peak Surface: <b>" + fmt(S.peak_surface_speed_mm_s || 0, 1) + " mm/s</b>");
  if (SIM.mujoco) {
    var mj = SIM.mujoco;
    var ok = !!mj.collision_free;
    var wpTxt = (mj.min_workpiece_clearance_mm !== null && mj.min_workpiece_clearance_mm !== undefined)
      ? (" / WP " + fmt(mj.min_workpiece_clearance_mm, 2) + " mm (" + (mj.min_workpiece_geom || "") + ")")
      : "";
    addPill("MuJoCo: <b>" + (ok ? "COLLISION-FREE" : "COLLISION") + "</b> (Bed " + fmt(mj.min_bed_clearance_mm, 2) + " mm" + wpTxt + ")", ok ? "ok" : "bad");
  }
  if ((S.warning_count || 0) > 0) {
    addPill("Warnings: <b>" + S.warning_count + "</b>", "warn");
  }

  // ---------------------------------------------------------------------------
  // 2. Time-Series Interpolation
  // ---------------------------------------------------------------------------
  var SER = SIM.series;
  var N = SER.t.length;
  var TOTAL_T = N > 0 ? SER.t[N - 1] : 0;

  function sampleAtTime(t) {
    if (N <= 1 || TOTAL_T <= 0) {
      return {
        idx: 0, t: SER.t[0] || 0, x_deg: SER.x_deg[0] || 0, y_mm: SER.y_mm[0] || 0,
        z_mm: SER.z_mm[0] || 0, r_mm: SER.r_mm[0] || SIM.r_center_mm,
        x_wp: SER.x_wp_mm[0] || 0, y_wp: SER.y_wp_mm[0] || 0,
        valve_deg: SER.valve_deg[0] || 0, valve_open: !!(SER.valve_open[0]),
        surface_speed: SER.surface_speed[0] || 0, planner_speed: SER.planner_speed[0] || 0,
        abs_vz: SER.abs_vz[0] || 0
      };
    }
    t = clamp(t, 0, TOTAL_T);
    // Binary search
    var lo = 0, hi = N - 1;
    while (lo < hi - 1) {
      var mid = (lo + hi) >> 1;
      if (SER.t[mid] <= t) lo = mid; else hi = mid;
    }
    var t0 = SER.t[lo], t1 = SER.t[hi];
    var a = (t1 > t0) ? clamp((t - t0) / (t1 - t0), 0, 1) : 0;
    function lerp(arr) { return arr[lo] + (arr[hi] - arr[lo]) * a; }
    var xDeg = lerp(SER.x_deg);
    var yMm = lerp(SER.y_mm);
    var zMm = lerp(SER.z_mm);
    var rMm = SIM.r_center_mm - yMm;
    var rad = xDeg * Math.PI / 180;
    return {
      idx: a < 0.5 ? lo : hi,
      t: t,
      x_deg: xDeg,
      y_mm: yMm,
      z_mm: zMm,
      r_mm: rMm,
      x_wp: rMm * Math.cos(rad),
      y_wp: -rMm * Math.sin(rad),
      valve_deg: lerp(SER.valve_deg),
      valve_open: !!(SER.valve_open[a < 0.5 ? lo : hi]),
      surface_speed: lerp(SER.surface_speed),
      planner_speed: lerp(SER.planner_speed),
      abs_vz: lerp(SER.abs_vz)
    };
  }

  // Shared UI State
  var state = {
    mode: "machine",       // "machine" | "fixed"
    camPreset: "iso",      // "iso" | "top" | "side" | "front" | "nozzle"
    multiView: true,
    playing: false,
    loop: true,
    speed: 1.0,
    currentTime: 0.0,
    layers: {
      workpiece: !!SIM.show_workpiece_default,
      outlines: !!SIM.show_workpiece_default,
      labels: !!SIM.show_workpiece_default,
      rig: true,
      pathClosed: true,
      pathOpen: true,
      nozzle: true,
      angleRay: true
    }
  };

  // ---------------------------------------------------------------------------
  // 3. Slider Ticks & Playback UI
  // ---------------------------------------------------------------------------
  var sliderEl = document.getElementById("time-slider");
  var ticksEl = document.getElementById("slider-ticks");
  var timeReadoutEl = document.getElementById("time-readout");
  var playBtn = document.getElementById("btn-play");

  function buildSliderTicks() {
    ticksEl.innerHTML = "";
    var count = 7;
    if (TOTAL_T <= 0) {
      var s0 = document.createElement("span");
      s0.textContent = "0.00s";
      ticksEl.appendChild(s0);
      sliderEl.max = "0";
      return;
    }
    sliderEl.max = "1000";
    for (var i = 0; i < count; i++) {
      var sp = document.createElement("span");
      sp.textContent = fmt((TOTAL_T * i) / (count - 1), 2) + "s";
      ticksEl.appendChild(sp);
    }
  }
  buildSliderTicks();

  function updatePlaybackUI() {
    timeReadoutEl.textContent = fmt(state.currentTime, 2) + "s / " + fmt(TOTAL_T, 2) + "s";
    if (TOTAL_T > 0) {
      sliderEl.value = String(Math.round((state.currentTime / TOTAL_T) * 1000));
    } else {
      sliderEl.value = "0";
    }
    playBtn.textContent = state.playing ? "⏸ Pause" : "▶ Play";
    playBtn.classList.toggle("playing", state.playing);
  }

  playBtn.addEventListener("click", function() {
    if (TOTAL_T <= 0) return;
    if (!state.playing && state.currentTime >= TOTAL_T - 1e-4) {
      state.currentTime = 0;
    }
    state.playing = !state.playing;
    updatePlaybackUI();
  });

  document.getElementById("btn-reset").addEventListener("click", function() {
    state.playing = false;
    state.currentTime = 0;
    updatePlaybackUI();
    onTimeUpdated();
  });

  sliderEl.addEventListener("input", function() {
    if (TOTAL_T <= 0) return;
    state.currentTime = (Number(sliderEl.value) / 1000) * TOTAL_T;
    updatePlaybackUI();
    onTimeUpdated();
  });

  document.getElementById("btn-loop").addEventListener("click", function(e) {
    state.loop = !state.loop;
    e.currentTarget.classList.toggle("active", state.loop);
  });

  document.querySelectorAll("#speed-group .btn").forEach(function(b) {
    b.addEventListener("click", function() {
      state.speed = Number(b.getAttribute("data-speed")) || 1.0;
      document.querySelectorAll("#speed-group .btn").forEach(function(x) { x.classList.remove("active"); });
      b.classList.add("active");
    });
  });

  // Keyboard shortcuts (Space = play/pause, Left/Right = step)
  window.addEventListener("keydown", function(e) {
    if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA")) return;
    if (e.code === "Space") {
      e.preventDefault();
      playBtn.click();
    } else if (e.code === "ArrowRight" && TOTAL_T > 0) {
      state.playing = false;
      state.currentTime = clamp(state.currentTime + Math.max(0.1, TOTAL_T / 120), 0, TOTAL_T);
      updatePlaybackUI();
      onTimeUpdated();
    } else if (e.code === "ArrowLeft" && TOTAL_T > 0) {
      state.playing = false;
      state.currentTime = clamp(state.currentTime - Math.max(0.1, TOTAL_T / 120), 0, TOTAL_T);
      updatePlaybackUI();
      onTimeUpdated();
    }
  });

  // ---------------------------------------------------------------------------
  // 4. Lightweight HiDPI 2D Telemetry Charts (Always Works Even Offline)
  // ---------------------------------------------------------------------------
  function TelemetryChart(canvasId, wrapId, legendId, tipId, seriesDefs, yUnit) {
    var canvas = document.getElementById(canvasId);
    var wrap = document.getElementById(wrapId);
    var legendEl = document.getElementById(legendId);
    var tipEl = document.getElementById(tipId);
    var ctx = canvas.getContext("2d");
    var visible = seriesDefs.map(function() { return true; });
    var hoverX = null;
    var dragging = false;
    var pad = { l: 44, r: 22, t: 10, b: 22 };

    // Build clickable legend chips
    seriesDefs.forEach(function(sd, idx) {
      var item = document.createElement("button");
      item.type = "button";
      item.className = "legend-item";
      item.innerHTML = (sd.dash
        ? '<span class="swatch-dash" style="border-color:' + sd.color + '"></span>'
        : '<span class="swatch-line" style="background:' + sd.color + '"></span>') +
        "<span>" + sd.name + "</span>";
      item.addEventListener("click", function() {
        visible[idx] = !visible[idx];
        item.classList.toggle("off", !visible[idx]);
        draw();
      });
      legendEl.appendChild(item);
    });

    function computeYMax() {
      var ym = 10;
      seriesDefs.forEach(function(sd, idx) {
        if (!visible[idx]) return;
        for (var i = 0; i < sd.data.length; i++) {
          if (sd.data[i] > ym) ym = sd.data[i];
        }
      });
      return ym * 1.08;
    }

    function timeFromClientX(clientX) {
      var rect = canvas.getBoundingClientRect();
      var x = clientX - rect.left;
      var plotW = Math.max(1, rect.width - pad.l - pad.r);
      var frac = clamp((x - pad.l) / plotW, 0, 1);
      return frac * TOTAL_T;
    }

    function updateHover(clientX, clientY) {
      var rect = canvas.getBoundingClientRect();
      var x = clientX - rect.left;
      var y = clientY - rect.top;
      if (x < pad.l || x > rect.width - pad.r || y < pad.t || y > rect.height - pad.b) {
        hoverX = null;
        tipEl.style.display = "none";
        draw();
        return;
      }
      hoverX = x;
      var t = timeFromClientX(clientX);
      var st = sampleAtTime(t);
      var lines = ["<b>t = " + fmt(st.t, 2) + "s</b>"];
      seriesDefs.forEach(function(sd, idx) {
        if (!visible[idx]) return;
        var val = sd.sample(st);
        lines.push('<span style="color:' + sd.color + '">●</span> ' + sd.short + ": " + fmt(val, sd.dec));
      });
      tipEl.innerHTML = lines.join("<br>");
      tipEl.style.display = "block";
      var tw = tipEl.offsetWidth || 120;
      var th = tipEl.offsetHeight || 60;
      var left = (x + 12 + tw < rect.width) ? (x + 12) : Math.max(4, x - tw - 12);
      var top = clamp(y - th / 2, 4, Math.max(4, rect.height - th - 4));
      tipEl.style.left = left + "px";
      tipEl.style.top = top + "px";
      draw();
    }

    canvas.addEventListener("mousedown", function(e) {
      if (TOTAL_T <= 0) return;
      dragging = true;
      state.playing = false;
      state.currentTime = timeFromClientX(e.clientX);
      updatePlaybackUI();
      onTimeUpdated();
    });
    window.addEventListener("mousemove", function(e) {
      if (dragging && TOTAL_T > 0) {
        state.currentTime = timeFromClientX(e.clientX);
        updatePlaybackUI();
        onTimeUpdated();
      }
    });
    window.addEventListener("mouseup", function() { dragging = false; });
    canvas.addEventListener("mousemove", function(e) { updateHover(e.clientX, e.clientY); });
    canvas.addEventListener("mouseleave", function() {
      hoverX = null;
      tipEl.style.display = "none";
      draw();
    });

    function draw() {
      var rect = wrap.getBoundingClientRect();
      var w = Math.max(100, Math.floor(rect.width));
      var h = Math.max(80, Math.floor(rect.height));
      var dpr = Math.min(window.devicePixelRatio || 1, 2);
      if (canvas.width !== Math.floor(w * dpr) || canvas.height !== Math.floor(h * dpr)) {
        canvas.width = Math.floor(w * dpr);
        canvas.height = Math.floor(h * dpr);
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);

      var plotW = Math.max(10, w - pad.l - pad.r);
      var plotH = Math.max(10, h - pad.t - pad.b);
      var yMax = computeYMax();
      var tMax = TOTAL_T > 0 ? TOTAL_T : 1;

      function xOf(t) { return pad.l + (t / tMax) * plotW; }
      function yOf(v) { return pad.t + plotH - (v / yMax) * plotH; }

      // Background grid
      ctx.fillStyle = "#0b1220";
      ctx.fillRect(pad.l, pad.t, plotW, plotH);

      ctx.strokeStyle = "#1e293b";
      ctx.lineWidth = 1;
      ctx.font = "10px ui-monospace, SFMono-Regular, Menlo, monospace";
      ctx.fillStyle = "#94a3b8";

      var ySteps = 4;
      for (var i = 0; i <= ySteps; i++) {
        var yv = (yMax * i) / ySteps;
        var yy = yOf(yv);
        ctx.beginPath();
        ctx.moveTo(pad.l, yy);
        ctx.lineTo(pad.l + plotW, yy);
        ctx.stroke();
        ctx.textAlign = "right";
        ctx.textBaseline = "middle";
        ctx.fillText(yv >= 100 ? String(Math.round(yv)) : fmt(yv, 0), pad.l - 5, yy);
      }

      var xSteps = 5;
      for (var j = 0; j <= xSteps; j++) {
        var tv = (TOTAL_T * j) / xSteps;
        var xx = xOf(tv);
        ctx.beginPath();
        ctx.moveTo(xx, pad.t);
        ctx.lineTo(xx, pad.t + plotH);
        ctx.stroke();
        ctx.textAlign = (j === 0) ? "left" : (j === xSteps ? "right" : "center");
        ctx.textBaseline = "top";
        ctx.fillText(fmt(tv, 1) + "s", xx, pad.t + plotH + 4);
      }

      // Draw series
      ctx.save();
      ctx.beginPath();
      ctx.rect(pad.l, pad.t, plotW, plotH);
      ctx.clip();

      seriesDefs.forEach(function(sd, idx) {
        if (!visible[idx] || N === 0) return;
        ctx.strokeStyle = sd.color;
        ctx.lineWidth = sd.width || 1.7;
        ctx.setLineDash(sd.dash ? [5, 3] : []);
        ctx.beginPath();
        for (var k = 0; k < N; k++) {
          var px = xOf(SER.t[k]);
          var py = yOf(sd.data[k]);
          if (k === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
        }
        ctx.stroke();
      });
      ctx.setLineDash([]);

      // Hover line
      if (hoverX !== null) {
        ctx.strokeStyle = "rgba(148,163,184,0.4)";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(hoverX, pad.t);
        ctx.lineTo(hoverX, pad.t + plotH);
        ctx.stroke();
      }

      // Active time cursor
      var cx = xOf(state.currentTime);
      ctx.strokeStyle = "#f8fafc";
      ctx.lineWidth = 1.6;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(cx, pad.t);
      ctx.lineTo(cx, pad.t + plotH);
      ctx.stroke();
      ctx.setLineDash([]);

      // Active dots on cursor
      var curS = sampleAtTime(state.currentTime);
      seriesDefs.forEach(function(sd, idx) {
        if (!visible[idx]) return;
        ctx.fillStyle = sd.color;
        ctx.beginPath();
        ctx.arc(cx, yOf(sd.sample(curS)), 3, 0, Math.PI * 2);
        ctx.fill();
      });

      ctx.restore();

      // Plot border
      ctx.strokeStyle = "#334155";
      ctx.lineWidth = 1;
      ctx.strokeRect(pad.l, pad.t, plotW, plotH);
    }

    return { draw: draw };
  }

  var posChart = TelemetryChart("canvas-pos", "wrap-pos", "legend-pos", "tip-pos", [
    { name: "X [deg]", short: "X", color: "#38bdf8", data: SER.x_deg, dec: 1, sample: function(s) { return s.x_deg; } },
    { name: "Y [mm]", short: "Y", color: "#34d399", data: SER.y_mm, dec: 1, sample: function(s) { return s.y_mm; } },
    { name: "Z [mm]", short: "Z", color: "#fbbf24", data: SER.z_mm, dec: 2, sample: function(s) { return s.z_mm; } },
    { name: "Valve [deg]", short: "Valve", color: "#fb7185", data: SER.valve_deg, dec: 1, dash: true, sample: function(s) { return s.valve_deg; } }
  ], "deg/mm");

  var spdChart = TelemetryChart("canvas-spd", "wrap-spd", "legend-spd", "tip-spd", [
    { name: "Surface Speed [mm/s]", short: "vSurf", color: "#f97316", width: 2.0, data: SER.surface_speed, dec: 1, sample: function(s) { return s.surface_speed; } },
    { name: "Planner Speed [u/s]", short: "vPlan", color: "#a78bfa", width: 1.6, data: SER.planner_speed, dec: 1, sample: function(s) { return s.planner_speed; } },
    { name: "|Vz| [mm/s]", short: "|Vz|", color: "#facc15", width: 1.4, data: SER.abs_vz, dec: 1, sample: function(s) { return s.abs_vz; } }
  ], "mm/s");

  // ---------------------------------------------------------------------------
  // 5. Bottom Dedicated 3D Legend Bar
  // ---------------------------------------------------------------------------
  var threeBridge = null;
  var legendDefs = [
    { key: "workpiece", label: "Workpiece 3D", kind: "box", color: "#94a3b8" },
    { key: "outlines", label: "Workpiece Outlines", kind: "line", color: "#cbd5e1" },
    { key: "rig", label: "Machine Rig (Base / Rail / Head)", kind: "box", color: "#3b82f6" },
    { key: "pathClosed", label: "Path (Valve CLOSED)", kind: "line", color: "#38bdf8" },
    { key: "pathOpen", label: "Path (Valve OPEN / Coating)", kind: "line", color: "#ff6b00" },
    { key: "nozzle", label: "Nozzle Tip & Z Drop Line", kind: "dot", color: "#38bdf8" },
    { key: "angleRay", label: "X=0° Mark / Active Angle Ray", kind: "line", color: "#f43f5e" }
  ];
  var legend3dEl = document.getElementById("legend-3d");
  var legendButtons = {};
  legendDefs.forEach(function(ld) {
    var btn = document.createElement("button");
    btn.type = "button";
    btn.className = "legend-item" + (state.layers[ld.key] ? "" : " off");
    var sw = "";
    if (ld.kind === "line") sw = '<span class="swatch-line" style="background:' + ld.color + '"></span>';
    else if (ld.kind === "box") sw = '<span class="swatch-box" style="background:' + ld.color + '"></span>';
    else sw = '<span class="swatch-dot" style="background:' + ld.color + '"></span>';
    btn.innerHTML = sw + "<span>" + ld.label + "</span>";
    btn.addEventListener("click", function() {
      state.layers[ld.key] = !state.layers[ld.key];
      syncLayerButtons();
      if (threeBridge) threeBridge.syncVisibility();
    });
    legend3dEl.appendChild(btn);
    legendButtons[ld.key] = btn;
  });

  function syncLayerButtons() {
    Object.keys(legendButtons).forEach(function(k) {
      legendButtons[k].classList.toggle("off", !state.layers[k]);
    });
    document.getElementById("btn-machine-rig").classList.toggle("active", state.layers.rig);
    document.getElementById("btn-labels").classList.toggle("active", state.layers.labels);
  }

  document.getElementById("btn-machine-rig").addEventListener("click", function() {
    state.layers.rig = !state.layers.rig;
    syncLayerButtons();
    if (threeBridge) threeBridge.syncVisibility();
  });
  document.getElementById("btn-labels").addEventListener("click", function() {
    state.layers.labels = !state.layers.labels;
    syncLayerButtons();
    if (threeBridge) threeBridge.syncVisibility();
  });

  function updateHud(st) {
    var hud = document.getElementById("hud-readout");
    var vCls = st.valve_open ? "hud-valve-open" : "hud-valve-closed";
    var vTxt = st.valve_open ? ("OPEN (" + fmt(st.valve_deg, 0) + "°)") : ("CLOSED (" + fmt(st.valve_deg, 0) + "°)");
    var modeTxt = state.mode === "machine" ? "Real Machine (Rotating)" : "Workpiece Fixed";
    hud.innerHTML =
      '<div class="hud-row">' +
        '<span><b>Mode:</b> ' + modeTxt + '</span>' +
        '<span><b>Valve:</b> <span class="' + vCls + '">' + vTxt + '</span></span>' +
      '</div>' +
      '<div class="hud-row">' +
        '<span><b>X:</b> ' + fmt(st.x_deg, 1) + '°</span>' +
        '<span><b>Y:</b> ' + fmt(st.y_mm, 1) + 'mm (r=' + fmt(st.r_mm, 1) + ')</span>' +
        '<span><b>Z:</b> ' + fmt(st.z_mm, 2) + 'mm</span>' +
        '<span><b>vSurf:</b> ' + fmt(st.surface_speed, 1) + 'mm/s</span>' +
      '</div>';
  }

  function onTimeUpdated() {
    var st = sampleAtTime(state.currentTime);
    updateHud(st);
    posChart.draw();
    spdChart.draw();
    if (threeBridge) threeBridge.updateAtState(st);
  }

  // ---------------------------------------------------------------------------
  // 6. Load Three.js UMD with CDN Fallbacks, then Init 3D Multi-View Engine
  // ---------------------------------------------------------------------------
  var CDN_URLS = [
    "https://cdn.jsdelivr.net/npm/three@0.147.0/build/three.min.js",
    "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js",
    "https://unpkg.com/three@0.147.0/build/three.min.js"
  ];

  function loadThree(idx, onReady, onFail) {
    if (window.THREE) { onReady(); return; }
    if (idx >= CDN_URLS.length) { onFail(); return; }
    var sc = document.createElement("script");
    sc.src = CDN_URLS[idx];
    sc.async = true;
    sc.onload = function() {
      if (window.THREE) onReady();
      else loadThree(idx + 1, onReady, onFail);
    };
    sc.onerror = function() { loadThree(idx + 1, onReady, onFail); };
    document.head.appendChild(sc);
  }

  function initThreeEngine() {
    var THREE = window.THREE;
    var host = document.getElementById("three-host");
    var rMax = SIM.r_center_mm;

    var renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: "high-performance" });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x0b1120, 1);
    host.insertBefore(renderer.domElement, host.firstChild);

    var scene = new THREE.Scene();

    // Lighting
    scene.add(new THREE.AmbientLight(0xdbeafe, 0.72));
    var dir1 = new THREE.DirectionalLight(0xffffff, 0.85);
    dir1.position.set(220, -280, 420);
    scene.add(dir1);
    var dir2 = new THREE.DirectionalLight(0x93c5fd, 0.42);
    dir2.position.set(-280, 240, 260);
    scene.add(dir2);

    // -------------------------------------------------------------------------
    // Static World Reference Floor & R-Z Side Scale Grid
    // -------------------------------------------------------------------------
    var worldRefGroup = new THREE.Group();
    scene.add(worldRefGroup);

    // Subtle polar grid at z = -0.2
    function makeCircleLine(radius, z, color, opacity, segs) {
      var pts = [];
      for (var i = 0; i <= segs; i++) {
        var a = (i / segs) * Math.PI * 2;
        pts.push(radius * Math.cos(a), radius * Math.sin(a), z);
      }
      var g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
      return new THREE.Line(g, new THREE.LineBasicMaterial({ color: color, transparent: true, opacity: opacity }));
    }
    worldRefGroup.add(makeCircleLine(rMax, 0.0, 0x94a3b8, 0.48, 128));
    worldRefGroup.add(makeCircleLine(rMax * 0.75, -0.1, 0x475569, 0.25, 96));
    worldRefGroup.add(makeCircleLine(rMax * 0.5, -0.1, 0x475569, 0.22, 96));
    worldRefGroup.add(makeCircleLine(rMax * 0.25, -0.1, 0x475569, 0.20, 64));

    // Faint crosshair axes on z=0
    (function() {
      var g = new THREE.BufferGeometry();
      var R = rMax + 16;
      g.setAttribute("position", new THREE.Float32BufferAttribute([
        -R, 0, -0.15,  R, 0, -0.15,
        0, -R, -0.15,  0, R, -0.15
      ], 3));
      worldRefGroup.add(new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: 0x334155, transparent: true, opacity: 0.45 })));
    })();

    // Side-view Z height reference lines (z = 10, 20, 30, 40, 50 mm along X axis at y = 0)
    var zGridGroup = new THREE.Group();
    (function() {
      var pts = [];
      for (var z = 10; z <= 50; z += 10) {
        pts.push(-rMax, 0, z, rMax, 0, z);
      }
      var g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
      zGridGroup.add(new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: 0x334155, transparent: true, opacity: 0.35 })));
    })();
    scene.add(zGridGroup);

    // -------------------------------------------------------------------------
    // Rotating Turntable + Workpiece + Toolpath Group (rotates by +X in Machine Mode)
    // -------------------------------------------------------------------------
    var turntableGroup = new THREE.Group();
    scene.add(turntableGroup);

    // Platter disk (top at z = 0, thickness 10 mm downward)
    var platterGroup = new THREE.Group();
    var platterGeo = new THREE.CylinderGeometry(rMax + 4, rMax + 6, 10, 72);
    platterGeo.rotateX(Math.PI / 2); // Align cylinder axis to Z
    var platterMesh = new THREE.Mesh(
      platterGeo,
      new THREE.MeshPhongMaterial({ color: 0x1e293b, shininess: 25, transparent: true, opacity: 0.85 })
    );
    platterMesh.position.z = -5.2;
    platterGroup.add(platterMesh);

    // Platter degree ticks (90, 180, 270 deg)
    (function() {
      var tickPts = [];
      [90, 180, 270].forEach(function(deg) {
        var rad = deg * Math.PI / 180;
        var c = Math.cos(rad), s = -Math.sin(rad);
        tickPts.push((rMax - 18) * c, (rMax - 18) * s, 0.25, rMax * c, rMax * s, 0.25);
      });
      var tg = new THREE.BufferGeometry();
      tg.setAttribute("position", new THREE.Float32BufferAttribute(tickPts, 3));
      platterGroup.add(new THREE.LineSegments(tg, new THREE.LineBasicMaterial({ color: 0x94a3b8, transparent: true, opacity: 0.7 })));
    })();
    turntableGroup.add(platterGroup);

    // Workpiece 0-degree reference mark on turntable (red radial bar along +X_wp)
    var zeroMarkGroup = new THREE.Group();
    (function() {
      var barGeo = new THREE.BoxGeometry(rMax, 3.2, 0.8);
      var barMesh = new THREE.Mesh(barGeo, new THREE.MeshBasicMaterial({ color: 0xf43f5e }));
      barMesh.position.set(rMax * 0.5, 0, 0.35);
      zeroMarkGroup.add(barMesh);
      var dotGeo = new THREE.CylinderGeometry(4.5, 4.5, 1.0, 24);
      dotGeo.rotateX(Math.PI / 2);
      var dotMesh = new THREE.Mesh(dotGeo, new THREE.MeshBasicMaterial({ color: 0xf43f5e }));
      dotMesh.position.set(0, 0, 0.4);
      zeroMarkGroup.add(dotMesh);
    })();
    turntableGroup.add(zeroMarkGroup);

    // 3D Workpiece Meshes
    var wpMeshGroup = new THREE.Group();
    (SIM.workpiece.meshes || []).forEach(function(m) {
      var geom = new THREE.BufferGeometry();
      geom.setAttribute("position", new THREE.Float32BufferAttribute(m.verts, 3));
      geom.setIndex(m.faces);
      geom.computeVertexNormals();
      var mat = new THREE.MeshPhongMaterial({
        color: new THREE.Color(m.color),
        transparent: true,
        opacity: m.opacity,
        flatShading: true,
        side: THREE.DoubleSide,
        shininess: 35,
        depthWrite: true
      });
      wpMeshGroup.add(new THREE.Mesh(geom, mat));
    });
    turntableGroup.add(wpMeshGroup);

    // 3D Workpiece Outlines
    var wpOutlineGroup = new THREE.Group();
    if (SIM.workpiece.wireframe_segments && SIM.workpiece.wireframe_segments.length > 0) {
      var wg = new THREE.BufferGeometry();
      wg.setAttribute("position", new THREE.Float32BufferAttribute(SIM.workpiece.wireframe_segments, 3));
      wpOutlineGroup.add(new THREE.LineSegments(
        wg,
        new THREE.LineBasicMaterial({ color: 0xe2e8f0, transparent: true, opacity: 0.62 })
      ));
    }
    turntableGroup.add(wpOutlineGroup);

    // 3D Workpiece Labels (Sprites always face camera)
    var wpLabelGroup = new THREE.Group();
    function makeTextSprite(text) {
      var c = document.createElement("canvas");
      c.width = 256;
      c.height = 64;
      var cx = c.getContext("2d");
      cx.font = "bold 26px -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif";
      var tw = cx.measureText(text).width;
      var bw = Math.min(248, tw + 24);
      cx.fillStyle = "rgba(15, 23, 42, 0.72)";
      cx.strokeStyle = "rgba(148, 163, 184, 0.45)";
      cx.lineWidth = 2;
      cx.beginPath();
      var rx = (256 - bw) / 2, ry = 10, rh = 44, rr = 10;
      cx.moveTo(rx + rr, ry);
      cx.lineTo(rx + bw - rr, ry);
      cx.quadraticCurveTo(rx + bw, ry, rx + bw, ry + rr);
      cx.lineTo(rx + bw, ry + rh - rr);
      cx.quadraticCurveTo(rx + bw, ry + rh, rx + bw - rr, ry + rh);
      cx.lineTo(rx + rr, ry + rh);
      cx.quadraticCurveTo(rx, ry + rh, rx, ry + rh - rr);
      cx.lineTo(rx, ry + rr);
      cx.quadraticCurveTo(rx, ry, rx + rr, ry);
      cx.closePath();
      cx.fill();
      cx.stroke();
      cx.fillStyle = "#f1f5f9";
      cx.textAlign = "center";
      cx.textBaseline = "middle";
      cx.fillText(text, 128, 33);
      var tex = new THREE.CanvasTexture(c);
      var sm = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
      var sp = new THREE.Sprite(sm);
      sp.scale.set(48, 12, 1);
      sp.renderOrder = 10;
      return sp;
    }
    (SIM.workpiece.labels || []).forEach(function(lb) {
      var sp = makeTextSprite(lb.text);
      sp.position.set(lb.x, lb.y, lb.z + 2.5);
      wpLabelGroup.add(sp);
    });
    turntableGroup.add(wpLabelGroup);

    // -------------------------------------------------------------------------
    // Toolpath Lines inside turntableGroup (rotate WITH workpiece in Machine Mode!)
    // Split into Valve CLOSED (cyan) & Valve OPEN (thick luminous orange ribbon)
    // Plus dynamic passed-progress highlight!
    // -------------------------------------------------------------------------
    function buildSegmentGeometry(pts, keepMask) {
      var seg = [];
      for (var i = 0; i < pts.length - 1; i++) {
        if (!keepMask[i] && !keepMask[i + 1]) continue;
        seg.push(
          pts[i][0], pts[i][1], pts[i][2],
          pts[i + 1][0], pts[i + 1][1], pts[i + 1][2]
        );
      }
      var g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.Float32BufferAttribute(seg, 3));
      return g;
    }

    // Thick 3D prism tube for Valve OPEN coating segments (visible thickness on all GPUs)
    function buildThickSegmentsMesh(pts, openMask, radius, colorHex, opacity) {
      var verts = [];
      var indices = [];
      var segSampleIdx = []; // maps each segment to its time index for live progress
      var vCount = 0;
      var upVec = new THREE.Vector3(0, 0, 1);
      var altUp = new THREE.Vector3(0, 1, 0);
      var p1 = new THREE.Vector3(), p2 = new THREE.Vector3();
      var dir = new THREE.Vector3(), u = new THREE.Vector3(), v = new THREE.Vector3();

      for (var i = 0; i < pts.length - 1; i++) {
        if (!openMask[i] && !openMask[i + 1]) continue;
        p1.set(pts[i][0], pts[i][1], pts[i][2]);
        p2.set(pts[i + 1][0], pts[i + 1][1], pts[i + 1][2]);
        dir.subVectors(p2, p1);
        if (dir.lengthSq() < 1e-6) continue;
        dir.normalize();
        var ref = Math.abs(dir.z) < 0.92 ? upVec : altUp;
        u.crossVectors(dir, ref).normalize().multiplyScalar(radius);
        v.crossVectors(dir, u).normalize().multiplyScalar(radius);

        // 4-sided tube segment (8 vertices, 8 triangles = 24 indices)
        var ringOffsets = [
          [ u.x,  u.y,  u.z],
          [ v.x,  v.y,  v.z],
          [-u.x, -u.y, -u.z],
          [-v.x, -v.y, -v.z]
        ];
        var base = vCount;
        for (var r = 0; r < 4; r++) {
          verts.push(p1.x + ringOffsets[r][0], p1.y + ringOffsets[r][1], p1.z + ringOffsets[r][2]);
        }
        for (var r2 = 0; r2 < 4; r2++) {
          verts.push(p2.x + ringOffsets[r2][0], p2.y + ringOffsets[r2][1], p2.z + ringOffsets[r2][2]);
        }
        for (var s = 0; s < 4; s++) {
          var sn = (s + 1) % 4;
          var a = base + s, b = base + sn, c = base + 4 + sn, d = base + 4 + s;
          indices.push(a, b, c, a, c, d);
        }
        vCount += 8;
        segSampleIdx.push(i);
      }

      var g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.Float32BufferAttribute(verts, 3));
      g.setIndex(indices);
      var mat = new THREE.MeshBasicMaterial({
        color: colorHex,
        transparent: opacity < 1,
        opacity: opacity,
        side: THREE.DoubleSide
      });
      var mesh = new THREE.Mesh(g, mat);
      mesh.renderOrder = 6;
      return { mesh: mesh, segSampleIdx: segSampleIdx };
    }

    var wpPts = [];
    var closedMask = [];
    var openMask = [];
    for (var i = 0; i < N; i++) {
      wpPts.push([SER.x_wp_mm[i], SER.y_wp_mm[i], SER.z_mm[i]]);
      var isOp = !!SER.valve_open[i];
      openMask.push(isOp);
      closedMask.push(!isOp);
    }

    // Full path preview (dimmed) + Live passed path (bright)
    var pathClosedGroup = new THREE.Group();
    var closedFullLine = new THREE.LineSegments(
      buildSegmentGeometry(wpPts, closedMask),
      new THREE.LineBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.36 })
    );
    closedFullLine.renderOrder = 4;
    pathClosedGroup.add(closedFullLine);

    // Live passed closed trail (updated via setDrawRange)
    var closedPassedSegPts = [];
    var closedPassedIdxMap = [];
    for (var ci = 0; ci < N - 1; ci++) {
      if (!closedMask[ci] && !closedMask[ci + 1]) continue;
      closedPassedSegPts.push(
        wpPts[ci][0], wpPts[ci][1], wpPts[ci][2],
        wpPts[ci + 1][0], wpPts[ci + 1][1], wpPts[ci + 1][2]
      );
      closedPassedIdxMap.push(ci);
    }
    var closedPassedGeom = new THREE.BufferGeometry();
    closedPassedGeom.setAttribute("position", new THREE.Float32BufferAttribute(closedPassedSegPts, 3));
    closedPassedGeom.setDrawRange(0, 0);
    var closedPassedLine = new THREE.LineSegments(
      closedPassedGeom,
      new THREE.LineBasicMaterial({ color: 0x7dd3fc, transparent: true, opacity: 0.95 })
    );
    closedPassedLine.renderOrder = 5;
    pathClosedGroup.add(closedPassedLine);
    turntableGroup.add(pathClosedGroup);

    var pathOpenGroup = new THREE.Group();
    var openFullObj = buildThickSegmentsMesh(wpPts, openMask, 1.35, 0xff6b00, 0.32);
    var openPassedObj = buildThickSegmentsMesh(wpPts, openMask, 2.05, 0xff7b1a, 0.98);
    openPassedObj.mesh.geometry.setDrawRange(0, 0);
    pathOpenGroup.add(openFullObj.mesh);
    pathOpenGroup.add(openPassedObj.mesh);
    turntableGroup.add(pathOpenGroup);

    // -------------------------------------------------------------------------
    // Fixed Machine Gantry Rig (Visible in Real Machine Mode)
    // Matches ksp_machine.xml: Y carriage slides along +X world axis (r = 180 - Y)
    // -------------------------------------------------------------------------
    var gantryStaticGroup = new THREE.Group();
    scene.add(gantryStaticGroup);

    var railZ = 108; // Clean elevated rail above Z_max=47.25mm
    var pillarMat = new THREE.MeshPhongMaterial({ color: 0x334155, transparent: true, opacity: 0.55 });
    var railMat = new THREE.MeshPhongMaterial({ color: 0x94a3b8, transparent: true, opacity: 0.72, shininess: 60 });

    // Base pedestal ring under platter
    var baseGeo = new THREE.CylinderGeometry(rMax + 12, rMax + 16, 14, 64);
    baseGeo.rotateX(Math.PI / 2);
    var baseMesh = new THREE.Mesh(baseGeo, new THREE.MeshPhongMaterial({ color: 0x111827, transparent: true, opacity: 0.85 }));
    baseMesh.position.z = -18;
    gantryStaticGroup.add(baseMesh);

    // Right side gantry pillar at x = rMax + 34
    var pillarMesh = new THREE.Mesh(new THREE.BoxGeometry(18, 22, railZ + 28), pillarMat);
    pillarMesh.position.set(rMax + 34, 0, (railZ - 14) / 2);
    gantryStaticGroup.add(pillarMesh);

    // Overhead bridge beam spanning x in [-10, rMax + 42]
    var bridgeLen = rMax + 52;
    var bridgeMesh = new THREE.Mesh(new THREE.BoxGeometry(bridgeLen, 14, 10), railMat);
    bridgeMesh.position.set(-10 + bridgeLen / 2, 0, railZ + 9);
    gantryStaticGroup.add(bridgeMesh);

    // Y linear rail along x in [0, rMax]
    var yRailMesh = new THREE.Mesh(new THREE.BoxGeometry(rMax + 14, 6, 5), railMat);
    yRailMesh.position.set(rMax / 2, 0, railZ + 1);
    gantryStaticGroup.add(yRailMesh);

    // Y-axis nozzle travel guide line on z = 0 along +X axis (x in [0, rMax])
    (function() {
      var g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0.5, rMax, 0, 0.5], 3));
      gantryStaticGroup.add(new THREE.Line(
        g,
        new THREE.LineBasicMaterial({ color: 0x34d399, transparent: true, opacity: 0.65 })
      ));
    })();

    // Moving Y-carriage + Z-post (moves along X = r_mm in Machine Mode)
    var yCarriageGroup = new THREE.Group();
    var yBlock = new THREE.Mesh(
      new THREE.BoxGeometry(22, 22, 16),
      new THREE.MeshPhongMaterial({ color: 0x0284c7, transparent: true, opacity: 0.82 })
    );
    yBlock.position.set(0, 0, railZ - 2);
    yCarriageGroup.add(yBlock);

    // Z vertical guide rod connecting carriage down toward nozzle head
    var zRod = new THREE.Mesh(
      new THREE.BoxGeometry(5, 5, 58),
      new THREE.MeshPhongMaterial({ color: 0x94a3b8, transparent: true, opacity: 0.65 })
    );
    zRod.position.set(0, 0, railZ - 32);
    yCarriageGroup.add(zRod);
    scene.add(yCarriageGroup);

    // -------------------------------------------------------------------------
    // Dynamic Nozzle Head + Tip + Valve Lever + Drop Line + Legacy Angle Ray
    // -------------------------------------------------------------------------
    var nozzleGroup = new THREE.Group();
    scene.add(nozzleGroup);

    // Nozzle tip cone (bottom tip exactly at local z = 0)
    var tipMat = new THREE.MeshPhongMaterial({ color: 0x38bdf8, emissive: 0x0c4a6e, shininess: 80 });
    var tipGeo = new THREE.ConeGeometry(4.2, 16, 20);
    tipGeo.rotateX(-Math.PI / 2); // Point apex downward (-Z)
    var tipMesh = new THREE.Mesh(tipGeo, tipMat);
    tipMesh.position.z = 8.0;
    nozzleGroup.add(tipMesh);

    // Glowing marker sphere right at exact tip (0,0,0) for precision reading
    var dotTipMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });
    var dotTip = new THREE.Mesh(new THREE.SphereGeometry(2.8, 16, 16), dotTipMat);
    dotTip.renderOrder = 8;
    nozzleGroup.add(dotTip);

    // Spray indicator ring when valve OPEN
    var sprayHaloMat = new THREE.MeshBasicMaterial({ color: 0xff6b00, transparent: true, opacity: 0.85, side: THREE.DoubleSide });
    var sprayHalo = new THREE.Mesh(new THREE.RingGeometry(4.5, 8.5, 24), sprayHaloMat);
    sprayHalo.position.z = 0.6;
    sprayHalo.visible = false;
    nozzleGroup.add(sprayHalo);

    // Upper nozzle manifold (shown in Machine Mode / Rig ON)
    var headRigGroup = new THREE.Group();
    var bodyGeo = new THREE.CylinderGeometry(9.5, 8.0, 26, 20);
    bodyGeo.rotateX(Math.PI / 2);
    var bodyMesh = new THREE.Mesh(
      bodyGeo,
      new THREE.MeshPhongMaterial({ color: 0x1d4ed8, transparent: true, opacity: 0.85 })
    );
    bodyMesh.position.z = 29;
    headRigGroup.add(bodyMesh);

    // Ball-valve servo lever (tilts with valve_deg)
    var valvePivot = new THREE.Group();
    valvePivot.position.set(0, 11, 29);
    var leverMesh = new THREE.Mesh(
      new THREE.BoxGeometry(18, 4, 4),
      new THREE.MeshPhongMaterial({ color: 0xf43f5e })
    );
    leverMesh.position.set(9, 0, 0);
    valvePivot.add(leverMesh);
    headRigGroup.add(valvePivot);
    nozzleGroup.add(headRigGroup);

    // Vertical Z drop line (yellow dashed from tip z=Z down to platter z=0)
    var dropLineGeom = new THREE.BufferGeometry();
    dropLineGeom.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 10], 3));
    var dropLine = new THREE.Line(
      dropLineGeom,
      new THREE.LineDashedMaterial({ color: 0xfacc15, dashSize: 3, gapSize: 2 })
    );
    dropLine.renderOrder = 7;
    scene.add(dropLine);

    // Legacy active X angle ray (used in Workpiece-Fixed Mode)
    var fixedRayGeom = new THREE.BufferGeometry();
    fixedRayGeom.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0.5, rMax, 0, 0.5], 3));
    var fixedRay = new THREE.Line(
      fixedRayGeom,
      new THREE.LineBasicMaterial({ color: 0xf43f5e })
    );
    fixedRay.renderOrder = 7;
    scene.add(fixedRay);

    // -------------------------------------------------------------------------
    // Cameras & Custom Smooth Orbit/Pan/Zoom Controller (Zero External Deps)
    // -------------------------------------------------------------------------
    var mainPersp = new THREE.PerspectiveCamera(42, 1, 1, 3000);
    mainPersp.up.set(0, 0, 1);

    var mainOrtho = new THREE.OrthographicCamera(-220, 220, 220, -220, 1, 3000);
    mainOrtho.up.set(0, 0, 1);

    // Sub-cameras for Multi-View (Top & Side Orthographic)
    var topCam = new THREE.OrthographicCamera(-215, 215, 215, -215, 1, 2000);
    topCam.up.set(0, 1, 0);
    topCam.position.set(0, 0, 600);
    topCam.lookAt(0, 0, 0);

    var sideCam = new THREE.OrthographicCamera(-220, 220, 100, -100, 1, 2000);
    sideCam.up.set(0, 0, 1);
    sideCam.position.set(15, -600, 32);
    sideCam.lookAt(15, 0, 32);

    // Spherical orbit state around target (Z-up)
    var orbit = {
      isOrtho: false,
      target: new THREE.Vector3(12, 0, 22),
      radius: 610,
      theta: -Math.PI / 2,  // azimuth around Z (-90 deg => looking from -Y toward +Y like Worker-Image.jpeg)
      phi: 0.96,            // polar angle from +Z axis
      orthoHalfH: 225,
      orthoMinHalfW: 225
    };

    function applyCameraPreset(preset) {
      state.camPreset = preset;
      document.querySelectorAll("[data-cam]").forEach(function(b) {
        b.classList.toggle("active", b.getAttribute("data-cam") === preset);
      });
      if (preset === "iso") {
        orbit.isOrtho = false;
        orbit.target.set(14, 0, 22);
        orbit.radius = 610;
        orbit.theta = -Math.PI / 2;
        orbit.phi = 0.96;
      } else if (preset === "top") {
        orbit.isOrtho = true;
        orbit.target.set(8, 0, 0);
        orbit.radius = 650;
        orbit.theta = -Math.PI / 2;
        orbit.phi = 0.001;
        orbit.orthoHalfH = rMax + 46;
        orbit.orthoMinHalfW = rMax + 52;
      } else if (preset === "side") {
        orbit.isOrtho = true;
        orbit.target.set(20, 0, 34);
        orbit.radius = 650;
        orbit.theta = -Math.PI / 2;
        orbit.phi = Math.PI / 2;
        orbit.orthoHalfH = 95;
        orbit.orthoMinHalfW = rMax + 58;
      } else if (preset === "front") {
        orbit.isOrtho = false;
        orbit.target.set(0, 0, 26);
        orbit.radius = 580;
        orbit.theta = 0.0;
        orbit.phi = 1.25;
      } else if (preset === "nozzle") {
        orbit.isOrtho = false;
        orbit.radius = 185;
        orbit.theta = -Math.PI / 2 + 0.35;
        orbit.phi = 1.12;
      }
      updateMainCameraMatrix(sampleAtTime(state.currentTime));
    }

    function updateMainCameraMatrix(st) {
      if (state.camPreset === "nozzle") {
        if (state.mode === "machine") {
          orbit.target.set(st.r_mm, 0, Math.max(8, st.z_mm * 0.7));
        } else {
          orbit.target.set(st.x_wp, st.y_wp, Math.max(8, st.z_mm * 0.7));
        }
      }
      var sinPhi = Math.sin(orbit.phi);
      var cosPhi = Math.cos(orbit.phi);
      var ex = orbit.target.x + orbit.radius * sinPhi * Math.cos(orbit.theta);
      var ey = orbit.target.y + orbit.radius * sinPhi * Math.sin(orbit.theta);
      var ez = orbit.target.z + orbit.radius * cosPhi;

      var cam = orbit.isOrtho ? mainOrtho : mainPersp;
      if (orbit.phi < 0.02) {
        cam.up.set(0, 1, 0);
      } else {
        cam.up.set(0, 0, 1);
      }
      cam.position.set(ex, ey, ez);
      cam.lookAt(orbit.target);
    }

    // Mouse / wheel interactions on main viewport
    var dragMode = null; // "rotate" | "pan"
    var lastMx = 0, lastMy = 0;

    renderer.domElement.addEventListener("contextmenu", function(e) { e.preventDefault(); });
    renderer.domElement.addEventListener("mousedown", function(e) {
      // Allow clicking only inside main viewport area when multi-view is on
      var rect = renderer.domElement.getBoundingClientRect();
      var mx = e.clientX - rect.left;
      var my = e.clientY - rect.top;
      if (state.multiView && mx > rect.width * 0.64) {
        // Clicking subviews focuses that preset
        if (my < rect.height * 0.5) applyCameraPreset("top");
        else applyCameraPreset("side");
        return;
      }
      dragMode = (e.button === 2 || e.shiftKey) ? "pan" : "rotate";
      lastMx = e.clientX;
      lastMy = e.clientY;
    });
    window.addEventListener("mousemove", function(e) {
      if (!dragMode) return;
      var dx = e.clientX - lastMx;
      var dy = e.clientY - lastMy;
      lastMx = e.clientX;
      lastMy = e.clientY;

      if (dragMode === "rotate") {
        // Switching out of strict top/side ortho into perspective on rotate feels natural
        if (orbit.isOrtho) {
          orbit.isOrtho = false;
          orbit.radius = Math.max(200, orbit.orthoHalfH * 2.2);
          if (orbit.phi < 0.05) orbit.phi = 0.25;
        }
        orbit.theta -= dx * 0.0075;
        orbit.phi = clamp(orbit.phi - dy * 0.0075, 0.04, Math.PI - 0.06);
        if (state.camPreset !== "nozzle") {
          document.querySelectorAll("[data-cam]").forEach(function(b) { b.classList.remove("active"); });
        }
      } else if (dragMode === "pan") {
        var cam = orbit.isOrtho ? mainOrtho : mainPersp;
        var panScale = orbit.isOrtho ? (orbit.orthoHalfH / 220) : (orbit.radius / 450);
        var forward = new THREE.Vector3().subVectors(orbit.target, cam.position).normalize();
        var right = new THREE.Vector3().crossVectors(forward, cam.up).normalize();
        var up = new THREE.Vector3().crossVectors(right, forward).normalize();
        orbit.target.addScaledVector(right, -dx * panScale * 0.75);
        orbit.target.addScaledVector(up, dy * panScale * 0.75);
      }
      updateMainCameraMatrix(sampleAtTime(state.currentTime));
    });
    window.addEventListener("mouseup", function() { dragMode = null; });
    renderer.domElement.addEventListener("wheel", function(e) {
      e.preventDefault();
      var factor = Math.exp(e.deltaY * 0.0012);
      if (orbit.isOrtho) {
        orbit.orthoHalfH = clamp(orbit.orthoHalfH * factor, 35, 480);
        orbit.orthoMinHalfW = clamp((orbit.orthoMinHalfW || 220) * factor, 45, 560);
      } else {
        orbit.radius = clamp(orbit.radius * factor, 75, 1100);
      }
      updateMainCameraMatrix(sampleAtTime(state.currentTime));
    }, { passive: false });
    renderer.domElement.addEventListener("dblclick", function() {
      applyCameraPreset("iso");
    });

    // Helper: count passed segments up to current sample index via binary search
    function countPassed(idxMap, curIdx) {
      var lo = 0, hi = idxMap.length;
      while (lo < hi) {
        var mid = (lo + hi) >> 1;
        if (idxMap[mid] < curIdx) lo = mid + 1;
        else hi = mid;
      }
      return lo;
    }

    function syncVisibility() {
      var isMachine = (state.mode === "machine");
      wpMeshGroup.visible = state.layers.workpiece;
      wpOutlineGroup.visible = state.layers.outlines;
      wpLabelGroup.visible = state.layers.labels;
      pathClosedGroup.visible = state.layers.pathClosed;
      pathOpenGroup.visible = state.layers.pathOpen;
      nozzleGroup.visible = state.layers.nozzle;
      dropLine.visible = state.layers.nozzle;

      platterGroup.visible = state.layers.rig;
      gantryStaticGroup.visible = isMachine && state.layers.rig;
      yCarriageGroup.visible = isMachine && state.layers.rig;
      headRigGroup.visible = isMachine && state.layers.rig;

      // In Machine mode, red 0-deg bar rotates WITH platter; in Fixed mode, active ray sweeps angle X
      zeroMarkGroup.visible = isMachine && state.layers.angleRay;
      fixedRay.visible = (!isMachine) && state.layers.angleRay;
    }

    function updateAtState(st) {
      var rad = st.x_deg * Math.PI / 180;
      if (state.mode === "machine") {
        // Real machine kinematics: platter + workpiece rotate +X (CCW around +Z),
        // nozzle moves strictly along +X_world (r = 180 - Y) and Z
        turntableGroup.rotation.z = rad;
        nozzleGroup.position.set(st.r_mm, 0, st.z_mm);
        yCarriageGroup.position.set(st.r_mm, 0, 0);
        // Drop line from nozzle tip to z=0
        var dp = dropLine.geometry.attributes.position.array;
        dp[0] = st.r_mm; dp[1] = 0; dp[2] = 0;
        dp[3] = st.r_mm; dp[4] = 0; dp[5] = st.z_mm;
        dropLine.geometry.attributes.position.needsUpdate = true;
        dropLine.computeLineDistances();
      } else {
        // Legacy Workpiece-Fixed Mode
        turntableGroup.rotation.z = 0;
        nozzleGroup.position.set(st.x_wp, st.y_wp, st.z_mm);
        var dp2 = dropLine.geometry.attributes.position.array;
        dp2[0] = st.x_wp; dp2[1] = st.y_wp; dp2[2] = 0;
        dp2[3] = st.x_wp; dp2[4] = st.y_wp; dp2[5] = st.z_mm;
        dropLine.geometry.attributes.position.needsUpdate = true;
        dropLine.computeLineDistances();

        var rp = fixedRay.geometry.attributes.position.array;
        rp[3] = rMax * Math.cos(rad);
        rp[4] = -rMax * Math.sin(rad);
        fixedRay.geometry.attributes.position.needsUpdate = true;
      }

      // Nozzle color & valve lever
      var col = st.valve_open ? 0xff6b00 : 0x38bdf8;
      tipMat.color.setHex(col);
      tipMat.emissive.setHex(st.valve_open ? 0x9a3412 : 0x0c4a6e);
      dotTipMat.color.setHex(col);
      sprayHalo.visible = st.valve_open;
      valvePivot.rotation.y = -(st.valve_deg * Math.PI / 180);

      // Live passed trajectory draw ranges
      var cCount = countPassed(closedPassedIdxMap, st.idx);
      closedPassedGeom.setDrawRange(0, cCount * 2);
      var oCount = countPassed(openPassedObj.segSampleIdx, st.idx);
      openPassedObj.mesh.geometry.setDrawRange(0, oCount * 24);

      if (state.camPreset === "nozzle") {
        updateMainCameraMatrix(st);
      }
    }

    // Multi-View overlay badges & dividers
    var splitV = document.getElementById("split-v");
    var splitH = document.getElementById("split-h");
    var badgeTop = document.getElementById("badge-top");
    var badgeSide = document.getElementById("badge-side");

    badgeTop.addEventListener("click", function() { applyCameraPreset("top"); });
    badgeSide.addEventListener("click", function() { applyCameraPreset("side"); });

    function updateSplitOverlays(w, h) {
      if (state.multiView && w >= 520) {
        var mw = Math.floor(w * 0.64);
        var hh = Math.floor(h * 0.5);
        splitV.style.display = "block";
        splitV.style.left = mw + "px";
        splitH.style.display = "block";
        splitH.style.left = mw + "px";
        splitH.style.top = hh + "px";
        badgeTop.style.display = "block";
        badgeTop.style.left = (mw + 8) + "px";
        badgeTop.style.top = "8px";
        badgeSide.style.display = "block";
        badgeSide.style.left = (mw + 8) + "px";
        badgeSide.style.top = (hh + 8) + "px";
      } else {
        splitV.style.display = "none";
        splitH.style.display = "none";
        badgeTop.style.display = "none";
        badgeSide.style.display = "none";
      }
    }

    function renderAll() {
      var rect = host.getBoundingClientRect();
      var w = Math.max(100, Math.floor(rect.width));
      var h = Math.max(100, Math.floor(rect.height));
      if (renderer.domElement.width !== Math.floor(w * renderer.getPixelRatio()) ||
          renderer.domElement.height !== Math.floor(h * renderer.getPixelRatio())) {
        renderer.setSize(w, h, false);
      }
      updateSplitOverlays(w, h);

      var useMulti = state.multiView && w >= 520;
      renderer.setScissorTest(true);

      // 1) Main viewport
      var mw = useMulti ? Math.floor(w * 0.64) : w;
      var mh = h;
      renderer.setViewport(0, 0, mw, mh);
      renderer.setScissor(0, 0, mw, mh);
      var mainCam = orbit.isOrtho ? mainOrtho : mainPersp;
      var mainAspect = mw / Math.max(1, mh);
      var prevLabelsMain = wpLabelGroup.visible;
      if (orbit.isOrtho) {
        var hh = Math.max(orbit.orthoHalfH, (orbit.orthoMinHalfW || 220) / Math.max(0.35, mainAspect));
        mainOrtho.left = -hh * mainAspect;
        mainOrtho.right = hh * mainAspect;
        mainOrtho.top = hh;
        mainOrtho.bottom = -hh;
        mainOrtho.updateProjectionMatrix();
      } else {
        // Keep full workpiece width comfortably inside narrow viewports
        var baseFov = 42;
        mainPersp.fov = mainAspect < 1.05 ? Math.min(58, baseFov / Math.max(0.72, mainAspect * 0.92)) : baseFov;
        mainPersp.aspect = mainAspect;
        mainPersp.updateProjectionMatrix();
      }
      zGridGroup.visible = (state.camPreset === "side");
      if (state.camPreset === "side") wpLabelGroup.visible = false;
      renderer.render(scene, mainCam);
      wpLabelGroup.visible = prevLabelsMain;

      if (useMulti) {
        var sw = w - mw;
        var shTop = Math.floor(h * 0.5);
        var shBot = h - shTop;
        var prevLabels = wpLabelGroup.visible;

        // 2) Top-right: Top View (Orthographic)
        renderer.setViewport(mw, shBot, sw, shTop);
        renderer.setScissor(mw, shBot, sw, shTop);
        var topAspect = sw / Math.max(1, shTop);
        var topSpan = rMax + 34;
        var topH = topAspect < 1 ? (topSpan / topAspect) : topSpan;
        topCam.left = -topH * topAspect;
        topCam.right = topH * topAspect;
        topCam.top = topH;
        topCam.bottom = -topH;
        topCam.position.set(10, 0, 600);
        topCam.lookAt(10, 0, 0);
        topCam.updateProjectionMatrix();
        zGridGroup.visible = false;
        wpLabelGroup.visible = false; // Keep sub-views crisp and uncluttered
        renderer.render(scene, topCam);

        // 3) Bottom-right: Side R-Z View (Orthographic)
        renderer.setViewport(mw, 0, sw, shBot);
        renderer.setScissor(mw, 0, sw, shBot);
        var sideAspect = sw / Math.max(1, shBot);
        var sideHalfW = rMax + 58;
        var sideHalfH = Math.max(76, sideHalfW / Math.max(0.4, sideAspect));
        sideCam.left = -sideHalfH * sideAspect;
        sideCam.right = sideHalfH * sideAspect;
        sideCam.top = sideHalfH;
        sideCam.bottom = -sideHalfH;
        sideCam.position.set(20, -600, 36);
        sideCam.lookAt(20, 0, 36);
        sideCam.updateProjectionMatrix();
        zGridGroup.visible = true;
        wpLabelGroup.visible = false;
        renderer.render(scene, sideCam);

        wpLabelGroup.visible = prevLabels;
      }

      renderer.setScissorTest(false);
    }

    threeBridge = {
      syncVisibility: syncVisibility,
      updateAtState: updateAtState,
      applyCameraPreset: applyCameraPreset,
      renderAll: renderAll
    };

    // Wire toolbar controls
    document.getElementById("mode-machine").addEventListener("click", function() {
      state.mode = "machine";
      document.getElementById("mode-machine").classList.add("active");
      document.getElementById("mode-fixed").classList.remove("active");
      syncVisibility();
      onTimeUpdated();
    });
    document.getElementById("mode-fixed").addEventListener("click", function() {
      state.mode = "fixed";
      document.getElementById("mode-fixed").classList.add("active");
      document.getElementById("mode-machine").classList.remove("active");
      syncVisibility();
      onTimeUpdated();
    });
    document.querySelectorAll("[data-cam]").forEach(function(b) {
      b.addEventListener("click", function() {
        applyCameraPreset(b.getAttribute("data-cam"));
      });
    });
    document.getElementById("btn-multiview").addEventListener("click", function(e) {
      state.multiView = !state.multiView;
      e.currentTarget.classList.toggle("active", state.multiView);
    });

    // Initial state
    applyCameraPreset("iso");
    syncVisibility();
    syncLayerButtons();
    updatePlaybackUI();
    onTimeUpdated();

    // 60fps animation loop
    var lastTs = performance.now();
    function tick(now) {
      var dt = Math.min(0.1, Math.max(0, (now - lastTs) / 1000));
      lastTs = now;
      if (state.playing && TOTAL_T > 0) {
        state.currentTime += dt * state.speed;
        if (state.currentTime >= TOTAL_T) {
          if (state.loop) {
            state.currentTime = state.currentTime % TOTAL_T;
          } else {
            state.currentTime = TOTAL_T;
            state.playing = false;
          }
        }
        updatePlaybackUI();
        onTimeUpdated();
      }
      renderAll();
      requestAnimationFrame(tick);
    }
    requestAnimationFrame(tick);
  }

  // Fallback message if completely offline and Three.js CDN cannot be reached
  function onThreeFailed() {
    var msg = document.getElementById("fallback-msg");
    msg.style.display = "flex";
    msg.innerHTML =
      '<div><h3 style="margin:0 0 8px;color:#f8fafc;">Unable to load 3D engine (Three.js)</h3>' +
      '<p style="margin:0;color:#94a3b8;line-height:1.6;">Please reload this page with an internet connection.<br>' +
      'The telemetry charts and timeline slider remain interactive offline.</p></div>';
    updatePlaybackUI();
    onTimeUpdated();
  }

  window.addEventListener("resize", function() {
    posChart.draw();
    spdChart.draw();
    if (threeBridge) threeBridge.renderAll();
  });

  // Render telemetry immediately, then boot Three.js
  updatePlaybackUI();
  onTimeUpdated();
  loadThree(0, initThreeEngine, onThreeFailed);
})();
</script>
</body>
</html>
"""


def generate_interactive_html(
    sim: SimulationResult,
    out_html: Path,
    mujoco_report: dict[str, Any] | None = None,
    show_workpiece: bool = True,
    workpiece_spec: WorkpieceSpec = DEFAULT_WORKPIECE,
    max_frames: int = 160,  # kept for API compatibility
) -> Path:
    """Generate a self-contained interactive 3D Three.js multi-view + telemetry HTML."""
    del max_frames  # Continuous 60fps interpolation replaces discrete Plotly frames
    out_html.parent.mkdir(parents=True, exist_ok=True)
    payload = _build_html_payload(sim, mujoco_report, show_workpiece, workpiece_spec)
    payload_json = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).replace("</", "<\\/")
    title_text = html_lib.escape(f"KSP-GCODE Simulator — {sim.file_name}")
    html_out = (
        _HTML_TEMPLATE
        .replace("__TITLE__", title_text)
        .replace("__PAYLOAD_JSON__", payload_json)
    )
    out_html.write_text(html_out, encoding="utf-8")
    return out_html



def _draw_workpiece_2d_footprint(ax: Any, spec: WorkpieceSpec) -> None:
    """Draw clean, low-clutter CAD blueprint footprint of the workpiece under the 2D path."""
    from matplotlib.patches import Circle, Wedge

    # Dark cast housing body
    ax.add_patch(
        Wedge((0, 0), spec.body_r_out_mm, 0, 360, width=spec.body_r_out_mm - spec.body_r_in_mm,
              facecolor="#334155", edgecolor="#64748b", lw=0.8, alpha=0.45, zorder=1)
    )
    # Bright silver inner machined ring
    ax.add_patch(
        Wedge((0, 0), spec.inner_ring_r_out_mm, 0, 360, width=spec.inner_ring_r_out_mm - spec.inner_ring_r_in_mm,
              facecolor="#cbd5e1", edgecolor="#e2e8f0", lw=0.9, alpha=0.35, zorder=1)
    )
    # Inner bore center hole
    ax.add_patch(Circle((0, 0), spec.inner_step_r_in_mm, facecolor="#0f172a", edgecolor="#475569", lw=0.7, alpha=0.7, zorder=1))

    # 5 outer bolt bosses (C1-C5)
    for b in spec.bolt_bosses:
        cx, cy = polar_to_wp_xy(b.r_mm, b.theta_deg)
        ax.add_patch(Circle((cx, cy), b.outer_r_mm, facecolor="#94a3b8", edgecolor="#cbd5e1", lw=0.9, alpha=0.40, zorder=2))
        ax.add_patch(Circle((cx, cy), b.hole_r_mm, facecolor="#0f172a", edgecolor="#475569", lw=0.6, alpha=0.85, zorder=2))
        lx, ly = polar_to_wp_xy(b.r_mm + 21.0, b.theta_deg)
        ax.text(lx, ly, b.label, color="#94a3b8", fontsize=7, ha="center", va="center", zorder=3)

    # Raised pads (CW angles -> matplotlib Wedge uses CCW angles, so [-theta_end, -theta_start])
    for pad in spec.raised_pads:
        ax.add_patch(
            Wedge((0, 0), pad.r_out_mm, -pad.theta_end_deg, -pad.theta_start_deg,
                  width=pad.r_out_mm - pad.r_in_mm,
                  facecolor="#e2e8f0", edgecolor="#f8fafc", lw=1.0, alpha=0.35, zorder=2)
        )
        r_mid = 0.5 * (pad.r_in_mm + pad.r_out_mm)
        for bang in pad.bolt_angles_deg:
            hx, hy = polar_to_wp_xy(r_mid, bang)
            ax.add_patch(Circle((hx, hy), 4.8, facecolor="#0f172a", edgecolor="#64748b", lw=0.6, alpha=0.85, zorder=2))

    # Blue purge pad
    p = spec.purge_pad
    ax.add_patch(
        Wedge((0, 0), p.r_out_mm, -p.theta_end_deg, -p.theta_start_deg,
              width=p.r_out_mm - p.r_in_mm,
              facecolor="#3b82f6", edgecolor="#93c5fd", lw=1.0, alpha=0.38, zorder=2)
    )
    px, py = polar_to_wp_xy(p.r_out_mm + 12.0, 0.5 * (p.theta_start_deg + p.theta_end_deg))
    ax.text(px, py, "Purge", color="#93c5fd", fontsize=7, ha="center", va="center", zorder=3)


def generate_summary_png(
    sim: SimulationResult,
    out_png: Path,
    mujoco_report: dict[str, Any] | None = None,
    show_workpiece: bool = True,
    workpiece_spec: WorkpieceSpec = DEFAULT_WORKPIECE,
) -> Path:
    """Generate a static 3-panel Matplotlib PNG summary (top-down polar + workpiece, positions, speeds)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_png.parent.mkdir(parents=True, exist_ok=True)
    r_max = sim.config.r_center_mm

    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.8), dpi=135)
    fig.patch.set_facecolor("#0f172a")
    for ax in axes:
        ax.set_facecolor("#1e293b")
        ax.tick_params(colors="#cbd5e1", labelsize=8)
        for sp in ax.spines.values():
            sp.set_color("#475569")
        ax.title.set_color("#f8fafc")
        ax.xaxis.label.set_color("#cbd5e1")
        ax.yaxis.label.set_color("#cbd5e1")
        ax.grid(True, color="#334155", linewidth=0.55, alpha=0.65)

    # Panel 1: Top-down polar footprint WITH parametric workpiece underneath
    ax0 = axes[0]
    if show_workpiece:
        _draw_workpiece_2d_footprint(ax0, workpiece_spec)
    ang = np.linspace(0, 2 * np.pi, 200)
    ax0.plot(r_max * np.cos(ang), -r_max * np.sin(ang), color="#64748b", lw=1.1, ls="--", label=f"R={r_max:.0f}mm", zorder=2)
    ax0.plot([0, r_max], [0, 0], color="#f43f5e", lw=0.9, alpha=0.55, label="0° mark", zorder=3)
    # Continuous path line for readability + Z-colored scatter dots
    ax0.plot(sim.x_wp_mm, sim.y_wp_mm, color="#38bdf8", lw=1.1, alpha=0.45, zorder=4)
    sc = ax0.scatter(sim.x_wp_mm, sim.y_wp_mm, c=sim.z_mm, cmap="viridis", s=6, vmin=0, vmax=47.25, zorder=5)
    if np.any(sim.valve_open):
        ax0.scatter(
            sim.x_wp_mm[sim.valve_open],
            sim.y_wp_mm[sim.valve_open],
            color="#ff6b00",
            s=18,
            label="Valve OPEN",
            zorder=6,
        )
    ax0.set_aspect("equal", "box")
    ax0.set_xlim(-r_max - 20, r_max + 20)
    ax0.set_ylim(-r_max - 20, r_max + 20)
    ax0.set_xlabel("X_wp [mm]")
    ax0.set_ylabel("Y_wp [mm] (Purge at top = Worker-Image.jpeg)")
    ax0.set_title("Workpiece + Toolpath (color=Z mm)")
    cb = fig.colorbar(sc, ax=ax0, fraction=0.046, pad=0.04)
    cb.ax.tick_params(colors="#cbd5e1", labelsize=7)
    ax0.legend(loc="lower right", fontsize=7, facecolor="#0f172a", labelcolor="#e2e8f0")

    # Panel 2: Axis trajectories
    ax1 = axes[1]
    ax1.plot(sim.t, sim.x_deg, label="X [deg]", color="#38bdf8", lw=1.5)
    ax1.plot(sim.t, sim.y_mm, label="Y [mm]", color="#34d399", lw=1.5)
    ax1.plot(sim.t, sim.z_mm, label="Z [mm]", color="#fbbf24", lw=1.5)
    ax1.plot(sim.t, sim.valve_deg, label="Valve [deg]", color="#fb7185", lw=1.3, ls="--")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Position / Angle")
    ax1.set_title("Axis Positions & Valve Angle")
    ax1.legend(loc="best", fontsize=7, facecolor="#0f172a", labelcolor="#e2e8f0")

    # Panel 3: Velocities
    ax2 = axes[2]
    ax2.plot(sim.t, sim.surface_speed_mm_s, label="Surface speed [mm/s]", color="#f97316", lw=1.7)
    ax2.plot(sim.t, sim.planner_speed, label="Planner speed [units/s]", color="#a78bfa", lw=1.3)
    ax2.plot(sim.t, np.abs(sim.vz_mm_s), label="|Vz| [mm/s]", color="#facc15", lw=1.2)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Speed")
    ax2.set_title("Velocity Profile")
    ax2.legend(loc="best", fontsize=7, facecolor="#0f172a", labelcolor="#e2e8f0")

    s = sim.summary()
    title = (
        f"{sim.file_name} — Total {s['total_time_s']:.2f}s | "
        f"Z=[{s['z_range_mm'][0]:.2f}, {s['z_range_mm'][1]:.2f}] mm | "
        f"Peak Surface {s['peak_surface_speed_mm_s']:.1f} mm/s"
    )
    if mujoco_report is not None:
        wp_c = mujoco_report.get("min_workpiece_clearance_mm")
        if wp_c is not None:
            title += f" | Min Workpiece Clearance {wp_c:.2f} mm ({mujoco_report.get('min_workpiece_geom')})"
        else:
            title += f" | Min Bed Clearance {mujoco_report['min_bed_clearance_mm']:.2f} mm"
    fig.suptitle(title, color="#f8fafc", fontsize=10, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out_png


def generate_index_html(rows: list[dict[str, Any]], out_index: Path) -> Path:
    """Create a dashboard index.html linking all simulated G-code reports."""
    out_index.parent.mkdir(parents=True, exist_ok=True)
    tr_html = []
    for r in rows:
        mj = r.get("mujoco") or {}
        mj_badge = (
            '<span class="ok">OK</span>'
            if mj.get("collision_free", True)
            else '<span class="bad">COLLISION</span>'
        )
        warn_cnt = r["klipper"]["warning_count"]
        warn_badge = (
            '<span class="ok">0</span>'
            if warn_cnt == 0
            else f'<span class="warn">{warn_cnt}</span>'
        )
        html_link = f'<a href="{r["html_rel"]}">3D HTML</a>' if r.get("html_rel") else "-"
        png_link = f'<a href="{r["png_rel"]}">PNG</a>' if r.get("png_rel") else "-"
        csv_link = f'<a href="{r["csv_rel"]}">CSV</a>' if r.get("csv_rel") else "-"
        k = r["klipper"]
        wp_clr = mj.get("min_workpiece_clearance_mm")
        wp_str = f"{wp_clr:.2f} mm ({mj.get('min_workpiece_geom')})" if wp_clr is not None else "-"
        tr_html.append(
            f"<tr>"
            f"<td><code>{k['file_name']}</code></td>"
            f"<td>{k['total_time_s']:.2f}s</td>"
            f"<td>{k['move_count']}</td>"
            f"<td>{k['valve_open_events']}/{k['valve_close_events']}</td>"
            f"<td>[{k['z_range_mm'][0]:.2f}, {k['z_range_mm'][1]:.2f}]</td>"
            f"<td>{k['peak_surface_speed_mm_s']:.1f}</td>"
            f"<td>{mj.get('min_bed_clearance_mm', '-')} mm</td>"
            f"<td>{wp_str}</td>"
            f"<td>{mj_badge}</td>"
            f"<td>{warn_badge}</td>"
            f"<td>{html_link} | {png_link} | {csv_link}</td>"
            f"</tr>"
        )

    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <title>KSP-GCODE Local Simulation Dashboard (with 3D Workpiece)</title>
  <style>
    body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #0f172a; color: #e2e8f0; margin: 24px; }}
    h1 {{ margin-bottom: 6px; }}
    p.sub {{ color: #94a3b8; margin-top: 0; }}
    table {{ border-collapse: collapse; width: 100%; background: #1e293b; border-radius: 8px; overflow: hidden; }}
    th, td {{ padding: 10px 12px; border-bottom: 1px solid #334155; text-align: left; font-size: 13px; }}
    th {{ background: #0b1220; color: #93c5fd; }}
    a {{ color: #38bdf8; text-decoration: none; }}
    a:hover {{ text-decoration: underline; }}
    .ok {{ color: #4ade80; font-weight: 600; }}
    .warn {{ color: #fbbf24; font-weight: 600; }}
    .bad {{ color: #f87171; font-weight: 600; }}
  </style>
</head>
<body>
  <h1>KSP-GCODE Local OSS Simulation Dashboard</h1>
  <p class="sub">Klipper GCodeMove + LookAheadQueue + Parametric 3D Workpiece (Worker-Image.jpeg) + Three.js Real-Machine / Multi-View 3D HTML + MuJoCo Collision/Clearance Verification</p>
  <table>
    <thead>
      <tr>
        <th>G-code</th>
        <th>Total Time</th>
        <th>Moves</th>
        <th>Valve (O/C)</th>
        <th>Z Range [mm]</th>
        <th>Peak Surface [mm/s]</th>
        <th>Min Bed Clear</th>
        <th>Min Workpiece Clear</th>
        <th>MuJoCo Status</th>
        <th>Warnings</th>
        <th>Outputs</th>
      </tr>
    </thead>
    <tbody>
      {"".join(tr_html)}
    </tbody>
  </table>
</body>
</html>
"""
    out_index.write_text(html, encoding="utf-8")
    return out_index
