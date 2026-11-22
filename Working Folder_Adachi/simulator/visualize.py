"""Interactive 3D Plotly HTML animation & Matplotlib PNG telemetry generator.

Renders the parametric 3D workpiece (inspired by Worker-Image.jpeg) with
deliberate visual hierarchy: muted semi-transparent workpiece geometry + crisp
feature outlines underneath luminous cyan/orange toolpath trajectories so the
3D model adds realism WITHOUT obscuring any G-code path lines.
"""
from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path
from typing import Any

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

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


def _sample_frame_indices(n: int, max_frames: int = 160) -> np.ndarray:
    if n <= max_frames:
        return np.arange(n, dtype=int)
    return np.unique(np.linspace(0, n - 1, max_frames).round().astype(int))


def generate_interactive_html(
    sim: SimulationResult,
    out_html: Path,
    mujoco_report: dict[str, Any] | None = None,
    show_workpiece: bool = True,
    workpiece_spec: WorkpieceSpec = DEFAULT_WORKPIECE,
    max_frames: int = 160,
) -> Path:
    """Generate a self-contained interactive 3D polar animation + telemetry HTML."""
    out_html.parent.mkdir(parents=True, exist_ok=True)
    r_max = sim.config.r_center_mm

    # Reference turntable rim circle (z=0)
    ang_ref = np.linspace(0.0, 2.0 * np.pi, 121)
    rim_x = r_max * np.cos(ang_ref)
    rim_y = -r_max * np.sin(ang_ref)
    rim_z = np.zeros_like(ang_ref)

    # Full workpiece-frame trajectory split into valve-closed vs valve-open segments
    x_closed = np.where(~sim.valve_open, sim.x_wp_mm, np.nan)
    y_closed = np.where(~sim.valve_open, sim.y_wp_mm, np.nan)
    z_closed = np.where(~sim.valve_open, sim.z_mm, np.nan)

    x_open = np.where(sim.valve_open, sim.x_wp_mm, np.nan)
    y_open = np.where(sim.valve_open, sim.y_wp_mm, np.nan)
    z_open = np.where(sim.valve_open, sim.z_mm, np.nan)

    fig = make_subplots(
        rows=2,
        cols=2,
        specs=[
            [{"type": "scene", "rowspan": 2}, {"type": "xy"}],
            [None, {"type": "xy"}],
        ],
        column_widths=[0.60, 0.40],
        row_heights=[0.5, 0.5],
        subplot_titles=(
            f"3D Workpiece + G-code Trajectory ({sim.file_name})",
            "Axis Positions (X [deg], Y [mm], Z [mm]) & Valve [deg]",
            "Planner Speed vs. True Workpiece Surface Speed [mm/s]",
        ),
        horizontal_spacing=0.06,
        vertical_spacing=0.12,
    )

    # Track exact trace indices for dynamic animation frames
    # 1) Static reference: Turntable rim
    fig.add_trace(
        go.Scatter3d(
            x=rim_x, y=rim_y, z=rim_z,
            mode="lines",
            line=dict(color="rgba(148,163,184,0.45)", width=2),
            name=f"Turntable Rim (R={r_max:.0f}mm)",
            hoverinfo="skip",
        ),
        row=1, col=1,
    )

    # 2) Optional Semi-Transparent 3D Workpiece + Crisp Feature Outlines + Labels
    if show_workpiece:
        meshes = build_plotly_workpiece_meshes(workpiece_spec)
        for m_idx, m in enumerate(meshes):
            v, f = m["verts"], m["faces"]
            fig.add_trace(
                go.Mesh3d(
                    x=v[:, 0], y=v[:, 1], z=v[:, 2],
                    i=f[:, 0], j=f[:, 1], k=f[:, 2],
                    color=m["color"],
                    opacity=m["opacity"],
                    flatshading=True,
                    lighting=dict(ambient=0.65, diffuse=0.65, specular=0.15, roughness=0.7),
                    name="Workpiece 3D (toggle)",
                    legendgroup="workpiece_3d",
                    showlegend=(m_idx == 0),
                    hoverinfo="skip",
                ),
                row=1, col=1,
            )

        wx, wy, wz = build_workpiece_wireframe_lines(workpiece_spec)
        fig.add_trace(
            go.Scatter3d(
                x=wx, y=wy, z=wz,
                mode="lines",
                line=dict(color="rgba(226,232,240,0.48)", width=1.6),
                name="Workpiece Outlines",
                legendgroup="workpiece_outlines",
                hoverinfo="skip",
            ),
            row=1, col=1,
        )

        lbls = build_workpiece_labels(workpiece_spec)
        fig.add_trace(
            go.Scatter3d(
                x=[l[1] for l in lbls],
                y=[l[2] for l in lbls],
                z=[l[3] for l in lbls],
                mode="text",
                text=[l[0] for l in lbls],
                textfont=dict(color="rgba(226,232,240,0.82)", size=10),
                name="Feature Labels",
                legendgroup="workpiece_outlines",
                showlegend=False,
                hoverinfo="skip",
            ),
            row=1, col=1,
        )

    # 3) Luminous Toolpath Lines (drawn AFTER workpiece so lines stay visually dominant)
    fig.add_trace(
        go.Scatter3d(
            x=x_closed, y=y_closed, z=z_closed,
            mode="lines",
            line=dict(color="#38bdf8", width=4.5),
            name="Path (Valve CLOSED)",
        ),
        row=1, col=1,
    )

    fig.add_trace(
        go.Scatter3d(
            x=x_open, y=y_open, z=z_open,
            mode="lines",
            line=dict(color="#ff6b00", width=7.5),
            name="Path (Valve OPEN)",
        ),
        row=1, col=1,
    )

    # 4) Dynamic 3D traces (record exact trace indices!)
    nozzle_trace_idx = len(fig.data)
    fig.add_trace(
        go.Scatter3d(
            x=[sim.x_wp_mm[0]], y=[sim.y_wp_mm[0]], z=[sim.z_mm[0]],
            mode="markers",
            marker=dict(size=8, color="#38bdf8", symbol="diamond", line=dict(color="#ffffff", width=1.5)),
            name="Nozzle Tip (Active)",
        ),
        row=1, col=1,
    )

    drop_trace_idx = len(fig.data)
    fig.add_trace(
        go.Scatter3d(
            x=[sim.x_wp_mm[0], sim.x_wp_mm[0]],
            y=[sim.y_wp_mm[0], sim.y_wp_mm[0]],
            z=[0.0, sim.z_mm[0]],
            mode="lines",
            line=dict(color="#facc15", width=4, dash="dot"),
            name="Nozzle Z Drop Line",
        ),
        row=1, col=1,
    )

    spoke_trace_idx = len(fig.data)
    th0 = math.radians(float(sim.x_deg[0]))
    fig.add_trace(
        go.Scatter3d(
            x=[0.0, r_max * math.cos(th0)],
            y=[0.0, -r_max * math.sin(th0)],
            z=[0.0, 0.0],
            mode="lines",
            line=dict(color="#f43f5e", width=3.5),
            name="Active X Angle Ray",
        ),
        row=1, col=1,
    )

    # 5) Right-top subplot (row 1, col 2): X, Y, Z, Valve
    fig.add_trace(
        go.Scatter(x=sim.t, y=sim.x_deg, name="X [deg]", line=dict(color="#38bdf8", width=2)),
        row=1, col=2,
    )
    fig.add_trace(
        go.Scatter(x=sim.t, y=sim.y_mm, name="Y [mm]", line=dict(color="#34d399", width=2)),
        row=1, col=2,
    )
    fig.add_trace(
        go.Scatter(x=sim.t, y=sim.z_mm, name="Z [mm]", line=dict(color="#fbbf24", width=2)),
        row=1, col=2,
    )
    fig.add_trace(
        go.Scatter(x=sim.t, y=sim.valve_deg, name="Valve [deg]", line=dict(color="#fb7185", width=2, dash="dash")),
        row=1, col=2,
    )
    ymax_pos = max(50.0, float(np.max(sim.x_deg)), float(np.max(sim.y_mm)), float(np.max(sim.z_mm))) * 1.05
    pos_cursor_idx = len(fig.data)
    fig.add_trace(
        go.Scatter(
            x=[sim.t[0], sim.t[0]], y=[0.0, ymax_pos],
            mode="lines", line=dict(color="#ffffff", width=1.5, dash="dot"),
            showlegend=False, hoverinfo="skip",
        ),
        row=1, col=2,
    )

    # 6) Right-bottom subplot (row 2, col 2): speeds
    fig.add_trace(
        go.Scatter(x=sim.t, y=sim.surface_speed_mm_s, name="Workpiece Surface Speed [mm/s]", line=dict(color="#f97316", width=2.2)),
        row=2, col=2,
    )
    fig.add_trace(
        go.Scatter(x=sim.t, y=sim.planner_speed, name="Klipper Planner Speed [units/s]", line=dict(color="#a78bfa", width=1.8)),
        row=2, col=2,
    )
    fig.add_trace(
        go.Scatter(x=sim.t, y=np.abs(sim.vz_mm_s), name="|Vz| [mm/s]", line=dict(color="#facc15", width=1.5)),
        row=2, col=2,
    )
    ymax_spd = max(25.0, float(np.max(sim.surface_speed_mm_s)), float(np.max(sim.planner_speed))) * 1.08
    spd_cursor_idx = len(fig.data)
    fig.add_trace(
        go.Scatter(
            x=[sim.t[0], sim.t[0]], y=[0.0, ymax_spd],
            mode="lines", line=dict(color="#ffffff", width=1.5, dash="dot"),
            showlegend=False, hoverinfo="skip",
        ),
        row=2, col=2,
    )

    # Animation frames updating only dynamic traces
    dyn_traces = [nozzle_trace_idx, drop_trace_idx, spoke_trace_idx, pos_cursor_idx, spd_cursor_idx]
    frame_idxs = _sample_frame_indices(len(sim.t), max_frames=max_frames)
    frames = []
    slider_steps = []
    for k in frame_idxs:
        tk = float(sim.t[k])
        xk = float(sim.x_wp_mm[k])
        yk = float(sim.y_wp_mm[k])
        zk = float(sim.z_mm[k])
        th = math.radians(float(sim.x_deg[k]))
        nozzle_color = "#ff6b00" if bool(sim.valve_open[k]) else "#38bdf8"
        fname = f"{tk:.2f}s"
        frames.append(
            go.Frame(
                name=fname,
                traces=dyn_traces,
                data=[
                    go.Scatter3d(
                        x=[xk], y=[yk], z=[zk],
                        marker=dict(size=8, color=nozzle_color, symbol="diamond", line=dict(color="#ffffff", width=1.5)),
                    ),
                    go.Scatter3d(x=[xk, xk], y=[yk, yk], z=[0.0, zk]),
                    go.Scatter3d(
                        x=[0.0, r_max * math.cos(th)],
                        y=[0.0, -r_max * math.sin(th)],
                        z=[0.0, 0.0],
                    ),
                    go.Scatter(x=[tk, tk], y=[0.0, ymax_pos]),
                    go.Scatter(x=[tk, tk], y=[0.0, ymax_spd]),
                ],
            )
        )
        slider_steps.append(
            dict(
                method="animate",
                args=[[fname], dict(mode="immediate", frame=dict(duration=40, redraw=True), transition=dict(duration=0))],
                label=fname,
            )
        )

    fig.frames = frames

    s = sim.summary()
    subtitle = (
        f"Total: {s['total_time_s']:.2f}s (Move {s['move_time_s']:.2f}s / Home {s['home_time_s']:.2f}s / Dwell {s['dwell_time_s']:.2f}s) | "
        f"Z=[{s['z_range_mm'][0]:.2f}, {s['z_range_mm'][1]:.2f}] mm | "
        f"Peak Surface Speed: {s['peak_surface_speed_mm_s']:.1f} mm/s"
    )
    if mujoco_report is not None:
        status = "COLLISION-FREE" if mujoco_report["collision_free"] else "COLLISION DETECTED"
        wp_c = mujoco_report.get("min_workpiece_clearance_mm")
        wp_txt = f" | Min Workpiece Clearance: {wp_c:.2f} mm ({mujoco_report.get('min_workpiece_geom')})" if wp_c is not None else ""
        subtitle += f" | MuJoCo: {status} (Bed {mujoco_report['min_bed_clearance_mm']:.2f} mm{wp_txt})"

    fig.update_layout(
        template="plotly_dark",
        title=dict(text=f"KSP-GCODE Simulator — {sim.file_name}<br><sup>{subtitle}</sup>", x=0.02),
        height=840,
        margin=dict(l=16, r=16, t=95, b=82),
        legend=dict(orientation="h", yanchor="bottom", y=-0.16, xanchor="left", x=0.0, itemclick="toggle"),
        scene=dict(
            xaxis=dict(title="X_wp [mm]", range=[-r_max - 15, r_max + 15], backgroundcolor="#0f172a", gridcolor="#1e293b"),
            yaxis=dict(title="Y_wp [mm]", range=[-r_max - 15, r_max + 15], backgroundcolor="#0f172a", gridcolor="#1e293b"),
            zaxis=dict(title="Z [mm]", range=[0, 55], backgroundcolor="#0f172a", gridcolor="#1e293b"),
            aspectmode="manual",
            aspectratio=dict(x=1.0, y=1.0, z=0.42),
            # Front-angled view matching Worker-Image.jpeg (Purge pad at back/top, C1-C3 in front)
            camera=dict(eye=dict(x=0.0, y=-1.60, z=1.30), center=dict(x=0.0, y=0.0, z=-0.08)),
        ),
        xaxis=dict(title="Time [s]"),
        yaxis=dict(title="deg / mm"),
        xaxis2=dict(title="Time [s]"),
        yaxis2=dict(title="Speed [mm/s or units/s]"),
        updatemenus=[
            dict(
                type="buttons",
                showactive=False,
                x=0.02,
                y=1.02,
                xanchor="left",
                yanchor="bottom",
                direction="left",
                buttons=[
                    dict(
                        label="▶ Play",
                        method="animate",
                        args=[None, dict(frame=dict(duration=50, redraw=True), fromcurrent=True, transition=dict(duration=0))],
                    ),
                    dict(
                        label="⏸ Pause",
                        method="animate",
                        args=[[None], dict(frame=dict(duration=0, redraw=False), mode="immediate", transition=dict(duration=0))],
                    ),
                ],
            )
        ],
        sliders=[
            dict(
                active=0,
                steps=slider_steps,
                x=0.14,
                y=0.0,
                len=0.84,
                currentvalue=dict(prefix="t = ", visible=True),
            )
        ],
    )

    fig.write_html(str(out_html), include_plotlyjs="cdn")
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
<html lang="ja">
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
  <p class="sub">Klipper GCodeMove + LookAheadQueue + Parametric 3D Workpiece (Worker-Image.jpeg) + MuJoCo Collision/Clearance Verification</p>
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
