"""Parametric 3D workpiece model inspired by Working Folder_Adachi/Worker-Image.jpeg.

Designed specifically for visual clarity and exact coordinate alignment with
printer_data/config/printer.cfg, testPaint.gcode, and 13_clearance_check_high_z.gcode.

Coordinate convention (turntable-fixed workpiece frame, mm):
  - Origin (0, 0, 0): turntable rotation center at platter top surface (z = 0).
  - Radius from Y:    r(Y) = R_center - Y   (default R_center = 180.0 mm).
  - Angle X [deg]:    x_wp = r * cos(X),  y_wp = -r * sin(X)
    (Matches Worker-Image.jpeg top-down orientation without mirroring AND aligns
     with a +Z CCW turntable hinge in MuJoCo so every feature passes directly
     beneath the nozzle at world (r, 0, Z) when G-code commands (X, Y, Z).)

All heights are parametric approximations derived from Worker-Image.jpeg and
the working Z heights in testPaint.gcode (top coating faces set 1.0 mm below
working Z to provide realistic visual reference without false collisions;
NOT a certified CAD drawing for physical collision sign-off).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class BoltBoss:
    label: str
    theta_deg: float
    r_mm: float
    outer_r_mm: float = 13.5
    hole_r_mm: float = 5.5
    z_top_mm: float = 5.0


@dataclass(frozen=True)
class RaisedArcPad:
    label: str
    theta_start_deg: float
    theta_end_deg: float
    r_in_mm: float
    r_out_mm: float
    z_base_mm: float
    z_top_mm: float
    bolt_angles_deg: tuple[float, ...]
    color_hex: str = "#cbd5e1"
    rgba_mj: tuple[float, float, float, float] = (0.76, 0.80, 0.85, 0.88)


@dataclass(frozen=True)
class WorkpieceSpec:
    # Main cast housing body (dark grey cast metal in Worker-Image.jpeg)
    body_r_in_mm: float = 58.0
    body_r_out_mm: float = 118.0
    body_z_top_mm: float = 7.0

    # Machined central silver bearing ring + stepped inner bore
    inner_ring_r_in_mm: float = 40.0
    inner_ring_r_out_mm: float = 58.0
    inner_ring_z_top_mm: float = 7.0
    inner_step_r_in_mm: float = 24.0
    inner_step_r_out_mm: float = 40.0
    inner_step_z_top_mm: float = 3.5

    # 5 outer machined bolt bosses (Circles 1-5 in testPaint.gcode: Z_work=6 mm)
    bolt_bosses: tuple[BoltBoss, ...] = (
        BoltBoss("C1", 47.0, 135.0, z_top_mm=5.0),
        BoltBoss("C2", 88.0, 135.0, z_top_mm=5.0),
        BoltBoss("C3", 130.0, 135.0, z_top_mm=5.0),
        BoltBoss("C4", 226.5, 133.0, z_top_mm=5.0),
        BoltBoss("C5", 309.0, 133.0, z_top_mm=5.0),
    )

    # 2 elevated machined raised pads (Y77 -> r=103 mm, Z_work=16 mm)
    raised_pads: tuple[RaisedArcPad, ...] = (
        # Raised Section 1: larger 2-hole kidney-shaped pad (X30..95)
        RaisedArcPad(
            label="Raised 1",
            theta_start_deg=36.0,
            theta_end_deg=88.0,
            r_in_mm=89.0,
            r_out_mm=115.0,
            z_base_mm=7.0,
            z_top_mm=15.0,
            bolt_angles_deg=(49.0, 75.0),
            color_hex="#cbd5e1",
            rgba_mj=(0.78, 0.82, 0.87, 0.88),
        ),
        # Raised Section 2: smaller 1-hole pad (X250..295, in front of blue purge cloth)
        RaisedArcPad(
            label="Raised 2",
            theta_start_deg=258.0,
            theta_end_deg=288.0,
            r_in_mm=90.0,
            r_out_mm=115.0,
            z_base_mm=7.0,
            z_top_mm=15.0,
            bolt_angles_deg=(273.0,),
            color_hex="#cbd5e1",
            rgba_mj=(0.78, 0.82, 0.87, 0.88),
        ),
    )

    # Outer blue purge / squeeze-out cloth pad (Y15 -> r=165 mm, X265..285, Z_work=19/18 mm)
    purge_pad: RaisedArcPad = RaisedArcPad(
        label="Purge Pad",
        theta_start_deg=261.0,
        theta_end_deg=289.0,
        r_in_mm=153.0,
        r_out_mm=177.0,
        z_base_mm=10.5,
        z_top_mm=17.0,
        bolt_angles_deg=(),
        color_hex="#60a5fa",
        rgba_mj=(0.28, 0.56, 0.88, 0.82),
    )


DEFAULT_WORKPIECE = WorkpieceSpec()


def polar_to_wp_xy(r_mm: float, theta_deg: float) -> tuple[float, float]:
    rad = math.radians(theta_deg)
    return r_mm * math.cos(rad), -r_mm * math.sin(rad)


# ---------------------------------------------------------------------------
# Lightweight Triangulated Mesh Builders (for Plotly Mesh3d)
# ---------------------------------------------------------------------------

def _mesh_annular_sector(
    r_in: float,
    r_out: float,
    theta_start_deg: float,
    theta_end_deg: float,
    z0: float,
    z1: float,
    n_arc: int = 24,
    closed_ring: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (vertices [V,3], faces [F,3]) for a clean annular cylinder or sector."""
    if closed_ring:
        angles = np.linspace(0.0, 360.0, n_arc, endpoint=False)
    else:
        angles = np.linspace(theta_start_deg, theta_end_deg, n_arc + 1)

    m = len(angles)
    rad = np.deg2rad(angles)
    c = np.cos(rad)
    s = -np.sin(rad)  # CW workpiece orientation

    # 4 rings of m vertices:
    # 0..m-1: inner bottom, m..2m-1: outer bottom, 2m..3m-1: inner top, 3m..4m-1: outer top
    ib = np.column_stack([r_in * c, r_in * s, np.full(m, z0)])
    ob = np.column_stack([r_out * c, r_out * s, np.full(m, z0)])
    it = np.column_stack([r_in * c, r_in * s, np.full(m, z1)])
    ot = np.column_stack([r_out * c, r_out * s, np.full(m, z1)])
    verts = np.vstack([ib, ob, it, ot])

    faces: list[tuple[int, int, int]] = []
    segs = m if closed_ring else (m - 1)
    for k in range(segs):
        kn = (k + 1) % m
        i0, i1 = k, kn
        o0, o1 = m + k, m + kn
        it0, it1 = 2 * m + k, 2 * m + kn
        ot0, ot1 = 3 * m + k, 3 * m + kn
        # Top cap
        faces.append((it0, ot0, ot1))
        faces.append((it0, ot1, it1))
        # Bottom cap
        faces.append((i0, o1, o0))
        faces.append((i0, i1, o1))
        # Outer wall
        faces.append((o0, o1, ot1))
        faces.append((o0, ot1, ot0))
        # Inner wall
        faces.append((i0, it1, i1))
        faces.append((i0, it0, it1))

    if not closed_ring:
        # Start end cap (k=0)
        faces.append((0, m, 3 * m))
        faces.append((0, 3 * m, 2 * m))
        # End cap (k=m-1)
        e = m - 1
        faces.append((e, 3 * m + e, m + e))
        faces.append((e, 2 * m + e, 3 * m + e))

    return verts, np.array(faces, dtype=int)


def _mesh_hollow_boss(
    cx: float,
    cy: float,
    r_in: float,
    r_out: float,
    z0: float,
    z1: float,
    n_arc: int = 20,
) -> tuple[np.ndarray, np.ndarray]:
    verts, faces = _mesh_annular_sector(r_in, r_out, 0.0, 360.0, z0, z1, n_arc=n_arc, closed_ring=True)
    verts[:, 0] += cx
    verts[:, 1] += cy
    return verts, faces


def _merge_meshes(parts: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, np.ndarray]:
    all_v = []
    all_f = []
    offset = 0
    for v, f in parts:
        all_v.append(v)
        all_f.append(f + offset)
        offset += len(v)
    return np.vstack(all_v), np.vstack(all_f)


def build_plotly_workpiece_meshes(spec: WorkpieceSpec = DEFAULT_WORKPIECE) -> list[dict[str, Any]]:
    """Return grouped translucent mesh dicts ready for Plotly go.Mesh3d."""
    # Group 1: Dark cast housing body
    cast_v, cast_f = _mesh_annular_sector(
        spec.body_r_in_mm, spec.body_r_out_mm, 0.0, 360.0, 0.0, spec.body_z_top_mm, n_arc=56, closed_ring=True
    )

    # Group 2: Machined silver surfaces (inner ring, inner step, 5 bolt bosses, 2 raised pads)
    machined_parts = [
        _mesh_annular_sector(
            spec.inner_ring_r_in_mm, spec.inner_ring_r_out_mm, 0.0, 360.0, 0.0, spec.inner_ring_z_top_mm, n_arc=48, closed_ring=True
        ),
        _mesh_annular_sector(
            spec.inner_step_r_in_mm, spec.inner_step_r_out_mm, 0.0, 360.0, 0.0, spec.inner_step_z_top_mm, n_arc=40, closed_ring=True
        ),
    ]
    for b in spec.bolt_bosses:
        cx, cy = polar_to_wp_xy(b.r_mm, b.theta_deg)
        machined_parts.append(_mesh_hollow_boss(cx, cy, b.hole_r_mm, b.outer_r_mm, 0.0, b.z_top_mm, n_arc=20))
    for pad in spec.raised_pads:
        machined_parts.append(
            _mesh_annular_sector(
                pad.r_in_mm, pad.r_out_mm, pad.theta_start_deg, pad.theta_end_deg, pad.z_base_mm, pad.z_top_mm, n_arc=18
            )
        )
    mach_v, mach_f = _merge_meshes(machined_parts)

    # Group 3: Blue purge cloth pad
    p = spec.purge_pad
    purge_v, purge_f = _mesh_annular_sector(
        p.r_in_mm, p.r_out_mm, p.theta_start_deg, p.theta_end_deg, p.z_base_mm, p.z_top_mm, n_arc=14
    )

    return [
        {
            "name": "Workpiece 3D (Cast Body)",
            "verts": cast_v,
            "faces": cast_f,
            "color": "#475569",
            "opacity": 0.30,
        },
        {
            "name": "Workpiece 3D (Machined Faces)",
            "verts": mach_v,
            "faces": mach_f,
            "color": "#cbd5e1",
            "opacity": 0.42,
        },
        {
            "name": "Workpiece 3D (Blue Purge Pad)",
            "verts": purge_v,
            "faces": purge_f,
            "color": "#60a5fa",
            "opacity": 0.42,
        },
    ]


def build_workpiece_wireframe_lines(spec: WorkpieceSpec = DEFAULT_WORKPIECE) -> tuple[list[float], list[float], list[float]]:
    """Build crisp 3D top-edge outline polylines (with None separators) for unambiguous depth cue."""
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []

    def add_ring(r: float, z: float, n: int = 64, cx: float = 0.0, cy: float = 0.0) -> None:
        a = np.linspace(0.0, 2.0 * np.pi, n + 1)
        xs.extend((cx + r * np.cos(a)).tolist() + [None])
        ys.extend((cy - r * np.sin(a)).tolist() + [None])
        zs.extend([z] * (n + 1) + [None])

    def add_sector_top(pad: RaisedArcPad, n: int = 20) -> None:
        a_out = np.deg2rad(np.linspace(pad.theta_start_deg, pad.theta_end_deg, n))
        a_in = np.deg2rad(np.linspace(pad.theta_end_deg, pad.theta_start_deg, n))
        px = np.concatenate([pad.r_out_mm * np.cos(a_out), pad.r_in_mm * np.cos(a_in), [pad.r_out_mm * math.cos(a_out[0])]])
        py = np.concatenate([-pad.r_out_mm * np.sin(a_out), -pad.r_in_mm * np.sin(a_in), [-pad.r_out_mm * math.sin(a_out[0])]])
        xs.extend(px.tolist() + [None])
        ys.extend(py.tolist() + [None])
        zs.extend([pad.z_top_mm] * len(px) + [None])

    # Cast outer edge + inner machined ring boundaries
    add_ring(spec.body_r_out_mm, spec.body_z_top_mm, n=72)
    add_ring(spec.inner_ring_r_out_mm, spec.inner_ring_z_top_mm, n=56)
    add_ring(spec.inner_ring_r_in_mm, spec.inner_ring_z_top_mm, n=48)
    add_ring(spec.inner_step_r_in_mm, spec.inner_step_z_top_mm, n=40)

    # 5 outer bolt bosses (outer + hole ring)
    for b in spec.bolt_bosses:
        cx, cy = polar_to_wp_xy(b.r_mm, b.theta_deg)
        add_ring(b.outer_r_mm, b.z_top_mm, n=24, cx=cx, cy=cy)
        add_ring(b.hole_r_mm, b.z_top_mm, n=16, cx=cx, cy=cy)

    # Raised pads + bolt holes + purge pad
    for pad in spec.raised_pads:
        add_sector_top(pad, n=20)
        r_mid = 0.5 * (pad.r_in_mm + pad.r_out_mm)
        for ang in pad.bolt_angles_deg:
            hx, hy = polar_to_wp_xy(r_mid, ang)
            add_ring(4.8, pad.z_top_mm, n=14, cx=hx, cy=hy)

    add_sector_top(spec.purge_pad, n=16)
    return xs, ys, zs


def build_workpiece_labels(spec: WorkpieceSpec = DEFAULT_WORKPIECE) -> list[tuple[str, float, float, float]]:
    """Return subtle 3D callout labels (text, x_mm, y_mm, z_mm)."""
    labels: list[tuple[str, float, float, float]] = [
        ("Inner (Z7)", 0.0, 0.0, spec.inner_ring_z_top_mm + 1.5),
    ]
    for b in spec.bolt_bosses:
        lx, ly = polar_to_wp_xy(b.r_mm + 17.0, b.theta_deg)
        labels.append((b.label, lx, ly, b.z_top_mm + 1.5))
    for pad in spec.raised_pads:
        mid_ang = 0.5 * (pad.theta_start_deg + pad.theta_end_deg)
        r_mid = 0.5 * (pad.r_in_mm + pad.r_out_mm)
        lx, ly = polar_to_wp_xy(r_mid, mid_ang)
        labels.append((f"{pad.label} (Z15)", lx, ly, pad.z_top_mm + 1.8))
    p = spec.purge_pad
    px, py = polar_to_wp_xy(0.5 * (p.r_in_mm + p.r_out_mm), 0.5 * (p.theta_start_deg + p.theta_end_deg))
    labels.append(("Purge (Z17)", px, py, p.z_top_mm + 1.8))
    return labels


# ---------------------------------------------------------------------------
# MuJoCo MJCF Parametric Workpiece XML Builder (Exact Collision Geoms)
# ---------------------------------------------------------------------------

def build_mujoco_workpiece_xml(spec: WorkpieceSpec = DEFAULT_WORKPIECE) -> str:
    """Generate clean MuJoCo XML geoms (in meters) attached to the rotating turntable body."""
    lines: list[str] = ['      <body name="parametric_workpiece" pos="0 0 0">']

    # 1. Main cast housing cylinder (approximated as clean outer disk + machined visual ring)
    # Using a solid cylinder for fast exact mj_geomDistance; top surface at body_z_top_mm.
    body_h = spec.body_z_top_mm / 1000.0
    body_r = spec.body_r_out_mm / 1000.0
    lines.append(
        f'        <geom name="wp_cast_body" type="cylinder" pos="0 0 {body_h / 2:.6f}" '
        f'size="{body_r:.6f} {body_h / 2:.6f}" rgba="0.22 0.25 0.30 0.85" contype="1" conaffinity="1"/>'
    )
    # Bright machined central ring visual cap (slightly proud by 0.1 mm for clear contrast)
    ring_r = spec.inner_ring_r_out_mm / 1000.0
    lines.append(
        f'        <geom name="wp_inner_ring_vis" type="cylinder" pos="0 0 {body_h + 0.0001:.6f}" '
        f'size="{ring_r:.6f} 0.0001" rgba="0.82 0.85 0.90 0.95" contype="0" conaffinity="0"/>'
    )
    bore_r = spec.inner_ring_r_in_mm / 1000.0
    lines.append(
        f'        <geom name="wp_inner_bore_vis" type="cylinder" pos="0 0 {body_h + 0.0002:.6f}" '
        f'size="{bore_r:.6f} 0.0001" rgba="0.16 0.18 0.22 0.95" contype="0" conaffinity="0"/>'
    )

    # 2. Five outer machined bolt bosses (Circles 1-5)
    for b in spec.bolt_bosses:
        cx, cy = polar_to_wp_xy(b.r_mm, b.theta_deg)
        bh = b.z_top_mm / 1000.0
        br = b.outer_r_mm / 1000.0
        hr = b.hole_r_mm / 1000.0
        tag = b.label.lower()
        lines.append(
            f'        <geom name="wp_boss_{tag}" type="cylinder" pos="{cx / 1000:.6f} {cy / 1000:.6f} {bh / 2:.6f}" '
            f'size="{br:.6f} {bh / 2:.6f}" rgba="0.74 0.78 0.83 0.92" contype="1" conaffinity="1"/>'
        )
        # Visual center bolt hole dark disk
        lines.append(
            f'        <geom name="wp_boss_{tag}_hole" type="cylinder" pos="{cx / 1000:.6f} {cy / 1000:.6f} {bh + 0.00015:.6f}" '
            f'size="{hr:.6f} 0.00015" rgba="0.12 0.14 0.18 0.98" contype="0" conaffinity="0"/>'
        )
        # Visual radial web connecting boss to main body
        web_r = 0.5 * (spec.body_r_out_mm + b.r_mm)
        wx, wy = polar_to_wp_xy(web_r, b.theta_deg)
        web_len = max(4.0, b.r_mm - spec.body_r_out_mm) / 1000.0
        yaw_rad = -math.radians(b.theta_deg)
        lines.append(
            f'        <geom name="wp_web_{tag}" type="box" pos="{wx / 1000:.6f} {wy / 1000:.6f} {bh / 2:.6f}" '
            f'euler="0 0 {yaw_rad:.6f}" size="{web_len / 2:.6f} 0.009 {bh / 2:.6f}" '
            f'rgba="0.22 0.25 0.30 0.85" contype="0" conaffinity="0"/>'
        )

    # 3. Curved raised pads & blue purge pad approximated cleanly by segmented boxes along the arc
    def add_arc_pad_geoms(pad: RaisedArcPad, prefix: str, n_boxes: int) -> None:
        r_mid = 0.5 * (pad.r_in_mm + pad.r_out_mm) / 1000.0
        rad_thick = (pad.r_out_mm - pad.r_in_mm) / 1000.0
        z_mid = 0.5 * (pad.z_base_mm + pad.z_top_mm) / 1000.0
        z_half = 0.5 * (pad.z_top_mm - pad.z_base_mm) / 1000.0
        span_deg = pad.theta_end_deg - pad.theta_start_deg
        step_deg = span_deg / n_boxes
        chord_half = 0.5 * r_mid * math.radians(step_deg) * 1.05
        rgba_str = " ".join(f"{c:.2f}" for c in pad.rgba_mj)
        for k in range(n_boxes):
            ang = pad.theta_start_deg + (k + 0.5) * step_deg
            px, py = polar_to_wp_xy(r_mid * 1000.0, ang)
            yaw = -math.radians(ang)
            lines.append(
                f'        <geom name="wp_{prefix}_{k}" type="box" '
                f'pos="{px / 1000:.6f} {py / 1000:.6f} {z_mid:.6f}" euler="0 0 {yaw:.6f}" '
                f'size="{rad_thick / 2:.6f} {chord_half:.6f} {z_half:.6f}" '
                f'rgba="{rgba_str}" contype="1" conaffinity="1"/>'
            )
        # Bolt hole visual markers on raised pads
        for idx, bang in enumerate(pad.bolt_angles_deg):
            hx, hy = polar_to_wp_xy(r_mid * 1000.0, bang)
            lines.append(
                f'        <geom name="wp_{prefix}_hole_{idx}" type="cylinder" '
                f'pos="{hx / 1000:.6f} {hy / 1000:.6f} {pad.z_top_mm / 1000.0 + 0.00015:.6f}" '
                f'size="0.0048 0.00015" rgba="0.12 0.14 0.18 0.98" contype="0" conaffinity="0"/>'
            )

    add_arc_pad_geoms(spec.raised_pads[0], "raised1", n_boxes=5)
    add_arc_pad_geoms(spec.raised_pads[1], "raised2", n_boxes=3)
    add_arc_pad_geoms(spec.purge_pad, "purge", n_boxes=3)

    # Support block under the blue purge pad (visual)
    p = spec.purge_pad
    p_mid_ang = 0.5 * (p.theta_start_deg + p.theta_end_deg)
    pr_mid = 0.5 * (p.r_in_mm + p.r_out_mm)
    px, py = polar_to_wp_xy(pr_mid, p_mid_ang)
    pz_base = p.z_base_mm / 1000.0
    lines.append(
        f'        <geom name="wp_purge_mount" type="box" '
        f'pos="{px / 1000:.6f} {py / 1000:.6f} {pz_base / 2:.6f}" euler="0 0 {-math.radians(p_mid_ang):.6f}" '
        f'size="0.011 0.036 {pz_base / 2:.6f}" rgba="0.16 0.18 0.22 0.85" contype="0" conaffinity="0"/>'
    )

    lines.append("      </body>")
    return "\n".join(lines)
