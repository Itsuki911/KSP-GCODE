#!/usr/bin/env python3
"""Unified CLI for local KSP-GCODE simulation (Klipper kinematics + MuJoCo 3D + Plotly/PNG)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
ADACHI_DIR = SIM_DIR.parent
SINGLE_MOTION_DIR = ADACHI_DIR / "single_motion_tests"
DEFAULT_OUT_DIR = SIM_DIR / "output"

if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from klipper_sim import DEFAULT_PRINTER_CFG, load_machine_config, simulate_gcode
from mujoco_sim import launch_mujoco_viewer, run_mujoco_headless
from visualize import generate_index_html, generate_interactive_html, generate_summary_png


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Local OSS simulator for KSP-GCODE (Klipper LookAhead + Polar Kinematics + MuJoCo 3D)"
    )
    p.add_argument(
        "gcode",
        nargs="?",
        help="Path to a .gcode file (omit when using --all).",
    )
    p.add_argument(
        "--all",
        action="store_true",
        help="Simulate all 16 single_motion_tests/*.gcode + Working Folder_Adachi/testPaint.gcode.",
    )
    p.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_PRINTER_CFG,
        help="Path to Klipper printer.cfg (default: printer_data/config/printer.cfg).",
    )
    p.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help="Output directory for HTML/PNG/CSV/JSON artifacts.",
    )
    p.add_argument("--dt", type=float, default=0.02, help="Sampling timestep in seconds (default: 0.02).")
    p.add_argument(
        "--r-center",
        type=float,
        default=180.0,
        help="Turntable center Y coordinate in mm so r(Y) = r_center - Y (default: 180.0).",
    )
    p.add_argument(
        "--valve-response-ms",
        type=float,
        default=0.0,
        help="Optional simulated ball-valve servo travel time in ms (default: 0 = ideal step).",
    )
    p.add_argument("--no-html", action="store_true", help="Skip Plotly interactive 3D HTML generation.")
    p.add_argument("--no-png", action="store_true", help="Skip Matplotlib PNG summary generation.")
    p.add_argument("--no-csv", action="store_true", help="Skip CSV telemetry export.")
    p.add_argument("--no-mujoco", action="store_true", help="Skip MuJoCo headless clearance/dynamics check.")
    p.add_argument(
        "--no-workpiece",
        action="store_true",
        help="Hide the parametric 3D workpiece (empty turntable bed mode).",
    )
    p.add_argument(
        "--view",
        action="store_true",
        help="Launch interactive 3D MuJoCo GUI viewer (on macOS, run via `mjpython`).",
    )
    p.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier for --view (default: 1.0).")
    p.add_argument("--loop", action="store_true", help="Loop playback in --view mode.")
    p.add_argument(
        "--workpiece-stl",
        type=Path,
        default=None,
        help="Optional STL mesh of workpiece/fixture mounted on the turntable for MuJoCo collision check.",
    )
    p.add_argument(
        "--workpiece-scale",
        type=float,
        default=0.001,
        help="Scale factor from STL units to meters (default: 0.001 for mm STLs).",
    )
    p.add_argument(
        "--workpiece-z-offset-mm",
        type=float,
        default=0.0,
        help="Vertical offset (mm) for mounted workpiece STL above turntable platter z=0.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_machine_config(args.config, r_center_mm=args.r_center)

    if args.all:
        targets = sorted(SINGLE_MOTION_DIR.glob("*.gcode"))
        test_paint = ADACHI_DIR / "testPaint.gcode"
        if test_paint.exists():
            targets.append(test_paint)
    elif args.gcode:
        targets = [Path(args.gcode).resolve()]
    else:
        print("Error: specify a G-code file or --all", file=sys.stderr)
        return 2

    if args.view and len(targets) != 1:
        print("Error: --view requires a single G-code file", file=sys.stderr)
        return 2

    args.out_dir.mkdir(parents=True, exist_ok=True)
    report_rows = []
    any_collision = False

    for gpath in targets:
        if not gpath.exists():
            print(f"Error: file not found: {gpath}", file=sys.stderr)
            return 2

        sim = simulate_gcode(
            gpath,
            cfg=cfg,
            dt=args.dt,
            valve_response_ms=args.valve_response_ms,
        )
        stem = gpath.stem
        mj_dict = None
        use_wp = not args.no_workpiece

        if not args.no_mujoco:
            mj_rep = run_mujoco_headless(
                sim,
                use_workpiece=use_wp,
                workpiece_stl=args.workpiece_stl,
                workpiece_scale=args.workpiece_scale,
                workpiece_z_offset_mm=args.workpiece_z_offset_mm,
            )
            mj_dict = mj_rep.to_dict()
            if not mj_rep.collision_free:
                any_collision = True

        html_rel = None
        png_rel = None
        csv_rel = None

        if not args.no_html:
            html_path = args.out_dir / f"{stem}.html"
            generate_interactive_html(sim, html_path, mujoco_report=mj_dict, show_workpiece=use_wp)
            html_rel = html_path.name

        if not args.no_png:
            png_path = args.out_dir / f"{stem}_summary.png"
            generate_summary_png(sim, png_path, mujoco_report=mj_dict, show_workpiece=use_wp)
            png_rel = png_path.name

        if not args.no_csv:
            csv_path = args.out_dir / f"{stem}_trace.csv"
            sim.write_csv(csv_path)
            csv_rel = csv_path.name

        ksum = sim.summary()
        report_rows.append(
            {
                "klipper": ksum,
                "mujoco": mj_dict,
                "html_rel": html_rel,
                "png_rel": png_rel,
                "csv_rel": csv_rel,
            }
        )

        if mj_dict:
            wp_clr = mj_dict.get("min_workpiece_clearance_mm")
            wp_part = f"wp_clear={wp_clr:5.2f}mm({mj_dict.get('min_workpiece_geom')}) " if wp_clr is not None else ""
            mj_str = (
                f"bed={mj_dict['min_bed_clearance_mm']:5.2f}mm "
                f"{wp_part}"
                f"con={mj_dict['contact_samples']} "
                f"tauX={mj_dict['peak_turntable_torque_nm']:.3f}Nm"
            )
        else:
            mj_str = "mujoco=skipped"
        print(
            f"[SIM] {ksum['file_name']:38s} "
            f"T={ksum['total_time_s']:6.2f}s "
            f"moves={ksum['move_count']:3d} "
            f"vSurfMax={ksum['peak_surface_speed_mm_s']:6.1f}mm/s "
            f"Z=[{ksum['z_range_mm'][0]:5.2f},{ksum['z_range_mm'][1]:5.2f}] "
            f"{mj_str} "
            f"warn={ksum['warning_count']}"
        )

        if args.view:
            launch_mujoco_viewer(
                sim,
                speed=args.speed,
                loop=args.loop,
                use_workpiece=use_wp,
                workpiece_stl=args.workpiece_stl,
                workpiece_scale=args.workpiece_scale,
                workpiece_z_offset_mm=args.workpiece_z_offset_mm,
            )

    summary_json = args.out_dir / "simulation_summary.json"
    summary_json.write_text(json.dumps(report_rows, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    if len(report_rows) > 1 or args.all:
        index_html = generate_index_html(report_rows, args.out_dir / "index.html")
        print(f"\nDashboard written to: {index_html}")
    print(f"JSON summary written to: {summary_json}")

    return 1 if any_collision else 0


if __name__ == "__main__":
    raise SystemExit(main())
