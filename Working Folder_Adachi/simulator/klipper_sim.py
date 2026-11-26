"""Klipper-faithful offline motion & polar-kinematics simulator for KSP-GCODE.

Uses the repository's own Klipper GCodeMove, Move, and LookAheadQueue classes
(no device I/O) and reads machine limits directly from printer_data/config/printer.cfg.
Transforms machine coordinates (X [deg], Y [mm], Z [mm]) into cylindrical/polar
workpiece-frame and world-frame trajectories with exact trapezoidal velocity profiles.
"""
from __future__ import annotations

import ast
import configparser
import csv
import json
import logging
import math
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

SIM_DIR = Path(__file__).resolve().parent
ADACHI_DIR = SIM_DIR.parent
ROOT = ADACHI_DIR.parent
DEFAULT_PRINTER_CFG = ROOT / "printer_data" / "config" / "printer.cfg"


@dataclass
class MachineConfig:
    max_velocity: float = 125.0
    max_accel: float = 1000.0
    max_z_velocity: float = 20.0
    max_z_accel: float = 500.0
    square_corner_velocity: float = 6.0
    minimum_cruise_ratio: float = 0.5
    x_min: float = 0.0
    x_max: float = 360.0
    x_endstop: float = 0.0
    x_homing_speed: float = 50.0
    y_min: float = 0.0
    y_max: float = 180.0
    y_endstop: float = 0.0
    y_homing_speed: float = 50.0
    z_min: float = 0.0
    z_max: float = 60.0
    z_endstop: float = 47.250
    z_homing_speed: float = 20.0
    valve_open_angle: float = 45.0
    valve_closed_angle: float = 0.0
    r_center_mm: float = 180.0  # Physical radius r(Y) = r_center_mm - Y

    @property
    def home_pos(self) -> tuple[float, float, float]:
        return (self.x_endstop, self.y_endstop, self.z_endstop)

    @property
    def safe_z_max(self) -> float:
        return min(self.z_max, self.z_endstop)


def load_machine_config(
    cfg_path: Path = DEFAULT_PRINTER_CFG,
    r_center_mm: float = 180.0,
) -> MachineConfig:
    """Parse relevant limits and SAVE_CONFIG overrides from Klipper printer.cfg."""
    raw_lines = cfg_path.read_text(encoding="utf-8", errors="replace").splitlines()
    # Unwrap SAVE_CONFIG block (#*# ...) so configparser sees overrides too.
    main_lines: list[str] = []
    save_lines: list[str] = []
    in_save = False
    for line in raw_lines:
        if "SAVE_CONFIG" in line:
            in_save = True
            continue
        if in_save:
            if line.startswith("#*#"):
                save_lines.append(line[3:].lstrip())
            continue
        # Strip inline comments only outside Jinja macro bodies
        main_lines.append(line)

    def parse_section_kv(lines: list[str]) -> dict[str, dict[str, str]]:
        sections: dict[str, dict[str, str]] = {}
        cur: str | None = None
        for ln in lines:
            stripped = ln.strip()
            if not stripped or stripped.startswith("#") or stripped.startswith(";"):
                continue
            if stripped.startswith("[") and stripped.endswith("]"):
                cur = stripped[1:-1].strip()
                sections.setdefault(cur, {})
                continue
            if cur is None:
                continue
            if ln[:1].isspace():
                # Indented macro/multiline continuation
                continue
            m = re.match(r"^([A-Za-z0-9_]+)\s*[:=]\s*(.*)$", stripped)
            if m:
                key = m.group(1).strip()
                val = m.group(2).split("#", 1)[0].strip()
                sections[cur][key] = val
        return sections

    sec = parse_section_kv(main_lines)
    save_sec = parse_section_kv(save_lines)
    for name, kv in save_sec.items():
        sec.setdefault(name, {}).update(kv)

    def getf(section: str, key: str, default: float) -> float:
        val = sec.get(section, {}).get(key)
        if val is None or val == "":
            return default
        try:
            return float(val)
        except ValueError:
            return default

    # Extract BALL_VALVE_OPEN / BALL_VALVE_CLOSED angles from macro text
    full_text = "\n".join(raw_lines)
    open_m = re.search(
        r"\[gcode_macro\s+BALL_VALVE_OPEN\][\s\S]*?ANGLE\s*=\s*([0-9.]+)",
        full_text,
    )
    closed_m = re.search(
        r"\[gcode_macro\s+BALL_VALVE_CLOSED\][\s\S]*?ANGLE\s*=\s*([0-9.]+)",
        full_text,
    )

    return MachineConfig(
        max_velocity=getf("printer", "max_velocity", 125.0),
        max_accel=getf("printer", "max_accel", 1000.0),
        max_z_velocity=getf("printer", "max_z_velocity", 20.0),
        max_z_accel=getf("printer", "max_z_accel", 500.0),
        square_corner_velocity=getf("printer", "square_corner_velocity", 6.0),
        minimum_cruise_ratio=getf("printer", "minimum_cruise_ratio", 0.5),
        x_min=getf("stepper_x", "position_min", 0.0),
        x_max=getf("stepper_x", "position_max", 360.0),
        x_endstop=getf("stepper_x", "position_endstop", 0.0),
        x_homing_speed=getf("stepper_x", "homing_speed", 50.0),
        y_min=getf("stepper_y", "position_min", 0.0),
        y_max=getf("stepper_y", "position_max", 180.0),
        y_endstop=getf("stepper_y", "position_endstop", 0.0),
        y_homing_speed=getf("stepper_y", "homing_speed", 50.0),
        z_min=getf("stepper_z", "position_min", 0.0),
        z_max=getf("stepper_z", "position_max", 60.0),
        z_endstop=getf("stepper_z", "position_endstop", 47.250),
        z_homing_speed=getf("stepper_z", "homing_speed", 20.0),
        valve_open_angle=float(open_m.group(1)) if open_m else 45.0,
        valve_closed_angle=float(closed_m.group(1)) if closed_m else 0.0,
        r_center_mm=r_center_mm,
    )


_KLIPPER_CACHE: dict[str, Any] = {}


def _load_klipper_classes() -> dict[str, Any]:
    """Load GCodeMove, Move, and LookAheadQueue from the local klipper repo."""
    if _KLIPPER_CACHE:
        return _KLIPPER_CACHE

    gm_src = ROOT / "klipper" / "klippy" / "extras" / "gcode_move.py"
    gm_tree = ast.parse(gm_src.read_text(encoding="utf-8"))
    gm_classes = [
        n for n in gm_tree.body if isinstance(n, ast.ClassDef) and n.name == "GCodeMove"
    ]
    assert len(gm_classes) == 1
    gm_ns: dict[str, Any] = {"logging": logging}
    exec(compile(ast.Module(body=gm_classes, type_ignores=[]), str(gm_src), "exec"), gm_ns)

    th_src = ROOT / "klipper" / "klippy" / "toolhead.py"
    th_tree = ast.parse(th_src.read_text(encoding="utf-8"))
    th_classes = [
        n
        for n in th_tree.body
        if isinstance(n, ast.ClassDef) and n.name in ("Move", "LookAheadQueue")
    ]
    assert len(th_classes) == 2
    th_ns: dict[str, Any] = {"math": math, "LOOKAHEAD_FLUSH_TIME": 0.150}
    exec(compile(ast.Module(body=th_classes, type_ignores=[]), str(th_src), "exec"), th_ns)

    _KLIPPER_CACHE["GCodeMove"] = gm_ns["GCodeMove"]
    _KLIPPER_CACHE["Move"] = th_ns["Move"]
    _KLIPPER_CACHE["LookAheadQueue"] = th_ns["LookAheadQueue"]
    return _KLIPPER_CACHE


class _GCodeCommand:
    def __init__(self, text: str) -> None:
        self.text = text
        self.params: dict[str, str] = {}
        for word in text.split()[1:]:
            if "=" in word:
                key, value = word.split("=", 1)
            else:
                key, value = word[0], word[1:]
            self.params[key.upper()] = value

    def get_command_parameters(self) -> dict[str, str]:
        return self.params

    def get_commandline(self) -> str:
        return self.text

    def get_float(self, key: str, default: float | None = None, **limits: float) -> float | None:
        value = float(self.params[key]) if key in self.params else default
        if value is not None:
            if "above" in limits and not (value > limits["above"]):
                raise ValueError(f"{key}={value} must be > {limits['above']}")
            if "minval" in limits and not (value >= limits["minval"]):
                raise ValueError(f"{key}={value} must be >= {limits['minval']}")
        return value

    def get_int(self, key: str, default: int | None = None, **limits: int) -> int | None:
        return int(float(self.params[key])) if key in self.params else default

    def error(self, message: str) -> ValueError:
        return ValueError(message)


@dataclass
class Segment:
    kind: str  # "move", "home", "dwell"
    line: int
    command: str
    t_start: float
    duration: float
    start_pos: tuple[float, float, float]
    end_pos: tuple[float, float, float]
    start_v: float
    cruise_v: float
    end_v: float
    accel: float
    accel_t: float
    cruise_t: float
    decel_t: float
    valve_target_deg: float
    valve_state: str  # "closed", "open", "partial", "unknown"


@dataclass
class SimEvent:
    t: float
    line: int
    kind: str
    command: str
    pos: tuple[float, float, float]
    detail: str = ""


@dataclass
class SimulationResult:
    file_name: str
    file_path: str
    config: MachineConfig
    dt: float
    valve_response_ms: float
    segments: list[Segment]
    events: list[SimEvent]
    warnings: list[str]
    # Uniformly sampled time-series (numpy arrays)
    t: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    x_deg: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    y_mm: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    z_mm: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    vx_deg_s: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    vy_mm_s: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    vz_mm_s: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    ax_deg_s2: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    ay_mm_s2: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    az_mm_s2: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    planner_speed: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    r_mm: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    surface_speed_mm_s: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    x_wp_mm: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    y_wp_mm: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    valve_deg: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    valve_open: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0, dtype=bool))
    seg_kind: list[str] = field(repr=False, default_factory=list)

    def summary(self) -> dict[str, Any]:
        move_segs = [s for s in self.segments if s.kind == "move"]
        home_segs = [s for s in self.segments if s.kind == "home"]
        dwell_segs = [s for s in self.segments if s.kind == "dwell"]
        return {
            "file_name": self.file_name,
            "total_time_s": round(float(self.t[-1]) if len(self.t) else 0.0, 3),
            "move_time_s": round(sum(s.duration for s in move_segs), 3),
            "home_time_s": round(sum(s.duration for s in home_segs), 3),
            "dwell_time_s": round(sum(s.duration for s in dwell_segs), 3),
            "move_count": len(move_segs),
            "home_count": len(home_segs),
            "valve_open_events": sum(1 for e in self.events if e.kind == "BALL_VALVE_OPEN"),
            "valve_close_events": sum(1 for e in self.events if e.kind == "BALL_VALVE_CLOSED"),
            "x_range_deg": [round(float(np.min(self.x_deg)), 3), round(float(np.max(self.x_deg)), 3)],
            "y_range_mm": [round(float(np.min(self.y_mm)), 3), round(float(np.max(self.y_mm)), 3)],
            "z_range_mm": [round(float(np.min(self.z_mm)), 3), round(float(np.max(self.z_mm)), 3)],
            "peak_planner_speed": round(float(np.max(self.planner_speed)) if len(self.t) else 0.0, 2),
            "peak_surface_speed_mm_s": round(float(np.max(self.surface_speed_mm_s)) if len(self.t) else 0.0, 2),
            "peak_z_speed_mm_s": round(float(np.max(np.abs(self.vz_mm_s))) if len(self.t) else 0.0, 2),
            "warning_count": len(self.warnings),
            "warnings": self.warnings,
        }

    def write_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([
                "t_s", "x_deg", "y_mm", "z_mm",
                "vx_deg_s", "vy_mm_s", "vz_mm_s",
                "planner_speed", "r_mm", "surface_speed_mm_s",
                "x_wp_mm", "y_wp_mm", "valve_deg", "valve_open", "segment_kind",
            ])
            for i in range(len(self.t)):
                w.writerow([
                    f"{self.t[i]:.4f}", f"{self.x_deg[i]:.4f}", f"{self.y_mm[i]:.4f}", f"{self.z_mm[i]:.4f}",
                    f"{self.vx_deg_s[i]:.3f}", f"{self.vy_mm_s[i]:.3f}", f"{self.vz_mm_s[i]:.3f}",
                    f"{self.planner_speed[i]:.3f}", f"{self.r_mm[i]:.4f}", f"{self.surface_speed_mm_s[i]:.3f}",
                    f"{self.x_wp_mm[i]:.4f}", f"{self.y_wp_mm[i]:.4f}", f"{self.valve_deg[i]:.2f}",
                    int(self.valve_open[i]), self.seg_kind[i],
                ])


def parse_gcode_lines(path: Path) -> list[tuple[int, str]]:
    lines: list[tuple[int, str]] = []
    for idx, raw in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        code = raw.split(";", 1)[0].strip()
        if code:
            lines.append((idx, code))
    return lines


def _eval_trapezoid(seg: Segment, tau: float) -> tuple[float, float, float]:
    """Return (distance_traveled, scalar_speed, scalar_accel) at local time tau in [0, seg.duration]."""
    if seg.duration <= 0.0:
        return 0.0, 0.0, 0.0
    tau = max(0.0, min(seg.duration, tau))
    ta, tc, td = seg.accel_t, seg.cruise_t, seg.decel_t
    a = seg.accel
    v0 = seg.start_v
    vc = seg.cruise_v
    sa = v0 * ta + 0.5 * a * ta * ta
    sc = vc * tc
    if tau < ta:
        return v0 * tau + 0.5 * a * tau * tau, v0 + a * tau, a
    if tau <= ta + tc:
        dt_c = tau - ta
        return sa + vc * dt_c, vc, 0.0
    dt_d = tau - ta - tc
    s = sa + sc + vc * dt_d - 0.5 * a * dt_d * dt_d
    v = max(0.0, vc - a * dt_d)
    return s, v, (-a if td > 0.0 else 0.0)


def simulate_gcode(
    gcode_path: Path,
    cfg: MachineConfig | None = None,
    dt: float = 0.02,
    initial_pos: tuple[float, float, float] | None = None,
    valve_response_ms: float = 0.0,
) -> SimulationResult:
    """Simulate a G-code file through Klipper GCodeMove + LookAheadQueue + Polar kinematics."""
    if cfg is None:
        cfg = load_machine_config()
    klip = _load_klipper_classes()

    # Build Klipper ToolHead stub matching printer.cfg
    scv = cfg.square_corner_velocity
    jd = (scv * scv) * (math.sqrt(2.0) - 1.0) / cfg.max_accel
    mcr_pseudo_accel = cfg.max_accel * (1.0 - cfg.minimum_cruise_ratio)
    toolhead = SimpleNamespace(
        max_accel=cfg.max_accel,
        max_velocity=cfg.max_velocity,
        junction_deviation=jd,
        mcr_pseudo_accel=mcr_pseudo_accel,
        extra_axes=[],
    )

    # Initialize GCodeMove
    g = klip["GCodeMove"].__new__(klip["GCodeMove"])
    start_xyz = initial_pos if initial_pos is not None else cfg.home_pos
    machine = [float(start_xyz[0]), float(start_xyz[1]), float(start_xyz[2]), 0.0]
    g.is_printer_ready = True
    g.absolute_coord = True
    g.absolute_extrude = True
    g.axis_map = {"X": 0, "Y": 1, "Z": 2, "E": 3}
    g.homing_position = [0.0, 0.0, 0.0, 0.0]
    g.base_position = [0.0, 0.0, 0.0, 0.0]
    g.last_position = machine.copy()
    g.speed_factor = 1.0 / 60.0
    g.speed = 25.0  # Default 1500 mm/min
    g.extrude_factor = 1.0
    g.position_with_transform = lambda: machine.copy()

    queue = klip["LookAheadQueue"]()
    segments: list[Segment] = []
    events: list[SimEvent] = []
    warnings: list[str] = []

    sim_time = 0.0
    homed: set[int] = set()
    valve_state = "unknown"
    valve_target_deg = cfg.valve_closed_angle
    cur_line = 0
    cur_text = ""

    def flush_queue() -> None:
        nonlocal sim_time
        for m in queue.flush():
            dur = m.accel_t + m.cruise_t + m.decel_t
            meta = m.sim_meta
            segments.append(
                Segment(
                    kind="move",
                    line=meta["line"],
                    command=meta["command"],
                    t_start=sim_time,
                    duration=dur,
                    start_pos=tuple(m.start_pos[:3]),
                    end_pos=tuple(m.end_pos[:3]),
                    start_v=m.start_v,
                    cruise_v=m.cruise_v,
                    end_v=m.end_v,
                    accel=m.accel,
                    accel_t=m.accel_t,
                    cruise_t=m.cruise_t,
                    decel_t=m.decel_t,
                    valve_target_deg=meta["valve_deg"],
                    valve_state=meta["valve_state"],
                )
            )
            sim_time += dur

    def enqueue_move(end: list[float], speed: float) -> None:
        nonlocal machine
        if machine[:3] != end[:3]:
            moved_axes = [i for i in range(3) if abs(machine[i] - end[i]) > 1e-9]
            for ax in moved_axes:
                if ax not in homed:
                    warnings.append(f"L{cur_line}: moved unhomed axis {'XYZ'[ax]} ({cur_text})")
            if 2 in moved_axes and len(moved_axes) > 1:
                warnings.append(f"L{cur_line}: simultaneous Z+XY diagonal move ({cur_text})")
            bounds_lo = (cfg.x_min, cfg.y_min, cfg.z_min)
            bounds_hi = (cfg.x_max, cfg.y_max, cfg.safe_z_max)
            for i in range(3):
                if end[i] < bounds_lo[i] - 1e-9 or end[i] > bounds_hi[i] + 1e-9:
                    warnings.append(
                        f"L{cur_line}: axis {'XYZ'[i]}={end[i]:.3f} outside safe [{bounds_lo[i]}, {bounds_hi[i]}]"
                    )

            m = klip["Move"](toolhead, machine.copy(), list(end), speed)
            if m.move_d > 0.0:
                if m.axes_d[2]:
                    ratio = m.move_d / abs(m.axes_d[2])
                    m.limit_speed(cfg.max_z_velocity * ratio, cfg.max_z_accel * ratio)
                m.sim_meta = {
                    "line": cur_line,
                    "command": cur_text,
                    "valve_deg": valve_target_deg,
                    "valve_state": valve_state,
                }
                queue.add_move(m)
        machine = list(end)

    g.move_with_transform = enqueue_move

    for line, text in parse_gcode_lines(gcode_path):
        cur_line = line
        cur_text = text
        op = text.split()[0].upper()
        cmd = _GCodeCommand(text)

        if op == "G28":
            flush_queue()
            axes = [i for i, a in enumerate("XYZ") if a in cmd.params] or [0, 1, 2]
            if any(a != 2 for a in axes) and 2 not in homed:
                warnings.append(f"L{line}: X/Y homed before Z ({text})")
            events.append(SimEvent(sim_time, line, "G28", text, tuple(machine[:3])))
            # Homing axes sequentially in Klipper Cartesian order (X, Y, Z or requested order)
            homing_speeds = (cfg.x_homing_speed, cfg.y_homing_speed, cfg.z_homing_speed)
            homing_accels = (cfg.max_accel, cfg.max_accel, cfg.max_z_accel)
            for ax in axes:
                target = cfg.home_pos[ax]
                dist = abs(target - machine[ax])
                if dist > 1e-9:
                    end_h = machine.copy()
                    end_h[ax] = target
                    v_h = homing_speeds[ax]
                    a_h = homing_accels[ax]
                    # Trapezoidal or triangular homing profile
                    ta = v_h / a_h
                    if dist >= v_h * ta:
                        tc = (dist - v_h * ta) / v_h
                        td = ta
                        vc = v_h
                    else:
                        ta = math.sqrt(dist / a_h)
                        tc = 0.0
                        td = ta
                        vc = a_h * ta
                    dur = ta + tc + td
                    segments.append(
                        Segment(
                            kind="home",
                            line=line,
                            command=f"{text} ({'XYZ'[ax]})",
                            t_start=sim_time,
                            duration=dur,
                            start_pos=tuple(machine[:3]),
                            end_pos=tuple(end_h[:3]),
                            start_v=0.0,
                            cruise_v=vc,
                            end_v=0.0,
                            accel=a_h,
                            accel_t=ta,
                            cruise_t=tc,
                            decel_t=td,
                            valve_target_deg=valve_target_deg,
                            valve_state=valve_state,
                        )
                    )
                    sim_time += dur
                    machine = end_h
                homed.add(ax)
            g._handle_home_rails_end(SimpleNamespace(get_axes=lambda a=axes: a), [])

        elif op in ("QUERY_ENDSTOPS", "M119"):
            flush_queue()
            events.append(SimEvent(sim_time, line, op, text, tuple(machine[:3]),
                                   "Sensor state unavailable: manual hardware check required"))
            warnings.append(f"L{line}: endstop query not simulated; physical sensor state unknown")

        elif op == "M118":
            flush_queue()
            events.append(SimEvent(sim_time, line, op, text, tuple(machine[:3]), text[4:].strip()))

        elif op == "M400":
            flush_queue()
            events.append(SimEvent(sim_time, line, "M400", text, tuple(machine[:3])))

        elif op == "G4":
            flush_queue()
            p_ms = cmd.get_float("P", default=0.0) or 0.0
            dur = max(0.0, p_ms / 1000.0)
            events.append(SimEvent(sim_time, line, "G4", text, tuple(machine[:3]), f"{dur:.3f}s"))
            if dur > 0.0:
                segments.append(
                    Segment(
                        kind="dwell",
                        line=line,
                        command=text,
                        t_start=sim_time,
                        duration=dur,
                        start_pos=tuple(machine[:3]),
                        end_pos=tuple(machine[:3]),
                        start_v=0.0,
                        cruise_v=0.0,
                        end_v=0.0,
                        accel=0.0,
                        accel_t=0.0,
                        cruise_t=dur,
                        decel_t=0.0,
                        valve_target_deg=valve_target_deg,
                        valve_state=valve_state,
                    )
                )
                sim_time += dur

        elif op in ("BALL_VALVE_OPEN", "BALL_VALVE_CLOSED", "BALL_VALVE"):
            # Note: in real Klipper, SET_SERVO executes at current lookahead print_time;
            # flushing isn't forced by SET_SERVO itself unless preceded by M400.
            if not queue.is_empty():
                warnings.append(f"L{line}: {op} issued with non-empty motion queue (missing M400)")
            if op == "BALL_VALVE_OPEN":
                valve_state = "open"
                valve_target_deg = cfg.valve_open_angle
            elif op == "BALL_VALVE_CLOSED":
                valve_state = "closed"
                valve_target_deg = cfg.valve_closed_angle
            else:
                pct = max(0.0, min(100.0, cmd.get_float("P", default=0.0) or 0.0))
                valve_target_deg = pct * 150.0 / 100.0
                valve_state = "closed" if pct == 0.0 else "open"
            events.append(
                SimEvent(sim_time, line, op, text, tuple(machine[:3]), f"angle={valve_target_deg:.1f}deg")
            )

        elif op in ("G0", "G1"):
            g.cmd_G1(cmd)

        elif op in ("G21", "G90", "G91", "G92", "M220", "SET_GCODE_OFFSET"):
            getattr(g, f"cmd_{op}")(cmd)

        else:
            warnings.append(f"L{line}: unhandled command ignored in simulator: {text}")

    flush_queue()

    # Sample uniform time-series merged with exact segment boundaries so every vertex is exact
    total_time = sim_time
    if total_time <= 0.0:
        t_arr = np.array([0.0], dtype=float)
    else:
        n_steps = max(2, int(math.ceil(total_time / dt)) + 1)
        t_uniform = np.linspace(0.0, total_time, n_steps)
        boundaries = [0.0, total_time]
        for s in segments:
            boundaries.append(s.t_start)
            boundaries.append(s.t_start + s.duration)
        t_merged = np.sort(np.concatenate([t_uniform, np.array(boundaries, dtype=float)]))
        # Deduplicate points closer than 1e-6 s
        keep = np.concatenate([[True], np.diff(t_merged) > 1e-6])
        t_arr = t_merged[keep]

    n = len(t_arr)
    x_deg = np.zeros(n)
    y_mm = np.zeros(n)
    z_mm = np.zeros(n)
    vx = np.zeros(n)
    vy = np.zeros(n)
    vz = np.zeros(n)
    ax_arr = np.zeros(n)
    ay_arr = np.zeros(n)
    az_arr = np.zeros(n)
    v_plan = np.zeros(n)
    v_target_arr = np.zeros(n)
    seg_kind: list[str] = ["idle"] * n

    if not segments:
        x_deg[:] = machine[0]
        y_mm[:] = machine[1]
        z_mm[:] = machine[2]
        v_target_arr[:] = valve_target_deg
    else:
        seg_idx = 0
        n_seg = len(segments)
        for i, ti in enumerate(t_arr):
            while seg_idx + 1 < n_seg and ti > segments[seg_idx].t_start + segments[seg_idx].duration + 1e-12:
                seg_idx += 1
            seg = segments[seg_idx]
            seg_kind[i] = seg.kind
            v_target_arr[i] = seg.valve_target_deg
            tau = ti - seg.t_start
            dx = seg.end_pos[0] - seg.start_pos[0]
            dy = seg.end_pos[1] - seg.start_pos[1]
            dz = seg.end_pos[2] - seg.start_pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if seg.kind == "dwell" or dist < 1e-12:
                x_deg[i], y_mm[i], z_mm[i] = seg.end_pos
                vx[i] = vy[i] = vz[i] = v_plan[i] = 0.0
                ax_arr[i] = ay_arr[i] = az_arr[i] = 0.0
            else:
                s, v, a_sc = _eval_trapezoid(seg, tau)
                frac = max(0.0, min(1.0, s / dist))
                x_deg[i] = seg.start_pos[0] + dx * frac
                y_mm[i] = seg.start_pos[1] + dy * frac
                z_mm[i] = seg.start_pos[2] + dz * frac
                v_plan[i] = v
                ux, uy, uz = dx / dist, dy / dist, dz / dist
                vx[i] = ux * v
                vy[i] = uy * v
                vz[i] = uz * v
                ax_arr[i] = ux * a_sc
                ay_arr[i] = uy * a_sc
                az_arr[i] = uz * a_sc

    # Apply optional first-order / rate-limited servo response model to valve angle
    valve_deg = v_target_arr.copy()
    if valve_response_ms > 0.0 and n > 1:
        max_rate_deg_s = max(1e-3, cfg.valve_open_angle / (valve_response_ms / 1000.0))
        valve_deg[0] = v_target_arr[0]
        for i in range(1, n):
            step_dt = t_arr[i] - t_arr[i - 1]
            delta = v_target_arr[i] - valve_deg[i - 1]
            max_step = max_rate_deg_s * step_dt
            if abs(delta) <= max_step:
                valve_deg[i] = v_target_arr[i]
            else:
                valve_deg[i] = valve_deg[i - 1] + math.copysign(max_step, delta)

    valve_open = valve_deg > (0.5 * (cfg.valve_closed_angle + cfg.valve_open_angle))

    # Cylindrical / polar kinematics transform
    # Matches physical CW workpiece layout (Worker-Image.jpeg) and +Z CCW turntable rotation:
    #   x_wp = r * cos(X),  y_wp = -r * sin(X)
    r_mm = cfg.r_center_mm - y_mm
    theta_rad = np.deg2rad(x_deg)
    omega_rad_s = np.deg2rad(vx)
    x_wp_mm = r_mm * np.cos(theta_rad)
    y_wp_mm = -r_mm * np.sin(theta_rad)
    # Relative linear speed between nozzle and rotating workpiece surface
    surface_speed_mm_s = np.sqrt((r_mm * omega_rad_s) ** 2 + vy ** 2)

    return SimulationResult(
        file_name=gcode_path.name,
        file_path=str(gcode_path),
        config=cfg,
        dt=dt,
        valve_response_ms=valve_response_ms,
        segments=segments,
        events=events,
        warnings=warnings,
        t=t_arr,
        x_deg=x_deg,
        y_mm=y_mm,
        z_mm=z_mm,
        vx_deg_s=vx,
        vy_mm_s=vy,
        vz_mm_s=vz,
        ax_deg_s2=ax_arr,
        ay_mm_s2=ay_arr,
        az_mm_s2=az_arr,
        planner_speed=v_plan,
        r_mm=r_mm,
        surface_speed_mm_s=surface_speed_mm_s,
        x_wp_mm=x_wp_mm,
        y_wp_mm=y_wp_mm,
        valve_deg=valve_deg,
        valve_open=valve_open,
        seg_kind=seg_kind,
    )
