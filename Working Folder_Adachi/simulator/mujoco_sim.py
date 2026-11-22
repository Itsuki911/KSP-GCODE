"""MuJoCo 3D physics, collision/clearance checker, and interactive viewer for KSP-GCODE.

Supports:
  - Parametric 3D workpiece modeled after Working Folder_Adachi/Worker-Image.jpeg
    (enabled by default; disable with use_workpiece=False or override with STL).
  - Headless verification: checks contacts, exact signed nozzle-to-platter AND
    nozzle-to-workpiece clearance via mj_geomDistance, plus inverse dynamics.
  - Interactive 3D GUI viewer (`mjpython ... --view`): renders BOTH the rotating
    3D workpiece AND the bright G-code toolpath lines simultaneously in 3D space!
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from klipper_sim import SimulationResult
from workpiece_model import DEFAULT_WORKPIECE, build_mujoco_workpiece_xml

MODEL_XML_PATH = Path(__file__).resolve().parent / "ksp_machine.xml"


@dataclass
class MuJoCoReport:
    file_name: str
    samples: int
    contact_samples: int
    max_contacts: int
    min_bed_clearance_mm: float
    min_bed_clearance_t_s: float
    min_workpiece_clearance_mm: float | None
    min_workpiece_clearance_t_s: float | None
    min_workpiece_geom: str | None
    peak_turntable_torque_nm: float
    peak_y_force_n: float
    peak_z_force_n: float
    workpiece_mode: str
    collision_free: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "file_name": self.file_name,
            "samples": self.samples,
            "contact_samples": self.contact_samples,
            "max_contacts": self.max_contacts,
            "min_bed_clearance_mm": round(self.min_bed_clearance_mm, 3),
            "min_bed_clearance_t_s": round(self.min_bed_clearance_t_s, 3),
            "min_workpiece_clearance_mm": (
                round(self.min_workpiece_clearance_mm, 3)
                if self.min_workpiece_clearance_mm is not None
                else None
            ),
            "min_workpiece_clearance_t_s": (
                round(self.min_workpiece_clearance_t_s, 3)
                if self.min_workpiece_clearance_t_s is not None
                else None
            ),
            "min_workpiece_geom": self.min_workpiece_geom,
            "peak_turntable_torque_nm": round(self.peak_turntable_torque_nm, 4),
            "peak_y_force_n": round(self.peak_y_force_n, 3),
            "peak_z_force_n": round(self.peak_z_force_n, 3),
            "workpiece_mode": self.workpiece_mode,
            "collision_free": self.collision_free,
        }


def build_mujoco_model(
    r_center_mm: float = 180.0,
    use_workpiece: bool = True,
    workpiece_stl: Path | None = None,
    workpiece_scale: float = 0.001,
    workpiece_z_offset_mm: float = 0.0,
) -> tuple[mujoco.MjModel, str]:
    """Load ksp_machine.xml and optionally attach parametric workpiece or custom STL."""
    xml = MODEL_XML_PATH.read_text(encoding="utf-8")
    r_m = r_center_mm / 1000.0
    xml = xml.replace('pos="0.180 0 0.162"', f'pos="{r_m:.6f} 0 0.162"')

    mode = "none"
    if workpiece_stl is not None:
        stl_abs = str(workpiece_stl.resolve())
        mesh_asset = (
            f'    <mesh name="workpiece_mesh" file="{stl_abs}" '
            f'scale="{workpiece_scale} {workpiece_scale} {workpiece_scale}"/>\n  </asset>'
        )
        xml = xml.replace("  </asset>", mesh_asset, 1)
        wp_geom = (
            f'<geom name="wp_stl_geom" type="mesh" mesh="workpiece_mesh" '
            f'pos="0 0 {workpiece_z_offset_mm / 1000.0:.6f}" '
            f'material="workpiece_mat" contype="1" conaffinity="1"/>'
        )
        xml = xml.replace("<!-- WORKPIECE_PLACEHOLDER -->", wp_geom, 1)
        mode = f"stl:{workpiece_stl.name}"
    elif use_workpiece:
        wp_xml = build_mujoco_workpiece_xml(DEFAULT_WORKPIECE)
        xml = xml.replace("<!-- WORKPIECE_PLACEHOLDER -->", wp_xml, 1)
        mode = "parametric_worker_image"

    return mujoco.MjModel.from_xml_string(xml), mode


def _joint_indices(model: mujoco.MjModel) -> dict[str, tuple[int, int]]:
    out: dict[str, tuple[int, int]] = {}
    for name in ("turntable_x", "carriage_y", "head_z", "ball_valve"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert jid >= 0, f"Joint {name} not found in MuJoCo model"
        out[name] = (int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid]))
    return out


def _workpiece_collidable_geoms(model: mujoco.MjModel) -> list[tuple[int, str]]:
    geoms: list[tuple[int, str]] = []
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if name.startswith("wp_") and model.geom_contype[gid] != 0:
            geoms.append((gid, name))
    return geoms


def run_mujoco_headless(
    sim: SimulationResult,
    use_workpiece: bool = True,
    workpiece_stl: Path | None = None,
    workpiece_scale: float = 0.001,
    workpiece_z_offset_mm: float = 0.0,
) -> MuJoCoReport:
    """Replay SimulationResult through MuJoCo headlessly to verify clearances & dynamics."""
    model, wp_mode = build_mujoco_model(
        r_center_mm=sim.config.r_center_mm,
        use_workpiece=use_workpiece,
        workpiece_stl=workpiece_stl,
        workpiece_scale=workpiece_scale,
        workpiece_z_offset_mm=workpiece_z_offset_mm,
    )
    data = mujoco.MjData(model)
    jidx = _joint_indices(model)

    nozzle_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "nozzle_tip")
    nozzle_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "nozzle_body")
    platter_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "platter_top")
    wp_geoms = _workpiece_collidable_geoms(model)

    n = len(sim.t)
    qx = np.deg2rad(sim.x_deg)
    qy = sim.y_mm / 1000.0
    qz = sim.z_mm / 1000.0
    qv = np.deg2rad(sim.valve_deg)

    vx = np.deg2rad(sim.vx_deg_s)
    vy = sim.vy_mm_s / 1000.0
    vz = sim.vz_mm_s / 1000.0
    ax = np.deg2rad(sim.ax_deg_s2)
    ay = sim.ay_mm_s2 / 1000.0
    az = sim.az_mm_s2 / 1000.0

    contact_samples = 0
    max_contacts = 0
    min_bed_m = float("inf")
    min_bed_t = 0.0
    min_wp_m = float("inf") if wp_geoms else None
    min_wp_t = 0.0 if wp_geoms else None
    min_wp_name: str | None = None
    peak_tx = 0.0
    peak_fy = 0.0
    peak_fz = 0.0

    for i in range(n):
        data.qpos[jidx["turntable_x"][0]] = qx[i]
        data.qpos[jidx["carriage_y"][0]] = qy[i]
        data.qpos[jidx["head_z"][0]] = qz[i]
        data.qpos[jidx["ball_valve"][0]] = qv[i]

        data.qvel[jidx["turntable_x"][1]] = vx[i]
        data.qvel[jidx["carriage_y"][1]] = vy[i]
        data.qvel[jidx["head_z"][1]] = vz[i]
        data.qvel[jidx["ball_valve"][1]] = 0.0

        # Run forward kinematics & collision detection first (mj_forward overwrites qacc)
        mujoco.mj_forward(model, data)

        # Set desired trajectory accelerations AFTER mj_forward, then run inverse dynamics
        data.qacc[jidx["turntable_x"][1]] = ax[i]
        data.qacc[jidx["carriage_y"][1]] = ay[i]
        data.qacc[jidx["head_z"][1]] = az[i]
        data.qacc[jidx["ball_valve"][1]] = 0.0
        mujoco.mj_inverse(model, data)

        # Count only nozzle-vs-platter/workpiece contacts (ignore internal turntable mounts)
        nozzle_contacts = 0
        for ci in range(data.ncon):
            con = data.contact[ci]
            g1, g2 = int(con.geom1), int(con.geom2)
            if g1 in (nozzle_tip_id, nozzle_body_id) or g2 in (nozzle_tip_id, nozzle_body_id):
                nozzle_contacts += 1
        if nozzle_contacts > 0:
            contact_samples += 1
            max_contacts = max(max_contacts, nozzle_contacts)

        d_bed = min(
            mujoco.mj_geomDistance(model, data, nozzle_tip_id, platter_id, 1.0, None),
            mujoco.mj_geomDistance(model, data, nozzle_body_id, platter_id, 1.0, None),
        )
        if d_bed < min_bed_m:
            min_bed_m = float(d_bed)
            min_bed_t = float(sim.t[i])

        if wp_geoms and min_wp_m is not None:
            for gid, gname in wp_geoms:
                d_g = min(
                    mujoco.mj_geomDistance(model, data, nozzle_tip_id, gid, 1.0, None),
                    mujoco.mj_geomDistance(model, data, nozzle_body_id, gid, 1.0, None),
                )
                if d_g < min_wp_m:
                    min_wp_m = float(d_g)
                    min_wp_t = float(sim.t[i])
                    min_wp_name = gname

        peak_tx = max(peak_tx, abs(float(data.qfrc_inverse[jidx["turntable_x"][1]])))
        peak_fy = max(peak_fy, abs(float(data.qfrc_inverse[jidx["carriage_y"][1]])))
        peak_fz = max(peak_fz, abs(float(data.qfrc_inverse[jidx["head_z"][1]])))

    collision_free = (
        contact_samples == 0
        and min_bed_m >= -1e-6
        and (min_wp_m is None or min_wp_m >= -1e-6)
    )
    return MuJoCoReport(
        file_name=sim.file_name,
        samples=n,
        contact_samples=contact_samples,
        max_contacts=max_contacts,
        min_bed_clearance_mm=min_bed_m * 1000.0,
        min_bed_clearance_t_s=min_bed_t,
        min_workpiece_clearance_mm=(min_wp_m * 1000.0 if min_wp_m is not None else None),
        min_workpiece_clearance_t_s=min_wp_t,
        min_workpiece_geom=min_wp_name,
        peak_turntable_torque_nm=peak_tx,
        peak_y_force_n=peak_fy,
        peak_z_force_n=peak_fz,
        workpiece_mode=wp_mode,
        collision_free=collision_free,
    )


def _draw_trajectory_overlay(
    user_scn: mujoco.MjvScene,
    pts_wp_m: np.ndarray,
    valve_open: np.ndarray,
    theta_rad: float,
    cur_idx: int,
) -> None:
    """Render the G-code toolpath lines rotating WITH the workpiece in MuJoCo user_scn."""
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    # Transform workpiece-frame points to current world frame via R_z(+theta)
    wx = c * pts_wp_m[:, 0] - s * pts_wp_m[:, 1]
    wy = s * pts_wp_m[:, 0] + c * pts_wp_m[:, 1]
    wz = pts_wp_m[:, 2]

    max_geoms = user_scn.maxgeom
    n_pts = len(wx)
    n_seg = max(0, n_pts - 1)
    # Downsample segments cleanly if trajectory has more points than user_scn capacity
    stride = max(1, int(math.ceil(n_seg / max(1, min(600, max_geoms - 8)))))

    user_scn.ngeom = 0
    for k in range(0, n_seg, stride):
        if user_scn.ngeom >= max_geoms - 2:
            break
        k2 = min(n_pts - 1, k + stride)
        p1 = np.array([wx[k], wy[k], wz[k]], dtype=np.float64)
        p2 = np.array([wx[k2], wy[k2], wz[k2]], dtype=np.float64)
        if np.linalg.norm(p2 - p1) < 1e-5:
            continue
        is_open = bool(valve_open[k]) or bool(valve_open[k2])
        passed = k <= cur_idx
        if is_open:
            rgba = np.array([1.0, 0.42, 0.05, 0.95 if passed else 0.45], dtype=np.float32)
            radius = 0.0020 if passed else 0.0014
        else:
            rgba = np.array([0.22, 0.78, 0.98, 0.85 if passed else 0.30], dtype=np.float32)
            radius = 0.0011 if passed else 0.0008
        g = user_scn.geoms[user_scn.ngeom]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3),
            np.zeros(3),
            np.eye(3).flatten(),
            rgba,
        )
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, p1, p2)
        user_scn.ngeom += 1


def launch_mujoco_viewer(
    sim: SimulationResult,
    speed: float = 1.0,
    loop: bool = False,
    use_workpiece: bool = True,
    workpiece_stl: Path | None = None,
    workpiece_scale: float = 0.001,
    workpiece_z_offset_mm: float = 0.0,
) -> None:
    """Launch interactive 3D MuJoCo GUI viewer with 3D workpiece + live toolpath lines."""
    import mujoco.viewer

    model, wp_mode = build_mujoco_model(
        r_center_mm=sim.config.r_center_mm,
        use_workpiece=use_workpiece,
        workpiece_stl=workpiece_stl,
        workpiece_scale=workpiece_scale,
        workpiece_z_offset_mm=workpiece_z_offset_mm,
    )
    data = mujoco.MjData(model)
    jidx = _joint_indices(model)

    qx = np.deg2rad(sim.x_deg)
    qy = sim.y_mm / 1000.0
    qz = sim.z_mm / 1000.0
    qv = np.deg2rad(sim.valve_deg)
    pts_wp_m = np.column_stack([sim.x_wp_mm, sim.y_wp_mm, sim.z_mm]) / 1000.0
    total_t = float(sim.t[-1]) if len(sim.t) else 0.0

    print(
        f"[MuJoCo Viewer] Replaying {sim.file_name} "
        f"(duration={total_t:.2f}s, speed={speed}x, workpiece={wp_mode})"
    )
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.02, 0.0, 0.03]
        viewer.cam.distance = 0.56
        viewer.cam.elevation = -34.0
        viewer.cam.azimuth = 90.0  # Matches Worker-Image.jpeg front view (Purge pad at back/top)

        while viewer.is_running():
            t0 = time.perf_counter()
            while viewer.is_running():
                sim_t = (time.perf_counter() - t0) * speed
                if sim_t >= total_t:
                    idx = len(sim.t) - 1
                else:
                    idx = int(np.searchsorted(sim.t, sim_t, side="right")) - 1
                    idx = max(0, min(len(sim.t) - 1, idx))

                data.qpos[jidx["turntable_x"][0]] = qx[idx]
                data.qpos[jidx["carriage_y"][0]] = qy[idx]
                data.qpos[jidx["head_z"][0]] = qz[idx]
                data.qpos[jidx["ball_valve"][0]] = qv[idx]
                mujoco.mj_forward(model, data)

                _draw_trajectory_overlay(viewer.user_scn, pts_wp_m, sim.valve_open, float(qx[idx]), idx)
                viewer.sync()

                if sim_t >= total_t:
                    break
                time.sleep(0.01)

            if not loop:
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.05)
                break
