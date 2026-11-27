"""Optical marker detection, 20 mm NavMap2D grid, explore, and self-docking controller."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import dock as dock_model
from . import mjcf, params

MM = 0.001
DOCK_MARKER_HALF_M = 0.023
DOCK_MARKER_X_MM = 93.205
DOCK_MARKER_Z_M = 0.0485
CUBE_MARKER_HALF_M = 0.0115

CELL_MM = 20.0
GRID_HALF = 25
GRID_DIM = 2 * GRID_HALF
CELL_UNKNOWN = 0
CELL_CLEAR = 1
CELL_PROX = 2
CELL_CUBE = 3
CELL_CLIFF = 4
CELL_CHARGER = 5
BACKUP_SPEEDS_MMPS = (-102.0, -112.0, -96.0, -122.0, -106.0)  # by wiggle count, as before
BACKUP_MIN_TICKS = 320
BACKUP_MARGIN = 1.3
SEAT_DEPTH_MM = 30.0  # the seated robot's origin is this far behind the lip (web/src/sim/robot.js _localizeOnCharger)
LIFT_UP_WAIT_TICKS = 200  # control ticks (1 s): the longest the dock waits for the lift to reach its backing height
# Control ticks (1.65 s) a retry may take pulling forward to its goal. At 165 the pull stopped about 47 mm out, short of
# its 78 or 90 mm goal with the rear skids still on the dock's lip; the alignment turn after it could not turn there,
# and the next backup set off 7.1 to 8.8 degrees off the dock's axis (web/src/sim/autonomy.js, the browser's dock).
PULL_MAX_TICKS = 330

DOCK_CORNERS_LOCAL = (
    (-0.0005, DOCK_MARKER_HALF_M, DOCK_MARKER_HALF_M),
    (-0.0005, -DOCK_MARKER_HALF_M, DOCK_MARKER_HALF_M),
    (-0.0005, -DOCK_MARKER_HALF_M, -DOCK_MARKER_HALF_M),
    (-0.0005, DOCK_MARKER_HALF_M, -DOCK_MARKER_HALF_M),
)


def _wrap(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def _clamp(value: float, low: float, high: float) -> float:
    assert low <= high, "invalid clamp bounds"
    return min(max(value, low), high)


def load_camera_spec() -> Dict[str, Any]:
    """Load Vector 640x360 pinhole factory calibration from web/camera_calibration.json."""
    cal_path = Path(__file__).resolve().parent.parent / "web" / "camera_calibration.json"
    cal = json.loads(cal_path.read_text(encoding="utf-8"))
    v = params.values(params.load())
    return {
        "name": "head_cam", "width_px": int(cal["width_px"]), "height_px": int(cal["height_px"]),
        "fx_px": float(cal["fx_px"]), "fy_px": float(cal["fy_px"]),
        "cx_px": float(cal["cx_px"]), "cy_px": float(cal["cy_px"]),
        "near_mm": 5.0, "pitch_down_deg": float(v["head_camera_pitch_down_deg"]),
        "pitch_down_in_model": True, "rgb_pitch_up_deg": 0.0,
    }


def pixel_of(spec: Dict[str, Any], p_cam: Sequence[float]) -> Optional[Tuple[float, float]]:
    """Project camera-frame meters point (x right, y up, -z forward) to pixel coordinates."""
    depth = -float(p_cam[2])
    if depth <= float(spec.get("near_mm", 5.0)) * MM:
        return None
    u = float(spec["cx_px"]) + float(spec["fx_px"]) * (float(p_cam[0]) / depth)
    v = float(spec["cy_px"]) - float(spec["fy_px"]) * (float(p_cam[1]) / depth)
    return (u, v)


def ray_of(spec: Dict[str, Any], u: float, v: float, depth: float = 1.0) -> np.ndarray:
    """Invert pixel coordinates back into a camera-frame ray of given forward depth."""
    x = ((u - float(spec["cx_px"])) / float(spec["fx_px"])) * depth
    y = ((float(spec["cy_px"]) - v) / float(spec["fy_px"])) * depth
    return np.array([x, y, -depth], dtype=float)


class NavMap2D:
    """20 mm table-centered occupancy grid with sticky cliff cells and landmark poses."""

    def __init__(self) -> None:
        self.cells = [CELL_UNKNOWN] * (GRID_DIM * GRID_DIM)
        self.charger_pose: Optional[Dict[str, Any]] = None
        self.cube_pose: Optional[Dict[str, Any]] = None
        self.last_frontier: Optional[Tuple[float, float]] = None

    def reset(self) -> None:
        self.cells = [CELL_UNKNOWN] * (GRID_DIM * GRID_DIM)
        self.charger_pose = None
        self.cube_pose = None
        self.last_frontier = None

    def _idx(self, x_mm: float, y_mm: float) -> int:
        gx = int(math.floor(x_mm / CELL_MM)) + GRID_HALF
        gy = int(math.floor(y_mm / CELL_MM)) + GRID_HALF
        if gx < 0 or gx >= GRID_DIM or gy < 0 or gy >= GRID_DIM:
            return -1
        return gy * GRID_DIM + gx

    def cell_center(self, idx: int) -> Tuple[float, float]:
        gx, gy = idx % GRID_DIM, idx // GRID_DIM
        return ((gx - GRID_HALF + 0.5) * CELL_MM, (gy - GRID_HALF + 0.5) * CELL_MM)

    def set_cell(self, x_mm: float, y_mm: float, kind: int) -> None:
        idx = self._idx(x_mm, y_mm)
        if idx >= 0 and (kind == CELL_CLIFF or self.cells[idx] != CELL_CLIFF):
            self.cells[idx] = kind

    def get_cell(self, x_mm: float, y_mm: float) -> int:
        idx = self._idx(x_mm, y_mm)
        return CELL_CLIFF if idx < 0 else self.cells[idx]

    def update_from_sensors(self, sim: Any) -> None:
        """Stamp footprint, active cliffs, and ToF ray clearance into the 20 mm grid."""
        s = sim.state
        c, si = math.cos(s.pose_angle_rad), math.sin(s.pose_angle_rad)
        for lx, ly in ((0.0, 0.0), (18.0, 0.0), (-18.0, 0.0), (0.0, 14.0), (0.0, -14.0)):
            self.set_cell(s.x_mm + c * lx - si * ly, s.y_mm + si * lx + c * ly, CELL_CLEAR)
        for flag, (ox, oy) in zip(sim.hazards.cliff, ((25.0, 22.0), (25.0, -22.0), (-42.0, 22.0), (-42.0, -22.0))):
            if flag:
                self.set_cell(s.x_mm + c * ox - si * oy, s.y_mm + si * ox + c * oy, CELL_CLIFF)
        if not s.prox_is_lift_in_fov and abs(s.pose_pitch_rad) <= 0.05:
            clear_d = max(0.0, s.prox_distance_mm - 20.0) if s.prox_found_object else (180.0 if s.prox_unobstructed else 0.0)
            for step in range(30, int(clear_d) + 1, int(CELL_MM)):
                self.set_cell(s.x_mm + c * (10.0 + step), s.y_mm + si * (10.0 + step), CELL_CLEAR)
            if s.prox_found_object and s.prox_distance_mm <= 380.0:
                hit_d = 10.0 + s.prox_distance_mm
                self.set_cell(s.x_mm + c * hit_d, s.y_mm + si * hit_d, CELL_PROX)

    def record_charger(self, obs: Dict[str, float], tick: int) -> None:
        self.charger_pose = {**obs, "seen_at": tick}
        lip_x, lip_y, ox, oy = obs["lip_x"], obs["lip_y"], obs["out_x"], obs["out_y"]
        for along in range(-20, -101, -20):
            for lat in (-36.0, -16.0, 4.0, 24.0):
                self.set_cell(lip_x - along * ox - lat * oy, lip_y - along * oy + lat * ox, CELL_CHARGER)

    def record_cube(self, obs: Dict[str, float], tick: int) -> None:
        self.cube_pose = {**obs, "seen_at": tick}
        self.set_cell(obs["x"], obs["y"], CELL_CUBE)

    def has_clearance(self, x_mm: float, y_mm: float, min_mm: float = 80.0, ignore_charger: bool = False) -> bool:
        for i, k in enumerate(self.cells):
            if k not in (CELL_CLIFF, CELL_PROX, CELL_CUBE, CELL_CHARGER):
                continue
            if ignore_charger and k == CELL_CHARGER:
                continue
            cx, cy = self.cell_center(i)
            pad = min_mm if k == CELL_CLIFF else (min(min_mm, 64.0) if k == CELL_CUBE else min(min_mm, 62.0))
            if math.hypot(x_mm - cx, y_mm - cy) < pad:
                return False
        return True

    def segment_clear(self, x0: float, y0: float, x1: float, y1: float, clearance_mm: float = 60.0,
                      ignore_charger: bool = False) -> bool:
        for step in range(1, 11):
            t = step / 10.0
            if not self.has_clearance(x0 + t * (x1 - x0), y0 + t * (y1 - y0), clearance_mm, ignore_charger):
                return False
        return True

    def score_unknown(self, x_mm: float, y_mm: float) -> int:
        gx0 = int(math.floor(x_mm / CELL_MM)) + GRID_HALF
        gy0 = int(math.floor(y_mm / CELL_MM)) + GRID_HALF
        unknown = 0
        for dy in range(-3, 4):
            for dx in range(-3, 4):
                gx, gy = gx0 + dx, gy0 + dy
                if 0 <= gx < GRID_DIM and 0 <= gy < GRID_DIM and self.cells[gy * GRID_DIM + gx] == CELL_UNKNOWN:
                    unknown += 1
        return unknown

    def counts(self) -> Tuple[int, int]:
        mapped = sum(1 for k in self.cells if k != CELL_UNKNOWN)
        cliffs = sum(1 for k in self.cells if k == CELL_CLIFF)
        return (mapped, cliffs)


def _camera_pose_odom(sim: Any, chassis_id: int, spec: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    import mujoco
    cam_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, spec["name"])
    if cam_id < 0:
        return None
    cw = sim.data.cam_xpos[cam_id].copy()
    raw_r = sim.data.cam_xmat[cam_id].reshape(3, 3)
    mount_down = 0.0 if spec.get("pitch_down_in_model") else float(spec["pitch_down_deg"])
    tilt = math.radians(float(spec.get("rgb_pitch_up_deg", 0.0)) - mount_down)
    ct, st = math.cos(tilt), math.sin(tilt)
    r_cam_w = raw_r @ np.array([[1.0, 0.0, 0.0], [0.0, ct, -st], [0.0, st, ct]], dtype=float)
    d_yaw = _wrap(sim.state.pose_angle_rad - sim._yaw())
    cd, sd = math.cos(d_yaw), math.sin(d_yaw)
    delta_w = cw - sim.data.xpos[chassis_id]
    cam_odom = np.array([
        sim.state.x_mm + (cd * float(delta_w[0]) - sd * float(delta_w[1])) / MM,
        sim.state.y_mm + (sd * float(delta_w[0]) + cd * float(delta_w[1])) / MM,
        float(delta_w[2]) / MM,
    ], dtype=float)
    rot_d = np.array([[cd, -sd, 0.0], [sd, cd, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return {"world": cw, "r_cam_w": r_cam_w, "odom_mm": cam_odom, "r_cam_odom": rot_d @ r_cam_w}


def _corners_visible(sim: Any, spec: Dict[str, Any], cam: Dict[str, Any],
                     world_corners: List[np.ndarray], out_normal_w: np.ndarray) -> Optional[List[Tuple[float, float]]]:
    import mujoco
    center = 0.25 * sum(world_corners)
    to_cam = cam["world"] - center
    dist = float(np.linalg.norm(to_cam))
    if dist < 0.04 or dist > 0.55:
        return None
    if float(np.dot(to_cam / dist, out_normal_w)) < math.cos(math.radians(70.0)):
        return None
    pixels: List[Tuple[float, float]] = []
    geomid = np.zeros(1, dtype=np.int32)
    for wc in world_corners:
        uv = pixel_of(spec, cam["r_cam_w"].T @ (wc - cam["world"]))
        if uv is None or uv[0] < 4.0 or uv[0] > spec["width_px"] - 4.0 or uv[1] < 4.0 or uv[1] > spec["height_px"] - 4.0:
            return None
        d_vec = wc - cam["world"]
        c_dist = float(np.linalg.norm(d_vec))
        hit = float(mujoco.mj_ray(sim.model, sim.data, cam["world"], d_vec / c_dist, sim._scene_only, 1, -1, geomid))
        if hit >= 0.0 and hit < c_dist - 0.020:
            return None
        pixels.append((round(uv[0] * 2.0) * 0.5, round(uv[1] * 2.0) * 0.5))
    return pixels


def _reconstruct_square(spec: Dict[str, Any], cam: Dict[str, Any],
                        pixels: List[Tuple[float, float]], size_mm: float) -> Optional[Tuple[float, float, float, float]]:
    rays = [cam["r_cam_odom"] @ ray_of(spec, u, v, 1.0) for u, v in pixels]
    pts: List[Tuple[float, float]] = []
    for top_i, bot_i in ((0, 3), (1, 2)):
        wt, wb = rays[top_i], rays[bot_i]
        h_top, h_bot = math.hypot(float(wt[0]), float(wt[1])), math.hypot(float(wb[0]), float(wb[1]))
        if h_top < 1e-4 or h_bot < 1e-4:
            return None
        slope = (float(wt[2]) / h_top) - (float(wb[2]) / h_bot)
        if slope <= 0.01:
            return None
        rng = size_mm / slope
        mx = 0.5 * (float(wt[0]) / h_top + float(wb[0]) / h_bot)
        my = 0.5 * (float(wt[1]) / h_top + float(wb[1]) / h_bot)
        pts.append((float(cam["odom_mm"][0]) + rng * mx, float(cam["odom_mm"][1]) + rng * my))
    cx = 0.5 * (pts[0][0] + pts[1][0])
    cy = 0.5 * (pts[0][1] + pts[1][1])
    dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
    chord = math.hypot(dx, dy)
    if chord < 5.0:
        return None
    nx, ny = -dy / chord, dx / chord
    if nx * (float(cam["odom_mm"][0]) - cx) + ny * (float(cam["odom_mm"][1]) - cy) < 0.0:
        nx, ny = -nx, -ny
    return (cx, cy, nx, ny)


def _detect_dock(sim: Any, dock_id: int, spec: Dict[str, Any], cam: Dict[str, Any]) -> Optional[Dict[str, float]]:
    d_pos = sim.data.xpos[dock_id]
    d_mat = sim.data.xmat[dock_id].reshape(3, 3)
    marker_w = d_pos + d_mat @ np.array([DOCK_MARKER_X_MM * MM, 0.0, DOCK_MARKER_Z_M], dtype=float)
    corners = [marker_w + d_mat @ np.array(lc, dtype=float) for lc in DOCK_CORNERS_LOCAL]
    px = _corners_visible(sim, spec, cam, corners, -d_mat[:, 0])
    if px is None:
        return None
    pose = _reconstruct_square(spec, cam, px, DOCK_MARKER_HALF_M * 2000.0)
    if pose is None:
        return None
    cx, cy, nx, ny = pose
    return {"lip_x": cx + DOCK_MARKER_X_MM * nx, "lip_y": cy + DOCK_MARKER_X_MM * ny,
            "out_x": nx, "out_y": ny, "marker_x": cx, "marker_y": cy}


def _detect_cube(sim: Any, cube_id: int, spec: Dict[str, Any], cam: Dict[str, Any]) -> Optional[Dict[str, float]]:
    c_pos = sim.data.xpos[cube_id]
    c_mat = sim.data.xmat[cube_id].reshape(3, 3)
    h = CUBE_MARKER_HALF_M
    for n in ((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, -1.0, 0.0)):
        nw = c_mat @ np.array(n, dtype=float)
        if abs(float(nw[2])) > 0.4:
            continue
        right = np.array([n[1], -n[0], 0.0], dtype=float)
        fc = np.array(n, dtype=float) * 0.0221
        corners = [c_pos + c_mat @ (fc + sr * h * right + np.array([0.0, 0.0, sz * h]))
                   for sr, sz in ((-1, 1), (1, 1), (1, -1), (-1, -1))]
        px = _corners_visible(sim, spec, cam, corners, nw)
        if px is None:
            continue
        pose = _reconstruct_square(spec, cam, px, h * 2000.0)
        if pose is not None:
            return {"x": pose[0] - 22.0 * pose[2], "y": pose[1] - 22.0 * pose[3]}
    return None


def observe_markers(sim: Any, spec: Optional[Dict[str, Any]] = None) -> Dict[str, Optional[Dict[str, float]]]:
    """Detect visible dock/cube fiducial markers through head_cam in odometry coordinates."""
    import mujoco
    spec = spec or load_camera_spec()
    if abs(float(sim._imu()[1][2])) > 1.2 or 38.0 < sim.lift_height_mm() < 62.0:
        return {"charger": None, "cube": None}
    chassis_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    cam = _camera_pose_odom(sim, chassis_id, spec)
    if cam is None:
        return {"charger": None, "cube": None}
    dock_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "dock")
    cube_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    charger = _detect_dock(sim, dock_id, spec, cam) if dock_id >= 0 else None
    cube = _detect_cube(sim, cube_id, spec, cam) if cube_id >= 0 else None
    return {"charger": charger, "cube": cube}


def _place_docked(sim: Any, head_deg: float, lift_mm: float) -> None:
    import mujoco
    xy, hd, z_mm, pitch_deg = dock_model.docked_robot_pose("docked")
    sim._place(xy, hd)
    yaw, pitch = math.radians(hd), math.radians(pitch_deg)
    sim.data.qpos[2] = z_mm * MM
    sim.data.qpos[3] = math.cos(0.5 * yaw) * math.cos(0.5 * pitch)
    sim.data.qpos[4] = -math.sin(0.5 * yaw) * math.sin(0.5 * pitch)
    sim.data.qpos[5] = math.cos(0.5 * yaw) * math.sin(0.5 * pitch)
    sim.data.qpos[6] = math.sin(0.5 * yaw) * math.cos(0.5 * pitch)
    mujoco.mj_forward(sim.model, sim.data)
    sim._yaw_prev = sim._yaw_total = sim._yaw()
    sim._pose_servos(head_deg, lift_mm)
    sim.charger_contact = True
    sim.battery_volts = 4.2


def make_dock_sim(start_xy_mm: Tuple[float, float] = (0.0, 0.0), heading_deg: float = 0.0,
                  head_deg: float = 4.0, lift_mm: float = 32.0, start_docked: bool = False,
                  noise_seed: int = 1) -> Any:
    """Instantiate Python VectorSim on the free-cube + charger-dock table scene."""
    import mujoco
    from .robot import VectorSim

    v = params.values(params.load())
    xml = mjcf.build_mjcf(v, (), cube="free", dock="scene")
    sim = VectorSim(values=v, noise_seed=noise_seed)
    sim.model = mujoco.MjModel.from_xml_string(xml)
    sim.data = mujoco.MjData(sim.model)
    sim._index_model()
    sim._init_commands()
    if start_docked:
        _place_docked(sim, head_deg, lift_mm)
    else:
        sim._place(start_xy_mm, heading_deg)
        sim._pose_servos(head_deg, lift_mm)
    # Start range acquisition in the assembled dock scene, after placing the robot.
    sim.body_range_clock = True
    sim._settle()
    for enc, tr in zip(sim._encoders, sim._tread_travel_mm()):
        enc._path_mm, enc._last_travel, enc.counts, enc.speed_mmps = 0.0, tr, 0, 0.0
    sim._dock_travel_mm = [0.0, 0.0]
    sync_dock_contact(sim)
    sim._publish_state()
    return sim


def sync_dock_contact(sim: Any) -> Dict[str, Any]:
    """Update sim.charger_contact and voltage from chassis/dock MuJoCo poses."""
    import mujoco
    dock_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "dock")
    if dock_id < 0:
        return {"on_charger": False, "on_charger_platform": False}
    cid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    state = dock_model.contact_state(sim.data.xpos[cid], sim.data.xmat[cid], sim.data.xpos[dock_id], sim.data.xmat[dock_id])
    sim.charger_contact = bool(state["on_charger"])
    if sim.charger_contact:
        sim.battery_volts = 4.2
    elif sim.battery_volts > float(sim.v["battery_rest_voltage_v"]):
        sim.battery_volts = float(sim.v["battery_rest_voltage_v"])
    return state


@dataclass
class _SubMotion:
    kind: str = "wait"
    ticks: int = 0
    wait_ticks: int = 1
    yaw0: float = 0.0
    angle: float = 0.0
    sign: int = 1
    rate: float = 1.8
    cmd: float = 0.0
    start: Tuple[float, float] = (0.0, 0.0)
    ux: float = 1.0
    uy: float = 0.0
    dist: float = 0.0
    speed: float = 85.0
    goal: Optional[Tuple[float, float]] = None
    goal_speed: float = 85.0
    max_ticks: int = 1200


@dataclass
class _DockCtx:
    stage: str = "find_dwell"
    attempts: int = 0
    relocations: int = 0
    scan_left: int = 8
    wiggle_count: int = 0
    fix_count: int = 0
    flank_count: int = 0
    tried_memory: bool = False
    cliff_recovered: bool = False
    mount_out_yaw: float = 0.0
    backup_ticks: int = 0
    backup_budget: int = 320
    pull_ticks: int = 0
    align_ticks: int = 0
    sub: _SubMotion = field(default_factory=_SubMotion)


class DockingExplorer:
    """Autonomous mapping, marker localization, undocking, and self-docking over Python VectorSim."""

    def __init__(self, sim: Any, spec: Optional[Dict[str, Any]] = None) -> None:
        self.sim = sim
        self.spec = spec or load_camera_spec()
        self.nav_map = NavMap2D()

    def _start_turn(self, sub: _SubMotion, angle_deg: float, speed_dps: float = 110.0) -> None:
        sub.kind, sub.yaw0 = "turn", self.sim._yaw_total
        sub.angle = abs(math.radians(angle_deg))
        sub.sign = 1 if angle_deg >= 0.0 else -1
        sub.rate = _clamp(abs(math.radians(speed_dps)), 0.2, math.radians(self.sim.v["body_rotation_speed_max_dps"]))
        sub.cmd, sub.ticks, sub.max_ticks = 0.0, 0, max(320, int(round(abs(angle_deg) * 5.5)) + 180)

    def _start_drive(self, sub: _SubMotion, dist_mm: float, speed_mmps: float = 85.0, max_ticks: int = 1200) -> None:
        s = self.sim.state
        sub.kind, sub.start, sub.yaw0 = "drive", (s.x_mm, s.y_mm), self.sim._yaw_total
        sub.ux, sub.uy = math.cos(s.pose_angle_rad), math.sin(s.pose_angle_rad)
        sub.dist, sub.sign = abs(dist_mm), (1 if dist_mm >= 0.0 else -1)
        sub.speed, sub.cmd, sub.ticks, sub.max_ticks = min(speed_mmps, self.sim.v["wheel_speed_max_mmps"]), 0.0, 0, max_ticks

    def _drive_to(self, sub: _SubMotion, tx: float, ty: float, speed_mmps: float = 85.0) -> bool:
        s = self.sim.state
        dx, dy = tx - s.x_mm, ty - s.y_mm
        dist = math.hypot(dx, dy)
        if dist <= self.sim.v["drive_done_mm"]:
            sub.kind, sub.ticks, sub.wait_ticks, sub.goal = "wait", 0, 1, None
            return True
        yaw_now = s.pose_angle_rad
        if abs(math.degrees(_wrap(math.atan2(dy, dx) - yaw_now))) > 8.0:
            wc_x, wc_y = s.x_mm - 24.0 * math.cos(yaw_now), s.y_mm - 24.0 * math.sin(yaw_now)
            turn_to = math.atan2(ty - wc_y, tx - wc_x) if math.hypot(tx - wc_x, ty - wc_y) > 28.0 else math.atan2(dy, dx)
            self._start_turn(sub, math.degrees(_wrap(turn_to - yaw_now)), 110.0)
            sub.goal, sub.goal_speed = (tx, ty), speed_mmps
            return False
        sub.goal = None
        self._start_drive(sub, dist, speed_mmps)
        return True

    def _finish_turn_to_goal(self, sub: _SubMotion) -> bool:
        if sub.goal is None:
            return True
        gx, gy = sub.goal
        gs = sub.goal_speed
        sub.goal = None
        dx, dy = gx - self.sim.state.x_mm, gy - self.sim.state.y_mm
        dist = math.hypot(dx, dy)
        if dist <= self.sim.v["drive_done_mm"]:
            return True
        self._start_drive(sub, dist, gs)
        sub.ux, sub.uy = dx / dist, dy / dist
        sub.yaw0 = self.sim._yaw_total + _wrap(math.atan2(dy, dx) - self.sim.state.pose_angle_rad)
        return False

    def _step_sub(self, sub: _SubMotion) -> bool:
        sim = self.sim
        sub.ticks += 1
        if sub.kind == "wait":
            sim._wheel_target = [0.0, 0.0]
            return sub.ticks >= sub.wait_ticks
        if sub.kind == "turn":
            rem = sub.angle - sub.sign * (sim._yaw_total - sub.yaw0)
            if rem <= math.radians(sim.v["turn_tolerance_deg"]) or sub.ticks > sub.max_ticks:
                sim._wheel_target = [0.0, 0.0]
                return self._finish_turn_to_goal(sub)
            acc = sim.v["point_turn_accel_default_radps2"]
            half = 0.5 * sim.v["tread_spacing_mm"]
            sub.cmd = min(sub.rate, sub.cmd + acc * sim.dt, math.sqrt(2.0 * acc * max(0.0, rem)))
            sim._wheel_target = [-sub.sign * sub.cmd * half, sub.sign * sub.cmd * half]
            return False
        s = sim.state
        gone = sub.sign * ((s.x_mm - sub.start[0]) * sub.ux + (s.y_mm - sub.start[1]) * sub.uy)
        rem = sub.dist - gone
        if rem <= sim.v["drive_done_mm"] or sub.ticks > sub.max_ticks:
            sim._wheel_target = [0.0, 0.0]
            return True
        sub.cmd = min(sub.speed, sub.cmd + sim.v["wheel_accel_default_mmps2"] * sim.dt,
                      math.sqrt(2.0 * sim.v["wheel_decel_default_mmps2"] * max(0.0, rem)))
        steer = sim.v["heading_hold_gain_mmps_per_rad"] * (sub.yaw0 - sim._yaw_total)
        sim._wheel_target = [sub.sign * sub.cmd - steer, sub.sign * sub.cmd + steer]
        return False

    def _update_observations(self) -> Dict[str, Optional[Dict[str, float]]]:
        self.nav_map.update_from_sensors(self.sim)
        obs = observe_markers(self.sim, self.spec)
        if obs["charger"] is not None:
            self.nav_map.record_charger(obs["charger"], self.sim.tick)
        if obs["cube"] is not None:
            self.nav_map.record_cube(obs["cube"], self.sim.tick)
        return obs

    def _pick_explore_target(self) -> Tuple[float, float]:
        s = self.sim.state
        best: Optional[Tuple[float, float]] = None
        best_score = -1.0
        for r in (140.0, 100.0, 185.0):
            for deg in (0, 45, -45, 90, -90, 135, -135, 180):
                ang = s.pose_angle_rad + math.radians(deg)
                tx, ty = s.x_mm + r * math.cos(ang), s.y_mm + r * math.sin(ang)
                if abs(tx) > 255.0 or abs(ty) > 255.0:
                    continue
                if self.nav_map.has_clearance(tx, ty, 82.0) and self.nav_map.segment_clear(s.x_mm, s.y_mm, tx, ty, 65.0):
                    sc = float(self.nav_map.score_unknown(tx, ty)) - abs(deg) * 0.03
                    if sc > best_score:
                        best_score, best = sc, (tx, ty)
        if best is None:
            ang = math.atan2(-s.y_mm, -s.x_mm)
            best = (s.x_mm + 90.0 * math.cos(ang), s.y_mm + 90.0 * math.sin(ang))
        self.nav_map.last_frontier = best
        return best

    def _recover_if_needed(self, ctx: _DockCtx) -> bool:
        if self.sim._react_phase != "idle":
            return True
        if not self.sim.has_control:
            self.sim.request_control()
            ctx.cliff_recovered = True
            self._start_drive(ctx.sub, -55.0, 75.0)
            return True
        return False

    def undock(self, max_s: float = 6.0) -> str:
        """Drive forward off the charger until clear of pins and ramp."""
        assert max_s > 0.0, "max_s must be positive"
        saved_lvl, saved_in = self.sim.control_level, self.sim._in_action
        self.sim.control_level = "override"
        self.sim._in_action = True
        x0, y0, _ = self.sim.pose_mm()
        self.sim.command_head_angle(-2.0)
        self.sim.command_lift_height(32.0)
        try:
            for _ in range(int(round(max_s / self.sim.dt))):
                self.sim._wheel_target = [85.0, 85.0]
                self.sim.step(1)
                self.sim._publish_state()
                self._update_observations()
                st = sync_dock_contact(self.sim)
                x, y, _ = self.sim.pose_mm()
                if not st["on_charger"] and not st["on_charger_platform"] and math.hypot(x - x0, y - y0) >= 115.0:
                    self.sim._wheel_target = [0.0, 0.0]
                    for _ in range(6):
                        self.sim.step(1)
                        self.sim._publish_state()
                        sync_dock_contact(self.sim)
                    return "completed"
        finally:
            self.sim.control_level = saved_lvl
            self.sim._in_action = saved_in
            self.sim._wheel_target = [0.0, 0.0]
        return "timeout"

    def _advance_explore(self, sub: _SubMotion, scan_left: int, dwell: bool) -> Tuple[int, bool]:
        if dwell and scan_left > 0:
            self.sim.command_head_angle(4.0)
            self.sim.command_lift_height(32.0)
            self._start_turn(sub, 45.0, 110.0)
            return scan_left - 1, False
        if not dwell:
            sub.kind, sub.ticks, sub.wait_ticks, sub.goal = "wait", 0, 18, None
            return scan_left, True
        tx, ty = self._pick_explore_target()
        self._drive_to(sub, tx, ty, 85.0)
        return scan_left, dwell

    def explore(self, duration_s: float = 8.0) -> Dict[str, Any]:
        """Run bounded autonomous exploration and build NavMap2D."""
        assert 0.5 <= duration_s <= 60.0, "duration_s out of range"
        if sync_dock_contact(self.sim)["on_charger_platform"] or self.sim.charger_contact:
            self.undock()
        sub = _SubMotion(kind="wait", wait_ticks=10)
        scan_left, dwell, saved_in = 8, True, self.sim._in_action
        self.sim._in_action = True
        try:
            for _ in range(int(round(duration_s / self.sim.dt))):
                self.sim.step(1)
                self.sim._publish_state()
                self._update_observations()
                sync_dock_contact(self.sim)
                if self.sim._react_phase == "idle" and not self.sim.has_control:
                    self.sim.request_control()
                    self._start_drive(sub, -55.0, 75.0)
                elif self._step_sub(sub):
                    scan_left, dwell = self._advance_explore(sub, scan_left, dwell)
        finally:
            self.sim._in_action = saved_in
            self.sim._wheel_target = [0.0, 0.0]
        mapped, cliffs = self.nav_map.counts()
        return {"result": "completed", "cells_mapped": mapped, "cliffs_found": cliffs,
                "cube_seen": self.nav_map.cube_pose is not None, "charger_seen": self.nav_map.charger_pose is not None}

    def _pick_relocate(self, relocations: int) -> Tuple[float, float]:
        s = self.sim.state
        sy = 1.0 if s.y_mm >= 0.0 else -1.0
        candidates = ((-60.0, sy * 145.0), (35.0, sy * 115.0), (35.0, -sy * 115.0), (20.0, 0.0), (140.0, sy * 110.0))
        for k in range(len(candidates)):
            tx, ty = candidates[(relocations + k - 1) % len(candidates)]
            if math.hypot(tx - s.x_mm, ty - s.y_mm) >= 45.0 and self.nav_map.has_clearance(tx, ty, 70.0, True):
                if self.nav_map.segment_clear(s.x_mm, s.y_mm, tx, ty, 50.0, True):
                    return (tx, ty)
        return (25.0, sy * 120.0)

    def _pick_flank(self, lx: float, ly: float, ox: float, oy: float, side_sign: float) -> Tuple[float, float]:
        s = self.sim.state
        px, py = -oy, ox
        for sgn in (side_sign, -side_sign):
            for lat_off in (125.0, 95.0, 150.0):
                fx = _clamp(lx + 85.0 * ox + sgn * lat_off * px, -235.0, 235.0)
                fy = _clamp(ly + 85.0 * oy + sgn * lat_off * py, -235.0, 235.0)
                if self.nav_map.has_clearance(fx, fy, 65.0, True) and self.nav_map.segment_clear(s.x_mm, s.y_mm, fx, fy, 52.0, True):
                    return (fx, fy)
        return (_clamp(lx + 85.0 * ox + side_sign * 125.0 * px, -235.0, 235.0),
                _clamp(ly + 85.0 * oy + side_sign * 125.0 * py, -235.0, 235.0))

    def _plan_approach(self, ctx: _DockCtx) -> None:
        s, ch = self.sim.state, self.nav_map.charger_pose
        assert ch is not None
        lx, ly, ox, oy = ch["lip_x"], ch["lip_y"], ch["out_x"], ch["out_y"]
        along = (s.x_mm - lx) * ox + (s.y_mm - ly) * oy
        lat_signed = -(s.x_mm - lx) * oy + (s.y_mm - ly) * ox
        dists = (78.0, 102.0, 135.0) if (65.0 <= along <= 165.0 and abs(lat_signed) <= 18.0) else (135.0, 102.0, 82.0)
        if along >= 25.0:
            for d in dists:
                tx, ty = lx + d * ox, ly + d * oy
                if self.nav_map.segment_clear(s.x_mm, s.y_mm, tx, ty, 52.0, True):
                    ctx.flank_count = 0
                    self._drive_to(ctx.sub, tx, ty, 80.0)
                    ctx.stage = "route_predock"
                    return
        if ctx.flank_count < 2:
            ctx.flank_count += 1
            fx, fy = self._pick_flank(lx, ly, ox, oy, 1.0 if lat_signed >= 0.0 else -1.0)
            self._drive_to(ctx.sub, fx, fy, 85.0)
            ctx.stage = "route_flank"
            return
        ctx.flank_count = 0
        self._drive_to(ctx.sub, lx + 85.0 * ox, ly + 85.0 * oy, 75.0)
        ctx.stage = "route_predock"

    def _step_dock_backup(self, ctx: _DockCtx, dock_st: Dict[str, Any]) -> Optional[str]:
        sim, ch = self.sim, self.nav_map.charger_pose
        ctx.backup_ticks += 1
        if ctx.backup_ticks == 1:
            # Ticks for the way to the seat at this attempt's speed, with a margin, and never fewer than the 320 it had:
            # first backups starting 140 to 149 mm out ran out of ticks 21.6 to 30.3 mm short of the lip (the browser's dock).
            fwd = (sim.state.x_mm - ch["lip_x"]) * ch["out_x"] + (sim.state.y_mm - ch["lip_y"]) * ch["out_y"] if ch else 0.0
            speed = abs(BACKUP_SPEEDS_MMPS[ctx.wiggle_count % len(BACKUP_SPEEDS_MMPS)])
            ctx.backup_budget = max(BACKUP_MIN_TICKS, int(math.ceil(BACKUP_MARGIN * (fwd + SEAT_DEPTH_MM) / (speed * sim.dt))))
        if sim.pose_mm()[2] < -5.0 or abs(math.degrees(sim._pitch())) > 25.0:
            return "aborted_off_ground"
        yaw, out_yaw = sim.state.pose_angle_rad, ctx.mount_out_yaw
        rx, ry = sim.state.x_mm - 44.0 * math.cos(yaw), sim.state.y_mm - 44.0 * math.sin(yaw)
        rfwd = (rx - ch["lip_x"]) * ch["out_x"] + (ry - ch["lip_y"]) * ch["out_y"] if ch else 0.0
        rlat = -(rx - ch["lip_x"]) * ch["out_y"] + (ry - ch["lip_y"]) * ch["out_x"] if ch else 0.0
        twisted = ctx.backup_ticks > 90 and abs(_wrap(out_yaw - yaw)) > math.radians(10.0) and not dock_st["on_charger"]
        past_rear = ch is not None and not dock_st["on_charger"] and not dock_st["on_charger_platform"] and rfwd < -95.0
        if dock_st["on_charger"] or ctx.backup_ticks > ctx.backup_budget or twisted or past_rear:
            sim._wheel_target = [0.0, 0.0]
            ctx.sub, ctx.stage = _SubMotion(kind="wait", wait_ticks=22), "verify"
            return None
        spd = BACKUP_SPEEDS_MMPS[ctx.wiggle_count % len(BACKUP_SPEEDS_MMPS)]
        des_err = _clamp(0.015 * rlat, -0.20, 0.20) if rfwd > -2.0 else 0.0
        # Past the lip the lateral term stops but the heading is still held. Backing straight on from 150 mm with up to
        # 3 mm and 1 degree of offset, 4 of 9 climbs seated unheld and 8 of 9 held (the browser's dock); a miss wedges
        # about 18 degrees round.
        steer = 140.0 * _wrap(out_yaw + des_err - yaw)
        sim._wheel_target = [spd - steer, spd + steer]
        return None

    def _step_dock_wiggle(self, ctx: _DockCtx, dock_st: Dict[str, Any]) -> Optional[str]:
        sim, ch = self.sim, self.nav_map.charger_pose
        if ctx.stage == "verify":
            if not self._step_sub(ctx.sub):
                return None
            if dock_st["on_charger"]:
                sim.command_head_angle(0.0)
                sim.command_lift_height(32.0)
                return "completed"
            if ctx.wiggle_count < 5:
                ctx.wiggle_count += 1
                ctx.pull_ticks, ctx.stage = 0, "wiggle_fwd"
                return None
            if ctx.attempts >= 3:
                return "dock_failed"
            ctx.wiggle_count = 0
            self._start_drive(ctx.sub, 125.0, 85.0)
            ctx.stage = "retry_away"
            return None
        if ctx.stage == "wiggle_fwd":
            ctx.pull_ticks += 1
            out_yaw = math.atan2(ch["out_y"], ch["out_x"]) if ch else ctx.mount_out_yaw
            fwd = (sim.state.x_mm - ch["lip_x"]) * ch["out_x"] + (sim.state.y_mm - ch["lip_y"]) * ch["out_y"] if ch else 80.0
            lat = -(sim.state.x_mm - ch["lip_x"]) * ch["out_y"] + (sim.state.y_mm - ch["lip_y"]) * ch["out_x"] if ch else 0.0
            if fwd >= (90.0 if ctx.wiggle_count % 2 == 0 else 78.0) or ctx.pull_ticks >= PULL_MAX_TICKS:
                sim._wheel_target = [0.0, 0.0]
                ctx.mount_out_yaw, ctx.align_ticks, ctx.stage = out_yaw, 0, "wiggle_align"
                return None
            steer = 160.0 * _wrap(out_yaw + _clamp(-0.015 * lat, -0.25, 0.25) - sim.state.pose_angle_rad)
            sim._wheel_target = [85.0 - steer, 85.0 + steer]
            return None
        ctx.align_ticks += 1
        err = _wrap(ctx.mount_out_yaw - sim.state.pose_angle_rad)
        if abs(err) < 0.015 or ctx.align_ticks >= 120:
            sim._wheel_target = [0.0, 0.0]
            ctx.backup_ticks, ctx.stage = 0, "backup"
            return None
        w = _clamp(140.0 * err, -45.0, 45.0)
        sim._wheel_target = [-w, w]
        return None

    def _step_dock_close(self, ctx: _DockCtx) -> Optional[str]:
        sim, ch = self.sim, self.nav_map.charger_pose
        if ctx.stage in ("route_flank", "route_predock", "face_marker"):
            if not self._step_sub(ctx.sub):
                return None
            if ctx.cliff_recovered:
                ctx.cliff_recovered, ctx.flank_count = False, 0
                ctx.relocations += 1
                self._drive_to(ctx.sub, *self._pick_relocate(ctx.relocations), 85.0)
                ctx.stage = "relocate"
                return None
            if ctx.stage == "route_flank":
                self._plan_approach(ctx)
                return None
            if ctx.stage == "route_predock":
                ctx.sub.goal = None
                self._start_turn(ctx.sub, math.degrees(_wrap(math.atan2(-ch["out_y"], -ch["out_x"]) - sim.state.pose_angle_rad)), 100.0)
                ctx.stage = "face_marker"
                return None
            ctx.sub, ctx.stage = _SubMotion(kind="wait", wait_ticks=24), "close_fix"
            return None
        if not self._step_sub(ctx.sub):
            return None
        if ch is None or sim.tick - ch["seen_at"] > 60:
            self.nav_map.charger_pose = None
            ctx.fix_count, ctx.flank_count, ctx.scan_left, ctx.sub, ctx.stage = 0, 0, 8, _SubMotion(kind="wait", wait_ticks=10), "find_dwell"
            return None
        lat = abs(-(sim.state.x_mm - ch["lip_x"]) * ch["out_y"] + (sim.state.y_mm - ch["lip_y"]) * ch["out_x"])
        fwd = (sim.state.x_mm - ch["lip_x"]) * ch["out_x"] + (sim.state.y_mm - ch["lip_y"]) * ch["out_y"]
        if (lat > 4.0 or fwd < 60.0 or fwd > 98.0) and ctx.fix_count < 2:
            ctx.fix_count += 1
            d = 145.0 if (lat > 5.0 and fwd < 120.0) else 78.0
            self._drive_to(ctx.sub, ch["lip_x"] + d * ch["out_x"], ch["lip_y"] + d * ch["out_y"], 70.0)
            ctx.stage = "route_predock"
            return None
        ctx.fix_count, ctx.wiggle_count = 0, 0
        # The lift moves only while the robot stands: down through the turn onto the dock's axis, up after it,
        # and in position before the robot backs on (align_back, lift_up). Raised as the turn began, with the
        # backup starting as it ended, the dock from a docked start failed for 7 of 8 noise seeds.
        ctx.mount_out_yaw = math.atan2(ch["out_y"], ch["out_x"])
        self._start_turn(ctx.sub, math.degrees(_wrap(ctx.mount_out_yaw - sim.state.pose_angle_rad)), 95.0)
        ctx.stage = "align_back"
        return None

    def _step_dock_find(self, ctx: _DockCtx) -> Optional[str]:
        sim, ch = self.sim, self.nav_map.charger_pose
        sim.command_head_angle(4.0)
        sim.command_lift_height(32.0)
        known = ch is not None and (sim.tick - ch["seen_at"] <= 30 or not ctx.tried_memory)
        if ctx.stage == "find_dwell" and known:
            ctx.tried_memory = True
            self._plan_approach(ctx)
            return None
        if ctx.stage == "relocate" and ch is not None and sim.tick - ch["seen_at"] <= 25:
            self._plan_approach(ctx)
            return None
        if not self._step_sub(ctx.sub):
            return None
        if ctx.cliff_recovered:
            ctx.cliff_recovered, ctx.flank_count, ctx.relocations, ctx.scan_left = False, 0, ctx.relocations + 1, 0
            self._drive_to(ctx.sub, *self._pick_relocate(ctx.relocations), 85.0)
            ctx.stage = "relocate"
            return None
        if ctx.stage == "find_dwell":
            if ctx.scan_left > 0:
                ctx.scan_left -= 1
                self._start_turn(ctx.sub, 45.0, 110.0)
                ctx.stage = "find_turn"
                return None
            if ctx.relocations >= 4:
                return "charger_not_found"
            ctx.relocations, ctx.flank_count, ctx.scan_left = ctx.relocations + 1, 0, 8
            self._drive_to(ctx.sub, *self._pick_relocate(ctx.relocations), 85.0)
            ctx.stage = "relocate"
            return None
        ctx.sub, ctx.stage = _SubMotion(kind="wait", wait_ticks=24), "find_dwell"
        return None

    def _step_dock_state(self, ctx: _DockCtx, dock_st: Dict[str, Any]) -> Optional[str]:
        if ctx.stage in ("find_dwell", "find_turn", "relocate"):
            return self._step_dock_find(ctx)
        if ctx.stage in ("route_flank", "route_predock", "face_marker", "close_fix"):
            return self._step_dock_close(ctx)
        if ctx.stage == "align_back":
            if self._step_sub(ctx.sub):
                self.sim.control_level = "override"
                self.sim.command_lift_height(45.0)
                ctx.attempts += 1
                ctx.sub, ctx.stage = _SubMotion(kind="wait", wait_ticks=LIFT_UP_WAIT_TICKS), "lift_up"
            return None
        if ctx.stage == "lift_up":
            # Stand still until the lift is in position (or the wait runs out), then back onto the dock.
            self._step_sub(ctx.sub)
            if self.sim._lift_in_pos or ctx.sub.ticks >= ctx.sub.wait_ticks:
                ctx.backup_ticks, ctx.stage = 0, "backup"
            return None
        if ctx.stage == "backup":
            return self._step_dock_backup(ctx, dock_st)
        if ctx.stage in ("verify", "wiggle_fwd", "wiggle_align"):
            return self._step_dock_wiggle(ctx, dock_st)
        if ctx.stage == "retry_away":
            if self._step_sub(ctx.sub):
                self._start_turn(ctx.sub, 180.0, 100.0)
                ctx.stage = "retry_turn"
            return None
        if self._step_sub(ctx.sub):
            ctx.flank_count, ctx.sub, ctx.stage = 0, _SubMotion(kind="wait", wait_ticks=24), "find_dwell"
        return None

    def dock(self, max_s: float = 55.0) -> Dict[str, Any]:
        """Locate charger marker optically, align in front, and mount charger pins."""
        assert max_s > 0.0, "max_s must be positive"
        dock_st = sync_dock_contact(self.sim)
        if dock_st["on_charger"]:
            return {"result": "completed", "attempts": 0}
        if dock_st["on_charger_platform"]:
            self.undock()
        ctx = _DockCtx(sub=_SubMotion(kind="wait", wait_ticks=24))
        saved, saved_in = self.sim.control_level, self.sim._in_action
        self.sim._in_action = True
        try:
            for _ in range(int(round(max_s / self.sim.dt))):
                self.sim.step(1)
                self.sim._publish_state()
                self._update_observations()
                dock_st = sync_dock_contact(self.sim)
                if self._recover_if_needed(ctx):
                    continue
                res = self._step_dock_state(ctx, dock_st)
                if res is not None:
                    return {"result": res, "attempts": ctx.attempts}
        finally:
            self.sim.control_level = saved
            self.sim._in_action = saved_in
            self.sim._wheel_target = [0.0, 0.0]
        return {"result": "timeout", "attempts": ctx.attempts}
