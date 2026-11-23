// Optical fiducial marker detector, 20 mm NavMapGrid occupancy map, and autonomous
// undock / explore / self-docking controller over VectorSim (no THREE dependency).

import { pixelOf, rayOf } from "./optics.js";
import { assert, clamp, pyRound } from "./util.js";

const MM = 0.001;
const DEG = Math.PI / 180;
const DOCK_MARKER_HALF_M = 0.023;
const DOCK_MARKER_X_MM = 93.205;
const DOCK_MARKER_Z_M = 0.0485;
const CUBE_MARKER_HALF_M = 0.0115;

export const CELL_MM = 20;
export const GRID_HALF_CELLS = 25;
export const GRID_DIM = 2 * GRID_HALF_CELLS; // 50x50 = [-500, +500) mm
export const CELL_UNKNOWN = 0;
export const CELL_CLEAR = 1;
export const CELL_PROX = 2;
export const CELL_CUBE = 3;
export const CELL_CLIFF = 4;
export const CELL_CHARGER = 5;

const DOCK_CORNERS_LOCAL = [
  [-0.0005, DOCK_MARKER_HALF_M, DOCK_MARKER_HALF_M],   // top-left as viewed from -X
  [-0.0005, -DOCK_MARKER_HALF_M, DOCK_MARKER_HALF_M],  // top-right
  [-0.0005, -DOCK_MARKER_HALF_M, -DOCK_MARKER_HALF_M], // bottom-right
  [-0.0005, DOCK_MARKER_HALF_M, -DOCK_MARKER_HALF_M],  // bottom-left
];

function wrapAngle(rad) {
  return Math.atan2(Math.sin(rad), Math.cos(rad));
}

function mulMatVec(m, o, v) {
  return [
    m[o] * v[0] + m[o + 1] * v[1] + m[o + 2] * v[2],
    m[o + 3] * v[0] + m[o + 4] * v[1] + m[o + 5] * v[2],
    m[o + 6] * v[0] + m[o + 7] * v[1] + m[o + 8] * v[2],
  ];
}

function transposeMulVec(m, o, v) {
  return [
    m[o] * v[0] + m[o + 3] * v[1] + m[o + 6] * v[2],
    m[o + 1] * v[0] + m[o + 4] * v[1] + m[o + 7] * v[2],
    m[o + 2] * v[0] + m[o + 5] * v[1] + m[o + 8] * v[2],
  ];
}

export class NavMapGrid {
  constructor() {
    this.cells = new Uint8Array(GRID_DIM * GRID_DIM);
    this.chargerPose = null; // { lipX, lipY, outX, outY, seenAt }
    this.cubePose = null;    // { x, y, seenAt }
    this.lastFrontier = null;
  }

  reset() {
    this.cells.fill(CELL_UNKNOWN);
    this.chargerPose = null;
    this.cubePose = null;
    this.lastFrontier = null;
  }

  _idx(xMm, yMm) {
    const gx = Math.floor(xMm / CELL_MM) + GRID_HALF_CELLS;
    const gy = Math.floor(yMm / CELL_MM) + GRID_HALF_CELLS;
    if (gx < 0 || gx >= GRID_DIM || gy < 0 || gy >= GRID_DIM) return -1;
    return gy * GRID_DIM + gx;
  }

  cellCenter(idx) {
    const gx = idx % GRID_DIM;
    const gy = Math.floor(idx / GRID_DIM);
    return [(gx - GRID_HALF_CELLS + 0.5) * CELL_MM, (gy - GRID_HALF_CELLS + 0.5) * CELL_MM];
  }

  setCell(xMm, yMm, kind) {
    const i = this._idx(xMm, yMm);
    if (i < 0) return;
    if (kind === CELL_CLIFF || this.cells[i] !== CELL_CLIFF) this.cells[i] = kind;
  }

  getCell(xMm, yMm) {
    const i = this._idx(xMm, yMm);
    return i < 0 ? CELL_CLIFF : this.cells[i];
  }

  updateFromSensors(sim) {
    const s = sim.state;
    const c = Math.cos(s.pose_angle_rad), si = Math.sin(s.pose_angle_rad);
    for (const [lx, ly] of [[0, 0], [18, 0], [-18, 0], [0, 14], [0, -14]]) {
      this.setCell(s.x_mm + c * lx - si * ly, s.y_mm + si * lx + c * ly, CELL_CLEAR);
    }
    const cliffOffsets = [[25, 22], [25, -22], [-42, 22], [-42, -22]];
    const flags = sim.hazards.cliff;
    for (let i = 0; i < 4; i++) {
      if (!flags[i]) continue;
      const [ox, oy] = cliffOffsets[i];
      this.setCell(s.x_mm + c * ox - si * oy, s.y_mm + si * ox + c * oy, CELL_CLIFF);
    }
    if (!s.prox_is_lift_in_fov && Math.abs(s.pose_pitch_rad ?? 0) <= 0.05) {
      const clearDist = s.prox_found_object ? Math.max(0, s.prox_distance_mm - 20)
        : (s.prox_unobstructed ? 180 : 0);
      for (let d = 30; d <= clearDist; d += CELL_MM) {
        this.setCell(s.x_mm + c * (10 + d), s.y_mm + si * (10 + d), CELL_CLEAR);
      }
      if (s.prox_found_object && s.prox_distance_mm <= 380) {
        const hitD = 10 + s.prox_distance_mm;
        this.setCell(s.x_mm + c * hitD, s.y_mm + si * hitD, CELL_PROX);
      }
    }
  }

  recordCharger(obs, tick) {
    this.chargerPose = { ...obs, seenAt: tick };
    const { lipX, lipY, outX, outY } = obs;
    for (let along = -20; along >= -100; along -= 20) {
      for (let lat = -36; lat <= 36; lat += 20) {
        this.setCell(lipX - along * outX - lat * outY, lipY - along * outY + lat * outX, CELL_CHARGER);
      }
    }
  }

  recordCube(obs, tick) {
    this.cubePose = { ...obs, seenAt: tick };
    this.setCell(obs.x, obs.y, CELL_CUBE);
  }

  hasObstacleClearance(xMm, yMm, minMm = 80, ignoreCharger = false) {
    for (let i = 0; i < this.cells.length; i++) {
      const k = this.cells[i];
      if (k !== CELL_CLIFF && k !== CELL_PROX && k !== CELL_CUBE && (ignoreCharger || k !== CELL_CHARGER)) continue;
      const [cx, cy] = this.cellCenter(i);
      const pad = k === CELL_CLIFF ? minMm : (k === CELL_CUBE ? Math.min(minMm, 64) : Math.min(minMm, 62));
      if (Math.hypot(xMm - cx, yMm - cy) < pad) return false;
    }
    return true;
  }

  segmentClear(x0, y0, x1, y1, clearanceMm = 68, ignoreCharger = false) {
    const steps = 10;
    for (let i = 1; i <= steps; i++) {
      const t = i / steps;
      if (!this.hasObstacleClearance(x0 + t * (x1 - x0), y0 + t * (y1 - y0), clearanceMm, ignoreCharger)) return false;
    }
    return true;
  }

  scoreUnknown(xMm, yMm) {
    const gx0 = Math.floor(xMm / CELL_MM) + GRID_HALF_CELLS;
    const gy0 = Math.floor(yMm / CELL_MM) + GRID_HALF_CELLS;
    let unknown = 0;
    for (let dy = -3; dy <= 3; dy++) {
      for (let dx = -3; dx <= 3; dx++) {
        const gx = gx0 + dx, gy = gy0 + dy;
        if (gx < 0 || gx >= GRID_DIM || gy < 0 || gy >= GRID_DIM) continue;
        if (this.cells[gy * GRID_DIM + gx] === CELL_UNKNOWN) unknown += 1;
      }
    }
    return unknown;
  }

  counts() {
    let mapped = 0, cliffs = 0;
    for (let i = 0; i < this.cells.length; i++) {
      if (this.cells[i] !== CELL_UNKNOWN) mapped += 1;
      if (this.cells[i] === CELL_CLIFF) cliffs += 1;
    }
    return { mapped, cliffs };
  }

  snapshot() {
    const obstacles = [];
    const cliffs = [];
    const clear = [];
    for (let i = 0; i < this.cells.length; i++) {
      const k = this.cells[i];
      if (k === CELL_UNKNOWN) continue;
      const pt = this.cellCenter(i);
      if (k === CELL_CLIFF) cliffs.push(pt);
      else if (k === CELL_PROX) obstacles.push(pt);
      else if (k === CELL_CLEAR) clear.push(pt);
    }
    return { clear, obstacles, cliffs, charger: this.chargerPose, cube: this.cubePose, frontier: this.lastFrontier };
  }
}

// Compute camera frame in world metres and its chassis-to-odom transform matrix.
function cameraPoseInOdom(sim, spec) {
  const camId = sim.mujoco.mj_name2id(sim.model, sim.mujoco.mjtObj.mjOBJ_CAMERA.value, spec.name);
  if (camId < 0) return null;
  const { data, _chassisBody: cb } = sim;
  const { xpos, xmat, cam_xpos, cam_xmat } = data;
  const cw = [cam_xpos[3 * camId], cam_xpos[3 * camId + 1], cam_xpos[3 * camId + 2]];
  const mountDown = spec.pitch_down_in_model ? 0 : spec.pitch_down_deg;
  const tilt = ((spec.rgb_pitch_up_deg ?? 0) - mountDown) * DEG;
  const ct = Math.cos(tilt), st = Math.sin(tilt);
  const o = 9 * camId;
  const rCamW = [
    cam_xmat[o], cam_xmat[o + 1] * ct + cam_xmat[o + 2] * st, -cam_xmat[o + 1] * st + cam_xmat[o + 2] * ct,
    cam_xmat[o + 3], cam_xmat[o + 4] * ct + cam_xmat[o + 5] * st, -cam_xmat[o + 4] * st + cam_xmat[o + 5] * ct,
    cam_xmat[o + 6], cam_xmat[o + 7] * ct + cam_xmat[o + 8] * st, -cam_xmat[o + 7] * st + cam_xmat[o + 8] * ct,
  ];
  // Keep world gravity up while shifting horizontal offset + yaw into odometry frame.
  const dYaw = wrapAngle(sim.state.pose_angle_rad - sim._yaw());
  const cd = Math.cos(dYaw), sd = Math.sin(dYaw);
  const dxW = cw[0] - xpos[3 * cb], dyW = cw[1] - xpos[3 * cb + 1], dzW = cw[2] - xpos[3 * cb + 2];
  const camOdomMm = [
    sim.state.x_mm + (cd * dxW - sd * dyW) / MM,
    sim.state.y_mm + (sd * dxW + cd * dyW) / MM,
    dzW / MM,
  ];
  const camToWorld = (v) => {
    const w = mulMatVec(rCamW, 0, v);
    return [cd * w[0] - sd * w[1], sd * w[0] + cd * w[1], w[2]];
  };
  return { camId, camWorld: cw, rCamW, camOdomMm, camToWorld };
}

function markerCornersVisible(sim, spec, cam, worldCorners, outwardNormalWorld) {
  const center = [0, 1, 2].map((k) => 0.25 * worldCorners.reduce((s, c) => s + c[k], 0));
  const toCam = [cam.camWorld[0] - center[0], cam.camWorld[1] - center[1], cam.camWorld[2] - center[2]];
  const dist = Math.hypot(...toCam);
  if (dist < 0.04 || dist > 0.55) return null;
  const cosInc = (toCam[0] * outwardNormalWorld[0] + toCam[1] * outwardNormalWorld[1] + toCam[2] * outwardNormalWorld[2]) / dist;
  if (cosInc < Math.cos(70 * DEG)) return null;
  const pixels = [];
  for (const wc of worldCorners) {
    const pCam = transposeMulVec(cam.rCamW, 0, [wc[0] - cam.camWorld[0], wc[1] - cam.camWorld[1], wc[2] - cam.camWorld[2]]);
    const uv = pixelOf(spec, pCam);
    if (!uv || uv[0] < 4 || uv[0] > spec.width_px - 4 || uv[1] < 4 || uv[1] > spec.height_px - 4) return null;
    // Quantize to 0.5 px subpixel accuracy and check line-of-sight against scene geoms.
    const dVec = [wc[0] - cam.camWorld[0], wc[1] - cam.camWorld[1], wc[2] - cam.camWorld[2]];
    const cDist = Math.hypot(...dVec);
    const hit = sim.mujoco.mj_ray(sim.model, sim.data, cam.camWorld, dVec.map((x) => x / cDist),
      sim._sceneOnly, true, -1, null, null);
    if (hit >= 0 && hit < cDist - 0.020) return null;
    pixels.push([Math.round(uv[0] * 2) / 2, Math.round(uv[1] * 2) / 2]);
  }
  return pixels;
}

// Reconstruct an upright 2h-sized vertical marker center & outward normal from quantized pixels in odometry frame.
function reconstructVerticalSquare(spec, cam, pixels, sizeMm) {
  const rays = pixels.map(([u, v]) => cam.camToWorld(rayOf(spec, u, v, 1)));
  const edgeXY = (topIdx, botIdx) => {
    const wt = rays[topIdx], wb = rays[botIdx];
    const hTop = Math.hypot(wt[0], wt[1]), hBot = Math.hypot(wb[0], wb[1]);
    if (hTop < 1e-4 || hBot < 1e-4) return null;
    const slope = (wt[2] / hTop) - (wb[2] / hBot);
    if (slope <= 0.01) return null;
    const range = sizeMm / slope;
    const mx = 0.5 * (wt[0] / hTop + wb[0] / hBot);
    const my = 0.5 * (wt[1] / hTop + wb[1] / hBot);
    return [cam.camOdomMm[0] + range * mx, cam.camOdomMm[1] + range * my];
  };
  const left = edgeXY(0, 3);
  const right = edgeXY(1, 2);
  if (!left || !right) return null;
  const cx = 0.5 * (left[0] + right[0]), cy = 0.5 * (left[1] + right[1]);
  const dx = right[0] - left[0], dy = right[1] - left[1];
  const chord = Math.hypot(dx, dy);
  if (chord < 5) return null;
  // Two perpendicular candidates in XY; choose the one pointing toward the camera.
  let nx = -dy / chord, ny = dx / chord;
  if (nx * (cam.camOdomMm[0] - cx) + ny * (cam.camOdomMm[1] - cy) < 0) {
    nx = -nx;
    ny = -ny;
  }
  return { cx, cy, nx, ny };
}

export function observeMarkers(sim, spec) {
  if (!spec || Math.abs(sim._imu()[1][2]) > 1.2) return { charger: null, cube: null };
  const liftMm = sim.state?.lift_height_mm ?? sim.liftHeightMm();
  if (liftMm > 38 && liftMm < 62) return { charger: null, cube: null };
  const cam = cameraPoseInOdom(sim, spec);
  if (!cam) return { charger: null, cube: null };
  let charger = null;
  if (sim._dockBody >= 0) {
    const { xpos, xmat } = sim.data;
    const db = sim._dockBody;
    const mPos = [xpos[3 * db], xpos[3 * db + 1], xpos[3 * db + 2]];
    const markerW = [
      mPos[0] + DOCK_MARKER_X_MM * MM * xmat[9 * db] + DOCK_MARKER_Z_M * xmat[9 * db + 2],
      mPos[1] + DOCK_MARKER_X_MM * MM * xmat[9 * db + 3] + DOCK_MARKER_Z_M * xmat[9 * db + 5],
      mPos[2] + DOCK_MARKER_X_MM * MM * xmat[9 * db + 6] + DOCK_MARKER_Z_M * xmat[9 * db + 8],
    ];
    const worldCorners = DOCK_CORNERS_LOCAL.map((lc) => {
      const rw = mulMatVec(xmat, 9 * db, lc);
      return [markerW[0] + rw[0], markerW[1] + rw[1], markerW[2] + rw[2]];
    });
    const outNormalW = [-xmat[9 * db], -xmat[9 * db + 3], -xmat[9 * db + 6]];
    const pixels = markerCornersVisible(sim, spec, cam, worldCorners, outNormalW);
    if (pixels) {
      const pose = reconstructVerticalSquare(spec, cam, pixels, DOCK_MARKER_HALF_M * 2000);
      if (pose) {
        charger = {
          lipX: pose.cx + DOCK_MARKER_X_MM * pose.nx,
          lipY: pose.cy + DOCK_MARKER_X_MM * pose.ny,
          outX: pose.nx,
          outY: pose.ny,
          markerX: pose.cx,
          markerY: pose.cy,
        };
      }
    }
  }
  let cube = null;
  if (sim._cubeBody >= 0) {
    const { xpos, xmat } = sim.data;
    const cb = sim._cubeBody;
    const cPos = [xpos[3 * cb], xpos[3 * cb + 1], xpos[3 * cb + 2]];
    const h = CUBE_MARKER_HALF_M;
    const faceX = [1, 0, 0], faceNX = [-1, 0, 0], faceY = [0, 1, 0], faceNY = [0, -1, 0];
    for (const n of [faceX, faceNX, faceY, faceNY]) {
      const nw = mulMatVec(xmat, 9 * cb, n);
      if (Math.abs(nw[2]) > 0.4) continue;
      const right = [n[1], -n[0], 0];
      const fc = n.map((k) => k * 0.0221);
      const corners = [[-1, 1], [1, 1], [1, -1], [-1, -1]].map(([sr, sz]) => {
        const lw = mulMatVec(xmat, 9 * cb, [fc[0] + sr * h * right[0], fc[1] + sr * h * right[1], fc[2] + sz * h]);
        return [cPos[0] + lw[0], cPos[1] + lw[1], cPos[2] + lw[2]];
      });
      const px = markerCornersVisible(sim, spec, cam, corners, nw);
      if (px) {
        const pose = reconstructVerticalSquare(spec, cam, px, h * 2000);
        if (pose) {
          cube = { x: pose.cx - 22 * pose.nx, y: pose.cy - 22 * pose.ny };
          break;
        }
      }
    }
  }
  return { charger, cube };
}

export class AutonomyController {
  constructor(sim, cameraSpec = null) {
    this.sim = sim;
    this.cameraSpec = cameraSpec;
    this.navMap = new NavMapGrid();
    this.phase = null;
    this._savedLevel = null;
  }

  _startSubTurn(sub, angleDeg, speedDps = 110) {
    const v = this.sim.v;
    sub.kind = "turn";
    sub.yaw0 = this.sim._yawTotal;
    sub.angle = Math.abs(angleDeg * DEG);
    sub.sign = angleDeg >= 0 ? 1 : -1;
    sub.rate = clamp(Math.abs(speedDps) * DEG, 0.2, v.body_rotation_speed_max_dps * DEG);
    sub.cmd = 0;
    sub.ticks = 0;
  }

  _startSubDrive(sub, distMm, speedMmps = 90) {
    const v = this.sim.v;
    sub.kind = "drive";
    sub.start = [this.sim.state.x_mm, this.sim.state.y_mm];
    sub.yaw0 = this.sim._yawTotal;
    sub.ux = Math.cos(this.sim.state.pose_angle_rad);
    sub.uy = Math.sin(this.sim.state.pose_angle_rad);
    sub.dist = Math.abs(distMm);
    sub.sign = distMm >= 0 ? 1 : -1;
    sub.speed = Math.min(speedMmps, v.wheel_speed_max_mmps);
    sub.cmd = 0;
    sub.ticks = 0;
  }

  _driveToPoint(sub, tx, ty, speedMmps = 85) {
    const dx = tx - this.sim.state.x_mm, dy = ty - this.sim.state.y_mm;
    const dist = Math.hypot(dx, dy);
    if (dist <= this.sim.v.drive_done_mm) {
      sub.kind = "wait";
      sub.ticks = 0;
      sub.waitTicks = 1;
      sub.goal = null;
      return true;
    }
    const yawNow = this.sim.state.pose_angle_rad;
    const errDirectDeg = wrapAngle(Math.atan2(dy, dx) - yawNow) / DEG;
    if (Math.abs(errDirectDeg) > 8) {
      const wcX = this.sim.state.x_mm - 24 * Math.cos(yawNow);
      const wcY = this.sim.state.y_mm - 24 * Math.sin(yawNow);
      const wcDist = Math.hypot(tx - wcX, ty - wcY);
      const turnTarget = wcDist > 28 ? Math.atan2(ty - wcY, tx - wcX) : Math.atan2(dy, dx);
      this._startSubTurn(sub, wrapAngle(turnTarget - yawNow) / DEG, 110);
      sub.goal = [tx, ty];
      sub.goalSpeed = speedMmps;
      return false;
    }
    sub.goal = null;
    this._startSubDrive(sub, dist, speedMmps);
    sub.waypoint = [tx, ty];
    return true;
  }

  _stepSub(sub) {
    const sim = this.sim;
    sub.ticks += 1;
    if (sub.kind === "wait") {
      sim._wheelTarget = [0, 0];
      sim._wheelAccel = [0, 0];
      return sub.ticks >= sub.waitTicks;
    }
    if (sub.kind === "turn") {
      const rem = sub.angle - sub.sign * (sim._yawTotal - sub.yaw0);
      const maxTurnTicks = Math.max(320, Math.round((sub.angle / DEG) * 5.5) + 180);
      if (rem <= sim.v.turn_tolerance_deg * DEG || sub.ticks > maxTurnTicks) {
        sim._wheelTarget = [0, 0];
        if (sub.goal) {
          const [gx, gy] = sub.goal;
          const gs = sub.goalSpeed ?? 85;
          sub.goal = null;
          const dx = gx - sim.state.x_mm, dy = gy - sim.state.y_mm;
          const dist = Math.hypot(dx, dy);
          if (dist > sim.v.drive_done_mm) {
            this._startSubDrive(sub, dist, gs);
            sub.ux = dx / dist;
            sub.uy = dy / dist;
            sub.yaw0 = sim._yawTotal + wrapAngle(Math.atan2(dy, dx) - sim.state.pose_angle_rad);
            return false;
          }
        }
        return true;
      }
      const acc = sim.v.point_turn_accel_default_radps2;
      const half = 0.5 * sim.v.tread_spacing_mm;
      sub.cmd = Math.min(sub.rate, sub.cmd + acc * sim.dt, Math.sqrt(2 * acc * Math.max(0, rem)));
      sim._wheelTarget = [-sub.sign * sub.cmd * half, sub.sign * sub.cmd * half];
      return false;
    }
    if (sub.kind === "drive") {
      const cur = [sim.state.x_mm, sim.state.y_mm];
      const gone = sub.sign * ((cur[0] - sub.start[0]) * sub.ux + (cur[1] - sub.start[1]) * sub.uy);
      const rem = sub.dist - gone;
      if (rem <= sim.v.drive_done_mm || sub.ticks > (sub.maxTicks ?? 1200)) {
        sim._wheelTarget = [0, 0];
        return true;
      }
      sub.cmd = Math.min(sub.speed, sub.cmd + sim.v.wheel_accel_default_mmps2 * sim.dt,
        Math.sqrt(2 * sim.v.wheel_decel_default_mmps2 * Math.max(0, rem)));
      const steer = sim.v.heading_hold_gain_mmps_per_rad * (sub.yaw0 - sim._yawTotal);
      sim._wheelTarget = [sub.sign * sub.cmd - steer, sub.sign * sub.cmd + steer];
      return false;
    }
    return true;
  }

  _updateObservations() {
    this.navMap.updateFromSensors(this.sim);
    const obs = observeMarkers(this.sim, this.cameraSpec);
    if (obs.charger) this.navMap.recordCharger(obs.charger, this.sim.tick_);
    if (obs.cube) {
      const firstCube = this.navMap.cubePose === null;
      this.navMap.recordCube(obs.cube, this.sim.tick_);
      if (firstCube && this.sim.cubeSensor?.firmware) {
        this.sim.cubeSensor.firmware.setLights([0, 220, 255, 0, 220, 255, 0, 220, 255, 0, 220, 255]);
      }
    }
    return obs;
  }

  _pickExploreTarget() {
    const s = this.sim.state;
    let best = null, bestScore = -1;
    const angles = [0, 45, -45, 90, -90, 135, -135, 180];
    for (const r of [140, 100, 185]) {
      for (const deg of angles) {
        const ang = s.pose_angle_rad + deg * DEG;
        const tx = s.x_mm + r * Math.cos(ang), ty = s.y_mm + r * Math.sin(ang);
        if (Math.abs(tx) > 255 || Math.abs(ty) > 255) continue;
        if (!this.navMap.hasObstacleClearance(tx, ty, 82)) continue;
        if (!this.navMap.segmentClear(s.x_mm, s.y_mm, tx, ty, 65)) continue;
        const score = this.navMap.scoreUnknown(tx, ty) - (Math.abs(deg) * 0.03);
        if (score > bestScore) { bestScore = score; best = [tx, ty]; }
      }
    }
    if (!best) {
      const backAng = Math.atan2(-s.y_mm, -s.x_mm);
      best = [s.x_mm + 90 * Math.cos(backAng), s.y_mm + 90 * Math.sin(backAng)];
    }
    this.navMap.lastFrontier = best;
    return best;
  }

  startUndock() {
    return {
      mode: "undock",
      stage: "drive_off",
      phaseName: "UNDOCKING",
      sub: { kind: "wait", ticks: 0, waitTicks: 1 },
      deadlineTick: this.sim.tick_ + Math.round(6 / this.sim.dt),
      savedLevel: null,
    };
  }

  startExplore(durationS = 15) {
    assert(Number.isFinite(durationS) && durationS > 0 && durationS <= 120, "bad explore duration");
    return {
      mode: "explore",
      stage: (this.sim.chargerContact || this.sim._onDockPlatform()) ? "undock" : "scan_step",
      phaseName: (this.sim.chargerContact || this.sim._onDockPlatform()) ? "UNDOCKING" : "EXPLORING",
      scanLeft: 8,
      sub: { kind: "wait", ticks: 0, waitTicks: 10 },
      deadlineTick: this.sim.tick_ + Math.round(durationS / this.sim.dt),
      savedLevel: null,
    };
  }

  startDock() {
    if (this.sim._dockBody < 0 || this.sim.chargerContact) {
      return {
        mode: "dock",
        stage: this.sim.chargerContact ? "done_ok" : "done_no_charger",
        phaseName: this.sim.chargerContact ? "ON CHARGER" : "NO CHARGER",
        attempts: 0, relocations: 0, scanLeft: 0, wiggleCount: 0,
        sub: { kind: "wait", ticks: 0, waitTicks: 0 },
        deadlineTick: this.sim.tick_, savedLevel: null,
      };
    }
    return {
      mode: "dock",
      stage: this.sim._onDockPlatform() ? "escape_dock" : "find_dwell",
      phaseName: "FINDING CHARGER",
      attempts: 0,
      relocations: 0,
      scanLeft: 8,
      wiggleCount: 0,
      flankCount: 0,
      sub: { kind: "wait", ticks: 0, waitTicks: 24 },
      deadlineTick: this.sim.tick_ + Math.round(70 / this.sim.dt),
      savedLevel: null,
    };
  }

  _pickRelocateTarget(ctx) {
    const s = this.sim.state;
    const sy = s.y_mm >= 0 ? 1 : -1;
    const candidates = [
      [-60, sy * 145], [35, sy * 115], [35, -sy * 115], [20, 0], [-25, -sy * 135], [140, sy * 110],
    ];
    for (let k = 0; k < candidates.length; k++) {
      const [tx, ty] = candidates[(ctx.relocations + k - 1) % candidates.length];
      if (Math.hypot(tx - s.x_mm, ty - s.y_mm) < 45) continue;
      if (!this.navMap.hasObstacleClearance(tx, ty, 70, true)) continue;
      if (this.navMap.segmentClear(s.x_mm, s.y_mm, tx, ty, 50, true)) return [tx, ty];
    }
    return [25, sy * 120];
  }

  restoreControlLevel(ctx) {
    if (ctx && ctx.savedLevel !== null) {
      this.sim.controlLevel = ctx.savedLevel;
      ctx.savedLevel = null;
    }
    this.phase = null;
  }

  step(ctx) {
    const sim = this.sim;
    this.phase = ctx.phaseName;
    if (sim._reactPhase !== "idle") {
      this._updateObservations();
      return { running: true };
    }
    if (!sim.hasControl) {
      sim.requestControl();
      ctx.cliffRecovered = true;
      ctx.sub = { kind: "drive", start: [sim.state.x_mm, sim.state.y_mm], yaw0: sim._yawTotal,
        ux: Math.cos(sim.state.pose_angle_rad), uy: Math.sin(sim.state.pose_angle_rad),
        dist: 55, sign: -1, speed: 75, cmd: 0, ticks: 0 };
      return { running: true };
    }
    this._updateObservations();
    if (ctx.mode === "undock") return this._stepUndock(ctx);
    if (ctx.mode === "explore") return this._stepExplore(ctx);
    return this._stepDock(ctx);
  }

  _stepUndock(ctx) {
    const sim = this.sim;
    if (ctx.stage === "drive_off") {
      ctx.savedLevel = ctx.savedLevel ?? sim.controlLevel;
      sim.controlLevel = "override";
      sim.commandHeadAngle(-2);
      sim.commandLiftHeight(32);
      this._startSubDrive(ctx.sub, 105, 85);
      ctx.stage = "driving";
      return { running: true };
    }
    const done = this._stepSub(ctx.sub);
    const awayFromDock = !sim.chargerContact && !sim._onDockPlatform();
    if ((done && awayFromDock) || sim.tick_ >= ctx.deadlineTick) {
      this.restoreControlLevel(ctx);
      return { running: false, result: awayFromDock ? "completed" : "timeout" };
    }
    return { running: true };
  }

  _stepExplore(ctx) {
    const sim = this.sim;
    if (sim.tick_ >= ctx.deadlineTick) {
      this.restoreControlLevel(ctx);
      const { mapped, cliffs } = this.navMap.counts();
      return {
        running: false, result: "completed", cells_mapped: mapped, cliffs_found: cliffs,
        cube_seen: this.navMap.cubePose !== null, charger_seen: this.navMap.chargerPose !== null,
      };
    }
    if (ctx.stage === "undock") {
      if (ctx.savedLevel === null) {
        ctx.savedLevel = sim.controlLevel;
        sim.controlLevel = "override";
        sim.commandHeadAngle(4);
        sim.commandLiftHeight(32);
        this._startSubDrive(ctx.sub, 105, 85);
        return { running: true };
      }
      if (!this._stepSub(ctx.sub)) return { running: true };
      ctx.cliffRecovered = false;
      this.restoreControlLevel(ctx);
      ctx.phaseName = "EXPLORING";
      ctx.stage = "scan_step";
      ctx.scanLeft = 8;
      this._startSubTurn(ctx.sub, 180, 110);
      return { running: true };
    }
    ctx.cliffRecovered = false;
    if (!this._stepSub(ctx.sub)) {
      if (ctx.stage === "drive" && this.navMap.getCell(sim.state.x_mm + 55 * Math.cos(sim.state.pose_angle_rad),
        sim.state.y_mm + 55 * Math.sin(sim.state.pose_angle_rad)) === CELL_PROX) {
        ctx.stage = "scan_step";
        ctx.scanLeft = 4;
        this._startSubTurn(ctx.sub, 65, 110);
      }
      return { running: true };
    }
    if (ctx.stage === "scan_step") {
      if (ctx.scanLeft > 0) {
        sim.commandHeadAngle(4);
        sim.commandLiftHeight(32);
        ctx.scanLeft -= 1;
        this._startSubTurn(ctx.sub, 45, 110);
        ctx.stage = "scan_dwell";
      } else {
        const [tx, ty] = this._pickExploreTarget();
        this._driveToPoint(ctx.sub, tx, ty, 85);
        ctx.stage = "drive";
      }
      return { running: true };
    }
    if (ctx.stage === "scan_dwell") {
      ctx.sub = { kind: "wait", ticks: 0, waitTicks: 18 };
      ctx.stage = "scan_step";
      return { running: true };
    }
    const [tx, ty] = this._pickExploreTarget();
    this._driveToPoint(ctx.sub, tx, ty, 85);
    ctx.stage = "drive";
    return { running: true };
  }

  _stepDock(ctx) {
    const sim = this.sim;
    if (ctx.stage === "done_ok") return { running: false, result: "completed", attempts: ctx.attempts };
    if (ctx.stage === "done_no_charger") return { running: false, result: "no_charger", attempts: 0 };
    if (sim.tick_ >= ctx.deadlineTick) {
      this.restoreControlLevel(ctx);
      return { running: false, result: "timeout", attempts: ctx.attempts };
    }
    if (ctx.stage === "escape_dock") {
      if (ctx.savedLevel === null) {
        ctx.savedLevel = sim.controlLevel;
        sim.controlLevel = "override";
        sim.commandHeadAngle(4);
        sim.commandLiftHeight(32);
        this._startSubDrive(ctx.sub, 105, 85);
        return { running: true };
      }
      if (!this._stepSub(ctx.sub)) return { running: true };
      this.restoreControlLevel(ctx);
      ctx.phaseName = "FINDING CHARGER";
      ctx.stage = "find_dwell";
      ctx.sub = { kind: "wait", ticks: 0, waitTicks: 24 };
      return { running: true };
    }
    if (ctx.stage === "find_dwell" || ctx.stage === "find_turn") {
      sim.commandHeadAngle(4);
      sim.commandLiftHeight(32);
      const chargerKnown = this.navMap.chargerPose
        && (sim.tick_ - this.navMap.chargerPose.seenAt <= 30 || !ctx.triedMemory);
      if (ctx.stage === "find_dwell" && chargerKnown) {
        ctx.triedMemory = true;
        ctx.phaseName = "APPROACHING CHARGER";
        return this._planApproach(ctx);
      }
      if (!this._stepSub(ctx.sub)) return { running: true };
      if (ctx.cliffRecovered) {
        ctx.cliffRecovered = false;
        ctx.relocations += 1;
        ctx.scanLeft = 0;
        const [tx, ty] = this._pickRelocateTarget(ctx);
        this._driveToPoint(ctx.sub, tx, ty, 85);
        ctx.stage = "relocate";
        return { running: true };
      }
      if (ctx.stage === "find_dwell") {
        if (this.navMap.chargerPose && (sim.tick_ - this.navMap.chargerPose.seenAt <= 30 || !ctx.triedMemory)) {
          ctx.triedMemory = true;
          ctx.phaseName = "APPROACHING CHARGER";
          return this._planApproach(ctx);
        }
        if (ctx.scanLeft > 0) {
          ctx.scanLeft -= 1;
          this._startSubTurn(ctx.sub, 45, 110);
          ctx.stage = "find_turn";
          return { running: true };
        }
        if (ctx.relocations >= 4) {
          this.restoreControlLevel(ctx);
          return { running: false, result: "charger_not_found", attempts: ctx.attempts };
        }
        ctx.relocations += 1;
        const [tx, ty] = this._pickRelocateTarget(ctx);
        this._driveToPoint(ctx.sub, tx, ty, 85);
        ctx.stage = "relocate";
        ctx.scanLeft = 8;
        return { running: true };
      }
      ctx.sub = { kind: "wait", ticks: 0, waitTicks: 24 };
      ctx.stage = "find_dwell";
      return { running: true };
    }
    if (ctx.stage === "relocate") {
      if (this.navMap.chargerPose && sim.tick_ - this.navMap.chargerPose.seenAt <= 25) {
        ctx.phaseName = "APPROACHING CHARGER";
        return this._planApproach(ctx);
      }
      if (!this._stepSub(ctx.sub)) return { running: true };
      if (ctx.cliffRecovered) {
        ctx.cliffRecovered = false;
        ctx.relocations += 1;
        const [tx, ty] = this._pickRelocateTarget(ctx);
        this._driveToPoint(ctx.sub, tx, ty, 85);
        return { running: true };
      }
      ctx.sub = { kind: "wait", ticks: 0, waitTicks: 24 };
      ctx.stage = "find_dwell";
      return { running: true };
    }
    if (ctx.stage === "route_flank" || ctx.stage === "route_predock" || ctx.stage === "face_marker") {
      if (!this._stepSub(ctx.sub)) return { running: true };
      if (ctx.cliffRecovered) {
        ctx.cliffRecovered = false;
        ctx.relocations += 1;
        const [tx, ty] = this._pickRelocateTarget(ctx);
        this._driveToPoint(ctx.sub, tx, ty, 85);
        ctx.stage = "relocate";
        return { running: true };
      }
      if (ctx.stage === "route_flank") return this._planApproach(ctx);
      if (ctx.stage === "route_predock") {
        const ch = this.navMap.chargerPose;
        const faceYaw = Math.atan2(-ch.outY, -ch.outX);
        ctx.sub.goal = null;
        this._startSubTurn(ctx.sub, wrapAngle(faceYaw - sim.state.pose_angle_rad) / DEG, 100);
        ctx.stage = "face_marker";
        return { running: true };
      }
      ctx.sub = { kind: "wait", ticks: 0, waitTicks: 24 };
      ctx.stage = "close_fix";
      return { running: true };
    }
    if (ctx.stage === "close_fix") {
      if (!this._stepSub(ctx.sub)) return { running: true };
      const ch = this.navMap.chargerPose;
      if (!ch || sim.tick_ - ch.seenAt > 60) {
        this.navMap.chargerPose = null;
        ctx.fixCount = 0;
        ctx.flankCount = 0;
        ctx.phaseName = "FINDING CHARGER";
        ctx.scanLeft = 8;
        ctx.sub = { kind: "wait", ticks: 0, waitTicks: 10 };
        ctx.stage = "find_dwell";
        return { running: true };
      }
      const lat = Math.abs(-(sim.state.x_mm - ch.lipX) * ch.outY + (sim.state.y_mm - ch.lipY) * ch.outX);
      const fwd = (sim.state.x_mm - ch.lipX) * ch.outX + (sim.state.y_mm - ch.lipY) * ch.outY;
      if ((lat > 4 || fwd < 60 || fwd > 98) && (ctx.fixCount ?? 0) < 2) {
        ctx.fixCount = (ctx.fixCount ?? 0) + 1;
        if (lat > 5 && fwd < 120) {
          this._driveToPoint(ctx.sub, ch.lipX + 145 * ch.outX, ch.lipY + 145 * ch.outY, 75);
        } else {
          this._driveToPoint(ctx.sub, ch.lipX + 78 * ch.outX, ch.lipY + 78 * ch.outY, 65);
        }
        ctx.stage = "route_predock";
        return { running: true };
      }
      ctx.fixCount = 0;
      ctx.wiggleCount = 0;
      ctx.phaseName = "DOCKING";
      sim.commandLiftHeight(45);
      const outYaw = Math.atan2(ch.outY, ch.outX);
      ctx.mountOutYaw = outYaw;
      this._startSubTurn(ctx.sub, wrapAngle(outYaw - sim.state.pose_angle_rad) / DEG, 95);
      ctx.stage = "align_back";
      return { running: true };
    }
    if (ctx.stage === "align_back") {
      if (!this._stepSub(ctx.sub)) return { running: true };
      ctx.savedLevel = sim.controlLevel;
      sim.controlLevel = "override";
      sim.commandLiftHeight(45);
      ctx.attempts += 1;
      ctx.backupTicks = 0;
      ctx.stage = "backup";
      return { running: true };
    }
    if (ctx.stage === "backup") {
      ctx.backupTicks += 1;
      const pitchDeg = Math.abs(sim._pitch() / DEG);
      const zMm = sim.poseMm()[2];
      if (zMm < -5 || pitchDeg > 25) {
        sim._wheelTarget = [0, 0];
        this.restoreControlLevel(ctx);
        return { running: false, result: "aborted_off_ground", attempts: ctx.attempts };
      }
      const ch = this.navMap.chargerPose;
      const outYaw = ctx.mountOutYaw;
      const yaw = sim.state.pose_angle_rad;
      const rearX = sim.state.x_mm - 44 * Math.cos(yaw);
      const rearY = sim.state.y_mm - 44 * Math.sin(yaw);
      const rearFwd = ch ? (rearX - ch.lipX) * ch.outX + (rearY - ch.lipY) * ch.outY : 0;
      const rearLat = ch ? -(rearX - ch.lipX) * ch.outY + (rearY - ch.lipY) * ch.outX : 0;
      const twisted = ctx.backupTicks > 90 && Math.abs(wrapAngle(outYaw - yaw)) > 10 * DEG && !sim.chargerContact;
      const pastRear = ch && !sim.chargerContact && !sim._onDockPlatform() && rearFwd < -95;
      if (sim.chargerContact || ctx.backupTicks > 320 || twisted || pastRear) {
        sim._wheelTarget = [0, 0];
        sim._wheelAccel = [0, 0];
        this.restoreControlLevel(ctx);
        ctx.sub = { kind: "wait", ticks: 0, waitTicks: 22 };
        ctx.stage = "verify";
        return { running: true };
      }
      const spd = [-102, -112, -96, -122, -106][ctx.wiggleCount % 5];
      const desYawErr = rearFwd > -2 ? clamp(0.015 * rearLat, -0.20, 0.20) : 0;
      const steer = rearFwd > -2 ? 140 * wrapAngle(outYaw + desYawErr - yaw) : 0;
      sim._wheelTarget = [spd - steer, spd + steer];
      return { running: true };
    }
    if (ctx.stage === "verify") {
      if (!this._stepSub(ctx.sub)) return { running: true };
      if (sim.chargerContact) {
        sim.commandHeadAngle(0);
        sim.commandLiftHeight(32);
        if (sim.cubeSensor?.firmware) {
          sim.cubeSensor.firmware.setLights([0, 255, 96, 0, 255, 96, 0, 255, 96, 0, 255, 96]);
        }
        this.phase = null;
        return { running: false, result: "completed", attempts: ctx.attempts };
      }
      if (ctx.wiggleCount < 5) {
        ctx.wiggleCount += 1;
        ctx.savedLevel = sim.controlLevel;
        sim.controlLevel = "override";
        ctx.pullTicks = 0;
        ctx.stage = "wiggle_fwd";
        return { running: true };
      }
      ctx.wiggleCount = 0;
      if (ctx.attempts >= 3) {
        this.restoreControlLevel(ctx);
        return { running: false, result: "dock_failed", attempts: ctx.attempts };
      }
      this._startSubDrive(ctx.sub, 125, 85);
      ctx.stage = "retry_away";
      return { running: true };
    }
    if (ctx.stage === "wiggle_fwd") {
      ctx.pullTicks += 1;
      const ch = this.navMap.chargerPose;
      const outYaw = ch ? Math.atan2(ch.outY, ch.outX) : ctx.mountOutYaw;
      const frontFwd = ch ? (sim.state.x_mm - ch.lipX) * ch.outX + (sim.state.y_mm - ch.lipY) * ch.outY : 80;
      const frontLat = ch ? -(sim.state.x_mm - ch.lipX) * ch.outY + (sim.state.y_mm - ch.lipY) * ch.outX : 0;
      const pullGoal = ctx.wiggleCount % 2 === 0 ? 90 : 78;
      if (frontFwd >= pullGoal || ctx.pullTicks >= 165) {
        sim._wheelTarget = [0, 0];
        ctx.mountOutYaw = outYaw;
        ctx.alignTicks = 0;
        ctx.stage = "wiggle_align";
        return { running: true };
      }
      const desYaw = outYaw + clamp(-0.015 * frontLat, -0.25, 0.25);
      const steer = 160 * wrapAngle(desYaw - sim.state.pose_angle_rad);
      sim._wheelTarget = [85 - steer, 85 + steer];
      return { running: true };
    }
    if (ctx.stage === "wiggle_align") {
      ctx.alignTicks += 1;
      const err = wrapAngle(ctx.mountOutYaw - sim.state.pose_angle_rad);
      if (Math.abs(err) < 0.015 || ctx.alignTicks >= 120) {
        sim._wheelTarget = [0, 0];
        ctx.backupTicks = 0;
        ctx.stage = "backup";
        return { running: true };
      }
      const w = clamp(140 * err, -45, 45);
      sim._wheelTarget = [-w, w];
      return { running: true };
    }
    if (ctx.stage === "retry_away") {
      if (!this._stepSub(ctx.sub)) return { running: true };
      this._startSubTurn(ctx.sub, 180, 100);
      ctx.stage = "retry_turn";
      return { running: true };
    }
    if (!this._stepSub(ctx.sub)) return { running: true };
    ctx.sub = { kind: "wait", ticks: 0, waitTicks: 24 };
    ctx.stage = "find_dwell";
    return { running: true };
  }

  _pickFlank(lipX, lipY, outX, outY, sideSign) {
    const s = this.sim.state;
    const perpX = -outY, perpY = outX;
    for (const sgn of [sideSign, -sideSign]) {
      for (const latOff of [125, 95, 150]) {
        const fx = clamp(lipX + 85 * outX + sgn * latOff * perpX, -235, 235);
        const fy = clamp(lipY + 85 * outY + sgn * latOff * perpY, -235, 235);
        if (this.navMap.hasObstacleClearance(fx, fy, 65, true)
          && this.navMap.segmentClear(s.x_mm, s.y_mm, fx, fy, 52, true)) {
          return [fx, fy];
        }
      }
    }
    return [
      clamp(lipX + 85 * outX + sideSign * 125 * perpX, -235, 235),
      clamp(lipY + 85 * outY + sideSign * 125 * perpY, -235, 235),
    ];
  }

  _planApproach(ctx) {
    const sim = this.sim, ch = this.navMap.chargerPose;
    const along = (sim.state.x_mm - ch.lipX) * ch.outX + (sim.state.y_mm - ch.lipY) * ch.outY;
    const latSigned = -(sim.state.x_mm - ch.lipX) * ch.outY + (sim.state.y_mm - ch.lipY) * ch.outX;
    const dists = along >= 65 && along <= 165 && Math.abs(latSigned) <= 18 ? [78, 102, 135] : [135, 102, 82];
    if (along >= 25) {
      for (const d of dists) {
        const tx = ch.lipX + d * ch.outX, ty = ch.lipY + d * ch.outY;
        if (this.navMap.segmentClear(sim.state.x_mm, sim.state.y_mm, tx, ty, 52, true)) {
          ctx.flankCount = 0;
          this._driveToPoint(ctx.sub, tx, ty, 80);
          ctx.stage = "route_predock";
          return { running: true };
        }
      }
    }
    if ((ctx.flankCount ?? 0) < 2) {
      ctx.flankCount = (ctx.flankCount ?? 0) + 1;
      const [fx, fy] = this._pickFlank(ch.lipX, ch.lipY, ch.outX, ch.outY, latSigned >= 0 ? 1 : -1);
      this._driveToPoint(ctx.sub, fx, fy, 85);
      ctx.stage = "route_flank";
      return { running: true };
    }
    ctx.flankCount = 0;
    this._driveToPoint(ctx.sub, ch.lipX + 85 * ch.outX, ch.lipY + 85 * ch.outY, 75);
    ctx.stage = "route_predock";
    return { running: true };
  }
}
