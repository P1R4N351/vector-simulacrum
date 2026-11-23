// Calibrated 64x36 RGB head_cam ray-caster mirroring simulacrum/nomad_cam.py.
import { assert } from "./util.js";

export const IMG_W = 64;
export const IMG_H = 36;
export const IMG_C = 3;
const MM = 0.001;
const NEAR_M = 0.005;
const DEG = Math.PI / 180;

const DEFAULT_CAMERA_SPEC = {
  name: "head_cam",
  width_px: 640,
  height_px: 360,
  fx_px: 361.1599,
  fy_px: 356.2424,
  cx_px: 310.661,
  cy_px: 183.827,
  pitch_down_deg: 4.0,
  pitch_down_in_model: true,
  rgb_pitch_up_deg: 0.0,
};

function clamp(value, low, high) {
  return value < low ? low : value > high ? high : value;
}

function cameraRaysWorld(sim, spec) {
  const camId = sim.mujoco.mj_name2id(sim.model, sim.mujoco.mjtObj.mjOBJ_CAMERA.value, spec.name);
  if (camId < 0) return null;
  const { cam_xpos, cam_xmat } = sim.data;
  const ox = cam_xpos[3 * camId], oy = cam_xpos[3 * camId + 1], oz = cam_xpos[3 * camId + 2];
  const mountDown = spec.pitch_down_in_model ? 0 : spec.pitch_down_deg;
  const tilt = ((spec.rgb_pitch_up_deg ?? 0) - mountDown) * DEG;
  const ct = Math.cos(tilt), st = Math.sin(tilt);
  const k = 9 * camId;
  const rCamW = [
    cam_xmat[k], cam_xmat[k + 1] * ct + cam_xmat[k + 2] * st, -cam_xmat[k + 1] * st + cam_xmat[k + 2] * ct,
    cam_xmat[k + 3], cam_xmat[k + 4] * ct + cam_xmat[k + 5] * st, -cam_xmat[k + 4] * st + cam_xmat[k + 5] * ct,
    cam_xmat[k + 6], cam_xmat[k + 7] * ct + cam_xmat[k + 8] * st, -cam_xmat[k + 7] * st + cam_xmat[k + 8] * ct,
  ];
  const dirsW = new Float64Array(IMG_W * IMG_H * 3);
  const sx = spec.width_px / IMG_W, sy = spec.height_px / IMG_H;
  for (let row = 0; row < IMG_H; row++) {
    const gy = (spec.cy_px - (row + 0.5) * sy) / spec.fy_px;
    for (let col = 0; col < IMG_W; col++) {
      const gx = ((col + 0.5) * sx - spec.cx_px) / spec.fx_px;
      const invNorm = 1 / Math.hypot(gx, gy, 1);
      const cx = gx * invNorm, cy = gy * invNorm, cz = -invNorm;
      const o = 3 * (row * IMG_W + col);
      dirsW[o] = rCamW[0] * cx + rCamW[1] * cy + rCamW[2] * cz;
      dirsW[o + 1] = rCamW[3] * cx + rCamW[4] * cy + rCamW[5] * cz;
      dirsW[o + 2] = rCamW[6] * cx + rCamW[7] * cy + rCamW[8] * cz;
    }
  }
  return { origin: [ox, oy, oz], dirsW };
}

function rayObb(origin, dx, dy, dz, pos, rot, rotOff, bmin, bmax, outLocal) {
  const rx = origin[0] - pos[0], ry = origin[1] - pos[1], rz = origin[2] - pos[2];
  const m0 = rot[rotOff], m1 = rot[rotOff + 1], m2 = rot[rotOff + 2];
  const m3 = rot[rotOff + 3], m4 = rot[rotOff + 4], m5 = rot[rotOff + 5];
  const m6 = rot[rotOff + 6], m7 = rot[rotOff + 7], m8 = rot[rotOff + 8];
  const o0 = m0 * rx + m3 * ry + m6 * rz;
  const o1 = m1 * rx + m4 * ry + m7 * rz;
  const o2 = m2 * rx + m5 * ry + m8 * rz;
  const d0 = m0 * dx + m3 * dy + m6 * dz;
  const d1 = m1 * dx + m4 * dy + m7 * dz;
  const d2 = m2 * dx + m5 * dy + m8 * dz;
  let tmin = -Infinity, tmax = Infinity;
  const oa = [o0, o1, o2], da = [d0, d1, d2];
  for (let a = 0; a < 3; a++) {
    if (Math.abs(da[a]) <= 1e-8) {
      if (oa[a] < bmin[a] || oa[a] > bmax[a]) return Infinity;
    } else {
      const r0 = (bmin[a] - oa[a]) / da[a], r1 = (bmax[a] - oa[a]) / da[a];
      const lo = r0 < r1 ? r0 : r1, hi = r0 > r1 ? r0 : r1;
      if (lo > tmin) tmin = lo;
      if (hi < tmax) tmax = hi;
      if (tmax < tmin) return Infinity;
    }
  }
  if (tmin <= NEAR_M) return Infinity;
  if (outLocal) {
    outLocal[0] = o0 + tmin * d0;
    outLocal[1] = o1 + tmin * d1;
    outLocal[2] = o2 + tmin * d2;
  }
  return tmin;
}

function shadeBackdropAndTable(origin, dirsW, sim, tBest, rgb) {
  const halfL = 0.5 * sim.v.table_length_mm * MM;
  const halfW = 0.5 * sim.v.table_width_mm * MM;
  const oz = origin[2];
  for (let p = 0; p < IMG_W * IMG_H; p++) {
    const o = 3 * p;
    const dx = dirsW[o], dy = dirsW[o + 1], dz = dirsW[o + 2];
    if (dz >= 0) {
      const up = clamp(dz * 2.5, 0, 1);
      rgb[o] = 28 + 28 * up;
      rgb[o + 1] = 34 + 34 * up;
      rgb[o + 2] = 46 + 42 * up;
    } else {
      rgb[o] = 18;
      rgb[o + 1] = 22;
      rgb[o + 2] = 28;
      if (dz < -1e-6) {
        const tp = -oz / dz;
        if (tp > NEAR_M && tp < tBest[p]) {
          const hx = origin[0] + tp * dx, hy = origin[1] + tp * dy;
          if (Math.abs(hx) <= halfL && Math.abs(hy) <= halfW) {
            const edgeDist = Math.min(halfL - Math.abs(hx), halfW - Math.abs(hy));
            const fade = clamp(1.12 - 0.55 * tp, 0.55, 1.0);
            tBest[p] = tp;
            if (edgeDist < 0.022) {
              rgb[o] = 224 * fade;
              rgb[o + 1] = 184 * fade;
              rgb[o + 2] = 118 * fade;
            } else {
              rgb[o] = 148 * fade;
              rgb[o + 1] = 118 * fade;
              rgb[o + 2] = 86 * fade;
            }
          }
        }
      }
    }
  }
}

function shadeObstacles(origin, dirsW, sim, tBest, rgb) {
  const { mujoco, model, data } = sim;
  const boxKind = mujoco.mjtGeom.mjGEOM_BOX.value;
  const geomObj = mujoco.mjtObj.mjOBJ_GEOM.value;
  for (let g = 0; g < model.ngeom; g++) {
    if (model.geom_type[g] !== boxKind) continue;
    const name = mujoco.mj_id2name(model, geomObj, g) || "";
    if (!name.startsWith("obstacle_")) continue;
    const pos = [data.geom_xpos[3 * g], data.geom_xpos[3 * g + 1], data.geom_xpos[3 * g + 2]];
    const sx = model.geom_size[3 * g], sy = model.geom_size[3 * g + 1], sz = model.geom_size[3 * g + 2];
    const bmin = [-sx, -sy, -sz], bmax = [sx, sy, sz];
    for (let p = 0; p < IMG_W * IMG_H; p++) {
      const o = 3 * p;
      const th = rayObb(origin, dirsW[o], dirsW[o + 1], dirsW[o + 2], pos, data.geom_xmat, 9 * g, bmin, bmax, null);
      if (th < tBest[p]) {
        tBest[p] = th;
        rgb[o] = 182;
        rgb[o + 1] = 190;
        rgb[o + 2] = 202;
      }
    }
  }
}

function shadeDock(origin, dirsW, sim, tBest, rgb) {
  const db = sim._dockBody;
  if (db < 0) return;
  const { xpos, xmat } = sim.data;
  const pos = [xpos[3 * db], xpos[3 * db + 1], xpos[3 * db + 2]];
  const hl = [0, 0, 0];
  const boxes = [
    [[0, -0.028, 0], [0.092, 0.028, 0.016], false],
    [[0.092, -0.028, 0.014], [0.116, 0.028, 0.082], true],
  ];
  for (const [bmin, bmax, isTower] of boxes) {
    for (let p = 0; p < IMG_W * IMG_H; p++) {
      const o = 3 * p;
      const th = rayObb(origin, dirsW[o], dirsW[o + 1], dirsW[o + 2], pos, xmat, 9 * db, bmin, bmax, hl);
      if (th >= tBest[p]) continue;
      tBest[p] = th;
      if (!isTower) {
        rgb[o] = 38; rgb[o + 1] = 42; rgb[o + 2] = 50;
      } else {
        const u = Math.abs(hl[1]), v = Math.abs(hl[2] - 0.0485);
        if (hl[0] < 0.0925 && u < 0.023 && v < 0.023) {
          const bright = Math.max(u, v) > 0.015 || (u < 0.009 && v < 0.009);
          rgb[o] = bright ? 242 : 20;
          rgb[o + 1] = bright ? 244 : 24;
          rgb[o + 2] = bright ? 248 : 30;
        } else {
          rgb[o] = 44; rgb[o + 1] = 48; rgb[o + 2] = 58;
        }
      }
    }
  }
}

function shadeCube(origin, dirsW, sim, tBest, rgb) {
  const cb = sim._cubeBody;
  if (cb < 0) return;
  const { xpos, xmat } = sim.data;
  const pos = [xpos[3 * cb], xpos[3 * cb + 1], xpos[3 * cb + 2]];
  const bmin = [-0.022, -0.022, -0.022], bmax = [0.022, 0.022, 0.022];
  const hl = [0, 0, 0];
  for (let p = 0; p < IMG_W * IMG_H; p++) {
    const o = 3 * p;
    const th = rayObb(origin, dirsW[o], dirsW[o + 1], dirsW[o + 2], pos, xmat, 9 * cb, bmin, bmax, hl);
    if (th >= tBest[p]) continue;
    tBest[p] = th;
    const ax = Math.abs(hl[0]), ay = Math.abs(hl[1]), az = Math.abs(hl[2]);
    const onX = ax >= 0.0216, onY = ay >= 0.0216, onZ = az >= 0.0216;
    const u = onX ? ay : ax;
    const v = onZ ? ay : az;
    const inSq = (onX || onY) && !onZ && u < 0.014 && v < 0.014;
    const whiteFrame = inSq && (Math.max(u, v) > 0.009 || Math.max(u, v) < 0.0045);
    const topLed = onZ && Math.max(ax, ay) > 0.008 && Math.max(ax, ay) < 0.015;
    if (topLed) {
      rgb[o] = 60; rgb[o + 1] = 205; rgb[o + 2] = 235;
    } else if (whiteFrame) {
      rgb[o] = 236; rgb[o + 1] = 238; rgb[o + 2] = 242;
    } else if (inSq) {
      rgb[o] = 24; rgb[o + 1] = 28; rgb[o + 2] = 34;
    } else {
      rgb[o] = 48; rgb[o + 1] = 52; rgb[o + 2] = 60;
    }
  }
}

export function renderHeadCamRgb(sim, spec = null) {
  assert(sim !== null && sim !== undefined, "sim required for head_cam render");
  const camSpec = spec ?? sim.cameraSpec ?? DEFAULT_CAMERA_SPEC;
  const rays = cameraRaysWorld(sim, camSpec);
  assert(rays !== null, "head_cam missing from MuJoCo model");
  const { origin, dirsW } = rays;
  const rgb = new Float64Array(IMG_W * IMG_H * IMG_C);
  const tBest = new Float64Array(IMG_W * IMG_H).fill(Infinity);
  shadeBackdropAndTable(origin, dirsW, sim, tBest, rgb);
  shadeObstacles(origin, dirsW, sim, tBest, rgb);
  shadeDock(origin, dirsW, sim, tBest, rgb);
  shadeCube(origin, dirsW, sim, tBest, rgb);
  const out = new Uint8Array(IMG_W * IMG_H * IMG_C);
  for (let i = 0; i < rgb.length; i++) {
    out[i] = clamp(Math.round(rgb[i]), 0, 255);
  }
  const liftMm = sim.state?.lift_height_mm ?? sim.liftHeightMm();
  if (liftMm > 38 && liftMm < 62) {
    for (let row = IMG_H - 7; row < IMG_H; row++) {
      for (let col = 8; col < IMG_W - 8; col++) {
        const o = 3 * (row * IMG_W + col);
        out[o] = 52; out[o + 1] = 56; out[o + 2] = 64;
      }
    }
  }
  return out;
}
