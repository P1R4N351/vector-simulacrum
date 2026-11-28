// Two-robot continuous free-body righting macro (direct and cube-assisted).
// Ports experiments/righting/rescue_pipeline.py to browser MuJoCo 4.0.1 WASM.

import { assert, clamp, pyRound } from "./util.js";

const DEG = Math.PI / 180;
const HALF_PI = Math.PI / 2;
const MM = 0.001;
const RESCUE_DT = 0.005;
const CARRY_RELPOSE_X = 0.0335;
const CARRY_RELPOSE_Z = -0.0125;
const APPROACH_STANDOFF_M = 0.120;
const DOCK_STANDOFF_M = 0.080;
const STAGE_A_MAX_TICKS = Math.round(6.0 / RESCUE_DT);
const STAGE_B_TICKS = Math.round(1.5 / RESCUE_DT);
const WAYPOINT_MAX_TICKS = Math.round(6.0 / RESCUE_DT);
const SETTLE_TICKS = Math.round(0.15 / RESCUE_DT);
const LEVER_TICKS = Math.round(3.8 / RESCUE_DT);
const MAX_STAGE_TRANSITIONS = 8;

function mulQuat(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

function axisQuat(ax, ay, az, angle) {
  const half = 0.5 * angle;
  const s = Math.sin(half);
  return [Math.cos(half), ax * s, ay * s, az * s];
}

// Quaternion placing patient on right (side=+1) or left (side=-1) side.
export function sidewaysQuat(side) {
  assert(side === 1 || side === -1, "side must be +1 (right) or -1 (left)");
  return mulQuat(axisQuat(0, 0, 1, side * HALF_PI), axisQuat(1, 0, 0, side * HALF_PI));
}

function normalizeSide(side) {
  if (side === -1 || side === "left") return -1;
  if (side === 1 || side === "right" || side === undefined || side === null) return 1;
  throw new Error(`unknown rescue side: ${side}`);
}

function treadDrive(v, wheelRadps, speedMmps) {
  const measured = wheelRadps * v.wheel_front_radius_mm;
  const raw = Math.sign(speedMmps) * v.tread_friction_power_fraction
    + (speedMmps + 0.6 * (speedMmps - measured)) / v.tread_noload_speed_mmps;
  return clamp(raw, -1.0, 1.0);
}

function wrapRad(angle) {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

function withMatMulVec(m, o, vec) {
  return [
    m[o] * vec[0] + m[o + 1] * vec[1] + m[o + 2] * vec[2],
    m[o + 3] * vec[0] + m[o + 4] * vec[1] + m[o + 5] * vec[2],
    m[o + 6] * vec[0] + m[o + 7] * vec[1] + m[o + 8] * vec[2],
  ];
}

export class RescueController {
  constructor(sim) {
    assert(sim && sim.mujoco && sim.model, "RescueController needs a VectorSim");
    this.sim = sim;
    const { mujoco, model } = sim;
    const id = (kind, name) => mujoco.mj_name2id(model, mujoco.mjtObj[kind].value, name);
    const req = (kind, name) => {
      const found = id(kind, name);
      assert(found >= 0, `rescue scene missing ${kind} ${name}`);
      return found;
    };
    this._chassis = req("mjOBJ_BODY", "chassis");
    this._patChassis = req("mjOBJ_BODY", "patient_chassis");
    this._forkBody = req("mjOBJ_BODY", "lift_fork");
    this._cubeBody = id("mjOBJ_BODY", "cube");
    this._carryWeld = id("mjOBJ_EQUALITY", "carry_weld");
    this._patForkGeom = req("mjOBJ_GEOM", "patient_lift_fork_geom");
    this._q = {
      root: model.jnt_qposadr[req("mjOBJ_JOINT", "root")],
      head: model.jnt_qposadr[req("mjOBJ_JOINT", "head_joint")],
      lift: model.jnt_qposadr[req("mjOBJ_JOINT", "lift_joint")],
      wrist: model.jnt_qposadr[req("mjOBJ_JOINT", "wrist_joint")],
      patRoot: model.jnt_qposadr[req("mjOBJ_JOINT", "patient_root")],
      patHead: model.jnt_qposadr[req("mjOBJ_JOINT", "patient_head_joint")],
      patLift: model.jnt_qposadr[req("mjOBJ_JOINT", "patient_lift_joint")],
      patWrist: model.jnt_qposadr[req("mjOBJ_JOINT", "patient_wrist_joint")],
      cubeRoot: this._cubeBody >= 0 ? model.jnt_qposadr[req("mjOBJ_JOINT", "cube_root")] : -1,
    };
    this._patWheelQ = ["fl", "fr", "rl", "rr"].map(k => model.jnt_qposadr[req("mjOBJ_JOINT", `patient_wheel_${k}_joint`)]);
    this._dofWheelFl = model.jnt_dofadr[req("mjOBJ_JOINT", "wheel_fl_joint")];
    this._act = {
      left: req("mjOBJ_ACTUATOR", "left_wheel"), right: req("mjOBJ_ACTUATOR", "right_wheel"),
      head: req("mjOBJ_ACTUATOR", "head"), lift: req("mjOBJ_ACTUATOR", "lift"),
      patLeft: req("mjOBJ_ACTUATOR", "patient_left_wheel"), patRight: req("mjOBJ_ACTUATOR", "patient_right_wheel"),
      patHead: req("mjOBJ_ACTUATOR", "patient_head"), patLift: req("mjOBJ_ACTUATOR", "patient_lift"),
    };
    this._geomSets = this._buildGeomSets();
    this.side = 1;
    this.mode = this._carryWeld >= 0 ? "cube" : "direct";
    this.stage = "ready";
    this.dropTarget = [0, 0, 0.022];
    this.contactPoint = null;
    this.lastContact = { pc: false, rc: false, dc: false, clear: true, point: null };
  }

  _buildGeomSets() {
    const { mujoco, model } = this.sim;
    const cube = new Set(), pf = new Set(), rf = new Set(), pat = new Set(), act = new Set();
    for (let i = 0; i < model.ngeom; i++) {
      const name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM.value, i) ?? "";
      if (name.startsWith("cube_")) cube.add(i);
      else if (name.startsWith("patient_")) {
        pat.add(i);
        if (name.startsWith("patient_lift_fork_geom") || name.startsWith("patient_fork_side_") || name.startsWith("patient_fork_hook_")) pf.add(i);
      } else if (name !== "floor" && name !== "table") {
        act.add(i);
        if (name.startsWith("lift_fork_geom") || name.startsWith("fork_side_") || name.startsWith("fork_hook_")) rf.add(i);
      }
    }
    return { cube, pf, rf, pat, act };
  }

  _resolveMode(mode) {
    const hasCube = this._carryWeld >= 0 && this._cubeBody >= 0;
    if (mode === "direct") return "direct";
    if (mode === "cube") {
      if (!hasCube) return null;
      return "cube";
    }
    if (mode === "auto" || mode === undefined || mode === null) return hasCube ? "cube" : "direct";
    throw new Error(`unknown rescue mode: ${mode}`);
  }

  patientTiltDeg() {
    const upZ = clamp(this.sim.data.xmat[9 * this._patChassis + 8], -1, 1);
    return Math.acos(upZ) / DEG;
  }

  sampleContacts(hasCube = this.mode === "cube") {
    const { data } = this.sim;
    const { cube, pf, rf, pat, act } = this._geomSets;
    let pc = false, rc = false, dc = false, clear = true, point = null;
    const contacts = data.contact;
    try {
      const ncon = Math.min(data.ncon, contacts.size());
      for (let i = 0; i < ncon; i++) {
        const c = contacts.get(i);
        if (!c) continue;
        try {
          const ga = c.geom1, gb = c.geom2;
          if (cube.has(ga) || cube.has(gb)) {
            const other = cube.has(ga) ? gb : ga;
            if (pf.has(other)) pc = true;
            if (rf.has(other)) {
              rc = true;
              if (point === null) {
                const pos = c.pos;
                point = [pos[0], pos[1], pos[2]];
              }
            }
          }
          if ((act.has(ga) && pat.has(gb)) || (act.has(gb) && pat.has(ga))) {
            dc = true;
            clear = false;
            if ((rf.has(ga) || rf.has(gb)) && !hasCube) rc = true;
            if ((rf.has(ga) || rf.has(gb)) && point === null) {
              const pos = c.pos;
              point = [pos[0], pos[1], pos[2]];
            }
          } else if ((act.has(ga) && cube.has(gb)) || (act.has(gb) && cube.has(ga))) {
            clear = false;
          }
        } finally {
          c.delete?.();
        }
      }
    } finally {
      contacts.delete?.();
    }
    this.contactPoint = point;
    this.lastContact = { pc, rc, dc, clear, point };
    return this.lastContact;
  }

  isRescuerClear() {
    const { data } = this.sim;
    const { act, pat, cube } = this._geomSets;
    const contacts = data.contact;
    try {
      const ncon = Math.min(data.ncon, contacts.size());
      for (let i = 0; i < ncon; i++) {
        const c = contacts.get(i);
        if (!c) continue;
        try {
          const ga = c.geom1, gb = c.geom2;
          if ((act.has(ga) && (pat.has(gb) || cube.has(gb)))
              || (act.has(gb) && (pat.has(ga) || cube.has(ga)))) {
            return false;
          }
        } finally {
          c.delete?.();
        }
      }
      return true;
    } finally {
      contacts.delete?.();
    }
  }

  resetScene({ side = 1, mode = "auto" } = {}) {
    const s = normalizeSide(side);
    const resolved = this._resolveMode(mode) ?? "direct";
    const { mujoco, model, data, d } = this.sim;
    const lo = d.lift_angle_min, hi = d.lift_angle_max;
    this.side = s;
    this.mode = resolved;
    data.qvel.fill(0);
    data.ctrl.fill(0);
    data.qpos.set([0, 0, 0.039, ...sidewaysQuat(s)], this._q.patRoot);
    data.qpos[this._q.patHead] = 0;
    data.qpos[this._q.patLift] = lo;
    data.qpos[this._q.patWrist] = -lo;
    data.ctrl[this._act.patLift] = lo;
    const lift0 = resolved === "cube" ? hi : lo;
    data.qpos[this._q.head] = 0;
    data.qpos[this._q.lift] = lift0;
    data.qpos[this._q.wrist] = -lift0;
    data.ctrl[this._act.lift] = lift0;
    data.qpos.set([0, 0, 0.001, 1, 0, 0, 0], this._q.root);
    mujoco.mj_forward(model, data);

    const forkX = data.geom_xpos[3 * this._patForkGeom], forkY = data.geom_xpos[3 * this._patForkGeom + 1];
    const halfDepth = model.geom_size[3 * this._patForkGeom];
    const laneY = forkY + s * (0.022 + halfDepth + 0.0005);
    this.dropTarget = [forkX, laneY, 0.022];
    data.qpos.set([forkX - 0.056 - APPROACH_STANDOFF_M, laneY, 0.001, 1, 0, 0, 0], this._q.root);
    mujoco.mj_forward(model, data);

    if (this._q.cubeRoot >= 0) {
      if (resolved === "cube" && this._carryWeld >= 0) {
        const wOff = 3 * this._forkBody, mOff = 9 * this._forkBody, qOff = 4 * this._forkBody;
        const delta = withMatMulVec(data.xmat, mOff, [CARRY_RELPOSE_X, 0, CARRY_RELPOSE_Z]);
        data.qpos.set([
          data.xpos[wOff] + delta[0], data.xpos[wOff + 1] + delta[1], data.xpos[wOff + 2] + delta[2],
          data.xquat[qOff], data.xquat[qOff + 1], data.xquat[qOff + 2], data.xquat[qOff + 3],
        ], this._q.cubeRoot);
        mujoco.mj_forward(model, data);
        data.eq_active[this._carryWeld] = 1;
      } else {
        if (this._carryWeld >= 0) data.eq_active[this._carryWeld] = 0;
        data.qpos.set([0.25, -s * 0.25, 0.0225, 1, 0, 0, 0], this._q.cubeRoot);
      }
      mujoco.mj_forward(model, data);
    }
    this.stage = "ready";
    this.contactPoint = null;
    this.lastContact = { pc: false, rc: false, dc: false, clear: true, point: null };
  }

  _buildWaypoints(side, refX) {
    const yaw = 168.7 * DEG * side;
    const targetPt = [refX + 0.025, 0.038 * side];
    const approachXy = [targetPt[0] - 0.140 * Math.cos(yaw), targetPt[1] - 0.140 * Math.sin(yaw)];
    const rx = this.sim.data.xpos[3 * this._chassis];
    return [
      { wp: [rx, -0.15 * side], yaw: 0 },
      { wp: [approachXy[0] + 0.02, -0.15 * side], yaw: 0 },
      { wp: approachXy, yaw },
    ];
  }

  startRescue({ mode = "auto" } = {}) {
    const resolved = this._resolveMode(mode);
    if (resolved === null) return null;
    const { d, data } = this.sim;
    const weldInactive = resolved === "cube" && (this._carryWeld < 0 || data.eq_active[this._carryWeld] === 0);
    if (this.patientTiltDeg() < 15 || this.stage !== "ready" || weldInactive) {
      this.sim.resetRescue(this.side, resolved);
    }
    this.mode = resolved;
    const lo = d.lift_angle_min, hi = d.lift_angle_max;
    const startTilt = this.patientTiltDeg();
    const ctx = {
      controller: this, mode: "rescue", rescueMode: resolved, side: this.side, prevLevel: this.sim.controlLevel,
      stage: resolved === "cube" ? "stage_a" : "stage_c_nav", stages: [], subPhase: resolved === "cube" ? "approach" : "to_wp",
      stageTicks: 0, releasedAtS: null, wpIndex: 0, waypoints: resolved === "cube" ? null : this._buildWaypoints(this.side, 0.0113),
      patLiftHold: lo, leverHitS: null, startTiltDeg: startTilt, closestTiltDeg: startTilt,
      directTicks: 0, forkCubeTicks: 0, deadlineTick: this.sim.tickCount + Math.round(45 / this.sim.dt),
      liftLo: lo, liftHi: hi,
    };
    this.stage = ctx.stage;
    return ctx;
  }

  _rescuerYaw() {
    const m = this.sim.data.xmat, o = 9 * this._chassis;
    return Math.atan2(m[o + 3], m[o]);
  }

  _stepStageA(ctx) {
    const { data, v } = this.sim;
    if (ctx.stageTicks >= STAGE_A_MAX_TICKS) {
      ctx.stages.push("stage_a");
      ctx.stage = "stage_b";
      ctx.stageTicks = 0;
      ctx.subPhase = "ramp";
      ctx.patStartS = data.time;
      return true;
    }
    const cx = data.xpos[3 * this._chassis];
    const stopX = this.dropTarget[0] - 0.0555;
    const reverseX = stopX - DOCK_STANDOFF_M;
    if (ctx.subPhase === "approach" && cx >= stopX) ctx.subPhase = "lower";
    if (ctx.subPhase === "lower" && data.qpos[this._q.lift] <= 0) {
      if (ctx.releasedAtS === null) {
        data.eq_active[this._carryWeld] = 0;
        ctx.releasedAtS = data.time;
      }
      if (data.time - ctx.releasedAtS >= 0.15) ctx.subPhase = "reverse";
    }
    if (ctx.subPhase === "reverse" && cx <= reverseX) {
      ctx.stages.push("stage_a");
      ctx.stage = "stage_b";
      ctx.stageTicks = 0;
      ctx.subPhase = "ramp";
      ctx.patStartS = data.time;
      return true;
    }
    const spd = ctx.subPhase === "approach" ? 40 : (ctx.subPhase === "lower" ? 0 : -40);
    const drv = treadDrive(v, data.qvel[this._dofWheelFl], spd);
    const lift = ctx.subPhase === "approach" ? ctx.liftHi : (ctx.subPhase === "lower" ? 0 : ctx.liftLo);
    this.sim._rescueCtrl = { left: drv, right: drv, lift, patientLift: ctx.liftLo };
    ctx.stageTicks += 1;
    return false;
  }

  _stepStageB(ctx) {
    const { data } = this.sim;
    if (ctx.stageTicks >= STAGE_B_TICKS) {
      ctx.patLiftHold = ctx.liftLo + clamp((data.time - ctx.patStartS) / 0.5, 0, 1) * (15 * DEG);
      const refX = data.xpos[3 * this._cubeBody];
      ctx.waypoints = this._buildWaypoints(ctx.side, refX);
      ctx.stages.push("stage_b");
      ctx.stage = "stage_c_nav";
      ctx.subPhase = "to_wp";
      ctx.wpIndex = 0;
      ctx.stageTicks = 0;
      return true;
    }
    const ramp = clamp((data.time - ctx.patStartS) / 0.5, 0, 1);
    const patLift = ctx.liftLo + (15 * DEG) * ramp;
    ctx.patLiftHold = patLift;
    this.sim._rescueCtrl = { left: 0, right: 0, lift: ctx.liftLo, patientLift: patLift };
    ctx.stageTicks += 1;
    return false;
  }

  _stepStageCNav(ctx) {
    const { data } = this.sim;
    const liftH0 = ctx.liftLo + 0.23 * (ctx.liftHi - ctx.liftLo);
    if (ctx.wpIndex >= ctx.waypoints.length) {
      ctx.stages.push("stage_c_nav");
      ctx.stage = "stage_c_settle";
      ctx.stageTicks = 0;
      return true;
    }
    const goal = ctx.waypoints[ctx.wpIndex];
    if (ctx.stageTicks >= WAYPOINT_MAX_TICKS) {
      if (ctx.subPhase === "to_wp") {
        ctx.subPhase = "align";
        ctx.stageTicks = 0;
      } else {
        ctx.wpIndex += 1;
        ctx.subPhase = "to_wp";
        ctx.stageTicks = 0;
      }
      return true;
    }
    const cyaw = this._rescuerYaw();
    let vCmd = 0, wCmd = 0;
    if (ctx.subPhase === "to_wp") {
      const dx = goal.wp[0] - data.xpos[3 * this._chassis], dy = goal.wp[1] - data.xpos[3 * this._chassis + 1];
      const dist = Math.hypot(dx, dy);
      if (dist <= 0.003) {
        ctx.subPhase = "align";
        ctx.stageTicks = 0;
        return true;
      }
      const err = wrapRad(Math.atan2(dy, dx) - cyaw);
      const turnOnly = Math.abs(err) > 10 * DEG;
      vCmd = turnOnly ? 0 : clamp(3.5 * dist + 0.22, 0.22, 0.55);
      wCmd = turnOnly ? clamp(2.2 * err + 0.18 * Math.sign(err), -0.65, 0.65) : clamp(1.8 * err, -0.35, 0.35);
    } else {
      const err = wrapRad(goal.yaw - cyaw);
      if (Math.abs(err) <= 1.5 * DEG) {
        ctx.wpIndex += 1;
        ctx.subPhase = "to_wp";
        ctx.stageTicks = 0;
        return true;
      }
      wCmd = clamp(2.0 * err + 0.18 * Math.sign(err), -0.6, 0.6);
    }
    this.sim._rescueCtrl = {
      left: clamp(vCmd - wCmd, -1, 1), right: clamp(vCmd + wCmd, -1, 1),
      lift: liftH0, patientLift: ctx.patLiftHold,
    };
    ctx.stageTicks += 1;
    return false;
  }

  _stepStageCLever(ctx) {
    const { data } = this.sim;
    const tilt = this.patientTiltDeg();
    ctx.closestTiltDeg = Math.min(ctx.closestTiltDeg, tilt);
    if (ctx.stageTicks > 0) {
      const { rc, dc } = this.lastContact;
      const prevT = data.time - ctx.leverStartS;
      if (ctx.leverHitS === null && (rc || dc)) ctx.leverHitS = prevT;
      if (dc) ctx.directTicks += 1;
      if (rc) ctx.forkCubeTicks += 1;
    }
    if (ctx.stageTicks >= LEVER_TICKS) {
      ctx.stages.push("stage_c_lever");
      ctx.stage = "done";
      this.stage = "done";
      this.sim._rescueCtrl = null;
      return {
        done: true, result: "completed", mode: ctx.rescueMode,
        side: ctx.side === 1 ? "right" : "left", stages: ctx.stages.slice(),
        start_tilt_deg: Number(ctx.startTiltDeg.toFixed(3)),
        final_tilt_deg: Number(tilt.toFixed(3)),
        closest_tilt_deg: Number(ctx.closestTiltDeg.toFixed(3)),
        rescuer_clear: this.isRescuerClear(),
        direct_contact_ticks: ctx.directTicks, fork_cube_ticks: ctx.forkCubeTicks,
      };
    }
    const t = data.time - ctx.leverStartS;
    const elapsed = ctx.leverHitS !== null ? t - ctx.leverHitS : (t > 1.1 ? Math.max(-1, t - 1.1) : -1);
    const rise = clamp((elapsed - 0.09) / 0.23, 0, 1);
    const lower = clamp((elapsed - 0.09 - 0.75 - 1.0) / 0.45, 0, 1);
    const lift = ctx.liftLo + (ctx.liftHi - ctx.liftLo) * (0.23 + 0.77 * rise) * (1 - lower);
    const drv = elapsed < 0.84 ? 0.92 : (elapsed < 1.64 ? -0.50 : 0);
    this.sim._rescueCtrl = { left: drv, right: drv, lift, patientLift: ctx.patLiftHold };
    ctx.stageTicks += 1;
    return null;
  }

  step(ctx) {
    assert(ctx && ctx.mode === "rescue", "invalid rescue context");
    for (let iter = 0; iter < MAX_STAGE_TRANSITIONS; iter++) {
      this.stage = ctx.stage;
      if (ctx.stage === "stage_a") {
        if (!this._stepStageA(ctx)) return { running: true };
      } else if (ctx.stage === "stage_b") {
        if (!this._stepStageB(ctx)) return { running: true };
      } else if (ctx.stage === "stage_c_nav") {
        if (!this._stepStageCNav(ctx)) return { running: true };
      } else if (ctx.stage === "stage_c_settle") {
        if (ctx.stageTicks >= SETTLE_TICKS) {
          ctx.stages.push("stage_c_settle");
          ctx.stage = "stage_c_lever";
          ctx.stageTicks = 0;
          ctx.leverStartS = this.sim.data.time;
        } else {
          const liftH0 = ctx.liftLo + 0.23 * (ctx.liftHi - ctx.liftLo);
          this.sim._rescueCtrl = { left: 0, right: 0, lift: liftH0, patientLift: ctx.patLiftHold };
          ctx.stageTicks += 1;
          return { running: true };
        }
      } else if (ctx.stage === "stage_c_lever") {
        const res = this._stepStageCLever(ctx);
        if (res && res.done) return { running: false, ...res };
        return { running: true };
      }
    }
    throw new Error(`rescue state machine exceeded ${MAX_STAGE_TRANSITIONS} transitions in one tick`);
  }

  restoreControlLevel(ctx) {
    this.sim._rescueCtrl = null;
    if (ctx && ctx.prevLevel !== undefined) this.sim.controlLevel = ctx.prevLevel;
    if (this.stage !== "ready" && this.stage !== "done") this.stage = "cancelled";
  }

  patientWheelAnglesRad() {
    const qpos = this.sim.data.qpos;
    return this._patWheelQ.map(adr => qpos[adr]);
  }

  patientHeadAngleRad() {
    return this.sim.data.qpos[this._q.patHead];
  }

  patientLiftAngleRad() {
    return this.sim.data.qpos[this._q.patLift];
  }

  snapshot() {
    const { data } = this.sim;
    const tilt = this.patientTiltDeg();
    const o = 3 * this._patChassis, m = 9 * this._patChassis;
    return {
      hasPatient: true,
      active: this.sim._rescueCtrl !== null,
      side: this.side === 1 ? "right" : "left",
      mode: this.mode,
      hasCube: this._carryWeld >= 0 && this._cubeBody >= 0,
      stage: this.stage,
      patientTiltDeg: tilt,
      patientUpright: tilt < 10,
      rescuerClear: this.lastContact.clear,
      patientPosMm: [data.xpos[o] / MM, data.xpos[o + 1] / MM, data.xpos[o + 2] / MM],
      patientYawDeg: Math.atan2(data.xmat[m + 3], data.xmat[m]) / DEG,
      cubePosMm: this._cubeBody >= 0
        ? [data.xpos[3 * this._cubeBody] / MM, data.xpos[3 * this._cubeBody + 1] / MM, data.xpos[3 * this._cubeBody + 2] / MM]
        : null,
      contactPoint: this.contactPoint ? this.contactPoint.slice() : null,
    };
  }
}
