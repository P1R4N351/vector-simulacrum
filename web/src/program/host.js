// The robot's end of a program written in the page: calls by name, one at a time, against a VectorSim.
//
// Robot time and program time run in lockstep. The simulation advances only while the
// program is waiting on a call that takes time: an action (drive_straight, turn_in_place,
// set_head_angle, set_lift_height) or wait. Calls that set or read something take none.
// A program therefore gives the same run every time, whatever the frame rate or the
// delay between the program's worker and the page. The real robot does not wait for a
// slow program; the cost of this choice is that a program which never calls the robot
// holds robot time still (the page says so, and Stop ends it).
//
// Every reply carries the robot state as the SDK last reported it (every state_period_ms),
// so the program can read it between calls without a round trip.

import { assert, clamp, pyRound } from "../sim/util.js";

const DEG = Math.PI / 180;
const MAX_WAIT_S = 3600; // one wait: a bound on the code loop, not a model of anything
export const MAX_ADVANCE_TICKS = 12000; // one advance() call: one minute of robot time
// Calls in a row with no robot time passing before the program is told it is spinning.
export const IDLE_CALL_LIMIT = 10000;

export class RobotCallError extends Error {
  constructor(message) {
    super(message);
    this.name = "RobotCallError";
  }
}

function number(name, label, value, { positive = false, optional = false } = {}) {
  if (optional && (value === undefined || value === null)) return null;
  if (typeof value !== "number" || !Number.isFinite(value)) throw new RobotCallError(`${name}: ${label} must be a number`);
  if (positive && value <= 0) throw new RobotCallError(`${name}: ${label} must be above 0`);
  return value;
}

function stringOpt(name, label, value) {
  if (value === undefined || value === null) return null;
  if (typeof value !== "string") throw new RobotCallError(`${name}: ${label} must be a string`);
  return value;
}

// name -> { args: how many the call takes at most, start(host, args) }.
// start returns { now: value } for a call that takes no time, or { timed } for one that does.
const CALLS = {
  drive_straight: { args: 2, start: (h, [d, s]) => h._action(() => h.sim.driveStraight(
    number("drive_straight", "distance_mm", d), number("drive_straight", "speed_mmps", s, { positive: true }))) },
  turn_in_place: { args: 2, start: (h, [a, s]) => h._action(() => h.sim.turnInPlace(
    number("turn_in_place", "angle_deg", a), number("turn_in_place", "speed_dps", s, { positive: true, optional: true }))) },
  set_head_angle: { args: 3, start: (h, [a, speed, duration]) => h._action(() => h.sim.setHeadAngle(
    number("set_head_angle", "angle_deg", a),
    number("set_head_angle", "speed_radps", speed, { positive: true, optional: true }),
    number("set_head_angle", "duration_s", duration, { optional: true }) ?? 0)) },
  // As the SDK: 0.0 is the bottom of the lift's travel and 1.0 the top, linear in height.
  set_lift_height: { args: 1, start: (h, [x]) => h._action(() => h.sim.setLiftHeight(h.liftMmOf(number("set_lift_height", "height", x)))) },
  set_wheel_motors: {
    args: 4, start: (h, [l, r, la, ra]) => ({
      now: h.sim.setWheelMotors(number("set_wheel_motors", "left_mmps", l), number("set_wheel_motors", "right_mmps", r),
        number("set_wheel_motors", "left_accel", la, { optional: true }) ?? 0,
        number("set_wheel_motors", "right_accel", ra, { optional: true }) ?? 0),
    }),
  },
  stop_all_motors: { args: 0, start: (h) => ({ now: h.sim.stopAllMotors() }) },
  request_control: { args: 0, start: (h) => ({ now: h.sim.requestControl() }) },
  drive_off_charger: { args: 0, start: (h) => h._action(() => h.sim.driveOffCharger()) },
  explore: { args: 1, start: (h, [s]) => h._action(() => h.sim.explore(number("explore", "seconds", s, { positive: true, optional: true }) ?? 12)) },
  dock_with_charger: { args: 0, start: (h) => h._action(() => h.sim.dockWithCharger()) },
  nomad_capture_goal: {
    args: 1,
    start: (h, [label]) => {
      const tag = label === undefined ? "goal" : String(label);
      const id = h.sim.nomadCaptureGoal(tag);
      if (id === null) throw new RobotCallError("NoMaD policy is not loaded");
      return { now: { node_id: id, label: tag } };
    },
  },
  nomad_explore: {
    args: 1,
    start: (h, [s]) => h._action(() => h.sim.nomadExplore(number("nomad_explore", "seconds", s, { positive: true, optional: true }) ?? 10)),
  },
  nomad_navigate: {
    args: 2,
    start: (h, [maxS, gid]) => h._action(() => {
      const limit = number("nomad_navigate", "max_seconds", maxS, { positive: true, optional: true }) ?? 20;
      const parsedId = number("nomad_navigate", "goal_id", gid, { optional: true });
      if (parsedId !== null && (!Number.isInteger(parsedId) || parsedId < 0)) {
        throw new RobotCallError("nomad_navigate: goal_id must be a non-negative integer");
      }
      return h.sim.nomadNavigate(limit, parsedId);
    }),
  },
  rescue_patient: {
    args: 1,
    start: (h, [mode]) => h._action(() => {
      const m = stringOpt("rescue_patient", "mode", mode) ?? "auto";
      if (!["auto", "cube", "direct"].includes(m)) {
        throw new RobotCallError('rescue_patient: mode must be "auto", "cube", or "direct"');
      }
      if (!h.sim.hasPatient) throw new RobotCallError("rescue_patient requires ?env=rescue");
      return h.sim.rescuePatient(m);
    }),
  },
  reset_rescue: {
    args: 2,
    start: (h, [side, mode]) => {
      const s = stringOpt("reset_rescue", "side", side) ?? "right";
      const m = stringOpt("reset_rescue", "mode", mode) ?? "auto";
      if (!["right", "left"].includes(s)) throw new RobotCallError('reset_rescue: side must be "right" or "left"');
      if (!["auto", "cube", "direct"].includes(m)) {
        throw new RobotCallError('reset_rescue: mode must be "auto", "cube", or "direct"');
      }
      if (!h.sim.hasPatient) throw new RobotCallError("reset_rescue requires ?env=rescue");
      return { now: h.sim.resetRescue(s, m) };
    },
  },
  state: { args: 0, start: () => ({ now: null }) },
  wait: {
    args: 1, start: (h, [s]) => {
      const seconds = number("wait", "seconds", s);
      if (seconds < 0 || seconds > MAX_WAIT_S) throw new RobotCallError(`wait: seconds must be from 0 to ${MAX_WAIT_S}`);
      const ticks = pyRound(seconds / h.sim.dt);
      return ticks === 0 ? { now: 0 } : { timed: { kind: "wait", left: ticks, seconds } };
    },
  },
};
export const CALL_NAMES = Object.freeze(Object.keys(CALLS));

export class ProgramHost {
  constructor(sim) {
    assert(sim && typeof sim.stepAction === "function", "ProgramHost needs a VectorSim");
    this.sim = sim;
    this._pending = null; // { id, reply, kind, left } while a timed call runs
    this._idleCalls = 0;
    this.finished = false;
  }

  // The name of the call the program is waiting on, or null while it computes.
  get waitingOn() {
    return this._pending === null ? null : this._pending.name;
  }

  liftMmOf(fraction) {
    const v = this.sim.v;
    const f = clamp(fraction, 0, 1);
    return v.lift_height_min_mm + f * (v.lift_height_max_mm - v.lift_height_min_mm);
  }

  // The state the program may read: what the robot last reported, plus the time now.
  snapshot() {
    const s = this.sim.state;
    assert(s && typeof s === "object", "the robot has not reported a state yet");
    const r = this.sim.rescueSnapshot();
    return {
      ...s, pose_angle_deg: s.pose_angle_rad / DEG, head_angle_deg: s.head_angle_rad / DEG,
      now_s: this.sim.tickCount * this.sim.dt, nomad_nodes: this.sim.nomadSnapshot().nodes.length,
      on_charger: this.sim.chargerContact, battery_volts: this.sim.batteryVolts,
      has_patient: Boolean(r),
      patient_tilt_deg: r ? r.patientTiltDeg : null,
      patient_upright: r ? r.patientUpright : false,
      patient_side: r ? r.side : null,
      rescue_stage: r ? r.stage : null,
      rescuer_clear: r ? r.rescuerClear : true,
    };
  }

  // Take one call. reply({ id, ok, value | error, state }) is called now for a call that
  // takes no time, or from advance() when a timed one finishes.
  call(message, reply) {
    assert(typeof reply === "function", "a call needs somewhere to reply");
    const id = message?.id;
    const answer = (ok, payload) => reply({ id, ok, ...payload, state: this.snapshot() });
    try {
      const value = this._start(message, reply);
      if (value !== undefined) answer(true, { value: value.now });
    } catch (err) {
      if (!(err instanceof RobotCallError)) throw err;
      answer(false, { error: err.message });
    }
  }

  _start(message, reply) {
    if (this.finished) throw new RobotCallError("the program has ended");
    const spec = Object.hasOwn(CALLS, message?.name) ? CALLS[message.name] : null;
    if (spec === null) throw new RobotCallError(`robot has no call named ${String(message?.name)}`);
    const args = Array.isArray(message.args) ? message.args : [];
    if (args.length > spec.args) throw new RobotCallError(`${message.name} takes at most ${spec.args} arguments, not ${args.length}`);
    if (this._pending !== null) {
      throw new RobotCallError(`${message.name} was called while ${this._pending.name} is still running: await each robot call before the next`);
    }
    this._idleCalls += 1;
    if (this._idleCalls > IDLE_CALL_LIMIT) {
      throw new RobotCallError(`${IDLE_CALL_LIMIT} robot calls in a row without robot time passing. The robot only moves while the program awaits an action or robot.wait(); add one to the loop`);
    }
    const started = spec.start(this, args);
    if (started.timed === undefined) return started;
    this._pending = { id: message.id, reply, name: message.name, ...started.timed };
    return undefined;
  }

  // Start an action; a refused one (no control) finishes at once with the robot's result.
  _action(start) {
    if (this.sim.actionRunning) throw new RobotCallError("an action started from the buttons or keys is still running");
    if (!start()) return { now: this._result() };
    return { timed: { kind: "action" } };
  }

  _result() {
    const a = this.sim.lastAction;
    assert(a !== null, "an action ended without a result");
    return { ...a, result: a.result, moved_mm: a.moved_mm, turned_deg: a.turned_deg, time_s: a.ticks * this.sim.dt };
  }

  // Run the pending call for at most maxTicks control ticks. Returns the ticks run.
  advance(maxTicks) {
    assert(Number.isInteger(maxTicks) && maxTicks >= 0 && maxTicks <= MAX_ADVANCE_TICKS, "tick count out of range");
    let ran = 0;
    // An action's last step can end it without a tick, so allow one step more than the ticks.
    for (let step = 0; step <= maxTicks && this._pending !== null && ran < maxTicks; step++) {
      const before = this.sim.tickCount;
      this._stepPending();
      ran += this.sim.tickCount - before;
    }
    if (ran > 0) this._idleCalls = 0;
    return ran;
  }

  _stepPending() {
    const p = this._pending;
    if (p.kind === "wait") {
      this.sim.tick(1);
      p.left -= 1;
      if (p.left === 0) this._finish(true, { value: p.seconds });
      return;
    }
    if (!this.sim.stepAction()) this._finish(true, { value: this._result() });
  }

  _finish(ok, payload) {
    const p = this._pending;
    this._pending = null;
    p.reply({ id: p.id, ok, ...payload, state: this.snapshot() });
  }

  // End the run: a call still waiting gets an error, a running action is cancelled, and the
  // treads stop, as they do when an SDK program lets go of the robot. Safe to call twice.
  finish(reason = "the program was stopped") {
    if (this._pending !== null) {
      if (this._pending.kind === "action") this.sim.cancelAction();
      this._finish(false, { error: reason });
    }
    // Only the treads: a head or lift move still settling on its target is left to finish.
    if (!this.finished) this.sim.setWheelMotors(0, 0);
    this.finished = true;
  }
}
