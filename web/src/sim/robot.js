// SDK-shaped robot layer over the MuJoCo model, in the browser..
// A port of ../simulacrum/robot.py. test/robot.test.js
// runs this file against recorded runs of the Python (same MJCF, same commands).
//
// Units are the robot's: wheel speeds in mm/s, distances in mm, angles in
// degrees, lift height in mm. Two control levels:
//   CONTROL_DEFAULT   a cliff stops the treads, takes control away and backs off
//   CONTROL_OVERRIDE  the reaction is off; the robot keeps driving and falls
//
// One difference from the Python: there, drive_straight and the other actions
// block until done. Here an action is started and then advanced one control
// tick per tick() call, so a frame loop can run it. The tick order is the same.

import { AutonomyController } from "./autonomy.js";
import { NomadAutonomy } from "./nomad.js";
import { CubeFirmwareVm } from "./cube_firmware.js";
import { TreadController, TreadEncoder, wholeMmps } from "./drive.js";
import { CubeSensor } from "./cube_sensor.js";
import { Hazards } from "./hazards.js";
import { CHANNELS, ImuFilter } from "./imu_filter.js";
import { ProxSensor } from "./prox.js";
import { RescueController } from "./rescue.js";
import { WireMotorEncoders } from "./wire_body.js";
import { SysconFirmware } from "./syscon_firmware.js";
import { assert, clamp, makeGauss, profileStep, pyRound, toward } from "./util.js";

export const CONTROL_DEFAULT = "default";
export const CONTROL_OVERRIDE = "override";
export const RESULT_COMPLETED = "completed";
export const RESULT_CLIFF = "cancelled_cliff";
export const RESULT_NO_CONTROL = "no_control";
export const RESULT_TIMEOUT = "timeout";
export const RESULT_OFF_GROUND = "aborted_off_ground";
export const RESULT_CANCELLED = "cancelled";
export const CLIFF_NAMES = ["front_left", "front_right", "back_left", "back_right"];
const MAX_ACTION_S = 60.0; // code-loop bound for one action, not a model of the robot
const MM = 0.001;
const DEG = Math.PI / 180;

// Shoulder angle (rad, up positive) for a lift height, as the firmware converts it.
export function liftAngleOfHeight(v, heightMm) {
  const clipped = clamp(heightMm, v.lift_height_min_mm, v.lift_height_max_mm);
  const ratio = (clipped - v.lift_base_position_mm[2]) / v.lift_arm_length_mm;
  assert(ratio >= -1 && ratio <= 1, "lift height unreachable with this arm length");
  return Math.asin(ratio);
}

export function liftHeightOfAngle(v, angleRad) {
  assert(Number.isFinite(angleRad), "lift angle must be finite");
  return Math.sin(angleRad) * v.lift_arm_length_mm + v.lift_base_position_mm[2];
}

// How far the lift's integral term moves the servo's target (rad), as simulacrum/mjcf.py:
// the firmware's power is Kp x error + Ki x error sum (LIFTC:832-835) and the servo stands for Kp x error.
export function liftIntegralOffsetRad(v, errorSum) {
  const [kp, ki] = v.lift_power_gains;
  assert(kp > 0 && ki >= 0 && Number.isFinite(errorSum), "bad lift gains or error sum");
  return (ki / kp) * errorSum;
}

// One control tick of the firmware lift's integral term (LIFTC:858-892), as simulacrum/robot.py:
// holding its final target within tolerance the lift only bleeds the sum while its power is above
// lift_power_in_position_max; off target, or while the target still moves, the error goes on the sum.
export function liftErrorSumStep(v, errorSum, errorRad, final) {
  assert(Number.isFinite(errorSum) && Number.isFinite(errorRad), "bad lift error");
  const [kp, ki] = v.lift_power_gains;
  const cap = v.lift_error_sum_max;
  assert(cap > 0 && v.lift_error_sum_decay > 0, "the error sum needs a positive cap and decay");
  let sum = errorSum;
  if (Math.abs(errorRad) < v.lift_angle_tol_deg * DEG && final) {
    const power = kp * errorRad + ki * sum;
    if (Math.abs(power) > v.lift_power_in_position_max) sum -= v.lift_error_sum_decay * (power > 0 ? 1 : -1);
  } else {
    sum += errorRad;
  }
  return clamp(sum, -cap, cap);
}

function polygonMargin(x, y, outline) {
  let inside = false;
  let distance = Infinity;
  for (let i = 0; i < outline.length; i++) {
    const [ax, ay] = outline[i];
    const [bx, by] = outline[(i + 1) % outline.length];
    const dx = bx - ax, dy = by - ay;
    const t = clamp(((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy), 0, 1);
    distance = Math.min(distance, Math.hypot(x - ax - t * dx, y - ay - t * dy));
    if ((ay > y) !== (by > y) && x < ax + (y - ay) * dx / dy) inside = !inside;
  }
  return inside ? distance : -distance;
}

export class VectorSim {
  // opts: { xml, values, derived, controlLevel, startXyMm, headingDeg, headDeg, liftMm, noiseSeed }
  constructor(mujoco, opts) {
    assert(mujoco && opts && typeof opts.xml === "string", "VectorSim needs the mujoco module and the MJCF");
    this.mujoco = mujoco;
    this.shelfOutline = opts.shelfOutline;
    this.cubeStartXY = opts.cubeStartXY;
    this.wakeHeadTrack = opts.wakeHeadTrack ?? [];
    this.wakeLiftFrame = opts.wakeLiftFrame ?? null;
    this.cameraSpec = opts.camera ?? null;
    this._cubeFirmwareBytes = null;
    this.sysconFirmware = null;
    this._firmwareMeta = null;
    this.v = opts.values;
    this.d = opts.derived;
    this.dt = this.v.control_period_ms * 0.001;
    this.model = mujoco.MjModel.from_xml_string(opts.xml);
    this.data = new mujoco.MjData(this.model);
    this._indexModel();
    this._rescue = this._patientBody >= 0 ? new RescueController(this) : null;
    this._rescueCtrl = null;
    this._autonomy = new AutonomyController(this, this.cameraSpec);
    this._nomad = new NomadAutonomy(this, this.cameraSpec);
    this.reset(opts);
  }

  _initSingleReset(opts) {
    if (this.cubeStartXY) {
      const joint = this.mujoco.mj_name2id(this.model, this.mujoco.mjtObj.mjOBJ_JOINT.value, "cube_root");
      assert(joint >= 0, "mapped cube placement requires a free cube");
      this.data.qpos.set(this.cubeStartXY.map(mm => mm * MM), this.model.jnt_qposadr[joint]);
    }
    this.startDocked = opts.startDocked ?? this.startDocked ?? false;
    this._place(opts.startXyMm ?? [0, 0], opts.headingDeg ?? 0);
    this._poseServos(opts.headDeg ?? 0, opts.liftMm ?? null);
    this._settle();
    if (this._dockBody >= 0 && (this.startDocked || !("chargerContact" in opts))) {
      this.chargerContact = this._padsTouchDock();
    }
  }

  _initRescueReset(opts) {
    this._rescueSide = opts.rescueSide ?? this._rescueSide ?? 1;
    this._rescueMode = opts.rescueMode ?? this._rescueMode ?? "auto";
    this._rescue.resetScene({ side: this._rescueSide, mode: this._rescueMode });
    this._rescueSide = this._rescue.side;
    this._rescueMode = this._rescue.mode;
    this.startDocked = false;
    assert(this.data.time === 0, "rescue reset time must be 0");
    this._resetImuFilter();
    this._yawPrev = this._yawTotal = this._yaw();
    this._dockTravelMm = [0, 0];
    this._odomYawOffset = 0;
    this._odomOffsetMm = [0, 0];
    const lift0 = this.data.qpos[this._q.lift];
    this._acceptedHeadTarget = this._headTarget = this._headCmd = 0;
    this._liftTarget = this._liftCmd = this._liftServoCmd = lift0;
    this._liftErrorSum = 0;
    this._servoRate = [0, 0];
    for (const slot of this._delay) {
      slot[2] = 0;
      slot[3] = lift0;
      slot[4] = null;
    }
    const travel = this._treadTravelMm();
    for (let i = 0; i < 2; i++) {
      this._encoders[i].update(travel[i]);
      this._encoders[i].counts = 0;
      this._encoders[i].speedMmps = 0;
    }
    this._publishState();
  }

  // Put the robot back at a start pose with every controller and flag cleared.
  reset(opts = {}) {
    const level = opts.controlLevel ?? this.controlLevel ?? CONTROL_DEFAULT;
    assert(level === CONTROL_DEFAULT || level === CONTROL_OVERRIDE, "unknown control level");
    this.controlLevel = level;
    this.calmMode = opts.calmMode ?? this.calmMode ?? false;
    this._wake = null;
    this._bodyLoopUs = Math.round(1000 * (this.v.supervisor_loop_ms ?? this.v.control_period_ms));
    assert(this._bodyLoopUs > 0, "supervisor loop must advance");
    this._nextBodyLoopUs = this._bodyLoopUs;
    this._bodyStateTicks = 0;
    this._bodyStateStampMs = 0;
    this._bodyImu = null;
    this._gauss = makeGauss(opts.noiseSeed ?? 0);
    // The time-of-flight sensor draws from its own stream, so its readings do not move the IMU's noise.
    this._proxSensor = new ProxSensor(this.v, (opts.noiseSeed ?? 0) + 1);
    this.bodyRangeClock = Boolean(this.sysconFirmware) || (opts.bodyRangeClock ?? this.bodyRangeClock ?? (this._dockBody >= 0 || Boolean(this.shelfOutline)));
    this.chargerContact = opts.chargerContact ?? this.chargerContact ?? false;
    this._rangeBodyTicks = 0;
    this._rangeBodyPhaseUs = 0;
    this._rangeSample = null;
    this._rangeSampleCount = 0;
    this._tofHitsMm = [];
    this._rescueCtrl = null;
    this.mujoco.mj_resetData(this.model, this.data);
    this._spineEncoders = null;
    this._initCommands();
    this.hazards = new Hazards(this.v);
    if (this._rescue) this._initRescueReset(opts);
    else this._initSingleReset(opts);
    if (this.chargerContact && this.calmMode && this.wakeHeadTrack.length) {
      this.hasControl = this.state.has_control = false;
    }
    if (this.chargerContact) this.batteryVolts = 4.20;
    this.cubeSensor = this._cubeAdr < 0 ? null : new CubeSensor(
      this.v, this.model.opt.timestep, opts.noiseSeed ?? 0, this._cubeForce(),
    );
    if (this.cubeSensor) {
      this.cubeSensor.handlingCube = this._cubeMountedOnFork;
      if (this._cubeFirmwareBytes) {
        this.cubeSensor.attachFirmware(
          new CubeFirmwareVm(this._cubeFirmwareBytes, { railCounts: this.shelfOutline ? 444 : 426 }),
        );
      }
    }
    this._autonomy?.navMap.reset();
    this._nomad?.reset();
  }

  dispose() {
    this.data.delete();
    this.model.delete();
  }

  // ---- set-up -----------------------------------------------------------

  _indexModel() {
    const { mujoco, model } = this;
    const id = (kind, name) => {
      const found = mujoco.mj_name2id(model, mujoco.mjtObj[kind].value, name);
      assert(found >= 0, `${name} missing from the model`);
      return found;
    };
    const sensor = (name) => model.sensor_adr[id("mjOBJ_SENSOR", name)];
    const qadr = (name) => model.jnt_qposadr[id("mjOBJ_JOINT", `${name}_joint`)];
    const dof = (name) => model.jnt_dofadr[id("mjOBJ_JOINT", `${name}_joint`)];
    this._cliffAdr = CLIFF_NAMES.map((name) => sensor(`cliff_${name}`));
    this._dockBody = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, "dock");
    this._patientBody = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, "patient_chassis");
    this._chassisBody = id("mjOBJ_BODY", "chassis");
    if (this._dockBody >= 0) this._dockPins = ["dock_pin_l", "dock_pin_r"].map(name => id("mjOBJ_SITE", name));
    this._tofSite = id("mjOBJ_SITE", "tof");
    this._sceneOnly = [1, 0, 0, 0, 0, 0]; // geom group 0 is the scene; the robot's own geoms are in group 1
    this._cubeBody = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, "cube");
    const cubeId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR.value, "cube_accel");
    this._cubeAdr = cubeId < 0 ? -1 : model.sensor_adr[cubeId];
    this._cubeMountedOnFork = cubeId >= 0 &&
      model.body_parentid[id("mjOBJ_BODY", "cube")] === id("mjOBJ_BODY", "lift_fork");
    this._gyroAdr = sensor("gyro");
    this._accelAdr = sensor("accel");
    this._imuAdr = [0, 1, 2].map((i) => this._accelAdr + i).concat([0, 1, 2].map((i) => this._gyroAdr + i));
    const lower = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT.value, "lift_lower_joint");
    this._lowerQ = lower >= 0 ? model.jnt_qposadr[lower] : -1;
    this._q = { head: qadr("head"), lift: qadr("lift"), wrist: qadr("wrist") };
    this._dof = { wheelFl: dof("wheel_fl"), wheelFr: dof("wheel_fr") };
    this._wheelQ = [qadr("wheel_fl"), qadr("wheel_fr")];
    // All four wheels, front left, front right, rear left, rear right: for display and tests.
    this._allWheelQ = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"].map(qadr);
    this._allWheelDof = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"].map(dof);
    this._act = {
      left: id("mjOBJ_ACTUATOR", "left_wheel"), right: id("mjOBJ_ACTUATOR", "right_wheel"),
      head: id("mjOBJ_ACTUATOR", "head"), lift: id("mjOBJ_ACTUATOR", "lift"),
    };
    assert(model.nu === (this._patientBody >= 0 ? 8 : 4) && this._cliffAdr.length === 4, "model shape differs from the layer's");
  }

  _initCommands() {
    const v = this.v;
    this.tick_ = 0;
    this.hasControl = true;
    this.reactions = 0;
    this._cancelled = false;
    this._inAction = false;
    this._action = null;
    this.lastAction = null;
    this._wheelTarget = [0, 0];
    this._wheelAccel = [0, 0];
    this._wheelCmd = [0, 0];
    this._applied = [0, 0];
    this._encoders = [new TreadEncoder(v), new TreadEncoder(v)];
    this._controllers = [new TreadController(v), new TreadController(v)];
    assert(v.motor_reference_voltage_v > 0 && v.battery_rest_voltage_v > 0, "voltages must be positive");
    assert(v.battery_sag_v_per_power >= 0, "sag must not be negative");
    this.batteryVolts = v.battery_rest_voltage_v;
    assert(v.tread_heat_tau_s > 0 && v.tread_heat_loss_rate_per_s.length === 2, "heat needs a time and two rates");
    this.treadHeat = [0, 0];
    this._measured = [0, 0];
    this._imuSample = null;
    this._imuFilter = new ImuFilter(
      v.accel_filter_cutoff_hz, v.gyro_filter_cutoff_hz, v.imu_filter_second_pole, this.model.opt.timestep,
    );
    this._headTarget = this._headCmd = 0;
    this._liftTarget = this._liftCmd = this.d.lift_angle_min;
    this._pendingHeadCommand = null;
    this._headInPos = true;
    this._headInToleranceSince = null;
    this._liftInPos = true;
    this._liftInToleranceSince = null;
    this._liftServoCmd = this._liftCmd; // the lift target the motor acts on this tick, before the integral term
    this._liftErrorSum = 0; // the firmware's angleErrorSum_ (rad x control ticks)
    this._headSpeed = v.head_speed_default_radps;
    this._liftSpeed = v.lift_speed_default_radps;
    this._headAccel = v.head_accel_default_radps2;
    this._liftAccel = v.lift_accel_default_radps2;
    this._servoRate = [0, 0]; // head, lift: the speed each commanded value is moving at
    this._wheelDelay = pyRound(v.command_delay_ms / v.control_period_ms);
    this._servoDelay = pyRound(v.servo_command_delay_ms / v.control_period_ms);
    assert(this._wheelDelay >= 0 && this._servoDelay >= 0, "command delays must not be negative");
    const slots = Math.max(this._wheelDelay, this._servoDelay) + 1;
    this._delay = Array.from({ length: slots }, () => [0, 0, this._headCmd, this._liftCmd, null]);
    this._delayAt = 0;
    this._reactPhase = "idle";
    this._reactTicks = 0;
    this._reactDir = -1;
    this._reactFrom = [0, 0];
    this._stateTicks = pyRound(v.state_period_ms / v.control_period_ms);
    assert(this._stateTicks >= 1, "state period shorter than the control period");
  }

  _place(startXyMm, headingDeg) {
    assert(startXyMm.length === 2 && Number.isFinite(headingDeg), "bad start pose");
    let yaw = headingDeg * DEG, pitch = 0, z = 0;
    if (this.startDocked) {
      assert(this._dockBody >= 0, "docked start requires a dock in the scene");
      this.mujoco.mj_forward(this.model, this.data);
      const dock = this._dockBody, orientation = this.data.xmat.subarray(9 * dock);
      const origin = this.data.xpos.subarray(3 * dock);
      startXyMm = [origin[0] / MM + 30 * orientation[0], origin[1] / MM + 30 * orientation[3]];
      yaw = Math.atan2(orientation[3], orientation[0]) - Math.PI;
      pitch = 8.2 * DEG;
      z = origin[2] + 6.3 * MM;
    }
    const qpos = this.data.qpos;
    qpos[0] = startXyMm[0] * MM;
    qpos[1] = startXyMm[1] * MM;
    qpos[2] = z;
    qpos[3] = Math.cos(yaw / 2) * Math.cos(pitch / 2);
    qpos[4] = -Math.sin(yaw / 2) * Math.sin(pitch / 2);
    qpos[5] = Math.cos(yaw / 2) * Math.sin(pitch / 2);
    qpos[6] = Math.sin(yaw / 2) * Math.cos(pitch / 2);
    qpos[this._q.lift] = this.d.lift_angle_min;
    if (this._lowerQ >= 0) qpos[this._lowerQ] = this.d.lift_angle_min;
    qpos[this._q.wrist] = -this.d.lift_angle_min;
    this.data.ctrl[this._act.lift] = this._liftServoCmd = this.d.lift_angle_min;
    this.mujoco.mj_forward(this.model, this.data);
    this._yawPrev = this._yawTotal = this._yaw();
    this._dockTravelMm = [0, 0];
    this._odomYawOffset = 0;
    this._odomOffsetMm = [0, 0];
  }

  // Start with the head and lift already at a given angle and height (clamped to the limits).
  _poseServos(headDeg, liftMm) {
    const v = this.v;
    const head = clamp(headDeg, v.head_angle_min_deg, v.head_angle_max_deg) * DEG;
    const lift = liftAngleOfHeight(v, liftMm ?? v.lift_height_min_mm);
    assert(Number.isFinite(head) && Number.isFinite(lift), "bad head or lift start");
    const qpos = this.data.qpos;
    qpos[this._q.head] = head;
    qpos[this._q.lift] = lift;
    if (this._lowerQ >= 0) qpos[this._lowerQ] = lift;
    qpos[this._q.wrist] = -lift;
    this._acceptedHeadTarget = this._headTarget = this._headCmd = head;
    this._liftTarget = this._liftCmd = lift;
    this._servoRate = [0, 0];
    for (const slot of this._delay) {
      slot[2] = head;
      slot[3] = lift;
    }
    const ctrl = this.data.ctrl;
    ctrl[this._act.head] = head;
    ctrl[this._act.lift] = lift;
    this._liftServoCmd = lift;
    this._liftErrorSum = 0;
    this.mujoco.mj_forward(this.model, this.data);
  }

  // Let the robot come to rest, then start the clock from zero.
  _settle() {
    const ticks = pyRound(this.v.settle_time_ms / this.v.control_period_ms);
    assert(ticks >= 0 && ticks <= 2000, "settle time out of range");
    const steps = ticks * this.d.substeps;
    for (let i = 0; i < steps; i++) this.mujoco.mj_step(this.model, this.data);
    this.data.time = 0;
    this._resetImuFilter();
    this._yawPrev = this._yawTotal = this._yaw();
    const travel = this._treadTravelMm();
    for (let i = 0; i < 2; i++) {
      this._encoders[i].update(travel[i]);
      this._encoders[i].counts = 0;
      this._encoders[i].speedMmps = 0;
    }
    this._publishState();
  }

  _resetImuFilter() {
    const sd = this.data.sensordata;
    this._imuFilter.reset(this._imuAdr.map((adr) => sd[adr]));
  }

  _advanceBodyClock(endUs) {
    while (this._nextBodyLoopUs <= endUs) {
      if (++this._bodyStateTicks >= (this.calmMode ? 50 : 6)) {
        this._bodyStateStampMs = Math.round(this._nextBodyLoopUs / 1000);
        this._bodyImu = this._imu();
        this._bodyStateTicks = 0;
      }
      this._nextBodyLoopUs += this._bodyLoopUs;
    }
  }

  _cubeForce() {
    const gravity = this.v.gravity_mmps2 * MM;
    return [0, 1, 2].map(axis => this.data.sensordata[this._cubeAdr + axis] / gravity);
  }

  _physicsStep() {
    const raw = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < this.d.substeps; i++) {
      this.mujoco.mj_step(this.model, this.data);
      const sd = this.data.sensordata;
      for (let ch = 0; ch < CHANNELS; ch++) raw[ch] = sd[this._imuAdr[ch]];
      this._imuFilter.update(raw);
      const endUs = Math.round(this.data.time * 1e6);
      if (this.cubeSensor) {
        // Packets at this step boundary precede a simultaneous body publication.
        this._advanceBodyClock(endUs - 1);
        this.cubeSensor.step(this.data.time, this._cubeForce(), this._bodyStateStampMs);
      }
      this._advanceBodyClock(endUs);
    }
  }

  // ---- reading the model ------------------------------------------------

  _yaw() {
    const q = this.data.qpos;
    return Math.atan2(2 * (q[3] * q[6] + q[4] * q[5]), 1 - 2 * (q[5] * q[5] + q[6] * q[6]));
  }

  _pitch() {
    const q = this.data.qpos;
    return Math.asin(clamp(2 * (q[4] * q[6] - q[3] * q[5]), -1, 1));
  }

  // True position of the robot origin, in mm, in the table frame.
  poseMm() {
    const q = this.data.qpos;
    return [q[0] / MM, q[1] / MM, q[2] / MM];
  }

  // Heading with whole turns kept, so a 360 degree turn reads 2 pi.
  headingTotalRad() {
    return this._yawTotal;
  }

  // Signed tread surface speeds from the model, mm/s.
  _wheelSpeeds() {
    const radius = this.v.wheel_front_radius_mm;
    const qvel = this.data.qvel;
    return [qvel[this._dof.wheelFl] * radius, qvel[this._dof.wheelFr] * radius];
  }

  // Signed travel of each tread surface so far, mm.
  _treadTravelMm() {
    const radius = this.v.wheel_front_radius_mm;
    const qpos = this.data.qpos;
    return [qpos[this._wheelQ[0]] * radius, qpos[this._wheelQ[1]] * radius];
  }

  // Wheel speeds as the firmware measures them: magnitude from encoder counts,
  // sign from the command (one encoder channel), kept in an integer.
  _reportedWheelSpeeds() {
    const v = this.v;
    const out = [0, 0];
    for (let i = 0; i < 2; i++) {
      const speed = this._encoders[i].speedMmps * this._encoders[i].direction;
      out[i] = v.wheel_speed_reported_whole_mmps ? wholeMmps(speed) : speed;
    }
    return out;
  }

  // Range under each cliff sensor in mm (FL, FR, BL, BR); -1 when nothing is below.
  cliffRangesMm() {
    const sd = this.data.sensordata;
    return this._cliffAdr.map((adr) => (sd[adr] >= 0 ? sd[adr] / MM : -1));
  }

  // The fan of ray directions in the sensor's own frame, for drawing.
  get tofRays() {
    return this._proxSensor.rays;
  }

  // Cast the sensor's fan of rays at the scene (not at the robot itself).
  // Per ray: the distance in mm to what it hit, or -1 for nothing.
  _castTof() {
    const { mujoco, model, data } = this;
    const p = data.site_xpos;
    const m = data.site_xmat;
    const o = 3 * this._tofSite;
    const f = 9 * this._tofSite;
    const origin = [p[o], p[o + 1], p[o + 2]];
    const hits = [];
    for (const [x, y, z] of this._proxSensor.rays) {
      const direction = [m[f] * x + m[f + 1] * y + m[f + 2] * z, m[f + 3] * x + m[f + 4] * y + m[f + 5] * z, m[f + 6] * x + m[f + 7] * y + m[f + 8] * z];
      const dist = mujoco.mj_ray(model, data, origin, direction, this._sceneOnly, true, -1, null, null);
      hits.push(dist >= 0 ? dist / MM : -1);
    }
    this._tofHitsMm = hits;
    return hits;
  }

  // What the time-of-flight sensor reports now (prox.js), from the lift height the robot reports.
  _prox() {
    const lift = this._reportedHeadAndLift()[1];
    if (!this.bodyRangeClock) return this._proxSensor.read(this._castTof(), lift, this._pitch());
    this.rawRangeSample();
    return this._proxSensor.report(this._rangeSample, lift, this._pitch());
  }

  rawRangeSample() {
    if (this._rangeSample === null) this._acquireRange();
    return { ...this._rangeSample, count: this._rangeSampleCount };
  }

  _acquireRange() {
    this._rangeSample = this._proxSensor.acquire(this._castTof(), this._reportedHeadAndLift()[1]);
    this._rangeSampleCount = (this._rangeSampleCount + 1) & 0xffff;
  }

  _advanceRangeClock(power) {
    // Syscon timer.h and opto.cpp: 5.12 ms per body tick. Preserve phase on mode changes.
    this._rangeBodyPhaseUs += Math.round(this.dt * 1e6);
    const ticks = Math.floor(this._rangeBodyPhaseUs / 5120);
    this._rangeBodyPhaseUs %= 5120;
    const driven = power ? (Math.abs(power[0]) > 0.1 || Math.abs(power[1]) > 0.1)
      : this._controllers.some(controller => Math.abs(controller.power) > 0.1);
    const target = driven ? 6 : (this.chargerContact ? 200 : 12);
    for (let i = 0; i < ticks; i++) {
      if (++this._rangeBodyTicks >= target) {
        this._rangeBodyTicks = 0;
        this._acquireRange();
      }
    }
  }

  // Read filtered gyro and accelerometer once, apply scale, bias and noise, clip to range.
  _sampleImu() {
    const v = this.v;
    const out = this._imuFilter.output;
    const mg = 0.001 * v.gravity_mmps2;
    // Stock HAL converts the signed chip word using MAX_16BIT_POSITIVE.
    const accelStep = v.accel_range_g * 9810 / 32767;
    const gMax = v.gyro_range_dps * DEG;
    const accel = [0, 0, 0];
    const gyro = [0, 0, 0];
    for (let i = 0; i < 3; i++) {
      const value = (out[i] / MM) * v.accel_scale[i] + mg * v.accel_bias_mg[i]
        + this._gauss(0, mg * v.accel_noise_rms_mg);
      accel[i] = clamp(Math.round(value / accelStep), -32768, 32767) * accelStep;
    }
    for (let i = 0; i < 3; i++) {
      // True rates are already corrected. The stock 0.989 Z correction scales each count.
      const step = gMax * (i === 2 ? 0.989 : 1) / 32768;
      const rate = out[3 + i] + this._gauss(0, v.gyro_noise_rms_dps * DEG);
      const count = clamp(Math.round(rate / step), -32768, 32767);
      gyro[i] = clamp(count * step + v.gyro_bias_dps[i] * DEG, -gMax, gMax);
    }
    this._imuSample = [accel, gyro];
  }

  _imu() {
    if (this._imuSample === null) this._sampleImu();
    return this._imuSample;
  }

  // Joint angle of each wheel (front left, front right, rear left, rear right), rad.
  // The motors act on the front two; the rear two are turned by the track coupling.
  wheelAnglesRad() {
    const qpos = this.data.qpos;
    return this._allWheelQ.map((adr) => qpos[adr]);
  }

  // Spin rate of each wheel, same order, rad/s.
  wheelSpinRadps() {
    const qvel = this.data.qvel;
    return this._allWheelDof.map((adr) => qvel[adr]);
  }

  // Where the head and lift commands have got to on their way to the targets.
  get headCommandRad() {
    return this._headCmd;
  }

  get liftCommandMm() {
    return liftHeightOfAngle(this.v, this._liftCmd);
  }

  // Where the head and lift have been told to go (not where they are).
  get headTargetRad() {
    return this._headTarget;
  }

  get liftTargetMm() {
    return liftHeightOfAngle(this.v, this._liftTarget);
  }

  // The lift's shoulder angle, rad, up positive: the physics joint itself.
  liftAngleRad() {
    return this.data.qpos[this._q.lift];
  }

  liftHeightMm() {
    return liftHeightOfAngle(this.v, this.data.qpos[this._q.lift]);
  }

  headAngleRad() {
    return this.data.qpos[this._q.head];
  }

  // Head angle (rad) and lift height (mm) in whole encoder counts.
  _reportedHeadAndLift() {
    const v = this.v;
    const headStep = v.head_rad_per_encoder_count;
    const liftStep = v.lift_rad_per_encoder_count;
    assert(headStep > 0 && liftStep > 0, "encoder steps must be positive");
    const head = pyRound(this.headAngleRad() / headStep) * headStep;
    const low = this.d.lift_angle_min;
    const counts = pyRound((this.data.qpos[this._q.lift] - low) / liftStep);
    const liftAngle = low + counts * liftStep;
    return [head, liftHeightOfAngle(v, liftAngle)];
  }

  // What the robot would report this control tick: the signals a recording of it carries.
  reportedNow() {
    const [left, right] = this._reportedWheelSpeeds();
    const [head, lift] = this._reportedHeadAndLift();
    const gyro = this._imu()[1];
    return { wl: left, wr: right, gyro_z_dps: gyro[2] / DEG, head_deg: head / DEG, lift_mm: lift };
  }

  // Engine localization changes the reported frame while MuJoCo continues in shelf coordinates.
  _reportedPlanarPose() {
    const [x, y] = this.poseMm();
    const dx = x - this._dockTravelMm[0], dy = y - this._dockTravelMm[1];
    const c = Math.cos(this._odomYawOffset), s = Math.sin(this._odomYawOffset);
    const yaw = this._yaw() + this._odomYawOffset;
    return [c * dx - s * dy + this._odomOffsetMm[0],
      s * dx + c * dy + this._odomOffsetMm[1], Math.abs(yaw) <= Math.PI ? yaw : Math.atan2(Math.sin(yaw), Math.cos(yaw))];
  }

  _localizeOnCharger(charger) {
    // Charger::GetRobotDockedPose puts the origin 30 mm inward, facing out.
    this._odomYawOffset = Math.atan2(charger.outY, charger.outX) - this._yaw();
    const [x, y] = this._reportedPlanarPose();
    this._odomOffsetMm[0] += charger.lipX - 30 * charger.outX - x;
    this._odomOffsetMm[1] += charger.lipY - 30 * charger.outY - y;
  }

  _publishState() {
    const [x, y, yaw] = this._reportedPlanarPose();
    const z = this.data.qpos[2] / MM;
    const [left, right] = this._reportedWheelSpeeds();
    const prox = this._prox();
    // In calm mode successive SDK reports repeat the last supervisor IMU sample.
    this._bodyImu ??= this._imu();
    const [accel, gyro] = this.calmMode ? this._bodyImu : this._imu();
    const [head, lift] = this._reportedHeadAndLift();
    this.state = {
      time_s: this.tick_ * this.dt, x_mm: x, y_mm: y, z_mm: z, pose_angle_rad: yaw,
      pose_pitch_rad: this._pitch(), left_wheel_speed_mmps: left, right_wheel_speed_mmps: right,
      head_angle_rad: head, lift_height_mm: lift, prox_distance_mm: prox.distance_mm,
      prox_found_object: prox.found_object, prox_is_lift_in_fov: prox.is_lift_in_fov,
      prox_signal_quality: prox.signal_quality, prox_unobstructed: prox.unobstructed,
      accel_mmps2: accel.slice(), gyro_radps: gyro.slice(), cliff_detected: this.hazards.cliffAny,
      picked_up: this.hazards.pickedUp, falling: this.hazards.falling,
      wheels_moving: Math.abs(left) + Math.abs(right) > this.v.wheel_stopped_threshold_mmps,
      has_control: this.hasControl,
      head_in_pos: this._headInPos,
      lift_in_pos: this._liftInPos,
    };
    if (this.sysconFirmware) {
      if (!this._spineEncoders) this._initSpineEncoders();
      const channels = this._spineEncoders.channels;
      this.lastSpineFrame = this.sysconFirmware.packBodyFrame({
        frameCounter: this.tick_,
        motorPos: channels.map(e => e.position),
        motorDelta: channels.map(e => e.delta),
        motorPeriod: channels.map(e => e.period),
        cliff: this.cliffRangesMm().map((mm) => (mm < 0 ? 4 : Math.max(4, Math.round(900 - 4.48 * mm)))),
        batteryRaw: Math.trunc(this.batteryVolts * 2048 / 2.8),
        chargerRaw: this.chargerContact ? Math.trunc(5 * 2048 / 2.8) : 0,
        chargerFlags: this.chargerContact ? 3 : 0,
        proxSample: this.rawRangeSample(),
      });
    }
  }

  // Facts the real robot does not report: for the display and for the tests.
  simTruth() {
    const v = this.v;
    const [x, y, z] = this.poseMm();
    const margin = this.shelfOutline ? polygonMargin(x, y, this.shelfOutline) : Math.min(0.5 * v.table_length_mm - Math.abs(x), 0.5 * v.table_width_mm - Math.abs(y));
    return {
      fallen: z < -v.table_thickness_mm, edge_margin_mm: margin, z_mm: z,
      cliff_flags: this.hazards.cliff.slice(), wheel_speeds_mmps: this._wheelSpeeds(),
      encoder_counts: [this._encoders[0].counts, this._encoders[1].counts],
      motor_power: [this._controllers[0].power, this._controllers[1].power],
      wheel_command_mmps: this._applied.slice(), reacting: this._reactPhase !== "idle",
      cliff_ranges_mm: this.cliffRangesMm(), tof_hits_mm: this._tofHitsMm.slice(),
      wheel_angles_rad: this.wheelAnglesRad(), wheel_spin_radps: this.wheelSpinRadps(),
      battery_volts: this.batteryVolts, tread_heat: this.treadHeat.slice(),
      on_charger: Boolean(this.chargerContact), on_charger_platform: this._onDockPlatform(),
      autonomy_phase: this._rescue && this._action?.autonomy?.mode === "rescue" ? `RESCUE · ${this._rescue.stage}`
        : (this._nomad?.phase ?? this._autonomy?.phase ?? null),
      nomad_ready: Boolean(this._nomad?.policy),
      firmware_active: Boolean(this.sysconFirmware || this.cubeSensor?.firmware),
      has_patient: Boolean(this._rescue),
      patient_tilt_deg: this._rescue ? this._rescue.patientTiltDeg() : null,
      patient_upright: this._rescue ? this._rescue.patientTiltDeg() < 10 : null,
      patient_side: this._rescue ? (this._rescue.side === 1 ? "right" : "left") : null,
      rescue_stage: this._rescue ? this._rescue.stage : null,
      rescuer_clear: this._rescue ? this._rescue.isRescuerClear() : null,
      patient_wheel_angles_rad: this._rescue ? this._rescue.patientWheelAnglesRad() : null,
    };
  }

  // ---- the control tick -------------------------------------------------

  // Move the commanded values toward their targets at the commanded rates.
  _rampCommands() {
    for (let i = 0; i < 2; i++) {
      const accel = this._wheelAccel[i];
      this._wheelCmd[i] = accel <= 0
        ? this._wheelTarget[i]
        : toward(this._wheelCmd[i], this._wheelTarget[i], accel * this.dt);
    }
    const headCap = Math.min(this._headSpeed, this.v.head_speed_max_radps);
    const liftCap = Math.min(this._liftSpeed, this.v.lift_speed_max_radps);
    [this._headCmd, this._servoRate[0]] = profileStep(this._headCmd, this._headTarget, this._servoRate[0], headCap, this._headAccel, this.dt);
    [this._liftCmd, this._servoRate[1]] = profileStep(this._liftCmd, this._liftTarget, this._servoRate[1], liftCap, this._liftAccel, this.dt);
  }

  // Queue this tick's command; return the wheel speeds issued command_delay_ms ago
  // and the head and lift targets issued servo_command_delay_ms ago.
  _delayedCommand() {
    const size = this._delay.length;
    const slot = this._delay[this._delayAt];
    slot[0] = this._wheelCmd[0];
    slot[1] = this._wheelCmd[1];
    slot[2] = this._headCmd;
    slot[3] = this._liftCmd;
    slot[4] = this._pendingHeadCommand;
    this._pendingHeadCommand = null;
    const wheels = this._delay[(this._delayAt - this._wheelDelay + size) % size];
    const servos = this._delay[(this._delayAt - this._servoDelay + size) % size];
    if (servos[4] !== null) this._acceptHeadTarget(servos[4]);
    this._delayAt = (this._delayAt + 1) % size;
    return [wheels[0], wheels[1], servos[2], servos[3]];
  }

  // A local supervisor stop supersedes tread commands still in the transport queue.
  _stopTreadsAtContact() {
    this._wheelTarget = [0, 0];
    this._wheelAccel = [0, 0];
    this._wheelCmd = [0, 0];
    for (const slot of this._delay) {
      slot[0] = 0;
      slot[1] = 0;
    }
  }

  // The robot's own response to a cliff: stop now, lose SDK control, back off after a pause.
  _startReaction() {
    const v = this.v;
    this.reactions += 1;
    this.hasControl = false;
    this._cancelled = true;
    const front = this.hazards.cliff[0] || this.hazards.cliff[1];
    this._reactDir = front ? -1 : 1;
    this._reactPhase = "pause";
    this._reactTicks = pyRound(v.cliff_reaction_pause_ms / v.control_period_ms);
    this._wheelTarget = [0, 0];
    this._wheelCmd = [0, 0];
    this._headCmd = this.headAngleRad();
    this._headTarget = this._headCmd;
    this._acceptHeadTarget(this._headCmd);
    this._pendingHeadCommand = null;
    this._liftTarget = this._liftCmd = this.data.qpos[this._q.lift];
    this._servoRate = [0, 0];
    for (const slot of this._delay) {
      slot[4] = null;
      slot[0] = 0;
      slot[1] = 0;
      slot[2] = this._headCmd;
      slot[3] = this._liftCmd;
    }
    assert(this._reactTicks >= 0, "cliff reaction pause must not be negative");
  }

  // Wheel speed (both treads) the reaction commands this tick.
  _reactionWheels() {
    const v = this.v;
    if (this._reactPhase === "pause") {
      this._reactTicks -= 1;
      if (this._reactTicks <= 0) {
        this._reactPhase = "move";
        this._reactFrom = this.poseMm().slice(0, 2);
        const budgetS = (3 * v.cliff_reaction_backup_mm) / v.cliff_reaction_backup_speed_mmps;
        this._reactTicks = Math.ceil(budgetS / this.dt);
      }
      return 0;
    }
    const [x, y] = this.poseMm();
    const gone = Math.hypot(x - this._reactFrom[0], y - this._reactFrom[1]);
    this._reactTicks -= 1;
    if (gone >= v.cliff_reaction_backup_mm || this._reactTicks <= 0) {
      this._reactPhase = "idle";
      return 0;
    }
    return this._reactDir * v.cliff_reaction_backup_speed_mmps;
  }

  // Firmware rule: stop when a cliff newly appears while driving and not already stopping.
  _maybeReact() {
    if (this.controlLevel !== CONTROL_DEFAULT || this._reactPhase !== "idle") return;
    const driving = Math.abs(this._measured[0]) + Math.abs(this._measured[1]) > this.v.wheel_stopped_threshold_mmps;
    const stopping = this._applied[0] === 0 && this._applied[1] === 0;
    if (this.hazards.cliffNew && driving && !stopping) this._startReaction();
  }

  _writeControls() {
    const delayed = this._delayedCommand();
    let left = 0;
    let right = 0;
    if (this._reactPhase !== "idle") {
      left = right = this._reactionWheels();
    } else if (this.hasControl && this._inAction) {
      [left, right] = this._wheelCmd;
    } else if (this.hasControl) {
      [left, right] = delayed;
    }
    this.data.ctrl[this._act.head] = delayed[2];
    this._writeLift(delayed[3]);
    this._applied = [left, right];
    this._driveTreads(left, right);
  }

  // The firmware's speed loop: commanded and measured speed in, motor power out.
  // The power is a PWM duty, so the motor sees duty x battery voltage: the duty is scaled by
  // the battery's voltage over the voltage the actuator's gain was set at. The battery sags
  // with the treads' summed duty of the last tick. Each tread also loses drive as it heats:
  // a first-order heat state follows its |duty|, and the drive is scaled by 1 - rate x tau x heat.
  _driveTreads(left, right) {
    const v = this.v;
    const measured = this._reportedWheelSpeeds();
    const load = Math.abs(this._controllers[0].power) + Math.abs(this._controllers[1].power);
    this.batteryVolts = v.battery_rest_voltage_v - v.battery_sag_v_per_power * load;
    assert(this.batteryVolts > 0, "battery voltage sagged to nothing");
    const scale = this.batteryVolts / v.motor_reference_voltage_v;
    const duties = [this._controllers[0].update(left, measured[0]), this._controllers[1].update(right, measured[1])];
    assert(Math.max(Math.abs(duties[0]), Math.abs(duties[1])) <= v.motor_power_max, "power outside its limit");
    this._applyWheelDuty(duties, scale);
  }

  _applyWheelDuty(duties, scale) {
    const v = this.v;
    const tau = v.tread_heat_tau_s;
    const rates = v.tread_heat_loss_rate_per_s;
    const keep = [1 - rates[0] * tau * this.treadHeat[0], 1 - rates[1] * tau * this.treadHeat[1]];
    assert(Math.min(keep[0], keep[1]) > 0, "a tread lost all its drive to heat");
    const ctrl = this.data.ctrl;
    ctrl[this._act.left] = duties[0] * scale * keep[0];
    ctrl[this._act.right] = duties[1] * scale * keep[1];
    for (let i = 0; i < 2; i++) this.treadHeat[i] += (Math.abs(duties[i]) - this.treadHeat[i]) * Math.min(1, this.dt / tau);
  }

  _setHeadTarget(target) {
    this._headTarget = this._pendingHeadCommand = target;
  }

  _acceptHeadTarget(target) {
    if (target !== this._acceptedHeadTarget || !this._headInPos || Math.abs(target - this.headAngleRad()) >= this.v.head_angle_tol_deg * DEG) {
      this._headInPos = false;
      this._headInToleranceSince = null;
    }
    this._acceptedHeadTarget = target;
  }

  _updateHeadStatus() {
    if (this._headInPos) return;
    const angleError = Math.abs(this.data.qpos[this._q.head] - this._acceptedHeadTarget);
    const finalCommand = this.data.ctrl[this._act.head] === this._acceptedHeadTarget;
    if (!finalCommand || angleError >= this.v.head_angle_tol_deg * DEG) {
      this._headInToleranceSince = null;
    } else if (this._headInToleranceSince === null) {
      this._headInToleranceSince = this.tick_;
    } else if ((this.tick_ - this._headInToleranceSince) * this.dt > 0.200) {
      this._headInPos = true;
    }
  }

  // Aim the lift servo at this tick's target plus the integral term, then update the error sum.
  // The firmware computes its power from the sum before this tick adds to it (LIFTC:834, 888).
  _writeLift(command) {
    const error = command - this.data.qpos[this._q.lift];
    this.data.ctrl[this._act.lift] = command + liftIntegralOffsetRad(this.v, this._liftErrorSum);
    this._liftServoCmd = command;
    this._liftErrorSum = liftErrorSumStep(this.v, this._liftErrorSum, error, command === this._liftTarget);
  }

  // A new lift target. Arriving while the lift is in position, it clears the error sum, so a short
  // move is not overpowered by the sum unwinding (LIFTC:566-573).
  _retargetLift() {
    if (this._liftInPos) this._liftErrorSum = 0;
    this._liftInPos = false;
  }

  _updateLiftStatus() {
    if (this._liftInPos) return;
    const angleError = Math.abs(this.data.qpos[this._q.lift] - this._liftTarget);
    const finalCommand = this._liftServoCmd === this._liftTarget;
    if (!finalCommand || angleError >= this.v.lift_angle_tol_deg * DEG) {
      this._liftInToleranceSince = null;
    } else if (this._liftInToleranceSince === null) {
      this._liftInToleranceSince = this.tick_;
    } else if ((this.tick_ - this._liftInToleranceSince) * this.dt > 0.100) {
      this._liftInPos = true;
    }
  }

  // Electrical contact envelope from simulacrum/dock.py, in dock-local metres.
  _padsTouchDock() {
    const { xpos, xmat, site_xpos } = this.data;
    const chassis = this._chassisBody, dock = this._dockBody;
    const c = xmat.subarray(9 * chassis, 9 * chassis + 9);
    const d = xmat.subarray(9 * dock, 9 * dock + 9);
    if (c[2] * d[2] + c[5] * d[5] + c[8] * d[8] <= 0.85) return false;
    const project = (v, axis) => v[0] * d[axis] + v[1] * d[3 + axis] + v[2] * d[6 + axis];
    const center = [0, 1, 2].map(i => xpos[3 * chassis + i] - xpos[3 * dock + i]);
    const x = project(center, 0), z = project(center, 2);
    if (x < -0.030 || x > 0.065 || Math.abs(project(center, 1)) > 0.032 || z < -0.002 || z > 0.035) return false;
    const leftSign = c[1] * d[1] + c[4] * d[4] + c[7] * d[7] > 0 ? -1 : 1;
    return this._dockPins.every((pin, side) => {
      const y = (side === 0 ? leftSign : -leftSign) * 0.00990;
      const delta = [0, 1, 2].map(i => xpos[3 * chassis + i] - site_xpos[3 * pin + i]
        - 0.0355 * c[3 * i] + y * c[3 * i + 1] + 0.001 * c[3 * i + 2]);
      const height = project(delta, 2);
      return Math.abs(project(delta, 0)) <= 0.009 && Math.abs(project(delta, 1)) <= 0.0038 && height >= -0.005 && height <= 0.004;
    });
  }

  _onDockPlatform() {
    if (this._dockBody < 0) return false;
    const { xpos, xmat } = this.data;
    const c = xmat.subarray(9 * this._chassisBody, 9 * this._chassisBody + 9);
    const d = xmat.subarray(9 * this._dockBody, 9 * this._dockBody + 9);
    if (c[2] * d[2] + c[5] * d[5] + c[8] * d[8] <= 0.85) return false;
    const delta = [0, 1, 2].map((i) => xpos[3 * this._chassisBody + i] - xpos[3 * this._dockBody + i]);
    const dot = (axis) => delta[0] * d[axis] + delta[1] * d[3 + axis] + delta[2] * d[6 + axis];
    const x = dot(0), z = dot(2);
    return x >= -0.025 && x <= 0.075 && Math.abs(dot(1)) <= 0.036 && z >= -0.002 && z <= 0.035;
  }

  // One body frame under the external WireOS motor controller.
  tickWireBody(power) {
    this._wake = null; // the external firmware owns the motors
    assert(power.length === 4 && power.every(n => Number.isFinite(n) && Math.abs(n) <= 1), "bad motor duty");
    this._tickOnce(power);
  }

  _advanceWakeHead() {
    const wake = this._wake;
    if (!wake) return;
    if (!this.chargerContact || !this.hasControl) {
      this._wake = null;
      return;
    }
    const elapsed = (this.tick_ - wake.startTick) * this.dt;
    if (wake.waiting && this._action?.kind !== "head" && this.state.head_in_pos) wake.waiting = false;
    while (wake.next < this.wakeHeadTrack.length) {
      const frame = this.wakeHeadTrack[wake.next];
      if (elapsed < frame.triggerTime_ms / 1000) break;
      wake.next++;
      if (!wake.waiting) {
        this._startHead(frame.angle_deg, null, frame.durationTime_ms / 1000);
      }
    }
    if (wake.next === this.wakeHeadTrack.length) this._wake = null;
  }

  _tickOnce(power = null) {
    const travel = this._treadTravelMm();
    const direction = this.v.encoder_tread_direction_sensed ? this._wheelSpeeds()
      : [this.data.ctrl[this._act.left], this.data.ctrl[this._act.right]];
    this._encoders[0].update(travel[0], direction[0]);
    this._encoders[1].update(travel[1], direction[1]);
    this._measured = this._reportedWheelSpeeds();
    this._sampleImu();
    const accel = this._imu()[0];
    this.hazards.update(this.cliffRangesMm(), Math.hypot(accel[0], accel[1], accel[2]));
    if (power === null && this._rescueCtrl !== null) {
      const rc = this._rescueCtrl;
      const act = this.data.ctrl;
      act[this._act.left] = rc.left;
      act[this._act.right] = rc.right;
      act[this._act.head] = 0;
      act[this._act.lift] = rc.lift;
      act[this._rescue._act.patLeft] = 0;
      act[this._rescue._act.patRight] = 0;
      act[this._rescue._act.patHead] = 0;
      act[this._rescue._act.patLift] = rc.patientLift;
      this._applied = [rc.left * this.v.tread_noload_speed_mmps, rc.right * this.v.tread_noload_speed_mmps];
    } else if (power === null) {
      this._maybeReact();
      this._advanceWakeHead();
      this._rampCommands();
      this._writeControls();
    } else {
      const v = this.v, ctrl = this.data.ctrl;
      this.batteryVolts = v.battery_rest_voltage_v - v.battery_sag_v_per_power * (Math.abs(power[0]) + Math.abs(power[1]));
      this._applyWheelDuty([power[0], -power[1]], this.batteryVolts / v.motor_reference_voltage_v);
      // The lift's control range reaches past its travel for the model's own integral term; the
      // firmware's power carries its own, so here the target stays within the travel.
      const bounds = this.model.actuator_ctrlrange, head = this._act.head;
      for (const [index, name, rate, lo, hi] of [
        [2, "lift", 20, this.d.lift_angle_min, this.d.lift_angle_max],
        [3, "head", 2 * Math.PI, bounds[2 * head], bounds[2 * head + 1]],
      ]) {
        const at = this._act[name];
        ctrl[at] = clamp(ctrl[at] + power[index] * rate * this.dt, lo, hi);
      }
      this._liftServoCmd = ctrl[this._act.lift];
    }
    const before = (this._dockBody >= 0 || this.chargerContact) ? this.poseMm() : null;
    const startSeconds = this.data.time;
    this._physicsStep();
    if (this._spineEncoders) {
      const ctrl = this.data.ctrl;
      const duty = power ?? [ctrl[this._act.left], -ctrl[this._act.right], 0, 0];
      this._spineEncoders.sample(this, duty, startSeconds, this.dt);
    }
    if (this._rescue) this._rescue.sampleContacts();
    this.tick_ += 1;
    this._updateHeadStatus();
    this._updateLiftStatus();

    if (this._dockBody >= 0) this.chargerContact = this._padsTouchDock();
    if (this.chargerContact) this.batteryVolts = 4.20;
    // The supervisor discards planar travel on electrical contact (VIC-1046).
    if (this.chargerContact && before) {
      const after = this.poseMm();
      this._dockTravelMm[0] += after[0] - before[0];
      this._dockTravelMm[1] += after[1] - before[1];
    }
    if (this.bodyRangeClock) this._advanceRangeClock(power);
    const yaw = this._yaw();
    this._yawTotal += Math.atan2(Math.sin(yaw - this._yawPrev), Math.cos(yaw - this._yawPrev));
    this._yawPrev = yaw;
    if (this.tick_ % this._stateTicks === 0) this._publishState();
  }

  get tickCount() {
    return this.tick_;
  }

  // Advance by whole control ticks. A running action is advanced with them.
  tick(ticks = 1) {
    assert(Number.isInteger(ticks) && ticks >= 0 && ticks <= Math.trunc(MAX_ACTION_S / this.dt), "tick count out of range");
    for (let i = 0; i < ticks; i++) {
      const ticked = this._action !== null && this._advanceAction();
      if (!ticked) this._tickOnce();
    }
    return this.state;
  }

  // Advance a running action by one step of its own loop, with no idle tick
  // after it ends: the blocking call of the Python, one step at a time.
  // Returns true while the action is still running.
  stepAction() {
    assert(this._action !== null, "no action is running");
    this._advanceAction();
    return this._action !== null;
  }

  // ---- SDK-shaped commands ----------------------------------------------

  // Ask for control back. Refused while the robot is still reacting.
  requestControl() {
    if (this._reactPhase !== "idle") return false;
    if (this.calmMode && this.chargerContact && this.wakeHeadTrack.length) {
      this.calmMode = false;
      // The pinned clip has a single lift target at grant, before later SDK commands.
      if (this.wakeLiftFrame) {
        this._startLift(this.wakeLiftFrame.height_mm, null, this.wakeLiftFrame.durationTime_ms / 1000);
      }
      this._wake = { startTick: this.tick_, next: 0, waiting: false };
    }
    this.hasControl = true;
    this._cancelled = false;
    return true;
  }

  // Set tread speeds. An acceleration of 0 means the speed is commanded at once.
  // Not clamped to wheel_speed_max_mmps: the firmware applies that only in path following,
  // not in direct drive, and the real robot went faster when sent more. The motor sets the ceiling.
  setWheelMotors(leftMmps, rightMmps, leftAccel = 0, rightAccel = 0) {
    assert([leftMmps, rightMmps, leftAccel, rightAccel].every(Number.isFinite), "bad wheel command");
    if (!this.hasControl || this._action !== null) return false;
    this._wheelTarget = [leftMmps, rightMmps];
    this._wheelAccel = [Math.abs(leftAccel), Math.abs(rightAccel)];
    return true;
  }

  stopAllMotors() {
    if (!this.hasControl || this._action !== null) return false;
    this._wake = null;
    this._wheelTarget = [0, 0];
    this._wheelAccel = [0, 0];
    this._headCmd = this.headAngleRad();
    this._setHeadTarget(this._headCmd);
    this._liftTarget = this._liftCmd = this.data.qpos[this._q.lift];
    this._retargetLift(); // the firmware stops the lift by aiming it where it is (LIFTC:927-930, 458-474)
    this._liftInToleranceSince = null;
    this._servoRate = [0, 0];
    return true;
  }

  _startHead(angleDeg, speedRadps, durationS) {
    assert(Number.isFinite(durationS) && durationS >= 0, "bad head duration");
    const v = this.v;
    const target = clamp(angleDeg, v.head_angle_min_deg, v.head_angle_max_deg) * DEG;
    this._headSpeed = speedRadps ? Math.abs(speedRadps) : v.head_speed_default_radps;
    this._headAccel = v.head_accel_default_radps2;
    this._servoRate[0] = 0;
    const span = Math.abs(target - this._headCmd);
    if (durationS > 0 && span > 0) {
      // WireOS allocates a tenth of the requested duration to each velocity ramp.
      const peak = span / (0.9 * durationS);
      this._headSpeed = Math.min(v.head_speed_max_radps, peak);
      this._headAccel = Math.min(10000, peak / (0.1 * durationS));
    }
    this._setHeadTarget(target);
  }

  // Set the head target and return at once.
  commandHeadAngle(angleDeg, durationS = 0) {
    assert(Number.isFinite(angleDeg), "bad head angle");
    if (!this.hasControl) return false;
    this._wake = null; // a direct motor command takes ownership of the head
    this._startHead(angleDeg, null, durationS);
    return true;
  }

  // Set the lift target and return at once.
  _startLift(heightMm, speedRadps, durationS) {
    assert(Number.isFinite(durationS) && durationS >= 0, "bad lift duration");
    const target = liftAngleOfHeight(this.v, heightMm);
    if (target !== this._liftTarget || Math.abs(target - this.data.qpos[this._q.lift]) >= this.v.lift_angle_tol_deg * DEG) {
      this._retargetLift();
    }
    this._liftInToleranceSince = null;
    this._liftTarget = target;
    this._liftSpeed = speedRadps ? Math.abs(speedRadps) : this.v.lift_speed_default_radps;
    this._liftAccel = this.v.lift_accel_default_radps2;
    this._servoRate[1] = 0;
    const span = Math.abs(this._liftTarget - this._liftCmd);
    if (durationS > 0 && span > 0) {
      // Stock SetLiftHeight reserves 10% of the duration for each ramp.
      const peak = span / (0.9 * durationS);
      this._liftSpeed = Math.min(peak, this.v.lift_speed_max_radps);
      this._liftAccel = Math.min(10000, peak / (0.1 * durationS));
    }
  }

  commandLiftHeight(heightMm, durationS = 0) {
    assert(Number.isFinite(heightMm), "bad lift height");
    if (!this.hasControl) return false;
    this._startLift(heightMm, null, durationS);
    return true;
  }

  get actionRunning() {
    return this._action !== null;
  }

  // End a running action at once, as cancelling an SDK action does: treads to zero,
  // head and lift held where they are. The Python has no counterpart (its actions block);
  // the page's program runner uses this when a program is stopped. False when none was running.
  cancelAction() {
    const a = this._action;
    if (a === null) return false;
    if (a.autonomy) (a.autonomy.controller ?? this._autonomy).restoreControlLevel(a.autonomy);
    const [x, y] = this.poseMm();
    this.lastAction = {
      kind: a.kind, result: RESULT_CANCELLED, moved_mm: Math.hypot(x - a.start[0], y - a.start[1]),
      turned_deg: (this._yawTotal - a.yaw0) / DEG, ticks: this.tick_ - a.tick0,
    };
    this._action = null;
    this._inAction = false;
    const stopped = this.stopAllMotors();
    assert(stopped || !this.hasControl, "motors must stop once the action is gone");
    return true;
  }

  _budgetTicks(expectedS) {
    assert(expectedS >= 0, "expected time must not be negative");
    return Math.trunc(Math.min(MAX_ACTION_S, 3 * expectedS + 5) / this.dt);
  }

  // Why a running action must end now, or null.
  _interrupt() {
    if (this._action?.autonomy) {
      return (this.hazards.falling && this.poseMm()[2] < -5) ? RESULT_OFF_GROUND : null;
    }
    if (this._cancelled) return RESULT_CLIFF;
    if (this.hazards.pickedUp || this.hazards.falling) return RESULT_OFF_GROUND;
    return null;
  }

  _refuse() {
    this.lastAction = { result: RESULT_NO_CONTROL, moved_mm: 0, turned_deg: 0, ticks: 0 };
    return false;
  }

  _newAction(kind, fields) {
    assert(this._action === null, "one action at a time");
    this._wheelTarget = [0, 0];
    this._wheelAccel = [0, 0];
    this._action = {
      kind, phase: "delay", left: fields.servo ? 0 : this._wheelDelay, start: this.poseMm(),
      yaw0: this._yawTotal, tick0: this.tick_, result: RESULT_TIMEOUT, cmd: 0, finishLeft: 0, ...fields,
    };
    return true;
  }

  // Drive a distance along the current heading. Negative distance is backwards.
  driveStraight(distMm, speedMmps) {
    assert(Number.isFinite(distMm) && Number.isFinite(speedMmps) && speedMmps > 0, "bad drive request");
    if (!this.hasControl || this._action !== null) return this._refuse();
    const speed = Math.min(speedMmps, this.v.wheel_speed_max_mmps);
    return this._newAction("drive", {
      dist: Math.abs(distMm), sign: distMm >= 0 ? 1 : -1, speed, budget: this._budgetTicks(Math.abs(distMm) / speed),
    });
  }

  // Turn by an angle; positive is counter-clockwise seen from above.
  turnInPlace(angleDeg, speedDps = null) {
    assert(Number.isFinite(angleDeg), "bad turn request");
    if (!this.hasControl || this._action !== null) return this._refuse();
    const v = this.v;
    const asked = speedDps !== null ? speedDps * DEG : v.point_turn_speed_default_radps;
    const rate = clamp(Math.abs(asked), 0.01, v.body_rotation_speed_max_dps * DEG);
    const angle = Math.abs(angleDeg * DEG);
    return this._newAction("turn", { angle, sign: angleDeg >= 0 ? 1 : -1, rate, budget: this._budgetTicks(angle / rate) });
  }

  // Tilt the head and wait for it; the angle is clamped to the robot's range.
  setHeadAngle(angleDeg, speedRadps = null, durationS = 0) {
    assert(Number.isFinite(angleDeg), "bad head angle");
    if (!this.hasControl || this._action !== null) return this._refuse();
    if (this._wake) this._wake.waiting = true;
    this._startHead(angleDeg, speedRadps, durationS);
    const target = this._headTarget;
    const span = Math.abs(target - this.headAngleRad());
    return this._newServoAction("head", target, this.v.head_angle_tol_deg * DEG, span, this._headSpeed);
  }

  // Raise or lower the lift and wait for it; the height is clamped to the robot's range.
  setLiftHeight(heightMm, speedRadps = null, durationS = 0) {
    assert(Number.isFinite(heightMm), "bad lift height");
    if (!this.hasControl || this._action !== null) return this._refuse();
    const v = this.v;
    this._startLift(heightMm, speedRadps, durationS);
    const target = this._liftTarget;
    const span = Math.abs(target - this.data.qpos[this._q.lift]);
    const made = this._newServoAction("lift", target, v.lift_angle_tol_deg * DEG, span, this._liftSpeed);
    return made;
  }

  _initSpineEncoders() {
    this._spineEncoders = new WireMotorEncoders();
    this._spineEncoders.sample(this, [0, 0, 0, 0], this.data.time, 0);
  }

  attachFirmware({ cubeBytes = null, sysconBytes = null, manifest = null } = {}) {
    this._cubeFirmwareBytes = cubeBytes;
    this._firmwareMeta = manifest;
    if (cubeBytes && this.cubeSensor) {
      this.cubeSensor.attachFirmware(new CubeFirmwareVm(cubeBytes, { railCounts: this.shelfOutline ? 444 : 426 }));
    }
    if (sysconBytes) {
      this.sysconFirmware = new SysconFirmware(sysconBytes);
      this.bodyRangeClock = true;
      this._initSpineEncoders();
    }
  }

  navMapSnapshot() {
    return this._autonomy.navMap.snapshot();
  }

  attachNomad(policy, spec = null) {
    this._nomad.attachPolicy(policy, spec ?? this.cameraSpec);
  }

  nomadSnapshot() {
    return this._nomad.snapshot();
  }

  nomadCaptureGoal(label = "goal") {
    if (!this._nomad.policy) return null;
    return this._nomad.captureGoal(label);
  }

  firmwareStatus() {
    return {
      active: Boolean(this.sysconFirmware || this.cubeSensor?.firmware),
      cubeVersion: this.cubeSensor?.firmware?.version ?? null,
      sysconVersion: this.sysconFirmware?.version ?? null,
      sysconReset: this.sysconFirmware ? `0x${this.sysconFirmware.resetHandler.toString(16).padStart(8, "0")}` : null,
      framesPacked: this.sysconFirmware?.framesPacked ?? 0,
      bmaFramesRead: this.cubeSensor?.firmware?.bma?.framesRead ?? 0,
      sourceCommit: this._firmwareMeta?.source_commit ?? null,
    };
  }

  _startAutonomy(ctx) {
    const ctrl = ctx.controller ?? this._autonomy;
    if (!this.hasControl || this._action !== null) {
      ctrl.restoreControlLevel(ctx);
      return this._refuse();
    }
    const maxS = Math.max(8, (ctx.deadlineTick - this.tick_) * this.dt + 2);
    return this._newAction(ctx.mode, { autonomy: ctx, budget: Math.trunc(maxS / this.dt) + 200 });
  }

  driveOffCharger() {
    return this._startAutonomy(this._autonomy.startUndock());
  }

  explore(durationS = 15) {
    return this._startAutonomy(this._autonomy.startExplore(durationS));
  }

  dockWithCharger() {
    return this._startAutonomy(this._autonomy.startDock());
  }

  get hasPatient() {
    return this._rescue !== null;
  }

  patientHeadAngleRad() {
    return this._rescue ? this._rescue.patientHeadAngleRad() : 0;
  }

  patientLiftAngleRad() {
    return this._rescue ? this._rescue.patientLiftAngleRad() : 0;
  }

  patientWheelAnglesRad() {
    return this._rescue ? this._rescue.patientWheelAnglesRad() : [0, 0, 0, 0];
  }

  rescueSnapshot() {
    return this._rescue ? this._rescue.snapshot() : null;
  }

  resetRescue(side = "right", mode = "auto") {
    if (!this._rescue || this._action !== null) return false;
    this.reset({ rescueSide: side, rescueMode: mode, noiseSeed: 1 });
    return true;
  }

  rescuePatient(mode = "auto") {
    if (!this._rescue || !this.hasControl || this._action !== null) return this._refuse();
    const ctx = this._rescue.startRescue({ mode });
    if (ctx === null) return this._refuse();
    return this._startAutonomy(ctx);
  }

  nomadExplore(durationS = 10) {
    if (!this._nomad.policy) return this._refuse();
    return this._startAutonomy(this._nomad.startExplore(durationS));
  }

  nomadNavigate(maxS = 20, goalNodeId = null) {
    if (!this._nomad.policy) return this._refuse();
    const gid = goalNodeId ?? this._nomad.goalNodeId;
    const hasNode = gid !== null && gid >= 0 && gid < this._nomad.graph.nodes.length;
    if (!hasNode && !this._nomad.goalImgU8) return this._refuse();
    return this._startAutonomy(this._nomad.startNavigate(maxS, goalNodeId));
  }

  // A servo action leaves the treads alone: the Python's _servo_action never zeroes them.
  _newServoAction(kind, target, tol, span, speed) {
    assert(tol > 0 && speed > 0, "servo tolerance and speed must be positive");
    const [target0, accel0] = [this._wheelTarget, this._wheelAccel];
    const made = this._newAction(kind, { servo: true, phase: "run", target, tol, startInTolerance: span <= tol, budget: this._budgetTicks(span / speed) });
    this._wheelTarget = target0;
    this._wheelAccel = accel0;
    return made;
  }

  // Advance the running action by at most one control tick. Returns true when
  // it ran a tick, false when it ended without one (the caller then ticks).
  _advanceAction() {
    const a = this._action;
    if (a.phase === "delay") {
      if (a.left > 0) {
        a.left -= 1;
        this._tickOnce();
        return true;
      }
      this._inAction = true;
      a.phase = "run";
    }
    if (a.phase === "run" && this._runAction(a)) return true;
    return this._finishAction(a);
  }

  // One pass of the action's steering loop. True when it ran a tick.
  _runAction(a) {
    const stop = a.budget <= 0 ? RESULT_TIMEOUT : this._interrupt();
    if (stop !== null) {
      a.result = stop;
      a.phase = "finish";
      return false;
    }
    a.budget -= 1;
    let done = false;
    if (a.autonomy) {
      const ctrl = a.autonomy.controller ?? this._autonomy;
      const stepRes = ctrl.step(a.autonomy);
      if (!stepRes.running) {
        const { running: _r, result, ...extra } = stepRes;
        a.result = result ?? RESULT_COMPLETED;
        a.extra = extra;
        a.phase = "finish";
        return false;
      }
    } else {
      done = a.servo ? this._servoDone(a) : a.kind === "drive" ? this._steerDrive(a) : this._steerTurn(a);
    }
    if (done) {
      a.result = RESULT_COMPLETED;
      a.phase = "finish";
      return false;
    }
    this._tickOnce();
    return true;
  }

  _servoDone(a) {
    if (a.kind === "head") {
      return this.tick_ - a.tick0 >= this._servoDelay && this._acceptedHeadTarget === a.target &&
        this._headInPos && Math.abs(this.headAngleRad() - a.target) <= a.tol;
    }
    return (a.startInTolerance || this._liftInPos) && Math.abs(this.data.qpos[this._q.lift] - a.target) <= a.tol;
  }

  _steerDrive(a) {
    const v = this.v;
    if (a.ux === undefined) {
      a.ux = Math.cos(this._yaw());
      a.uy = Math.sin(this._yaw());
    }
    const [x, y] = this.poseMm();
    const remaining = a.dist - a.sign * ((x - a.start[0]) * a.ux + (y - a.start[1]) * a.uy);
    if (remaining <= v.drive_done_mm) return true;
    a.cmd = Math.min(a.speed, a.cmd + v.wheel_accel_default_mmps2 * this.dt,
      Math.sqrt(2 * v.wheel_decel_default_mmps2 * remaining));
    const steer = v.heading_hold_gain_mmps_per_rad * (a.yaw0 - this._yawTotal);
    this._wheelTarget = [a.sign * a.cmd - steer, a.sign * a.cmd + steer];
    return false;
  }

  _steerTurn(a) {
    const v = this.v;
    const remaining = a.angle - a.sign * (this._yawTotal - a.yaw0);
    if (remaining <= v.turn_tolerance_deg * DEG) return true;
    const accel = v.point_turn_accel_default_radps2;
    const half = 0.5 * v.tread_spacing_mm;
    a.cmd = Math.min(a.rate, a.cmd + accel * this.dt, Math.sqrt(2 * accel * remaining));
    this._wheelTarget = [-a.sign * a.cmd * half, a.sign * a.cmd * half];
    return false;
  }

  // Stop the treads, wait for them to stop, and record what the action did.
  _finishAction(a) {
    if (!a.servo) {
      if (a.finishLeft === 0 && a.phase === "finish") {
        if (this.hasControl) {
          this._wheelTarget = [0, 0];
          this._wheelAccel = [0, 0];
        }
        a.finishLeft = Math.trunc(1 / this.dt);
        a.phase = "stopping";
      }
      const [left, right] = this._wheelSpeeds();
      const stopped = Math.abs(left) + Math.abs(right) <= this.v.wheel_stopped_threshold_mmps;
      const over = (stopped && this._reactPhase === "idle") || a.result === RESULT_OFF_GROUND;
      if (!over && a.finishLeft > 0) {
        a.finishLeft -= 1;
        this._tickOnce();
        return true;
      }
      this._inAction = false;
    }
    const [x, y] = this.poseMm();
    if (a.autonomy) (a.autonomy.controller ?? this._autonomy).restoreControlLevel(a.autonomy);
    this.lastAction = {
      kind: a.kind, result: a.result, moved_mm: Math.hypot(x - a.start[0], y - a.start[1]),
      turned_deg: (this._yawTotal - a.yaw0) / DEG, ticks: this.tick_ - a.tick0,
      ...(a.extra ?? {}),
    };
    this._action = null;
    return false;
  }
}
