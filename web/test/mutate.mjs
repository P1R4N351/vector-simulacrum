// Break each tested property once and confirm the tests notice..
//
//   node test/mutate.mjs
//
// Each arm copies src/sim, test and the exported model into a fresh temporary
// folder, changes one line there, and runs the whole suite on the copy. A
// faulty arm must make the suite fail; the untouched copy and a comment-only
// change must pass. An arm whose text is not found exactly once, or that
// leaves the file unchanged, is a fault of this script and is reported as
// SETUP-ERROR, never as a verdict on the tests. If the untouched or the
// comment-only copy fails, the copy itself is broken (a file it needs is not
// copied, say) and every other verdict would be void, so the run stops there.
// Exit: 0 every arm as required, 7 a fault survived, 3 a set-up error or a failing clean copy.
import { spawnSync } from "node:child_process";
import { cpSync, mkdtempSync, readFileSync, rmSync, symlinkSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = join(dirname(fileURLToPath(import.meta.url)), "..");
const COPIED = ["src/sim", "src/game", "src/program", "test", "public/robot/vector.xml", "public/robot/vector-contact42.xml",
  "public/robot/vector-free.xml", "public/robot/vector-contact42-free.xml", "public/robot/model.json",
  "public/robot/vector2", "camera_calibration.json", "camera_calibration_vector2.json", "package.json"];
const SUITE = ["test/drive.test.js", "test/hazards.test.js", "test/robot.test.js", "test/tracks.test.js", "test/optics.test.js",
  "test/headcam.test.js", "test/prox.test.js", "test/imu_filter.test.js", "test/face.test.js", "test/model3d.test.js",
  "test/program.test.js"];
const ARMS = [
  { name: "control: untouched copy", file: null, mustFail: false },
  { name: "neutral: a comment added", file: "src/sim/drive.js", old: "export class TreadEncoder {", new: "// neutral change\nexport class TreadEncoder {", mustFail: false },
  { name: "encoder rounds counts instead of truncating", file: "src/sim/drive.js", old: "const total = Math.trunc(this._pathMm / this._mmPerCount);", new: "const total = Math.round(this._pathMm / this._mmPerCount);", mustFail: true },
  { name: "speed loop integrates while saturated", file: "src/sim/drive.js", old: "if (Math.abs(this.power) < this._max) {", new: "if (this.power !== 12345) {", mustFail: true },
  { name: "feedforward loses its offset", file: "src/sim/drive.js", old: "this._slope * speed + this._offset", new: "this._slope * speed", mustFail: true },
  { name: "fall declared one tick early", file: "src/sim/hazards.js", old: "if (this._fallTicks > this._fallNeed) this.falling = true;", new: "if (this._fallTicks >= this._fallNeed) this.falling = true;", mustFail: true },
  { name: "no reading below is not a cliff", file: "src/sim/hazards.js", old: "this.cliff[i] = rangesMm[i] < 0 || rangesMm[i] > this._rangeMm;", new: "this.cliff[i] = rangesMm[i] > this._rangeMm;", mustFail: true },
  { name: "pick-up declared one tick early", file: "src/sim/hazards.js", old: "this.pickedUp = this._pickupTicks > this._pickupNeed;", new: "this.pickedUp = this._pickupTicks >= this._pickupNeed;", mustFail: true },
  { name: "cliff reaction drives toward the edge", file: "src/sim/robot.js", old: "this._reactDir = front ? -1 : 1;", new: "this._reactDir = front ? 1 : -1;", mustFail: true },
  { name: "override level still reacts to a cliff", file: "src/sim/robot.js", old: 'if (this.controlLevel !== CONTROL_DEFAULT || this._reactPhase !== "idle") return;', new: 'if (this._reactPhase !== "idle") return;', mustFail: true },
  { name: "cliff reaction leaves control with the caller", file: "src/sim/robot.js", old: "    this.hasControl = false;\n", new: "", mustFail: true },
  { name: "wheel commands arrive with no delay", file: "src/sim/robot.js", old: "this._wheelDelay = pyRound(v.command_delay_ms / v.control_period_ms);", new: "this._wheelDelay = 0;", mustFail: true },
  { name: "one physics step fewer per control tick", file: "src/sim/robot.js", old: "for (let i = 0; i < this.d.substeps; i++) {", new: "for (let i = 1; i < this.d.substeps; i++) {", mustFail: true },
  { name: "drive_straight does not hold its heading", file: "src/sim/robot.js", old: "const steer = v.heading_hold_gain_mmps_per_rad * (a.yaw0 - this._yawTotal);", new: "const steer = 0;", mustFail: true },
  { name: "reset keeps the old physics state", file: "src/sim/robot.js", old: "    this.mujoco.mj_resetData(this.model, this.data);\n", new: "", mustFail: true },
  { name: "rear wheel angles read from the front wheels", file: "src/sim/robot.js", old: 'this._allWheelQ = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"].map(qadr);', new: 'this._allWheelQ = ["wheel_fl", "wheel_fr", "wheel_fl", "wheel_fr"].map(qadr);', mustFail: true },
  { name: "rear wheel spin read from the front wheels", file: "src/sim/robot.js", old: 'this._allWheelDof = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"].map(dof);', new: 'this._allWheelDof = ["wheel_fl", "wheel_fr", "wheel_fl", "wheel_fr"].map(dof);', mustFail: true },
  { name: "lift target reported as an angle, not a height", file: "src/sim/robot.js", old: "return liftHeightOfAngle(this.v, this._liftTarget);", new: "return this._liftTarget;", mustFail: true },
  { name: "track path: rear arc too long", file: "src/game/tracks.js", old: "const rearArc = rear.r * (2 * Math.PI - 2 * gamma);", new: "const rearArc = rear.r * (2 * Math.PI - gamma);", mustFail: true },
  { name: "track path: runs backwards along the top", file: "src/game/tracks.js", old: "if (u < straight) return between2(onWheel(rear, top), onWheel(front, top), u / straight);", new: "if (u < straight) return between2(onWheel(front, top), onWheel(rear, top), u / straight);", mustFail: true },
  { name: "track path: normal points inward on the wheels", file: "src/game/tracks.js", old: "z: wheel.z + wheel.r * Math.sin(angle), nx: Math.cos(angle), nz: Math.sin(angle),", new: "z: wheel.z + wheel.r * Math.sin(angle), nx: -Math.cos(angle), nz: -Math.sin(angle),", mustFail: true },
  { name: "battery voltage does not scale the drive", file: "src/sim/robot.js", old: "ctrl[this._act.left] = duties[0] * scale * keep[0];", new: "ctrl[this._act.left] = duties[0] * keep[0];", mustFail: true },
  { name: "battery does not sag under load", file: "src/sim/robot.js", old: "this.batteryVolts = v.battery_rest_voltage_v - v.battery_sag_v_per_power * load;", new: "this.batteryVolts = v.battery_rest_voltage_v;", mustFail: true },
  { name: "right tread keeps its drive when hot", file: "src/sim/robot.js", old: "ctrl[this._act.right] = duties[1] * scale * keep[1];", new: "ctrl[this._act.right] = duties[1] * scale;", mustFail: true },
  { name: "tread heat does not build", file: "src/sim/robot.js", old: "this.treadHeat[i] += (Math.abs(duties[i]) - this.treadHeat[i]) * Math.min(1, this.dt / tau);", new: "this.treadHeat[i] += 0;", mustFail: true },
  { name: "head and lift ramp with no acceleration limit", file: "src/sim/util.js", old: "const speed = Math.min(cap, rate + accel * dt, Math.sqrt(2 * accel * remaining));", new: "const speed = cap;", mustFail: true },
  { name: "wheel command clamped to the path-following limit", file: "src/sim/robot.js", old: "this._wheelTarget = [leftMmps, rightMmps];", new: "this._wheelTarget = [clamp(leftMmps, -220, 220), clamp(rightMmps, -220, 220)];", mustFail: true },
  { name: "range sensor: signal ignores how much of the cone is filled", file: "src/sim/prox.js", old: "return [total / this.rays.length, total > 0 ? weighted / total : 0];", new: "return [total / Math.max(1, hits.filter((d) => d >= 0).length), total > 0 ? weighted / total : 0];", mustFail: true },
  { name: "range sensor: lift in the beam ignored", file: "src/sim/prox.js", old: "let [quality, raw] = inBeam ? [v.tof_lift_quality, v.tof_lift_distance_mm] : this._signal(hits);", new: "let [quality, raw] = this._signal(hits);", mustFail: true },
  { name: "range sensor: lift out of view at once", file: "src/sim/prox.js", old: "return this._clearCount < this._clearNeeded;", new: "return false;", mustFail: true },
  { name: "range sensor: pitch does not stop an object being found", file: "src/sim/prox.js", old: " && pitchRad >= -limit && pitchRad <= limit;", new: ";", mustFail: true },
  { name: "range sensor: reading not clamped at the top", file: "src/sim/prox.js", old: "pyRound(Math.min(Math.max(raw, v.tof_report_min_mm), v.tof_report_max_mm))", new: "pyRound(Math.max(raw, v.tof_report_min_mm))", mustFail: true },
  { name: "range sensor: reading not clamped at the bottom", file: "src/sim/prox.js", old: "pyRound(Math.min(Math.max(raw, v.tof_report_min_mm), v.tof_report_max_mm))", new: "pyRound(Math.min(raw, v.tof_report_max_mm))", mustFail: true },
  { name: "range sensor: a weak signal reports the maximum", file: "src/sim/prox.js", old: "raw = this._gauss(v.tof_weak_distance_mm, v.tof_weak_distance_sd_mm);", new: "raw = v.tof_report_max_mm;", mustFail: true },
  { name: "range sensor: rays see the robot's own lift", file: "src/sim/robot.js", old: "this._sceneOnly = [1, 0, 0, 0, 0, 0];", new: "this._sceneOnly = [1, 1, 0, 0, 0, 0];", mustFail: true },
  { name: "range sensor: fan cast backwards", file: "src/sim/robot.js", old: "const direction = [m[f] * x + m[f + 1] * y + m[f + 2] * z,", new: "const direction = [m[f] * x + m[f + 1] * y - m[f + 2] * z,", mustFail: true },
  { name: "camera: optical centre mirrored left to right", file: "src/sim/optics.js", old: "(2 * c.fx_px) / w, 0, 1 - (2 * c.cx_px) / w, 0,", new: "(2 * c.fx_px) / w, 0, (2 * c.cx_px) / w - 1, 0,", mustFail: true },
  { name: "camera: optical centre mirrored top to bottom", file: "src/sim/optics.js", old: "0, (2 * c.fy_px) / h, (2 * c.cy_px) / h - 1, 0,", new: "0, (2 * c.fy_px) / h, 1 - (2 * c.cy_px) / h, 0,", mustFail: true },
  { name: "camera: image drawn upside down", file: "src/sim/optics.js", old: "c.cy_px - (c.fy_px * point[1]) / depth];", new: "c.cy_px + (c.fy_px * point[1]) / depth];", mustFail: true },
  { name: "camera: edge angles ignore the optical centre", file: "src/sim/optics.js", old: "left: deg(c.cx_px / c.fx_px), right: deg((c.width_px - c.cx_px) / c.fx_px),", new: "left: deg(c.width_px / 2 / c.fx_px), right: deg(c.width_px / 2 / c.fx_px),", mustFail: true },
  { name: "camera: tilted up instead of down", file: "src/game/headcam.js", old: "makeRotationX(((rgbUp - mountDown) * Math.PI) / 180)", new: "makeRotationX(((mountDown - rgbUp) * Math.PI) / 180)", mustFail: true },
  { name: "camera: tilt left out", file: "src/game/headcam.js", old: "      ).multiply(tilt);\n", new: "      );\n", mustFail: true },
  { name: "camera: pose read from the wrong place in the solver", file: "src/game/headcam.js", old: "m[o], m[o + 1], m[o + 2], p[3 * id],", new: "m[o], m[o + 1], m[o + 2], p[3 * id] + 0.01,", mustFail: true },
  { name: "IMU filter corners ignore the second section", file: "src/sim/imu_filter.js", old: "const x = 2 / ((1 + q2) + Math.sqrt((1 + q2) ** 2 + 4 * q2));", new: "const x = 1;", mustFail: true },
  { name: "IMU filter runs one section", file: "src/sim/imu_filter.js", old: "s1[ch] += g1[ch] * (s0[ch] - s1[ch]);", new: "s1[ch] = s0[ch];", mustFail: true },
  { name: "IMU filter corner taken as rad/s", file: "src/sim/imu_filter.js", old: "const gain = 1 - Math.exp(-2 * Math.PI * corners[row] * stepS);", new: "const gain = 1 - Math.exp(-corners[row] * stepS);", mustFail: true },
  { name: "robot reads the raw sensor, not the filter", file: "src/sim/robot.js", old: "const out = this._imuFilter.output;", new: "const out = this._imuAdr.map((adr) => this.data.sensordata[adr]);", mustFail: true },
  { name: "IMU filter stepped once per tick", file: "src/sim/robot.js", old: "      this._imuFilter.update(raw);\n      if (this.cubeSensor) {", new: "      if (i === this.d.substeps - 1) this._imuFilter.update(raw);\n      if (this.cubeSensor) {", mustFail: true },
  { name: "IMU filter not reset after settling", file: "src/sim/robot.js", old: "this.data.time = 0;\n    this._resetImuFilter();", new: "this.data.time = 0;", mustFail: true },
  { name: "accel_scale dropped in _sampleImu", file: "src/sim/robot.js", old: "accel[i] = clamp((out[i] / MM) * v.accel_scale[i] + mg * v.accel_bias_mg[i]", new: "accel[i] = clamp(out[i] / MM + mg * v.accel_bias_mg[i]", mustFail: true },
  { name: "3D belt: outer face given sidewall normal", file: "src/game/tracks.js", old: "[p.nx, 0, p.nz]", new: "[0, 1, 0]", mustFail: true },
  { name: "3D shell: decal normals left unnormalized", file: "src/game/shell.js", old: "  geometry.normalizeNormals();\n", new: "", mustFail: true },
  { name: "3D rig: stock fork pivoted on wrist instead of fork_pivot", file: "src/game/sdkmodel.js", old: "s.position.set(-pose.fork_pivot[0], 0, -pose.fork_pivot[1]);", new: "s.position.set(0, 0, 0);", mustFail: true },
  { name: "3D topology: cover sliver normals left unaligned", file: "src/game/topology.js", old: "    alignGeometryNormals(geo);\n", new: "", mustFail: true },
  { name: "program: robot time runs while the program computes", file: "src/program/runner.js", old: "const ran = host.advance(ticks);", new: "const ran = host.waitingOn === null ? (sim.tick(ticks), ticks) : host.advance(ticks);", mustFail: true },
  { name: "program: wait one tick short", file: "src/program/host.js", old: "const ticks = pyRound(seconds / h.sim.dt);", new: "const ticks = pyRound(seconds / h.sim.dt) - 1;", mustFail: true },
  { name: "program: lift height taken as millimetres", file: "src/program/host.js", old: "return v.lift_height_min_mm + f * (v.lift_height_max_mm - v.lift_height_min_mm);", new: "return fraction;", mustFail: true },
  { name: "program: Stop leaves the action running", file: "src/program/host.js", old: '      if (this._pending.kind === "action") this.sim.cancelAction();\n', new: "", mustFail: true },
  { name: "program: the end leaves the treads running", file: "src/program/host.js", old: "if (!this.finished) this.sim.setWheelMotors(0, 0);", new: "if (!this.finished) this.finished = false;", mustFail: true },
  { name: "program: a call made before the last one ended is accepted", file: "src/program/host.js", old: "if (this._pending !== null) {\n      throw new RobotCallError(", new: "if (false) {\n      throw new RobotCallError(", mustFail: true },
  { name: "program: the spinning-loop limit never trips", file: "src/program/host.js", old: "if (this._idleCalls > IDLE_CALL_LIMIT) {", new: "if (this._idleCalls > Infinity) {", mustFail: true },
  { name: "program: error line not corrected for the engine's header", file: "src/program/runtime.js", old: "  return line - offset;\n", new: "  return line;\n", mustFail: true },
  { name: "program: a refused call loses the program's line", file: "src/program/runtime.js", old: "if (err instanceof Error) err.callStack = site.stack;", new: "", mustFail: true },
  { name: "cancelAction leaves the treads driving", file: "src/sim/robot.js", old: "    const stopped = this.stopAllMotors();\n", new: "    const stopped = true;\n", mustFail: true },
  { name: "3D cube: marker UVs mirrored horizontally", file: "src/game/cube.js", old: "uv.setXY(k, face.cx + (rMm / 11.5) * face.h, face.cy - (uMm / 11.5) * face.h);", new: "uv.setXY(k, face.cx - (rMm / 11.5) * face.h, face.cy - (uMm / 11.5) * face.h);", mustFail: true },
];

function makeCopy() {
  const dir = mkdtempSync(join(tmpdir(), "vector-web-mutant-"));
  for (const relative of COPIED) cpSync(join(ROOT, relative), join(dir, relative), { recursive: true });
  symlinkSync(join(ROOT, "node_modules"), join(dir, "node_modules"), "dir");
  return dir;
}

// Returns null when applied, or the reason it could not be.
function applyArm(dir, arm) {
  if (arm.file === null) return null;
  const path = join(dir, arm.file);
  const before = readFileSync(path, "utf8");
  const count = before.split(arm.old).length - 1;
  if (count !== 1) return `the text to change occurs ${count} times in ${arm.file}, not once`;
  if (arm.old === arm.new) return "the change is a no-op";
  const after = before.replace(arm.old, () => arm.new);
  if (after === before) return "the file is unchanged after the change";
  writeFileSync(path, after);
  return null;
}

function runSuite(dir) {
  const done = spawnSync(process.execPath, ["--test", ...SUITE],
    { cwd: dir, encoding: "utf8", timeout: 180000 });
  const counted = /^(?:ℹ|#) tests (\d+)$/m.exec(done.stdout ?? "");
  return { status: done.status, tests: counted ? Number(counted[1]) : 0, timedOut: done.error?.code === "ETIMEDOUT" };
}

const tally = { asRequired: 0, survived: [], setupErrors: [] };
for (const arm of ARMS) {
  const dir = makeCopy();
  let verdict = "";
  try {
    const problem = applyArm(dir, arm);
    const run = problem === null ? runSuite(dir) : null;
    if (problem !== null) verdict = `SETUP-ERROR (${problem})`;
    else if (run.timedOut || run.tests === 0) verdict = `SETUP-ERROR (the suite did not run to a count: status ${run.status})`;
    else if (arm.mustFail) verdict = run.status !== 0 ? "CAUGHT" : "SURVIVED";
    else verdict = run.status === 0 ? "PASSED AS REQUIRED" : "FAILED ON A CLEAN COPY";
  } finally {
    rmSync(dir, { recursive: true, force: true }); // this script's own temporary copy
  }
  console.log(`${verdict.padEnd(22)} ${arm.name}`);
  if (verdict === "FAILED ON A CLEAN COPY") {
    console.log("RUN VOID: a copy with no fault in it fails the suite, so no arm below it could mean anything.");
    console.log(JSON.stringify({ arms: ARMS.length, void: true, clean_copy_failed: arm.name }));
    process.exit(3);
  }
  if (verdict.startsWith("SETUP-ERROR")) tally.setupErrors.push(arm.name);
  else if (verdict === "SURVIVED") tally.survived.push(arm.name);
  else tally.asRequired += 1;
}
const faulty = ARMS.filter((arm) => arm.mustFail).length;
console.log(JSON.stringify({ arms: ARMS.length, faulty, as_required: tally.asRequired, survived: tally.survived, setup_errors: tally.setupErrors }));
process.exit(tally.setupErrors.length > 0 ? 3 : tally.survived.length > 0 ? 7 : 0);
