// Programs written in the page, run headless: the example programs, robot time, Stop and errors..
//
// Each program goes through the page's own runner and message protocol. The worker is
// replaced by one that runs the same endpoint in this thread, with messages cloned and
// delivered later, as postMessage does. A program whose loop never awaits cannot be run
// here (it would hold this thread); in the page the worker is terminated instead.
import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import { join } from "node:path";
import { test } from "node:test";
import { NomadPolicy } from "../src/sim/nomad.js";
import { CONTROL_DEFAULT, RESULT_CANCELLED, VectorSim } from "../src/sim/robot.js";
import { IDLE_CALL_LIMIT, ProgramHost } from "../src/program/host.js";
import { directCall, programEndpoint, runProgram } from "../src/program/runtime.js";
import { createProgramRunner } from "../src/program/runner.js";
import { EXAMPLES, exampleById } from "../src/program/examples.js";
import { ROOT, mujoco, readJson, readText } from "./helpers.js";

const model = readJson("public/robot/model.json");
const xml = readText("public/robot/vector.xml");
const v = model.values;
const DEG = 180 / Math.PI;
const later = () => new Promise((resolve) => setImmediate(resolve));

function attachNomadIfPresent(sim) {
  const metaFile = join(ROOT, "public/robot/nomad/nomad.json");
  const binFile = join(ROOT, "public/robot/nomad/nomad_weights.bin");
  if (existsSync(metaFile) && existsSync(binFile)) {
    const policy = NomadPolicy.fromBuffers(JSON.parse(readFileSync(metaFile, "utf8")), new Uint8Array(readFileSync(binFile)));
    sim.attachNomad(policy, model.camera);
  }
}

async function makeSim() {
  const sim = new VectorSim(await mujoco(), {
    xml, values: v, derived: model.derived, camera: model.camera, controlLevel: CONTROL_DEFAULT, noiseSeed: 1,
  });
  attachNomadIfPresent(sim);
  return sim;
}

// The page's worker, in this thread. Keeps every reply the page sent it.
class ThreadWorker {
  constructor() {
    this.onmessage = null;
    this.onerror = null;
    this.terminated = false;
    this.replies = [];
    this._handle = programEndpoint((message) => {
      const copy = structuredClone(message);
      setImmediate(() => !this.terminated && this.onmessage?.({ data: copy }));
    });
  }

  postMessage(message) {
    const copy = structuredClone(message);
    if (copy.type === "reply") this.replies.push(copy);
    setImmediate(() => !this.terminated && this._handle(copy));
  }

  terminate() {
    this.terminated = true;
  }
}

// Run a program through the runner, ticksPerFrame control ticks a frame, until it ends or
// stopAfterFrames frames have passed (then press Stop).
async function runInPage(sim, source, { ticksPerFrame = 24, stopAfterFrames = Infinity, clock } = {}) {
  let worker = null;
  const runner = createProgramRunner(sim, () => (worker = new ThreadWorker()), clock);
  runner.run(source);
  let frames = 0;
  for (; frames < 40000 && runner.running; frames++) {
    if (frames >= stopAfterFrames) runner.stop();
    runner.advance(ticksPerFrame);
    await later();
  }
  assert.equal(runner.running, false, "the program did not end within the frame budget");
  const view = runner.view();
  return { ...view, prints: view.log.filter((l) => l.kind === "print").map((l) => l.text), replies: worker.replies, runner };
}

const valuesOf = (replies) => replies.filter((r) => r.ok).map((r) => r.value);
const actionsOf = (replies) => valuesOf(replies).filter((x) => x && typeof x === "object" && "result" in x);

test("the square example drives four 120 mm sides and four quarter turns", async () => {
  const sim = await makeSim();
  const run = await runInPage(sim, exampleById("square").source);
  assert.equal(run.status, "finished", JSON.stringify(run.log));
  const actions = actionsOf(run.replies);
  assert.equal(actions.length, 8, "four drives and four turns");
  for (let side = 0; side < 4; side++) {
    const [drive, turn] = [actions[2 * side], actions[2 * side + 1]];
    assert.equal(drive.result, "completed");
    assert.ok(drive.moved_mm > 120 - v.drive_done_mm && drive.moved_mm < 124, `side ${side + 1} drove ${drive.moved_mm} mm`);
    assert.ok(Math.abs(drive.turned_deg) < 1, `side ${side + 1} held its heading (${drive.turned_deg} deg)`);
    assert.equal(turn.result, "completed");
    assert.ok(Math.abs(turn.turned_deg - 90) <= v.turn_tolerance_deg, `corner ${side + 1} turned ${turn.turned_deg} deg`);
  }
  // The physics, not the program's own report: the robot came round to where it started.
  const heading = sim.headingTotalRad() * DEG;
  assert.ok(Math.abs(heading - 360) <= 4 * v.turn_tolerance_deg, `turned ${heading} deg in all`);
  const [x, y, z] = sim.poseMm();
  assert.ok(Math.hypot(x, y) < 15, `ended ${Math.hypot(x, y).toFixed(1)} mm from the start`);
  assert.ok(Math.abs(z) < 1, "still on the table");
  assert.equal(run.prints.length, 5);
  assert.match(run.prints[0], /^side 1: completed, 12\d\.\d mm, turned \d\d\.\d deg$/);
  sim.dispose();
});

test("the head, lift and wheels example moves each as commanded", async () => {
  const sim = await makeSim();
  const run = await runInPage(sim, exampleById("look").source);
  assert.equal(run.status, "finished", JSON.stringify(run.log));
  const [head, lift] = run.prints[0].match(/-?\d+(\.\d+)?/g).map(Number);
  assert.ok(Math.abs(head - 40) <= v.head_angle_tol_deg, `head at ${head} deg`);
  assert.ok(lift > v.lift_height_max_mm - 5, `lift at ${lift} mm`);
  const turned = Number(run.prints[1].match(/-?\d+\.\d/)[0]);
  assert.ok(turned > 30 && turned < 60, `the arc turned ${turned} deg to the left`);
  assert.ok(Math.abs(sim.headAngleRad() * DEG) <= v.head_angle_tol_deg, "head back to level");
  assert.ok(sim.liftHeightMm() < v.lift_height_min_mm + 3, "lift back down");
  sim.dispose();
});

test("the edge example is stopped by the cliff reaction and drives away", async () => {
  const sim = await makeSim();
  const run = await runInPage(sim, exampleById("edge").source);
  assert.equal(run.status, "finished", JSON.stringify(run.log));
  assert.match(run.prints[0], /^drive ended: cancelled_cliff after \d+ mm$/);
  assert.deepEqual(run.prints.slice(1, 3), ["has control: false", "control back: true"]);
  assert.equal(sim.reactions, 1);
  const [x, , z] = sim.poseMm();
  assert.ok(x < 0 && Math.abs(z) < 1, `ended at x ${x} mm, z ${z} mm`);
  sim.dispose();
});

test("every example finishes without an error", async () => {
  assert.ok(EXAMPLES.length >= 3);
  for (const example of EXAMPLES) {
    const sim = await makeSim();
    const run = await runInPage(sim, example.source);
    assert.equal(run.status, "finished", `${example.id}: ${JSON.stringify(run.log)}`);
    assert.ok(!run.log.some((l) => l.kind === "error"), example.id);
    sim.dispose();
  }
});

test("robot time moves only while the program waits: frame size does not change the run", async () => {
  const poses = [];
  for (const ticksPerFrame of [1, 7, 24]) {
    const sim = await makeSim();
    const run = await runInPage(sim, exampleById("square").source, { ticksPerFrame });
    assert.equal(run.status, "finished");
    poses.push([sim.tickCount, ...sim.data.qpos]);
    sim.dispose();
  }
  assert.deepEqual(poses[1], poses[0]);
  assert.deepEqual(poses[2], poses[0]);
});

test("wait lets exactly that much robot time pass, and nothing else does", async () => {
  const sim = await makeSim();
  const start = sim.tickCount;
  const run = await runInPage(sim, `
    await robot.set_wheel_motors(50, 50);
    const before = robot.state.now_s;
    for (let i = 0; i < 50; i++) await robot.request_control();
    print(robot.state.now_s - before);
    await robot.wait(1.0);
    print(robot.state.now_s - before);
  `);
  assert.equal(run.status, "finished", JSON.stringify(run.log));
  assert.deepEqual(run.prints.map(Number), [0, 1]);
  assert.equal(sim.tickCount - start, Math.round(1.0 / sim.dt));
  // The program ended with the wheel motors on: ending lets go of the robot, and the treads stop.
  sim.tick(200);
  const [left, right] = sim.simTruth().wheel_speeds_mmps;
  assert.ok(Math.abs(left) + Math.abs(right) < v.wheel_stopped_threshold_mmps, `treads at ${left}, ${right} mm/s after the end`);
  sim.dispose();
});

test("set_lift_height takes the SDK's 0 to 1 range", async () => {
  const sim = await makeSim();
  const host = new ProgramHost(sim);
  const call = directCall(host);
  for (const [fraction, mm] of [[0.5, 62], [1.7, v.lift_height_max_mm], [-1, v.lift_height_min_mm]]) {
    const done = call("set_lift_height", [fraction]);
    for (let i = 0; i < 100 && host.waitingOn !== null; i++) host.advance(24);
    const result = await done;
    assert.ok(["completed", "timeout"].includes(result.result));
    assert.ok(Math.abs(sim.liftTargetMm - mm) < 1e-9, `${fraction} gave a ${sim.liftTargetMm} mm target`);
  }
  sim.dispose();
});

test("Stop cancels the running action and the treads stop", async () => {
  const sim = await makeSim();
  const run = await runInPage(sim, `
    await robot.drive_straight(250, 50);
    print("not reached");
  `, { stopAfterFrames: 30 });
  assert.equal(run.status, "stopped");
  assert.deepEqual(run.prints, []);
  assert.equal(sim.actionRunning, false);
  assert.equal(sim.lastAction.result, RESULT_CANCELLED);
  assert.equal(run.replies.at(-1).ok, false, "the waiting call is answered with an error");
  const stoppedAt = sim.poseMm()[0];
  sim.tick(200);
  const [left, right] = sim.simTruth().wheel_speeds_mmps;
  assert.ok(Math.abs(left) + Math.abs(right) < v.wheel_stopped_threshold_mmps, "treads still turning");
  // What a stop costs on this robot: the treads run on for the command delay, then brake
  // (5.25 mm from 50 mm/s, the same as stop_all_motors in direct drive). Not stopped, it goes 50 mm.
  const coast = Math.abs(sim.poseMm()[0] - stoppedAt);
  assert.ok(coast < 50 * (v.command_delay_ms / 1000) + 3, `the robot went on ${coast.toFixed(2)} mm after Stop`);
  sim.dispose();
});

test("cancelAction on its own ends the action and stops the treads", async () => {
  // Through Stop the host also zeroes the wheels, which would hide a cancel that does not.
  const sim = await makeSim();
  assert.equal(sim.cancelAction(), false, "nothing to cancel");
  assert.equal(sim.driveStraight(250, 50), true);
  for (let i = 0; i < 400; i++) sim.stepAction();
  const stoppedAt = sim.poseMm()[0];
  assert.equal(sim.cancelAction(), true);
  assert.equal(sim.actionRunning, false);
  assert.equal(sim.lastAction.result, RESULT_CANCELLED);
  assert.ok(Math.abs(sim.lastAction.moved_mm - stoppedAt) < 1, "the record is of the distance driven");
  sim.tick(200);
  const coast = Math.abs(sim.poseMm()[0] - stoppedAt);
  assert.ok(coast < 50 * (v.command_delay_ms / 1000) + 3, `the robot went on ${coast.toFixed(2)} mm after the cancel`);
  sim.dispose();
});

test("a program that calls the robot without awaiting is told so, at its line", async () => {
  const sim = await makeSim();
  const run = await runInPage(sim, "robot.drive_straight(100, 50);\nawait robot.turn_in_place(90);\n");
  assert.equal(run.status, "failed");
  assert.equal(run.errorLine, 2);
  assert.match(run.log.at(-1).text, /^Line 2: RobotError: turn_in_place was called while drive_straight is still running/);
  sim.dispose();
});

test("errors are reported with the program's own line number", async () => {
  const cases = [
    ["const a = 1;\nconst b = 2;\nnull.boom;\n", 3, /TypeError/],
    ["\n\nawait robot.drive_straight(100, 0);\n", 3, /RobotError: drive_straight: speed_mmps must be above 0/],
    ["await robot.wait(0.1);\nawait robot.drive_straight('far', 50);\n", 2, /TypeError: robot.drive_straight: argument 1 must be a number/],
    ["throw new Error('mine');", 1, /Error: mine/],
  ];
  for (const [source, line, message] of cases) {
    const sim = await makeSim();
    const run = await runInPage(sim, source);
    assert.equal(run.status, "failed", source);
    assert.equal(run.errorLine, line, source);
    assert.match(run.log.at(-1).text, message);
    sim.dispose();
  }
  const sim = await makeSim();
  const syntax = await runInPage(sim, "const = 3;");
  assert.equal(syntax.status, "failed");
  assert.match(syntax.log.at(-1).text, /SyntaxError/);
  sim.dispose();
});

test("a loop that never lets robot time pass is stopped with an explanation", async () => {
  const sim = await makeSim();
  const host = new ProgramHost(sim);
  const outcome = await runProgram(`for (let i = 0; i < ${2 * IDLE_CALL_LIMIT}; i++) await robot.stop_all_motors();`, {
    call: directCall(host), print: () => {}, state: () => host.snapshot(),
  });
  assert.equal(outcome.ok, false);
  assert.match(outcome.error, new RegExp(`${IDLE_CALL_LIMIT} robot calls in a row without robot time passing`));
  assert.equal(outcome.line, 1);
  sim.dispose();
});

test("a program that stops calling the robot gets a hint, and Stop still ends it", async () => {
  const sim = await makeSim();
  let ms = 0;
  const clock = () => (ms += 50);
  const run = await runInPage(sim, "await robot.wait(0.1);\nawait new Promise(() => {});\n", { stopAfterFrames: 200, clock });
  assert.equal(run.status, "stopped");
  assert.ok(run.log.some((l) => l.text.startsWith("No robot call for 3 s.")), JSON.stringify(run.log));
  sim.dispose();
});

test("drive_off_charger, explore, and dock_with_charger complete and update robot.state", async () => {
  const dockXml = readText("public/robot/vector-free-dock.xml");
  const sim = new VectorSim(await mujoco(), {
    xml: dockXml, values: v, derived: model.derived, camera: model.camera, startDocked: true, noiseSeed: 1,
  });
  try {
    const run = await runInPage(sim, `
      const off = await robot.drive_off_charger();
      print("off:", off.result, robot.state.on_charger);
      const exp = await robot.explore(3.5);
      print("exp:", exp.result, exp.cells_mapped >= 20);
      const docked = await robot.dock_with_charger();
      print("dock:", docked.result, robot.state.on_charger, robot.state.battery_volts);
    `);
    assert.equal(run.status, "finished", JSON.stringify(run.log));
    assert.deepEqual(run.prints, [
      "off: completed false",
      "exp: completed true",
      "dock: completed true 4.2",
    ]);
  } finally {
    sim.dispose();
  }
});

test("nomad_capture_goal, nomad_explore, and nomad_navigate run cleanly in Program", async () => {
  const sim = await makeSim();
  try {
    const run = await runInPage(sim, exampleById("nomad-nav").source);
    assert.equal(run.status, "finished", JSON.stringify(run.log));
    assert.match(run.prints[0], /^captured goal node 0$/);
    assert.match(run.prints[1], /^explore nodes: [1-9]\d* edges: \d+$/);
    assert.match(run.prints[2], /^navigate: (completed|timeout) pred_dist: /);
  } finally {
    sim.dispose();
  }
});

test("a page program can time one head move and return at default speed", async () => {
  const sim = await makeSim();
  try {
    const run = await runInPage(sim, "await robot.set_head_angle(30, null, 1.4); await robot.set_head_angle(0);");
    assert.equal(run.status, "finished", JSON.stringify(run.log));
    const [timed, normal] = actionsOf(run.replies);
    assert.equal(timed.result, "completed");
    assert.equal(normal.result, "completed");
    assert.ok(timed.ticks * sim.dt > 1.2);
    assert.ok(normal.ticks * sim.dt < 1.0);
  } finally {
    sim.dispose();
  }
});
