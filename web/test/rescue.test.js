// Two-robot continuous rescue macro tests: exported MJCFs, direct/cube righting, causality, and Program API.
import assert from "node:assert/strict";
import { afterEach, test } from "node:test";
import { createProgramRunner } from "../src/program/runner.js";
import { exampleById } from "../src/program/examples.js";
import { programEndpoint } from "../src/program/runtime.js";
import { CONTROL_DEFAULT, RESULT_CANCELLED, RESULT_NO_CONTROL, VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText, sha256 } from "./helpers.js";

const v1Model = readJson("public/robot/model.json");
const v2Model = readJson("public/robot/vector2/model.json");
const activeSims = [];

afterEach(() => {
  while (activeSims.length > 0) activeSims.pop().dispose();
});

async function makeSim(robotDir, sceneFile, opts = {}) {
  const model = robotDir === "vector2" ? v2Model : v1Model;
  const prefix = robotDir === "vector2" ? "public/robot/vector2" : "public/robot";
  const sim = new VectorSim(await mujoco(), {
    xml: readText(`${prefix}/${sceneFile}`),
    values: model.values,
    derived: model.derived,
    camera: model.camera,
    controlLevel: CONTROL_DEFAULT,
    noiseSeed: 1,
    ...opts,
  });
  activeSims.push(sim);
  return sim;
}

function runRescueAction(sim, mode = "auto") {
  assert.equal(sim.rescuePatient(mode), true, `rescuePatient(${mode}) was refused`);
  let peakLeverLiftMm = 0;
  for (let tick = 0; tick < 12000 && sim.actionRunning; tick++) {
    sim.stepAction();
    if (sim.rescueSnapshot().stage === "stage_c_lever") {
      peakLeverLiftMm = Math.max(peakLeverLiftMm, sim.liftHeightMm());
    }
  }
  assert.equal(sim.actionRunning, false, "rescue macro timed out within tick bound");
  return { ...sim.lastAction, peak_lever_lift_mm: peakLeverLiftMm };
}

function withExcludedRobotContacts(xml) {
  const names = [...xml.matchAll(/<body\b[^>]*\bname="([^"]+)"/g)].map((m) => m[1]);
  const pBodies = names.filter((name) => name.startsWith("patient_"));
  const rBodies = names.filter((name) => !name.startsWith("patient_") && name !== "cube" && name !== "dock");
  assert.ok(rBodies.length > 0 && pBodies.length > 0, "expected rescuer and patient bodies");
  const pairs = [];
  for (const rBody of rBodies) {
    for (const pBody of pBodies) {
      pairs.push(`    <exclude body1="${rBody}" body2="${pBody}"/>`);
    }
  }
  const block = `  <contact>\n${pairs.join("\n")}\n  </contact>\n</mujoco>`;
  assert.ok(xml.includes("</mujoco>"), "expected closing </mujoco> tag");
  return xml.replace("</mujoco>", block);
}

class ThreadWorker {
  constructor() {
    this.onmessage = null;
    this.onerror = null;
    this.terminated = false;
    this._handle = programEndpoint((message) => {
      const copy = structuredClone(message);
      setImmediate(() => !this.terminated && this.onmessage?.({ data: copy }));
    });
  }
  postMessage(message) {
    const copy = structuredClone(message);
    setImmediate(() => !this.terminated && this._handle(copy));
  }
  terminate() {
    this.terminated = true;
  }
}

async function runProgram(sim, source) {
  const later = () => new Promise((resolve) => setImmediate(resolve));
  const runner = createProgramRunner(sim, () => new ThreadWorker());
  runner.run(source);
  for (let frame = 0; frame < 40000 && runner.running; frame++) {
    runner.advance(24);
    await later();
  }
  assert.equal(runner.running, false, "program exceeded frame budget");
  const view = runner.view();
  return { ...view, prints: view.log.filter((entry) => entry.kind === "print").map((entry) => entry.text) };
}

test("exported rescue scenes match model.json hashes and expose 8 actuators", async () => {
  for (const [dir, model] of [["robot", v1Model], ["robot/vector2", v2Model]]) {
    for (const stem of ["vector-rescue", "vector-rescue-free"]) {
      const xml = readText(`public/${dir}/${stem}.xml`);
      assert.equal(sha256(xml), model.source[`${stem}_sha256`], `${dir}/${stem}.xml hash mismatch`);
    }
  }
  const sim = await makeSim("robot", "vector-rescue.xml");
  assert.equal(sim.hasPatient, true);
  assert.equal(sim.model.nu, 8);
});

test("direct rescue rights sideways patient on both sides", async () => {
  const sim = await makeSim("robot", "vector-rescue.xml");
  for (const side of ["right", "left"]) {
    assert.equal(sim.resetRescue(side, "direct"), true);
    const out = runRescueAction(sim, "direct");
    assert.equal(out.result, "completed");
    assert.equal(out.mode, "direct");
    assert.equal(out.side, side);
    assert.ok(out.start_tilt_deg > 85, `expected sideways start (${out.start_tilt_deg})`);
    assert.ok(out.final_tilt_deg < 5, `expected upright finish on ${side} (${out.final_tilt_deg})`);
    assert.ok(out.peak_lever_lift_mm > 80, `expected Stage C lever lift > 80 mm on ${side} (${out.peak_lever_lift_mm})`);
    assert.equal(out.rescuer_clear, true);
    assert.ok(out.direct_contact_ticks > 0, `expected direct contact on ${side}`);
  }
});

test("cube-assisted rescue rights v1 (both sides) and v2 through all stages", async () => {
  const v1 = await makeSim("robot", "vector-rescue-free.xml");
  for (const side of ["right", "left"]) {
    assert.equal(v1.resetRescue(side, "auto"), true);
    const out = runRescueAction(v1, "auto");
    assert.equal(out.result, "completed");
    assert.equal(out.mode, "cube");
    assert.deepEqual(out.stages, ["stage_a", "stage_b", "stage_c_nav", "stage_c_settle", "stage_c_lever"]);
    assert.ok(out.start_tilt_deg > 85, `expected sideways start (${out.start_tilt_deg})`);
    assert.ok(out.final_tilt_deg < 5, `expected upright finish on ${side} (${out.final_tilt_deg})`);
    assert.ok(out.peak_lever_lift_mm > 80, `expected Stage C lever lift > 80 mm on ${side} (${out.peak_lever_lift_mm})`);
    assert.equal(out.rescuer_clear, true);
    assert.ok(out.direct_contact_ticks > 0, `expected Stage C contact on ${side}`);
    assert.equal(v1.rescueSnapshot().patientUpright, true);
  }
  const v2 = await makeSim("vector2", "vector-rescue-free.xml");
  const out2 = runRescueAction(v2, "auto");
  assert.equal(out2.result, "completed");
  assert.equal(out2.mode, "cube");
  assert.ok(out2.final_tilt_deg < 5, `expected v2 upright finish (${out2.final_tilt_deg})`);
  assert.ok(out2.peak_lever_lift_mm > 80, `expected Stage C lever lift > 80 mm on v2 (${out2.peak_lever_lift_mm})`);
  assert.equal(out2.rescuer_clear, true);
  assert.ok(out2.direct_contact_ticks > 0, "expected Stage C contact on v2");
});

test("excluding rescuer-patient contacts prevents direct righting", async () => {
  const excludedXml = withExcludedRobotContacts(readText("public/robot/vector-rescue.xml"));
  const sim = new VectorSim(await mujoco(), {
    xml: excludedXml,
    values: v1Model.values,
    derived: v1Model.derived,
    camera: v1Model.camera,
    controlLevel: CONTROL_DEFAULT,
    noiseSeed: 1,
  });
  activeSims.push(sim);
  const out = runRescueAction(sim, "direct");
  assert.ok(out.final_tilt_deg > 85, `patient should remain sideways without contact (${out.final_tilt_deg})`);
  assert.equal(out.direct_contact_ticks, 0);
});

test("rescue refuses without control or while busy, and supports cancel and reset", async () => {
  const sim = await makeSim("robot", "vector-rescue.xml");
  assert.equal(sim.setWheelMotors(-200, -200), true);
  for (let i = 0; i < 800 && (sim.hasControl || sim.simTruth().reacting); i++) sim.tick(1);
  assert.equal(sim.hasControl, false, "backing off rear cliff edge should drop control");
  const snapBefore = sim.rescueSnapshot();
  assert.equal(sim.rescuePatient("direct"), false);
  assert.equal(sim.lastAction.result, RESULT_NO_CONTROL);
  assert.equal(sim.rescueSnapshot().stage, snapBefore.stage);

  assert.equal(sim.requestControl(), true);
  assert.equal(sim.resetRescue("right", "direct"), true);
  assert.equal(sim.driveStraight(40, 50), true);
  assert.equal(sim.rescuePatient("direct"), false, "refused while another action is running");
  assert.equal(sim.cancelAction(), true);

  assert.equal(sim.rescuePatient("direct"), true);
  for (let i = 0; i < 20; i++) sim.stepAction();
  assert.equal(sim.cancelAction(), true);
  assert.equal(sim.lastAction.result, RESULT_CANCELLED);
  assert.equal(sim.rescueSnapshot().stage, "cancelled");

  assert.equal(sim.resetRescue("left", "auto"), true);
  const afterReset = sim.rescueSnapshot();
  assert.equal(afterReset.stage, "ready");
  assert.equal(afterReset.side, "left");
  assert.ok(afterReset.patientTiltDeg > 85);
});

test("Program rescue example completes in rescue scene and rescue_patient fails cleanly on single robot", async () => {
  const sim = await makeSim("robot", "vector-rescue-free.xml");
  const run = await runProgram(sim, exampleById("rescue").source);
  assert.equal(run.status, "finished", JSON.stringify(run.log));
  assert.equal(run.prints.length, 2);
  assert.match(run.prints[0], /^right cube completed tilt: [0-4]\.\d° clear: true$/);
  assert.match(run.prints[1], /^left cube completed tilt: [0-4]\.\d° clear: true$/);

  const single = await makeSim("robot", "vector.xml");
  const errRun = await runProgram(single, "await robot.rescue_patient();\n");
  assert.equal(errRun.status, "failed");
  assert.match(errRun.log.at(-1).text, /\?env=rescue/);
});
