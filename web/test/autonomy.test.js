// Optical marker localization, NavMapGrid occupancy map, exploration, and self-docking.
import assert from "node:assert/strict";
import { test } from "node:test";
import {
  CELL_CLIFF,
  CELL_CLEAR,
  NavMapGrid,
  observeMarkers,
} from "../src/sim/autonomy.js";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

const model = readJson("public/robot/model.json");
const dockXml = readText("public/robot/vector-free-dock.xml");

test("NavMapGrid keeps sticky cliffs, tracks cube clearance, and snapshots state", () => {
  const grid = new NavMapGrid();
  grid.recordCube({ x: 100, y: 0 }, 5);
  assert.equal(grid.segmentClear(0, 0, 180, 0), false);
  assert.equal(grid.segmentClear(0, 100, 180, 100), true);
  grid.setCell(20, 20, CELL_CLIFF);
  grid.setCell(20, 20, CELL_CLEAR);
  assert.equal(grid.getCell(20, 20), CELL_CLIFF);
  const snap = grid.snapshot();
  assert.equal(snap.cliffs.length, 1);
  assert.deepEqual(snap.cube, { x: 100, y: 0, seenAt: 5 });
});

test("observeMarkers enforces FOV, Light Cube occlusion, gyro gate, and 4 mm localization", async () => {
  const runtime = await mujoco();
  const clearSim = new VectorSim(runtime, {
    xml: dockXml, values: model.values, derived: model.derived, camera: model.camera,
    startXyMm: [0, 0], headingDeg: 180, headDeg: -10, liftMm: 32, noiseSeed: 1,
  });
  try {
    clearSim.setHeadAngle(0);
    for (let i = 0; i < 60 && clearSim.actionRunning; i++) clearSim.tick(1);
    const obs = observeMarkers(clearSim, model.camera);
    assert.ok(obs.charger, "expected charger detection even while body pitches");
    assert.ok(Math.abs(obs.charger.lipX - (-120)) < 4, `lipX=${obs.charger.lipX}`);
    assert.ok(Math.abs(obs.charger.lipY) < 4, `lipY=${obs.charger.lipY}`);
    assert.ok(obs.charger.outX > 0.98);
    // Spinning fast gates marker detection
    assert.equal(clearSim.setWheelMotors(-220, 220), true);
    clearSim.tick(50);
    assert.ok(Math.abs(clearSim._imu()[1][2]) > 1.2, `expected gyro_z > 1.2, got ${clearSim._imu()[1][2]}`);
    assert.equal(observeMarkers(clearSim, model.camera).charger, null);
  } finally {
    clearSim.dispose();
  }

  // Light Cube at (100, 0) occludes charger marker when viewed from behind at (185, 0)
  const occSim = new VectorSim(runtime, {
    xml: dockXml, values: model.values, derived: model.derived, camera: model.camera,
    startXyMm: [185, 0], headingDeg: 180, headDeg: 4, liftMm: 32, noiseSeed: 1,
  });
  try {
    occSim.tick(5);
    assert.equal(observeMarkers(occSim, model.camera).charger, null);
  } finally {
    occSim.dispose();
  }

  // Looking away sees neither dock nor cube
  const awaySim = new VectorSim(runtime, {
    xml: dockXml, values: model.values, derived: model.derived, camera: model.camera,
    startXyMm: [140, 140], headingDeg: 0, headDeg: 4, liftMm: 32, noiseSeed: 1,
  });
  try {
    awaySim.tick(5);
    const none = observeMarkers(awaySim, model.camera);
    assert.equal(none.charger, null);
    assert.equal(none.cube, null);
  } finally {
    awaySim.dispose();
  }
});

test("driveOffCharger, explore, and multi-pose dockWithCharger seat cleanly at 4.2 V", async () => {
  const runtime = await mujoco();
  const sim = new VectorSim(runtime, {
    xml: dockXml, values: model.values, derived: model.derived, camera: model.camera,
    startDocked: true, noiseSeed: 1,
  });
  try {
    assert.equal(sim.driveOffCharger(), true);
    for (let i = 0; i < 1500 && sim.actionRunning; i++) sim.tick(1);
    assert.equal(sim.lastAction.result, "completed");
    assert.equal(sim.chargerContact, false);

    assert.equal(sim.explore(2.5), true);
    for (let i = 0; i < 2000 && sim.actionRunning; i++) sim.tick(1);
    assert.equal(sim.lastAction.result, "completed");
    assert.ok(sim.lastAction.cells_mapped >= 35);

    assert.equal(sim.dockWithCharger(), true);
    for (let i = 0; i < 14000 && sim.actionRunning; i++) sim.tick(1);
    assert.equal(sim.lastAction.result, "completed");
    assert.equal(sim.chargerContact, true);
    assert.equal(sim.batteryVolts, 4.2);
  } finally {
    sim.dispose();
  }

  const direct = new VectorSim(runtime, {
    xml: dockXml, values: model.values, derived: model.derived, camera: model.camera,
    startDocked: true, noiseSeed: 1,
  });
  try {
    assert.equal(direct.driveOffCharger(), true);
    for (let i = 0; i < 1500 && direct.actionRunning; i++) direct.tick(1);
    assert.equal(direct.lastAction.result, "completed");
    assert.equal(direct.dockWithCharger(), true);
    for (let i = 0; i < 14000 && direct.actionRunning; i++) direct.tick(1);
    assert.equal(direct.lastAction.result, "completed");
    assert.equal(direct.chargerContact, true);
    assert.equal(direct.batteryVolts, 4.2);
  } finally {
    direct.dispose();
  }

  const diag = new VectorSim(runtime, {
    xml: dockXml, values: model.values, derived: model.derived, camera: model.camera,
    startXyMm: [120, 80], headingDeg: 45, noiseSeed: 1,
  });
  try {
    assert.equal(diag.dockWithCharger(), true);
    let checkedLocalStop = false;
    for (let i = 0; i < 14000 && diag.actionRunning; i++) {
      diag.tick(1);
      if (!checkedLocalStop && diag.chargerContact && diag._action?.autonomy?.stage === "verify") {
        assert.ok(diag._delay.every(slot => slot[0] === 0 && slot[1] === 0));
        checkedLocalStop = true;
      }
    }
    assert.ok(checkedLocalStop, "expected a local wheel stop during charger verification");
    assert.equal(diag.lastAction.result, "completed");
    assert.equal(diag.chargerContact, true);
    assert.equal(diag.batteryVolts, 4.2);
    const dock = diag._autonomy.navMap.chargerPose;
    const target = [dock.lipX - 30 * dock.outX, dock.lipY - 30 * dock.outY];
    assert.ok(Math.hypot(diag.state.x_mm - target[0], diag.state.y_mm - target[1]) < 0.1);
    assert.ok(Math.abs(diag.state.pose_angle_rad - Math.atan2(dock.outY, dock.outX)) < 0.04);
  } finally {
    diag.dispose();
  }
});
