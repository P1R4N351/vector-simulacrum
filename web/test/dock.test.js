import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { join } from "node:path";
import { test } from "node:test";
import * as THREE from "three";
import {
  CHARGER_FROM_MAYA_M,
  HOME_MARKER_POLYGONS,
  drawHomeMarker,
  makeDockDecals,
} from "../src/game/dock.js";
import {
  CHARGER_SUPPLY_VOLTS,
  DEFAULT_DOCK_POSE_MM,
  contactState,
  dockedRobotPose,
} from "../src/sim/dock.js";
import { VectorSim } from "../src/sim/robot.js";
import { ROOT, mujoco, readJson, readText, sha256 } from "./helpers.js";

function withMockCanvas(fn) {
  const prev = globalThis.document;
  const noop = () => {};
  const ctx = {
    fillStyle: "",
    fillRect: noop,
    beginPath: noop,
    roundRect: noop,
    moveTo: noop,
    lineTo: noop,
    closePath: noop,
    fill: noop,
  };
  globalThis.document = { createElement: () => ({ width: 0, height: 0, getContext: () => ctx }) };
  try {
    return fn();
  } finally {
    if (prev === undefined) delete globalThis.document;
    else globalThis.document = prev;
  }
}

test("CHARGER_FROM_MAYA_M preserves handedness and maps pin coordinates", () => {
  const upper3 = new THREE.Matrix3().setFromMatrix4(CHARGER_FROM_MAYA_M);
  assert.ok(Math.abs(upper3.determinant() - 1e-9) < 1e-15);
  const mayaPinLeft = new THREE.Vector3(-9.85, 13.1124, -21.5301).applyMatrix4(CHARGER_FROM_MAYA_M);
  assert.ok(Math.abs(mayaPinLeft.x - 0.05971) < 1e-6);
  assert.ok(Math.abs(mayaPinLeft.y - (-0.00985)) < 1e-6);
  assert.ok(Math.abs(mayaPinLeft.z - 0.013671) < 1e-6);
});

test("dockedRobotPose and contactState enforce pin alignment and platform bounds", () => {
  const posed = dockedRobotPose(DEFAULT_DOCK_POSE_MM);
  assert.ok(Math.abs(posed.startXyMm[0] - (-150)) < 1e-12 && Math.abs(posed.startXyMm[1]) < 1e-12);
  assert.equal(posed.headingDeg, 0);
  const dockPos = [-0.12, 0, 0];
  const dockMat = [-1, 0, 0, 0, -1, 0, 0, 0, 1];
  const th = (posed.pitchDeg * Math.PI) / 180;
  const cMat = [Math.cos(th), 0, Math.sin(th), 0, 1, 0, -Math.sin(th), 0, Math.cos(th)];

  const nom = contactState([-0.15, 0, 0.0063], cMat, dockPos, dockMat);
  assert.equal(nom.on_charger, true);
  assert.equal(nom.on_charger_platform, true);

  const lateralShift = contactState([-0.15, 0.008, 0.0063], cMat, dockPos, dockMat);
  assert.equal(lateralShift.on_charger, false);
  assert.equal(lateralShift.on_charger_platform, true);

  const offDock = contactState([0, 0, 0], cMat, dockPos, dockMat);
  assert.equal(offDock.on_charger, false);
  assert.equal(offDock.on_charger_platform, false);

  const invertedMat = [Math.cos(th), 0, Math.sin(th), 0, -1, 0, Math.sin(th), 0, -Math.cos(th)];
  const inverted = contactState([-0.15, 0, 0.0063], invertedMat, dockPos, dockMat);
  assert.equal(inverted.on_charger, false);
  assert.equal(inverted.on_charger_platform, false);
});

test("Home marker contours, canvas draw, and dock decal options", () => {
  assert.ok(HOME_MARKER_POLYGONS.house.length >= 15);
  assert.ok(HOME_MARKER_POLYGONS.chimney.length >= 6);
  assert.ok(HOME_MARKER_POLYGONS.bolt.length >= 6);
  const fills = [];
  let polyMoves = 0;
  const ctx = {
    set fillStyle(v) { fills.push(v); },
    fillRect() {},
    beginPath() {},
    roundRect() {},
    moveTo() { polyMoves += 1; },
    lineTo() {},
    closePath() {},
    fill() {},
  };
  drawHomeMarker(ctx, 256);
  assert.ok(fills.includes("#24262a") && fills.includes("#eef1f4"));
  assert.equal(polyMoves, 3);

  const withPins = withMockCanvas(() => makeDockDecals({ includePins: true }));
  const withoutPins = withMockCanvas(() => makeDockDecals({ includePins: false }));
  assert.equal(withPins.children.length, 5);
  assert.equal(withoutPins.children.length, 3);
  assert.ok(withPins.getObjectByName("dock_home_marker"));
  assert.ok(withPins.getObjectByName("dock_pin_l_visual"));
  assert.equal(withoutPins.getObjectByName("dock_pin_l_visual"), undefined);
});

async function driveUntilCompleted(sim, actionFn, maxTicks = 800) {
  assert.equal(actionFn(sim), true);
  for (let i = 0; i < maxTicks && sim.actionRunning; i++) sim.tick(1);
  assert.equal(sim.actionRunning, false);
}

test("MuJoCo WASM dock settle, undock, self-centering backup, and ToF detection", async () => {
  const { source, values, derived } = readJson("public/robot/model.json");
  const xml = readText("public/robot/vector-contact42-dock.xml");
  assert.equal(sha256(xml), source["vector-contact42-dock_sha256"]);
  const runtime = await mujoco();

  const docked = new VectorSim(runtime, { xml, values, derived, docked: true, noiseSeed: 1 });
  try {
    assert.equal(docked.state.on_charger, true);
    assert.equal(docked.state.on_charger_platform, true);
    assert.equal(docked.simTruth().battery_volts, CHARGER_SUPPLY_VOLTS);
    await driveUntilCompleted(docked, (s) => s.driveStraight(120, 80));
    assert.equal(docked.state.on_charger, false);
    assert.equal(docked.state.on_charger_platform, false);
    assert.ok(docked.simTruth().battery_volts < 4.10);
  } finally {
    docked.dispose();
  }

  const backup = new VectorSim(runtime, {
    xml, values, derived, startXyMm: [-55, 3.5], headingDeg: -3.0, liftMm: 45, noiseSeed: 1,
  });
  try {
    backup.setWheelMotors(-45, -45);
    let contactTicks = 0;
    for (let i = 0; i < 720; i++) {
      backup.tick(1);
      if (backup.state.on_charger) {
        contactTicks += 1;
        if (contactTicks >= 35) {
          backup.stopAllMotors();
          break;
        }
      }
    }
    backup.tick(40);
    assert.equal(backup.state.on_charger, true);
    assert.equal(backup.reactions, 0);
    const hdgDeg = (backup.headingTotalRad() * 180) / Math.PI;
    assert.ok(Math.abs(hdgDeg) < 2.0, `unexpected backup heading ${hdgDeg}`);
    assert.equal(backup.simTruth().battery_volts, CHARGER_SUPPLY_VOLTS);
  } finally {
    backup.dispose();
  }

  const tofSim = new VectorSim(runtime, {
    xml, values, derived, startXyMm: [-30, 0], headingDeg: 180, noiseSeed: 1,
  });
  try {
    tofSim.tick(5);
    assert.equal(tofSim.state.prox_found_object, true);
    assert.ok(tofSim.state.prox_distance_mm >= 150 && tofSim.state.prox_distance_mm <= 195);
  } finally {
    tofSim.dispose();
  }
});

const HAVE_V2_DOCK = existsSync(join(ROOT, "public/robot/vector2/vector-contact42-dock.xml"));

test("Vector 2 dock XML compiles and settles on charger", { skip: !HAVE_V2_DOCK }, async () => {
  const { source, values, derived } = readJson("public/robot/vector2/model.json");
  const xml = readText("public/robot/vector2/vector-contact42-dock.xml");
  assert.equal(sha256(xml), source["vector-contact42-dock_sha256"]);
  const sim = new VectorSim(await mujoco(), { xml, values, derived, docked: true, noiseSeed: 1 });
  try {
    sim.tick(4);
    assert.equal(sim.state.on_charger, true);
    assert.equal(sim.simTruth().battery_volts, CHARGER_SUPPLY_VOLTS);
  } finally {
    sim.dispose();
  }
});
