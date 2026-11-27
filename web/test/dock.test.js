import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { join } from "node:path";
import { test } from "node:test";
import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { assembleCubeRigObj } from "../src/game/cube.js";
import {
  CHARGER_FROM_MAYA_M,
  HOME_MARKER_POLYGONS,
  assembleDockObj,
  drawHomeMarker,
  makeDockDecals,
  repairDockNormals,
} from "../src/game/dock.js";
import { buildVisuals } from "../src/game/scene.js";
import { VectorSim } from "../src/sim/robot.js";
import { ROOT, mujoco, readJson, readText, sha256 } from "./helpers.js";

function withMockCanvas(fn) {
  const prev = globalThis.document;
  const noop = () => {};
  const ctx = {
    fillStyle: "",
    strokeStyle: "",
    lineWidth: 1,
    shadowColor: "",
    shadowBlur: 0,
    font: "",
    fillRect: noop,
    translate: noop,
    scale: noop,
    rotate: noop,
    save: noop,
    restore: noop,
    beginPath: noop,
    roundRect: noop,
    moveTo: noop,
    lineTo: noop,
    quadraticCurveTo: noop,
    stroke: noop,
    closePath: noop,
    fill: noop,
    fillText: noop,
    createRadialGradient: () => ({ addColorStop: noop }),
  };
  globalThis.document = { createElement: () => ({ width: 0, height: 0, getContext: () => ctx }) };
  try {
    return fn();
  } finally {
    if (prev === undefined) delete globalThis.document;
    else globalThis.document = prev;
  }
}

function assertCleanMesh(mesh, label) {
  const g = mesh.geometry;
  const pos = g.attributes.position;
  const nor = g.attributes.normal;
  const idx = g.index;
  assert.ok(pos && nor, `${label}: missing position or normal attribute`);
  const triCount = idx ? idx.count / 3 : pos.count / 3;
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
  const e1 = new THREE.Vector3(), e2 = new THREE.Vector3(), fn = new THREE.Vector3(), vn = new THREE.Vector3();
  let inverted = 0, degenerate = 0, zeroNormals = 0, nonUnit = 0;
  for (let t = 0; t < triCount; t++) {
    const i0 = idx ? idx.getX(t * 3) : t * 3;
    const i1 = idx ? idx.getX(t * 3 + 1) : t * 3 + 1;
    const i2 = idx ? idx.getX(t * 3 + 2) : t * 3 + 2;
    a.fromBufferAttribute(pos, i0); b.fromBufferAttribute(pos, i1); c.fromBufferAttribute(pos, i2);
    fn.crossVectors(e1.subVectors(b, a), e2.subVectors(c, a));
    const area = fn.length();
    if (area <= 1e-10) {
      degenerate += 1;
      continue;
    }
    fn.divideScalar(area);
    for (const k of [i0, i1, i2]) {
      vn.fromBufferAttribute(nor, k);
      const len = vn.length();
      if (len < 1e-4) zeroNormals += 1;
      else {
        if (Math.abs(len - 1) > 1e-3) nonUnit += 1;
        if (vn.dot(fn) / len < -1e-3) inverted += 1;
      }
    }
  }
  assert.equal(inverted, 0, `${label}: inverted=${inverted}`);
  assert.equal(degenerate, 0, `${label}: degenerate=${degenerate}`);
  assert.equal(zeroNormals, 0, `${label}: zeroNormals=${zeroNormals}`);
  assert.equal(nonUnit, 0, `${label}: nonUnit=${nonUnit}`);
}

test("CHARGER_FROM_MAYA_M, Home marker contours, drawHomeMarker, and dock decal options", () => {
  const upper3 = new THREE.Matrix3().setFromMatrix4(CHARGER_FROM_MAYA_M);
  assert.ok(Math.abs(upper3.determinant() - 1e-9) < 1e-15);
  const mayaPinLeft = new THREE.Vector3(-9.85, 13.1124, -21.5301).applyMatrix4(CHARGER_FROM_MAYA_M);
  assert.ok(Math.abs(mayaPinLeft.x - 0.05971) < 1e-6);
  assert.ok(Math.abs(mayaPinLeft.y - (-0.00985)) < 1e-6);
  assert.ok(Math.abs(mayaPinLeft.z - 0.013671) < 1e-6);

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

  const repaired = repairDockNormals(new THREE.BoxGeometry(1, 1, 1));
  assertCleanMesh(new THREE.Mesh(repaired), "repaired-dock-box");
});

test("buildVisuals hides cube/dock overlays on !showShapes and drives sculpted cube LEDs", async () => {
  const { camera, values, derived } = readJson("public/robot/vector2/model.json");
  const xml = readText("public/robot/vector2/vector-contact42-free-shelf.xml");
  const runtime = await mujoco();
  const sim = new VectorSim(runtime, { xml, values, derived });
  try {
    const scene = new THREE.Scene();
    const fakeCubeObj = ["phong1SG", "phong2SG", "Block1_lightA_mat1SG", "Block1_lightC_matSG", "Block1_lightD_matSG"]
      .map((tag) => `o c_${tag}\nusemtl ${tag}\nv 0 0 0\nv 0.01 0 0\nv 0 0.01 0\nv 0 0 0.01\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4`)
      .join("\n");
    const cubeRig = assembleCubeRigObj(fakeCubeObj, 44, { includeMarks: false });
    const visuals = withMockCanvas(() => buildVisuals(runtime, sim, scene, null, camera, cubeRig, null, new THREE.Group()));
    const dockTower = scene.getObjectByName("dock_tower");
    const cubeCorner = scene.getObjectByName("cube_rubber_0");
    assert.ok(dockTower && cubeCorner, "dock_tower and cube_rubber_0 geoms should exist");
    visuals.sync(sim.simTruth(), "calm", 1, false, false, false);
    assert.equal(dockTower.visible, false);
    assert.equal(cubeCorner.visible, false);
    visuals.sync(sim.simTruth(), "calm", 1, false, true, false);
    assert.equal(dockTower.visible, true);
    assert.equal(cubeCorner.visible, true);
    visuals.updateCubeLights([255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 255]);
    assert.equal(cubeRig.userData.leds[0].material.emissive.getHex(), 0xff0000);
    assert.equal(cubeRig.userData.leds[1].material.emissive.getHex(), 0x00ff00);
    assert.equal(cubeRig.userData.leds[2].material.emissive.getHex(), 0x000000);
    assert.equal(cubeRig.userData.leds[3].material.color.getHex(), 0x0000ff);
  } finally {
    sim.dispose();
  }
});

test("vector-free-dock.xml and vector-contact42-free-dock.xml compile with Light Cube + Charger Dock and climb/undock", async () => {
  const { source, values, derived } = readJson("public/robot/model.json");
  const xml = readText("public/robot/vector-free-dock.xml");
  const c42Xml = readText("public/robot/vector-contact42-free-dock.xml");
  assert.equal(sha256(xml), source["vector-free-dock_sha256"]);
  assert.equal(sha256(c42Xml), source["vector-contact42-free-dock_sha256"]);
  const runtime = await mujoco();
  const docked = new VectorSim(runtime, { xml, values, derived, startDocked: true, noiseSeed: 1 });
  try {
    assert.equal(docked.chargerContact, true);
    assert.equal(docked.simTruth().on_charger, true);
    assert.equal(docked.simTruth().on_charger_platform, true);
    assert.equal(docked.simTruth().battery_volts, 4.2);
    assert.ok(docked.cubeSensor);
    assert.equal(docked.driveStraight(120, 80), true);
    for (let i = 0; i < 800 && docked.actionRunning; i++) docked.tick(1);
    assert.equal(docked.actionRunning, false);
    assert.equal(docked.simTruth().on_charger, false);
    assert.equal(docked.simTruth().on_charger_platform, false);
    assert.ok(docked.simTruth().battery_volts < 4.1);
  } finally {
    docked.dispose();
  }

  const idle = new VectorSim(runtime, { xml, values, derived, startDocked: true, noiseSeed: 1 });
  try {
    const x0 = idle.poseMm()[0];
    let connected = true;
    for (let i = 0; i < 3000; i++) {
      idle.tick(1);
      if (!idle.simTruth().on_charger) connected = false;
    }
    assert.equal(connected, true);
    assert.ok(Math.abs(idle.poseMm()[0] - x0) < 0.5, `unexpected 15s idle creep ${idle.poseMm()[0] - x0}`);
  } finally {
    idle.dispose();
  }

  for (const [sceneXml, speedMmps, holdTicks] of [[xml, -100, 1], [c42Xml, -45, 35]]) {
    const backup = new VectorSim(runtime, { xml: sceneXml, values, derived, startXyMm: [-55, 0], headingDeg: 0, liftMm: 45, noiseSeed: 1 });
    try {
      assert.equal(backup.simTruth().on_charger, false);
      backup.setWheelMotors(speedMmps, speedMmps);
      let ticks = 0;
      for (let i = 0; i < 720; i++) {
        backup.tick(1);
        if (backup.simTruth().on_charger) {
          ticks += 1;
          if (ticks >= holdTicks) {
            backup.stopAllMotors();
            break;
          }
        }
      }
      backup.tick(40);
      assert.equal(backup.simTruth().on_charger, true);
      assert.equal(backup.simTruth().battery_volts, 4.2);
    } finally {
      backup.dispose();
    }
  }
});

const HAVE_DOCK = existsSync(join(ROOT, "public/robot/sdk/Vic_charger_hp_geo.obj"));

test("optional Vic_charger_hp_geo.obj assembles cleanly with decals and clean normals", { skip: !HAVE_DOCK }, () => {
  const parsed = new OBJLoader().parse(readText("public/robot/sdk/Vic_charger_hp_geo.obj"));
  const dock = withMockCanvas(() => assembleDockObj(parsed));
  let meshCount = 0;
  dock.traverse((obj) => {
    if (!obj.isMesh || !obj.geometry?.attributes?.position) return;
    assertCleanMesh(obj, `dock/${obj.name || "part"}`);
    meshCount += 1;
  });
  assert.ok(meshCount > 0, "expected meshes in assembled dock");
  assert.ok(dock.getObjectByName("dock_home_marker"));
  assert.equal(dock.getObjectByName("dock_pin_l_visual"), undefined);
});
