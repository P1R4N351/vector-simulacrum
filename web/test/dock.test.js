import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { join } from "node:path";
import { test } from "node:test";
import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
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
import { ROOT, mujoco, readJson, readText } from "./helpers.js";

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

test("buildVisuals hides dock physics box overlay unless showShapes is true", async () => {
  const { camera, values, derived } = readJson("public/robot/vector2/model.json");
  const xml = readText("public/robot/vector2/vector-contact42-free-shelf.xml");
  const runtime = await mujoco();
  const sim = new VectorSim(runtime, { xml, values, derived });
  try {
    const scene = new THREE.Scene();
    const visuals = withMockCanvas(() => buildVisuals(runtime, sim, scene, null, camera, new THREE.Group(), null, new THREE.Group()));
    const dockTower = scene.getObjectByName("dock_tower");
    assert.ok(dockTower, "dock_tower collision geom should exist in shelf scene");
    visuals.sync(sim.simTruth(), "calm", 1, false, false, false);
    assert.equal(dockTower.visible, false);
    visuals.sync(sim.simTruth(), "calm", 1, false, true, false);
    assert.equal(dockTower.visible, true);
  } finally {
    sim.dispose();
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
