import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";
import { addFaceCarrier, visorGeometry } from "../src/game/face.js";
import { readJson } from "./helpers.js";

test("the installed cover clears the camera barrel, bracket and display hardware", () => {
  const head = new THREE.Mesh(new THREE.BufferGeometry());
  // The front axle lies 2.95 mm ahead of the SDK mesh origin.
  addFaceCarrier(head, readJson("public/robot/model.json").values, 0.295);
  const carrier = head.children.find(part => part.isGroup);
  const cover = new THREE.Mesh(visorGeometry());
  head.updateMatrixWorld(true);
  cover.updateMatrixWorld(true);
  const ray = new THREE.Raycaster();
  ray.ray.direction.set(-1, 0, 0);
  let samples = 0;
  for (let iz = 0; iz <= 42; iz++) {
    for (let iy = -22; iy <= 22; iy++) {
      ray.ray.origin.set(4, iy * 0.065, 3.6 + iz * 0.065);
      const hardware = ray.intersectObject(carrier, true)[0];
      if (!hardware) continue;
      const surface = ray.intersectObject(cover)[0];
      assert.ok(surface && surface.distance < hardware.distance, "hardware crosses the cover");
      samples++;
    }
  }
  assert.ok(samples > 300, "probe includes the populated face");
});

test("the installed faceplate continues the shell tangent around its entire edge", () => {
  const head = new THREE.Mesh(new THREE.BufferGeometry());
  addFaceCarrier(head, readJson("public/robot/model.json").values, 0.295);
  const cover = visorGeometry();
  const shell = head.geometry;
  const positions = shell.attributes.position;
  const byPosition = new Map();
  const vector = (attribute, i) => new THREE.Vector3().fromBufferAttribute(attribute, i);
  for (let i = 0; i < positions.count; i++) {
    const key = vector(positions, i).toArray().join(",");
    // The first occurrence is the exterior. The cut edge has its own normal.
    if (!byPosition.has(key)) byPosition.set(key, i);
  }
  const slices = 96;
  const glass = cover.attributes.position;
  for (let i = glass.count - slices; i < glass.count; i++) {
    const edge = vector(glass, i);
    const j = byPosition.get(edge.toArray().join(","));
    assert.notEqual(j, undefined, "cover and shell must share an edge vertex");
    const shellTangent = edge.clone().sub(vector(positions, j - slices)).normalize();
    const coverTangent = vector(glass, i - slices).sub(edge).normalize();
    assert.ok(shellTangent.dot(coverTangent) > Math.cos(3 * Math.PI / 180),
      "adjacent surface segments must continue in the same direction");
    assert.ok(vector(shell.attributes.normal, j).dot(vector(cover.attributes.normal, i)) > 0.99999,
      "the seam must have a continuous shading normal");
  }
});
