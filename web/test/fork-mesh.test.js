import assert from "node:assert/strict";
import test from "node:test";
import * as THREE from "three";
import { buildVisuals } from "../src/game/scene.js";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

test("collision overlay draws compiled mesh vertices at their authored wrist positions", async () => {
  const mj = await mujoco();
  const { values, derived, camera } = readJson("public/robot/vector2/model.json");
  const points = [[0, 0, 0], [0.004, 0, 0], [0, 0.007, 0], [0, 0, 0.012]];
  const xml = readText("public/robot/vector2/vector.xml")
    .replace("</mujoco>", `<asset><mesh name="probe" vertex="${points.flat().join(" ")}"/></asset></mujoco>`)
    .replace(/(<body name="lift_fork"[^>]*>)/,
      '$1<geom name="probe" type="mesh" mesh="probe" pos="0.002 0 -0.019" mass="0"/>');
  const sim = new VectorSim(mj, { xml, values, derived });
  try {
    const scene = new THREE.Scene();
    const sdk = { root: new THREE.Group(), head: new THREE.Group(), setPose() {} };
    const visuals = buildVisuals(mj, sim, scene, sdk, camera);
    const mesh = scene.getObjectByName("probe");
    const wrist = mj.mj_name2id(sim.model, mj.mjtObj.mjOBJ_BODY.value, "lift_fork");
    for (const height of [values.lift_height_min_mm, values.lift_height_max_mm]) {
      sim.setLiftHeight(height);
      sim.tick(200);
      visuals.sync(sim.simTruth(), "calm", 1, false, true, false);
      const pos = sim.data.xpos.subarray(3 * wrist, 3 * wrist + 3);
      const [w, x, y, z] = sim.data.xquat.subarray(4 * wrist, 4 * wrist + 4);
      const rotation = new THREE.Quaternion(x, y, z, w);
      const vertices = mesh.geometry.attributes.position;
      const drawn = Array.from({ length: vertices.count }, (_, i) =>
        new THREE.Vector3().fromBufferAttribute(vertices, i).applyMatrix4(mesh.matrix));
      assert.equal(mesh.geometry.index.count, 12);
      for (const [px, py, pz] of points) {
        const expected = new THREE.Vector3(px + 0.002, py, pz - 0.019)
          .applyQuaternion(rotation).add(new THREE.Vector3(...pos));
        assert.ok(drawn.some(vertex => vertex.distanceTo(expected) < 1e-7));
      }
    }
  } finally {
    sim.dispose();
  }
});
