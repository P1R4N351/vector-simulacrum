import assert from "node:assert/strict";
import { test } from "node:test";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText, sha256 } from "./helpers.js";

test("all Vector 2 scenes use the unit 3 camera and compile with the variant controller", async () => {
  const { camera, source, values, derived } = readJson("public/robot/vector2/model.json");
  const calibration = readJson("camera_calibration_vector2.json");
  for (const key of ["width_px", "height_px", "fx_px", "fy_px", "cx_px", "cy_px"]) {
    assert.equal(camera[key], calibration[key], key);
  }
  const runtime = await mujoco();
  for (const [scene, key] of [
    ["vector", "xml_sha256"], ["vector-contact42", "trial_xml_sha256"],
    ["vector-free", "vector-free_sha256"], ["vector-contact42-free", "vector-contact42-free_sha256"],
  ]) {
    const xml = readText(`public/robot/vector2/${scene}.xml`);
    assert.equal(sha256(xml), source[key]);
    const sim = new VectorSim(runtime, { xml, values, derived });
    try {
      sim.tick(12);
      if (scene.endsWith("-free")) {
        assert.ok(sim.state.prox_found_object);
        assert.equal(sim.cubeSensor.packet.slots.length, 3);
      } else {
        assert.equal(sim.cubeSensor, null);
      }
    } finally {
      sim.dispose();
    }
  }
});
