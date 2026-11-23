// Where the robot's camera is and where it looks, against the robot's own geometry..
//
// Expected values are worked out here from the table (neck joint, camera
// offset from the neck) and the tilt, independently of the code under test,
// which reads the camera's pose from the physics solver.
import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";
import { makeHeadCamera } from "../src/game/headcam.js";
import { pixelOf } from "../src/sim/optics.js";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

const model = readJson("public/robot/model.json");
const xml = readText("public/robot/vector.xml");
const DEG = Math.PI / 180;

async function rig(headDeg, sceneXml = xml, spec = model.camera) {
  const mj = await mujoco();
  const sim = new VectorSim(mj, { xml: sceneXml, values: model.values, derived: model.derived, headDeg });
  const parent = new THREE.Group(); // physics coordinates, unrotated
  const head = makeHeadCamera(mj, sim, spec, parent);
  head.update(sim.data);
  parent.updateMatrixWorld(true);
  return { sim, camera: head.camera };
}

function expectedPosition(headDeg) {
  const v = model.values;
  const [nx, , nz] = v.neck_joint_position_mm;
  const [a, , b] = v.head_camera_position_mm;
  const t = headDeg * DEG; // head up is positive: forward tips up, up tips back
  return [nx + a * Math.cos(t) - b * Math.sin(t), 0, nz + a * Math.sin(t) + b * Math.cos(t)];
}

for (const headDeg of [0, 30, -20]) {
  test(`camera position and direction with the head at ${headDeg} degrees`, async () => {
    const { sim, camera } = await rig(headDeg);
    const e = camera.matrixWorld.elements;
    const position = [e[12] * 1000, e[13] * 1000, e[14] * 1000];
    const want = expectedPosition(headDeg);
    for (let i = 0; i < 3; i++) assert.ok(Math.abs(position[i] - want[i]) < 0.2, `axis ${i}: ${position[i]} against ${want[i]} mm`);
    const forward = [-e[8], -e[9], -e[10]]; // a camera looks along its own -z
    const pitch = Math.asin(forward[2]) / DEG;
    const actualHead = sim.headAngleRad() / DEG;
    assert.ok(Math.abs(pitch - (actualHead - model.camera.pitch_down_deg)) < 0.1, `pitch ${pitch} with the head at ${actualHead}`);
    assert.ok(Math.abs(forward[1]) < 0.002 && forward[0] > 0, "looks straight ahead, not sideways");
    const right = [e[0], e[1], e[2]];
    assert.ok(right[1] < -0.999, "the image's right is the robot's right (-y)");
    sim.dispose();
  });
}

test("the tilt is what moves the horizon: a far point at camera height lands above the optical centre", async () => {
  const { sim, camera } = await rig(0);
  const toCamera = camera.matrixWorld.clone().invert();
  const height = camera.matrixWorld.elements[14];
  const far = new THREE.Vector3(5, 0, height).applyMatrix4(toCamera);
  const pixel = pixelOf(model.camera, [far.x, far.y, far.z]);
  const rise = model.camera.fy_px * Math.tan(model.camera.pitch_down_deg * DEG);
  assert.ok(Math.abs(model.camera.cy_px - pixel[1] - rise) < 1.5, `horizon ${model.camera.cy_px - pixel[1]} px above centre, expected ${rise}`);
  assert.ok(Math.abs(pixel[0] - model.camera.cx_px) < 1, "and stays on the optical centre's column");
  sim.dispose();
});


test("older level-camera models receive exactly one mount tilt", async () => {
  const levelXml = xml.replace(/(<camera name="head_cam"[^>]*xyaxes=")[^"]+/, (_, prefix) => prefix + "0 -1 0 0 0 1");
  const { sim, camera } = await rig(0, levelXml, { ...model.camera, pitch_down_in_model: false });
  const pitch = Math.asin(-camera.matrixWorld.elements[10]) / DEG;
  assert.ok(Math.abs(pitch - (sim.headAngleRad() / DEG - model.camera.pitch_down_deg)) < 0.1);
  sim.dispose();
});

test("unit 3 RGB view applies its measured pitch relative to the native camera", async () => {
  const spec = readJson("public/robot/vector2/model.json").camera;
  const { sim, camera } = await rig(-12, readText("public/robot/vector2/vector.xml"), spec);
  try {
    const pitch = Math.asin(-camera.matrixWorld.elements[10]) / DEG;
    const correction = readJson("camera_calibration_vector2.json").rgb_pitch_up_deg;
    assert.ok(Math.abs(pitch - (sim.headAngleRad() / DEG - spec.pitch_down_deg + correction)) < 0.1);
  } finally {
    sim.dispose();
  }
});
