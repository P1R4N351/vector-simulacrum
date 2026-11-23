// Browser Float32 NoMaD parity, goal-mask invariance, topological graph, camera, and closed-loop tests.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { test } from "node:test";
import {
  HORIZON,
  ACT_DIM,
  NomadPolicy,
  TopologicalGraph,
  selectSafeTrajectory,
  stackObsFrames,
  waypointToWheels,
} from "../src/sim/nomad.js";
import { IMG_H, IMG_W, renderHeadCamRgb } from "../src/sim/nomad_cam.js";
import { VectorSim } from "../src/sim/robot.js";
import { ROOT, mujoco, readJson, readText } from "./helpers.js";

const model = readJson("public/robot/model.json");
const dockXml = readText("public/robot/vector-free-dock.xml");

function loadNomadArtifacts() {
  const meta = readJson("public/robot/nomad/nomad.json");
  const weights = new Uint8Array(readFileSync(join(ROOT, "public/robot/nomad/nomad_weights.bin")));
  const parity = readJson("test/fixtures/nomad_parity.json");
  return { policy: NomadPolicy.fromBuffers(meta, weights), parity };
}

function maxAbsDiff(a, b) {
  assert.equal(a.length, b.length, "array length mismatch");
  let best = 0;
  for (let i = 0; i < a.length; i++) {
    const d = Math.abs(a[i] - b[i]);
    if (d > best) best = d;
  }
  return best;
}

function fakeObs(seed) {
  const out = new Uint8Array(IMG_H * IMG_W * 3);
  let s = seed + 1;
  for (let i = 0; i < out.length; i++) {
    s = (1664525 * s + 1013904223) >>> 0;
    out[i] = (s >>> 24) & 0xff;
  }
  return out;
}

test("Float32 JS policy matches PyTorch/ONNX parity (<1e-4) and enforces goal masking", () => {
  const { policy, parity } = loadNomadArtifacts();
  const obs = Float32Array.from(parity.obs_stack, (x) => x / 255);
  const goal = Float32Array.from(parity.goal_img, (x) => x / 255);
  const altGoal = Float32Array.from(parity.alt_goal_img, (x) => x / 255);

  const enc0 = policy.encode(obs, goal, 0);
  assert.ok(maxAbsDiff(enc0.context, parity.context_unmasked) < 1e-4, "context parity failed");
  assert.ok(Math.abs(enc0.predDist[0] - parity.pred_dist_unmasked) < 1e-4, "predDist parity failed");

  const initNoise0 = Float32Array.from(parity.init_noise.slice(0, HORIZON * ACT_DIM));
  const noisePred = policy.denoise(initNoise0, enc0.context, 15);
  assert.ok(maxAbsDiff(noisePred, parity.pred_noise_t15) < 1e-4, "denoise parity failed");

  const trajs = policy.ddimSample(enc0.context, {
    numSamples: 2,
    initNoise: Float32Array.from(parity.init_noise),
  });
  assert.ok(maxAbsDiff(trajs.flat(2), parity.ddim_trajectories.flat(2)) < 1e-4, "ddim trajectories parity failed");

  const maskedA = policy.encode(obs, goal, 1);
  const maskedB = policy.encode(obs, altGoal, 1);
  assert.ok(maxAbsDiff(maskedA.context, maskedB.context) < 1e-6, "masked context leaked goal");
  assert.ok(Math.abs(maskedA.predDist[0] - maskedB.predDist[0]) < 1e-6, "masked dist leaked goal");
});

test("TopologicalGraph routes lowest-weight multi-hop Dijkstra paths and subgoals", () => {
  const graph = new TopologicalGraph();
  const metricDist = (obsBatch, goalBatch, n) => {
    const out = new Float32Array(n);
    const frameStride = 3 * IMG_H * IMG_W;
    const obsTailOff = 3 * frameStride;
    for (let i = 0; i < n; i++) {
      let sum = 0;
      for (let k = 0; k < frameStride; k++) {
        sum += Math.abs(obsBatch[i * 4 * frameStride + obsTailOff + k] - goalBatch[i * frameStride + k]);
      }
      out[i] = (sum / frameStride) * 18;
    }
    return out;
  };
  const [f0, f1, f2] = [fakeObs(1), fakeObs(2), fakeObs(3)];
  assert.equal(graph.recordStep(stackObsFrames([f0]), f0, [0, 0, 0], metricDist, 0, "n0"), 0);
  assert.equal(graph.recordStep(stackObsFrames([f1]), f1, [100, 0, 0], metricDist, 25, "n1"), 1);
  assert.equal(graph.recordStep(stackObsFrames([f2]), f2, [200, 0, 0], metricDist, 50, "n2"), 2);
  graph.edges.set(0, new Map());
  graph.edges.set(1, new Map());
  graph.edges.set(2, new Map());
  graph.addEdge(0, 1, 1.5);
  graph.addEdge(1, 2, 1.5);
  graph.addEdge(0, 2, 10.0);
  assert.deepEqual(graph.shortestPath(0, 2), [0, 1, 2]);
  const obs0 = stackObsFrames([f0]);
  const farSub = graph.chooseSubgoal(obs0, 2, () => new Float32Array([0.1, 3.0, 6.0]));
  assert.equal(farSub.subId, 1);
  const nearSub = graph.chooseSubgoal(obs0, 2, () => new Float32Array([0.1, 2.0, 1.2]));
  assert.equal(nearSub.subId, 2);
  assert.equal(graph.snapshot().edges.length, 3);
});

test("selectSafeTrajectory and waypointToWheels enforce table, cliff, and prox safety", () => {
  const v = model.values;
  const fwdTraj = Array.from({ length: HORIZON }, (_, i) => [15 * (i + 1), 0]);
  const safeTurn = Array.from({ length: HORIZON }, (_, i) => [-12 * (i + 1), 45 * (i + 1)]);
  const cliffRes = selectSafeTrajectory(
    [fwdTraj, safeTurn, fwdTraj, fwdTraj],
    { x_mm: 0, y_mm: 0, pose_angle_rad: 0, prox_found_object: false, prox_distance_mm: 400, cliff_detected: true },
    v,
  );
  assert.equal(cliffRes.index, 1);
  assert.deepEqual(waypointToWheels(fwdTraj, v, true, false, 400), [0, 0]);
  assert.deepEqual(waypointToWheels(fwdTraj, v, false, true, 60), [0, 0]);
  const [l, r] = waypointToWheels(safeTurn, v, false, false, 400);
  assert.ok(l < r);
});

test("nomad_cam renders cube vs away views, occludes lift, and VectorSim executes closed-loop NoMaD", async () => {
  const runtime = await mujoco();
  const { policy } = loadNomadArtifacts();

  const cubeSim = new VectorSim(runtime, {
    xml: dockXml, values: model.values, derived: model.derived, camera: model.camera,
    startXyMm: [-30, 0], headingDeg: 0, headDeg: 4, liftMm: 32, noiseSeed: 1,
  });
  const awaySim = new VectorSim(runtime, {
    xml: dockXml, values: model.values, derived: model.derived, camera: model.camera,
    startXyMm: [160, 120], headingDeg: 0, headDeg: 4, liftMm: 32, noiseSeed: 1,
  });
  const liftSim = new VectorSim(runtime, {
    xml: dockXml, values: model.values, derived: model.derived, camera: model.camera,
    startXyMm: [-30, 0], headingDeg: 0, headDeg: 4, liftMm: 50, noiseSeed: 1,
  });
  try {
    cubeSim.tick(2);
    awaySim.tick(2);
    liftSim.tick(2);
    const imgCube = renderHeadCamRgb(cubeSim);
    const imgAway = renderHeadCamRgb(awaySim);
    const imgLift = renderHeadCamRgb(liftSim);
    let diff = 0;
    for (let i = 0; i < imgCube.length; i++) diff += Math.abs(imgCube[i] - imgAway[i]);
    assert.ok(diff / imgCube.length > 1.5, "cube view should differ from empty view");
    const rowOff = (IMG_H - 3) * IMG_W + 20;
    assert.deepEqual(Array.from(imgLift.subarray(3 * rowOff, 3 * rowOff + 3)), [52, 56, 64]);

    cubeSim.attachNomad(policy, model.camera);
    assert.equal(cubeSim.simTruth().nomad_ready, true);
    const gid = cubeSim.nomadCaptureGoal("start");
    assert.equal(gid, 0);
    assert.equal(cubeSim.nomadExplore(2.5), true);
    for (let i = 0; i < 1200 && cubeSim.actionRunning; i++) cubeSim.tick(1);
    assert.equal(cubeSim.lastAction.result, "completed");
    assert.equal(cubeSim.simTruth().fallen, false);
    assert.ok(cubeSim.nomadSnapshot().nodes.length >= 2);

    assert.equal(cubeSim.nomadNavigate(3.0, gid), true);
    for (let i = 0; i < 1500 && cubeSim.actionRunning; i++) cubeSim.tick(1);
    assert.ok(["completed", "timeout"].includes(cubeSim.lastAction.result));
    assert.equal(cubeSim.simTruth().fallen, false);

    cubeSim.reset({ startXyMm: [0, 0], headingDeg: 0, noiseSeed: 1 });
    assert.equal(cubeSim.nomadSnapshot().nodes.length, 0);
  } finally {
    cubeSim.dispose();
    awaySim.dispose();
    liftSim.dispose();
  }
});
