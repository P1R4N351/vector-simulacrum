import assert from "node:assert/strict";
import { gunzipSync } from "node:zlib";
import { readFileSync } from "node:fs";
import { test } from "node:test";
import { CubeSensor } from "../src/sim/cube_sensor.js";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

const { values, derived } = readJson("public/robot/model.json");
const quietChip = { ...values, cube_accel_noise_density_ug: 0 };

test("stock cube app agrees with the Python CubeAccelerometer on clipped and reversing force", () => {
  const sensor = new CubeSensor(quietChip, .001, 0, [0, 0, 1]);
  const packets = [];
  for (let i = 1; i <= 120; i++) {
    const x = i % 11 - 5;
    sensor.step(i * .001, [x, -x, i < 40 || i > 95 ? 1 : -6]);
    if (i % 30 === 0) {
      const { slots, taps, time_s } = sensor.packet;
      assert.ok(Math.abs(time_s - i * .001) < 1e-9);
      const wire = Buffer.alloc(20);
      wire.set([2, taps]);
      slots.flat().forEach((v, k) => wire.writeInt16LE(v, 2 + 2 * k));
      packets.push(wire.toString("hex"));
    }
  }
  assert.deepEqual(packets, [
    "0200f07f00800020e07920860020e05920a60020",
    "0200e03920c60080e01920e60080e0f920060080",
    "0200e0d92026008000ba00460080c09a40650080",
    "02007081907e00209094706b0020f07f00800020",
  ]);
});

test("the browser free cube publishes three stationary samples and reset clears the packet", async () => {
  const sim = new VectorSim(await mujoco(), {
    xml: readText("public/robot/vector-free.xml"), values: quietChip, derived,
  });
  try {
    sim.tick(10);
    const packet = sim.cubeSensor.packet;
    assert.ok(packet.time_s > 0);
    for (const slot of packet.slots) {
      assert.ok(Math.abs(Math.hypot(...slot) / values.cube_accel_counts_per_g - 1) < .02);
    }
    sim.reset();
    assert.equal(sim.cubeSensor.packet, null);
  } finally {
    sim.dispose();
  }
});

test("cube callbacks carry the last supervisor timestamp across calm and awake periods", async () => {
  const sim = new VectorSim(await mujoco(), {
    xml: readText("public/robot/vector-free.xml"), values: quietChip, derived, calmMode: true,
  });
  function nextCandidate() {
    // Force a counter change without introducing a motion-dependent timing assumption.
    sim.cubeSensor.taps++;
    sim.tick(6);
    return sim.cubeSensor.packet.events.find(event => "tap_count" in event);
  }
  try {
    sim.tick(6);
    assert.equal(nextCandidate().timestamp, 0);
    sim.tick(36);
    const calm = nextCandidate();
    assert.equal(calm.timestamp, 250);
    assert.ok(Math.abs(calm.time_s - .27) < 1e-8);
    sim.calmMode = false;
    assert.equal(nextCandidate().timestamp, 280);
    sim.reset();
    sim.tick(6);
    assert.equal(nextCandidate().timestamp, 30);
  } finally {
    sim.dispose();
  }
});

test("unit 3 awake clock advances independently of the physics control tick", async () => {
  const sim = new VectorSim(await mujoco(), {
    xml: readText("public/robot/vector-free.xml"), values: { ...quietChip, supervisor_loop_ms: 5.08 }, derived,
  });
  try {
    sim.tick(60);
    sim.cubeSensor.taps++;
    sim.tick(6);
    const event = sim.cubeSensor.packet.events.find(candidate => "tap_count" in candidate);
    assert.equal(event.timestamp, 305);
    assert.ok(Math.abs(event.time_s - .33) < 1e-8);
  } finally {
    sim.dispose();
  }
});

test("browser movement broadcasts follow the physical carry-state recordings", () => {
  for (const [run, expectedRaw] of [["R5",4], ["R8",1], ["R10",2], ["R11",2]]) {
    const sensor = new CubeSensor(quietChip, .001, 0, [0,0,1]);
    const directory = new URL("../../evidence/unit3-wheelstand/" + run + "/", import.meta.url);
    function rows(name) {
      const data = readFileSync(new URL(name, directory));
      return (name.endsWith(".gz") ? gunzipSync(data) : data).toString().trim().split(/\r?\n/)
        .slice(1).map(line => line.split(","));
    }
    const states = rows("state.csv.gz");
    const sdk = rows("events.csv").filter(row => ["object_moved","object_stopped_moving"].includes(row[1]));
    let idx = -1, slots = [], raw = 0;
    const broadcasts = [];
    for (const row of rows("cube.csv.gz")) {
      const [t, uptime, slot, taps, ...xyz] = row.map(Number);
      slots.push(xyz);
      if (slot !== 2) continue;
      while (idx+1 < states.length && Number(states[idx+1][0]) <= t) idx++;
      sensor.handlingCube = idx >= 0 && Boolean(Number(states[idx][6]) & 2);
      sensor.receivePacket({time_s:t, taps, slots}, Math.round(uptime * 1000));
      slots = [];
      raw += sensor.packet.events.filter(e => "moving" in e).length;
      broadcasts.push(...sensor.packet.movementBroadcasts);
    }
    assert.equal(raw, expectedRaw, run);
    assert.equal(sensor.movementBroadcastCount, sdk.length, run);
    broadcasts.forEach((e,i) => {
      assert.equal(e.moving, sdk[i][1] === "object_moved");
      assert.ok(Math.abs(e.timestamp - Number(sdk[i][3])) <= 35, run);
    });
  }
});
