import assert from "node:assert/strict";
import { test } from "node:test";
import { motorCommand, WireBody } from "../src/sim/wire_body.js";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

test("upstream v6 motor packet drives the WASM body and v1 free cube returns specific force", async () => {
  const bytes=new Uint8Array(132), cmd=new DataView(bytes.buffer);
  assert.equal(motorCommand(bytes),null);
  cmd.setUint32(0,0x56494342,true); cmd.setUint32(4,6,true);
  cmd.setInt16(24,3000,true); cmd.setInt16(26,-3000,true);
  assert.equal(motorCommand(bytes.subarray(1)),null);
  const profile=readJson("public/robot/vector2/model.json");
  const sim=new VectorSim(await mujoco(),{xml:readText("public/robot/vector2/vector-free.xml"),values:profile.values,derived:profile.derived});
  try {
    const body=new WireBody(sim), power=motorCommand(bytes);
    const first=new DataView(body.frame(power).buffer);
    const frames = body.cubeFrames();
    for(let i=0;i<80;i++) {
      body.frame(power);
      frames.push(...body.cubeFrames());
    }
    const last=new DataView(body.frame(power).buffer);
    assert.equal(last.byteLength,812);
    assert.equal(last.getUint32(16,true),81);
    assert.ok(last.getInt32(24,true)>first.getInt32(24,true));
    assert.ok(last.getInt32(36,true)<first.getInt32(36,true));
    assert.ok(Number.isFinite(last.getFloat32(804,true)));
    assert.equal(frames.filter(bytes => bytes[8] === 1).length, 1);
    const accel = frames.filter(bytes => bytes[8] === 2);
    assert.equal(accel.length, 40);
    const cube=new DataView(accel.at(-1).buffer);
    assert.equal(cube.getUint8(8),2);
    assert.equal(cube.getInt16(34,true),sim.cubeSensor.slots[0][2]);
    const expectedZ = profile.values.cube_accel_counts_per_g * profile.values.cube_accel_scale[2];
    assert.ok(Math.abs(cube.getInt16(34,true)-expectedZ)<150);
  } finally { sim.dispose(); }
});

test("WireOS receives held raw ranges and scene-dependent signal per SPAD", async () => {
  const profile = readJson("public/robot/vector2/model.json");
  const sim = new VectorSim(await mujoco(), {
    xml: readText("public/robot/vector2/vector-free.xml"), values: profile.values, derived: profile.derived,
  });
  try {
    const body = new WireBody(sim);
    const frame = () => new DataView(body.frame([0, 0, 0, 0]).buffer);
    let hit = -1;
    sim._castTof = () => sim._proxSensor.rays.map(() => hit);
    sim._rangeSample = null;
    const weak = frame();
    const quality = packet => (packet.getUint16(96) / 128) / (packet.getUint16(100) / 256);
    assert.ok(quality(weak) < sim.v.tof_quality_min);
    hit = 100;
    const held = frame();
    assert.equal(held.getUint16(102, true), weak.getUint16(102, true));
    assert.equal(held.getUint16(94), weak.getUint16(94));
    let near = held;
    for (let i = 0; i < 15; i++) near = frame();
    assert.ok(near.getUint16(102, true) > held.getUint16(102, true));
    assert.ok(Math.abs(quality(near) - sim.v.tof_quality_at_100mm) < 0.0001);
    assert.ok(Math.abs(near.getUint16(94) - 100) < 15);
  } finally { sim.dispose(); }
});

test("shelf charger supply follows both pads through seating and displacement", async () => {
  const profile = readJson("public/robot/vector2/model.json");
  const mj = await mujoco();
  const sim = new VectorSim(mj, { xml: readText("public/robot/vector2/vector-free-shelf.xml"),
    values: profile.values, derived: profile.derived, startDocked: true });
  try {
    const body = new WireBody(sim);
    assert.equal(new DataView(body.frame([0, 0, 0, 0]).buffer).getUint16(86, true), 3);
    const readAt = (offsetY, height) => {
      const halfPitch = 8.2 * Math.PI / 360;
      sim.data.qpos.set([-0.096, offsetY, height, Math.cos(halfPitch), 0, Math.sin(halfPitch), 0]);
      sim.data.qvel.fill(0);
      mj.mj_forward(sim.model, sim.data);
      return new DataView(body.frame([0, 0, 0, 0]).buffer);
    };
    const seated = readAt(0, 0.0063);
    assert.ok(seated.getInt16(82, true) > 0);
    assert.equal(seated.getUint16(86, true), 3);
    assert.equal(readAt(0.012, 0.0063).getInt16(82, true), 0);
    assert.equal(readAt(0, 0.030).getUint16(86, true), 0);
    assert.equal(readAt(0, 0.0063).getUint16(86, true), 3);
    const onCharger = power => new DataView(body.frame(power).buffer).getUint16(86, true);
    for (let i = 0; i < 200; i++) assert.equal(onCharger([0, 0, 0, 0]), 3);
    let releases = 0, previous = 3;
    for (let i = 0; i < 160; i++) {
      const current = onCharger([0.45, -0.45, 0, 0]);
      if (current !== previous) releases++;
      previous = current;
    }
    assert.equal(previous, 0);
    assert.equal(releases, 1);
  } finally { sim.dispose(); }
});

test("cube v1 preserves physical slot ordering and a wrapping firmware tap count", () => {
  const sensor = {ticks: 0, slots: [[-731,202,8001],[12,85,7909],[4,1,8157]], packet: null};
  const body = new WireBody({cubeSensor: sensor});
  body.seq = 2;
  body.cubeTaps = 255;
  sensor.packet = {taps: 0};
  for (let i = 0; i < 6; i++) {
    sensor.ticks++;
    const [bytes] = body.cubeFrames();
    const view = new DataView(bytes.buffer);
    assert.deepEqual([30,32,34].map(offset => view.getInt16(offset,true)), sensor.slots[i%3]);
    assert.equal(bytes[36], Number(i === 2));
    assert.equal(view.getUint16(38,true),426);
    assert.deepEqual(body.cubeFrames(), []);
  }
});

test("shelf cube advertises and reports the measured rail ADC word", () => {
  const sensor = {ticks: 0, slots: [[0,0,8192],[0,0,8192],[0,0,8192]], packet: null};
  const body = new WireBody({cubeSensor: sensor, shelfOutline: [[0,0],[200,0],[0,200]]});
  body.seq = 1;
  sensor.ticks = 1;
  const frames = body.cubeFrames();
  assert.deepEqual(frames.map(bytes => bytes[8]), [1,2]);
  for (const bytes of frames) assert.equal(new DataView(bytes.buffer).getUint16(38,true),444);
});
