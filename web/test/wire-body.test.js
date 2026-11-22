import assert from "node:assert/strict";
import { test } from "node:test";
import { motorCommand, WireBody, cubeFrame } from "../src/sim/wire_body.js";
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
    for(let i=0;i<80;i++) body.frame(power);
    const last=new DataView(body.frame(power).buffer);
    assert.equal(last.byteLength,812);
    assert.equal(last.getUint32(16,true),81);
    assert.ok(last.getInt32(24,true)>first.getInt32(24,true));
    assert.ok(last.getInt32(36,true)<first.getInt32(36,true));
    assert.ok(Number.isFinite(last.getFloat32(804,true)));
    const cube=new DataView(cubeFrame(sim,false,true).buffer);
    assert.equal(cube.getUint8(8),2);
    assert.equal(cube.getUint8(36),1);
    assert.ok(Math.abs(cube.getInt16(34,true)-8192)<150);
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
