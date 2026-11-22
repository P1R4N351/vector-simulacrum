import assert from "node:assert/strict";
import { test } from "node:test";
import { mujoco, readJson, readText } from "./helpers.js";

const runs = readJson("test/fixtures/cube-landing.json");

for (const scene of new Set(runs.map(run => run.scene))) {
  test(`${scene} reproduces native cube orientation and chip acceleration during landing`, async () => {
    const mj = await mujoco();
    const model = mj.MjModel.from_xml_string(readText(`public/robot/${scene}.xml`));
    const id = (type, name) => {
      const index = mj.mj_name2id(model, mj.mjtObj[type].value, name);
      assert.ok(index >= 0, name);
      return index;
    };
    try {
      const q = model.jnt_qposadr[id("mjOBJ_JOINT", "cube_root")];
      const body = id("mjOBJ_BODY", "cube");
      const sensor = model.sensor_adr[id("mjOBJ_SENSOR", "cube_accel")];
      for (const run of runs.filter(row => row.scene === scene)) {
        const data = new mj.MjData(model);
        try {
          data.qpos.set(run.pose, q);
          for (const [time, up, ...accel] of run.trace) {
            mj.mj_forward(model, data);
            const where = `axis ${run.axis} sign ${run.sign} time ${time}`;
            assert.ok(Math.abs(data.time - time) < 1e-10, where);
            assert.ok(Math.abs(data.xmat[9 * body + 8] - up) < 1e-6, where);
            for (let i = 0; i < 3; i++) {
              assert.ok(Math.abs(data.sensordata[sensor + i] - accel[i]) < 0.01, `${where} channel ${i} expected ${accel[i]} got ${data.sensordata[sensor+i]}`);
            }
            mj.mj_step(model, data);
          }
        } finally {
          data.delete();
        }
      }
    } finally {
      model.delete();
    }
  });
}
