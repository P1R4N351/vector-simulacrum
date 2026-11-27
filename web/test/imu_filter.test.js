// Port of ../simulacrum/../tests/test_imu_filter.py plus accel_scale check..
import assert from "node:assert/strict";
import { test } from "node:test";
import { CHANNELS, ImuFilter, cornersHz } from "../src/sim/imu_filter.js";
import { CONTROL_DEFAULT, VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

const model = readJson("public/robot/model.json");
const xml = readText("public/robot/vector.xml");
const TABLE = model.values;
const HALF_POWER = Math.SQRT1_2;
const PASS_THROUGH = { accel_filter_cutoff_hz: 1e6, gyro_filter_cutoff_hz: 1e6, imu_filter_second_pole: 0 };
const QUIET = { gyro_noise_rms_dps: 0, accel_noise_rms_mg: 0 };
const ACCEL_X = 0;
const GYRO_X = 3;
const GYRO_Y = 4;

function analogGain(freq, cutoff, secondPole) {
  const [lower, higher] = cornersHz(cutoff, secondPole);
  const hiTerm = Number.isFinite(higher) ? 1 + (freq / higher) ** 2 : 1;
  return 1 / Math.sqrt((1 + (freq / lower) ** 2) * hiTerm);
}

function toneGain(freq, channel, secondPole, stepS) {
  const filt = new ImuFilter(TABLE.accel_filter_cutoff_hz, TABLE.gyro_filter_cutoff_hz, secondPole, stepS);
  const n = Math.round(4 / stepS);
  const out = new Float64Array(n);
  const raw = new Float64Array(CHANNELS);
  for (let i = 0; i < n; i++) {
    raw[channel] = Math.sin(2 * Math.PI * freq * i * stepS);
    filt.update(raw);
    out[i] = filt.output[channel];
  }
  const cycles = Math.trunc(2 * freq);
  const tail = Math.round(cycles / freq / stepS);
  assert.ok(tail > 0 && tail <= Math.trunc(n / 2));
  let sinSum = 0;
  let cosSum = 0;
  for (let k = n - tail; k < n; k++) {
    const w = 2 * Math.PI * freq * k * stepS;
    sinSum += out[k] * Math.sin(w);
    cosSum += out[k] * Math.cos(w);
  }
  return Math.hypot((2 * sinSum) / tail, (2 * cosSum) / tail);
}

test("the two sections together are -3 dB at the cutoff", () => {
  for (const q of [0, 0.25, 0.5, 1]) {
    for (const cutoff of [18.65, 40]) {
      assert.ok(Math.abs(analogGain(cutoff, cutoff, q) - HALF_POWER) < 1e-12);
    }
  }
  const [lo0, hi0] = cornersHz(18.65, 0);
  assert.ok(Math.abs(lo0 - 18.65) < 1e-12 && hi0 === Infinity);
  const [lo1, hi1] = cornersHz(18.65, 1);
  assert.ok(Math.abs(lo1 - hi1) < 1e-9);
  assert.ok(Math.abs(lo1 / 18.65 - 1 / Math.sqrt(Math.sqrt(2) - 1)) < 1e-12);
});

test("the discrete filter is -3 dB within one percent at either timestep", () => {
  for (const stepS of [0.001, 0.0005]) {
    for (const q of [0, 1]) {
      const gyro = toneGain(TABLE.gyro_filter_cutoff_hz, GYRO_X, q, stepS);
      const accel = toneGain(TABLE.accel_filter_cutoff_hz, ACCEL_X, q, stepS);
      assert.ok(Math.abs(gyro - HALF_POWER) < 0.01 * HALF_POWER);
      assert.ok(Math.abs(accel - HALF_POWER) < 0.01 * HALF_POWER);
    }
  }
});

test("a 100 Hz tone is cut as one or two sections cut it", () => {
  const cutoff = TABLE.gyro_filter_cutoff_hz;
  const g0 = toneGain(100, GYRO_Y, 0, 0.001);
  const g1 = toneGain(100, GYRO_Y, 1, 0.001);
  assert.ok(Math.abs(g0 - analogGain(100, cutoff, 0)) < 0.004);
  assert.ok(Math.abs(g1 - analogGain(100, cutoff, 1)) < 0.004);
  assert.ok(g1 < 0.5 * g0);
});

test("a constant passes unchanged and each update is counted", () => {
  const filt = new ImuFilter(40, 18.65, 1, 0.001);
  const start = [10, -20, 9810, 0.1, -0.2, 0.3];
  filt.reset(start);
  for (let i = 0; i < 50; i++) filt.update(start);
  assert.equal(filt.updates, 50);
  for (let ch = 0; ch < CHANNELS; ch++) assert.ok(Math.abs(filt.output[ch] - start[ch]) < 1e-12);
  const twice = start.map((v) => 2 * v);
  for (let i = 0; i < 1000; i++) filt.update(twice);
  for (let ch = 0; ch < CHANNELS; ch++) assert.ok(Math.abs(filt.output[ch] - twice[ch]) <= 1e-6 * Math.abs(twice[ch]));
});

test("the filter steps at every physics step and starts settled", async () => {
  const sim = new VectorSim(await mujoco(), {
    xml, values: { ...TABLE, ...QUIET }, derived: model.derived, controlLevel: CONTROL_DEFAULT,
  });
  assert.equal(sim._imuFilter.updates, 0);
  sim.tick(10);
  assert.equal(sim._imuFilter.updates, 10 * sim.d.substeps);
  sim.dispose();
});

test("the first reading is the settled one, not a ramp from zero", async () => {
  const mj = await mujoco();
  const simFilt = new VectorSim(mj, { xml, values: TABLE, derived: model.derived, noiseSeed: 4 });
  const simRaw = new VectorSim(mj, {
    xml, values: { ...TABLE, ...PASS_THROUGH }, derived: model.derived, noiseSeed: 4,
  });
  const a = simFilt.state;
  const b = simRaw.state;
  for (let i = 0; i < 3; i++) {
    assert.ok(Math.abs(a.accel_mmps2[i] - b.accel_mmps2[i]) < 1e-6);
    assert.ok(Math.abs(a.gyro_radps[i] - b.gyro_radps[i]) < 1e-6);
  }
  assert.ok(a.accel_mmps2[2] > 0.9 * TABLE.gravity_mmps2);
  simFilt.dispose();
  simRaw.dispose();
});

test("the filter takes the treads' vibration out of a spin but not the turn", async () => {
  const mj = await mujoco();
  const head = (22.5 * Math.PI) / 180;
  const runSpin = (over) => {
    const sim = new VectorSim(mj, {
      xml, values: { ...TABLE, ...QUIET, ...over }, derived: model.derived, headDeg: 22.5,
    });
    sim.setWheelMotors(-150, 150);
    sim.tick(Math.round(1.5 / sim.dt));
    let rollSum = 0;
    let rateSum = 0;
    const count = 400;
    for (let i = 0; i < count; i++) {
      const s = sim.tick(1);
      rollSum += Math.abs(s.gyro_radps[0] - s.gyro_radps[2] * Math.tan(head));
      rateSum += Math.abs(s.gyro_radps[2]);
    }
    sim.dispose();
    return { roll: rollSum / count, rate: rateSum / count };
  };
  const raw = runSpin(PASS_THROUGH);
  const filtered = runSpin({});
  assert.ok(filtered.roll < 0.35 * raw.roll, `roll did not drop enough: ${filtered.roll} vs ${raw.roll}`);
  assert.ok(Math.abs(filtered.rate - raw.rate) <= 0.02 * raw.rate, `turn rate shifted: ${filtered.rate} vs ${raw.rate}`);
});

test("accel_scale multiplies the specific force before accel_bias_mg", async () => {
  const mj = await mujoco();
  const zeroImu = { ...TABLE, ...QUIET, accel_bias_mg: [0, 0, 0], accel_scale: [1, 1, 1], imu_pitch_offset_deg: 0 };
  const sim1 = new VectorSim(mj, { xml, values: zeroImu, derived: model.derived });
  const sim2 = new VectorSim(mj, {
    xml, values: { ...zeroImu, accel_scale: [1.014, 1, 1.02] }, derived: model.derived,
  });
  const a1 = sim1.state.accel_mmps2;
  const a2 = sim2.state.accel_mmps2;
  const step = TABLE.accel_range_g * 9810 / 32767;
  assert.ok(Math.abs(a2[2] - 1.02 * a1[2]) <= (1 + 1.02) * step / 2);
  sim1.dispose();
  sim2.dispose();
});

test("reported gyro retains the hardware count spacing before residual bias", async () => {
  const sim = new VectorSim(await mujoco(), { xml, values: { ...TABLE, ...QUIET }, derived: model.derived });
  for (const target of [-180.013, 0.037, 180.013]) {
    const rate = target * Math.PI / 180;
    sim._imuFilter.reset([0, 0, 0, rate, rate, rate]);
    sim._sampleImu();
    sim._imu()[1].forEach((reported, axis) => {
      const step = TABLE.gyro_range_dps / 32768 * (axis === 2 ? 0.989 : 1);
      const corrected = reported * 180 / Math.PI - TABLE.gyro_bias_dps[axis];
      assert.ok(Math.abs(corrected / step - Math.round(corrected / step)) < 1e-8);
      assert.ok(Math.abs(corrected - target) <= step / 2);
    });
  }
  sim.dispose();
});

test("accelerometer reports signed HAL counts including both saturation endpoints", async () => {
  const values = { ...TABLE, ...QUIET, accel_scale: [1,1,1], accel_bias_mg: [0,0,0] };
  const sim = new VectorSim(await mujoco(), { xml, values, derived: model.derived });
  try {
    sim._imuFilter.reset([-25, .12345, 25, 0, 0, 0]);
    sim._sampleImu();
    const step = 9810 * values.accel_range_g / 32767;
    assert.deepEqual(sim._imu()[0], [-32768 * step, Math.round(123.45 / step) * step, 32767 * step]);
  } finally { sim.dispose(); }
});
