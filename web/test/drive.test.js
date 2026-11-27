// The tread encoder and speed loop against traces of the Python..
import assert from "node:assert/strict";
import { test } from "node:test";
import { TreadController, TreadEncoder, wholeMmps } from "../src/sim/drive.js";
import { readJson } from "./helpers.js";

const fixture = readJson("test/fixtures/drive.json");
const values = readJson("public/robot/model.json").values;
const close = (a, b) => Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(b));

test("encoder counts and measured speed match the Python on 1200 ticks", () => {
  const encoder = new TreadEncoder(values);
  assert.equal(fixture.encoder.length, 1200);
  fixture.encoder.forEach(([travel, counts, speed], tick) => {
    const out = encoder.update(travel);
    assert.equal(encoder.counts, counts, `counts at tick ${tick}`);
    assert.ok(close(out, speed), `speed at tick ${tick}: ${out} against ${speed}`);
  });
  assert.ok(fixture.encoder.some((row) => row[2] > 0) && fixture.encoder.some((row) => row[2] === 0));
});

test("speed loop power and error sum match the Python on 1200 ticks", () => {
  const controller = new TreadController(values);
  assert.equal(fixture.controller.length, 1200);
  fixture.controller.forEach(([desired, measured, power, errorSum], tick) => {
    const out = controller.update(desired, measured);
    assert.ok(close(out, power), `power at tick ${tick}: ${out} against ${power}`);
    assert.ok(close(controller.errorSum, errorSum), `error sum at tick ${tick}`);
  });
});

test("feedforward line and integer speed match the Python", () => {
  const controller = new TreadController(values);
  for (const [speed, power] of fixture.feedforward) assert.ok(close(controller.feedforward(speed), power), `ff ${speed}`);
  for (const [speed, whole] of fixture.whole_mmps) assert.equal(wholeMmps(speed), whole, `whole ${speed}`);
});

test("signed speed holds the last edge across a change in motor duty", () => {
  const encoder = new TreadEncoder(values);
  const step = values.tread_mm_per_encoder_count;
  encoder.update(1.5*step, 0.5);
  encoder.update(1.6*step, -0.5);
  assert.equal(encoder.direction, 1);
  encoder.update(2.5*step, 0);
  assert.equal(encoder.direction, -1);
});
