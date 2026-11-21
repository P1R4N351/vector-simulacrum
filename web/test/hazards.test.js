// Cliff, fall and pick-up flags against a trace of the Python..
import assert from "node:assert/strict";
import { test } from "node:test";
import { Hazards } from "../src/sim/hazards.js";
import { readJson } from "./helpers.js";

const fixture = readJson("test/fixtures/hazards.json");
const values = readJson("public/robot/model.json").values;

test("every hazard flag matches the Python on 1500 ticks", () => {
  const hazards = new Hazards(values);
  assert.equal(fixture.rows.length, 1500);
  fixture.rows.forEach(([ranges, accel, cliff, any, fresh, falling, pickedUp], tick) => {
    hazards.update(ranges, accel);
    assert.deepEqual(hazards.cliff, cliff, `cliff flags at tick ${tick}`);
    assert.equal(hazards.cliffAny, any, `cliff_any at tick ${tick}`);
    assert.equal(hazards.cliffNew, fresh, `cliff_new at tick ${tick}`);
    assert.equal(hazards.falling, falling, `falling at tick ${tick}`);
    assert.equal(hazards.pickedUp, pickedUp, `picked_up at tick ${tick}`);
  });
  const seen = (index) => fixture.rows.some((row) => row[index]);
  assert.ok(seen(4) && seen(5) && seen(6), "the trace raises cliff_new, falling and picked_up");
});
