import assert from "node:assert/strict";
import { test } from "node:test";
import { CubeListeners } from "../src/sim/cube_listeners.js";
import { replayCsv } from "../tools/replay-cube-listeners.mjs";

test("the physical roll produces both starts and stops at the reference packet times", () => {
  const path = new URL("../../evidence/unit3-wheelstand/shelf-roll-stop/S22B/cube.csv.gz", import.meta.url);
  assert.deepEqual(replayCsv(path), [
    { moving: true, time_s: 37.122122 }, { moving: false, time_s: 37.82005 },
    { moving: true, time_s: 55.207216 }, { moving: false, time_s: 56.437308 },
  ]);
});

test("up-axis initializes silently and crosses from +Z to -X on slot 23 of a quarter turn", () => {
  const listener = new CubeListeners();
  function feed(time_s, sample) {
    return listener.receive({time_s, slots: [sample, sample, sample]});
  }
  for (let i = 1; i <= 10; i++) assert.deepEqual(feed(i*.03, [0,0,8192]), []);
  assert.equal(listener.axis, 6);
  for (let i = 11; i <= 17; i++) assert.deepEqual(feed(i*.03, [-8192,0,0]), []);
  assert.deepEqual(feed(.54, [-8192,0,0]), [{axis: 1, time_s: .54}]);
});
