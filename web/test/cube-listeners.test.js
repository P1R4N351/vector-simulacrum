import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";
import { CubeListeners } from "../src/sim/cube_listeners.js";
import { replayCsv } from "../tools/replay-cube-listeners.mjs";

test("the physical roll produces both starts and stops at the reference packet times", () => {
  const path = new URL("../../evidence/unit3-wheelstand/shelf-roll-stop/S22B/cube.csv.gz", import.meta.url);
  assert.deepEqual(replayCsv(path).filter(event => "moving" in event), [
    { moving: true, time_s: 37.122122 }, { moving: false, time_s: 37.82005 },
    { moving: true, time_s: 55.207216 }, { moving: false, time_s: 56.437308 },
  ]);
});

test("up-axis initializes silently and crosses from +Z to -X on slot 23 of a quarter turn", () => {
  const listener = new CubeListeners();
  function feed(time_s, sample) {
    return listener.receive({time_s, taps: 0, slots: [sample, sample, sample]});
  }
  for (let i = 1; i <= 10; i++) assert.deepEqual(feed(i*.03, [0,0,8192]), []);
  assert.equal(listener.axis, 6);
  for (let i = 11; i <= 17; i++) assert.deepEqual(feed(i*.03, [-8192,0,0]), []);
  assert.deepEqual(feed(.54, [-8192,0,0]), [{axis: 1, time_s: .54}]);
});

test("a changed wrapping tap counter emits one candidate per packet", () => {
  const listener = new CubeListeners();
  const slots = Array.from({length: 3}, () => [0, 0, 8192]);
  const send = (time_s, taps) => listener.receive({time_s, taps, slots});
  assert.deepEqual(send(.03, 255), []);
  assert.deepEqual(send(.06, 0), [{time_s: .06, tap_count: 0}]);
  assert.deepEqual(send(.09, 0), []);
  assert.deepEqual(send(.12, 3), [{time_s: .12, tap_count: 3}]);
});

test("docked grip retains three taps and both motion transitions after the cube settles", () => {
  const readRows = file => readFileSync(new URL(
    "../../evidence/unit3-docked-grip/" + file, import.meta.url), "utf8"
  ).trim().split(/\r?\n/).slice(1).map(line => line.split(","));
  const samples = readRows("cube.csv");
  const events = readRows("release-events.csv");
  const listener = new CubeListeners();
  const observed = [];
  let offset = 0;
  for (const [kind, receipt, uptime, taps] of events) {
    if (kind !== "packet") continue;
    const slots = samples.slice(offset, offset + 3).map(([stamp, index, ...xyz], i) => {
      assert.equal(stamp, receipt);
      assert.equal(Number(index), i);
      return xyz.map(Number);
    });
    offset += 3;
    observed.push(...listener.receive({time_s: Number(uptime), taps: Number(taps), slots}));
  }
  assert.equal(offset, 95 * 3);
  assert.equal(samples.length, offset);
  const candidates = observed.filter(event => "moving" in event || "tap_count" in event);
  const sdk = events.filter(([kind]) => kind !== "packet");
  assert.equal(candidates.length, sdk.length);
  candidates.forEach((event, i) => {
    const kind = "tap_count" in event ? "object_tapped" :
      event.moving ? "object_moved" : "object_stopped_moving";
    assert.equal(kind, sdk[i][0]);
    assert.ok(Math.abs(Math.round(event.time_s * 1000) - Math.round(Number(sdk[i][2]) * 1000)) <= 36);
  });
  assert.equal(listener.tapEvents, 3);
  assert.equal(listener.motionTransitions, 2);
  assert.equal(listener.lastTapTime, candidates[2].time_s);
  assert.equal(listener.lastMotionTime, candidates[4].time_s);
  assert.equal(listener.moving, false);
});
