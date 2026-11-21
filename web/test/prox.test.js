// The time-of-flight sensor's reporting rules against readings of the Python..
import assert from "node:assert/strict";
import { test } from "node:test";
import { ProxSensor, rayDirections } from "../src/sim/prox.js";
import { readJson } from "./helpers.js";

const fixture = readJson("test/fixtures/prox.json");
const close = (a, b) => Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(b));

test("the fan of rays is the Python's: 19 unit rays inside the cone, axis first", () => {
  const rays = rayDirections(fixture.values);
  assert.equal(rays.length, 19);
  assert.equal(fixture.rays.length, rays.length);
  rays.forEach((ray, i) => ray.forEach((c, k) => assert.ok(Math.abs(c - fixture.rays[i][k]) < 1e-12, `ray ${i} axis ${k}`)));
  assert.deepEqual(rays[0], [0, 0, 1]);
  const half = (0.5 * fixture.values.tof_fov_deg * Math.PI) / 180;
  for (const ray of rays) assert.ok(Math.acos(Math.min(1, ray[2])) <= half + 1e-12 && close(Math.hypot(...ray), 1));
});

function replay(values, rows, label) {
  const sensor = new ProxSensor(values, 0);
  assert.equal(rows.length, 900);
  rows.forEach(([hits, lift, pitch, distance, quality, unobstructed, found, liftInView], step) => {
    const got = sensor.read(hits, lift, pitch);
    assert.equal(got.distance_mm, distance, `${label}: distance at step ${step}`);
    assert.ok(close(got.signal_quality, quality), `${label}: quality at step ${step}: ${got.signal_quality} against ${quality}`);
    assert.equal(got.unobstructed, unobstructed, `${label}: unobstructed at step ${step}`);
    assert.equal(got.found_object, found, `${label}: found at step ${step}`);
    assert.equal(got.is_lift_in_fov, liftInView, `${label}: lift in view at step ${step}`);
  });
}

test("every reading matches the Python on 900 fans of hits", () => {
  replay(fixture.values, fixture.rows, "table values");
  assert.ok(fixture.rows.some((r) => r[3] === fixture.values.tof_report_min_mm), "the lower limit is reached");
});

test("with a strong signal, far hits register and the reading stops at the upper limit, as in the Python", () => {
  replay(fixture.strong_values, fixture.strong_rows, "strong signal");
  assert.ok(fixture.strong_rows.some((r) => r[3] === fixture.values.tof_report_max_mm), "the upper limit is reached");
});

test("with noise on, weak readings scatter about the recorded distance and stay inside the limits", () => {
  const v = { ...fixture.values, tof_weak_distance_sd_mm: 22.2 };
  const sensor = new ProxSensor(v, 7);
  const nothing = new Array(sensor.rays.length).fill(-1);
  const readings = Array.from({ length: 2000 }, () => sensor.read(nothing, 80, 0).distance_mm);
  const mean = readings.reduce((a, b) => a + b, 0) / readings.length;
  const spread = Math.sqrt(readings.reduce((a, b) => a + (b - mean) ** 2, 0) / readings.length);
  assert.ok(Math.abs(mean - v.tof_weak_distance_mm) < 2 && Math.abs(spread - 22.2) < 2, `mean ${mean}, spread ${spread}`);
  assert.ok(Math.min(...readings) >= v.tof_report_min_mm && Math.max(...readings) <= v.tof_report_max_mm);
});
