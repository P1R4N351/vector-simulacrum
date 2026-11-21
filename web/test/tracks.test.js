// The track path the moving cleats follow..
import assert from "node:assert/strict";
import { test } from "node:test";
import { makeBeltPath } from "../src/game/tracks.js";

// The SDK model's belt, in its own units (cm), as measured from the model file.
const FRONT = { x: 0.295, z: 1.407, r: 1.458 };
const REAR = { x: -4.444, z: 1.049, r: 1.036 };

test("the belt path is closed, has the stated length, and is paced by arc length", () => {
  const path = makeBeltPath(FRONT, REAR);
  const steps = 20000;
  let walked = 0;
  let longest = 0;
  let previous = path.pointAt(0);
  for (let i = 1; i <= steps; i++) {
    const point = path.pointAt((i * path.length) / steps);
    const hop = Math.hypot(point.x - previous.x, point.z - previous.z);
    walked += hop;
    longest = Math.max(longest, hop);
    previous = point;
  }
  const end = path.pointAt(path.length);
  const start = path.pointAt(0);
  assert.ok(Math.hypot(end.x - start.x, end.z - start.z) < 1e-9, "the path closes");
  assert.ok(Math.abs(walked - path.length) < 1e-4 * path.length, `walked ${walked} against ${path.length}`);
  assert.ok(longest < (1.001 * path.length) / steps, "no jump between neighbouring points");
  assert.ok(Math.abs(2 * path.straight + path.frontArc + path.rearArc - path.length) < 1e-12);
});

test("every point is on a wheel or on a line touching both, with a unit outward normal", () => {
  const path = makeBeltPath(FRONT, REAR);
  for (let i = 0; i < 2000; i++) {
    const p = path.pointAt((i * path.length) / 2000);
    assert.ok(Math.abs(Math.hypot(p.nx, p.nz) - 1) < 1e-12, "unit normal");
    const fromFront = Math.hypot(p.x - FRONT.x, p.z - FRONT.z);
    const fromRear = Math.hypot(p.x - REAR.x, p.z - REAR.z);
    assert.ok(fromFront > FRONT.r - 1e-9 && fromRear > REAR.r - 1e-9, "never inside a wheel");
    // Outward: stepping along the normal moves away from both wheel centres' nearer one.
    const nearer = fromFront - FRONT.r < fromRear - REAR.r ? FRONT : REAR;
    assert.ok((p.x - nearer.x) * p.nx + (p.z - nearer.z) * p.nz > 0, "normal points away from the wheel");
  }
});

test("driving forward carries the top of the belt forward and the bottom back", () => {
  const path = makeBeltPath(FRONT, REAR);
  const top = [path.pointAt(0.2 * path.straight), path.pointAt(0.3 * path.straight)];
  assert.ok(top[1].x > top[0].x && top[0].nz > 0.9, "top run moves toward the front, normal up");
  const base = path.straight + path.frontArc;
  const bottom = [path.pointAt(base + 0.2 * path.straight), path.pointAt(base + 0.3 * path.straight)];
  assert.ok(bottom[1].x < bottom[0].x && bottom[0].nz < -0.9, "bottom run moves toward the rear, normal down");
});

test("equal wheels give a stadium; a wheel inside the other is refused", () => {
  const path = makeBeltPath({ x: 1, z: 0, r: 0.5 }, { x: -1, z: 0, r: 0.5 });
  assert.ok(Math.abs(path.length - (4 + Math.PI)) < 1e-12, `stadium length ${path.length}`);
  assert.throws(() => makeBeltPath({ x: 0, z: 0, r: 2 }, { x: 0.5, z: 0, r: 1 }), /inside/);
});
