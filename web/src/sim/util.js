// Small shared helpers for the simulator port..

export function assert(condition, message) {
  if (!condition) throw new Error(`assertion failed: ${message}`);
}

export function clamp(value, low, high) {
  assert(low <= high, "empty clamp range");
  return Math.min(Math.max(value, low), high);
}

export function toward(current, target, maxStep) {
  assert(maxStep >= 0, "step must not be negative");
  return current + clamp(target - current, -maxStep, maxStep);
}

// One tick of a speed-and-acceleration profile toward a target: speed up at accel, never
// above cap, and slow down so as to stop on the target. Returns [new position, new speed].
export function profileStep(current, target, rate, cap, accel, dt) {
  assert(cap > 0 && accel > 0 && dt > 0 && rate >= 0, "profile needs positive speed, acceleration and step");
  const remaining = Math.abs(target - current);
  if (remaining === 0) return [current, 0];
  const speed = Math.min(cap, rate + accel * dt, Math.sqrt(2 * accel * remaining));
  const next = toward(current, target, speed * dt);
  return [next, next === target ? 0 : speed];
}

// Python's round(): halves go to the even neighbour. Math.round sends them up.
export function pyRound(x) {
  assert(Number.isFinite(x), "cannot round a non-finite number");
  const floor = Math.floor(x);
  const diff = x - floor;
  if (diff < 0.5) return floor;
  if (diff > 0.5) return floor + 1;
  return floor % 2 === 0 ? floor : floor + 1;
}

// A seeded standard-normal source (mulberry32 and Box-Muller). The Python
// simulacrum draws its sensor noise from random.Random; the two streams are
// not the same numbers, only the same distribution.
export function makeGauss(seed) {
  assert(Number.isInteger(seed), "noise seed must be an integer");
  let state = seed >>> 0;
  const uniform = () => {
    state = (state + 0x6d2b79f5) >>> 0;
    let t = state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  return (mean, sigma) => {
    if (sigma === 0) return mean;
    const u = Math.max(uniform(), 1e-12);
    const w = uniform();
    return mean + sigma * Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * w);
  };
}
