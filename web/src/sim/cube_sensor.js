// Stock last-frame cube app, following simulacrum/cube.py. Packet slots are in chip-axis counts.
import { CubeListeners } from "./cube_listeners.js";
import { ImuFilter } from "./imu_filter.js";
import { assert, clamp, makeGauss, pyRound } from "./util.js";

export class CubeSensor {
  constructor(values, step, seed, restingForce) {
    this.v = values;
    this.framePeriod = values.cube_accel_frame_period_ms / 1000;
    this.tickPeriod = values.cube_tick_ms / 1000;
    assert(this.framePeriod > 0 && this.tickPeriod > 0, "cube clock periods must be positive");
    this.gauss = makeGauss(seed);
    this.sigma = values.cube_accel_noise_density_ug * 1e-6 * Math.sqrt(.5 / step);
    const cutoff = values.cube_accel_bandwidth_hz;
    this.filter = new ImuFilter(cutoff, cutoff, values.cube_accel_filter_second_pole, step);
    const initial = this.sense(restingForce, 0);
    this.filter.reset([...initial, ...initial]);
    this.frames = this.ticks = this.buffered = this.debounce = this.previousZ = this.taps = 0;
    this.positiveFirst = false;
    this.slots = Array.from({ length: 3 }, () => [0, 0, 0]);
    this.packet = null;
    this.handlingCube = false;
    this.movementBroadcastCount = 0;
    this.listeners = new CubeListeners();
  }

  sense(force, sigma) {
    return Array.from(force, (value, axis) =>
      value * this.v.cube_accel_scale[axis] + this.v.cube_accel_bias_mg[axis] / 1000 +
      this.gauss(0, sigma) * (this.v.cube_accel_noise_axis_scale?.[axis] ?? 1));
  }

  store(frame) {
    if (this.buffered >= 32) return;
    this.buffered++;
    this.last = frame;
    const z = frame[2];
    const delta = ((z - this.previousZ + 32768) & 65535) - 32768;
    this.previousZ = z;
    if (this.debounce > 0) {
      // accel.c uses +90 here for the negative-first case.
      if (this.debounce > 80 && delta < (this.positiveFirst ? -2880 : 90)) {
        this.taps = (this.taps + 1) & 255;
        this.debounce = 80;
      }
      this.debounce--;
    } else if (Math.abs(delta) > 2880) {
      this.positiveFirst = delta > 0;
      this.debounce = 90;
    }
  }

  receivePacket(packet, bodyStateStampMs) {
    const events = this.listeners.receive(packet);
    if (bodyStateStampMs !== undefined) {
      for (const event of events) event.timestamp = bodyStateStampMs;
    }
    // The engine keeps filtering the handled cube but suppresses its movement broadcasts.
    const movementBroadcasts = this.handlingCube ? [] : events.filter(event => "moving" in event);
    this.movementBroadcastCount += movementBroadcasts.length;
    this.packet = { ...packet, events, movementBroadcasts };
  }

  step(time, force, bodyStateStampMs) {
    const sensed = this.sense(force, this.sigma);
    this.filter.update([...sensed, ...sensed]);
    const [shift, lowBits] = this.v.cube_accel_word_format ?? [4, 0];
    const limit = 2 ** (15 - shift);
    const scale = this.v.cube_accel_counts_per_g / 2 ** shift;
    const frame = Array.from(this.filter.output.subarray(0, 3), v =>
      (clamp(pyRound(v * scale), -limit, limit - 1) << shift) | lowBits);
    for (let i = 0; i < 64; i++) {
      const nextFrame = (this.frames + 1) * this.framePeriod;
      const nextTick = (this.ticks + 1) * this.tickPeriod;
      if (Math.min(nextFrame, nextTick) > time + 1e-9) return;
      if (nextFrame <= nextTick + 1e-9) {
        this.store(frame);
        this.frames++;
      } else {
        if (this.buffered) this.slots[this.ticks % 3] = this.last;
        this.buffered = 0;
        this.ticks++;
        if (this.ticks % 3 === 0) {
          this.receivePacket({ time_s: nextTick, taps: this.taps, slots: this.slots.slice() }, bodyStateStampMs);
        }
      }
    }
    throw new Error("cube clock overrun");
  }
}
