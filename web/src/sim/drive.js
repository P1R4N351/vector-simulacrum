// The tread drive as the firmware runs it: encoder and speed control.
// test/drive.test.js replays the Python's own traces through this file.
//
// No MuJoCo here. Both pieces are stepped once per 5 ms control tick.

import { assert } from "./util.js";

// Counts of one tread and the unsigned speed measured from them. The encoder
// has one channel: it counts edges whichever way the tread turns.
export class TreadEncoder {
  constructor(v) {
    this._mmPerCount = Number(v.tread_mm_per_encoder_count);
    this._zeroTicks = Math.trunc(v.encoder_zero_speed_ticks);
    this._dt = v.control_period_ms * 0.001;
    assert(this._mmPerCount > 0 && this._zeroTicks >= 1, "encoder scale and zero threshold must be positive");
    this.counts = 0;
    this.speedMmps = 0;
    this.direction = 1;
    this._driveDirection = 1;
    this._pathMm = 0;
    this._lastTravel = 0;
    this._lastEdgeS = 0;
    this._nowS = 0;
    this._idleTicks = 0;
  }

  // Feed the tread's signed travel so far; returns the unsigned measured speed in mm/s.
  update(travelMm, drive = 0) {
    if (drive !== 0) this._driveDirection = Math.sign(drive);
    assert(Number.isFinite(travelMm), "tread travel must be finite");
    const step = Math.abs(travelMm - this._lastTravel);
    this._lastTravel = travelMm;
    const startPath = this._pathMm;
    this._pathMm += step;
    const total = Math.trunc(this._pathMm / this._mmPerCount);
    const fresh = total - this.counts;
    if (fresh > 0) {
      this.direction = this._driveDirection;
      const edgePath = total * this._mmPerCount;
      const edgeS = this._nowS + (this._dt * (edgePath - startPath)) / step;
      const span = edgeS - this._lastEdgeS;
      if (span > 0) this.speedMmps = (fresh * this._mmPerCount) / span;
      this._lastEdgeS = edgeS;
      this.counts = total;
      this._idleTicks = 0;
    } else {
      this._idleTicks = Math.min(this._idleTicks + 1, this._zeroTicks);
      if (this._idleTicks >= this._zeroTicks) this.speedMmps = 0;
    }
    this._nowS += this._dt;
    return this.speedMmps;
  }
}

// The firmware's wheel speed loop for one tread. Output is motor power in -1..1.
export class TreadController {
  constructor(v) {
    this._kp = v.wheel_kp_power_per_mmps;
    this._ki = v.wheel_ki_power_per_mmps_tick;
    this._sumMax = v.wheel_error_sum_max;
    this._slope = v.wheel_ff_slope_power_per_mmps;
    this._offset = v.wheel_ff_offset_power;
    this._lowSpeed = v.wheel_ff_low_speed_mmps;
    this._lowSlope = v.wheel_ff_low_slope_power_per_mmps;
    this._stopped = v.wheel_command_stopped_mmps;
    this._max = v.motor_power_max;
    assert(this._max > 0 && this._sumMax > 0, "power and error-sum limits must be positive");
    assert(this._slope > 0 && this._lowSlope > 0, "feedforward slopes must be positive");
    this.errorSum = 0;
    this.power = 0;
  }

  // The firmware's fitted line of power against commanded speed, with the command's sign.
  feedforward(desiredMmps) {
    const speed = Math.abs(desiredMmps);
    const out = speed > this._lowSpeed ? this._slope * speed + this._offset : this._lowSlope * speed;
    assert(out >= 0, "feedforward magnitude must not be negative");
    return desiredMmps < 0 ? -out : out;
  }

  // One control tick. measuredMmps is the signed speed the firmware believes.
  update(desiredMmps, measuredMmps) {
    assert(Number.isFinite(desiredMmps) && Number.isFinite(measuredMmps), "speeds must be finite");
    const error = desiredMmps - measuredMmps;
    const out = this.feedforward(desiredMmps) + this._kp * error + this._ki * this.errorSum;
    if (Math.abs(desiredMmps) <= this._stopped) {
      this.power = 0;
      this.errorSum = 0;
      return this.power;
    }
    this.power = desiredMmps > 0
      ? Math.min(Math.max(out, 0), this._max)
      : Math.min(Math.max(out, -this._max), 0);
    if (Math.abs(this.power) < this._max) {
      this.errorSum = Math.min(Math.max(this.errorSum + error, -this._sumMax), this._sumMax);
    }
    return this.power;
  }

  reset() {
    this.errorSum = 0;
    this.power = 0;
  }
}

// The firmware stores the measured wheel speed in an integer: the fraction is dropped.
export function wholeMmps(speedMmps) {
  assert(Number.isFinite(speedMmps), "speed must be finite");
  return Math.trunc(speedMmps) + 0;
}
