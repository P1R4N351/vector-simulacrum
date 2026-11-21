// Cliff, fall and pick-up flags, as the robot derives them..
// A port of ../simulacrum/hazards.py; test/hazards.test.js
// replays the Python's own trace through this file.
//
// Inputs once per control tick: the four cliff ranges (mm) and the
// accelerometer magnitude (mm/s^2). The pick-up rule is the simulacrum's
// reduced form of the firmware's (cliff sensors only).

import { assert, pyRound } from "./util.js";

export class Hazards {
  constructor(v) {
    assert(v.fall_accel_low_mmps2 < v.fall_accel_high_mmps2, "fall thresholds out of order");
    assert(v.cliff_detect_range_mm > v.cliff_sensor_height_mm, "cliff range inside the sensor height");
    this._rangeMm = Number(v.cliff_detect_range_mm);
    this._low = Number(v.fall_accel_low_mmps2);
    this._high = Number(v.fall_accel_high_mmps2);
    this._fallNeed = pyRound(v.fall_detect_time_ms / v.control_period_ms);
    this._pickupNeed = Math.trunc(v.pickup_detect_ticks);
    this._pickupCount = Math.trunc(v.pickup_cliff_sensor_count);
    this.cliff = [false, false, false, false];
    this.cliffAny = false;
    this.cliffNew = false;
    this.falling = false;
    this.pickedUp = false;
    this._fallTicks = 0;
    this._pickupTicks = 0;
  }

  _updateCliffs(rangesMm) {
    assert(rangesMm.length === 4, "four cliff ranges");
    const wasAny = this.cliffAny;
    for (let i = 0; i < 4; i++) this.cliff[i] = rangesMm[i] < 0 || rangesMm[i] > this._rangeMm;
    this.cliffAny = this.cliff.some(Boolean);
    this.cliffNew = this.cliffAny && !wasAny;
  }

  // Free fall: low acceleration with a cliff under a sensor, held for the detection time.
  _updateFall(accelMagMmps2) {
    assert(accelMagMmps2 >= 0, "acceleration magnitude must not be negative");
    if (this.falling) {
      if (accelMagMmps2 > this._high) {
        this.falling = false;
        this._fallTicks = 0;
      }
      return;
    }
    if (accelMagMmps2 < this._low && this.cliffAny) this._fallTicks += 1;
    else if (accelMagMmps2 >= this._high || !this.cliffAny) this._fallTicks = 0;
    if (this._fallTicks > this._fallNeed) this.falling = true;
  }

  _updatePickup() {
    const count = this.cliff.filter(Boolean).length;
    assert(count >= 0 && count <= 4, "cliff count out of range");
    this._pickupTicks = count >= this._pickupCount ? Math.min(this._pickupTicks + 1, this._pickupNeed + 1) : 0;
    this.pickedUp = this._pickupTicks > this._pickupNeed;
  }

  update(rangesMm, accelMagMmps2) {
    this._updateCliffs(rangesMm);
    this._updateFall(accelMagMmps2);
    this._updatePickup();
  }
}
