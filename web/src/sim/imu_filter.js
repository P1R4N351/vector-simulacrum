// The IMU chip's own low-pass filter, run at every physics step.../simulacrum/imu_filter.py by a lane).
//
// Needs no MuJoCo. Models the BMI160's two digital filter stages in series as
// two one-pole EMA sections placed so their series response is -3 dB at cutoffHz.
import { assert } from "./util.js";

export const SECTIONS = 2;
export const CHANNELS = 6; // accelerometer x, y, z, then gyro x, y, z

// Corner frequencies [lower, higher] whose series response is -3 dB at cutoffHz.
export function cornersHz(cutoffHz, secondPole) {
  assert(Number.isFinite(cutoffHz) && cutoffHz > 0, "cutoff must be positive and finite");
  assert(secondPole >= 0 && secondPole <= 1, "second_pole runs from 0 (one section) to 1 (two equal sections)");
  const q2 = secondPole * secondPole;
  const x = 2 / ((1 + q2) + Math.sqrt((1 + q2) ** 2 + 4 * q2));
  const lower = cutoffHz / Math.sqrt(x);
  const higher = secondPole > 0 ? lower / secondPole : Infinity;
  assert(lower >= cutoffHz && higher >= lower, "both corners lie at or above the cutoff");
  return [lower, higher];
}

// Per-section, per-channel EMA gains: row 0 is lower corner, row 1 is higher.
export function sectionGains(cutoffsHz, secondPole, stepS) {
  assert(cutoffsHz.length === CHANNELS, "one cutoff per channel");
  assert(Number.isFinite(stepS) && stepS > 0, "step must be positive");
  const gains = [new Float64Array(CHANNELS), new Float64Array(CHANNELS)];
  for (let ch = 0; ch < CHANNELS; ch++) {
    const corners = cornersHz(cutoffsHz[ch], secondPole);
    for (let row = 0; row < SECTIONS; row++) {
      const gain = 1 - Math.exp(-2 * Math.PI * corners[row] * stepS);
      assert(gain > 0 && gain <= 1, "a gain left (0, 1]");
      gains[row][ch] = gain;
    }
  }
  return gains;
}

export class ImuFilter {
  constructor(accelCutoffHz, gyroCutoffHz, secondPole, stepS) {
    this.gains = sectionGains(
      [accelCutoffHz, accelCutoffHz, accelCutoffHz, gyroCutoffHz, gyroCutoffHz, gyroCutoffHz],
      secondPole,
      stepS,
    );
    this.state = [new Float64Array(CHANNELS), new Float64Array(CHANNELS)];
    this.updates = 0;
  }

  // Start both sections at a steady reading.
  reset(raw) {
    assert(raw.length === CHANNELS && Array.from(raw, Number.isFinite).every(Boolean), "reset needs six finite values");
    for (let ch = 0; ch < CHANNELS; ch++) {
      this.state[0][ch] = raw[ch];
      this.state[1][ch] = raw[ch];
    }
  }

  // One physics substep.
  update(raw) {
    assert(raw.length === CHANNELS, "update needs six values");
    const s0 = this.state[0];
    const s1 = this.state[1];
    const g0 = this.gains[0];
    const g1 = this.gains[1];
    for (let ch = 0; ch < CHANNELS; ch++) {
      s0[ch] += g0[ch] * (raw[ch] - s0[ch]);
      s1[ch] += g1[ch] * (s0[ch] - s1[ch]);
    }
    this.updates += 1;
  }

  get output() {
    return this.state[1];
  }
}
