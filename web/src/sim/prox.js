// The forward time-of-flight sensor, as the robot reports it..
// A port of ../simulacrum/prox.py; test/prox.test.js replays the Python's own
// readings through this file.
//
// No MuJoCo here. The robot layer casts a fan of rays across the sensor's cone
// and hands over the distance each one reached. Each hit returns a signal that
// falls with distance by a power law fitted to recordings of a real robot;
// signal quality is the mean over the fan, and the distance is the
// signal-weighted mean of the hits. The reporting rules are the robot
// engine's: the distance is clamped to 30..400 mm, the lift is in the beam
// between two lift heights (and lingers), a weak signal means the way is
// clear, and nothing counts as found while the robot is pitched. With no
// usable signal the real sensor still reports a distance, and so does this.

import { assert, makeGauss, pyRound } from "./util.js";

// (fraction of the half-angle, rays on that ring), plus one ray on the axis.
export const RINGS = [[0.5, 6], [0.9, 12]];

// Unit vectors of the fan in the sensor's frame: z along the axis, x and y across it.
export function rayDirections(v) {
  const half = (0.5 * v.tof_fov_deg * Math.PI) / 180;
  assert(half > 0 && half < 0.5 * Math.PI, "time-of-flight field of view out of range");
  const out = [[0, 0, 1]];
  for (const [fraction, count] of RINGS) {
    const polar = fraction * half;
    for (let k = 0; k < count; k++) {
      const azimuth = (2 * Math.PI * k) / count;
      out.push([Math.sin(polar) * Math.cos(azimuth), Math.sin(polar) * Math.sin(azimuth), Math.cos(polar)]);
    }
  }
  return out;
}

export class ProxSensor {
  constructor(v, noiseSeed) {
    assert(v.tof_report_min_mm < v.tof_report_max_mm && v.tof_report_max_mm <= v.tof_range_max_mm, "report limits out of order");
    assert(v.tof_lift_occluding_min_mm < v.tof_lift_occluding_max_mm, "lift window is empty");
    assert(v.tof_quality_at_100mm > 0 && v.tof_quality_distance_exponent < 0, "bad signal law");
    assert(v.tof_quality_floor < v.tof_quality_min, "the weak-signal floor must be below the threshold");
    this.v = v;
    this.rays = rayDirections(v);
    this._gauss = makeGauss(noiseSeed);
    this._clearNeeded = Math.max(1, Math.ceil(v.tof_lift_clear_ms / v.state_period_ms));
    this._clearCount = this._clearNeeded;
  }

  // In the beam between two heights; still counted in view until it has been out for a while.
  _liftInView(liftHeightMm) {
    const v = this.v;
    if (liftHeightMm >= v.tof_lift_occluding_min_mm && liftHeightMm <= v.tof_lift_occluding_max_mm) {
      this._clearCount = 0;
      return true;
    }
    this._clearCount = Math.min(this._clearCount + 1, this._clearNeeded);
    return this._clearCount < this._clearNeeded;
  }

  // [signal quality, signal-weighted distance in mm] of the fan's hits; distance is 0 with no signal.
  _signal(hits) {
    const v = this.v;
    assert(hits.length === this.rays.length, "one hit per ray");
    let total = 0;
    let weighted = 0;
    for (const distanceMm of hits) {
      if (distanceMm < 0 || distanceMm > v.tof_range_max_mm) continue;
      const near = Math.max(distanceMm, v.tof_report_min_mm);
      const strength = v.tof_quality_at_100mm * (near / 100) ** v.tof_quality_distance_exponent;
      total += strength;
      weighted += strength * distanceMm;
    }
    return [total / this.rays.length, total > 0 ? weighted / total : 0];
  }

  // One reading. hits: per ray, the distance in mm to what it hit, or -1 for nothing.
  read(hits, liftHeightMm, pitchRad) {
    return this.report(this.acquire(hits, liftHeightMm), liftHeightMm, pitchRad);
  }

  acquire(hits, liftHeightMm) {
    const v = this.v;
    assert(Number.isFinite(liftHeightMm), "lift height must be finite");
    const inBeam = liftHeightMm >= v.tof_lift_occluding_min_mm && liftHeightMm <= v.tof_lift_occluding_max_mm;
    let [quality, raw] = inBeam ? [v.tof_lift_quality, v.tof_lift_distance_mm] : this._signal(hits);
    if (quality <= v.tof_quality_min) {
      quality = Math.max(quality, v.tof_quality_floor);
      raw = this._gauss(v.tof_weak_distance_mm, v.tof_weak_distance_sd_mm);
    } else {
      raw *= 1 + this._gauss(0, v.tof_range_std_fraction);
    }
    return { quality, raw };
  }

  report({ quality, raw }, liftHeightMm, pitchRad) {
    const v = this.v;
    assert(Number.isFinite(liftHeightMm) && Number.isFinite(pitchRad), "lift height and pitch must be finite");
    const liftInView = this._liftInView(liftHeightMm);
    const limit = (v.tof_found_pitch_limit_deg * Math.PI) / 180;
    const unobstructed = quality <= v.tof_quality_min || raw >= v.tof_report_max_mm;
    const found = !liftInView && quality > v.tof_quality_min && raw < v.tof_report_max_mm && pitchRad >= -limit && pitchRad <= limit;
    const distance = pyRound(Math.min(Math.max(raw, v.tof_report_min_mm), v.tof_report_max_mm));
    return { distance_mm: distance, signal_quality: quality, unobstructed, found_object: found, is_lift_in_fov: liftInView };
  }
}

// Sensor register bytes are big-endian; syscon appends its little-endian sample counter.
export function writeBodyRange(view, offset, sample, status = 0x58) {
  const quality = Math.max(0, sample.quality);
  // The two fixed-point words encode a ratio. Keep the nominal 90 effective
  // SPADs while the 9.7 return rate fits; scale down for strong reflections.
  const spads = Math.max(1, Math.min(90, Math.floor(65535 / (128 * quality))));
  view.setUint8(offset, status);
  view.setUint16(offset + 2, Math.min(65535, Math.max(0, Math.round(sample.raw))));
  view.setUint16(offset + 4, Math.min(65535, Math.round(128 * spads * quality)));
  view.setUint16(offset + 8, 256 * spads);
  view.setUint16(offset + 10, sample.count, true);
}
