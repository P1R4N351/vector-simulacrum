// Engine-side CubeAccelComponent listeners. Each delivered packet supplies three
// ordered samples. SDK event suppression during carrying/docking is downstream.
const f32 = Math.fround;
// Physical BlockTapFilterComponent waits 75 ms from engine intake. The first
// eligible 60 ms engine update publishes the last tap's original payload.
export class TapBroadcasts {
  constructor() {
    this.deadlineUs = 0;
    this.pending = null;
    this.count = 0;
    this.last = null;
  }

  receive(event) {
    const now = Math.round(event.time_s * 1e6);
    this.advanceUs(now - 1);
    if (!this.deadlineUs) this.deadlineUs = (Math.ceil(now / 60000) + 2) * 60000;
    this.pending = event;
  }

  advanceUs(now) {
    if (!this.deadlineUs || now < this.deadlineUs) return;
    this.last = { ...this.pending, broadcast_s: this.deadlineUs / 1e6 };
    this.count++;
    this.deadlineUs = 0;
    this.pending = null;
  }
}

export class CubeListeners {
  constructor() {
    this.previousTapCount = null;
    this.tapEvents = 0;
    this.motionTransitions = 0;
    this.lastTapTime = null;
    this.lastMotionTime = null;
    this.axis = 0;
    this.moving = false;
    this.score = 0;
    this.low = null;
    this.high = new Float32Array(3);
    this.previous = null;
  }

  sample(slot) {
    const accel = Float32Array.from(slot, count => f32(f32(count / 32767) * 4) * 9810);
    if (this.previous === null) {
      this.low = accel.slice();
      this.previous = accel;
      return;
    }
    let strongest = 0;
    for (let i = 0; i < 3; i++) {
      this.low[i] = f32(accel[i] * f32(.03)) + f32(this.low[i] * f32(.97));
      this.high[i] = f32(.8) * f32(f32(this.high[i] + accel[i]) - this.previous[i]);
      if (Math.abs(this.low[i]) > Math.abs(this.low[strongest])) strongest = i;
    }
    const dominant = this.low[strongest];
    if (dominant !== 0) this.axis = 1 + 2 * strongest + Number(dominant > 0);
    const strength = Math.min(500, f32(Math.hypot(...this.high)));
    this.score = Math.min(20000, Math.max(0, f32(this.score + strength) - 300));
    if (this.score > 10000) this.moving = true;
    else if (this.score > 0 && this.score < 5000) this.moving = false;
    this.previous = accel;
  }

  receive(packet) {
    const events = [];
    if (this.previousTapCount !== null && packet.taps !== this.previousTapCount) {
      this.tapEvents++;
      this.lastTapTime = packet.time_s;
      events.push({ tap_count: packet.taps, time_s: packet.time_s });
    }
    this.previousTapCount = packet.taps;
    for (const slot of packet.slots) {
      const axis = this.axis, moving = this.moving;
      this.sample(slot);
      if (axis && this.axis !== axis) events.push({ axis: this.axis, time_s: packet.time_s });
      if (moving !== this.moving) {
        this.motionTransitions++;
        this.lastMotionTime = packet.time_s;
        events.push({ moving: this.moving, time_s: packet.time_s });
      }
    }
    return events;
  }
}
