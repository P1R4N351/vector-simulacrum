import { writeBodyRange } from "./prox.js";
// WireOS v6 body datagram layout, checked against vic_bridge_protocol.h.
const MAGIC = 0x56494342;
const ENCODER_SCALE = [0.96 * 29 * Math.PI / (4 * 172.3), -0.96 * 29 * Math.PI / (4 * 172.3),
  Math.PI / (4 * 149.7), Math.PI / (4 * 366.211)];

export function motorCommand(bytes) {
  if (bytes.byteLength !== 132) return null;
  const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (v.getUint32(0, true) !== MAGIC || v.getUint32(4, true) !== 6) return null;
  return ENCODER_SCALE.map((_, i) => Math.max(-1, v.getInt16(24 + 2*i, true) / 32767));
}

// Wheels infer direction from drive duty. Head and lift decode quadrature on the shaft.
class WireEncoder {
  constructor(quadrature) {
    this.quadrature = quadrature;
    this.previous = null;
    this.position = 0;
    this.direction = -1;
    this.delta = 0;
    this.period = 0;
    this.idleFrames = 0;
    this.lastEdgeClock = 0;
  }

  sample(coordinate, duty, startSeconds, dt) {
    if (duty !== 0) this.direction = Math.sign(duty);
    if (this.previous === null) {
      this.position = Math.floor(coordinate + 0.5);
      this.lastEdgeClock = Math.round(startSeconds * 48000000);
    } else {
      const step = coordinate - this.previous;
      const index = Math.floor(coordinate + 0.5);
      const edges = Math.abs(index - Math.floor(this.previous + 0.5));
      if (edges) {
        const boundary = index - 0.5 * Math.sign(step);
        const fraction = (boundary - this.previous) / step;
        const clock = Math.round((startSeconds + dt * fraction) * 48000000);
        this.period = ((clock - this.lastEdgeClock) * 256) >>> 0;
        this.lastEdgeClock = clock;
        this.delta = (this.quadrature ? Math.sign(step) : this.direction) * edges;
        this.position += this.delta;
        this.idleFrames = 0;
      } else if (++this.idleFrames > 25) {
        this.delta = 0;
      }
    }
    this.previous = coordinate;
  }
}

export class WireMotorEncoders {
  constructor() {
    this.channels = ENCODER_SCALE.map((_, i) => new WireEncoder(i >= 2));
  }

  sample(sim, power, startSeconds, dt) {
    const positions = [...sim._treadTravelMm(), sim.liftAngleRad(), sim.headAngleRad()];
    this.channels.forEach((encoder, i) => {
      encoder.sample(positions[i] / ENCODER_SCALE[i], power[i], startSeconds, dt);
    });
  }
}

export class WireBody {
  constructor(sim) {
    this.sim = sim;
    sim.bodyRangeClock = true;
    this.seq = 0;
    this.encoders = new WireMotorEncoders();
    this.cubeTick = sim.cubeSensor?.ticks ?? 0;
    this.cubeTaps = sim.cubeSensor?.packet?.taps ?? 0;
    this.lastAdvertisementSeq = null;
    this.cubeSensor = sim.cubeSensor;
  }

  frame(power) {
    const s = this.sim;
    const startSeconds = s.data.time;
    if (this.encoders.channels[0].previous === null) {
      this.encoders.sample(s, power, startSeconds, 0);
    }
    s.tickWireBody(power);
    const bytes = new Uint8Array(812), v = new DataView(bytes.buffer);
    for (const [offset, value] of [[0,MAGIC], [4,6], [8,this.seq], [16,this.seq]]) v.setUint32(offset,value,true);
    v.setUint8(20,1);
    this.encoders.sample(s, power, startSeconds, s.dt);
    for (let i=0; i<4; i++) {
      const encoder = this.encoders.channels[i];
      v.setInt32(24+12*i, encoder.position, true);
      v.setInt32(28+12*i, encoder.delta, true);
      v.setUint32(32+12*i, encoder.period, true);
    }
    s.cliffRangesMm().forEach((mm,i) => v.setUint16(72+2*i,mm<0 ? 4 : Math.round(Math.max(4,900-4.48*mm)),true));
    v.setInt16(80,Math.trunc(s.batteryVolts*2048/2.8),true);
    v.setInt16(82,s.chargerContact ? Math.trunc(5*2048/2.8) : 0,true);
    v.setInt16(84,25,true);
    v.setUint16(86,s.chargerContact ? 3 : 0,true);
    writeBodyRange(v, 92, s.rawRangeSample());
    for (const offset of [112,114]) v.setUint16(offset,(this.seq&1) ? 0 : 65535,true);
    const [accel, gyro] = s._imu();
    [...gyro,...accel,70-50*Math.exp(-.0032*s.tickCount*s.dt)].forEach((x,i) => v.setFloat32(784+4*i,x,true));
    this.seq = (this.seq+1)>>>0;
    return bytes;
  }

  // simCubeClient assembles three consecutive v1 messages into a single BLE report.
  cubeFrames() {
    const sensor = this.sim.cubeSensor;
    if (!sensor) return [];
    if (sensor !== this.cubeSensor) {
      this.cubeSensor = sensor;
      this.cubeTick = this.cubeTaps = 0;
    }
    // The unit 3 shelf cell reported 0x01bc in direct BLE captures. Other scenes
    // use the upstream LightCube PROTO default (1.5 V, truncated ADC counts).
    const railCounts = this.sim.shelfOutline ? 444 : 426;
    const frames = [];
    if ((this.seq - 1) % 200 === 0 && this.lastAdvertisementSeq !== this.seq) {
      frames.push(cubeFrame([0, 0, 0], true, false, railCounts));
      this.lastAdvertisementSeq = this.seq;
    }
    // The sensor only retains its latest three slots after a delivery interruption.
    this.cubeTick = Math.max(this.cubeTick, sensor.ticks - sensor.slots.length);
    while (this.cubeTick < sensor.ticks) {
      const slot = sensor.slots[this.cubeTick % 3];
      this.cubeTick++;
      // tap_count belongs to the completed three-slot report, including its wrap.
      const reportEnd = this.cubeTick % 3 === 0;
      const tapped = reportEnd && sensor.packet.taps !== this.cubeTaps;
      if (reportEnd) this.cubeTaps = sensor.packet.taps;
      frames.push(cubeFrame(slot, false, tapped, railCounts));
    }
    return frames;
  }
}

function cubeFrame(slot, advertised, tapped, railCounts) {
  const bytes = new Uint8Array(40), v = new DataView(bytes.buffer);
  v.setUint32(0,0x56494343,true);
  v.setUint32(4,1,true);
  bytes[8] = advertised ? 1 : 2;
  bytes.set(new TextEncoder().encode("AA:BB:CC:DD:EE:01"),9);
  slot.forEach((counts,i) => v.setInt16(30+2*i,counts,true));
  bytes[36] = tapped ? 1 : 0;
  v.setUint16(38,railCounts,true);
  return bytes;
}
