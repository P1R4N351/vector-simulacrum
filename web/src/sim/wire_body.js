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

export class WireBody {
  constructor(sim) {
    this.sim = sim;
    sim.bodyRangeClock = true;
    this.seq = 0;
    this.prev = null;
    this.elapsed = [0, 0, 0, 0];
    this.delta = [0, 0, 0, 0];
    this.period = [0, 0, 0, 0];
  }

  frame(power) {
    const s = this.sim;
    s.tickWireBody(power);
    const bytes = new Uint8Array(812), v = new DataView(bytes.buffer);
    for (const [offset, value] of [[0,MAGIC], [4,6], [8,this.seq], [16,this.seq]]) v.setUint32(offset,value,true);
    v.setUint8(20,1);
    const positions = [...s._treadTravelMm(), s.liftAngleRad(), s.headAngleRad()];
    const counts = positions.map((value,i) => Math.sign(value/ENCODER_SCALE[i]) * Math.round(Math.abs(value/ENCODER_SCALE[i])));
    for (let i=0; i<4; i++) {
      if (this.prev) {
        this.elapsed[i] += Math.round(s.dt * 48000000 * 256);
        if (counts[i] !== this.prev[i]) {
          this.delta[i] = counts[i] - this.prev[i];
          this.period[i] = this.elapsed[i];
          this.elapsed[i] = 0;
        } else if (this.elapsed[i] >= 1536000000) {
          this.delta[i] = this.period[i] = 0;
          this.elapsed[i] = 1536000000;
        }
      }
      v.setInt32(24+12*i, counts[i], true);
      v.setInt32(28+12*i, this.delta[i], true);
      v.setUint32(32+12*i, this.period[i], true);
    }
    this.prev = counts;
    s.cliffRangesMm().forEach((mm,i) => v.setUint16(72+2*i,mm<0 ? 4 : Math.round(Math.max(4,900-4.48*mm)),true));
    v.setInt16(80,Math.trunc(s.batteryVolts*2048/2.8),true);
    v.setInt16(82,s.chargerContact ? Math.trunc(5*2048/2.8) : 0,true);
    v.setInt16(84,25,true);
    v.setUint16(86,s.chargerContact ? 3 : 0,true);
    const prox = s.rawRangeSample();
    // Wire HAL decodes 9.7 rate and 8.8 SPAD count. Engine takes their ratio.
    const spads = 90;
    v.setUint8(92,0x58);
    v.setUint16(94, Math.max(0,Math.min(65535,Math.round(prox.raw))));
    v.setUint16(96,Math.min(65535,Math.round(prox.quality * spads * 128)));
    v.setUint16(100,spads*256);
    v.setUint16(102,prox.count,true);
    for (const offset of [112,114]) v.setUint16(offset,(this.seq&1) ? 0 : 65535,true);
    const [accel, gyro] = s._imu();
    [...gyro,...accel,70-50*Math.exp(-.0032*s.tickCount*s.dt)].forEach((x,i) => v.setFloat32(784+4*i,x,true));
    this.seq = (this.seq+1)>>>0;
    return bytes;
  }
}

export function cubeFrame(sim, advertised, tapped) {
  const bytes = new Uint8Array(40), v = new DataView(bytes.buffer);
  v.setUint32(0,0x56494343,true);
  v.setUint32(4,1,true);
  bytes[8] = advertised ? 1 : 2;
  bytes.set(new TextEncoder().encode("AA:BB:CC:DD:EE:01"),9);
  sim._cubeForce().forEach((g,i) => v.setInt16(30+2*i,Math.max(-32768,Math.min(32767,Math.round(g*32767/4))),true));
  bytes[36] = tapped ? 1 : 0;
  v.setUint16(38,Math.trunc(1.5*1024/3.6),true);
  return bytes;
}
