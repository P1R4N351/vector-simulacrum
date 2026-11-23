// Execute the real Dialog DA14580 Light Cube binary (cube.plain.bin) on ThumbCpu,
// modeling CRG, GP_ADC, GPIO + bit-banged BMA253 3-wire SPI FIFO, Timer0 SWTIM, and 12 LED pins.

import { ThumbCpu } from "./thumb.js";
import { assert } from "./util.js";

const APP_BASE = 0x20005800;
const APP_TABLE = 0x20005810;
const STACK_TOP = 0x2000A7F0;
const SEND_HOOK = 0x20009800;
const RECV_BUF = 0x20009E10;
const IRQ_VECTORS_8 = 0x20000060;
const TIMER0_CYCLES_PER_TICK = 80000; // 10 ms at 8 MHz fast clock (/2 divider)
const MAX_TIMER0_EXPIRIES = 64;
const FIFO_LIMIT = 32;

// app/lights.c pin_assignment order: blue 0..3, green 0..3, red 0..3
const LED_PINS = [
  [2, 1], [2, 7], [2, 5], [2, 4],
  [0, 7], [2, 8], [1, 0], [1, 1],
  [2, 2], [2, 9], [2, 6], [2, 3],
];
// Maps each LED_PINS index into ledRgb[12] = [r0,g0,b0, r1,g1,b1, r2,g2,b2, r3,g3,b3].
const LED_CHANNEL = [2, 5, 8, 11, 1, 4, 7, 10, 0, 3, 6, 9];

function decodeGpio(offset) {
  if (offset < 0x60) {
    const port = Math.floor(offset / 0x20);
    const reg = offset % 0x20;
    if (port <= 2 && (reg <= 4 || (reg >= 6 && reg < 26))) return [port, reg];
  } else if (offset >= 0x80 && offset < 0xA0) {
    return [3, offset - 0x80];
  }
  return null;
}

class Bma253Spi {
  constructor() {
    this.fifo = [];
    this.regs = new Uint8Array(0x80);
    this.regs[0] = 0xFA;
    this.csLow = false;
    this.sck = true;
    this.sdaOut = false;
    this.haveAddress = false;
    this.reading = false;
    this.bitCount = 0;
    this.shift = 0;
    this.address = 0;
    this.fifoByte = 0;
    this.readByte = 0;
    this.transactions = 0;
    this.framesRead = 0;
  }

  pushFrame(xyz) {
    if (this.fifo.length < FIFO_LIMIT) {
      this.fifo.push([(xyz[0] << 16) >> 16, (xyz[1] << 16) >> 16, (xyz[2] << 16) >> 16]);
    }
  }

  onPort0(before, after) {
    const csNow = (after & (1 << 0)) === 0;
    const sckNow = (after & (1 << 3)) !== 0;
    if (csNow && !this.csLow) {
      this.bitCount = this.shift = 0;
      this.haveAddress = this.reading = this.sdaOut = false;
    }
    if (!csNow && this.csLow) this.transactions += 1;
    this.csLow = csNow;
    if (this.csLow && sckNow !== this.sck) {
      if (sckNow) this._risingEdge((after & (1 << 6)) !== 0);
      else if (this.reading && this.haveAddress) this.sdaOut = (this.readByte & (0x80 >> this.bitCount)) !== 0;
    }
    this.sck = sckNow;
  }

  _risingEdge(sdaIn) {
    if (this.reading && this.haveAddress) {
      this.bitCount += 1;
      if (this.bitCount === 8) {
        this.bitCount = 0;
        if (this.address === 0x3F) {
          this.fifoByte += 1;
          if (this.fifoByte === 6) {
            this.fifoByte = 0;
            if (this.fifo.length > 0) {
              this.fifo.shift();
              this.framesRead += 1;
            }
          }
        } else {
          this.address = (this.address + 1) & 0x7F;
        }
        this._loadReadByte();
      }
      return;
    }
    this.shift = ((this.shift << 1) | (sdaIn ? 1 : 0)) & 0xFF;
    this.bitCount += 1;
    if (this.bitCount < 8) return;
    this.bitCount = 0;
    const b = this.shift;
    this.shift = 0;
    if (!this.haveAddress) {
      this.haveAddress = true;
      this.reading = (b & 0x80) !== 0;
      this.address = b & 0x7F;
      if (this.reading) this._loadReadByte();
      return;
    }
    this._writeReg(this.address, b);
    this.address = (this.address + 1) & 0x7F;
  }

  _loadReadByte() {
    if (this.address === 0x3F) {
      if (this.fifo.length === 0) { this.readByte = 0; return; }
      const val = this.fifo[0][Math.floor(this.fifoByte / 2)] & 0xFFFF;
      this.readByte = (this.fifoByte & 1) ? (val >> 8) : (val & 0xFF);
      return;
    }
    if (this.address === 0x0E) {
      this.readByte = Math.min(this.fifo.length, 0x7F);
      return;
    }
    this.readByte = this.regs[this.address];
  }

  _writeReg(reg, value) {
    if (reg === 0x14 && value === 0xB6) {
      this.regs.fill(0);
      this.regs[0] = 0xFA;
      this.fifo.length = 0;
      this.fifoByte = 0;
      return;
    }
    if (reg === 0x3E) {
      this.fifo.length = 0;
      this.fifoByte = 0;
    }
    this.regs[reg] = value;
  }
}

export class CubeFirmwareVm {
  constructor(imageBytes, { railCounts = 426 } = {}) {
    assert(imageBytes && imageBytes.length > 0x40 && imageBytes.length <= 0x27C0, "invalid cube.plain.bin size");
    this.version = new TextDecoder().decode(imageBytes.subarray(0, 16));
    this.railCounts = railCounts;
    this.cpu = new ThumbCpu({ ramBase: 0x20000000, ramSize: 0xA800, maxSteps: 250000 });
    this.cpu.load(APP_BASE, imageBytes);
    this.bma = new Bma253Spi();
    this.outLatch = new Uint16Array(4);
    this.mode = Array.from({ length: 4 }, () => new Uint16Array(10));
    this.padpwr = new Uint16Array(3);
    this.crg = new Uint32Array(0x40);
    this.gpadc = new Uint16Array(16);
    this.timerCtrl = 0;
    this.timerOn = 0;
    this.timerLimit = 40000;
    this.timerRem = 40000;
    this.nvicIser = 0;
    this.packets = [];
    this.ledRgb = new Array(12).fill(0);
    this._installMmio();
    this.cpu.view.setUint32(APP_TABLE - 0x20000000 + 16, SEND_HOOK | 1, true);
    this.cpu.hooks.set(SEND_HOOK, (c) => {
      const len = c.r[0], ptr = c.r[1];
      const pkt = new Uint8Array(len);
      for (let i = 0; i < len; i++) pkt[i] = c.read(ptr + i, 1);
      this.packets.push(pkt);
    });
    this.fnMain = this.cpu.view.getUint32(APP_TABLE - 0x20000000, true) & ~1;
    this.fnTick = this.cpu.view.getUint32(APP_TABLE - 0x20000000 + 8, true) & ~1;
    this.fnRecv = this.cpu.view.getUint32(APP_TABLE - 0x20000000 + 12, true) & ~1;
    this.cpu.call(this.fnMain, [], STACK_TOP);
    const failed = this.packets.find((p) => p[0] === 3);
    assert(!failed, "cube firmware reported COMMAND_ACCEL_FAILURE");
    this.ticks = 0;
  }

  _installMmio() {
    this.cpu.addMmio(0x50000000, 0x50000100,
      (off) => this.crg[off >> 2] ?? 0,
      (off, _, v) => { this.crg[off >> 2] = v; });
    this.cpu.addMmio(0x50001500, 0x50001520,
      (off) => (off === 0xA ? ((this.gpadc[0] & 0x1000) ? 0x200 : this.railCounts) : (this.gpadc[off >> 1] ?? 0)),
      (off, _, v) => { this.gpadc[off >> 1] = off === 0 ? (v & ~0x0002) : (v & 0xFFFF); });
    this.cpu.addMmio(0x50003000, 0x500030A0,
      (off) => this._readGpio(off),
      (off, _, v) => this._writeGpio(off, v & 0xFFFF));
    this.cpu.addMmio(0x50003400, 0x50003410,
      (off) => (off === 0 ? this.timerCtrl : off === 2 ? this.timerOn : 0),
      (off, _, v) => {
        if (off === 0) {
          const wasOn = (this.timerCtrl & 1) !== 0;
          this.timerCtrl = v & 0xFFFF;
          if (!wasOn && (this.timerCtrl & 1)) {
            this.timerLimit = Math.max(1, this.timerOn);
            this.timerRem = this.timerLimit;
          }
        } else if (off === 2) {
          this.timerOn = v & 0xFFFF;
        }
      });
    this.cpu.addMmio(0xE000E100, 0xE000E420,
      (off) => (off === 0 ? this.nvicIser : 0),
      (off, _, v) => {
        if (off === 0) this.nvicIser |= v >>> 0;
        else if (off === 0x80) this.nvicIser &= ~(v >>> 0);
      });
  }

  _pinLevels(port) {
    let val = 0;
    for (let pin = 0; pin < 10; pin++) {
      const m = (this.mode[port][pin] >> 8) & 3;
      const lvl = m === 3 ? ((this.outLatch[port] & (1 << pin)) !== 0)
        : (port === 0 && pin === 6) ? this.bma.sdaOut : (m === 1);
      if (lvl) val |= (1 << pin);
    }
    return val;
  }

  _readGpio(off) {
    const dec = decodeGpio(off);
    if (!dec) return (off >= 0x70 && off < 0x76) ? this.padpwr[(off - 0x70) >> 1] : 0;
    const [port, reg] = dec;
    if (reg === 0) return this._pinLevels(port);
    if (reg >= 6) return this.mode[port][(reg - 6) >> 1];
    return 0;
  }

  _writeGpio(off, value) {
    const dec = decodeGpio(off);
    if (!dec) {
      if (off >= 0x70 && off < 0x76) this.padpwr[(off - 0x70) >> 1] = value;
      return;
    }
    const [port, reg] = dec;
    const before = this._pinLevels(port);
    if (reg === 0) this.outLatch[port] = value;
    else if (reg === 2) this.outLatch[port] |= value;
    else if (reg === 4) this.outLatch[port] &= ~value;
    else if (reg >= 6 && ((reg - 6) >> 1) < 10) this.mode[port][(reg - 6) >> 1] = value;
    if (port === 0) {
      const after = this._pinLevels(0);
      if (before !== after) this.bma.onPort0(before, after);
    }
  }

  pushFrame(xyz) {
    this.bma.pushFrame(xyz);
  }

  tick() {
    const emittedBefore = this.packets.length;
    this.cpu.call(this.fnTick, [], STACK_TOP);
    this._stepLeds();
    this.ticks += 1;
    return this.packets.slice(emittedBefore);
  }

  _stepLeds() {
    if ((this.timerCtrl & 1) === 0 || (this.nvicIser & (1 << 8)) === 0) return;
    const irq = this.cpu.view.getUint32(IRQ_VECTORS_8 - 0x20000000, true) & ~1;
    if (!irq) return;
    const lowCycles = new Float64Array(12);
    let left = TIMER0_CYCLES_PER_TICK;
    for (let i = 0; i < MAX_TIMER0_EXPIRIES && left > 0; i++) {
      const span = Math.min(left, Math.max(1, this.timerRem));
      for (let led = 0; led < 12; led++) {
        const [port, pin] = LED_PINS[led];
        if ((this._pinLevels(port) & (1 << pin)) === 0) lowCycles[led] += span;
      }
      left -= span;
      this.timerRem -= span;
      if (this.timerRem === 0) {
        this.timerLimit = Math.max(1, this.timerOn);
        this.timerRem = this.timerLimit;
        this.cpu.call(irq, [], STACK_TOP);
      }
    }
    for (let led = 0; led < 12; led++) {
      const duty = lowCycles[led] / TIMER0_CYCLES_PER_TICK;
      // lights.c duration = (intensity^2) >> 4 out of 40000 cycles per 5 ms frame
      this.ledRgb[LED_CHANNEL[led]] = Math.min(255, Math.round(Math.sqrt(duty * 40000 * 16)));
    }
  }

  setLights(rgb12) {
    assert(rgb12.length === 12, "setLights expects 12 RGB bytes for 4 LEDs");
    // Send two FrameCommands (indices 0..2 then 3) and one MapCommand to activate all 4 channels.
    const send = (bytes) => {
      this.cpu.load(RECV_BUF, bytes);
      this.cpu.call(this.fnRecv, [bytes.length, RECV_BUF], STACK_TOP);
    };
    const frame0 = new Uint8Array(20);
    frame0[0] = 1;
    frame0[1] = 0;
    for (let i = 0; i < 3; i++) {
      const off = 2 + 6 * i;
      frame0[off] = rgb12[3 * i];
      frame0[off + 1] = rgb12[3 * i + 1];
      frame0[off + 2] = rgb12[3 * i + 2];
      frame0[off + 3] = 255;
      frame0[off + 4] = 0;
      frame0[off + 5] = i;
    }
    send(frame0);
    const frame1 = new Uint8Array(20);
    frame1[0] = 1;
    frame1[1] = 3;
    frame1[2] = rgb12[9];
    frame1[3] = rgb12[10];
    frame1[4] = rgb12[11];
    frame1[5] = 255;
    frame1[7] = 3;
    send(frame1);
    send(new Uint8Array([0, 0, 0, 1, 2, 3]));
  }
}
