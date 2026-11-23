// ARMv6-M 16-bit Thumb interpreter for running real Cortex-M0 firmware images
// (Dialog DA14580 light-cube app) in browser and Node without native addons.

export const SP = 13;
export const LR = 14;
export const PC = 15;
const RETURN_SENTINEL = 0xFFFFFFFE;

export class ThumbError extends Error {
  constructor(message) {
    super(message);
    this.name = "ThumbError";
  }
}

function signExtend(value, bits) {
  const mask = (1 << bits) - 1;
  const masked = value & mask;
  const signBit = 1 << (bits - 1);
  return (masked & signBit) ? (masked - (1 << bits)) : masked;
}

export class ThumbCpu {
  constructor({ ramBase = 0x20000000, ramSize = 0xA800, maxSteps = 200000 } = {}) {
    this.ramBase = ramBase >>> 0;
    this.ram = new Uint8Array(ramSize);
    this.view = new DataView(this.ram.buffer);
    this.r = new Uint32Array(16);
    this.n = false;
    this.z = false;
    this.c = false;
    this.v = false;
    this.primask = 0;
    this.maxSteps = maxSteps;
    this.steps = 0;
    this.hooks = new Map();
    this.mmio = [];
    this.readable = [];
    this.writable = [];
  }

  load(address, bytes) {
    const start = (address >>> 0) - this.ramBase;
    if (start < 0 || start + bytes.length > this.ram.length) {
      throw new ThumbError(`load out of RAM range at 0x${(address >>> 0).toString(16)}`);
    }
    this.ram.set(bytes, start);
  }

  addMmio(lo, hi, readFn, writeFn) {
    this.mmio.push({ lo: lo >>> 0, hi: hi >>> 0, read: readFn, write: writeFn });
  }

  _findMmio(address) {
    const addr = address >>> 0;
    for (const dev of this.mmio) {
      if (addr >= dev.lo && addr < dev.hi) return dev;
    }
    return null;
  }

  _inRanges(ranges, address, size) {
    if (ranges.length === 0) return true;
    const loA = address >>> 0;
    const hiA = loA + size;
    return ranges.some(([lo, hi]) => loA >= lo && hiA <= hi);
  }

  read(address, size) {
    const addr = address >>> 0;
    const dev = this._findMmio(addr);
    if (dev) return dev.read(addr - dev.lo, size) >>> 0;
    if (!this._inRanges(this.readable, addr, size)) {
      throw new ThumbError(`read of ${size}B outside readable ranges at 0x${addr.toString(16)}`);
    }
    const off = addr - this.ramBase;
    if (off < 0 || off + size > this.ram.length) {
      throw new ThumbError(`unmapped read of ${size}B at 0x${addr.toString(16)}`);
    }
    if (size === 1) return this.view.getUint8(off);
    if (size === 2) return this.view.getUint16(off, true);
    if (size === 4) return this.view.getUint32(off, true);
    throw new ThumbError(`bad read size ${size}`);
  }

  write(address, size, value) {
    const addr = address >>> 0;
    const dev = this._findMmio(addr);
    if (dev) {
      dev.write(addr - dev.lo, size, value >>> 0);
      return;
    }
    if (!this._inRanges(this.writable, addr, size)) {
      throw new ThumbError(`write of ${size}B outside writable ranges at 0x${addr.toString(16)}`);
    }
    const off = addr - this.ramBase;
    if (off < 0 || off + size > this.ram.length) {
      throw new ThumbError(`unmapped write of ${size}B at 0x${addr.toString(16)}`);
    }
    if (size === 1) this.view.setUint8(off, value & 0xFF);
    else if (size === 2) this.view.setUint16(off, value & 0xFFFF, true);
    else if (size === 4) this.view.setUint32(off, value >>> 0, true);
    else throw new ThumbError(`bad write size ${size}`);
  }

  _push(value) {
    this.r[SP] = (this.r[SP] - 4) >>> 0;
    this.write(this.r[SP], 4, value >>> 0);
  }

  _pop() {
    const val = this.read(this.r[SP], 4);
    this.r[SP] = (this.r[SP] + 4) >>> 0;
    return val;
  }

  _nz(value) {
    const u = value >>> 0;
    this.n = (u & 0x80000000) !== 0;
    this.z = u === 0;
    return u;
  }

  _add(a, b, carry = 0) {
    const ua = a >>> 0, ub = b >>> 0;
    const total = ua + ub + (carry ? 1 : 0);
    const res = this._nz(total >>> 0);
    this.c = total > 0xFFFFFFFF;
    this.v = (((ua | 0) ^ (res | 0)) & ((ub | 0) ^ (res | 0))) < 0;
    return res;
  }

  _sub(a, b) {
    return this._add(a, (~b) >>> 0, 1);
  }

  _cond(cond) {
    switch (cond) {
      case 0: return this.z;
      case 1: return !this.z;
      case 2: return this.c;
      case 3: return !this.c;
      case 4: return this.n;
      case 5: return !this.n;
      case 6: return this.v;
      case 7: return !this.v;
      case 8: return this.c && !this.z;
      case 9: return !this.c || this.z;
      case 10: return this.n === this.v;
      case 11: return this.n !== this.v;
      case 12: return !this.z && this.n === this.v;
      case 13: return this.z || this.n !== this.v;
      default: throw new ThumbError(`bad condition code ${cond}`);
    }
  }

  call(address, args = [], sp) {
    if (args.length > 4 || (sp & 3) !== 0) {
      throw new ThumbError("Thumb call accepts <= 4 args and 4-byte aligned SP");
    }
    this.r.fill(0);
    for (let i = 0; i < args.length; i++) this.r[i] = args[i] >>> 0;
    this.r[SP] = sp >>> 0;
    this.r[LR] = RETURN_SENTINEL | 1;
    this.r[PC] = address & ~1;
    for (let i = 0; i < this.maxSteps; i++) {
      if (this.r[PC] === RETURN_SENTINEL) return this.r[0] >>> 0;
      const hook = this.hooks.get(this.r[PC]);
      if (hook) {
        hook(this);
        this.r[PC] = this.r[LR] & ~1;
        continue;
      }
      this.step();
    }
    throw new ThumbError(`call to 0x${address.toString(16)} exceeded ${this.maxSteps} steps`);
  }

  step() {
    this.steps += 1;
    const pc = this.r[PC];
    const op = this.read(pc, 2);
    this.r[PC] = (pc + 2) >>> 0;
    if (this._shiftAddSub(op) || this._imm8(op) || this._dataProc(op, pc)
      || this._hiBxLit(op, pc) || this._loadStore(op) || this._multiLoadStore(op)
      || this._misc(op) || this._branch(op, pc)) {
      return;
    }
    throw new ThumbError(`unhandled Thumb op 0x${op.toString(16).padStart(4, "0")} at 0x${pc.toString(16)}`);
  }

  _shiftAddSub(op) {
    const top = op >> 11;
    if (top > 0b00011) return false;
    const rd = op & 7, rm = (op >> 3) & 7, imm5 = (op >> 6) & 31;
    if (top === 0b00000) {
      let v = this.r[rm];
      if (imm5) {
        this.c = ((v >>> (32 - imm5)) & 1) !== 0;
        v = (v << imm5) >>> 0;
      }
      this.r[rd] = this._nz(v);
      return true;
    }
    if (top === 0b00001 || top === 0b00010) {
      const shift = imm5 || 32;
      const v = this.r[rm];
      if (top === 0b00001) {
        this.c = ((v >>> (shift - 1)) & 1) !== 0;
        this.r[rd] = this._nz(shift >= 32 ? 0 : (v >>> shift));
      } else {
        const s = v | 0;
        this.c = ((s >> Math.min(31, shift - 1)) & 1) !== 0;
        this.r[rd] = this._nz(shift >= 32 ? (s < 0 ? 0xFFFFFFFF : 0) : (s >> shift));
      }
      return true;
    }
    const rn = (op >> 3) & 7, field = (op >> 6) & 7;
    const operand = (op & (1 << 10)) ? field : this.r[field];
    this.r[rd] = (op & (1 << 9)) ? this._sub(this.r[rn], operand) : this._add(this.r[rn], operand);
    return true;
  }

  _imm8(op) {
    if ((op >> 13) !== 0b001) return false;
    const kind = (op >> 11) & 3, rd = (op >> 8) & 7, imm = op & 0xFF;
    if (kind === 0) this.r[rd] = this._nz(imm);
    else if (kind === 1) this._sub(this.r[rd], imm);
    else if (kind === 2) this.r[rd] = this._add(this.r[rd], imm);
    else this.r[rd] = this._sub(this.r[rd], imm);
    return true;
  }

  _shiftReg(code, value, shift) {
    if (shift === 0) return this._nz(value);
    if (code === 0b0010) {
      this.c = shift <= 32 && ((value >>> (32 - shift)) & 1) !== 0;
      return this._nz(shift < 32 ? (value << shift) : 0);
    }
    if (code === 0b0011) {
      this.c = shift <= 32 && ((value >>> (shift - 1)) & 1) !== 0;
      return this._nz(shift < 32 ? (value >>> shift) : 0);
    }
    const s = value | 0;
    const clamped = Math.min(shift, 32);
    this.c = ((s >> Math.min(31, clamped - 1)) & 1) !== 0;
    return this._nz(clamped >= 32 ? (s < 0 ? 0xFFFFFFFF : 0) : (s >> clamped));
  }

  _dataProc(op, pc) {
    if ((op >> 10) !== 0b010000) return false;
    const code = (op >> 6) & 15, rm = (op >> 3) & 7, rd = op & 7;
    const a = this.r[rd], b = this.r[rm];
    switch (code) {
      case 0b0000: this.r[rd] = this._nz(a & b); break;
      case 0b0001: this.r[rd] = this._nz(a ^ b); break;
      case 0b0010: case 0b0011: case 0b0100: this.r[rd] = this._shiftReg(code, a, b & 0xFF); break;
      case 0b0101: this.r[rd] = this._add(a, b, this.c ? 1 : 0); break;
      case 0b0110: this.r[rd] = this._add(a, (~b) >>> 0, this.c ? 1 : 0); break;
      case 0b0111: {
        const s = b & 31;
        const rot = s === 0 ? a : ((a >>> s) | (a << (32 - s))) >>> 0;
        if (b & 0xFF) this.c = (rot & 0x80000000) !== 0;
        this.r[rd] = this._nz(rot);
        break;
      }
      case 0b1000: this._nz(a & b); break;
      case 0b1001: this.r[rd] = this._sub(0, b); break;
      case 0b1010: this._sub(a, b); break;
      case 0b1011: this._add(a, b); break;
      case 0b1100: this.r[rd] = this._nz(a | b); break;
      case 0b1101: this.r[rd] = this._nz(Math.imul(a, b)); break;
      case 0b1110: this.r[rd] = this._nz(a & ~b); break;
      case 0b1111: this.r[rd] = this._nz(~b); break;
      default: throw new ThumbError(`bad data proc ${code} at 0x${pc.toString(16)}`);
    }
    return true;
  }

  _hiBxLit(op, pc) {
    if ((op >> 10) === 0b010001) {
      const kind = (op >> 8) & 3, rm = (op >> 3) & 15, rd = (op & 7) | ((op >> 4) & 8);
      const val = rm === PC ? ((pc + 4) >>> 0) : this.r[rm];
      if (kind === 0) this.r[rd] = (this.r[rd] + val) >>> 0;
      else if (kind === 1) this._sub(this.r[rd], val);
      else if (kind === 2) {
        if (rd === PC) throw new ThumbError(`MOV to PC at 0x${pc.toString(16)}`);
        this.r[rd] = val;
      } else if (op & 0x80) {
        this.r[LR] = (pc + 2) | 1;
        this.r[PC] = this.r[rm] & ~1;
      } else {
        this.r[PC] = this.r[rm] & ~1;
      }
      return true;
    }
    if ((op >> 11) === 0b01001) {
      const rd = (op >> 8) & 7, base = ((pc + 4) & ~3) >>> 0;
      this.r[rd] = this.read((base + ((op & 0xFF) << 2)) >>> 0, 4);
      return true;
    }
    return false;
  }

  _loadStore(op) {
    if ((op >> 12) === 0b0101) {
      const kind = (op >> 9) & 7, rm = (op >> 6) & 7, rn = (op >> 3) & 7, rt = op & 7;
      const addr = (this.r[rn] + this.r[rm]) >>> 0;
      const sizes = [4, 2, 1, 1, 4, 2, 1, 2];
      const sz = sizes[kind];
      if (kind < 3) this.write(addr, sz, this.r[rt]);
      else if (kind === 3) this.r[rt] = signExtend(this.read(addr, 1), 8) >>> 0;
      else if (kind === 7) this.r[rt] = signExtend(this.read(addr, 2), 16) >>> 0;
      else this.r[rt] = this.read(addr, sz);
      return true;
    }
    const g = op >> 11, imm5 = (op >> 6) & 31, rn = (op >> 3) & 7, rt = op & 7;
    if (g >= 0b01100 && g <= 0b10001) {
      const sz = g <= 0b01101 ? 4 : g <= 0b01111 ? 1 : 2;
      const isLoad = (g & 1) === 1 && g !== 0b10000 ? (g !== 0b01110) : (g === 0b10001);
      const addr = (this.r[rn] + imm5 * sz) >>> 0;
      if (isLoad) this.r[rt] = this.read(addr, sz);
      else this.write(addr, sz, this.r[rt]);
      return true;
    }
    if (g === 0b10010 || g === 0b10011) {
      const rtSp = (op >> 8) & 7;
      const addr = (this.r[SP] + ((op & 0xFF) << 2)) >>> 0;
      if (g === 0b10011) this.r[rtSp] = this.read(addr, 4);
      else this.write(addr, 4, this.r[rtSp]);
      return true;
    }
    return false;
  }

  _multiLoadStore(op) {
    const g = op >> 11;
    if (g !== 0b11000 && g !== 0b11001) return false;
    const rn = (op >> 8) & 7, mask = op & 0xFF;
    let addr = this.r[rn];
    for (let i = 0; i < 8; i++) {
      if ((mask & (1 << i)) === 0) continue;
      if (g === 0b11000) this.write(addr, 4, this.r[i]);
      else this.r[i] = this.read(addr, 4);
      addr = (addr + 4) >>> 0;
    }
    if (g === 0b11000 || (mask & (1 << rn)) === 0) this.r[rn] = addr;
    return true;
  }

  _misc(op) {
    const g = op >> 11;
    if (g === 0b10100) {
      const pcAligned = ((this.r[PC] + 2) & ~3) >>> 0;
      this.r[(op >> 8) & 7] = (pcAligned + ((op & 0xFF) << 2)) >>> 0;
      return true;
    }
    if (g === 0b10101) {
      this.r[(op >> 8) & 7] = (this.r[SP] + ((op & 0xFF) << 2)) >>> 0;
      return true;
    }
    if ((op >> 8) === 0b10110000) {
      const imm = (op & 0x7F) << 2;
      this.r[SP] = ((op & 0x80) ? (this.r[SP] - imm) : (this.r[SP] + imm)) >>> 0;
      return true;
    }
    if ((op >> 8) === 0b10110010) {
      const kind = (op >> 6) & 3, rm = (op >> 3) & 7, rd = op & 7;
      if (kind === 0) this.r[rd] = signExtend(this.r[rm], 16) >>> 0;
      else if (kind === 1) this.r[rd] = signExtend(this.r[rm], 8) >>> 0;
      else if (kind === 2) this.r[rd] = this.r[rm] & 0xFFFF;
      else this.r[rd] = this.r[rm] & 0xFF;
      return true;
    }
    if ((op & 0xFFEF) === 0xB662) {
      this.primask = (op >> 4) & 1;
      return true;
    }
    if ((op >> 9) === 0b1011010) {
      const regs = [];
      for (let i = 0; i < 8; i++) if (op & (1 << i)) regs.push(i);
      if (op & 0x100) regs.push(LR);
      for (let i = regs.length - 1; i >= 0; i--) this._push(this.r[regs[i]]);
      return true;
    }
    if ((op >> 9) === 0b1011110) {
      for (let i = 0; i < 8; i++) if (op & (1 << i)) this.r[i] = this._pop();
      if (op & 0x100) this.r[PC] = this._pop() & ~1;
      return true;
    }
    return op === 0xBF00 || op === 0xBF30;
  }

  _branch(op, pc) {
    if ((op >> 12) === 0b1101) {
      const cond = (op >> 8) & 15;
      if (cond >= 14) throw new ThumbError(`UDF or SVC at 0x${pc.toString(16)}`);
      if (this._cond(cond)) this.r[PC] = (pc + 4 + (signExtend(op & 0xFF, 8) << 1)) >>> 0;
      return true;
    }
    if ((op >> 11) === 0b11100) {
      this.r[PC] = (pc + 4 + (signExtend(op & 0x7FF, 11) << 1)) >>> 0;
      return true;
    }
    if ((op >> 11) === 0b11110) {
      const low = this.read(pc + 2, 2);
      if ((low >> 14) !== 0b11 || (low & 0x1000) === 0) {
        throw new ThumbError(`not a BL at 0x${pc.toString(16)}`);
      }
      const s = (op >> 10) & 1;
      const i1 = 1 - (((low >> 13) & 1) ^ s), i2 = 1 - (((low >> 11) & 1) ^ s);
      const off = (s << 24) | (i1 << 23) | (i2 << 22) | ((op & 0x3FF) << 12) | ((low & 0x7FF) << 1);
      this.r[LR] = (pc + 4) | 1;
      this.r[PC] = (pc + 4 + signExtend(off, 25)) >>> 0;
      return true;
    }
    return false;
  }
}
