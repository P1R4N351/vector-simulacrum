"""A small ARMv6-M Thumb interpreter, enough to run the cube app's accelerometer tick. Authored-by: CPCS.

Why: the replacement tick (tick_avg.s) is hand-written and goes onto a cube, so the exact bytes are run here
before they are flashed, against the model in simulacrum/cube.py. The shipped tick runs here too; it must
agree with the model of the shipped app, which checks this interpreter and that model against each other.

It decodes the 16-bit Thumb instructions and BL that ARMv6-M has, with the flags as the ARMv6-M Architecture
Reference Manual defines them, and raises on anything it does not implement rather than guessing. Memory is
one flat little-endian space; reads of bytes never written return 0, and readable/writable, when set, bound
every access. Calls to addresses in `hooks` run a
Python function instead (it gets the interpreter, and returns as BX LR would). Execution is bounded.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Tuple

MASK = 0xFFFFFFFF
SP, LR, PC = 13, 14, 15
RETURN_SENTINEL = 0xFFFFFFFE  # the LR a top-level call returns to


class ThumbError(Exception):
    """An instruction or state the interpreter does not handle."""


def _sign(value: int, bits: int) -> int:
    value &= (1 << bits) - 1
    return value - (1 << bits) if value >> (bits - 1) else value


class Thumb:
    def __init__(self, max_steps: int = 200000) -> None:
        self.mem: Dict[int, int] = {}
        self.r: List[int] = [0] * 16
        self.n = self.z = self.c = self.v = False
        self.hooks: Dict[int, Callable[["Thumb"], None]] = {}
        self.max_steps = max_steps
        self.steps = 0
        # Address ranges [lo, hi) that code may read and write; empty means anywhere. An access outside them
        # raises, as a stray store on the cube would fault or corrupt something.
        self.readable: List[Tuple[int, int]] = []
        self.writable: List[Tuple[int, int]] = []

    # ---- memory -----------------------------------------------------------
    def load(self, address: int, data: bytes) -> None:
        for i, b in enumerate(data):
            self.mem[address + i] = b

    @staticmethod
    def _inside(ranges: List[Tuple[int, int]], address: int, size: int) -> bool:
        return not ranges or any(lo <= address and address + size <= hi for lo, hi in ranges)

    def read(self, address: int, size: int) -> int:
        if not self._inside(self.readable, address, size):
            raise ThumbError("read of %d bytes at %#x, outside the readable ranges" % (size, address))
        return int.from_bytes(bytes(self.mem.get(address + i, 0) for i in range(size)), "little")

    def write(self, address: int, size: int, value: int) -> None:
        if not self._inside(self.writable, address, size):
            raise ThumbError("write of %d bytes at %#x, outside the writable ranges" % (size, address))
        for i, b in enumerate((value & ((1 << (8 * size)) - 1)).to_bytes(size, "little")):
            self.mem[address + i] = b

    def push(self, value: int) -> None:
        self.r[SP] = (self.r[SP] - 4) & MASK
        self.write(self.r[SP], 4, value)

    # ---- flags ------------------------------------------------------------
    def _nz(self, result: int) -> int:
        result &= MASK
        self.n, self.z = bool(result >> 31), result == 0
        return result

    def _add(self, a: int, b: int, carry: int = 0) -> int:
        a &= MASK
        b &= MASK
        total = a + b + carry
        result = self._nz(total)
        self.c = total > MASK
        self.v = ((a ^ result) & (b ^ result)) >> 31 == 1
        return result

    def _sub(self, a: int, b: int) -> int:
        return self._add(a, (~b) & MASK, 1)

    def _cond(self, cond: int) -> bool:
        table = {0: self.z, 1: not self.z, 2: self.c, 3: not self.c, 4: self.n, 5: not self.n, 6: self.v,
                 7: not self.v, 8: self.c and not self.z, 9: (not self.c) or self.z, 10: self.n == self.v,
                 11: self.n != self.v, 12: (not self.z) and self.n == self.v, 13: self.z or self.n != self.v}
        if cond not in table:
            raise ThumbError("condition %d" % cond)
        return table[cond]

    # ---- running ----------------------------------------------------------
    def call(self, address: int, args: List[int], sp: int) -> int:
        """Call a Thumb function (address without the Thumb bit) with r0-r3 = args; returns r0."""
        assert len(args) <= 4 and sp % 4 == 0, "up to four arguments, aligned stack"
        self.r = [0] * 16
        for i, a in enumerate(args):
            self.r[i] = a & MASK
        self.r[SP], self.r[LR], self.r[PC] = sp, RETURN_SENTINEL | 1, address
        for _ in range(self.max_steps):
            if self.r[PC] == RETURN_SENTINEL:
                return self.r[0]
            if self.r[PC] in self.hooks:
                self.hooks[self.r[PC]](self)
                self.r[PC] = self.r[LR] & ~1
                continue
            self.step()
        raise ThumbError("no return within %d steps" % self.max_steps)

    def _branch_link(self, target: int, return_to: int) -> None:
        self.r[LR] = return_to | 1
        self.r[PC] = target & ~1

    def step(self) -> None:
        self.steps += 1
        pc = self.r[PC]
        op = self.read(pc, 2)
        self.r[PC] = pc + 2
        handlers = (self._shift_add_sub, self._imm8, self._data_processing, self._hi_bx_literal,
                    self._load_store, self._misc, self._branches)
        for handler in handlers:
            if handler(op, pc):
                return
        raise ThumbError("unhandled instruction %04x at %#x" % (op, pc))

    def _pcrel(self, pc: int) -> int:
        return (pc + 4) & ~3

    def _shift_add_sub(self, op: int, pc: int) -> bool:
        top = op >> 11
        rd, rm, imm5 = op & 7, (op >> 3) & 7, (op >> 6) & 31
        if top == 0b00000:  # LSLS imm (MOVS reg when imm5 is 0)
            value = self.r[rm]
            if imm5:
                self.c = bool((value >> (32 - imm5)) & 1)
                value = (value << imm5) & MASK
            self.r[rd] = self._nz(value)
            return True
        if top in (0b00001, 0b00010):  # LSRS / ASRS imm (imm5 of 0 means 32)
            shift = imm5 or 32
            value = self.r[rm]
            if top == 0b00010:
                value = _sign(value, 32)
            self.c = bool((value >> (shift - 1)) & 1)
            self.r[rd] = self._nz(value >> shift)
            return True
        if top == 0b00011:
            rn, field = (op >> 3) & 7, (op >> 6) & 7
            operand = field if op & (1 << 10) else self.r[field]
            sub = bool(op & (1 << 9))
            self.r[rd] = self._sub(self.r[rn], operand) if sub else self._add(self.r[rn], operand)
            return True
        return False

    def _imm8(self, op: int, pc: int) -> bool:
        if op >> 13 != 0b001:
            return False
        kind, rd, imm = (op >> 11) & 3, (op >> 8) & 7, op & 0xFF
        if kind == 0:
            self.r[rd] = self._nz(imm)
        elif kind == 1:
            self._sub(self.r[rd], imm)
        elif kind == 2:
            self.r[rd] = self._add(self.r[rd], imm)
        else:
            self.r[rd] = self._sub(self.r[rd], imm)
        return True

    def _data_processing(self, op: int, pc: int) -> bool:
        if op >> 10 != 0b010000:
            return False
        code, rm, rd = (op >> 6) & 15, (op >> 3) & 7, op & 7
        a, b = self.r[rd], self.r[rm]
        if code == 0b0000:
            self.r[rd] = self._nz(a & b)
        elif code == 0b0001:
            self.r[rd] = self._nz(a ^ b)
        elif code in (0b0010, 0b0011, 0b0100):  # LSLS, LSRS, ASRS by register
            shift = b & 0xFF
            self.r[rd] = self._shift_reg(code, a, shift)
        elif code == 0b0101:
            self.r[rd] = self._add(a, b, int(self.c))
        elif code == 0b0110:
            self.r[rd] = self._add(a, (~b) & MASK, int(self.c))
        elif code == 0b1000:
            self._nz(a & b)
        elif code == 0b1001:
            self.r[rd] = self._sub(0, b)
        elif code == 0b1010:
            self._sub(a, b)
        elif code == 0b1011:
            self._add(a, b)
        elif code == 0b1100:
            self.r[rd] = self._nz(a | b)
        elif code == 0b1101:
            self.r[rd] = self._nz((a * b) & MASK)
        elif code == 0b1110:
            self.r[rd] = self._nz(a & ~b)
        elif code == 0b1111:
            self.r[rd] = self._nz(~b)
        else:
            raise ThumbError("data processing %d at %#x" % (code, pc))
        return True

    def _shift_reg(self, code: int, value: int, shift: int) -> int:
        if shift == 0:
            return self._nz(value)
        if code == 0b0010:
            self.c = shift <= 32 and bool((value >> (32 - shift)) & 1)
            return self._nz((value << shift) & MASK if shift < 32 else 0)
        if code == 0b0011:
            self.c = shift <= 32 and bool((value >> (shift - 1)) & 1)
            return self._nz(value >> shift if shift < 32 else 0)
        signed = _sign(value, 32)
        shift = min(shift, 32)
        self.c = bool((signed >> (shift - 1)) & 1)
        return self._nz(signed >> shift)

    def _hi_bx_literal(self, op: int, pc: int) -> bool:
        if op >> 10 == 0b010001:
            kind = (op >> 8) & 3
            rm = (op >> 3) & 15
            rd = (op & 7) | ((op >> 4) & 8)
            value = pc + 4 if rm == PC else self.r[rm]
            if kind == 0:
                self.r[rd] = (self.r[rd] + value) & MASK
            elif kind == 1:
                self._sub(self.r[rd], value)
            elif kind == 2:
                if rd == PC:
                    raise ThumbError("MOV to PC at %#x" % pc)
                self.r[rd] = value
            elif op & 0x80:
                self._branch_link(self.r[rm], pc + 2)  # BLX register
            else:
                self.r[PC] = self.r[rm] & ~1  # BX
            return True
        if op >> 11 == 0b01001:
            rd, imm = (op >> 8) & 7, (op & 0xFF) << 2
            self.r[rd] = self.read(self._pcrel(pc) + imm, 4)
            return True
        return False

    def _load_store(self, op: int, pc: int) -> bool:
        if op >> 12 == 0b0101:
            kind, rm, rn, rt = (op >> 9) & 7, (op >> 6) & 7, (op >> 3) & 7, op & 7
            address = (self.r[rn] + self.r[rm]) & MASK
            size = (4, 2, 1, 1, 4, 2, 1, 2)[kind]
            if kind < 3:
                self.write(address, size, self.r[rt])
            elif kind == 3:
                self.r[rt] = _sign(self.read(address, 1), 8) & MASK
            elif kind == 7:
                self.r[rt] = _sign(self.read(address, 2), 16) & MASK
            else:
                self.r[rt] = self.read(address, size)
            return True
        group = op >> 11
        imm5, rn, rt = (op >> 6) & 31, (op >> 3) & 7, op & 7
        forms = {0b01100: (4, False), 0b01101: (4, True), 0b01110: (1, False), 0b01111: (1, True),
                 0b10000: (2, False), 0b10001: (2, True)}
        if group in forms:
            size, is_load = forms[group]
            address = (self.r[rn] + imm5 * size) & MASK
            if is_load:
                self.r[rt] = self.read(address, size)
            else:
                self.write(address, size, self.r[rt])
            return True
        if group in (0b10010, 0b10011):
            rt, address = (op >> 8) & 7, (self.r[SP] + ((op & 0xFF) << 2)) & MASK
            if group == 0b10011:
                self.r[rt] = self.read(address, 4)
            else:
                self.write(address, 4, self.r[rt])
            return True
        return False

    def _misc(self, op: int, pc: int) -> bool:
        if op >> 11 == 0b10101:  # ADD Rd, SP, #imm8 x 4
            self.r[(op >> 8) & 7] = (self.r[SP] + ((op & 0xFF) << 2)) & MASK
            return True
        if op >> 8 == 0b10110000:  # ADD/SUB SP, #imm7 x 4
            imm = (op & 0x7F) << 2
            self.r[SP] = (self.r[SP] - imm if op & 0x80 else self.r[SP] + imm) & MASK
            return True
        if op >> 6 in (0b1011001000, 0b1011001010):  # SXTH, UXTH
            rm, rd = (op >> 3) & 7, op & 7
            low = self.r[rm] & 0xFFFF
            self.r[rd] = (_sign(low, 16) & MASK) if op >> 6 == 0b1011001000 else low
            return True
        if op >> 6 in (0b1011001001, 0b1011001011):  # SXTB, UXTB
            rm, rd = (op >> 3) & 7, op & 7
            low = self.r[rm] & 0xFF
            self.r[rd] = (_sign(low, 8) & MASK) if op >> 6 == 0b1011001001 else low
            return True
        if op >> 9 == 0b1011010:  # PUSH
            regs = [i for i in range(8) if op & (1 << i)] + ([LR] if op & 0x100 else [])
            for reg in reversed(regs):
                self.push(self.r[reg])
            return True
        if op >> 9 == 0b1011110:  # POP
            regs = [i for i in range(8) if op & (1 << i)]
            for reg in regs:
                self.r[reg] = self.read(self.r[SP], 4)
                self.r[SP] = (self.r[SP] + 4) & MASK
            if op & 0x100:
                self.r[PC] = self.read(self.r[SP], 4) & ~1
                self.r[SP] = (self.r[SP] + 4) & MASK
            return True
        if op == 0xBF00:  # NOP
            return True
        return False

    def _branches(self, op: int, pc: int) -> bool:
        if op >> 12 == 0b1101:
            cond = (op >> 8) & 15
            if cond >= 14:
                raise ThumbError("UDF or SVC at %#x" % pc)
            if self._cond(cond):
                self.r[PC] = (pc + 4 + _sign(op & 0xFF, 8) * 2) & MASK
            return True
        if op >> 11 == 0b11100:
            self.r[PC] = (pc + 4 + _sign(op & 0x7FF, 11) * 2) & MASK
            return True
        if op >> 11 == 0b11110:  # BL, the first half
            low = self.read(pc + 2, 2)
            if low >> 14 != 0b11 or not low & 0x1000:
                raise ThumbError("not a BL at %#x" % pc)
            s = (op >> 10) & 1
            j1, j2 = (low >> 13) & 1, (low >> 11) & 1
            i1, i2 = 1 - (j1 ^ s), 1 - (j2 ^ s)
            offset = (s << 24) | (i1 << 23) | (i2 << 22) | ((op & 0x3FF) << 12) | ((low & 0x7FF) << 1)
            self._branch_link(pc + 4 + _sign(offset, 25), pc + 4)
            return True
        return False
