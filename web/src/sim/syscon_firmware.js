// Verifies the real 64 KB STM32F030 syscon flash image (sysboot.bin + syscon.dfu)
// and implements the wire-os-victor spine CRC32 (0xEDB88320, init 0xFFFFFFFF, xorout 0)
// and BodyToHead DATA_FRAME codec matching robot/fixture/stm/lib/spine_crc.c.

import { assert } from "./util.js";

const FLASH_SIZE = 0x10000;
const APP_OFF = 0x2000;
const WHISKEY_MAGIC = 0x574B;
const SYNC_BODY_TO_HEAD = 0x483242AA;
const PAYLOAD_DATA_FRAME = 0x6466;
const BODY_FRAME_BYTES = 776; // 8B header + 768B BodyToHead + 4B CRC = 780B total

const CRC_TABLE = (() => {
  const table = new Uint32Array(256);
  for (let i = 0; i < 256; i++) {
    let c = i;
    for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
    table[i] = c >>> 0;
  }
  return table;
})();

export function spineCrc32(bytes, start = 0, end = bytes.length) {
  let crc = 0xFFFFFFFF;
  for (let i = start; i < end; i++) {
    crc = (CRC_TABLE[(crc ^ bytes[i]) & 0xFF] ^ (crc >>> 8)) >>> 0;
  }
  return crc >>> 0;
}

export class SysconFirmware {
  constructor(flashBytes) {
    assert(flashBytes && flashBytes.length === FLASH_SIZE, "syscon-flash.bin must be 65536 bytes");
    const v = new DataView(flashBytes.buffer, flashBytes.byteOffset, flashBytes.byteLength);
    const fp = new TextDecoder().decode(flashBytes.subarray(APP_OFF, APP_OFF + 4));
    assert(fp === "C2MO", `bad syscon fingerprint ${fp}`);
    const magic = v.getUint16(APP_OFF + 0x128, true);
    assert(magic === WHISKEY_MAGIC, `bad whiskeyCompatible 0x${magic.toString(16)}`);
    this.sp = v.getUint32(APP_OFF + 0x110, true);
    this.resetHandler = v.getUint32(APP_OFF + 0x114, true);
    this.version = new TextDecoder().decode(flashBytes.subarray(APP_OFF + 0x118, APP_OFF + 0x128)).replace(/\0+$/, "");
    this.flash = flashBytes;
    this.framesPacked = 0;
  }

  packBodyFrame({ frameCounter = 0, flags = 1, motorPos = [0, 0, 0, 0], cliff = [600, 600, 600, 600],
    batteryRaw = 2948, chargerRaw = 0, chargerFlags = 0, proxMm = 400, proxStatus = 0x58 } = {}) {
    const buf = new Uint8Array(BODY_FRAME_BYTES + 4);
    const view = new DataView(buf.buffer);
    view.setUint32(0, SYNC_BODY_TO_HEAD, true);
    view.setUint16(4, PAYLOAD_DATA_FRAME, true);
    view.setUint16(6, 768, true);
    const p = 8;
    view.setUint32(p, frameCounter >>> 0, true);
    view.setUint8(p + 4, flags & 0xFF);
    for (let i = 0; i < 4; i++) {
      view.setInt32(p + 8 + 12 * i, motorPos[i] | 0, true);
      view.setUint16(p + 56 + 2 * i, cliff[i] & 0xFFFF, true);
    }
    view.setInt16(p + 64, batteryRaw, true);
    view.setInt16(p + 66, chargerRaw, true);
    view.setInt16(p + 68, 25, true);
    view.setUint16(p + 70, chargerFlags, true);
    view.setUint8(p + 76, proxStatus & 0xFF);
    view.setUint16(p + 78, ((proxMm & 0xFF) << 8) | ((proxMm >> 8) & 0xFF), true);
    const crc = spineCrc32(buf, p, p + 768);
    view.setUint32(p + 768, crc, true);
    this.framesPacked += 1;
    return buf;
  }

  static verifyBodyFrame(buf) {
    if (!buf || buf.length !== BODY_FRAME_BYTES + 4) return false;
    const view = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
    if (view.getUint32(0, true) !== SYNC_BODY_TO_HEAD) return false;
    if (view.getUint16(4, true) !== PAYLOAD_DATA_FRAME || view.getUint16(6, true) !== 768) return false;
    return spineCrc32(buf, 8, 776) === view.getUint32(776, true);
  }
}
