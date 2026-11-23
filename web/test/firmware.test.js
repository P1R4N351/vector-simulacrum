// ThumbCpu interpreter, CubeFirmwareVm (including unmodified cube.plain.bin when present),
// SysconFirmware + spine CRC-32, and VectorSim firmware attachment.
import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import { join } from "node:path";
import { test } from "node:test";
import { CubeFirmwareVm } from "../src/sim/cube_firmware.js";
import { VectorSim } from "../src/sim/robot.js";
import { SysconFirmware, spineCrc32 } from "../src/sim/syscon_firmware.js";
import { ThumbCpu, ThumbError } from "../src/sim/thumb.js";
import { ROOT, mujoco, readJson, readText } from "./helpers.js";

const FW_DIR = join(ROOT, "public/robot/firmware");
const HAVE_FIRMWARE = ["cube.plain.bin", "syscon-flash.bin", "firmware_manifest.json"]
  .every((name) => existsSync(join(FW_DIR, name)));

function wordsToBytes(words) {
  const out = new Uint8Array(words.length * 2);
  const view = new DataView(out.buffer);
  for (let i = 0; i < words.length; i++) view.setUint16(2 * i, words[i], true);
  return out;
}

function makeSyntheticSysconFlash() {
  const flash = new Uint8Array(0x10000);
  const view = new DataView(flash.buffer);
  flash.set(new TextEncoder().encode("C2MO"), 0x2000);
  view.setUint32(0x2110, 0x20001A00, true);
  view.setUint32(0x2114, 0x0800212D, true);
  flash.set(new TextEncoder().encode("Ws00000009c263e\0"), 0x2118);
  view.setUint16(0x2128, 0x574B, true);
  return flash;
}

test("ThumbCpu executes arithmetic, backward signed branch, MMIO, hooks, and bounds checks", () => {
  const cpu = new ThumbCpu({ ramBase: 0x20000000, ramSize: 0x8000, maxSteps: 200 });
  // r0 = 3; r1 = 0; loop: r1 += r0; r0 -= 1; bne -8; r0 = r1; bx lr
  const prog = wordsToBytes([0x2003, 0x2100, 0x1809, 0x3801, 0xD1FC, 0x0008, 0x4770]);
  cpu.load(0x20001000, prog);
  const ret = cpu.call(0x20001000, [], 0x20004000);
  assert.equal(ret, 6);
  let mmioVal = 0;
  cpu.addMmio(0x50000000, 0x50000100, () => mmioVal, (_off, _sz, v) => { mmioVal = v; });
  cpu.write(0x50000010, 4, 0xCAFE);
  assert.equal(cpu.read(0x50000010, 4), 0xCAFE);
  assert.throws(() => cpu.read(0x10000000, 4), ThumbError);
  // Infinite loop hits step limit
  cpu.load(0x20001200, wordsToBytes([0xE7FE]));
  assert.throws(() => cpu.call(0x20001200, [], 0x20004000), ThumbError);
});

test("spineCrc32 matches reference C libspinecrc vector and SysconFirmware verifies BodyToHead frames", () => {
  const head = Buffer.from(
    "2a000000010000006400000000000000000000009cffffff00000000000000002000000000000000"
    + "000000000000000000000000000000008403840384038403b80b490e19000300000000005800007800000000",
    "hex",
  );
  const payload = new Uint8Array(768);
  payload.set(head);
  // Exact 32-bit CRC emitted by wire-os-victor spine_crc.c calc_crc (init 0xFFFFFFFF, xorout 0)
  assert.equal(spineCrc32(payload), 0x137BCBB1);

  const fw = new SysconFirmware(makeSyntheticSysconFlash());
  assert.equal(fw.version, "Ws00000009c263e");
  assert.equal(fw.resetHandler, 0x0800212D);
  const frame = fw.packBodyFrame({
    frameCounter: 42,
    motorPos: [100, -100, 32, 0],
    cliff: [900, 900, 900, 900],
    batteryRaw: 3000,
    chargerRaw: 3657,
    chargerFlags: 3,
    proxMm: 120,
  });
  assert.equal(frame.length, 780);
  assert.equal(SysconFirmware.verifyBodyFrame(frame), true);
  frame[12] ^= 1;
  assert.equal(SysconFirmware.verifyBodyFrame(frame), false);
});

test("optional real unmodified cube.plain.bin + syscon-flash.bin execute in CubeFirmwareVm and VectorSim", { skip: !HAVE_FIRMWARE }, async () => {
  const cubeBytes = new Uint8Array(readFileSync(join(FW_DIR, "cube.plain.bin")));
  const sysconBytes = new Uint8Array(readFileSync(join(FW_DIR, "syscon-flash.bin")));
  const manifest = JSON.parse(readFileSync(join(FW_DIR, "firmware_manifest.json"), "utf8"));

  const vm = new CubeFirmwareVm(cubeBytes, { railCounts: 426 });
  assert.equal(vm.version, "2018-08-14T17:40");
  for (let i = 0; i < 95; i++) {
    vm.pushFrame([0, 0, i === 92 ? 3200 : 128]);
    vm.tick();
  }
  assert.ok(vm.bma.framesRead > 0);
  vm.setLights([255, 0, 0, 0, 128, 0, 0, 0, 64, 0, 0, 0]);
  for (let i = 0; i < 4; i++) vm.tick();
  assert.deepEqual(vm.ledRgb, [255, 0, 0, 0, 128, 0, 0, 0, 64, 0, 0, 0]);

  const { values, derived, camera } = readJson("public/robot/model.json");
  const xml = readText("public/robot/vector-free-dock.xml");
  const sim = new VectorSim(await mujoco(), { xml, values, derived, camera, noiseSeed: 1, startDocked: true });
  try {
    sim.attachFirmware({ cubeBytes, sysconBytes, manifest });
    sim.tick(40);
    const status = sim.firmwareStatus();
    assert.equal(status.active, true);
    assert.equal(status.cubeVersion, "2018-08-14T17:40");
    assert.equal(status.sysconVersion, "Ws00000009c263e");
    assert.ok(status.bmaFramesRead > 0);
    assert.ok(status.framesPacked > 0);
    assert.equal(SysconFirmware.verifyBodyFrame(sim.lastSpineFrame), true);
  } finally {
    sim.dispose();
  }
});
