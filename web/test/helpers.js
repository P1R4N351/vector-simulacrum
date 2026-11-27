// Shared test set-up: the MuJoCo WebAssembly module and the exported model..
import { createHash } from "node:crypto";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import loadMujoco from "@mujoco/mujoco";

export const ROOT = join(dirname(fileURLToPath(import.meta.url)), "..");

export function readJson(relative) {
  return JSON.parse(readFileSync(join(ROOT, relative), "utf8"));
}

export function readText(relative) {
  return readFileSync(join(ROOT, relative), "utf8");
}

export function sha256(text) {
  return createHash("sha256").update(text, "utf8").digest("hex");
}

let modulePromise = null;
export function mujoco() {
  if (modulePromise === null) modulePromise = loadMujoco();
  return modulePromise;
}

export function makeSyntheticSysconFlash() {
  const flash = new Uint8Array(0x10000);
  const view = new DataView(flash.buffer);
  flash.set(new TextEncoder().encode("C2MO"), 0x2000);
  view.setUint32(0x2110, 0x20001A00, true);
  view.setUint32(0x2114, 0x0800212D, true);
  flash.set(new TextEncoder().encode("Ws00000009c263e\0"), 0x2118);
  view.setUint16(0x2128, 0x574B, true);
  return flash;
}
