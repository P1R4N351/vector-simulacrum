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
