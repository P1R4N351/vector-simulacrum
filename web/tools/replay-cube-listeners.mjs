import assert from "node:assert/strict";
import { gunzipSync } from "node:zlib";
import { readFileSync } from "node:fs";
import { pathToFileURL } from "node:url";
import { CubeListeners } from "../src/sim/cube_listeners.js";

export function replayCsv(path) {
  const text = gunzipSync(readFileSync(path)).toString().trim().split("\n");
  const listeners = new CubeListeners(), events = [];
  let packet;
  for (const line of text.slice(1)) {
    const [time_s, , slot, , x, y, z] = line.split(",").map(Number);
    if (slot === 0) packet = { time_s, slots: [] };
    assert.equal(slot, packet.slots.length);
    assert.equal(time_s, packet.time_s);
    packet.slots.push([x, y, z]);
    if (slot === 2) events.push(...listeners.receive(packet));
  }
  return events;
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  console.log(JSON.stringify(process.argv.slice(2).map(replayCsv)));
}
