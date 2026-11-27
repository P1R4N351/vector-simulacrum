import { readFileSync } from "node:fs";
import { createHash } from "node:crypto";
import { CubeFirmwareVm } from "../../web/src/sim/cube_firmware.js";

const image = readFileSync(process.argv[2]);
const vm = new CubeFirmwareVm(image, {railCounts:444});
console.error(vm.version, createHash("sha256").update(image).digest("hex"));
let count = 0;
console.log("tick,accelerometer_reports_so_far");
for (let tick=1; tick<=1300; tick++) {
  vm.pushFrame([0,0,8192]);
  for (const payload of vm.tick()) {
    if (payload[0]===2) count++;
    if (payload[0]===4) console.log(tick+","+count);
  }
}
