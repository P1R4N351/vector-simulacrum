import assert from "node:assert/strict";
import { readFileSync, writeFileSync } from "node:fs";
import { gunzipSync } from "node:zlib";
import { CubeSensor } from "../../web/src/sim/cube_sensor.js";
import { WireBody } from "../../web/src/sim/wire_body.js";
import { readJson } from "../../web/test/helpers.js";

const { values } = readJson("public/robot/vector2/model.json");
console.log("run,reports,samples,incorrect_samples,incorrect_reports,max_abs_word_error");
for (const run of ["R5", "R8"]) {
  const path = new URL(`../unit3-wheelstand/${run}/cube.csv.gz`, import.meta.url);
  const rows = gunzipSync(readFileSync(path)).toString().trim().split(/\r?\n/).slice(1)
    .map(line => line.split(",").map(Number));
  assert.equal(rows.length % 3, 0);
  const sensor = new CubeSensor(values, .005, 1, [0, 0, 1]);
  const body = new WireBody({cubeSensor: sensor});
  body.seq = 2;
  const trace = ["sample,sec,physical_z,relayed_z"];
  let badSamples = 0, badReports = 0, maxError = 0;
  for (let i = 0; i < rows.length; i += 3) {
    const report = new Uint8Array(20), view = new DataView(report.buffer);
    report[0] = 2;
    report[1] = rows[i][3];
    const expected = [];
    for (let j = 0; j < 3; j++) {
      assert.equal(rows[i+j][2], j);
      assert.equal(rows[i+j][0], rows[i][0]);
      expected.push(rows[i+j].slice(4, 7));
      expected[j].forEach((word, k) => view.setInt16(2+6*j+2*k, word, true));
    }
    const frames = [];
    for (let j = 0; j < 3; j++) {
      sensor.firmware = {tick: () => j === 2 ? [report] : []};
      sensor._tickFirmware((i+j+1)*.01, 0);
      frames.push(...body.cubeFrames());
    }
    assert.equal(frames.length, 3);
    let wrong = 0;
    for (let j = 0; j < 3; j++) {
      const frame = new DataView(frames[j].buffer);
      const error = Math.max(...expected[j].map((word,k) => Math.abs(frame.getInt16(30+2*k,true)-word)));
      trace.push([i+j,rows[i+j][0],expected[j][2],frame.getInt16(34,true)].join(","));
      if (error) wrong++;
      maxError = Math.max(maxError,error);
    }
    badSamples += wrong;
    if (wrong) badReports++;
  }
  if (process.argv[2]) writeFileSync(`${process.argv[2]}-${run}.csv`, trace.join("\n")+"\n");
  console.log([run, rows.length/3, rows.length, badSamples, badReports, maxError].join(","));
}
