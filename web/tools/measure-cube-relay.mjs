// Loopback measurement. Port 5803 must be free. Run from a checkout with web dependencies.
import assert from "node:assert/strict";
import dgram from "node:dgram";
import {once} from "node:events";
import {readFileSync, writeFileSync} from "node:fs";
import {setTimeout as sleep} from "node:timers/promises";
import {gunzipSync} from "node:zlib";
import {createServer} from "vite";
import {wireRelay} from "./wire_relay.js";
import {WireBody} from "../src/sim/wire_body.js";

process.env.VIC_BRIDGE_HOST = "127.0.0.1";
const udp = dgram.createSocket("udp4");
const bound = once(udp, "listening");
udp.bind(5803, "127.0.0.1");
await bound;
const server = await createServer({configFile:false,plugins:[wireRelay()],logLevel:"error",
  server:{host:"127.0.0.1",port:0},optimizeDeps:{noDiscovery:true,include:[]}});
let ws;
try {
  await server.listen();
  const port = server.httpServer.address().port;
  ws = new WebSocket(`ws://127.0.0.1:${port}/?token=${server.config.webSocketToken}`, "vite-hmr");
  function send(event,data) { ws.send(JSON.stringify({type:"custom",event,data})); }
  const ready = new Promise((resolve,reject) => {
    const timer = setTimeout(() => reject(Error("Wire relay did not become ready")), 5000);
    ws.onmessage = ({data}) => {
      const msg = JSON.parse(data);
      if (msg.type === "connected") send("wire:attach",{});
      if (msg.event === "wire:ready") { clearTimeout(timer); resolve(); }
    };
  });
  await ready;
  const log = ["run,report,recorded_sec,send_ms,recv_first_ms,recv_last_ms,max_word_error"];
  for (const run of ["R5","R8"]) {
    const path = new URL(`../../evidence/unit3-wheelstand/${run}/cube.csv.gz`, import.meta.url);
    const rows = gunzipSync(readFileSync(path)).toString().trim().split(/\r?\n/).slice(1)
      .map(line=>line.split(",").map(Number));
    const received = [], sent = [];
    function capture(bytes) {
      if (bytes.length === 40 && bytes[8] === 2) received.push({ms:performance.now(),bytes});
    }
    udp.on("message",capture);
    const sensor = {packet:null};
    const body = new WireBody({cubeSensor:sensor});
    body.seq = 2;
    const start = performance.now();
    for (let i=0; i<rows.length; i+=3) {
      await sleep(Math.max(0,start + i*10 - performance.now()));
      const group = rows.slice(i,i+3);
      assert.deepEqual(group.map(row=>row[2]),[0,1,2]);
      sensor.packet = {slots:group.map(row=>row.slice(4,7)), taps:group[0][3]};
      sent.push(performance.now());
      for (const frame of body.cubeFrames()) send("wire:cube",Buffer.from(frame).toString("base64"));
    }
    const deadline = performance.now()+1500;
    while (received.length < rows.length && performance.now()<deadline) await sleep(5);
    udp.off("message",capture);
    assert.equal(received.length,rows.length,run+" received sample count");
    for (let i=0; i<rows.length; i+=3) {
      let error = 0;
      for(let j=0;j<3;j++) for(let k=0;k<3;k++)
        error=Math.max(error,Math.abs(received[i+j].bytes.readInt16LE(30+2*k)-rows[i+j][4+k]));
      log.push([run,i/3,rows[i][0],sent[i/3]-start,received[i].ms-start,received[i+2].ms-start,error].join(","));
    }
  }
  writeFileSync(process.argv[2] || "wire-udp.csv",log.join("\n")+"\n");
} finally { ws?.close(); await server.close(); udp.close(); }
