// Optional localhost relay. One browser owns one simulated body.
import { cameraRelay } from "./wire_camera.js";
import dgram from "node:dgram";
import { motorCommand } from "../src/sim/wire_body.js";

export function wireRelay() {
  return {
    name: "wireos-body",
    configureServer(server) {
      const host = process.env.VIC_BRIDGE_HOST || "127.0.0.1";
      const camera = cameraRelay(server,host);
      let owner = null, socket = null, cube = null;
      function close() {
        camera.stop();
        cube?.close();
        cube=null;
        socket?.close();
        socket = owner = null;
      }
      server.ws.on("wire:attach", (_, client) => {
        if (owner && owner !== client) {
          client.send("wire:error","Another simulator tab owns the WireOS body");
          return;
        }
        if (owner) return;
        owner = client;
        socket = dgram.createSocket("udp4");
        socket.on("error", err => server.config.logger.warn("WireOS UDP: "+err.message));
        socket.on("message", bytes => {
          const power = motorCommand(bytes);
          if (power) client.send("wire:power",power);
        });
        camera.start(client);
        cube = dgram.createSocket("udp4");
        cube.on("error",()=>{});
        cube.connect(5803,host);
        socket.connect(5800,host, () => client.send("wire:ready",{}));
        client.socket.once("close", () => { if (owner===client) close(); });
      });
      server.ws.on("wire:frame",(data,client) => {
        if (client===owner && typeof data==="string" && data.length===1084) socket.send(Buffer.from(data,"base64"));
      });
      server.ws.on("wire:cube",(data,client) => {
        if (client!==owner || typeof data!=="string" || data.length!==56) return;
        const packet=Buffer.from(data,"base64");
        if (packet.length!==40) return;
        if (packet[8]===1) {
          const hello=Buffer.alloc(8);
          hello.writeUInt32LE(0x56484c4f,0);
          hello.writeUInt32LE(5803,4);
          cube.send(hello);
        }
        cube.send(packet);
      });
      server.httpServer?.once("close",close);
    },
  };
}
