import { randomUUID } from "node:crypto";
import net from "node:net";

// Request one browser frame at a time, only while WireOS has a camera reader.
export function cameraRelay(server,host) {
  let owner, socket, timer, key, waiting=false;
  const bytes = 16+1280*720*1.5;
  server.middlewares.use("/__wire_camera", (req,res) => {
    if (!owner || req.url !== "/"+key || !waiting || req.method!=="POST" || Number(req.headers["content-length"])!==bytes) {
      res.statusCode=404; res.end(); return;
    }
    waiting=false;
    const current=socket, chunks=[];
    let size=0;
    req.on("data",chunk => {
      size+=chunk.length;
      if (size<=bytes) chunks.push(chunk);
    });
    req.on("end",() => {
      if (size===bytes && current===socket && current?.writable) current.write(Buffer.concat(chunks));
      res.end();
      if (owner && current===socket) schedule(70);
    });
    req.on("aborted",() => { if (current===socket) current?.destroy(); });
  });
  function schedule(delay) {
    clearTimeout(timer);
    if (owner) timer=setTimeout(pump,delay);
  }
  function pump() {
    if (!socket) {
      const stream=net.createConnection(5802,host);
      socket=stream;
      stream.on("error",()=>{});
      stream.on("close",()=>{ if (socket===stream) {socket=null; waiting=false; schedule(1000);} });
      stream.on("connect",()=>schedule(0));
    } else if (socket.writableLength===0) {
      waiting=true;
      owner.send("wire:camera",{path:"/__wire_camera/"+key});
    } else schedule(70);
  }
  return {
    start(client){owner=client;key=randomUUID();schedule(0);},
    stop(){owner=null;waiting=false;clearTimeout(timer);socket?.destroy();socket=null;},
  };
}
