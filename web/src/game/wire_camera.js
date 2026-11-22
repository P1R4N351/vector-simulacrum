import * as THREE from "three";

// WebGL pixels start at the bottom left. The WireOS camera expects top-down NV12.
export function cameraPacket(ctx, sequence, target, pixels, camera) {
  const { renderer, scene, visuals } = ctx;
  const { width, height } = target;
  const previous = renderer.getRenderTarget();
  try {
    renderer.setRenderTarget(target);
    const mounted = visuals.headCamera.camera;
    mounted.updateWorldMatrix(true,false);
    camera.matrix.copy(mounted.matrixWorld);
    camera.matrixWorldNeedsUpdate = true;
    renderer.render(scene, camera);
    renderer.readRenderTargetPixels(target, 0, 0, width, height, pixels);
  } finally {
    renderer.setRenderTarget(previous);
  }
  const packet = new Uint8Array(16 + width*height*1.5);
  const header = new DataView(packet.buffer);
  header.setUint32(0,0x56494343,true);
  header.setUint32(4,2,true);
  header.setUint32(8,sequence,true);
  header.setUint16(12,width,true);
  header.setUint16(14,height,true);
  for (let y=0; y<height; y++) {
    let p = (height-1-y)*width*4;
    for (let x=0; x<width; x++,p+=4) {
      const r=pixels[p], g=pixels[p+1], b=pixels[p+2];
      packet[16+y*width+x] = 16+((66*r+129*g+25*b+128)>>8);
      if ((y%2===0) && (x%2===0)) {
        const uv = 16+width*height+(y/2)*width+x;
        packet[uv] = 128+((-38*r-74*g+112*b+128)>>8);
        packet[uv+1] = 128+((112*r-94*g-18*b+128)>>8);
      }
    }
  }
  return packet;
}

export function attachWireCamera(hot, ctx) {
  let sequence=0, target=null, pixels=null;
  // CozmoBot2 HeadCamera uses 1.5708 rad horizontal at 1280x720.
  const aspect = 1280/720;
  const vfov = THREE.MathUtils.radToDeg(2*Math.atan(Math.tan(1.5708/2)/aspect));
  const camera = new THREE.PerspectiveCamera(vfov,aspect,.005,10);
  camera.matrixAutoUpdate = false;
  camera.layers.mask = ctx.visuals.headCamera.camera.layers.mask;
  hot.on("wire:camera", async ({ path }) => {
    if (!target) {
      target = new THREE.WebGLRenderTarget(1280,720);
      target.texture.colorSpace = THREE.SRGBColorSpace;
      pixels = new Uint8Array(1280*720*4);
    }
    const body = cameraPacket(ctx,sequence++,target,pixels,camera);
    await fetch(path,{method:"POST",body});
  });
  hot.dispose(() => target?.dispose());
}
