import * as THREE from "three";

export function faceTexture() {
  const rgb = new Uint8Array(184*96*4);
  const texture = new THREE.DataTexture(rgb,184,96);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.flipY = true;
  return {
    texture,
    update(base64) {
      const frame = atob(base64);
      for (let i=0; i<184*96; i++) {
        const pixel = frame.charCodeAt(2*i) | frame.charCodeAt(2*i+1)<<8;
        rgb[4*i] = Math.round((pixel>>11)*255/31);
        rgb[4*i+1] = Math.round(((pixel>>5)&63)*255/63);
        rgb[4*i+2] = Math.round((pixel&31)*255/31);
        rgb[4*i+3] = 255;
      }
      texture.needsUpdate=true;
    },
  };
}
