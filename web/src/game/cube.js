// Use the SDK's optical surface when installed; the collision boxes remain visible otherwise.
import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";

// The pinned SDK atlas stores its UV origin at the image top. Its six black
// symbols occupy the upper-left area; the other patches are material swatches.
function stockCubeTexture(image) {
  const canvas = document.createElement("canvas");
  canvas.width = image.width;
  canvas.height = image.height;
  const ctx = canvas.getContext("2d");
  ctx.drawImage(image, 0, 0);
  const pixels = ctx.getImageData(0, 0, canvas.width, canvas.height);
  for (let y = 0; y < canvas.height; y++) {
    for (let x = 0; x < canvas.width; x++) {
      const i = (y * canvas.width + x) * 4;
      const tone = pixels.data[i] / 255;
      const isMarker = x < canvas.width * 0.51 && y < canvas.height * 0.73;
      const gray = isMarker ? 230 - 175 * tone : 24 + 31 * tone;
      pixels.data[i] = pixels.data[i + 1] = pixels.data[i + 2] = gray;
    }
  }
  ctx.putImageData(pixels, 0, 0);
  const texture = new THREE.CanvasTexture(canvas);
  texture.flipY = false;
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

export function repairMeshNormals(geometry) {
  geometry.computeVertexNormals();
  return geometry;
}

export async function loadCubeLook(sizeMm) {
  const base = "./robot/sdk/";
  try {
    const manifest = await (await fetch(base + "manifest.json")).json();
    if (!manifest.sha256?.["cube.obj"] || !manifest.sha256?.["cube.jpg"]) return null;
    const [shape, image] = await Promise.all([
      new OBJLoader().loadAsync(base + "cube.obj"),
      new THREE.ImageLoader().loadAsync(base + "cube.jpg"),
    ]);
    const material = new THREE.MeshStandardMaterial({ map: stockCubeTexture(image), roughness: 0.62 });
    shape.traverse((node) => {
      if (node.isMesh) {
        repairMeshNormals(node.geometry);
        node.material = material;
        node.castShadow = true;
        node.receiveShadow = true;
      }
    });
    const bounds = new THREE.Box3().setFromObject(shape);
    const size = bounds.getSize(new THREE.Vector3());
    shape.scale.set(sizeMm / 1000 / size.x, sizeMm / 1000 / size.y, sizeMm / 1000 / size.z);
    shape.position.copy(bounds.getCenter(new THREE.Vector3())).multiply(shape.scale).negate();
    return new THREE.Group().add(shape);
  } catch (error) {
    console.warn("Cube optical mesh unavailable; using contact boxes", error);
    return null;
  }
}
