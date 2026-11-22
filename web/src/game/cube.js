// Use the SDK's optical surface when installed; the collision boxes remain visible otherwise.
import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";

export const CUBE_FACE_ATLAS = {
  negX: { n: [-1, 0, 0], right: [0, -1, 0], up: [0, 0, 1], cx: 0.16211, cy: 0.15625, h: 0.10156 },
  posX: { n: [1, 0, 0], right: [0, 1, 0], up: [0, 0, 1], cx: 0.16309, cy: 0.37988, h: 0.10059 },
  negY: { n: [0, -1, 0], right: [1, 0, 0], up: [0, 0, 1], cx: 0.16211, cy: 0.60352, h: 0.10156 },
  posY: { n: [0, 1, 0], right: [-1, 0, 0], up: [0, 0, 1], cx: 0.39355, cy: 0.15527, h: 0.10059 },
  posZ: { n: [0, 0, 1], right: [0, -1, 0], up: [1, 0, 0], cx: 0.39551, cy: 0.37988, h: 0.10059 },
  negZ: { n: [0, 0, -1], right: [0, -1, 0], up: [-1, 0, 0], cx: 0.39453, cy: 0.60352, h: 0.10156 },
};

// The pinned SDK atlas stores its UV origin at the image top. Its six black
// symbols occupy the upper-left area; the other patches are material swatches.
export function stockCubeTexture(image) {
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

// These three adjacent faces were rectified from a stock cube photograph.
// Local orientations describe the photographed upright pose; the robot approaches -X.
export function makeCubeMarks(sizeMm) {
  const group = new THREE.Group();
  const loader = new THREE.TextureLoader();
  const faces = [
    [new URL("./cube-marks/front.png", import.meta.url), [-1, 0, 0], [0, 0, 1]],
    [new URL("./cube-marks/side.png", import.meta.url), [0, -1, 0], [0, 0, 1]],
    [new URL("./cube-marks/top.png", import.meta.url), [0, 0, 1], [1, 0, 0]],
  ];
  const width = (23 / 44) * sizeMm * .001 * 272 / 256;
  const geometry = new THREE.PlaneGeometry(width, width);
  for (const [url, normal, up] of faces) {
    const map = loader.load(url.href);
    map.colorSpace = THREE.SRGBColorSpace;
    const material = new THREE.MeshStandardMaterial({
      map, transparent: true, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -1, roughness: .8,
    });
    const mark = new THREE.Mesh(geometry, material);
    mark.up.fromArray(up);
    mark.lookAt(new THREE.Vector3(...normal));
    mark.position.fromArray(normal).multiplyScalar(sizeMm * .0005);
    group.add(mark);
  }
  return group;
}

export function repairMeshNormals(geometry) {
  geometry.computeVertexNormals();
  return geometry;
}

function selectFace(nx, ny, nz) {
  const ax = Math.abs(nx), ay = Math.abs(ny), az = Math.abs(nz);
  if (ax >= ay && ax >= az) return nx >= 0 ? CUBE_FACE_ATLAS.posX : CUBE_FACE_ATLAS.negX;
  if (ay >= ax && ay >= az) return ny >= 0 ? CUBE_FACE_ATLAS.posY : CUBE_FACE_ATLAS.negY;
  return nz >= 0 ? CUBE_FACE_ATLAS.posZ : CUBE_FACE_ATLAS.negZ;
}

function remapTriangleUV(pos, uv, i0, i1, i2, center, scale) {
  const ids = [i0, i1, i2];
  if (!ids.every((k) => uv.getX(k) < 0.51 && uv.getY(k) < 0.73)) return;
  const ax = pos.getX(i0), ay = pos.getY(i0), az = pos.getZ(i0);
  const bx = pos.getX(i1) - ax, by = pos.getY(i1) - ay, bz = pos.getZ(i1) - az;
  const cx = pos.getX(i2) - ax, cy = pos.getY(i2) - ay, cz = pos.getZ(i2) - az;
  const nx = by * cz - bz * cy, ny = bz * cx - bx * cz, nz = bx * cy - by * cx;
  if (nx * nx + ny * ny + nz * nz < 1e-18) return;
  const face = selectFace(nx, ny, nz);
  for (const k of ids) {
    const mx = (pos.getX(k) - center[0]) * scale[0];
    const my = (pos.getY(k) - center[1]) * scale[1];
    const mz = (pos.getZ(k) - center[2]) * scale[2];
    const rMm = mx * face.right[0] + my * face.right[1] + mz * face.right[2];
    const uMm = mx * face.up[0] + my * face.up[1] + mz * face.up[2];
    uv.setXY(k, face.cx + (rMm / 11.5) * face.h, face.cy - (uMm / 11.5) * face.h);
  }
}

export function repairCubeUVs(geometry) {
  const pos = geometry.attributes.position, uv = geometry.attributes.uv;
  if (!pos || !uv) return geometry;
  geometry.computeBoundingBox();
  const c = new THREE.Vector3(), s = new THREE.Vector3();
  geometry.boundingBox.getCenter(c);
  geometry.boundingBox.getSize(s);
  if (Math.min(s.x, s.y, s.z) < 1e-6) return geometry;
  const scale = [44 / s.x, 44 / s.y, 44 / s.z];
  const center = [c.x, c.y, c.z];
  const idx = geometry.index;
  const count = idx ? idx.count : pos.count;
  for (let t = 0; t + 2 < count; t += 3) {
    const i0 = idx ? idx.getX(t) : t, i1 = idx ? idx.getX(t + 1) : t + 1, i2 = idx ? idx.getX(t + 2) : t + 2;
    remapTriangleUV(pos, uv, i0, i1, i2, center, scale);
  }
  uv.needsUpdate = true;
  return geometry;
}

export async function loadCubeLook(sizeMm, base = "./robot/sdk/", options = {}) {
  try {
    if (options.requireManifest !== false) {
      const manifest = await (await fetch(base + "manifest.json")).json();
      if (!manifest.sha256?.["cube.obj"] || !manifest.sha256?.["cube.jpg"]) return null;
    }
    const [shape, image] = await Promise.all([
      new OBJLoader().loadAsync(base + "cube.obj"),
      new THREE.ImageLoader().loadAsync(base + "cube.jpg"),
    ]);
    const material = new THREE.MeshStandardMaterial({ map: stockCubeTexture(image), roughness: 0.62 });
    shape.traverse((node) => {
      if (node.isMesh) {
        repairMeshNormals(node.geometry);
        repairCubeUVs(node.geometry);
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
