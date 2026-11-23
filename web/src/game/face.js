// Behind Vector's removable visor. The LCD proportions follow the stock
// screen resolution; the carrier, clips and fastener positions are photo fits.
import * as THREE from "three";
import { RoundedBoxGeometry } from "three/addons/geometries/RoundedBoxGeometry.js";

function finish(color, roughness, metalness = 0) {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness });
}

// u is robot left, v is up, depth is forward, all in SDK centimetres.
function pad(parent, u, v, depth, width, height, thickness, radius, material) {
  const mesh = new THREE.Mesh(new RoundedBoxGeometry(width, height, thickness, 3, radius), material);
  mesh.position.set(u, v, depth);
  parent.add(mesh);
}

function screw(parent, u, v, material) {
  const cap = new THREE.Mesh(new THREE.CylinderGeometry(0.094, 0.10, 0.035, 20), material);
  cap.rotation.x = Math.PI / 2;
  cap.position.set(u, v, 0.102);
  parent.add(cap);
  const recess = finish(0x111113, 0.72);
  for (const angle of [0.21, 0.21 + Math.PI / 2]) {
    const slot = new THREE.Mesh(new THREE.BoxGeometry(0.115, 0.026, 0.008), recess);
    slot.position.set(u, v, 0.122);
    slot.rotation.z = angle;
    parent.add(slot);
  }
}

const HEAD_SECTIONS = [
    [-3.11, 0.03, 0.035], [-3.095, 0.62, 0.68], [-3.01, 1.21, 1.13],
    [-2.83, 1.56, 1.36], [-2.57, 1.72, 1.53], [-2.29, 1.83, 1.65],
    [-2.00, 1.90, 1.74], [-1.65, 1.94, 1.79], [-1.28, 1.96, 1.82],
    [-0.95, 1.96, 1.83],
];

const EQUATOR = HEAD_SECTIONS.at(-1);
const FRONT_LENGTH = 2.08; // SDK cm, from the widest section to the face centre
const SLICES = 96;
const SHELL_RINGS = 24;
const VISOR_RINGS = 36;

function frontSection(u) {
  const radius = Math.max(0, Math.cos(u));
  return [EQUATOR[0] + FRONT_LENGTH * Math.sin(u),
    EQUATOR[1] * radius ** 0.67, EQUATOR[2] * radius ** 0.61];
}

function sectionPoint([x, width, height], angle) {
  const c = Math.cos(angle), t = Math.sin(angle);
  const v = Math.sign(t) * Math.abs(t) ** 0.72;
  const jaw = Math.max(0, -v);
  const centreZ = 4.90 - 0.19 * THREE.MathUtils.smoothstep(x, EQUATOR[0], EQUATOR[0] + FRONT_LENGTH);
  return new THREE.Vector3(x, width * (1 + 0.018 * v - 0.055 * jaw ** 2)
    * Math.sign(c) * Math.abs(c) ** 0.64, centreZ + height * v);
}

// The seam divides one convex surface into two materials. Changing its
// course moves the glass/plastic boundary without reshaping the head.
function seamLatitude(angle) {
  const t = Math.sin(angle);
  return 0.88 - 0.57 * THREE.MathUtils.smoothstep(t, -0.15, 0.85)
    - 0.17 * THREE.MathUtils.smoothstep(-t, 0.25, 0.95);
}

function skin(u, angle) {
  return sectionPoint(frontSection(u), angle);
}

function skinNormal(u, angle) {
  const h = 0.0001;
  const across = skin(u, angle + h).sub(skin(u, angle - h));
  const along = skin(u + h, angle).sub(skin(u - h, angle));
  return across.cross(along).normalize();
}

// Height under the removable vent panel, on the same skin used by the shell.
export function headRoof(x, y) {
  let section;
  if (x >= EQUATOR[0]) {
    section = frontSection(Math.asin((x - EQUATOR[0]) / FRONT_LENGTH));
  } else {
    const i = HEAD_SECTIONS.findIndex(s => s[0] >= x);
    const a = HEAD_SECTIONS[i - 1], b = HEAD_SECTIONS[i];
    const t = (x - a[0]) / (b[0] - a[0]);
    section = a.map((n, k) => THREE.MathUtils.lerp(n, b[k], t));
  }
  let angle = Math.PI / 2;
  for (let i = 0; i < 4; i++) {
    const width = section[1] * (1 + 0.018 * Math.sin(angle) ** 0.72);
    angle = Math.acos(Math.min(1, Math.abs(y) / width) ** (1 / 0.64));
  }
  return sectionPoint(section, angle).z;
}

function innerSeat(angle, inset) {
  const point = skin(seamLatitude(angle), angle);
  const t = Math.sin(angle);
  const side = 1 - THREE.MathUtils.smoothstep(Math.abs(t), 0.30, 0.90);
  const lower = THREE.MathUtils.smoothstep(-t, 0.22, 0.90);
  point.x -= inset * (0.38 * side + 0.055 * lower);
  point.y *= 1 - 0.05 * inset;
  point.z = THREE.MathUtils.lerp(point.z, 4.90, 0.035 * inset);
  return point;
}

export function visorGeometry() {
  const vertices = [EQUATOR[0] + FRONT_LENGTH, 0, 4.71], indices = [], normals = [1, 0, 0];
  for (let i = 1; i <= VISOR_RINGS; i++) {
    for (let j = 0; j < SLICES; j++) {
      const angle = j * 2 * Math.PI / SLICES;
      const u = THREE.MathUtils.lerp(Math.PI / 2, seamLatitude(angle), i / VISOR_RINGS);
      vertices.push(...skin(u, angle).toArray());
      normals.push(...skinNormal(u, angle).toArray());
      const k = 1 + (i - 1) * SLICES + j, n = 1 + (i - 1) * SLICES + (j + 1) % SLICES;
      if (i === 1) indices.push(0, k, n);
      else indices.push(k - SLICES, k, n, k - SLICES, n, n - SLICES);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setAttribute("normal", new THREE.Float32BufferAttribute(normals, 3));
  geometry.setIndex(indices);
  return geometry;
}

function headShell() {
  const vertices = [], indices = [], browIndices = [];
  const rearRing = HEAD_SECTIONS.length - 1;
  const seam = rearRing + SHELL_RINGS;
  // Duplicate the seam for the inward-facing return. Its normals belong
  // to the open rim and must never be averaged into the exterior skin.
  for (let i = 0; i <= seam + 2; i++) {
    for (let j = 0; j < SLICES; j++) {
      const angle = j * 2 * Math.PI / SLICES;
      let point;
      if (i <= rearRing) point = sectionPoint(HEAD_SECTIONS[i], angle);
      else if (i <= seam) point = skin(seamLatitude(angle) * (i - rearRing) / SHELL_RINGS, angle);
      else point = innerSeat(angle, i - seam - 1);
      vertices.push(...point.toArray());
      if (!i || i === seam + 1) continue;
      const k = i * SLICES + j, n = i * SLICES + (j + 1) % SLICES;
      const brow = Math.sin(angle) > 0.72 && i > seam - 7 && i <= seam;
      (brow ? browIndices : indices).push(k - SLICES, n - SLICES, k, n - SLICES, n, k);
    }
  }
  for (let j = 1; j < SLICES - 1; j++) indices.push(0, j + 1, j);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setIndex(indices.concat(browIndices));
  geometry.addGroup(0, indices.length, 0);
  geometry.addGroup(indices.length, browIndices.length, 1);
  geometry.computeVertexNormals();
  for (let i = rearRing + 1; i <= seam; i++) {
    for (let j = 0; j < SLICES; j++) {
      const angle = j * 2 * Math.PI / SLICES;
      const normal = skinNormal(seamLatitude(angle) * (i - rearRing) / SHELL_RINGS, angle);
      geometry.attributes.normal.setXYZ(i * SLICES + j, normal.x, normal.y, normal.z);
    }
  }
  return geometry;
}

export function addFaceCarrier(head, values, axleX) {
  // One continuous surface from the rear cap to the return inside the
  // bezel. Leave the front open so the carrier is visible from the side.
  head.geometry.dispose();
  head.geometry = headShell();
  head.material = [finish(0x626367, 0.38), finish(0x6b6c70, 0.28)];
  const carrier = new THREE.Group();
  carrier.quaternion.setFromAxisAngle(new THREE.Vector3(1, 1, 1).normalize(), 2 * Math.PI / 3);
  carrier.position.set(0.265, 0, 4.89);
  head.add(carrier);

  const housing = finish(0x242427, 0.65);
  const bezel = finish(0x38383b, 0.44);
  const hardware = finish(0x6a6969, 0.35, 0.80);
  // The carrier follows the shell at its four corners. A square-backed
  // board would poke through the rounded cheeks above and below the ears.
  const boardVertices = [0.315, 0, 4.90], boardIndices = [];
  const slices = 96;
  for (const [ring, scale] of [0.78, 0.89, 0.995].entries()) {
    for (let j = 0; j < slices; j++) {
      const [edgeX, y, z] = innerSeat(2 * Math.PI * j / slices, 1).toArray();
      const depth = THREE.MathUtils.lerp(0.315, edgeX - 0.018, ring / 2);
      boardVertices.push(depth, y * scale, 4.90 + (z - 4.90) * scale);
      const k = 1 + ring * slices + j, n = 1 + ring * slices + (j + 1) % slices;
      if (!ring) boardIndices.push(0, k, n);
      else boardIndices.push(k - slices, k, n, k - slices, n, n - slices);
    }
  }
  const board = new THREE.BufferGeometry();
  board.setAttribute("position", new THREE.Float32BufferAttribute(boardVertices, 3));
  board.setIndex(boardIndices);
  board.computeVertexNormals();
  head.add(new THREE.Mesh(board, housing));
  const width = 2.34;
  const height = width * values.display_resolution_px[1] / values.display_resolution_px[0];
  pad(carrier, 0, 0.36, 0.060, width + 0.14, height + 0.15, 0.065, 0.03, bezel);
  pad(carrier, 0, 0.36, 0.103, width, height, 0.026, 0.01, finish(0x0c1013, 0.21));
  const display = new THREE.Mesh(new THREE.PlaneGeometry(width,height),new THREE.MeshBasicMaterial({toneMapped:false}));
  display.name = "wireos_lcd";
  display.position.set(0,.36,.118);
  display.visible = false;
  carrier.add(display);
  for (const u of [-1.34, 1.34]) {
    pad(carrier, u, 0.78, 0.077, 0.10, 0.23, 0.045, 0.014, hardware);
    pad(carrier, u, -0.05, 0.077, 0.11, 0.20, 0.045, 0.014, bezel);
  }
  screw(carrier, -1.06, -1.14, hardware);
  screw(carrier, 1.06, -1.14, hardware);

  const neck = values.neck_joint_position_mm, offset = values.head_camera_position_mm;
  const u = (neck[1] + offset[1]) / 10;
  const v = (neck[2] + offset[2]) / 10 - carrier.position.z;
  const lensDepth = axleX + (neck[0] + offset[0]) / 10 - carrier.position.x;
  pad(carrier, u - 0.45, v + 0.30, 0.071, 0.18, 0.21, 0.023, 0.009, finish(0xb5792d, 0.45));
  pad(carrier, u, v, (lensDepth - 0.11) / 2, 0.85, 0.87, lensDepth - 0.11, 0.055, housing);
  for (const sign of [-1, 1]) {
    pad(carrier, u + sign * 0.445, v, (lensDepth - 0.10) / 2, 0.075, 0.90, lensDepth - 0.10, 0.035, bezel);
  }
  for (const [radius, depth, mat] of [
    [0.292, -0.045, bezel], [0.230, -0.018, finish(0x15171a, 0.32)],
    [0.162, 0, finish(0x45505a, 0.16, 0.40)], [0.077, 0.016, finish(0x090d13, 0.10)],
  ]) {
    const ring = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 0.035, 36), mat);
    ring.rotation.x = Math.PI / 2;
    ring.position.set(u, v, lensDepth + depth);
    carrier.add(ring);
  }
}
