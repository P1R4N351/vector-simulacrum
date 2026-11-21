// Moving track cleats and wheel spokes, so the drive train can be seen working.
//
// On the robot each side has a motor-driven front wheel, a free-spinning rear
// wheel, and a track around both. The physics model has the same arrangement:
// the motor acts on the front wheel's joint only, and the rear wheel is turned
// by a coupling that stands for the track (rear angle = front angle times the
// ratio of the radii). Everything in this file is drawing: cleats that travel
// round the track path by the front wheel's rotation, and spokes that show
// each wheel's own rotation. None of it is in the physics model.
//
// makeBeltPath has no three.js in it and is tested in test/tracks.test.js.

import * as THREE from "three";
import { RoundedBoxGeometry } from "three/addons/geometries/RoundedBoxGeometry.js";

function assert(condition, message) {
  if (!condition) throw new Error(`assertion failed: ${message}`);
}

// The closed path of a belt around two wheels, in the x-z plane.
// front, rear: { x, z, r } of the belt's outer surface round each wheel.
// pointAt(s) gives the point at arc length s and the outward normal there.
// s grows along the top from the rear wheel to the front, down round the
// front wheel, back along the bottom, and up round the rear wheel: the way
// the belt runs when the robot drives forward.
export function makeBeltPath(front, rear) {
  const dx = front.x - rear.x;
  const dz = front.z - rear.z;
  const between = Math.hypot(dx, dz);
  assert(front.r > 0 && rear.r > 0, "wheel radii must be positive");
  assert(between > Math.abs(front.r - rear.r), "one wheel lies inside the other");
  const alpha = Math.atan2(dz, dx);
  const gamma = Math.acos((rear.r - front.r) / between); // angle from the centre line to each tangent's normal
  const straight = Math.sqrt(between * between - (front.r - rear.r) ** 2);
  const frontArc = front.r * 2 * gamma;
  const rearArc = rear.r * (2 * Math.PI - 2 * gamma);
  const length = 2 * straight + frontArc + rearArc;
  const top = alpha + gamma;
  const bottom = alpha - gamma;
  const onWheel = (wheel, angle) => ({
    x: wheel.x + wheel.r * Math.cos(angle), z: wheel.z + wheel.r * Math.sin(angle), nx: Math.cos(angle), nz: Math.sin(angle),
  });
  const between2 = (a, b, t) => ({ x: a.x + (b.x - a.x) * t, z: a.z + (b.z - a.z) * t, nx: a.nx, nz: a.nz });

  function pointAt(s) {
    assert(Number.isFinite(s), "arc length must be finite");
    let u = ((s % length) + length) % length;
    if (u < straight) return between2(onWheel(rear, top), onWheel(front, top), u / straight);
    u -= straight;
    if (u < frontArc) return onWheel(front, top - u / front.r);
    u -= frontArc;
    if (u < straight) return between2(onWheel(front, bottom), onWheel(rear, bottom), u / straight);
    return onWheel(rear, bottom - (u - straight) / rear.r);
  }

  return { length, straight, frontArc, rearArc, pointAt };
}

// Sweep a closed rubber belt around the exact tangent path used by the ribs.
// Four corners at each station give the outside, inside and both sidewalls.
export function beltGeometry(path, width, thickness) {
  const positions = [], indices = [];
  const stations = 160;
  for (let i = 0; i < stations; i++) {
    const p = path.pointAt(i * path.length / stations);
    const ring = i * 4, next = ((i + 1) % stations) * 4;
    for (const [j, [y, depth]] of [[-1, 0], [1, 0], [1, thickness], [-1, thickness]].entries()) {
      positions.push(p.x - depth * p.nx, y * width / 2, p.z - depth * p.nz);
      const k = (j + 1) % 4;
      indices.push(ring + j, next + j, ring + k, next + j, next + k, ring + k);
    }
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geom.setIndex(indices);
  geom.computeVertexNormals();
  return geom;
}

// Cleats for both tracks as one instanced mesh.
// spec: { front, rear, sideY: [left, right], width, pitch, along, proud } in the parent's units.
// update(leftTravel, rightTravel): how far each belt has run, in the same units.
export function makeCleats(spec) {
  const path = makeBeltPath(spec.front, spec.rear);
  const perSide = Math.max(8, Math.round(path.length / spec.pitch));
  const spacing = path.length / perSide;
  const geometry = new RoundedBoxGeometry(spec.along, spec.width, spec.proud, 1, spec.proud * 0.4);
  const material = new THREE.MeshStandardMaterial({ color: 0x343436, roughness: 0.8 });
  const mesh = new THREE.InstancedMesh(geometry, material, 2 * perSide);
  mesh.name = "track_cleats";
  mesh.castShadow = true;
  mesh.frustumCulled = false;
  const matrix = new THREE.Matrix4();
  const place = (index, y, s) => {
    const p = path.pointAt(s);
    const lift = 0.5 * spec.proud;
    // Columns: along the belt, across it, outward.
    matrix.set(
      p.nz, 0, p.nx, p.x + p.nx * lift,
      0, 1, 0, y,
      -p.nx, 0, p.nz, p.z + p.nz * lift,
      0, 0, 0, 1,
    );
    mesh.setMatrixAt(index, matrix);
  };
  return {
    mesh,
    path,
    update(leftTravel, rightTravel) {
      for (let i = 0; i < perSide; i++) {
        place(i, spec.sideY[0], i * spacing + leftTravel);
        place(perSide + i, spec.sideY[1], i * spacing + rightTravel);
      }
      mesh.instanceMatrix.needsUpdate = true;
    },
  };
}

// Three bars across a wheel's face, in a plane with the given normal axis ("y" or "z").
// The bars are children of whatever turns with the wheel, so they show its rotation.
export function makeSpokes(radius, faceOffset, axis, thickness) {
  const group = new THREE.Group();
  const material = new THREE.MeshStandardMaterial({ color: 0xcfd4da, roughness: 0.5, metalness: 0.3 });
  const size = axis === "y" ? [1.7 * radius, thickness, 0.22 * radius] : [1.7 * radius, 0.22 * radius, thickness];
  for (let i = 0; i < 3; i++) {
    const bar = new THREE.Mesh(new THREE.BoxGeometry(size[0], size[1], size[2]), material);
    if (axis === "y") {
      bar.position.y = faceOffset;
      bar.rotation.y = (i * Math.PI) / 3;
    } else {
      bar.position.z = faceOffset;
      bar.rotation.z = (i * Math.PI) / 3;
    }
    group.add(bar);
  }
  return group;
}
