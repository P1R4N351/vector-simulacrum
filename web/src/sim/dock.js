// Charger geometry constants and spring-pin contact evaluation (port of simulacrum/dock.py).
import { assert } from "./util.js";

export const DEFAULT_DOCK_POSE_MM = [-120.0, 0.0, 180.0];
export const MARKER_WALL_X_MM = 93.205;
export const MARKER_POS_MM = [93.205, 0.0, 48.5];
export const PIN_LEFT_MM = [59.710, -9.850, 13.671];
export const PIN_RIGHT_MM = [59.710, 9.850, 13.671];
export const ROBOT_PAD_X_MM = -35.5;
export const ROBOT_PAD_Y_MM = 9.90;
export const ROBOT_PAD_Z_MM = 1.0;
export const PIN_TOL_X_MM = 9.0;
export const PIN_TOL_Y_MM = 3.8;
export const PIN_TOL_Z_LOW_MM = -5.0;
export const PIN_TOL_Z_HIGH_MM = 4.0;
export const CHARGER_SUPPLY_VOLTS = 4.20;
export const DOCKED_START_DIST_MM = 30.0;
export const DOCKED_START_Z_MM = 6.3;
export const DOCKED_START_PITCH_DEG = 8.2;

const DEG = Math.PI / 180;

export function dockedRobotPose(dockPose = DEFAULT_DOCK_POSE_MM, distMm = DOCKED_START_DIST_MM) {
  assert(dockPose.length === 3 && Number.isFinite(distMm), "docked pose requires [x, y, yawDeg] and finite distance");
  const yaw = dockPose[2] * DEG;
  return {
    startXyMm: [dockPose[0] + distMm * Math.cos(yaw), dockPose[1] + distMm * Math.sin(yaw)],
    headingDeg: dockPose[2] - 180.0,
    startZMm: DOCKED_START_Z_MM,
    pitchDeg: DOCKED_START_PITCH_DEG,
  };
}

function inPinBox(delta) {
  const okX = Math.abs(delta[0]) <= PIN_TOL_X_MM;
  const okY = Math.abs(delta[1]) <= PIN_TOL_Y_MM;
  const okZ = delta[2] >= PIN_TOL_Z_LOW_MM && delta[2] <= PIN_TOL_Z_HIGH_MM;
  return okX && okY && okZ;
}

function toDockMm(padLocalM, cPos, cRot, dPos, dRot) {
  const wx = cPos[0] + cRot[0] * padLocalM[0] + cRot[1] * padLocalM[1] + cRot[2] * padLocalM[2] - dPos[0];
  const wy = cPos[1] + cRot[3] * padLocalM[0] + cRot[4] * padLocalM[1] + cRot[5] * padLocalM[2] - dPos[1];
  const wz = cPos[2] + cRot[6] * padLocalM[0] + cRot[7] * padLocalM[1] + cRot[8] * padLocalM[2] - dPos[2];
  return [
    (dRot[0] * wx + dRot[3] * wy + dRot[6] * wz) * 1000,
    (dRot[1] * wx + dRot[4] * wy + dRot[7] * wz) * 1000,
    (dRot[2] * wx + dRot[5] * wy + dRot[8] * wz) * 1000,
  ];
}

// chassisMat / dockMat: row-major 3x3 world rotation matrices; positions in metres.
export function contactState(chassisPosM, chassisMat, dockPosM, dockMat) {
  assert(chassisPosM.length >= 3 && chassisMat.length >= 9, "bad chassis pose");
  assert(dockPosM.length >= 3 && dockMat.length >= 9, "bad dock pose");
  const bx = chassisPosM[0] - dockPosM[0];
  const by = chassisPosM[1] - dockPosM[1];
  const bz = chassisPosM[2] - dockPosM[2];
  const centerMm = [
    (dockMat[0] * bx + dockMat[3] * by + dockMat[6] * bz) * 1000,
    (dockMat[1] * bx + dockMat[4] * by + dockMat[7] * bz) * 1000,
    (dockMat[2] * bx + dockMat[5] * by + dockMat[8] * bz) * 1000,
  ];
  const padA = toDockMm([ROBOT_PAD_X_MM * 0.001, ROBOT_PAD_Y_MM * 0.001, ROBOT_PAD_Z_MM * 0.001], chassisPosM, chassisMat, dockPosM, dockMat);
  const padB = toDockMm([ROBOT_PAD_X_MM * 0.001, -ROBOT_PAD_Y_MM * 0.001, ROBOT_PAD_Z_MM * 0.001], chassisPosM, chassisMat, dockPosM, dockMat);
  const ordered = padA[1] <= padB[1] ? [padA, padB] : [padB, padA];
  const dLeft = [ordered[0][0] - PIN_LEFT_MM[0], ordered[0][1] - PIN_LEFT_MM[1], ordered[0][2] - PIN_LEFT_MM[2]];
  const dRight = [ordered[1][0] - PIN_RIGHT_MM[0], ordered[1][1] - PIN_RIGHT_MM[1], ordered[1][2] - PIN_RIGHT_MM[2]];
  const upDot = dockMat[2] * chassisMat[2] + dockMat[5] * chassisMat[5] + dockMat[8] * chassisMat[8];
  const onPlatform = upDot > 0.85 && centerMm[0] >= -30.0 && centerMm[0] <= 65.0
    && Math.abs(centerMm[1]) <= 32.0 && centerMm[2] >= -2.0 && centerMm[2] <= 35.0;
  const onCharger = onPlatform && inPinBox(dLeft) && inPinBox(dRight);
  return { on_charger: onCharger, on_charger_platform: onPlatform, pin_error_mm: [dLeft, dRight] };
}
