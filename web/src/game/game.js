// The simulator core in the page: boot, the control loop, input, camera..
//
// Structure after the MicroDuck simulator (pollen-robotics, Apache-2.0): an
// imperative core that owns the physics and the rendering, a small store that
// carries state out to React, and a gameApi object that carries intents in.
// MuJoCo runs here as WebAssembly; there is no server.
//
// Time: the robot's control tick is 5 ms and the physics step 1 ms, both from
// the parameter table. Each frame runs as many whole control ticks as real
// time has passed, up to MAX_TICKS_PER_FRAME; past that the simulation runs
// slower than real time (and says so in the readout) rather than stall the page.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import mujocoWasmUrl from "@mujoco/mujoco/mujoco.wasm?url";
import { CONTROL_DEFAULT, CONTROL_OVERRIDE, VectorSim } from "../sim/robot.js";
import { bootHalt, bootLine, gameApi, useGame } from "../store.js";
import { HIDDEN_FROM_ROBOT } from "./headcam.js";
import { createInput } from "./input.js";
import { mappedCubeXY } from "../sim/shelf_cube_pose.js";
import shelfCube from "../sim/shelf_cube.json";
import { loadCubeLook } from "./cube.js";
import { buildVisuals } from "./scene.js";
import { loadSdkModel } from "./sdkmodel.js";

const MAX_TICKS_PER_FRAME = 24;
const TELEMETRY_PERIOD_MS = 100;
const DRIVE_MMPS = 100; // held-key forward speed; Shift doubles it
const TURN_MMPS = 45; // held-key wheel speed difference, each side
const ACTIONS = {
  forward100: (sim) => sim.driveStraight(100, 80),
  back100: (sim) => sim.driveStraight(-100, 80),
  turnLeft90: (sim) => sim.turnInPlace(90),
  turnRight90: (sim) => sim.turnInPlace(-90),
  headUp: (sim) => sim.setHeadAngle(sim.v.head_angle_max_deg),
  headLevel: (sim) => sim.setHeadAngle(0),
  headDown: (sim) => sim.setHeadAngle(sim.v.head_angle_min_deg),
  liftUp: (sim) => sim.setLiftHeight(sim.v.lift_height_max_mm),
  liftDown: (sim) => sim.setLiftHeight(sim.v.lift_height_min_mm),
};
const RPM = 60 / (2 * Math.PI);

async function traced(label, promise) {
  const done = bootLine(label);
  try {
    const value = await promise;
    done("OK");
    return value;
  } catch (err) {
    done("FAILED");
    throw new Error(`${label}: ${err?.message ?? err}`);
  }
}

async function fetchText(url) {
  const response = await fetch(url, { cache: "no-cache" });
  if (!response.ok) throw new Error(`${url} answered ${response.status}`);
  return response.text();
}

async function sha256Hex(text) {
  const digest = await crypto.subtle.digest("SHA-256", new TextEncoder().encode(text));
  return Array.from(new Uint8Array(digest), (b) => b.toString(16).padStart(2, "0")).join("");
}

// The model the page compiles must be the one model.json describes.
async function checkModel(xml, expectedHash) {
  if (!globalThis.crypto?.subtle) return "SKIPPED (not a secure origin)";
  const hash = await sha256Hex(xml);
  if (hash !== expectedHash) throw new Error("vector.xml does not match the hash in model.json; run tools/export_model.py");
  return "OK";
}

async function loadEverything() {
  const options = new URLSearchParams(location.search);
  const trial = options.get("contact") === "42";
  const shelf = options.get("env") === "shelf" ||
    (!options.has("env") && options.get("cubeSensor") === "unit3");
  const free = shelf || options.get("cube") === "free";
  const vector2 = shelf || options.get("robot") === "2";
  const printedFork = options.get("fork") === "printed";
  if (printedFork && !vector2) throw new Error("Printed fork calibration requires robot=2");
  const directory = vector2 ? "./robot/vector2" + (printedFork ? "/printed" : "") : "./robot";
  const sceneName = (trial ? "vector-contact42" : "vector") + (free ? "-free" : "") + (shelf ? "-shelf" : "");
  const [{ default: loadMujoco }, xml, modelText] = await Promise.all([
    traced("PHYSICS RUNTIME", import("@mujoco/mujoco")),
    traced(trial ? "ROBOT MODEL · 42 MM CONTACT" : "ROBOT MODEL",
      fetchText(`${directory}/${sceneName}.xml`)),
    traced("PARAMETER TABLE", fetchText(`${directory}/model.json`)),
  ]);
  const model = JSON.parse(modelText);
  if (free && options.get("cubeSensor") === "unit3") Object.assign(model.values, shelfCube);
  if (free && vector2 && options.get("cubeSensor") === "unit3") model.values.supervisor_loop_ms = 5.083;
  const doneCheck = bootLine("MODEL CHECKSUM");
  doneCheck(await checkModel(xml, free ? model.source[sceneName + "_sha256"] : (trial ? model.source.trial_xml_sha256 : model.source.xml_sha256)));
  const mujoco = await traced("WEBASSEMBLY", loadMujoco({ locateFile: (p) => (p.endsWith(".wasm") ? mujocoWasmUrl : p) }));
  const doneCompile = bootLine("COMPILE AND SETTLE");
  const mapFile = options.get("map") === "live" ? "shelf-navmap-live.json" : "shelf-navmap.json";
  const navMap = shelf ? JSON.parse(await fetchText("./robot/vector2/" + mapFile)) : null;
  const sim = new VectorSim(mujoco, { xml, values: model.values, derived: model.derived, controlLevel: CONTROL_DEFAULT, noiseSeed: 1,
    cubeStartXY: shelf ? mappedCubeXY(navMap, model.values.cube_size_mm, -66, 30) : null,
    shelfOutline: shelf ? model.shelf_outline_mm : null,
    bodyRangeClock: options.get("rangeClock") === "body", chargerContact: options.get("chargerContact") === "1" });
  doneCompile("OK");
  return { mujoco, model, sim, free, navMap };
}

function makeRenderer(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b0d10);
  scene.fog = new THREE.Fog(0x0b0d10, 2.5, 7);
  scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment()).texture;
  scene.environmentIntensity = 0.55;
  const camera = new THREE.PerspectiveCamera(38, 1, 0.005, 20);
  camera.position.set(0.19, 0.125, 0.19);
  camera.layers.enable(HIDDEN_FROM_ROBOT); // the main view sees the drawing aids the robot's camera does not
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.minDistance = 0.08;
  controls.maxDistance = 3;
  controls.maxPolarAngle = Math.PI * 0.62;
  controls.target.set(0, 0.02, 0);
  const resize = () => {
    const { clientWidth: w, clientHeight: h } = canvas;
    renderer.setSize(w, h, false);
    camera.aspect = w / Math.max(h, 1);
    camera.fov = camera.aspect < 1 ? 58 : 38; // a tall window sees less width: widen the view
    camera.updateProjectionMatrix();
  };
  resize();
  window.addEventListener("resize", resize);
  return { renderer, scene, camera, controls };
}

// Held input -> SDK commands. A command is sent when what is held changes, not every frame:
// each head or lift command restarts that motor's acceleration ramp, as it does on the robot.
function makeDriver(sim) {
  const v = sim.v;
  const st = { sent: [0, 0], head: 0, lift: 0 };
  return {
    reset() {
      st.sent = [0, 0];
      st.head = 0;
      st.lift = 0;
    },
    apply(intent) {
      // Direct drive is not limited to the path-following top speed; the motor sets the ceiling.
      const scale = intent.fast ? 2 : 1;
      const left = scale * (intent.fwd * DRIVE_MMPS - intent.turn * TURN_MMPS);
      const right = scale * (intent.fwd * DRIVE_MMPS + intent.turn * TURN_MMPS);
      if ((left !== st.sent[0] || right !== st.sent[1]) && sim.setWheelMotors(left, right)) st.sent = [left, right];
      if (!sim.hasControl || sim.actionRunning) st.sent = [NaN, NaN]; // force a re-send once commands are accepted again
      if (sim.actionRunning) return; // a held key must not move the target an action is waiting on
      // Held: head for the end stop at the default speed. Released: stop where the command has got to.
      const head = Math.sign(intent.head);
      if (head !== st.head) {
        const to = head > 0 ? v.head_angle_max_deg : head < 0 ? v.head_angle_min_deg : (sim.headCommandRad * 180) / Math.PI;
        if (sim.commandHeadAngle(to)) st.head = head;
      }
      const lift = Math.sign(intent.lift);
      if (lift !== st.lift) {
        const to = lift > 0 ? v.lift_height_max_mm : lift < 0 ? v.lift_height_min_mm : sim.liftCommandMm;
        if (sim.commandLiftHeight(to)) st.lift = lift;
      }
    },
  };
}

function telemetryOf(sim, extra) {
  const s = sim.state;
  const t = sim.simTruth();
  return {
    ...extra, time_s: sim.tickCount * sim.dt, x_mm: sim.poseMm()[0], y_mm: sim.poseMm()[1],
    z_mm: t.z_mm, heading_deg: (sim.headingTotalRad() * 180) / Math.PI, wheel_cmd: t.wheel_command_mmps,
    wheel_meas: [s.left_wheel_speed_mmps, s.right_wheel_speed_mmps], motor_power: t.motor_power,
    front_rpm: [t.wheel_spin_radps[0] * RPM, t.wheel_spin_radps[1] * RPM],
    rear_rpm: [t.wheel_spin_radps[2] * RPM, t.wheel_spin_radps[3] * RPM],
    cube_axis: sim.cubeSensor?.listeners.axis, cube_moving: sim.cubeSensor?.listeners.moving,
    cube_tap_events: sim.cubeSensor?.listeners.tapEvents,
    cube_last_tap_s: sim.cubeSensor?.listeners.lastTapTime,
    cube_movement_broadcasts: sim.cubeSensor?.movementBroadcastCount,
    cube_motion_transitions: sim.cubeSensor?.listeners.motionTransitions,
    cube_last_motion_s: sim.cubeSensor?.listeners.lastMotionTime,
    cube_slot_g: sim.cubeSensor?.packet?.slots.map(slot =>
      Math.hypot(...slot) / sim.v.cube_accel_counts_per_g),
    battery_v: t.battery_volts, tread_heat: t.tread_heat,
    head_deg: (s.head_angle_rad * 180) / Math.PI, lift_mm: s.lift_height_mm, prox_mm: s.prox_distance_mm,
    prox_found: s.prox_found_object, prox_quality: s.prox_signal_quality, prox_unobstructed: s.prox_unobstructed,
    prox_lift: s.prox_is_lift_in_fov, cliff: t.cliff_flags, cliff_detected: s.cliff_detected, picked_up: s.picked_up,
    falling: s.falling, has_control: sim.hasControl, reacting: t.reacting, reactions: sim.reactions,
    fallen: t.fallen, edge_margin_mm: t.edge_margin_mm, action_running: sim.actionRunning,
  };
}

function moodOf(truth, sim) {
  if (truth.fallen) return "dim";
  if (sim.hazards.falling || sim.hazards.cliffAny) return "alarm";
  if (truth.reacting || !sim.hasControl) return "squint";
  return "calm";
}

function installApi(ctx) {
  const { sim, visuals, driver } = ctx;
  const note = (text) => useGame.setState({ notice: text === null ? null : { text, at: performance.now() } });
  const press = {
    stop: () => sim.stopAllMotors(),
    reset: () => {
      sim.reset({ noiseSeed: 1 });
      driver.reset();
      visuals.clearTrail();
      useGame.setState({ lastAction: null });
      note(null);
    },
    chase: () => useGame.setState((s) => ({ chase: !s.chase })),
    rays: () => useGame.setState((s) => ({ rays: !s.rays })),
    shapes: () => useGame.setState((s) => ({ shapes: !s.shapes })),
    camera: () => useGame.setState((s) => ({ camView: !s.camView })),
    pause: () => useGame.setState((s) => ({ paused: !s.paused })),
    level: () => {
      sim.controlLevel = sim.controlLevel === CONTROL_DEFAULT ? CONTROL_OVERRIDE : CONTROL_DEFAULT;
      useGame.setState({ level: sim.controlLevel });
    },
    control: () => note(sim.requestControl() ? "Control returned" : "Refused: the robot is still reacting"),
  };
  for (const [name, start] of Object.entries(ACTIONS)) {
    press[name] = () => {
      if (!start(sim)) note(sim.hasControl ? "Refused: an action is already running" : "Refused: no control (press X)");
    };
  }
  Object.assign(gameApi, { press: (name) => press[name]?.(), setPad: (fwd, turn) => ctx.input.setPad(fwd, turn), trail: () => visuals.trailPoints() });
  return press;
}

// Draw what the robot's camera sees into the window the panel leaves for it.
// The window is a page element (.camview-window); its place on the canvas is read each frame.
function renderHeadCamera(ctx) {
  const { renderer, scene, visuals } = ctx;
  ctx.camWindow = ctx.camWindow?.isConnected ? ctx.camWindow : document.querySelector(".camview-window");
  if (!ctx.camWindow) return;
  const box = ctx.camWindow.getBoundingClientRect();
  const canvas = renderer.domElement.getBoundingClientRect();
  if (box.width < 2 || box.height < 2) return;
  const x = box.left - canvas.left;
  const y = canvas.bottom - box.bottom; // the renderer counts from the bottom edge
  renderer.setScissorTest(true);
  renderer.setViewport(x, y, box.width, box.height);
  renderer.setScissor(x, y, box.width, box.height);
  renderer.render(scene, visuals.headCamera.camera);
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, canvas.width, canvas.height);
}

function startLoop(ctx) {
  const { sim, visuals, driver, renderer, scene, camera, controls } = ctx;
  const follow = new THREE.Vector3();
  const delta = new THREE.Vector3();
  let last = performance.now();
  let debt = 0;
  let lastTelemetry = 0;
  let ticksInWindow = 0;
  let blinkAt = last + 3000;

  function advance(now, dtS) {
    const intent = ctx.input.read();
    if (intent.padConnected !== useGame.getState().padConnected) useGame.setState({ padConnected: intent.padConnected });
    driver.apply(intent);
    debt += dtS;
    let ticks = Math.floor(debt / sim.dt);
    if (ticks > MAX_TICKS_PER_FRAME) {
      ticks = MAX_TICKS_PER_FRAME;
      debt = 0;
    } else debt -= ticks * sim.dt;
    const wasRunning = sim.actionRunning;
    sim.tick(ticks);
    ticksInWindow += ticks;
    if (wasRunning && !sim.actionRunning) useGame.setState({ lastAction: sim.lastAction });
  }

  function frame(now) {
    ctx.raf = requestAnimationFrame(frame);
    // A frame's timestamp can precede the clock read when the loop was started: never step back.
    const dtS = Math.max(0, Math.min((now - last) / 1000, 0.1));
    last = now;
    const ui = useGame.getState();
    try {
      if (!ui.paused) advance(now, dtS);
    } catch (err) {
      cancelAnimationFrame(ctx.raf);
      bootHalt(`simulation stopped: ${err?.message ?? err}`);
      return;
    }
    const truth = sim.simTruth();
    if (now > blinkAt + 140) blinkAt = now + 2500 + 3000 * Math.random();
    visuals.sync(truth, moodOf(truth, sim), now > blinkAt ? 0.1 : 1, ui.rays, ui.shapes, ui.camView);
    const q = sim.data.qpos;
    follow.set(q[0], q[2] + 0.02, -q[1]);
    if (ui.chase) {
      delta.subVectors(follow, controls.target);
      controls.target.add(delta);
      camera.position.add(delta);
    }
    controls.update();
    renderer.render(scene, camera);
    if (ui.camView) renderHeadCamera(ctx);
    if (now - lastTelemetry >= TELEMETRY_PERIOD_MS) {
      const rate = (ticksInWindow * sim.dt * 1000) / (now - lastTelemetry);
      useGame.setState({ telemetry: telemetryOf(sim, { rate: ui.paused ? 0 : rate }) });
      lastTelemetry = now;
      ticksInWindow = 0;
    }
  }
  ctx.raf = requestAnimationFrame(frame);
}

// The SDK's robot model if it has been fetched and is usable; otherwise null (the robot is drawn as shapes).
async function loadRobotLook(model) {
  const done = bootLine("SDK ROBOT MODEL");
  const options = new URLSearchParams(window.location.search);
  if (options.get("look") === "shapes") {
    done("SKIPPED: ?look=shapes");
    return null;
  }
  try {
    const sdk = await loadSdkModel(model.values, { faceplate: options.get("faceplate") !== "off", shellVersion: (options.get("shell") ?? (model.robot_model === "vector_2" ? "2" : "1")) === "2" ? 2 : 1 });
    done(sdk === null ? "NOT FETCHED: DRAWING SHAPES" : "OK");
    return sdk;
  } catch (err) {
    console.error("[sdk model]", err);
    done("UNUSABLE: DRAWING SHAPES");
    return null;
  }
}

// The core owns a WebGL context and a WebAssembly heap and cannot be swapped in place:
// during development an edit to it, or to anything it imports, reloads the page.
if (import.meta.hot) import.meta.hot.accept(() => window.location.reload());

let booted = false;

export async function bootGame(canvas) {
  if (booted) return;
  booted = true;
  try {
    const view = makeRenderer(canvas);
    const { mujoco, model, sim, free, navMap } = await loadEverything();
    const sdk = await loadRobotLook(model);
    const doneScene = bootLine("SCENE");
    const cube = free && sdk ? await loadCubeLook(model.values.cube_size_mm) : null;
    const visuals = buildVisuals(mujoco, sim, view.scene, sdk, model.camera, cube, navMap);
    const ctx = { ...view, sim, visuals, driver: makeDriver(sim), input: null, raf: 0 };
    const press = installApi(ctx);
    ctx.input = createInput((name) => press[name]?.());
    doneScene("OK");
    useGame.setState({
      model, level: sim.controlLevel, telemetry: telemetryOf(sim, { rate: 0 }), bootDone: true,
      robotLook: sdk === null ? "shapes" : "sdk",
    });
    if (typeof window !== "undefined") {
      window.__sim = sim; // debug handles, as window.__store
      window.__headCamera = visuals.headCamera.camera;
    }
    startLoop(ctx);
  } catch (err) {
    console.error("[boot]", err);
    bootHalt(err?.message ?? String(err));
  }
}
