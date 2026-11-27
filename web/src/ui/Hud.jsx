// Status, readouts and controls over the 3D view..
import { useEffect, useRef, useState } from "react";
import { gameApi, useGame } from "../store.js";
import MiniMap from "./MiniMap.jsx";
import ProgramPanel from "./ProgramPanel.jsx";

// toFixed keeps the sign of a value that rounds to zero ("-0"); drop it.
const fmt = (value, digits = 0) => (Number.isFinite(value) ? value.toFixed(digits).replace(/^-(0(\.0+)?)$/, "$1") : "-");
// What the robot makes of its range reading, by its own rules.
const rangeVerdict = (t) => (t.prox_lift ? "lift in the beam" : t.prox_found ? "object" : t.prox_unobstructed ? "clear" : "not counted");
const pair = (values, digits = 0) => `${fmt(values[0], digits)} / ${fmt(values[1], digits)}`;
const KEYS = [
  ["W A S D / arrows", "drive (Shift: faster)"], ["R F", "head up / down, while held"], ["T G", "lift up / down, while held"],
  ["1 2", "forward / back 100 mm"], ["3 4", "turn left / right 90°"], ["5 6", "head to its top / bottom"],
  ["7 8", "lift to its top / bottom"], ["Space", "stop (a program too)"],
  ["X", "take control back"], ["O", "cliff reaction on / off"], ["Backspace", "reset"],
  ["C  V  B  Q  P", "follow camera, sensor beams, physics shapes, robot camera, pause"],
];
const RESULT_TEXT = {
  completed: "completed", cancelled_cliff: "cancelled by a cliff", no_control: "refused: no control",
  timeout: "timed out", aborted_off_ground: "aborted: off the ground", cancelled: "cancelled by Stop",
};

function Chip({ tone, children }) {
  return <span className={`chip ${tone}`}>{children}</span>;
}

function StatusChips({ t, level, program }) {
  const control = t.fallen ? ["bad", "FELL OFF THE TABLE"]
    : t.reacting ? ["warn", "REACTING TO A CLIFF"]
      : t.has_control ? ["ok", "HAS CONTROL"] : ["warn", "NO CONTROL · PRESS X"];
  return (
    <div className="chips">
      <Chip tone={level === "default" ? "ok" : "bad"}>{level === "default" ? "CLIFF REACTION ON" : "CLIFF REACTION OFF"}</Chip>
      <Chip tone={control[0]}>{control[1]}</Chip>
      {t.cliff_detected ? <Chip tone="bad">CLIFF</Chip> : null}
      {t.picked_up ? <Chip tone="warn">PICKED UP</Chip> : null}
      {t.falling ? <Chip tone="bad">FALLING</Chip> : null}
      {t.action_running ? <Chip tone="info">ACTION RUNNING</Chip> : null}
      {t.on_charger ? <Chip tone="ok">ON CHARGER</Chip>
        : t.on_charger_platform ? <Chip tone="info">CHARGER RAMP</Chip> : null}
      {t.firmware_active ? <Chip tone="ok">FIRMWARE ARMv6-M</Chip> : null}
      {t.nomad_ready ? <Chip tone="ok">NOMAD DIFFUSION</Chip> : null}
      {t.autonomy_phase ? <Chip tone="warn">{t.autonomy_phase}</Chip> : null}
      {t.has_patient ? (
        <Chip tone={t.patient_upright ? "ok" : "warn"}>{t.patient_upright ? "PATIENT UPRIGHT" : "PATIENT SIDEWAYS"}</Chip>
      ) : null}
      {program === "running" ? <Chip tone="info">PROGRAM RUNNING</Chip> : null}
    </div>
  );
}

function CliffPads({ cliff }) {
  const names = ["front left", "front right", "back left", "back right"];
  return (
    <div className="cliffpads" aria-label="Cliff sensors">
      {cliff.map((flag, i) => <span key={names[i]} className={flag ? "pad bad" : "pad"} title={`${names[i]}: ${flag ? "cliff" : "surface"}`} />)}
    </div>
  );
}

function Readouts({ t }) {
  const rows = [
    ["position", `${fmt(t.x_mm)}, ${fmt(t.y_mm)} mm`], ["heading", `${fmt(t.heading_deg, 1)}°`],
    ["to nearest edge", t.edge_margin_mm < 0 ? "off the table" : `${fmt(t.edge_margin_mm)} mm`], ["wheels commanded", `${pair(t.wheel_cmd)} mm/s`],
    ["wheels measured", `${pair(t.wheel_meas)} mm/s`], ["motor power", pair(t.motor_power, 2)],
    ["front wheels (motor)", `${pair(t.front_rpm)} rpm`], ["rear wheels (idler)", `${pair(t.rear_rpm)} rpm`],
    ["battery", `${fmt(t.battery_v, 2)} V`], ["tread heat", pair(t.tread_heat, 3)],
    ["head", `${fmt(t.head_deg, 1)}°`], ["lift", `${fmt(t.lift_mm, 1)} mm`],
    ["range sensor", `${fmt(t.prox_mm)} mm \u00b7 ${rangeVerdict(t)}`], ["range signal", fmt(t.prox_quality, 3)],
    ["robot time", `${fmt(t.time_s, 1)} s`], ["speed", `${fmt(t.rate, 2)}× real time`],
  ];
  if (t.cube_slot_g) {
    const axes = ["unknown", "−X", "+X", "−Y", "+Y", "−Z", "+Z"];
    const countWithTime = (count, time) => time == null ? String(count) : `${count} · last at ${fmt(time, 2)} s`;
    rows.push(["cube tap broadcasts", countWithTime(t.cube_tap_broadcasts, t.cube_last_tap_broadcast_s)],
      ["cube movement broadcasts", t.cube_movement_broadcasts],
      ["cube motion transitions", countWithTime(t.cube_motion_transitions, t.cube_last_motion_s)],
      ["cube acceleration", t.cube_slot_g.map(g => fmt(g, 2)).join(" / ") + " g"],
      ["cube sensor", axes[t.cube_axis] + " up · " + (t.cube_moving ? "motion detected" : "no motion detected")]);
  }
  if (t.has_patient) {
    rows.push(["patient tilt", `${fmt(t.patient_tilt_deg, 1)}° · ${t.patient_side ?? ""}`],
      ["rescue stage", t.rescue_stage || "idle"],
      ["rescuer clear", t.rescuer_clear ? "yes" : "in contact"]);
  }
  return (
    <dl className="readouts">
      {rows.map(([label, value]) => (
        <div key={label}><dt>{label}</dt><dd>{value}</dd></div>
      ))}
    </dl>
  );
}

function Notice() {
  const notice = useGame((s) => s.notice);
  const [shown, setShown] = useState(null);
  useEffect(() => {
    setShown(notice);
    if (notice === null) return undefined;
    const timer = setTimeout(() => setShown(null), 2600);
    return () => clearTimeout(timer);
  }, [notice]);
  return shown ? <div className="notice" role="status">{shown.text}</div> : null;
}

function TouchPad() {
  const hold = (fwd, turn) => ({
    onPointerDown: (e) => { e.currentTarget.setPointerCapture(e.pointerId); gameApi.setPad?.(fwd, turn); },
    onPointerUp: () => gameApi.setPad?.(0, 0),
    onPointerCancel: () => gameApi.setPad?.(0, 0),
  });
  return (
    <div className="touchpad" aria-label="Drive">
      <button type="button" className="up" aria-label="Forward" {...hold(1, 0)}>{"▲"}</button>
      <button type="button" className="left" aria-label="Turn left" {...hold(0, 1)}>{"◀"}</button>
      <button type="button" className="right" aria-label="Turn right" {...hold(0, -1)}>{"▶"}</button>
      <button type="button" className="down" aria-label="Back" {...hold(-1, 0)}>{"▼"}</button>
    </div>
  );
}

// The frame round the robot's own camera view. The picture itself is drawn on the canvas behind it.
function CameraView({ camera }) {
  return (
    <figure className="camview">
      <div className="camview-window" style={{ aspectRatio: `${camera.width_px} / ${camera.height_px}` }} />
      <figcaption>
        Robot camera {"·"} {camera.width_px} {"×"} {camera.height_px} {"·"} {fmt(camera.fov_horizontal_deg, 1)}{"°"} {"×"}{" "}
        {fmt(camera.fov_vertical_deg, 1)}{"°"} {"·"} {fmt(camera.pitch_down_deg)}{"°"} down
      </figcaption>
    </figure>
  );
}

function About({ model, robotLook }) {
  const c = model.provenance_counts;
  const total = Object.values(c).reduce((a, b) => a + b, 0);
  return (
    <details className="about">
      <summary>What this is</summary>
      <p>
        Vector {model.robot_model === "vector_2" ? "2.0" : "1.0"} uses the same MuJoCo model and control law as the Python simulacrum.
        Of its {total} parameters, {c.measured} are measured on hardware, {c.fitted} are
        fitted to it, {c.manual} come from the manual, {c.sdk} from vendor or household software, {c.assumed} are
        still assumed and {c.chosen} are scene and solver settings.
      </p>
      <p>
        {robotLook === "sdk"
          ? "The robot drawn is the 3D model from the Vector Python SDK's viewer. Press B to see the collision shapes. Each wheel turns by the physics model's angle for it: the front wheels are the motor-driven ones and the rear wheels are turned by the track. The tread cleats follow the belt. When installed, the three replacement-part STLs supply the outer wheels and lift fork."
          : "The robot is drawn using its collision shapes. The tread bands and the face are added for the eye."}
        {" "}The pose shown is the simulator's true pose.
      </p>
      <p>
        The robot camera view uses one real Vector's factory calibration ({fmt(model.camera.fov_horizontal_deg, 1)}{"°"} by{" "}
        {fmt(model.camera.fov_vertical_deg, 1)}{"°"}; the manual says {model.camera.documents.manual_deg[0]}{"°"} by{" "}
        {model.camera.documents.manual_deg[1]}{"°"}), at the firmware's mounting point, tilted {fmt(model.camera.pitch_down_deg)}{"°"} down as
        the robot's engine has it. It has no lens distortion, exposure or noise.
      </p>
      <p className="hash">table {model.source.table_sha256.slice(0, 12)} · model {model.source.xml_sha256.slice(0, 12)}</p>
    </details>
  );
}

function drawRgbThumb(canvas, rgbU8) {
  if (!canvas || !rgbU8 || rgbU8.length !== 64 * 36 * 3) return;
  const ctx = canvas.getContext("2d");
  const img = ctx.createImageData(64, 36);
  for (let p = 0; p < 64 * 36; p++) {
    img.data[4 * p] = rgbU8[3 * p];
    img.data[4 * p + 1] = rgbU8[3 * p + 1];
    img.data[4 * p + 2] = rgbU8[3 * p + 2];
    img.data[4 * p + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

function NomadCard({ telemetry }) {
  const obsRef = useRef(null);
  const goalRef = useRef(null);
  const snap = gameApi.nomad?.();
  useEffect(() => {
    if (!snap) return;
    drawRgbThumb(obsRef.current, snap.obsU8);
    drawRgbThumb(goalRef.current, snap.goalU8);
  }, [telemetry, snap?.nodes.length, snap?.hasGoal]);
  if (!snap?.ready) return null;
  return (
    <div className="nomad-card" aria-label="NoMaD Visual Navigation">
      <div className="nomad-meta">
        NoMaD {snap.nodes.length} nodes {"·"} {snap.edges.length} edges {"·"} dist {fmt(snap.predDist, 1)}
      </div>
      <div className="nomad-cams">
        <div className="nomad-thumb">
          <span>obs 64{"×"}36</span>
          <canvas ref={obsRef} width={64} height={36} aria-label="NoMaD observation frame" />
        </div>
        <div className="nomad-thumb">
          <span>{snap.hasGoal ? `goal #${snap.goalNodeId ?? 0}` : "goal none"}</span>
          <canvas ref={goalRef} width={64} height={36} aria-label="NoMaD goal frame" />
        </div>
      </div>
    </div>
  );
}

export default function Hud() {
  const t = useGame((s) => s.telemetry);
  const level = useGame((s) => s.level);
  const paused = useGame((s) => s.paused);
  const chase = useGame((s) => s.chase);
  const model = useGame((s) => s.model);
  const lastAction = useGame((s) => s.lastAction);
  const padConnected = useGame((s) => s.padConnected);
  const robotLook = useGame((s) => s.robotLook);
  const hasDock = useGame((s) => s.hasDock);
  const startDocked = useGame((s) => s.startDocked);
  const hasPatient = useGame((s) => s.hasPatient);
  const hasCube = useGame((s) => s.hasCube);
  const rescueSide = useGame((s) => s.rescueSide);
  const shapes = useGame((s) => s.shapes);
  const camView = useGame((s) => s.camView);
  const programStatus = useGame((s) => s.program.status);
  const [tab, setTab] = useState("controls");
  if (!t || !model) return null;
  const press = (name) => () => gameApi.press?.(name);
  return (
    <div className="hud">
      <header className="panel top-left">
        <h1>Vector simulacrum</h1>
        <StatusChips t={t} level={level} program={programStatus} />
        {paused ? <div className="paused">PAUSED</div> : null}
      </header>
      <section className="panel right">
        <MiniMap />
        <NomadCard telemetry={t} />
        <CliffPads cliff={t.cliff} />
        <Readouts t={t} />
        {lastAction ? (
          <p className="last-action">
            Last action: {RESULT_TEXT[lastAction.result] ?? lastAction.result}, moved {fmt(lastAction.moved_mm, 1)} mm,
            turned {fmt(lastAction.turned_deg, 1)}{"°"} in {fmt(lastAction.ticks * 0.005, 2)} s
          </p>
        ) : null}
        <About model={model} robotLook={robotLook} />
      </section>
      <section className="panel bottom-left">
        <div className="tabs" role="tablist" aria-label="Panel">
          {[["controls", "Controls"], ["program", "Program"]].map(([id, label]) => (
            <button key={id} type="button" role="tab" aria-selected={tab === id} onClick={() => setTab(id)}>{label}</button>
          ))}
        </div>
        {tab === "program" ? <ProgramPanel /> : <Controls press={press} level={level} chase={chase} robotLook={robotLook}
          hasDock={hasDock} startDocked={startDocked} hasPatient={hasPatient} hasCube={hasCube} rescueSide={rescueSide}
          shapes={shapes} camView={camView} paused={paused} padConnected={padConnected} nomadReady={t.nomad_ready} />}
      </section>
      {camView ? <CameraView camera={model.camera} /> : null}
      {tab === "program" ? null : <TouchPad />}
      <Notice />
    </div>
  );
}

function RescueControls({ hasPatient, hasCube, rescueSide, press }) {
  if (!hasPatient) {
    return (
      <div className="buttons">
        <button type="button" onClick={press("openRescueScene")}>Rescue scene</button>
      </div>
    );
  }
  return (
    <div className="buttons">
      <button type="button" onClick={press("rescueRun")}>Run rescue macro</button>
      {hasCube ? <button type="button" onClick={press("rescueDirect")}>Direct rescue</button> : null}
      <button type="button" onClick={press("toggleRescueSide")}>Patient: {rescueSide}</button>
      <button type="button" onClick={press("openTableScene")}>Table scene</button>
    </div>
  );
}

function Controls({ press, level, chase, robotLook, hasDock, startDocked, hasPatient, hasCube, rescueSide, shapes, camView, paused, padConnected, nomadReady }) {
  return (
    <>
    <div className="buttons">
      <button type="button" onClick={press("forward100")}>Forward 100 mm</button>
      <button type="button" onClick={press("back100")}>Back 100 mm</button>
      <button type="button" onClick={press("turnLeft90")}>Turn left 90{"°"}</button>
      <button type="button" onClick={press("turnRight90")}>Turn right 90{"°"}</button>
      <button type="button" onClick={press("stop")}>Stop</button>
    </div>
    <div className="buttons">
      <button type="button" onClick={press("explore")}>Explore</button>
      <button type="button" onClick={press("dock")}>Dock</button>
      {hasDock ? <button type="button" onClick={press("undock")}>Undock</button> : null}
    </div>
    {nomadReady ? (
      <div className="buttons">
        <button type="button" onClick={press("nomadGoal")}>Capture goal</button>
        <button type="button" onClick={press("nomadExplore")}>NoMaD explore</button>
        <button type="button" onClick={press("nomadNavigate")}>NoMaD navigate</button>
      </div>
    ) : null}
    <div className="buttons">
      <button type="button" onClick={press("headUp")}>Head up</button>
      <button type="button" onClick={press("headLevel")}>Head level</button>
      <button type="button" onClick={press("headDown")}>Head down</button>
      <button type="button" onClick={press("liftUp")}>Lift up</button>
      <button type="button" onClick={press("liftDown")}>Lift down</button>
    </div>
    <div className="buttons">
      <button type="button" onClick={press("control")}>Take control</button>
      <button type="button" className={level === "default" ? "" : "danger"} onClick={press("level")}>
        {level === "default" ? "Turn cliff reaction off" : "Turn cliff reaction on"}
      </button>
      <button type="button" onClick={press("chase")}>{chase ? "Free camera" : "Follow robot"}</button>
      {robotLook === "sdk" ? (
        <button type="button" onClick={press("shapes")}>{shapes ? "Hide physics shapes" : "Show physics shapes"}</button>
      ) : null}
      <button type="button" onClick={press("camera")}>{camView ? "Hide robot camera" : "Show robot camera"}</button>
      <button type="button" onClick={press("pause")}>{paused ? "Resume" : "Pause"}</button>
      {hasDock ? (
        <button type="button" onClick={press("toggleDockStart")}>{startDocked ? "Start off charger" : "Seat on charger"}</button>
      ) : null}
      <button type="button" onClick={press("reset")}>Reset</button>
    </div>
    <RescueControls hasPatient={hasPatient} hasCube={hasCube} rescueSide={rescueSide} press={press} />
    <ul className="keys">
      {KEYS.map(([key, what]) => <li key={key}><kbd>{key}</kbd><span>{what}</span></li>)}
      {padConnected ? <li><kbd>Gamepad</kbd><span>left stick drives, right stick tilts the head</span></li> : null}
    </ul>
    </>
  );
}
