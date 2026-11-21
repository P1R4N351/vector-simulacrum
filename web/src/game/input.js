// Keyboard, gamepad and on-screen pad, merged into one drive intent.
//
// Held keys give a continuous intent (forward, turn, head, lift); single
// presses call a handler by name. Nothing here knows about the simulator.

const HELD = {
  KeyW: "fwd", ArrowUp: "fwd", KeyS: "back", ArrowDown: "back", KeyA: "left", ArrowLeft: "left",
  KeyD: "right", ArrowRight: "right", KeyR: "headUp", KeyF: "headDown", KeyT: "liftUp", KeyG: "liftDown",
  ShiftLeft: "fast", ShiftRight: "fast",
};
const PRESSED = {
  Space: "stop", Backspace: "reset", KeyC: "chase", KeyO: "level", KeyX: "control", KeyP: "pause",
  KeyV: "rays", KeyB: "shapes", KeyQ: "camera", Digit5: "headUp", Digit6: "headDown", Digit7: "liftUp", Digit8: "liftDown", Digit1: "forward100", Digit2: "back100", Digit3: "turnLeft90", Digit4: "turnRight90",
};
const DEADZONE = 0.15;
const PAD_BUTTONS = { 0: "stop", 1: "control", 2: "reset", 3: "level" }; // A, B, X, Y on a standard pad

function axis(value) {
  return Math.abs(value) < DEADZONE ? 0 : value;
}

export function createInput(onPress) {
  const held = new Set();
  const pad = { fwd: 0, turn: 0 }; // on-screen pad, -1..1
  const padWas = new Map();
  const isTyping = (event) => ["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName);

  const down = (event) => {
    if (isTyping(event) || event.metaKey || event.ctrlKey || event.altKey) return;
    if (HELD[event.code]) {
      held.add(HELD[event.code]);
      event.preventDefault();
    } else if (PRESSED[event.code]) {
      if (!event.repeat) onPress(PRESSED[event.code]);
      event.preventDefault();
    }
  };
  const up = (event) => {
    if (HELD[event.code]) held.delete(HELD[event.code]);
  };
  const clear = () => held.clear();
  window.addEventListener("keydown", down);
  window.addEventListener("keyup", up);
  window.addEventListener("blur", clear);

  function readGamepad() {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    const gp = Array.from(pads).find((p) => p && p.connected && p.mapping === "standard") ?? null;
    if (gp === null) return null;
    for (const [index, name] of Object.entries(PAD_BUTTONS)) {
      const now = Boolean(gp.buttons[index]?.pressed);
      if (now && !padWas.get(index)) onPress(name);
      padWas.set(index, now);
    }
    return {
      fwd: -axis(gp.axes[1] ?? 0), turn: -axis(gp.axes[0] ?? 0), head: -axis(gp.axes[3] ?? 0),
      lift: (gp.buttons[5]?.pressed ? 1 : 0) - (gp.buttons[4]?.pressed ? 1 : 0),
      fast: Boolean(gp.buttons[7]?.pressed),
    };
  }

  // One merged intent: fwd and turn in -1..1 (turn positive is counter-clockwise).
  function read() {
    const key = (plus, minus) => (held.has(plus) ? 1 : 0) - (held.has(minus) ? 1 : 0);
    const gp = readGamepad();
    const clip = (x) => Math.max(-1, Math.min(1, x));
    return {
      fwd: clip(key("fwd", "back") + pad.fwd + (gp?.fwd ?? 0)),
      turn: clip(key("left", "right") + pad.turn + (gp?.turn ?? 0)),
      head: clip(key("headUp", "headDown") + (gp?.head ?? 0)),
      lift: clip(key("liftUp", "liftDown") + (gp?.lift ?? 0)),
      fast: held.has("fast") || Boolean(gp?.fast),
      padConnected: gp !== null,
    };
  }

  return {
    read,
    setPad(fwd, turn) {
      pad.fwd = fwd;
      pad.turn = turn;
    },
    dispose() {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
      window.removeEventListener("blur", clear);
    },
  };
}
