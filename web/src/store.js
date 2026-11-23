// Bridge between the imperative simulator core and the React readouts.
// (pollen-robotics, Apache-2.0): state flows game -> store, intents flow
// UI -> gameApi. Telemetry is throttled where it is produced (10 Hz), so
// React never renders at frame rate.
import { create } from "zustand";

export const useGame = create(() => ({
  bootLog: [], // { label, status } ; status null while pending
  bootDone: false,
  bootError: null,
  model: null, // model.json: values, derived, provenance counts, hashes
  level: "default",
  paused: false,
  chase: true,
  rays: true,
  shapes: false, // physics shapes drawn over the SDK model
  camView: true, // the robot's own camera view
  robotLook: "shapes", // "sdk" when the SDK's robot model is drawn
  padConnected: false,
  telemetry: null,
  lastAction: null,
  notice: null,
  // A program written in the page: status idle | running | finished | failed | stopped.
  program: { status: "idle", errorLine: null, log: [], waitingOn: null },
}));

// Imperative surface, filled in by game/game.js once it has booted.
export const gameApi = {};

export function bootLine(label) {
  const entry = { label, status: null };
  useGame.setState((s) => ({ bootLog: [...s.bootLog, entry] }));
  return (status = "OK") => {
    entry.status = status;
    useGame.setState((s) => ({ bootLog: [...s.bootLog] }));
  };
}

export function bootHalt(detail) {
  if (useGame.getState().bootError !== null) return;
  useGame.setState({ bootError: String(detail) });
}

if (typeof window !== "undefined") {
  window.__store = useGame;
  window.__gameApi = gameApi;
}
