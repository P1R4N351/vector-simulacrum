import { useEffect, useRef } from "react";
import { bootGame } from "./game/game.js";
import { useGame } from "./store.js";
import BootOverlay from "./ui/BootOverlay.jsx";
import Hud from "./ui/Hud.jsx";

export default function App() {
  const canvasRef = useRef(null);
  const bootDone = useGame((s) => s.bootDone);
  const bootError = useGame((s) => s.bootError);
  useEffect(() => {
    bootGame(canvasRef.current);
  }, []);
  return (
    <>
      <canvas ref={canvasRef} className="stage" aria-label="Simulated Vector robot on a table" />
      {bootDone && bootError === null ? <Hud /> : null}
      {!bootDone || bootError !== null ? <BootOverlay /> : null}
    </>
  );
}
