import assert from "node:assert/strict";
import { test } from "node:test";
import { faceTexture } from "../src/game/wire_face.js";
test("WireOS RGB565 scanlines decode to the LCD texture", () => {
  const lcd=faceTexture(), bytes=Buffer.alloc(184*96*2);
  bytes.writeUInt16LE(0xf800,0);
  bytes.writeUInt16LE(0x07e0,2);
  bytes.writeUInt16LE(0x001f,184*2);
  lcd.update(bytes.toString("base64"));
  assert.deepEqual(Array.from(lcd.texture.image.data.slice(0,8)),[255,0,0,255,0,255,0,255]);
  assert.deepEqual(Array.from(lcd.texture.image.data.slice(184*4,184*4+4)),[0,0,255,255]);
  lcd.texture.dispose();
});
