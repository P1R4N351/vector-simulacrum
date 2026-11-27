// The pinned WireOS animation archive contains one uncompressed JSON member.
const NAME = "anim_gotosleep_wakeup_01";
const URL = "https://raw.githubusercontent.com/os-vector/wire-os-externals/6aec7f1ddf341a898692768650fe945e334fb865/animation-assets/animations/" + NAME + ".tar";
const HASH = "5eb484bf0eb5ea4f4def6abdd7f7b236ea6ae7d840eb2cff317b1ee4edacb588";

export async function readDockWakeTracks(bytes) {
  const digest = new Uint8Array(await crypto.subtle.digest("SHA-256", bytes));
  if (Array.from(digest, b => b.toString(16).padStart(2, "0")).join("") !== HASH) {
    throw Error("WireOS wake archive checksum mismatch");
  }
  const decode = new TextDecoder();
  const size = parseInt(decode.decode(bytes.subarray(124, 136)), 8);
  const clip = JSON.parse(decode.decode(bytes.subarray(512, 512 + size)))[NAME];
  // WireOS locks BODY_TRACK for this clip on the charger platform.
  const head = clip.filter(frame => frame.Name === "HeadAngleKeyFrame");
  const lift = clip.find(frame => frame.Name === "LiftHeightKeyFrame");
  return { head, lift };
}

export async function loadDockWakeTracks() {
  const response = await fetch(URL);
  if (!response.ok) throw Error(`WireOS wake archive returned HTTP ${response.status}`);
  return readDockWakeTracks(new Uint8Array(await response.arrayBuffer()));
}
