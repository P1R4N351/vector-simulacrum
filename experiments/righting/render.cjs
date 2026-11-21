const { chromium } = require("playwright-core");
const fs = require("node:fs"),
  path = require("node:path");
const { spawn } = require("node:child_process");
const { once } = require("node:events");
async function main() {
  const dir = path.resolve(process.argv[2]);
  const sequences = ["right", "left"].map((s) =>
    JSON.parse(fs.readFileSync(path.join(dir, s + ".json"))),
  );
  if (!sequences[0].length || sequences[0].length !== sequences[1].length)
    throw Error("Expected equal, nonempty paired recordings");
  const output = path.join(dir, "righting.mp4");
  const temp = fs.mkdtempSync(path.join(dir, ".render-"));
  let server, b, encoder;
  try {
    const web = path.resolve(__dirname, "../../web");
    const { createServer } = await import(
      path.join(web, "node_modules/vite/dist/node/index.js")
    );
    const sdkAssets = process.env.VECTOR_SDK_ASSETS;
    const useSdkCube =
      sdkAssets &&
      ["cube.obj", "cube.jpg"].every((name) =>
        fs.existsSync(path.join(sdkAssets, name)),
      );
    server = await createServer({
      plugins: useSdkCube
        ? [
            {
              name: "local-sdk-cube",
              configureServer(dev) {
                for (const name of ["cube.obj", "cube.jpg"]) {
                  dev.middlewares.use("/replay-assets/" + name, (_req, res) => {
                    res.setHeader(
                      "Content-Type",
                      name.endsWith("jpg") ? "image/jpeg" : "text/plain",
                    );
                    res.end(fs.readFileSync(path.join(sdkAssets, name)));
                  });
                }
              },
            },
          ]
        : [],
      root: web,
      configFile: false,
      logLevel: "error",
      server: { host: "127.0.0.1", port: 0 },
    });
    await server.listen();
    b = await chromium.launch({
      executablePath: process.env.CHROMIUM_PATH || undefined,
      args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader"],
    });
    const page = await b.newPage();
    await page.goto(server.resolvedUrls.local[0] + "robot/model.json");
    await page.evaluate(
      async ({ boxes, useSdkCube, fixedGrip }) => {
        history.replaceState(null, "", "/");
        const T = await import("/node_modules/three/build/three.module.js"),
          { loadSdkModel } = await import("/src/game/sdkmodel.js");
        const v = (await (await fetch("/robot/model.json")).json()).values;
        const rigs = await Promise.all([loadSdkModel(v), loadSdkModel(v)]);
        const r = new T.WebGLRenderer({
          antialias: true,
          preserveDrawingBuffer: true,
        });
        r.setSize(560, 370);
        r.toneMapping = T.ACESFilmicToneMapping;
        const scene = new T.Scene();
        scene.background = new T.Color(0xc9d7d9);
        const { RoomEnvironment } =
          await import("/node_modules/three/examples/jsm/environments/RoomEnvironment.js");
        scene.environment = new T.PMREMGenerator(r).fromScene(
          new RoomEnvironment(),
        ).texture;
        scene.environmentIntensity = 0.72;
        scene.add(new T.HemisphereLight(0xfff9ee, 0x828fa1, 1.8));
        const light = new T.DirectionalLight(0xfff2da, 2.2);
        light.position.set(0.3, -0.3, 0.6);
        scene.add(light);
        const floor = new T.Mesh(
          new T.PlaneGeometry(3, 3),
          new T.MeshStandardMaterial({ color: 0xbecaca }),
        );
        floor.position.z = -0.0015;
        scene.add(floor);
        const grid = new T.GridHelper(1, 40, 0xa3b2b5, 0xa3b2b5);
        grid.rotation.x = Math.PI / 2;
        grid.position.z = -0.0013;
        scene.add(grid);
        rigs.forEach((g) => scene.add(g.root));
        const cube = new T.Group();
        scene.add(cube);
        if (boxes.length && useSdkCube) {
          const { OBJLoader } =
            await import("/node_modules/three/examples/jsm/loaders/OBJLoader.js");
          const shape = await new OBJLoader().loadAsync(
            "/replay-assets/cube.obj",
          );
          const texture = await new T.TextureLoader().loadAsync(
            "/replay-assets/cube.jpg",
          );
          texture.colorSpace = T.SRGBColorSpace;
          const material = new T.MeshStandardMaterial({
            map: texture,
            roughness: 0.62,
          });
          shape.traverse((mesh) => {
            if (mesh.isMesh) mesh.material = material;
          });
          const bounds = new T.Box3().setFromObject(shape);
          const extent = bounds.getSize(new T.Vector3()),
            center = bounds.getCenter(new T.Vector3());
          shape.scale.set(0.044 / extent.x, 0.044 / extent.y, 0.044 / extent.z);
          shape.position.copy(center).multiply(shape.scale).negate();
          cube.add(shape);
        } else
          for (const [pos, half] of boxes) {
            const mesh = new T.Mesh(
              new T.BoxGeometry(...half.map((x) => 2 * x)),
              new T.MeshStandardMaterial({ color: 0x356979, roughness: 0.65 }),
            );
            mesh.position.fromArray(pos);
            cube.add(mesh);
          }
        const dot = new T.Mesh(
          new T.SphereGeometry(0.0024),
          new T.MeshBasicMaterial({ color: 0xff6425, depthTest: false }),
        );
        dot.renderOrder = 50;
        scene.add(dot);
        const cam = new T.OrthographicCamera(
          -0.159,
          0.159,
          0.105,
          -0.105,
          0.001,
          3,
        );
        cam.up.set(0, 0, 1);
        const sheet = document.createElement("canvas");
        sheet.width = 1128;
        sheet.height = 490;
        const ctx = sheet.getContext("2d");
        window.frame = (frames) => {
          ctx.fillStyle = "#18292f";
          ctx.fillRect(0, 0, 1128, 490);
          ctx.fillStyle = "white";
          ctx.font = "bold 26px sans-serif";
          ctx.fillText(
            boxes.length
              ? fixedGrip
                ? "Patient already engaged / rescuer leverage test"
                : "Cube-assisted righting / adjacent-face approach"
              : "Belly-side approach / lift-assisted righting",
            18,
            33,
          );
          for (const [i, f] of frames.entries()) {
            const s = i === 0 ? 1 : -1;
            cam.position.set(-0.19, s * 0.44, 0.25);
            cam.lookAt(-0.057, 0, 0.036);
            rigs.forEach((g, i) => {
              const p = f.poses[i];
              g.root.position.fromArray(p);
              g.root.quaternion.set(p[4], p[5], p[6], p[3]);
              g.setPose(p[7], p[8], "calm", p.slice(9));
            });
            if (f.cube) {
              cube.position.fromArray(f.cube);
              cube.quaternion.set(f.cube[4], f.cube[5], f.cube[6], f.cube[3]);
            }
            dot.visible = !!f.contact.length;
            if (dot.visible) dot.position.fromArray(f.contact[0]);
            r.render(scene, cam);
            ctx.drawImage(r.domElement, i * 568, 78);
            ctx.font = "19px sans-serif";
            ctx.fillStyle = "white";
            ctx.fillText(
              i === 0 ? "Patient on right side" : "Patient on left side",
              18 + i * 568,
              66,
            );
          }
          ctx.fillText(frames[0].t.toFixed(1) + " s", 18, 476);
          ctx.fillStyle = "#ff6425";
          ctx.beginPath();
          ctx.arc(480, 469, 5, 0, 2 * Math.PI);
          ctx.fill();
          ctx.fillStyle = "white";
          ctx.fillText("Fork contact", 494, 476);
          ctx.fillText("Exploratory simulation", 902, 476);
          return sheet.toDataURL("image/jpeg", 0.94).split(",")[1];
        };
      },
      {
        boxes:
          JSON.parse(fs.readFileSync(path.join(dir, "run.json"))).cube_boxes ??
          [],
        useSdkCube: !!useSdkCube,
        fixedGrip: JSON.parse(fs.readFileSync(path.join(dir, "run.json")))
          .fixed_grip,
      },
    );
    const video = path.join(temp, "video.mp4");
    encoder = spawn(
      "ffmpeg",
      [
        "-loglevel",
        "error",
        "-f",
        "image2pipe",
        "-framerate",
        "15",
        "-i",
        "pipe:0",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        video,
      ],
      { stdio: ["pipe", "inherit", "inherit"] },
    );
    const finished = once(encoder, "close");
    finished.catch(() => {});
    for (
      let frame = 0;
      frame < Math.round(sequences[0].at(-1).t * 15) + 1;
      frame++
    ) {
      const t = frame / 15;
      const pair = sequences.map((seq) =>
        seq.reduce((a, b) => (Math.abs(b.t - t) < Math.abs(a.t - t) ? b : a)),
      );
      const img = await page.evaluate((f) => window.frame(f), pair);
      await new Promise((resolve, reject) =>
        encoder.stdin.write(Buffer.from(img, "base64"), (err) =>
          err ? reject(err) : resolve(),
        ),
      );
    }
    encoder.stdin.end();
    if ((await finished)[0] !== 0) throw Error("ffmpeg failed");
    fs.renameSync(video, output);
    console.log(output);
  } finally {
    if (encoder?.exitCode == null) encoder?.kill();
    await b?.close();
    await server?.close();
    fs.rmSync(temp, { recursive: true, force: true });
  }
}
main().catch((err) => {
  console.error(err);
  process.exitCode = 1;
});
