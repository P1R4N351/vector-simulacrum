// Run against the normal Vite server with optional SDK assets installed.
const {chromium} = require("../../experiments/righting/node_modules/playwright-core");
const fs = require("node:fs");
const path = require("node:path");

(async () => {
  const browser = await chromium.launch({executablePath:process.env.CHROMIUM,
    args:["--use-gl=angle","--use-angle=swiftshader","--enable-unsafe-swiftshader"]});
  try {
    const page=await browser.newPage();
    await page.goto(process.argv[2]);
    const result=await page.evaluate(async () => {
      const T=await import("/node_modules/three/build/three.module.js");
      const {loadCubeLook}=await import("/src/game/cube.js");
      const cube=await loadCubeLook(44);
      if (!cube) throw Error("SDK cube required");
      const renderer=new T.WebGLRenderer({preserveDrawingBuffer:true,antialias:true});
      renderer.setSize(460,460);
      const scene=new T.Scene();
      scene.background=new T.Color(0x767676);
      scene.add(cube,new T.AmbientLight(0xffffff,Math.PI));
      const original=await new T.TextureLoader().loadAsync("/robot/sdk/cube.jpg");
      original.colorSpace=T.SRGBColorSpace;
      const camera=new T.OrthographicCamera(-.023,.023,.023,-.023,.001,1);
      const views={};
      for (const axis of ["x","y","z"]) for (const sign of [-1,1]) {
        camera.position.set(0,0,0);
        camera.position[axis]=sign*.15;
        camera.up.set(0,axis==="z"?1:0,axis==="z"?0:1);
        camera.lookAt(0,0,0);
        for (const mode of ["corrected","previous"]) {
          let material;
          cube.traverse(n=>{if(n.isMesh)material=n.material;});
          const stock=material.map;
          if(mode==="previous")material.map=original;
          renderer.render(scene,camera);
          views[axis+sign+mode]=renderer.domElement.toDataURL().split(",")[1];
          material.map=stock;
        }
      }
      return views;
    });
    fs.mkdirSync(process.argv[3],{recursive:true});
    for (const [name,data] of Object.entries(result))
      fs.writeFileSync(path.join(process.argv[3],name+".png"),Buffer.from(data,"base64"));
  } finally { await browser.close(); }
})().catch(e=>{console.error(e);process.exitCode=1;});
