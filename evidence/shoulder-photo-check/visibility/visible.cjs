// Render the complete web rig in a crop of a calibrated native photo.
// Input R,t transform robot millimetres into optical x-right, y-down, z-forward.
const fs = require('node:fs');
const {chromium} = require(process.env.PLAYWRIGHT_CORE || 'playwright-core');
async function main() {
  const [cameraFile, originalFile, prefix] = process.argv.slice(2);
  const camera = JSON.parse(fs.readFileSync(cameraFile));
  const original = JSON.parse(fs.readFileSync(originalFile));
  const browser = await chromium.launch({
    executablePath:process.env.CHROMIUM || '/Applications/Chromium.app/Contents/MacOS/Chromium',
    args:['--enable-unsafe-swiftshader','--use-angle=swiftshader']
  });
  try {
    const page = await browser.newPage();
    await page.goto((process.env.VECTOR_WEB || 'http://localhost:5198')+'/robot/model.json');
    const result = await page.evaluate(async ({camera,original})=>{
      history.replaceState(null,'','/');
      const T=await import('/node_modules/three/build/three.module.js');
      const {loadSdkModel}=await import('/src/game/sdkmodel.js');
      const values=(await(await fetch('/robot/model.json')).json()).values;
      const rig=await loadSdkModel(values,{shellVersion:2,faceplate:false});
      rig.setPose(-0.86*Math.PI/180,-0.198,'calm',[0,0,0,0]);
      const scene=new T.Scene();scene.background=new T.Color(0);scene.add(rig.root);
      const shell=rig.root.getObjectByName('Backpack and rear cover');
      const r=new T.WebGLRenderer({antialias:false,preserveDrawingBuffer:true});
      r.outputColorSpace=T.LinearSRGBColorSpace;
      const materials=[0x646464,0xff0000].map(color=>new T.MeshBasicMaterial({color,side:T.DoubleSide}));
      const nodes=[];
      rig.root.traverse(o=>{if(o.isMesh){
        nodes.push([o,o.visible]);o.material=materials[Number(o===shell)];
      }});
      const cam=new T.PerspectiveCamera();
      const {R,t}=camera;
      const focal=camera.f ?? camera.focal_px;
      const f=Array.isArray(focal)?focal:[focal,focal];
      const pp=camera.pp ?? camera.principal_point;
      cam.matrixAutoUpdate=false;
      cam.matrix.set(...R[0],t[0]/1000,...R[1].map(x=>-x),-t[1]/1000,...R[2].map(x=>-x),-t[2]/1000,0,0,0,1).invert();
      cam.matrixWorldNeedsUpdate=true;cam.updateMatrixWorld(true);
      const g=shell.geometry;rig.root.updateMatrixWorld(true);
      const orig=g.attributes.position.array.slice();
      const b=new T.Box2();const v=new T.Vector3();
      for(let i=0;i<g.attributes.position.count;i++){
        v.fromBufferAttribute(g.attributes.position,i).applyMatrix4(shell.matrixWorld).applyMatrix4(cam.matrix.clone().invert());
        b.expandByPoint(new T.Vector2(pp[0]-f[0]*v.x/v.z,pp[1]+f[1]*v.y/v.z));
      }
      const x=Math.floor(b.min.x)-18,y=Math.floor(b.min.y)-18;
      const w=Math.ceil(b.max.x)-x+18,h=Math.ceil(b.max.y)-y+18;
      if(Math.max(w,h)>4400||Math.min(w,h)<10)throw Error('Unexpected crop '+[x,y,w,h]);
      const scale=Math.min(12,Math.floor(4000/Math.max(w,h)));
      const near=.001;
      cam.projectionMatrix.makePerspective((x-pp[0])*near/f[0],(x+w-pp[0])*near/f[0],
        (pp[1]-y)*near/f[1],(pp[1]-y-h)*near/f[1],near,5);
      r.setSize(w*scale,h*scale);
      const n=g.attributes.position.count;
      const colors=new Float32Array(n*3);
      for(let i=0;i<n;i++){
        const k=i*3;
        colors[k]=1;
        // Green records displacement in mm, scaled so 1 mm maps to 255.
        colors[k+1]=10*Math.hypot(orig[k]-original[k],orig[k+1]-original[k+1],orig[k+2]-original[k+2]);
      }
      g.setAttribute('color',new T.BufferAttribute(colors,3));
      materials[1]=new T.MeshBasicMaterial({vertexColors:true,side:T.DoubleSide});
      const images={};
      for(const [model,position] of [['old',original],['new',orig]]){
        g.attributes.position.array.set(position);g.attributes.position.needsUpdate=true;
        for(const only of [false,true]){
          for(const [o,visible] of nodes) o.visible=visible&&(!only||o===shell||o.children.length>0);
          // Preserve the shell's parent transform but suppress the parent's own triangles.
          for(const [o] of nodes)o.material=only&&o!==shell?new T.MeshBasicMaterial({visible:false}):materials[Number(o===shell)];
          r.render(scene,cam);images[model+(only?'shell':'rig')]=r.domElement.toDataURL().split(',')[1];
        }
      }
      return {x,y,w,h,scale,images};
    },{camera,original});
    for(const [name,b64] of Object.entries(result.images))fs.writeFileSync(prefix+'-'+name+'.png',Buffer.from(b64,'base64'));
    delete result.images;fs.writeFileSync(prefix+'-crop.json',JSON.stringify(result));
  } finally {await browser.close();}
}
main().catch(e=>{console.error(e);process.exit(1)});
