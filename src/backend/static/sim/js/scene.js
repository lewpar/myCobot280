/* The 3D scene: renderer, camera, lights, the arm model, target, ghost and trail, the work-area slice.
   Three is Y-up; the robot root turns the Z-up base frame upright. Building it needs WebGL: if that fails,
   this module throws and the page shows #fail (nothing else can start without it). */
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {TransformControls} from 'three/addons/controls/TransformControls.js';
import {DEG,N,FIX,toolLen,toolR,makeFK,fk} from './kinematics.js';
import {COLLISION,area} from './collision.js';
import {$,cssVar,V} from './util.js';

export let renderer;
try{renderer=new THREE.WebGLRenderer({antialias:true});}catch(err){$('#fail').style.display='flex';throw err;}
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
renderer.outputEncoding=THREE.sRGBEncoding;
$('#stage').appendChild(renderer.domElement);
const scene=new THREE.Scene();
const camera=new THREE.PerspectiveCamera(38,1,0.005,20);
camera.position.set(0.62,0.5,0.7);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.target.set(0,0.2,0);orbit.enableDamping=true;orbit.minDistance=0.2;orbit.maxDistance=3;orbit.maxPolarAngle=Math.PI*0.495;

scene.add(new THREE.HemisphereLight(0xffffff,0x8a939b,0.75));
const key=new THREE.DirectionalLight(0xffffff,0.95);key.position.set(0.8,1.4,0.6);key.castShadow=true;
key.shadow.mapSize.set(2048,2048);Object.assign(key.shadow.camera,{left:-0.6,right:0.6,top:0.6,bottom:-0.6,near:0.2,far:4});
key.shadow.bias=-0.0003;scene.add(key);
const rim=new THREE.DirectionalLight(0xbcd4ff,0.35);rim.position.set(-1,0.8,-0.9);scene.add(rim);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(4,4),new THREE.ShadowMaterial({opacity:0.22}));
floor.rotation.x=-Math.PI/2;floor.receiveShadow=true;scene.add(floor);
let grid;
function buildGrid(){
  if(grid)scene.remove(grid);
  const dark=new THREE.Color(cssVar('--scene')||'#d9dee2').getHSL({}).l<0.4;
  grid=new THREE.GridHelper(2,40,dark?0x3a434d:0xa9b2ba,dark?0x232a31:0xc6cdd3);grid.position.y=0.0004;scene.add(grid);
}
const root=new THREE.Group();root.rotation.x=-Math.PI/2;scene.add(root);  // base frame: X,Y on the floor, Z up

const reachRing=new THREE.Mesh(new THREE.RingGeometry(0.278,0.282,128),new THREE.MeshBasicMaterial({color:0xe79a00,transparent:true,opacity:0.5,side:THREE.DoubleSide}));
reachRing.position.z=0.001;root.add(reachRing);
// the work area on the floor: a translucent slice with its edges
const areaMat=new THREE.MeshBasicMaterial({color:0x23845a,transparent:true,opacity:0.1,side:THREE.DoubleSide,depthWrite:false});
const areaEdgeMat=new THREE.LineBasicMaterial({color:0x23845a,transparent:true,opacity:0.7});
const areaG=new THREE.Group();areaG.position.z=0.0006;root.add(areaG);
function drawArea(){
  areaG.children.slice().forEach(c=>{areaG.remove(c);c.geometry.dispose();});
  areaG.visible=area.enabled;if(!area.enabled)return;
  const R=area.radius_mm?area.radius_mm/1000:0.3,r0=Math.max(COLLISION.AREA_CORE,(area.base_mm||0)/1000),full=area.span>=360,a0=(area.center-area.span/2)*DEG,len=Math.min(360,area.span)*DEG;
  areaG.add(new THREE.Mesh(new THREE.RingGeometry(r0,R,96,1,a0,len),areaMat));
  const pts=[];
  if(!full)pts.push(V(Math.cos(a0)*r0,Math.sin(a0)*r0,0));
  for(let k=0;k<=96;k++){const a=a0+len*k/96;pts.push(V(Math.cos(a)*R,Math.sin(a)*R,0));}
  if(!full){const a1=a0+len;pts.push(V(Math.cos(a1)*r0,Math.sin(a1)*r0,0));}
  areaG.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),areaEdgeMat));
}
// small base-frame axes so the X/Y/Z readout is easy to relate to the arm
const axesHelper=new THREE.AxesHelper(0.08);axesHelper.position.set(-0.13,-0.13,0.001);root.add(axesHelper);

/* ---------- Materials ---------- */
const shell=new THREE.MeshStandardMaterial({color:0xf1f2f0,roughness:0.42,metalness:0.05});   // white arm shell
const capMat=new THREE.MeshStandardMaterial({color:0xe3e6e8,roughness:0.38,metalness:0.05});  // servo caps, a shade darker
const baseMat=new THREE.MeshStandardMaterial({color:0xa3a8ad,roughness:0.6,metalness:0.1});   // grey Pi base
const graphite=new THREE.MeshStandardMaterial({color:0x31353c,roughness:0.55,metalness:0.35});
const flangeMat=new THREE.MeshStandardMaterial({color:0xaab0b5,roughness:0.5,metalness:0.1});
const holeMat=new THREE.MeshStandardMaterial({color:0x1f2226,roughness:0.9});
const ledMats=[];
function shade(o){o.traverse(m=>{if(m.isMesh){m.castShadow=true;m.receiveShadow=true;}});return o;}

/* Visual layout, authored in the base frame at the zero pose, following a photo of the real arm (servo_ids.png).
   Housing centres sit on each joint's axis line, so the geometry matches the kinematics.
   J1 is the seam where the white column leaves the grey base; J2, J3 and J4 sit in one line ~30 mm to -Y of the
   base axis and the tube links zigzag either side of them (link 1 at y=0, link 2 at y=-68 mm, link 3 back at y=0,
   link 4 out to the J5 axis at y=-63.6 mm). The J5 body carries the ATOM head (LED face backwards, -X, opposite the flange) and the J6
   servo, whose flange (3x3 Lego peg holes) is the only part J6 turns.
   s = where the seam (the glowing motion ring) sits along the housing axis, -1..1. */
const HOUSINGS=[
  {c:V(0,0,0.074),       a:V(0,0,1), r:0.0255,L:0.016},
  {c:V(0,-0.035,0.1386), a:V(0,1,0), r:0.030, L:0.036},
  {c:V(0,-0.031,0.249),  a:V(0,1,0), r:0.028, L:0.032},
  {c:V(0,-0.029,0.345),  a:V(0,1,0), r:0.024, L:0.028},
  {c:V(0,-0.0636,0.381), a:V(0,0,1), r:0.0205,L:0.014},
  {c:V(0.023,-0.0636,0.4181),a:V(1,0,0),r:0.021,L:0.034,s:1},
];
const LINKS=[ // link k moves with joint k: a round tube through pts (rounded at the corners), housing k to housing k+1
  {r:0.0245,pts:[V(0,0,0.074),V(0,0,0.1386),V(0,-0.035,0.1386)]},
  {r:0.023, pts:[V(0,-0.035,0.1386),V(0,-0.068,0.1386),V(0,-0.068,0.249),V(0,-0.031,0.249)]},
  {r:0.021, pts:[V(0,-0.031,0.249),V(0,0,0.249),V(0,0,0.345),V(0,-0.029,0.345)]},
  {r:0.020, pts:[V(0,-0.029,0.345),V(0,-0.0636,0.345),V(0,-0.0636,0.381)]},
  {r:0.019, pts:[V(0,-0.0636,0.381),V(0,-0.0636,0.4181)],end:true},
];
function makeHousing(h){
  const g=new THREE.Group();
  g.add(new THREE.Mesh(new THREE.CylinderGeometry(h.r,h.r,h.L,40),capMat));
  const led=new THREE.MeshStandardMaterial({color:0x3a3d42,emissive:0xffa000,emissiveIntensity:0.15,roughness:0.4});
  ledMats.push(led);
  const ring=new THREE.Mesh(new THREE.TorusGeometry(h.r*1.005,Math.max(h.r*0.05,0.0009),8,48),led);
  ring.rotation.x=Math.PI/2;ring.position.y=(h.s||0)*h.L/2;g.add(ring);
  g.quaternion.setFromUnitVectors(V(0,1,0),h.a);g.position.copy(h.c);
  return shade(g);
}
function makeLink(l){
  const g=new THREE.Group(),ball=new THREE.SphereGeometry(l.r,32,16);
  l.pts.forEach((p,k)=>{
    if(k>0){const q=l.pts[k-1],d=p.clone().sub(q),len=d.length();
      const c=new THREE.Mesh(new THREE.CylinderGeometry(l.r,l.r,len,32),shell);
      c.quaternion.setFromUnitVectors(V(0,1,0),d.normalize());c.position.copy(p).add(q).multiplyScalar(0.5);g.add(c);}
    if((k>0&&k<l.pts.length-1)||(l.end&&k===l.pts.length-1)){const b=new THREE.Mesh(ball,shell);b.position.copy(p);g.add(b);}
  });
  return shade(g);
}
const atomDotMats=[];  // one per LED, index row*5+x as on the ATOM; tinted from the LED panel's state
function makeAtomHead(){ // on the J5 body, behind the J5 axis on the J6 line, LED matrix facing backwards (-X)
  const g=new THREE.Group(),c=V(-0.012,-0.0636,0.4181);
  const head=new THREE.Mesh(new THREE.CylinderGeometry(0.0155,0.0155,0.036,40),shell);
  head.rotation.z=Math.PI/2;head.position.copy(c);g.add(head);
  const faceX=c.x-0.018;
  const panel=new THREE.Mesh(new THREE.BoxGeometry(0.003,0.021,0.021),new THREE.MeshStandardMaterial({color:0x3b3f44,roughness:0.5}));
  panel.position.set(faceX-0.0015,c.y,c.z);g.add(panel);
  const dotG=new THREE.BoxGeometry(0.0008,0.0022,0.0022);
  for(let k=0;k<25;k++){ // seen from behind the arm (looking along +X): x runs toward -Y, rows run downward
    const m=new THREE.MeshStandardMaterial({color:0x1d5a33,emissive:0x2bd46a,emissiveIntensity:0.55});atomDotMats.push(m);
    const d=new THREE.Mesh(dotG,m);d.position.set(faceX-0.0033,c.y-(k%5-2)*0.0038,c.z-(Math.floor(k/5)-2)*0.0038);g.add(d);}
  return shade(g);
}

// base: the grey Pi base, ports on its -X face, and the collar the J1 seam sits on
const base=new THREE.Group();root.add(base);
const foot=new THREE.Mesh(new THREE.BoxGeometry(0.092,0.092,0.062),baseMat);foot.position.z=0.031;base.add(foot);
const collar=new THREE.Mesh(new THREE.CylinderGeometry(0.028,0.042,0.012,48),baseMat);collar.rotation.x=Math.PI/2;collar.position.z=0.068;base.add(collar);
[ // [left,right,top,bottom] as fractions of the face seen from -X (left = +Y, top = z 62 mm), colour
  [0.23,0.40,0.23,0.37,0xc83232],  // power switch
  [0.60,0.77,0.22,0.41,0x26292d],  // power jack
  [0.20,0.82,0.44,0.50,0x26292d],  // GPIO header
  [0.20,0.36,0.67,0.88,0x33373c],  // USB 2
  [0.41,0.56,0.67,0.88,0x2d6fb7],  // USB 3
  [0.62,0.80,0.67,0.88,0x33373c],  // Ethernet
].forEach(([u0,u1,v0,v1,col])=>{
  const w=(u1-u0)*0.092,h=(v1-v0)*0.062;
  const m=new THREE.Mesh(new THREE.BoxGeometry(0.002,w,h),new THREE.MeshStandardMaterial({color:col,roughness:0.6}));
  m.position.set(-0.0465,0.046-(u0+u1)/2*0.092,0.062-(v0+v1)/2*0.062);base.add(m);
});
shade(base);
root.add(makeHousing(HOUSINGS[0]));

// joint chain: fixed origin group -> rotating group; link k visuals are re-expressed in its zero-pose frame
const rotGroups=[];const zeroInv=[];
{let parent=root;const T=new THREE.Matrix4();
 for(let i=0;i<N;i++){
   const o=new THREE.Group();o.matrixAutoUpdate=false;o.matrix.copy(FIX[i]);parent.add(o);
   const r=new THREE.Group();o.add(r);rotGroups.push(r);
   T.multiply(FIX[i]);zeroInv.push(T.clone().invert());parent=r;
 }}
function attach(obj,k){obj.applyMatrix4(zeroInv[k]);rotGroups[k].add(obj);}
for(let k=0;k<N-1;k++){attach(makeLink(LINKS[k]),k);attach(makeHousing(HOUSINGS[k+1]),k);}
attach(makeAtomHead(),4);
// J6 flange with the 3x3 grid of Lego peg holes (in J6's own frame the flange normal is +Z, face at z=0)
const flangeG=new THREE.Group();rotGroups[N-1].add(flangeG);
const flange=new THREE.Mesh(new THREE.CylinderGeometry(0.019,0.019,0.005,40),flangeMat);flange.rotation.x=Math.PI/2;flange.position.z=-0.0025;flangeG.add(flange);
const holeG=new THREE.CylinderGeometry(0.0024,0.0024,0.0003,20);
for(const hx of [-0.008,0,0.008])for(const hy of [-0.008,0,0.008]){
  const h=new THREE.Mesh(holeG,holeMat);h.rotation.x=Math.PI/2;h.position.set(hx,hy,0.0001);flangeG.add(h);}
const mark=new THREE.Mesh(new THREE.BoxGeometry(0.003,0.004,0.001),holeMat);mark.position.set(0,0.0155,0.0003);flangeG.add(mark); // shows J6 rotation
const toolStub=new THREE.Mesh(new THREE.CylinderGeometry(1,1,1,24),graphite);toolStub.rotation.x=Math.PI/2;flangeG.add(toolStub);  // custom tool
// vacuum suction attachment, built along +Z from the flange face: mount plate, 25 mm body, hose barb, suction cup
const vacuumG=new THREE.Group();flangeG.add(vacuumG);
{const rubber=new THREE.MeshStandardMaterial({color:0x202226,roughness:0.85});
 const body=new THREE.MeshStandardMaterial({color:0x5b6470,roughness:0.35,metalness:0.55});
 const add=(geo,mat,z,rot=true)=>{const m=new THREE.Mesh(geo,mat);if(rot)m.rotation.x=Math.PI/2;m.position.z=z;vacuumG.add(m);return m;};
 add(new THREE.CylinderGeometry(0.0165,0.0165,0.004,40),graphite,0.002);             // mount plate on the peg holes
 add(new THREE.CylinderGeometry(0.0125,0.0125,0.058,40),body,0.004+0.029);           // body, 25 mm
 add(new THREE.CylinderGeometry(0.0132,0.0132,0.003,40),graphite,0.02);              // knurled collar
 add(new THREE.CylinderGeometry(0.0085,0.006,0.006,32),body,0.065);                  // neck
 const barb=add(new THREE.CylinderGeometry(0.003,0.003,0.014,16),graphite,0.03,false);barb.rotation.z=Math.PI/2;barb.position.x=0.018;  // hose barb
 add(new THREE.CylinderGeometry(0.0125,0.0065,0.012,40,1,true),rubber,0.074);        // suction cup, open at the tip
 const lip=add(new THREE.TorusGeometry(0.0122,0.0009,8,40),rubber,0.0795,false);}
// translucent envelope the collision check uses for the attachment
const envelope=new THREE.Mesh(new THREE.CylinderGeometry(1,1,1,32,1,true),new THREE.MeshBasicMaterial({color:0x8b5cf6,transparent:true,opacity:0.18,depthWrite:false,side:THREE.DoubleSide}));
envelope.rotation.x=Math.PI/2;envelope.visible=false;flangeG.add(envelope);
shade(flangeG);
const tcpDot=new THREE.Mesh(new THREE.SphereGeometry(0.004,16,12),new THREE.MeshBasicMaterial({color:0xff5a36}));flangeG.add(tcpDot);
function applyTool(attachment){
  const custom=attachment==='custom'&&toolLen>0.002;
  toolStub.visible=custom;toolStub.scale.set(toolR,Math.max(toolLen,0.001),toolR);toolStub.position.z=toolLen/2;
  vacuumG.visible=attachment==='vacuum';
  envelope.scale.set(toolR,Math.max(toolLen,0.001),toolR);envelope.position.z=toolLen/2;
  envelope.visible=toolLen>0.002&&$('#optEnvelope').checked;
  tcpDot.position.z=toolLen;
}

/* ---------- Target, ghost, trail (all in the base frame) ---------- */
const targetMat=new THREE.MeshStandardMaterial({color:0xe79a00,emissive:0xe79a00,emissiveIntensity:0.55,transparent:true,opacity:0.9});
const targetObj=new THREE.Mesh(new THREE.SphereGeometry(0.009,24,16),targetMat);root.add(targetObj);   // state.setTarget moves it
const dropLine=new THREE.Line(new THREE.BufferGeometry().setFromPoints([V(0,0,0),V(0,0,0)]),new THREE.LineDashedMaterial({color:0x888888,dashSize:0.008,gapSize:0.005}));
root.add(dropLine);
const floorRing=new THREE.Mesh(new THREE.RingGeometry(0.01,0.013,40),new THREE.MeshBasicMaterial({color:0x888888,side:THREE.DoubleSide}));root.add(floorRing);
const ghostGeo=new THREE.BufferGeometry();ghostGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array((N+2)*3),3));
const ghost=new THREE.Line(ghostGeo,new THREE.LineBasicMaterial({color:0x3b82c4,transparent:true,opacity:0.85,depthTest:false}));
const ghostDots=new THREE.Points(ghostGeo,new THREE.PointsMaterial({color:0x3b82c4,size:0.008,depthTest:false}));
ghost.renderOrder=ghostDots.renderOrder=5;ghost.frustumCulled=ghostDots.frustumCulled=false;root.add(ghost);root.add(ghostDots);
const realGeo=new THREE.BufferGeometry();realGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array((N+2)*3),3));
const realLine=new THREE.Line(realGeo,new THREE.LineBasicMaterial({color:0x2f9a5a,transparent:true,opacity:0.9,depthTest:false}));
const realDots=new THREE.Points(realGeo,new THREE.PointsMaterial({color:0x2f9a5a,size:0.009,depthTest:false}));
realLine.renderOrder=realDots.renderOrder=6;realLine.frustumCulled=realDots.frustumCulled=false;realLine.visible=realDots.visible=false;root.add(realLine);root.add(realDots);
const TRAIL=600;const trailGeo=new THREE.BufferGeometry();
trailGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(TRAIL*3),3));
const trail=new THREE.Line(trailGeo,new THREE.LineBasicMaterial({color:0xff5a36,transparent:true,opacity:0.75}));trail.frustumCulled=false;root.add(trail);
let trailCount=0;const lastTrail=V(1e9,0,0);
function clearTrail(){trailCount=0;trailGeo.setDrawRange(0,0);}
function pushTrail(p){
  if(p.distanceToSquared(lastTrail)<0.0015*0.0015)return;lastTrail.copy(p);
  const a=trailGeo.attributes.position.array;
  if(trailCount<TRAIL)trailCount++;else a.copyWithin(0,3);
  a.set([p.x,p.y,p.z],(trailCount-1)*3);trailGeo.setDrawRange(0,trailCount);trailGeo.attributes.position.needsUpdate=true;
}
function applyTheme(){
  const c=new THREE.Color(cssVar('--scene')||'#d9dee2');scene.background=c;scene.fog=new THREE.Fog(c,2,5);buildGrid();
  reachRing.material.color.set(cssVar('--amber')||'#e79a00');
  areaMat.color.set(cssVar('--good')||'#23845a');areaEdgeMat.color.set(cssVar('--good')||'#23845a');
}

// the arrows on the target (main.js wires what dragging them does)
const gizmo=new TransformControls(camera,renderer.domElement);gizmo.setSize(0.7);gizmo.setSpace('local');
gizmo.attach(targetObj);scene.add(gizmo);

/* the selected recording's tool path, drawn while the Play tab is open (play.js fills it) */
const PATH_MAX=3000,pathGeo=new THREE.BufferGeometry();
pathGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(PATH_MAX*3),3));
const pathLine=new THREE.Line(pathGeo,new THREE.LineBasicMaterial({color:0x8b5cf6,transparent:true,opacity:0.9,depthTest:false}));
pathLine.renderOrder=4;pathLine.frustumCulled=false;pathLine.visible=false;root.add(pathLine);
/* Calibration wizard effects: a glowing ring around one joint and an arrow showing which way is positive
   (right-handed about the joint's axis), both drawn over the arm. updateJointFx places them each frame. */
const fxMat=new THREE.MeshBasicMaterial({color:0xe79a00,transparent:true,opacity:0.85,depthTest:false});
const fxArrowMat=new THREE.MeshBasicMaterial({color:0x3b82c4,transparent:true,opacity:0.95,depthTest:false});
const halo=new THREE.Mesh(new THREE.TorusGeometry(0.036,0.0022,10,64),fxMat);
const arrowG=new THREE.Group(),ARC=Math.PI*1.35,AR=0.05;
{const arc=new THREE.Mesh(new THREE.TorusGeometry(AR,0.0028,8,48,ARC),fxArrowMat);arrowG.add(arc);
 const cone=new THREE.Mesh(new THREE.ConeGeometry(0.0085,0.02,20),fxArrowMat);
 cone.position.set(AR*Math.cos(ARC),AR*Math.sin(ARC),0);cone.rotation.z=ARC;   // cone points +Y: along the arc's tangent
 arrowG.add(cone);}
const fxG=new THREE.Group();fxG.add(halo);fxG.add(arrowG);fxG.visible=false;root.add(fxG);
fxG.traverse(o=>{o.renderOrder=7;o.frustumCulled=false;});
let fxSpec=null,fxSpin=0;
const _zAxis=V(0,0,1);
function setJointFx(spec){fxSpec=spec;fxG.visible=!!spec;}   // spec: {joint 0-5, arrow: bool} or null
function updateJointFx(F,dt){ // F: fk output of the pose on screen
  if(!fxSpec)return;
  const j=fxSpec.joint;fxG.position.copy(F.pos[j]);fxG.quaternion.setFromUnitVectors(_zAxis,F.axis[j]);
  const pulse=0.5+0.5*Math.sin(performance.now()*0.006);
  fxMat.opacity=0.45+0.45*pulse;halo.scale.setScalar(1+0.08*pulse);
  arrowG.visible=!!fxSpec.arrow;fxSpin=(fxSpin+dt*1.4)%(Math.PI*2);arrowG.rotation.z=fxSpin*0.25;
}
/* ---------- Motion Studio sandbox: the same scene from another camera, into another canvas, with the arm in
   the sandbox's own pose and none of the main view's overlays (target, ghost, real arm, trail, path). Only
   one of the two views is drawn each frame. ---------- */
const sandPathGeo=new THREE.BufferGeometry();
sandPathGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(PATH_MAX*3),3));
const sandPath=new THREE.Line(sandPathGeo,new THREE.LineBasicMaterial({color:0x8b5cf6,transparent:true,opacity:0.9,depthTest:false}));
sandPath.renderOrder=4;sandPath.frustumCulled=false;sandPath.visible=false;root.add(sandPath);
const sandMark=new THREE.Mesh(new THREE.SphereGeometry(0.008,20,14),new THREE.MeshStandardMaterial({color:0xe79a00,emissive:0xe79a00,emissiveIntensity:0.6}));
sandMark.visible=false;root.add(sandMark);
let sand=null,sandPathOn=false,sandMarkAt=null;
function makeSandbox(el){
  const r=new THREE.WebGLRenderer({antialias:true});
  r.setPixelRatio(Math.min(devicePixelRatio,2));r.shadowMap.enabled=true;r.shadowMap.type=THREE.PCFSoftShadowMap;r.outputEncoding=THREE.sRGBEncoding;
  el.appendChild(r.domElement);
  const cam=new THREE.PerspectiveCamera(38,1,0.005,20);cam.position.set(0.55,0.42,0.62);
  const orb=new OrbitControls(cam,r.domElement);orb.target.set(0,0.18,0);orb.enableDamping=true;orb.minDistance=0.2;orb.maxDistance=3;orb.maxPolarAngle=Math.PI*0.495;
  const fit=()=>{const w=Math.max(1,el.clientWidth),h=Math.max(1,el.clientHeight);r.setSize(w,h);cam.aspect=w/h;cam.updateProjectionMatrix();};
  if(window.ResizeObserver)new ResizeObserver(fit).observe(el);else window.addEventListener('resize',fit);
  // arrows on the selected point (a Move to point block): drag them to move it; onMove/onDrag are the Studio's
  const giz=new TransformControls(cam,r.domElement);giz.setSize(0.7);giz.setSpace('local');giz.enabled=false;scene.add(giz);
  fit();sand={renderer:r,camera:cam,orbit:orb,fit,gizmo:giz,onMove:null,onDrag:null,dragging:false};
  giz.addEventListener('dragging-changed',e=>{orb.enabled=!e.value;sand.dragging=e.value;if(sand.onDrag)sand.onDrag(e.value);});
  giz.addEventListener('objectChange',()=>{sandMark.position.z=Math.max(0.003,sandMark.position.z);   // not below the table
    if(sand.onMove)sand.onMove(sandMark.position.clone());});
  return sand;
}
function setSandPath(points){ // base-frame Vector3s, or null
  sandPathOn=!!(points&&points.length>1);if(!sandPathOn)return;
  const a=sandPathGeo.attributes.position.array,n=Math.min(points.length,PATH_MAX);
  for(let k=0;k<n;k++)a.set([points[k].x,points[k].y,points[k].z],k*3);
  sandPathGeo.setDrawRange(0,n);sandPathGeo.attributes.position.needsUpdate=true;
}
function setSandMark(p){sandMarkAt=p?p.clone():null;}
const sandGizmo=()=>sand&&sand.gizmo;   // (for tests)
/* rings round each joint, to turn it by dragging in the sandbox (a selected pose block); the Studio picks one
   with pickSandJoint and marks the one under the pointer or being dragged with setSandJoints */
const ringMats=[],rings=[],ringF=makeFK();
for(let j=0;j<N;j++){
  const m=new THREE.MeshBasicMaterial({color:0x3b82c4,transparent:true,opacity:0.55,depthTest:false});
  const r=new THREE.Mesh(new THREE.TorusGeometry(j===0?0.05:0.04,0.0045,10,56),m);
  r.renderOrder=8;r.visible=false;r.userData.joint=j;root.add(r);rings.push(r);ringMats.push(m);
}
let ringsOn=false,ringHot=-1;
const _zA=V(0,0,1);
function setSandJoints(on,hot=-1){ringsOn=on;ringHot=hot;}
function placeRings(q){fk(q,ringF);rings.forEach((r,j)=>{r.position.copy(ringF.pos[j]);r.quaternion.setFromUnitVectors(_zA,ringF.axis[j]);r.updateMatrixWorld();});}
/* which ring (joint number) is under the sandbox canvas point (clientX, clientY), or -1; with the joint's centre
   and axis, for turning it: {joint, centre, axis} (base frame) */
const _ray=new THREE.Raycaster(),_ndc=new THREE.Vector2();
function pickSandJoint(x,y,q){
  if(!sand||!ringsOn)return null;
  placeRings(q);rings.forEach(r=>{r.visible=true;});
  const rc=sand.renderer.domElement.getBoundingClientRect();
  _ndc.set((x-rc.left)/rc.width*2-1,-(y-rc.top)/rc.height*2+1);_ray.setFromCamera(_ndc,sand.camera);
  const hit=_ray.intersectObjects(rings,false)[0];rings.forEach(r=>{r.visible=false;});
  if(!hit)return null;
  const j=hit.object.userData.joint;
  return {joint:j,centre:ringF.pos[j].clone(),axis:ringF.axis[j].clone()};
}
/* a base-frame point on the sandbox canvas, in client pixels (for measuring a drag round a joint) */
function sandToScreen(p){
  const v=p.clone().applyMatrix4(root.matrixWorld).project(sand.camera),rc=sand.renderer.domElement.getBoundingClientRect();
  return {x:rc.left+(v.x+1)/2*rc.width,y:rc.top+(1-v.y)/2*rc.height};
}
const sandCamPos=()=>{const p=sand.camera.position.clone();return root.worldToLocal(p);};   // the camera, base frame
const MAIN_ONLY=()=>[targetObj,dropLine,floorRing,ghost,ghostDots,realLine,realDots,trail,pathLine,gizmo,fxG];
function renderSandbox(q,led){ // q: radians; led: [r,g,b] for the ATOM while previewing, or null
  if(!sand)return;
  const hidden=MAIN_ONLY().map(o=>[o,o.visible]),rot=rotGroups.map(g=>g.rotation.z);
  hidden.forEach(([o])=>{o.visible=false;});
  rotGroups.forEach((g,i)=>{g.rotation.z=q[i];});
  if(ringsOn){placeRings(q);rings.forEach((r,j)=>{r.visible=true;ringMats[j].color.set(j===ringHot?0xe79a00:0x3b82c4);ringMats[j].opacity=j===ringHot?0.95:0.5;});}
  sandPath.visible=sandPathOn;sandMark.visible=!!sandMarkAt;
  if(sandMarkAt&&!sand.dragging)sandMark.position.copy(sandMarkAt);   // (while dragging, the arrows move it)
  const giz=sand.gizmo;
  if(sandMarkAt){if(giz.object!==sandMark)giz.attach(sandMark);giz.enabled=true;}
  else if(giz.object){giz.detach();giz.enabled=false;}
  giz.visible=!!sandMarkAt;
  const dots=led?atomDotMats.map(m=>[m,m.color.clone(),m.emissive.clone(),m.emissiveIntensity]):null;
  if(led){const [r,g,b]=led,on=r+g+b>0;atomDotMats.forEach(m=>{m.color.setRGB(on?r/255*0.4:0.05,on?g/255*0.4:0.05,on?b/255*0.4:0.05);
    m.emissive.setRGB(r/255,g/255,b/255);m.emissiveIntensity=on?1.2:0;});}
  sand.orbit.update();sand.renderer.render(scene,sand.camera);
  if(dots)dots.forEach(([m,c,e,i])=>{m.color.copy(c);m.emissive.copy(e);m.emissiveIntensity=i;});
  sandPath.visible=sandMark.visible=false;giz.visible=false;rings.forEach(r=>{r.visible=false;});   // (none of it in the main view)
  rotGroups.forEach((g,i)=>{g.rotation.z=rot[i];});hidden.forEach(([o,v])=>{o.visible=v;});
}

/* ---------- Obstacles (the Workspace view): drawn in every view; the Workspace edits them ---------- */
const obstG=new THREE.Group();root.add(obstG);
const OGEO={box:new THREE.BoxGeometry(1,1,1),cylinder:new THREE.CylinderGeometry(0.5,0.5,1,48).rotateX(Math.PI/2),sphere:new THREE.SphereGeometry(0.5,40,24)};
const OEDGE={box:new THREE.EdgesGeometry(OGEO.box),cylinder:new THREE.EdgesGeometry(OGEO.cylinder,30),sphere:null};
const obstMeshes=new Map();   // obstacle id -> mesh
/* make the meshes match the obstacles (base frame, mm and degrees); sel: the selected one's id; hits: ids the
   arm touches, drawn red */
function setObstacleMeshes(obs,sel=null,hits=new Set()){
  const keep=new Set(obs.map(o=>o.id));
  for(const [id,m] of obstMeshes)if(!keep.has(id)){obstG.remove(m);obstMeshes.delete(id);}
  for(const o of obs){
    let m=obstMeshes.get(o.id);
    if(!m||m.userData.shape!==o.shape){
      if(m)obstG.remove(m);
      m=new THREE.Mesh(OGEO[o.shape],new THREE.MeshStandardMaterial({roughness:0.55,metalness:0.05,transparent:true,opacity:0.8}));
      m.castShadow=m.receiveShadow=true;m.userData={id:o.id,shape:o.shape};
      if(OEDGE[o.shape]){const e=new THREE.LineSegments(OEDGE[o.shape],new THREE.LineBasicMaterial({color:0x000000,transparent:true,opacity:0.25}));m.add(e);}
      obstG.add(m);obstMeshes.set(o.id,m);
    }
    m.position.set(o.pos[0]/1000,o.pos[1]/1000,o.pos[2]/1000);
    m.rotation.set(o.rot[0]*DEG,o.rot[1]*DEG,o.rot[2]*DEG);   // Euler XYZ, as the collision check
    m.scale.set(o.size[0]/1000,(o.shape==='box'?o.size[1]:o.size[0])/1000,(o.shape==='sphere'?o.size[0]:o.size[2])/1000);
    const hit=hits.has(o.id),on=o.id===sel;
    m.material.color.set(o.color);m.material.emissive.set(hit?0xc8402c:on?0xe79a00:0x000000);m.material.emissiveIntensity=hit?0.55:on?0.25:0;
  }
}
const obstacleMesh=id=>obstMeshes.get(id)||null;

/* the Workspace's own view: another canvas and camera on the same scene, the arm in a reference pose, a finer
   grid (1 cm), and move/rotate/size handles on the selected obstacle */
const workGrid=new THREE.GridHelper(1.2,120,0x9aa3ab,0xc9d0d6);workGrid.position.y=0.0006;workGrid.material.transparent=true;workGrid.material.opacity=0.5;
workGrid.visible=false;scene.add(workGrid);
let work=null;
function makeWorkView(el){
  const r=new THREE.WebGLRenderer({antialias:true});
  r.setPixelRatio(Math.min(devicePixelRatio,2));r.shadowMap.enabled=true;r.shadowMap.type=THREE.PCFSoftShadowMap;r.outputEncoding=THREE.sRGBEncoding;
  el.appendChild(r.domElement);
  const cam=new THREE.PerspectiveCamera(38,1,0.005,20);cam.position.set(0.75,0.65,0.8);
  const orb=new OrbitControls(cam,r.domElement);orb.target.set(0.12,0.08,0);orb.enableDamping=true;orb.minDistance=0.15;orb.maxDistance=4;orb.maxPolarAngle=Math.PI*0.495;
  const fit=()=>{const w=Math.max(1,el.clientWidth),h=Math.max(1,el.clientHeight);r.setSize(w,h);cam.aspect=w/h;cam.updateProjectionMatrix();};
  if(window.ResizeObserver)new ResizeObserver(fit).observe(el);else window.addEventListener('resize',fit);
  const giz=new TransformControls(cam,r.domElement);giz.setSize(0.8);giz.setSpace('local');giz.enabled=false;giz.visible=false;scene.add(giz);
  work={renderer:r,camera:cam,orbit:orb,fit,gizmo:giz,dragging:false,onChange:null,onDrag:null};
  giz.addEventListener('dragging-changed',e=>{orb.enabled=!e.value;work.dragging=e.value;if(work.onDrag)work.onDrag(e.value);});
  giz.addEventListener('objectChange',()=>{if(work.onChange&&giz.object)work.onChange(giz.object);});
  fit();return work;
}
function setWorkGizmo(id,mode,snap){ // the handles on this obstacle (or none), in 'translate' | 'rotate' | 'scale' mode
  if(!work)return;const g=work.gizmo,m=id&&obstMeshes.get(id);
  if(!m){if(g.object)g.detach();g.enabled=false;return;}
  if(g.object!==m)g.attach(m);g.enabled=true;g.setMode(mode);g.setSpace(mode==='translate'?'world':'local');
  g.setTranslationSnap(snap?0.01:null);g.setRotationSnap(snap?15*DEG:null);g.setScaleSnap(snap?0.01:null);
}
function pickObstacle(x,y){ // the obstacle id under a Workspace canvas point, or null
  if(!work)return null;
  const rc=work.renderer.domElement.getBoundingClientRect();
  _ndc.set((x-rc.left)/rc.width*2-1,-(y-rc.top)/rc.height*2+1);_ray.setFromCamera(_ndc,work.camera);
  const hit=_ray.intersectObjects([...obstMeshes.values()],false)[0];
  return hit?hit.object.userData.id:null;
}
const workGizmo=()=>work&&work.gizmo;   // (for tests)
function workToScreen(p){ // a base-frame point on the Workspace canvas, in client pixels
  const v=p.clone().applyMatrix4(root.matrixWorld).project(work.camera),rc=work.renderer.domElement.getBoundingClientRect();
  return {x:rc.left+(v.x+1)/2*rc.width,y:rc.top+(1-v.y)/2*rc.height};
}
function renderWorkView(q){ // q: the arm's reference pose (radians)
  if(!work)return;
  const hidden=[...MAIN_ONLY(),sandPath,sandMark,...(sand?[sand.gizmo]:[])].map(o=>[o,o.visible]),rot=rotGroups.map(g=>g.rotation.z);
  hidden.forEach(([o])=>{o.visible=false;});
  rotGroups.forEach((g,i)=>{g.rotation.z=q[i];});
  workGrid.visible=true;work.gizmo.visible=!!work.gizmo.object;
  work.orbit.update();work.renderer.render(scene,work.camera);
  workGrid.visible=false;work.gizmo.visible=false;
  rotGroups.forEach((g,i)=>{g.rotation.z=rot[i];});hidden.forEach(([o,v])=>{o.visible=v;});
}

/* Shift the picture sideways by px (positive: right), e.g. so the arm isn't hidden behind the wizard panel. */
let viewShift=0;
function setViewShift(px){viewShift=px;resize();}
function resize(){const st=$('#stage'),w=Math.max(1,st.clientWidth),h=Math.max(1,st.clientHeight);renderer.setSize(w,h);camera.aspect=w/h;
  if(viewShift&&w>760)camera.setViewOffset(w,h,-viewShift,0,w,h);else camera.clearViewOffset();
  camera.updateProjectionMatrix();}

export {scene,camera,orbit,gizmo,root,drawArea,ledMats,atomDotMats,rotGroups,applyTool,
  targetMat,targetObj,dropLine,floorRing,ghost,ghostGeo,ghostDots,realLine,realGeo,realDots,
  trail,clearTrail,pushTrail,PATH_MAX,pathGeo,pathLine,applyTheme,resize,setJointFx,updateJointFx,setViewShift,
  makeSandbox,renderSandbox,setSandPath,setSandMark,sandGizmo,setSandJoints,pickSandJoint,sandToScreen,sandCamPos,
  setObstacleMeshes,obstacleMesh,makeWorkView,setWorkGizmo,pickObstacle,renderWorkView,workGizmo,workToScreen};
