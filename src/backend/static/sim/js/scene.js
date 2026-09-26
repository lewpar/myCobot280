/* The 3D scene: renderer, camera, lights, the arm model, target, ghost and trail, the work-area slice.
   Three is Y-up; the robot root turns the Z-up base frame upright. Building it needs WebGL: if that fails,
   this module throws and the page shows #fail (nothing else can start without it). */
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {TransformControls} from 'three/addons/controls/TransformControls.js';
import {DEG,N,FIX,toolLen,toolR} from './kinematics.js';
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
function resize(){const st=$('#stage'),w=Math.max(1,st.clientWidth),h=Math.max(1,st.clientHeight);renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();}

export {scene,camera,orbit,gizmo,root,drawArea,ledMats,atomDotMats,rotGroups,applyTool,
  targetMat,targetObj,dropLine,floorRing,ghost,ghostGeo,ghostDots,realLine,realGeo,realDots,
  trail,clearTrail,pushTrail,PATH_MAX,pathGeo,pathLine,applyTheme,resize};
