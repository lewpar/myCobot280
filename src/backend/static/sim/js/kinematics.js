/* Kinematics: joint origins, orientations and limits from Elephant Robotics' mycobot_280_pi URDF
   (mycobot_ros). Base frame is Z-up, metres. Every joint turns about its local Z.
   Keep JOINTS in sync with URDF_JOINTS / URDF_LIMITS_DEG in arm_model.py (tests/test_page.py checks). */
import * as THREE from 'three';

export const DEG=Math.PI/180;
export const JOINTS=[
  {name:'J1', xyz:[0,0,0.13956],        rpy:[0,0,0],             min:-2.9321,max:2.9321},
  {name:'J2', xyz:[0,0,-0.001],         rpy:[0,1.5708,-1.5708],  min:-2.4434,max:2.4434},
  {name:'J3', xyz:[-0.1104,0,0],        rpy:[0,0,0],             min:-2.6179,max:2.6179},
  {name:'J4', xyz:[-0.096,0,0.06462],   rpy:[0,0,-1.5708],       min:-2.6179,max:2.6179},
  {name:'J5', xyz:[0,-0.07318,-0.001],  rpy:[1.5708,-1.5708,0],  min:-2.7052,max:2.7925},
  {name:'J6', xyz:[0,0.0456,0],         rpy:[-1.5708,0,0],       min:-3.14,  max:3.14159},
];
export const N=JOINTS.length;
export const FIX=JOINTS.map(j=>{const m=new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(j.rpy[0],j.rpy[1],j.rpy[2],'ZYX'));m.setPosition(j.xyz[0],j.xyz[1],j.xyz[2]);return m;});
export const URDF_LIM=JOINTS.map(j=>[j.min,j.max]);
export const LIM=URDF_LIM.map(l=>l.slice());   // narrowed to the servos' own safe range once connected
export const clampJ=(j,deg)=>THREE.MathUtils.clamp(deg*DEG,LIM[j][0],LIM[j][1]);

/* The attachment on the flange, as the kinematics and the collision model see it. Only setToolSize changes it. */
export let toolLen=0;   // length in metres, along the flange normal (flange local +Z); the TCP is its tip
export let toolR=0.01;  // radius in metres (collision model)
export function setToolSize(len,r){toolLen=len;toolR=r;}
/* Attachments: keep in sync with ATTACHMENTS in arm_model.py. Sizes in mm; custom takes the sliders. */
export const ATTACHMENTS={
  none:{name:'No attachment',length:0,diameter:0,note:'Bare flange. The target is the flange centre.'},
  vacuum:{name:'Vacuum suction',length:80,diameter:25,note:'25 mm tube, 80 mm from the flange to the suction cup. The target is the cup; keep "flange facing down" on to pick things up.'},
  custom:{name:'Custom',length:null,diameter:null,note:'A straight tool along the flange axis. Set its length and diameter.'},
};

export function makeFK(){return{pos:FIX.map(()=>new THREE.Vector3()),axis:FIX.map(()=>new THREE.Vector3()),mat:FIX.map(()=>new THREE.Matrix4()),tcp:new THREE.Vector3(),dir:new THREE.Vector3()};}
const _T=new THREE.Matrix4(),_R=new THREE.Matrix4(),_Z=new THREE.Vector3(0,0,1);
export function fk(q,out){ // q in radians; fills out (from makeFK) with joint positions/axes, the TCP and the flange normal
  _T.identity();
  for(let i=0;i<N;i++){
    _T.multiply(FIX[i]);
    out.pos[i].setFromMatrixPosition(_T);
    out.axis[i].copy(_Z).transformDirection(_T);
    _R.makeRotationZ(q[i]);_T.multiply(_R);
    if(out.mat)out.mat[i].copy(_T);   // joint i's frame after its turn (what scene.js rotGroups[i] is)
  }
  out.dir.copy(_Z).transformDirection(_T);
  out.tcp.setFromMatrixPosition(_T).addScaledVector(out.dir,toolLen);
  return out;
}
