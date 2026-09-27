/* State shared between the page's modules: the poses, the target, and the flags several modules read
   and write (kept together in S, since an imported `let` can't be assigned from outside its module).
   Anything only one module uses lives in that module. */
import * as THREE from 'three';
import {DEG,N,makeFK,fk} from './kinematics.js';
import {area} from './collision.js';
import {targetObj} from './scene.js';
import {$,mm} from './util.js';

export const S={
  stopped:false,          // Stop pressed (or the arm stopped itself): nothing new is sent until Resume
  homeLock:false,         // qIK was set directly (zero pose, saved pose, playback, jog): the solver leaves it alone
  demo:false,demoT:0,     // Figure-8 running, and its phase
  ikRestart:false,        // the next solve restarts from seeded poses (solve.js, the backend's ik.py)
  ws:null,                // the /ws/arm socket, null when offline
  measured:null,measuredAt:0,   // the real arm's joint angles (degrees, null where a servo didn't answer)
  limp:false,             // hand-guide mode: torque off, the sim follows the arm
  remoteBlocked:null,     // the backend refused the last goal (collision), and why
  armFault:null,          // why the backend stopped the arm (stall guard, bus error)
  play:null,simSpeeds:null,     // local playback (Player) and its per-joint speeds for the sim servos
  remotePlay:null,remotePending:false,remoteStopSent:false,playEndN:null,   // playback on the backend
  armRange:null,          // the backend won't move the arm: a joint reads outside what its servo can reach (why)
  armConfig:null,armState:null,armTorque:null,   // the latest config and state from /ws/arm, torque as it reports it
  wizard:false,           // the calibration wizard is open: it poses the sim arm, and nothing drives the real one
};
export const qIK=new Array(N).fill(0);   // what the solver found (radians)
export const qCmd=new Array(N).fill(0);  // what the servos are told: only collision-free poses get here
export const servo=qIK.map(()=>({pos:0,vel:0}));   // the simulated servos (radians, rad/s)
export const target=new THREE.Vector3(0.16,-0.06,0.12);   // where the solver puts the TCP, base frame

const tx=$('#tx'),ty=$('#ty'),tz=$('#tz');
export function syncUI(){tx.value=mm(target.x);ty.value=mm(target.y);tz.value=mm(target.z);$('#oxv').textContent=mm(target.x);$('#oyv').textContent=mm(target.y);$('#ozv').textContent=mm(target.z);
  $('#sbTarget').textContent=`${mm(target.x)}, ${mm(target.y)}, ${mm(target.z)} mm`;}
export function setTarget(v){target.copy(v);target.z=Math.max(0.003,target.z);targetObj.position.copy(target);syncUI();}
const poseF=makeFK();
export function targetFromPose(){fk(qIK,poseF);setTarget(poseF.tcp.clone());}   // put the target where qIK has the TCP
export function setDemo(on){S.demo=on;$('#btnDemo').setAttribute('aria-pressed',on);}
export function areaDir(){return area.enabled&&area.span<360?area.center*DEG:0;}   // the work area's middle, radians

export const haveRealNow=()=>!!(S.measured&&S.measured.every(v=>v!==null)&&performance.now()-S.measuredAt<1500);
export function recPose(){return haveRealNow()?S.measured.slice():servo.map(s=>s.pos/DEG);}   // degrees: the real arm if it reads back
const textF=makeFK();
export const tcpText=a=>{fk(a.map(v=>v*DEG),textF);return `${mm(textF.tcp.x)}, ${mm(textF.tcp.y)}, ${mm(textF.tcp.z)} mm`;};
