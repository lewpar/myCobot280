/* The simulator page's entry point: wires the modules up (in this order, since later ones read what earlier
   ones set up) and runs the frame loop.

   Each frame: solve IK for the target into qIK (or follow the real arm in hand-guide mode and during backend
   playback), copy qIK to qCmd only if the pose and the path to it are collision-free, move the simulated
   servos toward qCmd with a trapezoidal velocity profile, then draw. link.js streams qCmd to the arm.

   Modules only declare things when imported; everything that touches the page happens in their init*(). */
import * as THREE from 'three';
import {DEG,N,LIM,makeFK,fk,toolLen,toolR} from './kinematics.js';
import {area,outsideArea,checkPose,checkPath} from './collision.js';
import {ikIterate,ikRescue} from './ik.js';
import {renderer,scene,camera,orbit,gizmo,root,rotGroups,ledMats,targetMat,targetObj,dropLine,floorRing,
  ghost,ghostGeo,ghostDots,realLine,realGeo,realDots,trail,pushTrail,pathLine,applyTheme,resize} from './scene.js';
import {S,qIK,qCmd,servo,target,setTarget,targetFromPose,syncUI,setDemo,areaDir,haveRealNow} from './state.js';
import {initApi} from './api.js';
import {initSettings} from './settings.js';
import {initMotion,spd,acc} from './motion.js';
import {initChrome,updateLinkChip,jointUI,pct} from './chrome.js';
import {initLink} from './link.js';
import {initAtom} from './atom.js';
import {initRecord,recTick} from './record.js';
import {initPlay,playStep,playStop,endPlay,playNote,pathWanted} from './play.js';
import {$,cssVar,V} from './util.js';

initChrome();
initSettings();
initMotion();
initLink();
initAtom();
initRecord();
initPlay();
initApi();

// dragging the target's arrows
gizmo.addEventListener('dragging-changed',e=>{orbit.enabled=!e.value;if(e.value){setDemo(false);S.homeLock=false;}});
gizmo.addEventListener('objectChange',()=>{targetObj.position.z=Math.max(0.003,targetObj.position.z);target.copy(targetObj.position);syncUI();});
// click on the floor: move the target there (same height)
const ray=new THREE.Raycaster(),ndc=new THREE.Vector2(),floorPlane=new THREE.Plane(V(0,1,0),0);let down=null;
renderer.domElement.addEventListener('pointerdown',e=>{down={x:e.clientX,y:e.clientY,onGizmo:gizmo.axis!==null};});
renderer.domElement.addEventListener('pointerup',e=>{
  if(!down||down.onGizmo||e.button!==0){down=null;return;}
  const moved=Math.hypot(e.clientX-down.x,e.clientY-down.y);down=null;if(moved>5)return;
  const rect=renderer.domElement.getBoundingClientRect();
  ndc.set((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1);ray.setFromCamera(ndc,camera);
  const hit=new THREE.Vector3();
  if(ray.ray.intersectPlane(floorPlane,hit)&&hit.length()<1){root.worldToLocal(hit);setDemo(false);S.homeLock=false;setTarget(V(hit.x,hit.y,target.z));}
});

if(window.ResizeObserver)new ResizeObserver(resize).observe($('#stage'));else window.addEventListener('resize',resize);
resize();applyTheme();
setTarget(target.clone());
{const b=ikRescue(qIK,target,true);if(b)for(let i=0;i<N;i++)qIK[i]=b.q[i];}

const servoF=makeFK(),ghostF=makeFK(),realF=makeFK();
let ikErr={pos:0,ori:0},lastRescueT=0;
const clock=new THREE.Clock();
function frame(){
  requestAnimationFrame(frame);
  const dt=Math.min(clock.getDelta(),0.05);
  if(S.demo){S.demoT+=dt*0.7;const u=0.23+0.04*Math.sin(2*S.demoT),v=0.08*Math.sin(S.demoT),c=areaDir();   // centred in the work area
    setTarget(V(u*Math.cos(c)-v*Math.sin(c),u*Math.sin(c)+v*Math.cos(c),0.11+0.03*Math.sin(3*S.demoT)));}
  const orient=$('#optDown').checked;
  const haveReal=haveRealNow(),measured=S.measured;
  if(S.play)playStep();
  if(S.remotePlay&&!S.homeLock&&!S.remoteStopSent){S.remoteStopSent=true;playStop();playNote('Stopping playback: the target was moved.');}
  const following=haveReal&&(S.limp||!!S.remotePlay);   // the sim mirrors the real arm
  if(following){for(let i=0;i<N;i++)qIK[i]=qCmd[i]=measured[i]*DEG;targetFromPose();ikErr={pos:0,ori:0};}
  else if(!S.homeLock){
    ikErr=ikIterate(qIK,target,14,orient);
    const stuck=ikErr.pos>0.002||(orient&&ikErr.ori>2*DEG)||!!checkPose(qIK);
    const k=target.toArray().map(v=>v.toFixed(4)).join()+orient+toolLen+toolR;
    if(stuck&&!S.demo&&k!==S.lastRescueKey&&performance.now()-lastRescueT>250){
      S.lastRescueKey=k;lastRescueT=performance.now();
      const b=ikRescue(qIK,target,orient);
      if(b&&(b.r.pos<ikErr.pos-0.001||(b.r.pos<0.002&&b.r.ori<ikErr.ori-DEG))){for(let i=0;i<N;i++)qIK[i]=b.q[i];ikErr=b.r;}
    }
  }else ikErr={pos:0,ori:0};
  // only collision-free poses (and paths) reach the servos
  let blocked=null;
  if(!S.stopped&&!following){
    const cur=servo.map(s=>s.pos);
    blocked=checkPose(qIK)||checkPath(cur,qIK);
    if(!blocked)for(let i=0;i<N;i++)qCmd[i]=qIK[i];
  }
  if(S.play&&blocked)endPlay(`Playback stopped: ${blocked}.`);
  else if(S.play&&S.play.done)endPlay('Playback finished.');

  const vmax=spd.value*DEG,amax=acc.value*DEG;let moving=false;
  for(let i=0;i<N;i++){
    const s=servo[i],err=qCmd[i]-s.pos,vj=S.simSpeeds?S.simSpeeds[i]*DEG:vmax;
    const vdes=Math.sign(err)*Math.min(vj,Math.sqrt(2*amax*Math.abs(err))*0.95);
    s.vel+=THREE.MathUtils.clamp(vdes-s.vel,-amax*dt,amax*dt);
    let step=s.vel*dt;
    if(Math.abs(step)>Math.abs(err)&&Math.sign(step)===Math.sign(err)){step=err;s.vel=0;}
    s.pos=THREE.MathUtils.clamp(s.pos+step,LIM[i][0],LIM[i][1]);
    if(Math.abs(qCmd[i]-s.pos)<3e-4&&Math.abs(s.vel)<0.02){s.pos=qCmd[i];s.vel=0;}
    rotGroups[i].rotation.z=s.pos;
    ledMats[i].emissiveIntensity=0.12+1.6*Math.abs(s.vel)/vmax;
    if(Math.abs(s.vel)>0.01||Math.abs(qCmd[i]-s.pos)>3e-4)moving=true;
    const ui=jointUI[i];ui.pos.style.left=pct(i,s.pos);ui.tgt.style.left=pct(i,qCmd[i]);ui.val.textContent=(s.pos/DEG).toFixed(1)+'°';
  }
  fk(servo.map(s=>s.pos),servoF);
  if(trail.visible)pushTrail(servoF.tcp);
  fk(qIK,ghostF);
  const ga=ghostGeo.attributes.position.array;ga.set([0,0,0.06],0);
  ghostF.pos.forEach((p,k)=>ga.set([p.x,p.y,p.z],(k+1)*3));ga.set([ghostF.tcp.x,ghostF.tcp.y,ghostF.tcp.z],(N+1)*3);
  ghostGeo.attributes.position.needsUpdate=true;
  const gc=blocked?cssVar('--bad'):'#3b82c4';ghost.material.color.set(gc);ghostDots.material.color.set(gc);

  const reachable=ikErr.pos<0.003&&(!orient||ikErr.ori<3*DEG);
  targetMat.color.set(reachable?cssVar('--amber'):cssVar('--bad'));targetMat.emissive.copy(targetMat.color);
  targetObj.scale.setScalar(1+0.12*Math.sin(performance.now()*0.004));
  const lp=dropLine.geometry.attributes.position.array;lp.set([target.x,target.y,target.z,target.x,target.y,0.0005]);
  dropLine.geometry.attributes.position.needsUpdate=true;dropLine.computeLineDistances();
  floorRing.position.set(target.x,target.y,0.0008);

  const st=$('#status');
  if(S.stopped){st.className='status bad';$('#statusText').textContent=S.armFault?'Stopped: '+S.armFault:'Stopped. Press Resume to move again';}
  else if(area.enabled&&outsideArea(target)&&!S.limp){st.className='status bad';$('#statusText').textContent='The target is outside the work area (Robot tab)';}
  else if(blocked||S.remoteBlocked){st.className='status bad';const why=blocked||S.remoteBlocked;
    $('#statusText').textContent=`Blocked: ${why}${blocked?'':' (checked by the arm)'}`;}
  else if(!reachable){st.className='status bad';$('#statusText').textContent=orient&&ikErr.pos<0.003?'Reachable, but not facing straight down':'Out of reach, holding the closest pose';}
  else if(moving){st.className='status';$('#statusText').textContent='Servos moving';}
  else{st.className='status ok';$('#statusText').textContent='At target';}
  $('#errText').textContent=(servoF.tcp.distanceTo(target)*1000).toFixed(1)+' mm';
  updateLinkChip();
  recTick(performance.now());
  pathLine.visible=pathWanted&&!$('#tab-play').hidden;
  const showReal=haveReal&&$('#optReal').checked;
  realLine.visible=realDots.visible=showReal;
  if(showReal){fk(measured.map(v=>v*DEG),realF);const ra=realGeo.attributes.position.array;ra.set([0,0,0.06],0);
    realF.pos.forEach((p,k)=>ra.set([p.x,p.y,p.z],(k+1)*3));ra.set([realF.tcp.x,realF.tcp.y,realF.tcp.z],(N+1)*3);realGeo.attributes.position.needsUpdate=true;}
  for(let i=0;i<N;i++){const r=jointUI[i].real;if(haveReal){r.style.display='block';r.style.left=pct(i,measured[i]*DEG);}else r.style.display='none';}
  orbit.update();renderer.render(scene,camera);
}
frame();
