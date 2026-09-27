/* The simulator page's entry point: wires the modules up (in this order, since later ones read what earlier
   ones set up) and runs the frame loop.

   Each frame: take the backend's latest IK answer (solve.js: /ws/ik, or the arm's own solve while link.js
   drives it with `target`) into qIK, or follow the real arm in hand-guide mode and during backend playback;
   copy the next pose on the answer's clear route into qCmd; move the simulated servos toward qCmd with a
   trapezoidal velocity profile, then draw.

   Modules only declare things when imported; everything that touches the page happens in their init*(). */
import * as THREE from 'three';
import {DEG,N,LIM,makeFK,fk} from './kinematics.js';
import {area,outsideArea,checkPose,checkPath} from './collision.js';
import {ikRes,requestSolve,solverNote,nearPose,initSolve} from './solve.js';
import {renderer,scene,camera,orbit,gizmo,root,rotGroups,ledMats,targetMat,targetObj,dropLine,floorRing,
  ghost,ghostGeo,ghostDots,realLine,realGeo,realDots,trail,pushTrail,pathLine,applyTheme,resize,updateJointFx} from './scene.js';
import {S,qIK,qCmd,servo,target,setTarget,targetFromPose,syncUI,setDemo,areaDir,haveRealNow} from './state.js';
import {initApi} from './api.js';
import {initSettings} from './settings.js';
import {initMotion,spd,acc} from './motion.js';
import {initChrome,updateLinkChip,jointUI,pct} from './chrome.js';
import {initLink,armDriving} from './link.js';
import {initAtom} from './atom.js';
import {initRecord,recTick} from './record.js';
import {initPlay,playStep,playStop,endPlay,playNote,pathWanted} from './play.js';
import {initWizard,wizardFrame,wizardStatus} from './wizard.js';
import {initLanding} from './landing.js';
import {initStudio,studioFrame} from './studio.js';
import {initTour} from './tour.js';
import {initViews} from './views.js';
import {initWorkspace,workspaceFrame} from './workspace.js';
import {$,cssVar,V} from './util.js';

initChrome();
initViews();
initSettings();
initMotion();
initLink();
initAtom();
initRecord();
initPlay();
initApi();
initSolve();
initWizard();
initLanding();
initStudio();
initTour();
initWorkspace();

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
S.ikRestart=true;   // the first solve restarts from seeded poses: the zero pose may be a poor start

const servoF=makeFK(),ghostF=makeFK(),realF=makeFK();
let ikErr={pos:0,ori:0},detour=false;
const deg2rad=a=>a.map(v=>v*DEG);
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
  const wiz=S.wizard?wizardFrame(dt):null;   // the wizard's own pose for the sim arm (radians), or null
  if(wiz)for(let i=0;i<N;i++){qIK[i]=qCmd[i]=servo[i].pos=wiz[i];servo[i].vel=0;}
  // the sim mirrors the real arm: hand-guide, a backend playback, or a pose the backend won't move from
  const following=!wiz&&haveReal&&(S.limp||!!S.remotePlay||!!S.armRange);
  if(following)for(let i=0;i<N;i++){qIK[i]=qCmd[i]=measured[i]*DEG;}
  // the backend solves: ask /ws/ik for the next answer, unless the arm is solving (link.js sends it the target)
  if(!wiz&&!following&&!armDriving()&&!S.play){
    const restart=S.ikRestart&&!S.homeLock;
    if(requestSolve({xyz:S.homeLock?null:target,angles:S.homeLock?qIK:null,down:orient,q:qIK,from:servo.map(s=>s.pos),
      rescue:!S.demo,restart})&&restart)S.ikRestart=false;
  }
  const r=ikRes;
  // take the solution only if it started from the pose we still have (nothing set qIK directly meanwhile);
  // the arm's own solve always counts
  if(!wiz&&!following&&!S.homeLock&&r&&r.fresh&&r.target&&(r.src==='arm'||nearPose(r.basis,qIK,1e-6))){
    for(let i=0;i<N;i++)qIK[i]=r.angles[i]*DEG;r.fresh=false;}
  if(following)targetFromPose();
  if(wiz||following||S.homeLock)ikErr={pos:0,ori:0};
  else if(r&&r.target)ikErr={pos:r.pos_err_mm/1000,ori:r.ori_err_deg*DEG};
  else ikErr=null;   // no answer yet
  // only collision-free poses reach the servos, along a clear route: straight there, or (not during a
  // playback, which must follow its recording) through raised poses around whatever is in the way. The
  // backend plans it from where the servos are with every answer, so qCmd is the next pose on the route.
  let blocked=null;detour=false;
  if(!S.stopped&&!following&&!wiz){
    const cur=servo.map(s=>s.pos);
    // the servos are still heading for the last qCmd: if the rest of that move isn't clear (checked finely,
    // from where they are now), hold them here
    if(checkPath(cur,qCmd,16,DEG))for(let i=0;i<N;i++)qCmd[i]=cur[i];
    blocked=checkPose(qIK);
    if(!blocked&&S.play){if(checkPath(cur,qIK))blocked=checkPath(cur,qIK);else for(let i=0;i<N;i++)qCmd[i]=qIK[i];}
    else if(!blocked&&r&&nearPose(deg2rad(r.angles),qIK)){   // the answer is for this pose (else wait for one)
      if(r.next){const next=deg2rad(r.next);detour=r.detour;for(let i=0;i<N;i++)qCmd[i]=next[i];}
      else blocked=r.blocked||'no clear route there';}
  }
  if(S.play&&blocked)endPlay(`Playback stopped: ${blocked}.`);
  else if(S.play&&S.play.done)endPlay('Playback finished.');

  // simulated servos: a trapezoidal profile per joint. Like the backend, the speed is split between the joints
  // by how far each has to go, so they all arrive together and the arm follows the straight joint-space path
  // the collision check looked at (roughly: every joint keeps the full acceleration, so stopping stays quick).
  // A timed playback sets its own per-joint speeds.
  const vmax=spd.value*DEG,amax=acc.value*DEG;let moving=false,far=0;
  for(let i=0;i<N;i++)far=Math.max(far,Math.abs(qCmd[i]-servo[i].pos));
  for(let i=0;i<N;i++){
    const s=servo[i],err=qCmd[i]-s.pos,vj=S.simSpeeds?S.simSpeeds[i]*DEG:vmax*(far>1e-9?Math.abs(err)/far:1);
    const vdes=Math.sign(err)*Math.min(vj,Math.sqrt(2*amax*Math.abs(err))*0.95);
    s.vel+=THREE.MathUtils.clamp(vdes-s.vel,-amax*dt,amax*dt);
    let step=s.vel*dt;
    if(Math.abs(step)>Math.abs(err)&&Math.sign(step)===Math.sign(err)){step=err;s.vel=0;}
    s.pos=wiz||following?s.pos+step:THREE.MathUtils.clamp(s.pos+step,LIM[i][0],LIM[i][1]);   // (a real pose is shown as it is)
    if(Math.abs(qCmd[i]-s.pos)<3e-4&&Math.abs(s.vel)<0.02){s.pos=qCmd[i];s.vel=0;}
    rotGroups[i].rotation.z=s.pos;
    ledMats[i].emissiveIntensity=0.12+1.6*Math.abs(s.vel)/vmax;
    if(Math.abs(s.vel)>0.01||Math.abs(qCmd[i]-s.pos)>3e-4)moving=true;
    const ui=jointUI[i];ui.pos.style.left=pct(i,s.pos);ui.tgt.style.left=pct(i,qCmd[i]);ui.val.textContent=(s.pos/DEG).toFixed(1)+'°';
  }
  fk(servo.map(s=>s.pos),servoF);
  updateJointFx(servoF,dt);
  if(trail.visible)pushTrail(servoF.tcp);
  // the wizard shows only the arm: no target, arrows or ghost in the way
  targetObj.visible=dropLine.visible=floorRing.visible=!wiz;gizmo.visible=gizmo.enabled=!wiz;
  ghost.visible=ghostDots.visible=!wiz&&$('#optGhost').checked;
  fk(qIK,ghostF);
  const ga=ghostGeo.attributes.position.array;ga.set([0,0,0.06],0);
  ghostF.pos.forEach((p,k)=>ga.set([p.x,p.y,p.z],(k+1)*3));ga.set([ghostF.tcp.x,ghostF.tcp.y,ghostF.tcp.z],(N+1)*3);
  ghostGeo.attributes.position.needsUpdate=true;
  const gc=blocked?cssVar('--bad'):'#3b82c4';ghost.material.color.set(gc);ghostDots.material.color.set(gc);

  const reachable=!!ikErr&&ikErr.pos<0.003&&(!orient||ikErr.ori<3*DEG);
  targetMat.color.set(reachable?cssVar('--amber'):cssVar('--bad'));targetMat.emissive.copy(targetMat.color);
  targetObj.scale.setScalar(1+0.12*Math.sin(performance.now()*0.004));
  const lp=dropLine.geometry.attributes.position.array;lp.set([target.x,target.y,target.z,target.x,target.y,0.0005]);
  dropLine.geometry.attributes.position.needsUpdate=true;dropLine.computeLineDistances();
  floorRing.position.set(target.x,target.y,0.0008);

  const st=$('#status');
  if(wiz){st.className='status';$('#statusText').textContent=wizardStatus();}
  else if(S.stopped){st.className='status bad';$('#statusText').textContent=S.armFault?'Stopped: '+S.armFault:'Stopped. Press Resume to move again';}
  else if(S.armRange){st.className='status bad';$('#statusText').textContent='Won\'t move: '+S.armRange;}
  else if(area.enabled&&outsideArea(target)&&!S.limp){st.className='status bad';
    $('#statusText').textContent=/close to the base/.test(outsideArea(target))?'The target is too close to the base (Setup tab, Work area)':'The target is outside the work area (Setup tab)';}
  else if(blocked||S.remoteBlocked){st.className='status bad';const why=blocked||S.remoteBlocked;
    $('#statusText').textContent=`Blocked: ${why}${blocked?'':' (checked by the arm)'}`;}
  else if(!ikErr){st.className='status bad';$('#statusText').textContent=solverNote()||'Solving…';}
  else if(!reachable){st.className='status bad';$('#statusText').textContent=orient&&ikErr.pos<0.003?'Reachable, but not facing straight down':'Out of reach, holding the closest pose';}
  else if(moving){st.className='status';$('#statusText').textContent=detour?'Servos moving, going up and around':'Servos moving';}
  else{st.className='status ok';$('#statusText').textContent='At target';}
  $('#errText').textContent=(servoF.tcp.distanceTo(target)*1000).toFixed(1)+' mm';
  updateLinkChip();
  recTick(performance.now());
  pathLine.visible=pathWanted&&!$('#tab-play').hidden;
  const showReal=haveReal&&($('#optReal').checked||!!wiz);
  realLine.visible=realDots.visible=showReal;
  if(showReal){fk(measured.map(v=>v*DEG),realF);const ra=realGeo.attributes.position.array;ra.set([0,0,0.06],0);
    realF.pos.forEach((p,k)=>ra.set([p.x,p.y,p.z],(k+1)*3));ra.set([realF.tcp.x,realF.tcp.y,realF.tcp.z],(N+1)*3);realGeo.attributes.position.needsUpdate=true;}
  for(let i=0;i<N;i++){const r=jointUI[i].real;if(haveReal){r.style.display='block';r.style.left=pct(i,measured[i]*DEG);}else r.style.display='none';}
  if(S.view==='studio')studioFrame(dt);   // the Studio's sandbox or the Workspace instead of the main view (only one is drawn)
  else if(S.view==='workspace')workspaceFrame(dt);
  else{orbit.update();renderer.render(scene,camera);}
}
frame();
