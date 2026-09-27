/* Move tab: target sliders and presets, the flange-down option, speed, jog and saved poses. The 3D view's
   ghost and trail toggles (View menu) are wired here too. */
import {DEG,JOINTS,N,clampJ} from './kinematics.js';
import {area,checkPose} from './collision.js';
import {trail,clearTrail,ghost,ghostDots} from './scene.js';
import {S,qIK,target,setTarget,targetFromPose,setDemo,areaDir,recPose,tcpText} from './state.js';
import {setStopped,setLimp} from './link.js';
import {endPlay} from './play.js';
import {api} from './api.js';
import {toast} from './toast.js';
import {$,V,item,r2} from './util.js';

export const spd=$('#spd'),acc=$('#acc');   // max speed (deg/s) and acceleration (deg/s²), for the sim and the arm
function syncSliders(){$('#spdv').textContent=spd.value+'°/s';$('#accv').textContent=acc.value+'°/s²';$('#speedMeta').textContent=spd.value+'°/s';}

/* ---- going to a pose (saved poses, waypoints) ---- */
export function goPose(a,noteEl){ // a: degrees
  const say=t=>{(noteEl||$('#poseNote')).textContent=t;};
  if(S.stopped){say('The arm is stopped. Press Resume first.');return;}
  if(S.remotePlay||S.remotePending){say('A recording is playing. Stop it first.');return;}
  if(S.play)endPlay('Playback stopped.');
  if(S.limp)setLimp(false);
  setDemo(false);for(let j=0;j<N;j++)qIK[j]=clampJ(j,a[j]);targetFromPose();S.homeLock=true;
  const why=checkPose(qIK);say(why?`Can't go there: ${why}.`:'');
}
let poseItems=[];
const poseNote=t=>{$('#poseNote').textContent=t;};
export async function poseRefresh(){try{poseItems=await api('GET','/poses');}catch(e){poseNote(e.message);return;}poseRender();}
function poseRender(){
  const box=$('#poseList');box.textContent='';
  poseItems.forEach(p=>box.appendChild(item(p.name,tcpText(p.angles),null,()=>goPose(p.angles),async()=>{
    if(!confirm(`Delete the pose "${p.name}"?`))return;
    try{await api('DELETE','/poses/'+p.id);await poseRefresh();}catch(e){poseNote(e.message);}})));
  if(!poseItems.length){const e=document.createElement('p');e.className='note';e.style.margin='0';e.textContent='No saved poses yet.';box.appendChild(e);}
}

/* ---- jog: nudge the target (tool X/Y/Z in mm) or one joint (degrees); hold a button to repeat ---- */
let jogMode='xyz',jogStep=5;
const JOG_STEPS={xyz:[1,5,10,25],joint:[1,5,10,20]};
function jog(axis,dir){
  if(S.stopped||S.limp||S.remotePlay||S.remotePending)return;
  setDemo(false);if(S.play)endPlay('Playback stopped: jogged.');
  if(jogMode==='xyz'){S.homeLock=false;const v=target.clone();v.setComponent(axis,v.getComponent(axis)+dir*jogStep/1000);setTarget(v);}
  else{qIK[axis]=clampJ(axis,qIK[axis]/DEG+dir*jogStep);targetFromPose();S.homeLock=true;}
}
function holdRepeat(b,fn){
  let t1=null,t2=null;const stop=()=>{clearTimeout(t1);clearInterval(t2);};
  b.addEventListener('pointerdown',e=>{e.preventDefault();stop();fn();t1=setTimeout(()=>{t2=setInterval(fn,120);},400);});
  ['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,stop));
  b.addEventListener('click',e=>{if(e.detail===0)fn();});   // keyboard
}
function jogBuild(){
  [['#jogXYZ',['X','Y','Z']],['#jogJ',JOINTS.map(j=>j.name)]].forEach(([id,names])=>{
    const box=$(id);names.forEach((n,axis)=>{const l=document.createElement('b');l.textContent=n;box.appendChild(l);
      [-1,1].forEach(d=>{const b=document.createElement('button');b.type='button';b.textContent=d<0?'−':'+';b.setAttribute('aria-label',`${n} ${d<0?'minus':'plus'}`);holdRepeat(b,()=>jog(axis,d));box.appendChild(b);});});
  });
}
function jogRender(){
  $('#jogModeXYZ').setAttribute('aria-pressed',jogMode==='xyz');$('#jogModeJ').setAttribute('aria-pressed',jogMode==='joint');
  $('#jogXYZ').hidden=jogMode!=='xyz';$('#jogJ').hidden=jogMode!=='joint';
  const box=$('#jogSteps');box.textContent='';
  if(!JOG_STEPS[jogMode].includes(jogStep))jogStep=5;
  JOG_STEPS[jogMode].forEach(v=>{const b=document.createElement('button');b.type='button';b.textContent=v+(jogMode==='xyz'?' mm':'°');
    b.setAttribute('aria-pressed',v===jogStep);b.addEventListener('click',()=>{jogStep=v;jogRender();});box.appendChild(b);});
}

export function initMotion(){
  const tx=$('#tx'),ty=$('#ty'),tz=$('#tz');
  [tx,ty,tz].forEach(el=>el.addEventListener('input',()=>{setDemo(false);S.homeLock=false;setTarget(V(tx.value/1000,ty.value/1000,tz.value/1000));}));
  [spd,acc].forEach(el=>el.addEventListener('input',syncSliders));
  syncSliders();
  $('#btnDemo').addEventListener('click',()=>{S.homeLock=false;setDemo(!S.demo);if(S.demo){S.demoT=0;$('#optDown').checked=true;
    S.ikRestart=true;}});   // a good start for the whole figure (it solves without restarts while it runs)
  $('#btnHome').addEventListener('click',()=>{setDemo(false);setStopped(false);for(let i=0;i<N;i++)qIK[i]=0;targetFromPose();$('#optDown').checked=false;S.homeLock=true;});
  $('#btnRandom').addEventListener('click',()=>{setDemo(false);S.homeLock=false;
    const span=area.enabled?Math.min(area.span,360)*0.8:360,a=areaDir()+(Math.random()-0.5)*span*DEG;   // inside the work area
    const r0=Math.max(0.12,area.enabled?(area.base_mm||0)/1000+0.02:0),r=r0+Math.random()*Math.max(0.02,0.25-r0),z=0.03+Math.random()*0.2;setTarget(V(Math.cos(a)*r,Math.sin(a)*r,z));});
  $('#optDown').addEventListener('change',()=>{S.homeLock=false;});
  $('#optTrail').addEventListener('change',e=>{trail.visible=e.target.checked;if(!e.target.checked)clearTrail();});
  $('#optGhost').addEventListener('change',e=>{ghost.visible=ghostDots.visible=e.target.checked;});
  $('#poseSave').addEventListener('click',async()=>{
    const name=$('#poseName').value.trim();if(!name){poseNote('Type a name for the pose first.');$('#poseName').focus();return;}
    try{await api('POST','/poses',{name,angles:recPose().map(r2)});$('#poseName').value='';poseNote(`Saved "${name}".`);toast(`Saved the pose "${name}"`,'good');await poseRefresh();}
    catch(e){poseNote(e.message);}
  });
  $('#poseName').addEventListener('keydown',e=>{if(e.key==='Enter')$('#poseSave').click();});
  $('#poseRefresh').addEventListener('click',poseRefresh);
  $('#tabbtn-motion').addEventListener('click',()=>{if(!poseItems.length&&$('#wsPw').value)poseRefresh();});
  $('#jogModeXYZ').addEventListener('click',()=>{jogMode='xyz';jogRender();});
  $('#jogModeJ').addEventListener('click',()=>{jogMode='joint';jogRender();});
  jogBuild();jogRender();
}
