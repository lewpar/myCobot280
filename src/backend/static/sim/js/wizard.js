/* Calibration wizard: sets up the real arm step by step while the 3D scene shows what to do. The sim arm
   demonstrates each step (the zero pose to copy, which way to turn a joint) and the real arm is drawn over it.

   Steps: check the link → torque off (the user holds the arm) → pose it at the zero pose → re-centre the
   servos whose 0/4095 point is within reach (`recenter`) → set the zero (`set_zero`) → turn each joint a
   little to check its direction (reversing it with `set_dir` if it counts the wrong way) → torque back on →
   done. While it's open nothing drives the arm (link.armDriving is off) and main.js takes the sim arm's pose
   from wizardFrame. Everything it asks the arm goes over /ws/arm like the rest of the page. */
import * as THREE from 'three';
import {DEG,N} from './kinematics.js';
import {camera,orbit,setJointFx,setViewShift} from './scene.js';
import {S} from './state.js';
import {send,onArm,setLimp,setStopped,adoptMeasured} from './link.js';
import {showConn} from './chrome.js';
import {$} from './util.js';

const TPD=4096/360;
const FAR_DEG=30;       // a servo reading this far from its centre in the zero pose is worth re-centring
const TURN_DEG=12;      // how far a joint must be turned to tell its direction
const BACK_DEG=4;       // ...and how close to where it started counts as put back
const NAMES=['J1 · base','J2 · shoulder','J3 · elbow','J4 · wrist bend','J5 · wrist turn','J6 · flange'];
const HINTS=[
  'Turn the whole arm to face the front: +X, away from the Pi\'s ports, where the model\'s flange points.',
  'Stand the lower arm straight up.',
  'Straighten the elbow so the upper arm carries on straight up.',
  'Straighten the wrist in line with the rest.',
  'Turn the wrist so the flange faces the front, like the model.',
  'Turn the flange to match the model: its small notch at the bottom.',
];
// camera for each joint's direction check [direction, height to look at, distance]: roughly along its axis,
// from high enough to see the whole arm (three.js frame: Y up, the base frame's -Y, the arm's right, is +Z)
const JOINT_VIEWS=[[[0.7,1.1,0.8],0.2,1.05],[[0.35,0.35,1],0.2,0.95],[[0.35,0.35,1],0.24,0.9],[[0.35,0.4,1],0.3,0.85],
  [[0.5,1.1,0.6],0.3,0.9],[[1,0.45,0.45],0.34,0.85]];

const ok=v=>v?'<span class="wz-i ok" aria-label="done">✓</span>':'<span class="wz-i" aria-hidden="true">○</span>';
const warnI='<span class="wz-i warn" aria-label="needs attention">!</span>';
const complete=a=>!!(a&&a.length===N&&a.every(v=>typeof v==='number'));
const fmt=v=>(v>0?'+':'')+v.toFixed(0)+'°';

let step=0,t0=0,stepT=0,from=null,cam=null,unsub=null,lastTick=0;
// what happened, for the summary (and so going back doesn't lose it)
const W={held:false,recentered:null,recenterBusy:false,recenterErr:'',zeroSet:false,zeroBusy:false,zeroKey:'',
  dirs:[],dirJ:0,dirPhase:'turn',dirBase:0,dirWait:false,dirAt:0,torqueSent:false};

/* ---------- steps ---------- */
const STEPS=[
  {id:'start',title:'Set up the arm',
    body:()=>`<p class="wz-lead">This walks you through calibrating the real arm, about three minutes. The model on the
      left shows each step.</p>
      <ul class="wz-list" id="wzReq"></ul>
      <p class="wz-note">You'll need both hands for a moment: one holds the arm while torque is off.</p>`,
    tick(){const a=S.measured,st=S.armState;
      const rows=[[!!S.ws&&!!st,'Connected to the arm'],[complete(a),'All six servos answering'],[!(st&&st.playback),'No playback running']];
      $('#wzReq').innerHTML=rows.map(([v,t])=>`<li>${ok(v)}${t}</li>`).join('');
      return rows.every(r=>r[0]);},
    pose:'measured'},
  {id:'hold',title:'Hold the arm',
    body:()=>`<p class="wz-lead">Torque goes off so you can move the arm by hand. <b>Support it first</b>: the upper arm
      and wrist drop as soon as the servos let go.</p>
      <label class="chk wz-chk"><input type="checkbox" id="wzHeld"> I'm holding the arm</label>
      <button id="wzTorqueOff" class="block primary" disabled>Turn torque off</button>
      <p class="wz-state" id="wzHoldState"></p>`,
    enter(){$('#wzHeld').checked=W.held;
      $('#wzHeld').addEventListener('change',e=>{W.held=e.target.checked;});
      $('#wzTorqueOff').addEventListener('click',()=>{setLimp(true);});},
    tick(){const off=S.armTorque===false;$('#wzTorqueOff').disabled=!W.held||off;
      $('#wzHoldState').innerHTML=off?`${ok(true)}Torque is off. The model follows your arm now.`:'';
      return off;},
    pose:'measured'},
  {id:'pose',title:'Pose it straight up',
    body:()=>`<p class="wz-lead">Move the arm into the <b>zero pose</b> the model is showing: straight up, the flange
      facing the front. If your arm has small alignment marks at the joints, line them up.</p>
      <ol class="wz-joints" id="wzHints">${HINTS.map((h,j)=>`<li data-j="${j}"><b>${NAMES[j]}</b><span>${h}</span></li>`).join('')}</ol>
      <p class="wz-note">Point at a joint to see it on the model. A joint folded right back: turn it the short way round.</p>`,
    enter(){hintHover('#wzHints');view([1,0.45,0.8],0.22,0.95);},
    tick:()=>true,pose:'zero'},
  {id:'centre',title:'Centre the servos',
    body:()=>`<p class="wz-lead">Each servo counts 0 to 4095 over one turn and wraps round at the ends. A servo that reads
      far from its middle here has that wrap point within the joint's reach, where it would read half a turn out and
      turn the wrong way. Re-centring makes it read its middle here.</p>
      <div class="wz-table" id="wzCentre"></div>
      <button id="wzRecenter" class="block primary">Re-centre the marked servos</button>
      <p class="wz-state" id="wzCentreState"></p>
      <p class="wz-note">Stored in each servo. The calibration shifts with it, so nothing else changes meaning.
      <code>tools/recenter_servos.py --undo</code> puts the last re-centring back.</p>`,
    enter(){renderCentre(true);$('#wzRecenter').addEventListener('click',recenter);},
    tick(){renderCentre(false);
      const busy=W.recenterBusy,need=picked().length;
      $('#wzRecenter').disabled=busy||!need||S.armTorque!==false;
      $('#wzRecenter').textContent=busy?'Re-centring…':need?`Re-centre ${need} servo${need>1?'s':''}`:'Nothing to re-centre';
      const res=W.recentered;
      $('#wzCentreState').innerHTML=W.recenterErr?`${warnI}${W.recenterErr}`:res?res.map(r=>`<span class="wz-tag ${r.ok?'good':'bad'}">J${r.joint+1} ${r.ok?'centred':'failed'}</span>`).join(' '):
        (S.armTorque!==false?`${warnI}Torque is on: turn it off (previous step) first.`:'');
      return !busy;},
    next:()=>picked().length&&!W.recentered?'Skip':'Next',pose:'zero'},
  {id:'zero',title:'Save the zero',
    body:()=>`<p class="wz-lead">With the arm still in the zero pose, save it: every joint reads 0° here from now on.</p>
      <button id="wzZero" class="block primary">Set zero here</button>
      <div class="wz-table" id="wzAngles"></div>`,
    enter(){$('#wzZero').addEventListener('click',()=>{W.zeroBusy=true;W.zeroKey=JSON.stringify(S.armConfig&&S.armConfig.zero);send({type:'set_zero'});});},
    tick(){const c=S.armConfig;
      if(W.zeroBusy&&c&&JSON.stringify(c.zero)!==W.zeroKey){W.zeroBusy=false;W.zeroSet=true;}
      $('#wzZero').disabled=W.zeroBusy;$('#wzZero').textContent=W.zeroBusy?'Saving…':W.zeroSet?'Set it again':'Set zero here';
      const a=S.measured||[];
      $('#wzAngles').innerHTML=a.map((v,j)=>`<div class="wz-row"><b>${NAMES[j]}</b><span class="wz-num">${v===null?'–':fmt(v)}</span>${ok(W.zeroSet&&v!==null&&Math.abs(v)<3)}</div>`).join('');
      return W.zeroSet;},
    pose:'zero'},
  {id:'dirs',title:'Check each joint\'s direction',
    body:()=>`<p class="wz-lead" id="wzDirLead"></p>
      <div class="wz-gauge" aria-hidden="true"><i class="wz-mid"></i><i class="wz-mark" id="wzMark"></i></div>
      <p class="wz-state" id="wzDirState"></p>
      <div class="wz-dots" id="wzDirDots"></div>
      <button id="wzDirSkip" class="ghost">Skip this joint</button>`,
    enter(){if(W.dirs.length!==N)W.dirs=new Array(N).fill(null);
      W.dirJ=Math.max(0,W.dirs.findIndex(d=>d===null));if(W.dirs.every(d=>d!==null))W.dirJ=N-1;startJoint();
      $('#wzDirSkip').addEventListener('click',()=>{W.dirs[W.dirJ]=W.dirs[W.dirJ]||'skipped';nextJoint();});},
    leave(){setJointFx(null);},
    tick:dirTick,pose:'wiggle'},
  {id:'torque',title:'Torque back on',
    body:()=>`<p class="wz-lead">Put the arm in a safe pose and keep holding it. When torque comes back it holds exactly
      where it is.</p>
      <button id="wzTorqueOn" class="block primary">Turn torque on</button>
      <button id="wzResume" class="block" hidden>Resume the arm</button>
      <p class="wz-state" id="wzTorqueState"></p>`,
    enter(){$('#wzTorqueOn').addEventListener('click',()=>{W.torqueSent=true;setLimp(false);});
      $('#wzResume').addEventListener('click',()=>setStopped(false));},
    tick(){const on=S.armTorque===true;
      $('#wzTorqueOn').disabled=on;$('#wzResume').hidden=!(on&&S.stopped);$('#wzResume').disabled=!!S.armRange;
      $('#wzTorqueState').innerHTML=S.armRange?`${warnI}${S.armRange}`:on&&S.stopped?`${ok(true)}Torque is on. The arm was stopped earlier: resume it to use it.`:
        on?`${ok(true)}Torque is on and the arm is holding.`:'';
      return on&&!S.stopped;},
    pose:'measured'},
  {id:'done',title:'All set',
    body:()=>{const rc=(W.recentered||[]).filter(r=>r.ok).map(r=>'J'+(r.joint+1)),rev=W.dirs.map((d,j)=>d==='reversed'?'J'+(j+1):null).filter(Boolean);
      return `<p class="wz-lead">The arm is calibrated.</p>
      <ul class="wz-list">
        <li>${ok(true)}Zero saved in the straight-up pose</li>
        <li>${ok(true)}${rc.length?'Re-centred '+rc.join(', '):'Servo centres left as they were'}</li>
        <li>${ok(true)}${rev.length?'Reversed '+rev.join(', '):'Every joint already counted the right way'}</li>
      </ul>
      <p class="wz-note">Next: pick what's on the flange in Setup → Attachment, then try a slow first move from the
      Move tab (Max speed about 30°/s, a target a few cm away, a hand near Stop).</p>`;},
    tick:()=>true,next:()=>'Finish',pose:'measured'},
];

/* ---------- step helpers ---------- */
function picked(){return [...document.querySelectorAll('#wzCentre input:checked')].map(b=>+b.dataset.j);}
function renderCentre(first){
  const ticks=S.armState&&S.armState.ticks,box=$('#wzCentre');if(!box)return;
  if(first||!box.children.length){
    box.innerHTML=NAMES.map((n,j)=>`<label class="wz-row" data-j="${j}"><b>${n}</b><span class="wz-num" id="wzC${j}">–</span>
      <input type="checkbox" data-j="${j}" aria-label="Re-centre ${n}"></label>`).join('');
    NAMES.forEach((_,j)=>{const t=ticks&&ticks[j];box.querySelector(`input[data-j="${j}"]`).checked=typeof t==='number'&&Math.abs(t-2048)/TPD>FAR_DEG;});
    hintHover('#wzCentre');
  }
  NAMES.forEach((_,j)=>{const t=ticks&&ticks[j],el=$('#wzC'+j);
    if(typeof t!=='number'){el.textContent='–';return;}
    const off=(t-2048)/TPD;el.textContent=`${fmt(off)} from centre`;el.classList.toggle('far',Math.abs(off)>FAR_DEG);});
}
function recenter(){
  const joints=picked();if(!joints.length)return;
  W.recenterBusy=true;W.recenterErr='';W.recentered=null;send({type:'recenter',joints});
}
function hintHover(sel){
  document.querySelectorAll(`${sel} [data-j]`).forEach(el=>{
    const j=+el.dataset.j;
    el.addEventListener('pointerenter',()=>setJointFx({joint:j,arrow:false}));
    el.addEventListener('pointerleave',()=>setJointFx(null));
  });
}
function startJoint(){
  W.dirPhase='turn';W.dirWait=false;W.dirBase=S.measured&&typeof S.measured[W.dirJ]==='number'?S.measured[W.dirJ]:0;
  const j=W.dirJ,[dir,at,dist]=JOINT_VIEWS[j];view(dir,at,dist);setJointFx({joint:j,arrow:true});
  $('#wzDirLead').innerHTML=`<b>${NAMES[j]}</b>: turn it about 20° the way the blue arrow points, like the model, then back.`;
}
function nextJoint(){
  const n=W.dirs.findIndex(d=>d===null);
  if(n<0){setJointFx(null);$('#wzDirLead').innerHTML='Every joint checked.';return;}
  W.dirJ=n;startJoint();
}
function dirTick(){
  const j=W.dirJ,a=S.measured,cfg=S.armConfig,state=$('#wzDirState');
  $('#wzDirDots').innerHTML=W.dirs.map((d,k)=>`<span class="wz-dot ${d?(d==='skipped'?'skip':'good'):k===j?'now':''}" title="J${k+1}">J${k+1}</span>`).join('');
  if(W.dirs.every(d=>d!==null)){state.innerHTML=`${ok(true)}Done. Every joint counts the way the model does.`;$('#wzDirSkip').hidden=true;return true;}
  $('#wzDirSkip').hidden=false;
  if(!complete(a)||!cfg){state.textContent='Waiting for the arm…';return false;}
  const d=a[j]-W.dirBase;
  $('#wzMark').style.left=`${50+Math.max(-45,Math.min(45,d*1.5))}%`;
  if(W.dirWait){state.textContent='Reversing…';if(cfg.dir[j]===W.dirWait){W.dirWait=false;W.dirBase=-W.dirBase;}return false;}
  if(W.dirPhase==='turn'){
    state.innerHTML=Math.abs(d)<2?'Turn it the way the arrow points.':`Turned ${fmt(d)}…`;
    if(Math.abs(d)>=TURN_DEG){
      if(d>0)W.dirs[j]='ok';
      else{W.dirs[j]='reversed';W.dirWait=-cfg.dir[j];send({type:'set_dir',joint:j,dir:W.dirWait});}
      W.dirPhase='back';
    }
  }else if(W.dirPhase==='back'){
    state.innerHTML=`${ok(true)}${W.dirs[j]==='reversed'?'It counted the other way, so it\'s reversed now.':'Right way round.'} Now turn it back.`;
    if(Math.abs(d)<BACK_DEG){W.dirPhase='settle';W.dirAt=performance.now();}
  }else if(performance.now()-W.dirAt>600)nextJoint();
  return false;
}

/* ---------- camera: a short glide to each step's view ---------- */
function view(dir,atY,dist){
  const to=new THREE.Vector3(0,atY,0),d=new THREE.Vector3(...dir).normalize();
  cam={t:0,p0:camera.position.clone(),t0:orbit.target.clone(),p1:to.clone().addScaledVector(d,dist),t1:to};
}
function camStep(dt){
  if(!cam)return;cam.t=Math.min(1,cam.t+dt/0.9);const e=cam.t<0.5?2*cam.t*cam.t:1-(-2*cam.t+2)**2/2;
  camera.position.lerpVectors(cam.p0,cam.p1,e);orbit.target.lerpVectors(cam.t0,cam.t1,e);orbit.update();if(cam.t>=1)cam=null;
}

/* ---------- rendering ---------- */
function render(){
  const s=STEPS[step];
  $('#wizKicker').textContent=`Calibration · step ${step+1} of ${STEPS.length}`;$('#wizTitle').textContent=s.title;
  $('#wizProgress').innerHTML=STEPS.map((x,k)=>`<li class="${k<step?'done':k===step?'now':''}" title="${x.title}"></li>`).join('');
  const body=$('#wizBody');body.innerHTML=s.body();body.classList.remove('wz-in');void body.offsetWidth;body.classList.add('wz-in');
  $('#wizBack').disabled=step===0;
  stepT=0;from=S.measured&&complete(S.measured)?S.measured.map(v=>v*DEG):null;
  if(s.enter)s.enter();
  lastTick=0;update();
}
function update(){
  const s=STEPS[step],can=s.tick();
  $('#wizNext').textContent=s.next?s.next():'Next';
  $('#wizNext').disabled=!can&&$('#wizNext').textContent!=='Skip';
}
function go(k){const s=STEPS[step];if(s.leave)s.leave();setJointFx(null);step=Math.max(0,Math.min(STEPS.length-1,k));render();}

export function openWizard(){
  if(!S.ws){showConn();$('#wsNote').textContent='Connect to the arm first, then start the calibration wizard.';return;}
  S.wizard=true;step=0;t0=performance.now();
  Object.assign(W,{held:false,recentered:null,recenterBusy:false,recenterErr:'',zeroSet:false,zeroBusy:false,dirs:[],torqueSent:false});
  document.body.classList.add('wiz-open');$('#wiz').hidden=false;setViewShift(180);render();
}
export function closeWizard(){
  if(!S.wizard)return;
  const s=STEPS[step];if(s.leave)s.leave();
  S.wizard=false;setJointFx(null);$('#wiz').hidden=true;document.body.classList.remove('wiz-open');cam=null;setViewShift(0);
  if(complete(S.measured))adoptMeasured();   // start from where the arm really is: nothing moves on closing
}
export const wizardStatus=()=>`Calibration: ${STEPS[step].title.toLowerCase()}`;

/* The sim arm's pose for this frame (radians): the real arm, the zero pose, or the zero pose with the joint
   being checked turning back and forth (positive first). */
export function wizardFrame(dt){
  stepT+=dt;camStep(dt);
  const now=performance.now();if(now-lastTick>100){lastTick=now;update();}
  const s=STEPS[step],a=S.measured;
  if(s.pose==='measured')return complete(a)?a.map(v=>v*DEG):(from||new Array(N).fill(0));
  const e=Math.min(1,stepT/1.2),ease=e*e*(3-2*e),q=(from||new Array(N).fill(0)).map(v=>v*(s.id==='pose'?1-ease:0));
  if(s.pose==='wiggle'&&W.dirPhase==='turn'&&!W.dirs.every(d=>d!==null))q[W.dirJ]=20*DEG*Math.sin(Math.min(stepT,1e6)*2.2);
  return q;
}

export function initWizard(){
  $('#btnWizard').addEventListener('click',openWizard);$('#btnCalib').addEventListener('click',openWizard);
  $('#wizClose').addEventListener('click',()=>{
    if(step>0&&step<STEPS.length-1&&!confirm('Stop calibrating? What\'s been saved so far stays saved.'))return;
    closeWizard();});
  $('#wizBack').addEventListener('click',()=>go(step-1));
  $('#wizNext').addEventListener('click',()=>{if(step===STEPS.length-1)closeWizard();else go(step+1);});
  unsub=onArm(m=>{
    if(m.type==='recentered'&&S.wizard){W.recenterBusy=false;W.recentered=m.results;
      const bad=m.results.filter(r=>!r.ok);W.recenterErr=bad.map(r=>r.message).join(' ');}
    else if(m.type==='error'&&S.wizard&&['recenter','set_zero','set_dir','torque'].includes(m.ref)){
      if(m.ref==='recenter'){W.recenterBusy=false;W.recenterErr=m.message;}
      if(m.ref==='set_zero'){W.zeroBusy=false;$('#wzAngles')&&($('#wzAngles').insertAdjacentHTML('beforebegin',`<p class="wz-state">${warnI}${m.message}</p>`));}
      if(m.ref==='set_dir')W.dirWait=false;}
  });
  // the Calibrate pill in the top bar: shown while the arm needs it (not calibrated, or a joint it won't move)
  setInterval(()=>{const c=S.armConfig,need=!!S.ws&&!S.wizard&&(!!S.armRange||(c&&c.calibrated===false));$('#btnCalib').hidden=!need;},500);
}
