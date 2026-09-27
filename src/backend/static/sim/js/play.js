/* Play tab: the recordings, sequences and motions library (REST /api/*), the recording editor, the sequence
   editor and playback. Motions are the Motion Studio's programs: the backend compiles one into frames
   (/api/programs/compile) for the path, the pre-check and playing it in the simulation, and plays it on the
   arm itself (/api/playback {program}). Connected, playback runs on the backend (/api/playback) and the page only follows the
   measured pose. Offline, Player (a copy of player.py) runs here on the simulated servos and feeds qIK, so
   the usual collision check -> qCmd path applies. */
import {DEG,N,makeFK,fk,clampJ} from './kinematics.js';
import {checkSteps} from './collision.js';
import {Player} from './player.js';
import {PATH_MAX,pathGeo} from './scene.js';
import {S,qIK,qCmd,servo,targetFromPose,setDemo} from './state.js';
import {linkLive,setLimp} from './link.js';
import {spd,acc} from './motion.js';
import {isRecording,recUI} from './record.js';
import {ledApply} from './atom.js';
import {api} from './api.js';
import {toast} from './toast.js';
import {blockLabel} from './studio.js';
import {$,item,fmtDur} from './util.js';

let recItems=[],seqItems=[],progItems=[],recCache={},sel=null,selSteps=null,selCheck=null;
let progInfo=null;   // the selected motion's name, where each block starts and what it's called: the now-playing bar
const itemsOf=kind=>kind==='rec'?recItems:kind==='seq'?seqItems:progItems;
export const playNote=t=>{$('#playNote').textContent=t;};
export const busyPlaying=()=>!!(S.play||S.remotePlay||S.remotePending);

/* the selected recording's tool path, drawn in the 3D view while the Play tab is open */
const pathF=makeFK();
export let pathWanted=false;
function pathShow(list,range){
  pathWanted=!!list;if(!list)return;
  const fr=list.flatMap(f=>range?f.filter(r=>r[0]>=range[0]-1e-6&&r[0]<=range[1]+1e-6):f);
  const stride=Math.max(1,Math.ceil(fr.length/(PATH_MAX-1))),a=pathGeo.attributes.position.array;let n=0;
  fr.forEach((f,k)=>{if(k%stride&&k!==fr.length-1)return;fk(f.slice(1).map(v=>v*DEG),pathF);a.set([pathF.tcp.x,pathF.tcp.y,pathF.tcp.z],n*3);n++;});
  pathGeo.setDrawRange(0,n);pathGeo.attributes.position.needsUpdate=true;
}

/* ---- library: recordings and sequences ---- */
async function loadRec(id){if(!recCache[id])recCache[id]=await api('GET','/recordings/'+id);return recCache[id];}
export async function libRefresh(){
  try{[recItems,seqItems,progItems]=await Promise.all([api('GET','/recordings'),api('GET','/sequences'),api('GET','/programs')]);}
  catch(e){playNote(e.message);return false;}
  if(sel&&!itemsOf(sel.kind).some(x=>x.id===sel.id))selectItem(null);
  libRender();return true;
}
function libRender(){
  const date=c=>new Date(c*1000).toLocaleDateString(undefined,{day:'numeric',month:'short'});
  const rl=$('#recList');rl.textContent='';
  recItems.forEach(r=>{
    const b=item(r.name,`${r.return_zero?'→ 0 · ':''}${r.events?'LED · ':''}${fmtDur(r.duration)} · ${date(r.created)}`,
      !!sel&&sel.kind==='rec'&&sel.id===r.id,()=>{if(!busyPlaying())selectItem({kind:'rec',id:r.id});});
    b.addEventListener('dblclick',()=>{if(!busyPlaying()&&selSteps&&sel.id===r.id)playStart();});rl.appendChild(b);});
  $('#recEmpty').textContent=recItems.length?'':'No recordings saved yet. Make one in the Record tab.';
  const sl=$('#seqList');sl.textContent='';
  seqItems.forEach(q=>sl.appendChild(item(q.name,`${q.steps.length} step${q.steps.length>1?'s':''}`,
    !!sel&&sel.kind==='seq'&&sel.id===q.id,()=>{if(!busyPlaying())selectItem({kind:'seq',id:q.id});})));
  $('#seqEmpty').textContent=seqItems.length?'':'A sequence plays several recordings in a row, with a pause after each.';
  const pl=$('#progList');pl.textContent='';
  progItems.forEach(q=>{const b=item(q.name,`${q.blocks} block${q.blocks===1?'':'s'} · ${date(q.updated||q.created)}`,
      !!sel&&sel.kind==='prog'&&sel.id===q.id,()=>{if(!busyPlaying())selectItem({kind:'prog',id:q.id});});
    b.addEventListener('dblclick',()=>{if(!busyPlaying()&&selSteps&&sel.id===q.id)playStart();});pl.appendChild(b);});
  $('#progEmpty').textContent=progItems.length?'':'Motions you build in the Studio (top bar) appear here to play.';
  const pick=$('#seqPick'),keep=pick.value;pick.textContent='';
  recItems.forEach(r=>{const o=document.createElement('option');o.value=r.id;o.textContent=r.name;pick.appendChild(o);});
  if(recItems.some(r=>r.id===keep))pick.value=keep;
  if(seqDraft)seqRenderSteps();
  playUI();
}
export async function selectItem(s){
  sel=s;selSteps=null;selCheck=null;pathShow(null);$('#recEdit').hidden=true;$('#progEditBtn').hidden=true;libRender();
  if(!s)return;
  try{
    if(s.kind==='rec'){
      const r=await loadRec(s.id);if(sel!==s)return;
      selSteps=[{...r,pause:0}];editShow(r);
    }else if(s.kind==='prog'){   // the backend compiles it: the same frames it plays on the arm
      const p=await api('GET','/programs/'+s.id),c=await api('POST','/programs/compile',{blocks:p.blocks});
      if(sel!==s)return;
      const labels={},walk=bs=>bs.forEach(b=>{labels[b.id]=blockLabel(b);if(b.blocks)walk(b.blocks);});walk(p.blocks);
      progInfo={name:p.name,marks:c.marks,labels,count:Object.keys(labels).length};
      if(c.frames.length>1){selSteps=[{name:p.name,frames:c.frames,events:c.events,return_zero:false,pause:0}];pathShow([c.frames]);}
      selCheck=c.problems.length?c.problems[0].message:c.frames.length<2?'it doesn\'t move the arm':null;
      $('#progEditBtn').hidden=false;
      playNote(selCheck?`This motion can't be played: ${selCheck}. Fix it in the Studio.`:`${fmtDur(c.duration)}${c.events.length?` · ${c.events.length} LED cue${c.events.length>1?'s':''}`:''}. Path clear.`);
    }else{
      const q=seqItems.find(x=>x.id===s.id),steps=[];
      for(const st of q.steps){const r=await loadRec(st.recording);steps.push({...r,pause:st.pause});}
      if(sel!==s)return;
      selSteps=steps;selCheck=checkSteps(steps);pathShow(steps.map(x=>x.frames));seqOpen(q);
      playNote(selCheck?`This sequence would collide: ${selCheck}.`:'');
    }
  }catch(e){playNote(e.message);}
  playUI();
}

/* ---- recording editor ---- */
function editShow(r){
  const dur=r.frames[r.frames.length-1][0];
  $('#recEdit').hidden=false;$('#edName').value=r.name;$('#edZero').checked=!!r.return_zero;
  for(const [id,v] of [['#edT0',0],['#edT1',dur]]){const el=$(id);el.max=dur;el.value=v;}
  editTrimUI();
}
function editTrimUI(){
  const r=recCache[sel.id],dur=r.frames[r.frames.length-1][0];
  let t0=+$('#edT0').value,t1=+$('#edT1').value;
  if(t1-t0<0.2){if(document.activeElement===$('#edT0'))t0=Math.max(0,t1-0.2);else t1=Math.min(dur,t0+0.2);$('#edT0').value=t0;$('#edT1').value=t1;}
  $('#edT0v').textContent=t0.toFixed(1)+' s';$('#edT1v').textContent=t1.toFixed(1)+' s';
  const trimmed=t0>0.05||t1<dur-0.05;$('#edTrim').disabled=!trimmed;
  const frames=r.frames.filter(f=>f[0]>=t0-1e-6&&f[0]<=t1+1e-6);
  selCheck=checkSteps([{name:r.name,frames:frames.length>1?frames:r.frames,return_zero:$('#edZero').checked}]);
  pathShow([r.frames],[t0,t1]);
  $('#edCheck').textContent=selCheck?`Collision: ${selCheck}. It can't be played like this.`
    :`Path clear${trimmed?' (trimmed part)':''}. ${r.frames.length} samples${(r.events||[]).length?`, ${r.events.length} LED cues`:''}.`;
  $('#edCheck').className='callout'+(selCheck?' warn':'');
  playUI();
}
async function editApply(body,msg){
  const id=sel.id;
  try{await api('PATCH','/recordings/'+id,body);delete recCache[id];playNote(msg);await libRefresh();await selectItem({kind:'rec',id});}
  catch(e){playNote(e.message);}
}

/* ---- sequence editor ---- */
let seqDraft=null;   // {id|null, name, steps:[{recording, pause}]}
function seqOpen(q){
  seqDraft=q?{id:q.id,name:q.name,steps:q.steps.map(s=>({...s}))}:{id:null,name:'',steps:[]};
  $('#seqEdit').hidden=false;$('#seqEditTitle').textContent=q?'Edit sequence':'New sequence';$('#seqName').value=seqDraft.name;
  $('#seqDel').hidden=!q;seqRenderSteps();
}
function seqRenderSteps(){
  const box=$('#seqSteps');box.textContent='';
  seqDraft.steps.forEach((st,k)=>{
    const r=recItems.find(x=>x.id===st.recording),row=document.createElement('div');row.className='step';
    const n=document.createElement('span');n.className='n';n.textContent=`${k+1}. ${r?r.name:'(deleted recording)'}`;
    const p=document.createElement('input');p.type='number';p.min=0;p.max=600;p.step=0.5;p.value=st.pause;p.title='Pause after this step (s)';p.setAttribute('aria-label',`Pause after step ${k+1}, seconds`);
    p.addEventListener('input',()=>{st.pause=Math.max(0,Math.min(600,+p.value||0));});
    const btn=(t,label,fn,dis)=>{const b=document.createElement('button');b.type='button';b.textContent=t;b.title=label;b.setAttribute('aria-label',label);b.disabled=!!dis;b.addEventListener('click',fn);return b;};
    const swap=d=>{const s=seqDraft.steps;[s[k],s[k+d]]=[s[k+d],s[k]];seqRenderSteps();};
    row.append(n,p,btn('↑','Move up',()=>swap(-1),k===0),btn('↓','Move down',()=>swap(1),k===seqDraft.steps.length-1),
      btn('×','Remove step',()=>{seqDraft.steps.splice(k,1);seqRenderSteps();}));
    box.appendChild(row);
  });
  if(!seqDraft.steps.length){const e=document.createElement('p');e.className='note';e.style.margin='0';e.textContent='No steps yet. Pick a recording below and add it; the number is the pause after it, in seconds.';box.appendChild(e);}
}

/* ---- playback ---- */
function selName(){const l=sel&&itemsOf(sel.kind).find(x=>x.id===sel.id);return l?l.name:'';}
export function playUI(){
  const busy=busyPlaying(),st=S.remotePlay||(S.play&&S.play.status());
  $('#recPlay').textContent=busy?'Stop playback':'Play';
  $('#recPlay').disabled=!busy&&(!selSteps||isRecording()||!!selCheck);
  $('#playWhere').textContent=linkLive()?'on the arm':'simulation';
  const nm=$('#playName');
  if(st){nm.textContent=st.name;const sm=document.createElement('small');
    sm.textContent=`${st.steps>1?`${st.step+1}/${st.steps} · `:''}${{approach:'moving to start',run:'playing',finish:'finishing',zero:'returning to zero',pause:'pause'}[st.phase]||st.phase}`;nm.appendChild(sm);}
  else nm.textContent=S.remotePending?'Starting…':(selName()||'Pick a recording, a sequence or a motion below');
  $('#recProg').style.width=st&&st.phase!=='approach'?(Math.min(1,st.t/Math.max(st.duration,1e-3))*100).toFixed(1)+'%':'0';
  nowPlaying(st);
  recUI();
}
/* the bar over the 3D view while something plays: name, what it's doing (for a motion, which block), time left */
const PHASE={approach:'moving to the start',run:'playing',finish:'finishing',zero:'returning to zero',pause:'pausing'};
function nowPlaying(st){
  const bar=$('#nowPlaying');
  if(!st&&!S.remotePending){bar.hidden=true;return;}
  bar.hidden=false;bar.classList.toggle('local',!!S.play);
  $('#npName').textContent=st?st.name:'Starting…';
  let sub='',frac=0;
  if(st){
    sub=PHASE[st.phase]||st.phase;
    if(st.steps>1)sub=`step ${st.step+1} of ${st.steps} · ${sub}`;
    if(st.phase==='run'&&progInfo&&progInfo.name===st.name){ // a motion: the block it's on
      let id=null;for(const m of progInfo.marks){if(m[0]<=st.t+1e-6)id=m[1];else break;}
      if(id&&progInfo.labels[id])sub=progInfo.labels[id];}
    if(st.phase!=='approach'){frac=Math.min(1,st.t/Math.max(st.duration,1e-3));
      const left=Math.max(0,(st.duration-st.t)/(st.rate||1));$('#npLeft').textContent=st.phase==='run'?`${left<10?left.toFixed(1):Math.round(left)} s left`:'';}
    else $('#npLeft').textContent='';
  }
  $('#npSub').textContent=sub+(S.play?' · simulation':'');
  $('#npRing').style.setProperty('--p',frac);
}
async function playStart(){
  if(busyPlaying()||isRecording()||!selSteps||selCheck)return;
  if(S.stopped){playNote('The arm is stopped. Press Resume first.');return;}
  const o={rate:$('#recRate').value/100,loop:$('#recLoop').checked,timed:$('#recTimed').checked,speed:Math.min(150,+spd.value),acc:+acc.value};
  if(S.limp)setLimp(false);
  if(linkLive()){
    S.remotePending=true;S.remoteStopSent=false;playUI();
    setTimeout(()=>{if(S.remotePending&&!S.remotePlay){S.remotePending=false;playUI();}},3000);
    try{await api('POST','/playback',{...({rec:{recording:sel.id},seq:{sequence:sel.id},prog:{program:sel.id}}[sel.kind]),...o});playNote('Playing on the arm. It keeps going if you close this page.');
      toast(`Playing "${selName()}" on the arm`);}
    catch(e){S.remotePending=false;playNote(e.message);toast(e.message,'bad');playUI();}
    return;
  }
  setDemo(false);S.homeLock=true;
  S.play=new Player(selName(),selSteps,performance.now()/1000,o);
  playNote('Playing in the simulation (the arm isn\'t connected).');toast(`Playing "${selName()}" in the simulation`);playUI();
}
export function playStop(){
  if(S.remotePlay||S.remotePending)api('POST','/playback/stop').catch(e=>playNote(e.message));
  else if(S.play)endPlay('Playback stopped.');
}
export function endPlay(msg){ // end local playback
  if(!S.play)return;
  S.play=null;S.simSpeeds=null;
  for(let i=0;i<N;i++)qIK[i]=qCmd[i];   // hold the last pose that was actually sent
  targetFromPose();S.homeLock=true;
  if(msg){playNote(msg);toast(msg,/finished/i.test(msg)?'good':/stopped:/i.test(msg)?'bad':'info');}playUI();
}
export function playStep(){ // local playback, once per frame: the Player's goal becomes qIK
  if(!S.homeLock){endPlay('Playback stopped: the target was moved.');return;}
  const act=S.play.tick(performance.now()/1000,servo.map(s=>s.pos/DEG));
  act.events.forEach(ledApply);
  if(act.goal){for(let j=0;j<N;j++)qIK[j]=clampJ(j,act.goal[j]);S.simSpeeds=act.speeds;targetFromPose();}
  if(act.done)S.play.done=true;
  playUI();
}

export const selectedMotion=()=>sel&&sel.kind==='prog'?sel.id:null;
export function initPlay(){
  $('#npStop').addEventListener('click',playStop);
  ['#edT0','#edT1'].forEach(id=>$(id).addEventListener('input',editTrimUI));
  $('#edRename').addEventListener('click',()=>{const n=$('#edName').value.trim();if(n)editApply({name:n},`Renamed to "${n}".`);});
  $('#edName').addEventListener('keydown',e=>{if(e.key==='Enter')$('#edRename').click();});
  $('#edZero').addEventListener('change',e=>editApply({return_zero:e.target.checked},e.target.checked?'It will return to the zero pose after playing.':'It will stay at its last pose after playing.'));
  $('#edTrim').addEventListener('click',()=>{const a=+$('#edT0').value,b=+$('#edT1').value;
    if(confirm(`Keep only ${a.toFixed(1)} s to ${b.toFixed(1)} s of this recording? The rest is deleted.`))editApply({trim:[a,b]},'Trimmed.');});
  $('#edExport').addEventListener('click',()=>{
    const r=recCache[sel.id],blob=new Blob([JSON.stringify({format:'mycobot280-recording',version:1,name:r.name,return_zero:!!r.return_zero,frames:r.frames,events:r.events||[]})],{type:'application/json'});
    const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=r.name.replace(/[^\w.-]+/g,'_')+'.json';document.body.appendChild(a);a.click();a.remove();
    setTimeout(()=>URL.revokeObjectURL(a.href),1000);
  });
  $('#edDel').addEventListener('click',async()=>{
    const r=recItems.find(x=>x.id===sel.id);if(!r||!confirm(`Delete the recording "${r.name}"?`))return;
    try{await api('DELETE','/recordings/'+r.id);delete recCache[r.id];playNote(`Deleted "${r.name}".`);selectItem(null);await libRefresh();}
    catch(e){playNote(e.message);}
  });
  $('#recImport').addEventListener('click',()=>$('#recFile').click());
  $('#recFile').addEventListener('change',async e=>{
    const f=e.target.files[0];e.target.value='';if(!f)return;
    try{
      let j;try{j=JSON.parse(await f.text());}catch(_){throw new Error('That file isn\'t JSON.');}
      if(!j||!Array.isArray(j.frames))throw new Error('That file isn\'t a recording exported from this page.');
      const name=String(j.name||f.name.replace(/\.json$/i,'')).slice(0,60)||'Imported';
      const r=await api('POST','/recordings',{name,frames:j.frames,events:Array.isArray(j.events)?j.events:[],return_zero:!!j.return_zero});
      playNote(`Imported "${r.name}".`);await libRefresh();selectItem({kind:'rec',id:r.id});
    }catch(err){playNote('Import failed: '+err.message);}
  });
  $('#recRefresh').addEventListener('click',()=>libRefresh());
  $('#seqNew').addEventListener('click',()=>{seqOpen(null);$('#seqName').focus();});
  $('#seqAdd').addEventListener('click',()=>{const id=$('#seqPick').value;if(!seqDraft||!id)return;seqDraft.steps.push({recording:id,pause:0});seqRenderSteps();});
  $('#seqCancel').addEventListener('click',()=>{seqDraft=null;$('#seqEdit').hidden=true;});
  $('#seqSave').addEventListener('click',async()=>{
    seqDraft.name=$('#seqName').value.trim();
    if(!seqDraft.name){$('#seqNote').textContent='Give the sequence a name.';return;}
    if(!seqDraft.steps.length){$('#seqNote').textContent='Add at least one step.';return;}
    try{
      const body={name:seqDraft.name,steps:seqDraft.steps};
      const q=seqDraft.id?await api('PUT','/sequences/'+seqDraft.id,body):await api('POST','/sequences',body);
      $('#seqNote').textContent=`Saved "${q.name}".`;await libRefresh();selectItem({kind:'seq',id:q.id});
    }catch(e){$('#seqNote').textContent=e.message;}
  });
  $('#seqDel').addEventListener('click',async()=>{
    if(!seqDraft||!seqDraft.id||!confirm(`Delete the sequence "${seqDraft.name}"? Its recordings are kept.`))return;
    try{await api('DELETE','/sequences/'+seqDraft.id);$('#seqNote').textContent='Deleted.';seqDraft=null;$('#seqEdit').hidden=true;selectItem(null);await libRefresh();}
    catch(e){$('#seqNote').textContent=e.message;}
  });
  $('#recPlay').addEventListener('click',()=>busyPlaying()?playStop():playStart());
  $('#recRate').addEventListener('input',e=>{$('#recRatev').textContent=(e.target.value/100)+'×';});
  $('#tabbtn-play').addEventListener('click',()=>{if(!recItems.length&&!seqItems.length&&$('#wsPw').value)libRefresh();});
}
