/* Motion Studio: build arm motions from blocks, try them in a sandbox, save them, play them later from the Play tab.

   The sandbox is its own 3D view (scene.makeSandbox: the same scene, another camera and canvas) whose arm is
   never linked to the real one. A program is a list of blocks (program.py on the backend has the format): move
   to a joint pose, move the tool tip to a point, go to zero, wait, set the LEDs, repeat; each may carry a note.
   Every change is compiled by the backend (/api/programs/compile: the same code that plays it, IK for the points,
   collision check included), and the sandbox previews the compiled frames. There can be any number of programs,
   saved on the backend (/api/programs), one tab each above the blocks; they're played from the Play tab (Motions),
   on the arm through /api/playback like a recording, with every guard, or in the simulation.

   Editing: click a block type (or drag it) to add it after the selected block; drag blocks by their grip to
   reorder them, into and out of repeats; the small buttons move, copy, label and delete. A selected pose block has
   sliders, and rings on the sandbox arm's joints to turn them by dragging; a point block has its coordinates, and
   arrows on its point in the sandbox. "From the arm" copies the real arm's pose. Undo/redo (Ctrl+Z, Ctrl+Shift+Z)
   covers every change; a slider or a drag is one step. The timeline under the sandbox shows where the time goes,
   one segment per block. The last view and the open motion are remembered (uiSet). */
import {DEG,N,URDF_LIM,makeFK,fk} from './kinematics.js';
import {makeSandbox,renderSandbox,setSandPath,setSandMark,setSandJoints,pickSandJoint,sandToScreen,sandCamPos} from './scene.js';
import {S,haveRealNow} from './state.js';
import {libRefresh,selectedMotion} from './play.js';
import {api} from './api.js';
import {toast} from './toast.js';
import {onView,setView} from './views.js';
import {$,V,uiGet,uiSet} from './util.js';

export const TYPES={
  pose:{label:'Move to pose',hint:'a joint pose',make:()=>({type:'pose',angles:sandPose().map(v=>+(v/DEG).toFixed(1)),speed:40})},
  point:{label:'Move to point',hint:'the tool tip to x, y, z',make:()=>{const p=tipOf(sandPose());
    // where the sandbox's tool tip is, unless that's up near the top (straight up can't face down): in front
    const xyz=p.z>0.3?[180,0,150]:[p.x,p.y,p.z].map(v=>Math.round(v*1000));
    return{type:'point',xyz,down:true,speed:40};}},
  home:{label:'Go to zero',hint:'straight up',make:()=>({type:'home',speed:40})},
  wait:{label:'Wait',hint:'hold still',make:()=>({type:'wait',seconds:1})},
  led:{label:'LED colour',hint:'the ATOM\'s LEDs',make:()=>({type:'led',color:[255,160,0]})},
  repeat:{label:'Repeat',hint:'the blocks inside',make:()=>({type:'repeat',times:2,blocks:[]})},
};
const JN=['J1','J2','J3','J4','J5','J6'];
const HIST_MAX=100,COALESCE_MS=700;   // undo steps kept; changes this close together (a slider, a drag) are one step
let prog=blank(),dirty=false,sel=null,compiled=null,compileTimer=null,compiling=false,again=false,listCache=[];
let preview=null,sandbox=null,inited=false,lastPose=new Array(N).fill(0),editingNote=null,jointDrag=null,hotJoint=-1;
let hist=[],fut=[],lastSnap='',lastEdit=0;
const tipF=makeFK();

function blank(){return{id:null,name:'Untitled motion',blocks:[{id:newId(),type:'home',speed:40}]};}
function newId(){return Math.random().toString(36).slice(2,10);}
function tipOf(q){fk(q,tipF);return tipF.tcp.clone();}
const note=t=>{$('#stNote').textContent=t;};
const esc=t=>String(t).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const hex=c=>'#'+c.map(v=>v.toString(16).padStart(2,'0')).join('');
const rgb=h=>{const n=parseInt(h.slice(1),16);return[n>>16&255,n>>8&255,n&255];};
export const blockLabel=b=>b.note||TYPES[b.type].label;

/* ---------- the block tree ---------- */
function* walk(list,parent=null){for(let i=0;i<list.length;i++){yield{b:list[i],list,i,parent};if(list[i].blocks)yield*walk(list[i].blocks,list[i]);}}
function find(id){for(const w of walk(prog.blocks))if(w.b.id===id)return w;return null;}
function contains(b,id){return b.id===id||(b.blocks||[]).some(c=>contains(c,id));}
function cloneWithNewIds(b){const c=JSON.parse(JSON.stringify(b));for(const w of walk([c]))w.b.id=newId();return c;}
function insertNear(block,refId,where){ // where: 'after' | 'before' | 'into'; no ref: at the end
  const ref=refId&&find(refId);
  if(!ref)prog.blocks.push(block);
  else if(where==='into')ref.b.blocks.push(block);
  else ref.list.splice(ref.i+(where==='after'?1:0),0,block);
}
/* something changed: re-render (unless the caller updated the DOM itself), remember it for undo, recompile.
   continuous: part of a slider move or a drag, merged with the changes just before it into one undo step */
function changed(rerender=true,continuous=false){
  const was=dirty;dirty=true;if(rerender)render();$('#stName').classList.toggle('dirty',dirty);
  record(continuous);if(!was)renderTabs();scheduleCompile();
}

/* ---------- undo / redo: snapshots of the name and the blocks ---------- */
const snap=()=>JSON.stringify({name:prog.name,blocks:prog.blocks});
function resetHistory(){hist=[];fut=[];lastSnap=snap();lastEdit=0;undoUI();}
function record(continuous){
  const s=snap();if(s===lastSnap)return;
  const now=performance.now();
  if(!(continuous&&now-lastEdit<COALESCE_MS&&hist.length)){hist.push(lastSnap);if(hist.length>HIST_MAX)hist.shift();}
  lastEdit=continuous?now:0;fut=[];lastSnap=s;undoUI();
}
function restore(s){
  const o=JSON.parse(s);prog.name=o.name;prog.blocks=o.blocks;lastSnap=s;lastEdit=0;
  if(sel&&!find(sel))sel=null;editingNote=null;dirty=true;stopPreview();render();renderTabs();scheduleCompile();undoUI();
}
function undo(){if(!hist.length)return;fut.push(snap());restore(hist.pop());}
function redo(){if(!fut.length)return;hist.push(snap());restore(fut.pop());}
function undoUI(){$('#stUndo').disabled=!hist.length;$('#stRedo').disabled=!fut.length;}

/* ---------- the sandbox's pose ---------- */
function sandPose(){ // radians
  if(preview)return frameAt(preview.t).map(v=>v*DEG);
  const w=sel&&find(sel);
  if(w&&w.b.type==='pose')return w.b.angles.map(v=>v*DEG);
  if(w&&compiled&&compiled.solved[sel])return compiled.solved[sel].map(v=>v*DEG);
  if(w&&compiled&&(w.b.type==='wait'||w.b.type==='led'||w.b.type==='repeat')){const t=markTime(sel);if(t!==null)return frameAt(t).map(v=>v*DEG);}
  if(compiled&&compiled.frames.length)return compiled.frames[0].slice(1).map(v=>v*DEG);
  return new Array(N).fill(0);
}
function frameAt(t){ // degrees, linear between the compiled frames
  const f=compiled&&compiled.frames;if(!f||!f.length)return new Array(N).fill(0);
  if(t<=f[0][0])return f[0].slice(1);if(t>=f[f.length-1][0])return f[f.length-1].slice(1);
  let lo=0,hi=f.length-1;while(hi-lo>1){const m=(lo+hi)>>1;if(f[m][0]<=t)lo=m;else hi=m;}
  const a=f[lo],b=f[hi],u=(t-a[0])/Math.max(1e-9,b[0]-a[0]);
  return a.slice(1).map((v,j)=>v+(b[j+1]-v)*u);
}
function markTime(id){const m=compiled&&compiled.marks.find(m=>m[1]===id);return m?m[0]:null;}
function blockAt(t){let id=null;if(compiled)for(const m of compiled.marks){if(m[0]<=t+1e-6)id=m[1];else break;}return id;}
function ledAt(t){let c=null;if(compiled)for(const e of compiled.events){if(e[0]<=t+1e-6&&e[1]==='color')c=e[2];}return c;}

/* ---------- rendering the program ---------- */
function summary(b){
  switch(b.type){
    case 'pose':return b.angles.map(v=>Math.round(v)+'°').join(' ');
    case 'point':return `${b.xyz.join(', ')} mm${b.down?' · facing down':''}`;
    case 'home':return 'every joint at 0°';
    case 'wait':return `${b.seconds} s`;
    case 'led':return '';
    case 'repeat':return `${b.times} times`;
  }
}
function fieldsFor(b){ // inline fields in the block's head
  switch(b.type){
    case 'wait':return `<label class="bf">for <input type="number" data-f="seconds" min="0" max="600" step="0.1" value="${b.seconds}"> s</label>`;
    case 'led':return `<label class="bf"><input type="color" data-f="color" value="${hex(b.color)}" aria-label="LED colour"></label>`;
    case 'repeat':return `<label class="bf"><input type="number" data-f="times" min="1" max="100" step="1" value="${b.times}"> times</label>`;
    default:return `<span class="bsum">${esc(summary(b))}</span>`;
  }
}
function editorFor(b){ // the selected move block's editor, under its head
  if(!['pose','point','home'].includes(b.type))return '';
  let h='';
  if(b.type==='pose')h+=b.angles.map((v,j)=>{const lo=Math.ceil(URDF_LIM[j][0]/DEG),hi=Math.floor(URDF_LIM[j][1]/DEG);
    return `<label class="bj"><b>${JN[j]}</b><input type="range" data-f="angle" data-j="${j}" min="${lo}" max="${hi}" step="0.5" value="${v}"><output>${v.toFixed(1)}°</output></label>`;}).join('')
    +'<p class="bhint">Or drag the rings on the sandbox arm\'s joints.</p>';
  if(b.type==='point')h+=`<div class="bxyz">${['x','y','z'].map((c,k)=>`<label>${c}<input type="number" data-f="xyz" data-k="${k}" step="1" value="${b.xyz[k]}"></label>`).join('')}<span>mm</span></div>
    <label class="chk bdown"><input type="checkbox" data-f="down" ${b.down?'checked':''}> Flange facing down</label>
    <p class="bhint">Or drag the arrows on the point in the sandbox.</p>`;
  h+=`<label class="bj bspeed"><b>Speed</b><input type="range" data-f="speed" min="5" max="150" step="5" value="${b.speed}"><output>${b.speed}°/s</output></label>`;
  if(b.type!=='home')h+=`<div class="bbtns"><button class="ghost" data-act="fromArm" ${haveRealNow()?'':'disabled'} title="Copy the real arm's pose (pose it by hand with Hand-guide)">From the arm</button></div>`;
  return `<div class="bedit">${h}</div>`;
}
function blockEl(b){
  const el=document.createElement('div');el.className=`blk blk-${b.type}${b.id===sel?' sel':''}`;el.dataset.id=b.id;
  const noteH=editingNote===b.id
    ?`<div class="bnote-row"><input class="bnote-in" maxlength="80" placeholder="What this block is for, e.g. pick up the part" value="${esc(b.note||'')}" aria-label="Note"></div>`
    :b.note?`<div class="bnote-row"><span class="bnote">${esc(b.note)}</span></div>`:'';
  // only the grip drags the block: the rest of it has fields and sliders to use
  el.innerHTML=`<div class="bhead"><span class="bgrip" draggable="true" title="Drag to move" aria-hidden="true">⋮⋮</span><b class="btitle">${TYPES[b.type].label}</b>${fieldsFor(b)}
    <span class="bacts"><button data-act="note" title="Add a note" aria-label="Note">✎</button><button data-act="up" title="Move up" aria-label="Move up">↑</button><button data-act="down" title="Move down" aria-label="Move down">↓</button><button data-act="copy" title="Duplicate" aria-label="Duplicate">⧉</button><button data-act="del" title="Delete" aria-label="Delete">✕</button></span></div>
    ${noteH}${b.id===sel?editorFor(b):''}<p class="bprob" hidden></p>`;
  if(b.type==='repeat'){
    const inner=document.createElement('div');inner.className='binner';inner.dataset.into=b.id;
    b.blocks.forEach(c=>inner.appendChild(blockEl(c)));
    if(!b.blocks.length){const e=document.createElement('p');e.className='bempty';e.textContent='Drag blocks in here';inner.appendChild(e);}
    const foot=document.createElement('div');foot.className='bfoot';
    el.appendChild(inner);el.appendChild(foot);
  }
  return el;
}
function render(){
  const box=$('#stProg');box.textContent='';
  prog.blocks.forEach(b=>box.appendChild(blockEl(b)));
  if(!prog.blocks.length){const e=document.createElement('p');e.className='bempty top';e.textContent='Add a block from above to start.';box.appendChild(e);}
  $('#stName').value=prog.name;
  markProblems();renderSegs();
  const ni=box.querySelector('.bnote-in');if(ni){ni.focus();ni.select();}
}
function markProblems(){
  document.querySelectorAll('#stProg .blk').forEach(el=>{el.classList.remove('bad');const p=el.querySelector(':scope>.bprob');p.hidden=true;p.textContent='';});
  if(!compiled)return;
  for(const pr of compiled.problems){
    const el=document.querySelector(`#stProg .blk[data-id="${pr.block}"]`);if(!el)continue;
    el.classList.add('bad');const p=el.querySelector(':scope>.bprob');p.hidden=false;p.textContent=pr.message;
  }
}
/* the timeline: one segment per top-level block, as wide as its share of the time; click one to go there */
function renderSegs(){
  const box=$('#stSegs');box.textContent='';
  if(!compiled||compiled.duration<=0)return;
  const top=new Map(prog.blocks.map(b=>[b.id,b])),starts=[];
  for(const [t,id] of compiled.marks)if(top.has(id)&&!starts.some(s=>s.id===id))starts.push({id,t});
  const dur=compiled.duration,bad=new Set(compiled.problems.map(p=>p.block));
  starts.forEach((s,k)=>{
    const end=k+1<starts.length?starts[k+1].t:dur,b=top.get(s.id),span=Math.max(0,end-s.t);
    const el=document.createElement('button');el.type='button';
    el.className=`sseg blk-${b.type}${s.id===sel?' sel':''}${[...walk([b])].some(w=>bad.has(w.b.id))?' bad':''}`;
    el.style.flexGrow=String(Math.max(span,dur*0.012));el.dataset.id=s.id;el.dataset.t=s.t;
    el.title=`${blockLabel(b)} · ${s.t.toFixed(1)}–${end.toFixed(1)} s`;el.setAttribute('aria-label',el.title);
    box.appendChild(el);
  });
}

/* ---------- compiling ---------- */
function scheduleCompile(){clearTimeout(compileTimer);compileTimer=setTimeout(compileNow,(sandbox&&sandbox.dragging)||jointDrag?40:300);}
async function compileNow(){
  if(compiling){again=true;return;}
  compiling=true;$('#stState').textContent='Checking…';
  try{compiled=await api('POST','/programs/compile',{blocks:prog.blocks});}
  catch(e){compiled=null;$('#stState').textContent='';note(e.message);compiling=false;return;}
  compiling=false;
  if(again){again=false;compileNow();return;}
  afterCompile();
}
function afterCompile(){
  const c=compiled,f=c.frames;
  setSandPath(f.length>1?f.map(r=>tipOf(r.slice(1).map(v=>v*DEG))):null);
  $('#stTime').max=Math.max(0.1,c.duration);
  const n=[...walk(prog.blocks)].length;
  $('#stState').textContent=c.problems.length?`${c.problems.length} problem${c.problems.length>1?'s':''}`:`${c.duration.toFixed(1)} s · ${n} block${n===1?'':'s'}`;
  $('#stState').classList.toggle('bad',!!c.problems.length);
  note(c.problems.length?'Fix the blocks marked in red: the arm won\'t play a motion that has problems.':
    f.length<2?'Add a move so the arm has somewhere to go.':'');
  markProblems();renderSegs();ui();
}

/* ---------- preview ---------- */
function play(){if(!compiled||compiled.frames.length<2)return;preview={t:preview&&preview.t<compiled.duration-1e-3?preview.t:0,playing:true};ui();}
function pause(){if(preview)preview.playing=false;ui();}
function stopPreview(){preview=null;document.querySelectorAll('#stProg .blk.running,#stSegs .now').forEach(e=>e.classList.remove('running','now'));ui();}
function ui(){
  const ok=!!(compiled&&compiled.frames.length>1),playing=!!(preview&&preview.playing);
  $('#stPlay').disabled=!ok;$('#stPlay').setAttribute('aria-pressed',playing);$('#stPlay').textContent=playing?'Pause':'Preview';
  $('#stStop').disabled=!preview;
  ['#stmDelete','#stmDuplicate'].forEach(id=>{$(id).disabled=!prog.id;});
}

/* ---------- saving, opening, the motion menu ---------- */
async function save(){
  const name=$('#stName').value.trim()||'Untitled motion';prog.name=name;
  const body={name,blocks:prog.blocks};
  const r=prog.id?await api('PUT','/programs/'+prog.id,body):await api('POST','/programs',body);
  prog.id=r.id;dirty=false;$('#stName').classList.remove('dirty');uiSet('mycobot-motion',prog.id);
  note(`Saved "${name}". Play it from the arm view's Play tab (Motions).`);toast(`Saved "${name}"`,'good');ui();
  return r;
}
/* one tab per saved motion (the open one highlighted; an unsaved new one too), and "+ New" */
async function refreshList(){try{listCache=await api('GET','/programs');}catch(e){listCache=[];note(e.message);}renderTabs();}
function renderTabs(){
  const box=$('#stTabs');box.textContent='';
  const tab=(label,on,fn,cls='')=>{const b=document.createElement('button');b.className='st-tab'+(on?' on':'')+cls;b.setAttribute('role','tab');
    b.setAttribute('aria-selected',on);b.textContent=label;b.addEventListener('click',fn);box.appendChild(b);return b;};
  listCache.forEach(p=>{const t=tab(p.id===prog.id?(prog.name||p.name):p.name,p.id===prog.id,()=>{if(p.id!==prog.id)openProgram(p.id);});t.dataset.id=p.id;
    if(p.id===prog.id&&dirty)t.classList.add('dirty');});
  if(!prog.id)tab(prog.name||'Untitled motion',true,()=>{},' dirty');
  tab('+ New',false,newProgram,' st-new').title='Start a new motion';
}
function load(p){prog=p;dirty=false;sel=null;editingNote=null;stopPreview();render();resetHistory();compileNow();renderTabs();ui();}
async function openProgram(id,quiet){
  if(dirty&&!confirm('Discard the changes to this motion?'))return false;
  let p;try{p=await api('GET','/programs/'+id);}catch(e){if(!quiet)note(e.message);return false;}
  load({id:p.id,name:p.name,blocks:p.blocks});uiSet('mycobot-motion',p.id);if(!quiet)note(`Opened "${p.name}".`);
  return true;
}
function newProgram(){
  if(dirty&&!confirm('Discard the changes to this motion?'))return;
  load(blank());uiSet('mycobot-motion',null);note('A new motion: give it a name and save it.');
}
async function deleteProgram(){
  if(!prog.id||!confirm(`Delete "${prog.name}"?`))return;
  try{await api('DELETE','/programs/'+prog.id);}catch(e){note(e.message);toast(e.message,'bad');return;}
  toast(`Deleted "${prog.name}"`);load(blank());uiSet('mycobot-motion',null);refreshList();
}
async function duplicateProgram(){
  if(!prog.id)return;
  const copy={id:null,name:(prog.name+' copy').slice(0,60),blocks:prog.blocks.map(cloneWithNewIds)};
  try{const r=await api('POST','/programs',{name:copy.name,blocks:copy.blocks});copy.id=r.id;}catch(e){toast(e.message,'bad');return;}
  load(copy);uiSet('mycobot-motion',copy.id);refreshList();toast(`Made "${copy.name}"`,'good');
}
function exportProgram(){
  const data={format:'mycobot280-motion',version:1,name:prog.name,blocks:prog.blocks};
  const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(data,null,1)],{type:'application/json'}));
  a.download=(prog.name||'motion').replace(/[^\w.-]+/g,'_')+'.motion.json';document.body.appendChild(a);a.click();a.remove();
  setTimeout(()=>URL.revokeObjectURL(a.href),1000);toast(`Exported "${prog.name}"`);
}
async function importProgram(file){
  let j;try{j=JSON.parse(await file.text());}catch(_){toast('That file isn\'t JSON.','bad');return;}
  if(!j||j.format!=='mycobot280-motion'||!Array.isArray(j.blocks)){toast('That file isn\'t a motion exported from the Studio.','bad');return;}
  if(dirty&&!confirm('Discard the changes to this motion?'))return;
  const p={id:null,name:String(j.name||file.name.replace(/\.(motion\.)?json$/i,'')).slice(0,60)||'Imported',blocks:j.blocks.map(cloneWithNewIds)};
  try{const r=await api('POST','/programs',{name:p.name,blocks:p.blocks});p.id=r.id;}catch(e){toast('Import failed: '+e.message,'bad');return;}
  load(p);uiSet('mycobot-motion',p.id);refreshList();toast(`Imported "${p.name}"`,'good');
}
function menu(open){$('#stMenu').hidden=!open;$('#stMore').setAttribute('aria-expanded',open);}

/* ---------- editing events ---------- */
function onField(e){
  const el=e.target,bEl=el.closest('.blk');if(!bEl||!el.dataset.f)return;
  const w=find(bEl.dataset.id);if(!w)return;const b=w.b,f=el.dataset.f;
  if(f==='angle'){b.angles[+el.dataset.j]=+el.value;el.nextElementSibling.textContent=(+el.value).toFixed(1)+'°';
    bEl.querySelector('.bsum').textContent=summary(b);}
  else if(f==='speed'){b.speed=+el.value;el.nextElementSibling.textContent=el.value+'°/s';}
  else if(f==='xyz'){const v=+el.value;if(!Number.isFinite(v))return;b.xyz[+el.dataset.k]=Math.max(-1000,Math.min(1000,v));bEl.querySelector('.bsum').textContent=summary(b);}
  else if(f==='down'){b.down=el.checked;bEl.querySelector('.bsum').textContent=summary(b);}
  else if(f==='seconds'){const v=+el.value;if(!(v>=0&&v<=600))return;b.seconds=v;}
  else if(f==='times'){const v=Math.round(+el.value);if(!(v>=1&&v<=100))return;b.times=v;}
  else if(f==='color')b.color=rgb(el.value);
  changed(false,true);
}
function commitNote(input,keep){
  const bEl=input.closest('.blk'),w=bEl&&find(bEl.dataset.id);editingNote=null;
  if(w&&keep){const v=input.value.trim();if(v)w.b.note=v;else delete w.b.note;changed();}else render();
}
function onAct(btn){
  const bEl=btn.closest('.blk'),w=bEl&&find(bEl.dataset.id);if(!w)return;
  const act=btn.dataset.act;
  if(act==='note'){editingNote=w.b.id;render();return;}
  if(act==='del'){w.list.splice(w.i,1);if(sel&&contains(w.b,sel))sel=null;}
  else if(act==='copy'){const c=cloneWithNewIds(w.b);w.list.splice(w.i+1,0,c);sel=c.id;}
  else if(act==='up'&&w.i>0)[w.list[w.i-1],w.list[w.i]]=[w.list[w.i],w.list[w.i-1]];
  else if(act==='down'&&w.i<w.list.length-1)[w.list[w.i+1],w.list[w.i]]=[w.list[w.i],w.list[w.i+1]];
  else if(act==='fromArm'&&haveRealNow()){
    const a=S.measured.slice();
    if(w.b.type==='pose')w.b.angles=a.map(v=>+v.toFixed(1));
    else{const p=tipOf(a.map(v=>v*DEG));w.b.xyz=[p.x,p.y,p.z].map(v=>Math.round(v*1000));}
    toast('Copied the arm\'s pose','good');}
  else return;
  stopPreview();changed();
}
function addBlock(type,refId,where){
  const b={id:newId(),...TYPES[type].make()};
  const ref=refId||sel;
  const w=ref&&find(ref);
  insertNear(b,ref,where||(w&&w.b.type==='repeat'&&!refId?'into':'after'));
  sel=b.id;stopPreview();changed();
  const el=document.querySelector(`#stProg .blk[data-id="${b.id}"]`);if(el&&el.scrollIntoView)el.scrollIntoView({block:'nearest'});
}
/* drag and drop: palette buttons carry "new:<type>", blocks "move:<id>"; dropping on a block's upper or lower
   half puts it before or after, on a repeat's inside puts it in there */
let dropAt=null;
function clearDrop(){document.querySelectorAll('#stProg .drop-before,#stProg .drop-after,#stProg .drop-into').forEach(e=>e.classList.remove('drop-before','drop-after','drop-into'));dropAt=null;}
function onDragOver(e){
  const inner=e.target.closest('.binner'),blk=e.target.closest('.blk');
  e.preventDefault();clearDrop();
  if(blk&&(!inner||inner.contains(blk))&&!blk.classList.contains('dragging')){
    const r=blk.getBoundingClientRect(),before=e.clientY<r.top+Math.min(r.height/2,22);
    blk.classList.add(before?'drop-before':'drop-after');dropAt={ref:blk.dataset.id,where:before?'before':'after'};
    if(blk.classList.contains('blk-repeat')&&!before&&inner===null&&e.clientY<r.bottom-10){blk.classList.remove('drop-after');blk.querySelector('.binner').classList.add('drop-into');dropAt={ref:blk.dataset.id,where:'into'};}
  }else if(inner){inner.classList.add('drop-into');dropAt={ref:inner.dataset.into,where:'into'};}
  else dropAt={ref:null,where:'after'};
}
function onDrop(e){
  e.preventDefault();const data=(e.dataTransfer&&e.dataTransfer.getData('text/plain'))||'',at=dropAt;clearDrop();
  if(!at)return;
  if(data.startsWith('new:')&&TYPES[data.slice(4)]){addBlock(data.slice(4),at.ref,at.where);return;}
  if(data.startsWith('move:')){
    const w=find(data.slice(5));if(!w)return;
    if(at.ref&&contains(w.b,at.ref))return;   // not into itself
    w.list.splice(w.i,1);insertNear(w.b,at.ref,at.where);sel=w.b.id;stopPreview();changed();
  }
}

/* ---------- the sandbox: arrows on a selected point, rings on a selected pose's joints ---------- */
function movePoint(p){ // the arrows moved the selected point (p: base frame, metres)
  const w=sel&&find(sel);if(!w||w.b.type!=='point')return;
  w.b.xyz=[p.x,p.y,p.z].map(v=>Math.max(-1000,Math.min(1000,Math.round(v*1000))));
  const el=document.querySelector(`#stProg .blk[data-id="${sel}"]`);
  if(el){el.querySelectorAll('input[data-f="xyz"]').forEach(i=>{i.value=w.b.xyz[+i.dataset.k];});el.querySelector('.bsum').textContent=summary(w.b);}
  changed(false,true);
}
const poseSel=()=>{const w=!preview&&sel&&find(sel);return w&&w.b.type==='pose'?w.b:null;};
/* turning a joint by dragging its ring: the pointer's angle round the joint's centre on screen, the right way
   round for which way the joint's axis faces the camera (edge-on: side to side) */
function screenAngle(e,c){return Math.atan2(-(e.clientY-c.y),e.clientX-c.x);}
function jointDown(e){
  const b=poseSel();if(!b||e.button!==0)return;
  const hit=pickSandJoint(e.clientX,e.clientY,b.angles.map(v=>v*DEG));if(!hit)return;
  e.preventDefault();e.stopImmediatePropagation();   // not an orbit
  const c=sandToScreen(hit.centre),toCam=sandCamPos().sub(hit.centre).normalize(),facing=hit.axis.dot(toCam);
  jointDrag={j:hit.joint,c,sign:facing>=0?1:-1,edge:Math.abs(facing)<0.25,last:screenAngle(e,c),lastX:e.clientX};
  hotJoint=hit.joint;e.target.setPointerCapture&&e.target.setPointerCapture(e.pointerId);
}
function jointMove(e){
  const b=poseSel(),cv=e.target;
  if(!jointDrag){ // hover: light up the ring under the pointer
    const hit=b&&pickSandJoint(e.clientX,e.clientY,b.angles.map(v=>v*DEG));hotJoint=hit?hit.joint:-1;cv.style.cursor=hit?'grab':'';return;}
  if(!b){jointDrag=null;return;}
  e.preventDefault();cv.style.cursor='grabbing';
  let d;
  if(jointDrag.edge){d=(e.clientX-jointDrag.lastX)*0.6;jointDrag.lastX=e.clientX;}
  else{const a=screenAngle(e,jointDrag.c);d=a-jointDrag.last;if(d>Math.PI)d-=2*Math.PI;if(d<-Math.PI)d+=2*Math.PI;jointDrag.last=a;d=jointDrag.sign*d/DEG;}
  const j=jointDrag.j,lo=URDF_LIM[j][0]/DEG,hi=URDF_LIM[j][1]/DEG;
  b.angles[j]=+Math.max(lo,Math.min(hi,b.angles[j]+d)).toFixed(1);
  const el=document.querySelector(`#stProg .blk[data-id="${b.id}"]`);
  if(el){const s=el.querySelector(`input[data-j="${j}"]`);if(s){s.value=b.angles[j];s.nextElementSibling.textContent=b.angles[j].toFixed(1)+'°';}
    el.querySelector('.bsum').textContent=summary(b);}
  changed(false,true);
}
function jointUp(e){if(!jointDrag)return;jointDrag=null;e.target.style.cursor='';compileNow();}

/* ---------- entering and leaving the view, and the frame ---------- */
function enterStudio(){
  {
    if(!sandbox){
      sandbox=makeSandbox($('#stStage'));sandbox.onMove=movePoint;sandbox.onDrag=on=>{if(!on)compileNow();};
      const cv=sandbox.renderer.domElement;
      cv.addEventListener('pointerdown',jointDown,true);cv.addEventListener('pointermove',jointMove);
      cv.addEventListener('pointerup',jointUp);cv.addEventListener('pointercancel',jointUp);
    }else sandbox.fit();
    if(!inited){inited=true;render();resetHistory();compileNow();
      const rem=uiGet('mycobot-motion');if(rem)openProgram(rem,true);}   // the motion that was open last time
    refreshList();ui();
  }
}
function leaveStudio(){stopPreview();libRefresh();}   // the Play tab's Motions list shows what was saved here
/* Called by main.js every frame while the Studio shows: advance the preview and draw the sandbox. */
export function studioFrame(dt){
  if(preview&&preview.playing&&compiled){
    preview.t=Math.min(compiled.duration,preview.t+dt);
    if(preview.t>=compiled.duration)preview.playing=false,ui();
    const id=blockAt(preview.t);
    document.querySelectorAll('#stProg .blk.running').forEach(e=>{if(e.dataset.id!==id)e.classList.remove('running');});
    const el=id&&document.querySelector(`#stProg .blk[data-id="${id}"]`);if(el)el.classList.add('running');
    let top=null;for(const s of document.querySelectorAll('#stSegs .sseg'))if(+s.dataset.t<=preview.t+1e-6)top=s;
    document.querySelectorAll('#stSegs .sseg').forEach(s=>s.classList.toggle('now',s===top));
  }
  if(preview){$('#stTime').value=preview.t;$('#stTimev').textContent=preview.t.toFixed(1)+' s';}
  const q=sandPose();lastPose=q;
  const w=!preview&&sel&&find(sel);
  setSandMark(w&&w.b.type==='point'?V(w.b.xyz[0]/1000,w.b.xyz[1]/1000,w.b.xyz[2]/1000):null);
  setSandJoints(!!poseSel(),jointDrag?jointDrag.j:hotJoint);
  renderSandbox(q,preview?ledAt(preview.t):null);
}
export const studioPose=()=>lastPose.slice();

export function initStudio(){
  const pal=$('#stPalette');
  Object.entries(TYPES).forEach(([k,t])=>{const b=document.createElement('button');b.className=`pal pal-${k}`;b.draggable=true;b.dataset.type=k;
    b.innerHTML=`<b>${t.label}</b><span>${t.hint}</span>`;b.addEventListener('click',()=>addBlock(k));
    b.addEventListener('dragstart',e=>{if(e.dataTransfer)e.dataTransfer.setData('text/plain','new:'+k);});pal.appendChild(b);});
  const box=$('#stProg');
  box.addEventListener('input',e=>{if(!e.target.classList.contains('bnote-in'))onField(e);});
  box.addEventListener('change',e=>{if(!e.target.classList.contains('bnote-in'))onField(e);});
  box.addEventListener('keydown',e=>{if(!e.target.classList.contains('bnote-in'))return;
    if(e.key==='Enter'){e.preventDefault();commitNote(e.target,true);}else if(e.key==='Escape'){e.preventDefault();e.stopPropagation();commitNote(e.target,false);}});
  box.addEventListener('focusout',e=>{if(e.target.classList.contains('bnote-in')&&editingNote)commitNote(e.target,true);});
  box.addEventListener('click',e=>{
    const btn=e.target.closest('button[data-act]');if(btn){onAct(btn);return;}
    if(e.target.closest('input,label.chk,label.bf'))return;
    const blk=e.target.closest('.blk');const id=blk?blk.dataset.id:null;
    if(id!==sel){sel=id;stopPreview();render();}
  });
  box.addEventListener('dragstart',e=>{
    const grip=e.target.closest&&e.target.closest('.bgrip'),blk=grip&&grip.closest('.blk');
    if(!blk){e.preventDefault();return;}   // nothing else in a block starts a drag
    if(!e.dataTransfer)return;
    e.dataTransfer.setData('text/plain','move:'+blk.dataset.id);e.dataTransfer.effectAllowed='move';
    const r=blk.getBoundingClientRect();if(e.dataTransfer.setDragImage)e.dataTransfer.setDragImage(blk,e.clientX-r.left,e.clientY-r.top);
    blk.classList.add('dragging');});
  box.addEventListener('dragend',e=>{const blk=e.target.closest('.blk');if(blk)blk.classList.remove('dragging');clearDrop();});
  box.addEventListener('dragover',onDragOver);box.addEventListener('dragleave',e=>{if(!box.contains(e.relatedTarget))clearDrop();});
  box.addEventListener('drop',onDrop);
  $('#stSegs').addEventListener('click',e=>{const s=e.target.closest('.sseg');if(!s||!compiled)return;
    sel=s.dataset.id;preview={t:+s.dataset.t,playing:false};render();ui();});
  $('#stName').addEventListener('input',()=>{dirty=true;$('#stName').classList.add('dirty');});
  $('#stName').addEventListener('change',()=>{prog.name=$('#stName').value.trim()||prog.name;record(false);renderTabs();});
  $('#stSave').addEventListener('click',async()=>{try{await save();refreshList();}catch(e){note(e.message);toast(e.message,'bad');}});
  $('#stUndo').addEventListener('click',undo);$('#stRedo').addEventListener('click',redo);
  $('#stMore').addEventListener('click',e=>{e.stopPropagation();menu($('#stMenu').hidden);});
  document.addEventListener('click',e=>{if(!$('#stMenu').hidden&&!e.target.closest('#stMenu'))menu(false);});
  $('#stmDuplicate').addEventListener('click',()=>{menu(false);duplicateProgram();});
  $('#stmExport').addEventListener('click',()=>{menu(false);exportProgram();});
  $('#stmImport').addEventListener('click',()=>{menu(false);$('#stFile').click();});
  $('#stFile').addEventListener('change',e=>{const f=e.target.files[0];e.target.value='';if(f)importProgram(f);});
  $('#stmDelete').addEventListener('click',()=>{menu(false);deleteProgram();});
  $('#stPlay').addEventListener('click',()=>preview&&preview.playing?pause():play());
  $('#stStop').addEventListener('click',stopPreview);
  $('#stTime').addEventListener('input',e=>{if(!compiled)return;preview={t:+e.target.value,playing:false};ui();});
  $('#progEditBtn').addEventListener('click',async()=>{const id=selectedMotion();if(!id)return;setView('studio');if(id!==prog.id)await openProgram(id);});
  onView('studio',{enter:enterStudio,leave:leaveStudio});
  $('#btnToStudio').addEventListener('click',()=>setView('studio'));
  window.addEventListener('keydown',e=>{
    if(!S.studio)return;
    const typing=e.target.closest&&e.target.closest('input[type=text],input[type=number],textarea');
    const mod=e.ctrlKey||e.metaKey;
    if(mod&&!typing&&(e.key==='z'||e.key==='Z')){e.preventDefault();e.shiftKey?redo():undo();return;}
    if(mod&&!typing&&e.key==='y'){e.preventDefault();redo();return;}
    if(mod&&e.key==='s'){e.preventDefault();$('#stSave').click();return;}
    if(!sel||(e.target.closest&&e.target.closest('input,textarea')))return;
    if(e.key==='Delete'||e.key==='Backspace'){const w=find(sel);if(w){w.list.splice(w.i,1);sel=null;stopPreview();changed();e.preventDefault();}}
  });
  // unsaved changes: the browser asks before the page goes away
  window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();e.returnValue='';}});
  undoUI();
}
