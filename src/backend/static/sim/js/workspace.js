/* Workspace: a small CAD-style editor for the things around the arm (boxes, cylinders, spheres) it must keep
   clear of. They're part of the work area (collision.js area.obstacles, arm_model's too), so every check, the
   solver, the route planner and playback avoid them, and they're drawn in every 3D view.

   Add a shape from the palette (it lands on the table in front of the arm), select it by clicking it in the view
   or the list, and change it with the handles (move W, rotate E, size R; snapping to 1 cm and 15° unless it's
   off) or the numbers in its panel. Every change applies at once (setObstacles) and is saved to the backend a
   moment later (PUT /api/obstacles, which works without the arm too). Undo/redo covers every change; a drag is
   one step. The arm is shown in a reference pose (the real one when connected, else zero) and anything it
   touches there, or anything in the arm's base (which the checks can't avoid), is flagged. */
import * as THREE from 'three';
import {DEG,N} from './kinematics.js';
import {setObstacles,checkPose,prepObstacle,obstacleDistance,COLLISION} from './collision.js';
import {setObstacleMeshes,makeWorkView,setWorkGizmo,pickObstacle,renderWorkView} from './scene.js';
import {S,haveRealNow} from './state.js';
import {onView} from './views.js';
import {api} from './api.js';
import {toast} from './toast.js';
import {$} from './util.js';

const COLORS=['#8a94a6','#d98a00','#3b82c4','#23845a','#c8402c','#7c5cd6','#d04d8e','#1f9a8a'];
const SHAPES={box:{label:'Box',size:[100,100,100],dims:[['W','x'],['D','y'],['H','z']]},
  cylinder:{label:'Cylinder',size:[80,80,120],dims:[['Diameter','x'],['Height','z']]},
  sphere:{label:'Sphere',size:[80,80,80],dims:[['Diameter','x']]}};
const AX={x:0,y:1,z:2},HIST_MAX=100,COALESCE_MS=700,SAVE_MS=600;
let obs=[],sel=null,mode='translate',snap=true,view=null,saveTimer=null,saving=false,saveErr='',lastSaved='[]';
let hist=[],fut=[],lastSnap='[]',lastEdit=0,hits=new Set(),inBase=new Set(),refPose=new Array(N).fill(0);

const newId=()=>'o'+Math.random().toString(36).slice(2,9);
const find=id=>obs.find(o=>o.id===id)||null;
const r1=v=>Math.round(v*10)/10;
const esc=t=>String(t).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
function nextName(shape){const base=SHAPES[shape].label;let k=1;while(obs.some(o=>o.name===`${base} ${k}`))k++;return `${base} ${k}`;}

/* ---------- changes: apply now, undo step, save soon ---------- */
const snapOf=()=>JSON.stringify(obs);
function changed(continuous=false){
  const s=snapOf();
  if(s!==lastSnap){
    const now=performance.now();
    if(!(continuous&&now-lastEdit<COALESCE_MS&&hist.length)){hist.push(lastSnap);if(hist.length>HIST_MAX)hist.shift();}
    lastEdit=continuous?now:0;fut=[];lastSnap=s;
  }
  apply();scheduleSave();ui();
}
function apply(){setObstacles(obs.map(o=>({...o,pos:[...o.pos],size:[...o.size],rot:[...o.rot]})));check();}
function restore(s){obs=JSON.parse(s);lastSnap=s;lastEdit=0;if(sel&&!find(sel))sel=null;apply();scheduleSave();render();}
function undo(){if(!hist.length)return;fut.push(snapOf());restore(hist.pop());}
function redo(){if(!fut.length)return;hist.push(snapOf());restore(fut.pop());}
function scheduleSave(){clearTimeout(saveTimer);saveTimer=setTimeout(save,SAVE_MS);status();}
async function save(){
  const body=snapOf();if(body===lastSaved){status();return;}
  if(saving){scheduleSave();return;}
  saving=true;status();
  try{await api('PUT','/obstacles',{obstacles:obs});lastSaved=body;saveErr='';}
  catch(e){saveErr=e.message;toast('Obstacles not saved: '+e.message,'bad');}
  saving=false;status();
}
function status(){
  const el=$('#wsState');if(!el)return;
  const pending=snapOf()!==lastSaved;
  el.textContent=saving?'Saving…':saveErr&&pending?'Not saved':pending?'Unsaved changes':'Saved';
  el.className='ws-state'+(saveErr&&pending?' bad':pending?' dirty':'');el.title=saveErr||'';
}
/* the arm's reference pose, and what it touches: the obstacles it hits there, and any that overlap its base */
function check(){
  hits=new Set();inBase=new Set();
  const why=checkPose(refPose);
  if(why)for(const o of obs)if(why.endsWith(' '+o.name))hits.add(o.id);
  for(const o of obs){ // the base and the column can't move out of the way: sample them
    const p=prepObstacle(o),C=COLLISION;
    for(let z=0.01;z<=0.19;z+=0.02){const r=z<C.BASE_TOP?C.BASE_R:C.COLUMN_R;
      for(let a=0;a<Math.PI*2;a+=Math.PI/6){if(obstacleDistance(new THREE.Vector3(Math.cos(a)*r*0.7,Math.sin(a)*r*0.7,z),p)<r*0.3){inBase.add(o.id);break;}}
      if(inBase.has(o.id))break;}
  }
  drawMeshes();
  renderWarn();
}
/* the meshes: the selected one and the ones in the way lit only in the Workspace, plain everywhere else */
function drawMeshes(){setObstacleMeshes(obs,S.workspace?sel:null,S.workspace?new Set([...hits,...inBase]):new Set());}

/* ---------- adding, selecting, editing ---------- */
function add(shape){
  const size=[...SHAPES[shape].size],o={id:newId(),name:nextName(shape),shape,pos:[230,0,size[2]/2],size,rot:[0,0,0],
    color:COLORS[(obs.length+1)%COLORS.length]};
  obs.push(o);sel=o.id;changed();render();toast(`Added ${o.name}`);
}
function select(id){sel=id;render();}
function remove(id){const o=find(id);if(!o)return;obs=obs.filter(x=>x.id!==id);if(sel===id)sel=null;changed();render();toast(`Deleted ${o.name}`);}
function duplicate(id){
  const o=find(id);if(!o)return;
  const c={...JSON.parse(JSON.stringify(o)),id:newId(),name:nextName(o.shape)};c.pos[0]=Math.min(1000,c.pos[0]+30);c.pos[1]=Math.min(1000,c.pos[1]+30);
  obs.push(c);sel=c.id;changed();render();
}
function onTable(id){ // sit it on the table: its lowest point at z = 0 (rotated shapes included)
  const o=find(id);if(!o)return;
  const m=new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(o.rot[0]*DEG,o.rot[1]*DEG,o.rot[2]*DEG));
  const h=[o.size[0]/2,(o.shape==='box'?o.size[1]:o.size[0])/2,(o.shape==='sphere'?o.size[0]:o.size[2])/2];
  let low=0;
  if(o.shape==='sphere')low=h[0];
  else if(o.shape==='box')for(const sx of [-1,1])for(const sy of [-1,1])for(const sz of [-1,1])
    low=Math.max(low,-new THREE.Vector3(sx*h[0],sy*h[1],sz*h[2]).applyMatrix4(m).z);
  else{const ax=new THREE.Vector3(0,0,1).applyMatrix4(m);low=Math.abs(ax.z)*h[2]+Math.sqrt(Math.max(0,1-ax.z*ax.z))*h[0];}
  o.pos[2]=r1(low);changed();renderPanel();
}
/* the handles moved the selected mesh: read its transform back (mm, degrees; a cylinder stays round) */
function fromMesh(m){
  const o=find(m.userData.id);if(!o)return;
  const p=m.position,r=m.rotation,s=m.scale;
  o.pos=[p.x,p.y,p.z].map(v=>Math.max(-1000,Math.min(1000,r1(v*1000))));
  o.rot=[r.x,r.y,r.z].map(v=>Math.round(v/DEG*2)/2);
  const mm=[s.x,s.y,s.z].map(v=>Math.max(5,Math.min(1000,r1(Math.abs(v)*1000))));
  if(o.shape==='box')o.size=mm;
  else if(o.shape==='cylinder'){const d=Math.abs(mm[0]-o.size[0])>0.05?mm[0]:mm[1];o.size=[d,d,mm[2]];}
  else{const d=[0,1,2].map(k=>mm[k]).find((v,k)=>Math.abs(v-o.size[k])>0.05)||mm[0];o.size=[d,d,d];}
  changed(true);renderPanel(true);
}

/* ---------- the panel ---------- */
function render(){renderList();renderPanel();ui();}
function renderList(){
  const box=$('#wsList');box.textContent='';
  if(!obs.length){box.innerHTML='<p class="note" style="margin:6px 2px">Nothing yet. Add a shape above: it lands on the table in front of the arm.</p>';return;}
  obs.forEach(o=>{const b=document.createElement('button');b.type='button';b.className='ws-item'+(o.id===sel?' on':'');b.dataset.id=o.id;
    const flag=inBase.has(o.id)?'<span class="ws-flag bad" title="In the arm\'s base">!</span>':hits.has(o.id)?'<span class="ws-flag" title="The arm touches it in the pose shown">!</span>':'';
    b.innerHTML=`<span class="ws-sw" style="background:${o.color}"></span><span class="ws-ic ws-${o.shape}" aria-hidden="true"></span><b>${esc(o.name)}</b>${flag}
      <span class="ws-dim">${o.shape==='box'?o.size.join(' × '):o.shape==='cylinder'?`⌀${o.size[0]} × ${o.size[2]}`:`⌀${o.size[0]}`} mm</span>`;
    b.addEventListener('click',()=>select(o.id));box.appendChild(b);});
}
function numField(label,f,k,val,step,min,max,unit){
  return `<label class="ws-num"><span>${label}</span><input type="number" data-f="${f}" data-k="${k}" step="${step}" min="${min}" max="${max}" value="${val}"><i>${unit}</i></label>`;
}
function renderPanel(keepFocus){
  const box=$('#wsProps'),o=sel&&find(sel);
  if(!o){box.innerHTML='<p class="note ws-empty">Select a shape to change it: click it in the view or in the list.</p>';return;}
  if(keepFocus&&box.dataset.id===o.id){ // just the numbers (during a drag): don't rebuild under the pointer
    box.querySelectorAll('input[data-f]').forEach(i=>{if(document.activeElement===i)return;const f=i.dataset.f,k=+i.dataset.k;
      if(f==='pos'||f==='rot'||f==='size')i.value=o[f][k];});return;}
  box.dataset.id=o.id;
  box.innerHTML=`<div class="ws-row"><input type="text" id="wsName" maxlength="40" value="${esc(o.name)}" aria-label="Name" spellcheck="false"></div>
    <div class="ws-colors">${COLORS.map(c=>`<button type="button" class="ws-color${c===o.color?' on':''}" data-color="${c}" style="background:${c}" aria-label="Colour ${c}"></button>`).join('')}</div>
    <div class="ws-group"><b>Position</b>${['x','y','z'].map((c,k)=>numField(c.toUpperCase(),'pos',k,o.pos[k],1,-1000,1000,'mm')).join('')}</div>
    <div class="ws-group"><b>Size</b>${SHAPES[o.shape].dims.map(([l,a])=>numField(l,'size',AX[a],o.size[AX[a]],1,5,1000,'mm')).join('')}</div>
    <div class="ws-group"><b>Rotation</b>${['x','y','z'].map((c,k)=>numField(c.toUpperCase(),'rot',k,o.rot[k],15,-360,360,'°')).join('')}</div>
    <div id="wsWarn"></div>
    <div class="ws-btns"><button type="button" data-act="table" title="Put its lowest point on the table">On the table</button>
      <button type="button" data-act="dup" title="Duplicate (Ctrl+D)">Duplicate</button><button type="button" data-act="del" class="danger" title="Delete (Del)">Delete</button></div>`;
  renderWarn();
}
function renderWarn(){ // under the selected shape's numbers: whether it's in the way
  const el=$('#wsWarn'),o=sel&&find(sel);if(!el)return;
  el.innerHTML=!o?'':inBase.has(o.id)?'<p class="callout warn">It\'s in the arm\'s base, which can\'t move out of its way: move it clear.</p>':
    hits.has(o.id)?'<p class="callout warn">The arm touches it in the pose shown. Moves that go through it are refused.</p>':'';
}
function onProp(e){
  const o=sel&&find(sel),el=e.target;if(!o)return;
  if(el.id==='wsName'){o.name=el.value.trim().slice(0,40)||SHAPES[o.shape].label;changed(true);renderList();return;}
  const f=el.dataset.f;if(!f)return;
  const k=+el.dataset.k,v=+el.value;if(!Number.isFinite(v))return;
  if(f==='pos')o.pos[k]=Math.max(-1000,Math.min(1000,v));
  else if(f==='rot')o.rot[k]=Math.max(-360,Math.min(360,v));
  else if(f==='size'){const d=Math.max(5,Math.min(1000,v));
    if(o.shape==='box')o.size[k]=d;else if(o.shape==='cylinder'){if(k===2)o.size[2]=d;else o.size=[d,d,o.size[2]];}else o.size=[d,d,d];}
  changed(true);renderList();
}
function ui(){
  $('#wsUndo').disabled=!hist.length;$('#wsRedo').disabled=!fut.length;
  ['translate','rotate','scale'].forEach(m=>$('#wsMode-'+m).setAttribute('aria-pressed',m===mode));
  $('#wsSnap').setAttribute('aria-pressed',snap);
  const flagged=[...new Set([...hits,...inBase])].length;
  $('#wsCount').textContent=obs.length?`${obs.length} shape${obs.length>1?'s':''}${flagged?` · ${flagged} in the way`:''}`:'';
  status();
}

/* ---------- the view ---------- */
function enter(){
  if(!view){
    view=makeWorkView($('#wsStage'));
    view.onChange=fromMesh;view.onDrag=on=>{if(!on){lastEdit=0;check();renderList();}};
    const cv=view.renderer.domElement;let down=null;
    cv.addEventListener('pointerdown',e=>{down={x:e.clientX,y:e.clientY};});
    cv.addEventListener('pointerup',e=>{ // a click (not an orbit or a handle drag) selects what's under it
      if(!down||view.dragging||Math.hypot(e.clientX-down.x,e.clientY-down.y)>4){down=null;return;}
      down=null;const id=pickObstacle(e.clientX,e.clientY);if(id!==sel)select(id);});
  }else view.fit();
  if(snapOf()===lastSaved)load();
  render();
}
/* Called by main.js every frame while the Workspace shows. */
export function workspaceFrame(){
  const q=haveRealNow()?S.measured.map(v=>v*DEG):new Array(N).fill(0);
  if(q.some((v,i)=>Math.abs(v-refPose[i])>1e-4)){refPose=q;check();}
  drawMeshes();
  setWorkGizmo(sel,mode,snap);
  renderWorkView(refPose);
}
/* obstacles from the backend (on connect, or changed elsewhere): taken unless there are unsaved edits here */
export function obstaclesFromArm(list){
  const s=JSON.stringify(list||[]);
  if(s===lastSaved&&s===snapOf())return;
  if(snapOf()!==lastSaved)return;   // editing here: this page's version wins, and is saved shortly
  obs=JSON.parse(s);lastSaved=s;lastSnap=s;hist=[];fut=[];if(sel&&!find(sel))sel=null;apply();if(S.workspace)render();
}
async function load(){ // what the backend has saved (no arm needed)
  try{const r=await api('GET','/obstacles');obstaclesFromArm(r.obstacles);}catch(_){}
}

export function initWorkspace(){
  const pal=$('#wsPalette');
  Object.entries(SHAPES).forEach(([k,s])=>{const b=document.createElement('button');b.type='button';b.className='ws-pal';b.dataset.shape=k;
    b.innerHTML=`<span class="ws-ic ws-${k}" aria-hidden="true"></span><b>${s.label}</b>`;b.addEventListener('click',()=>add(k));pal.appendChild(b);});
  $('#wsProps').addEventListener('input',onProp);
  $('#wsProps').addEventListener('change',()=>{lastEdit=0;check();renderList();ui();});
  $('#wsProps').addEventListener('click',e=>{
    const c=e.target.closest('[data-color]');if(c&&sel){find(sel).color=c.dataset.color;changed();renderPanel();renderList();return;}
    const b=e.target.closest('[data-act]');if(!b||!sel)return;
    ({table:()=>onTable(sel),dup:()=>duplicate(sel),del:()=>remove(sel)})[b.dataset.act]();
  });
  ['translate','rotate','scale'].forEach(m=>$('#wsMode-'+m).addEventListener('click',()=>{mode=m;ui();}));
  $('#wsSnap').addEventListener('click',()=>{snap=!snap;ui();});
  $('#wsUndo').addEventListener('click',undo);$('#wsRedo').addEventListener('click',redo);
  window.addEventListener('keydown',e=>{
    if(!S.workspace)return;
    const typing=e.target.closest&&e.target.closest('input,textarea'),mod=e.ctrlKey||e.metaKey;
    if(mod&&!typing&&(e.key==='z'||e.key==='Z')){e.preventDefault();e.shiftKey?redo():undo();return;}
    if(mod&&!typing&&e.key==='y'){e.preventDefault();redo();return;}
    if(mod&&e.key==='d'&&sel){e.preventDefault();duplicate(sel);return;}
    if(typing||mod)return;
    if(e.key==='w'||e.key==='W'){mode='translate';ui();}else if(e.key==='e'||e.key==='E'){mode='rotate';ui();}
    else if(e.key==='r'||e.key==='R'){mode='scale';ui();}
    else if((e.key==='Delete'||e.key==='Backspace')&&sel){e.preventDefault();remove(sel);}
  });
  onView('workspace',{enter,leave:()=>{setWorkGizmo(null);drawMeshes();if(snapOf()!==lastSaved){clearTimeout(saveTimer);save();}}});
  window.addEventListener('beforeunload',e=>{if(snapOf()!==lastSaved){e.preventDefault();e.returnValue='';}});
  setTimeout(load,0);   // (needs the password: tried again when the Workspace opens)
  ui();
}
