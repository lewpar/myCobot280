/* Record tab: samples the pose at 10 Hz (the real arm when it reads back, otherwise the sim) into
   [t, deg x6] frames, logs ATOM panel changes as LED cues, trims still ends, and saves via /api/recordings.
   Waypoints make a minimum-jerk recording through poses added one at a time. */
import * as THREE from 'three';
import {checkFrames} from './collision.js';
import {S,haveRealNow,recPose,tcpText} from './state.js';
import {setLimp} from './link.js';
import {goPose} from './motion.js';
import {busyPlaying,libRefresh,selectItem} from './play.js';
import {api} from './api.js';
import {$,item,r2,fmtDur} from './util.js';

const REC_DT=100,REC_MAX=36000;
let rec=null,take=null;                    // rec: while recording; take: {frames, events} waiting to be saved
export const isRecording=()=>!!rec;
const recNote=t=>{$('#recNote').textContent=t;};

export function recEvent(kind,args){if(rec)rec.events.push([+((performance.now()-rec.t0)/1000).toFixed(3),kind,args]);}
export function recTick(now){ // called every frame
  if(!rec||now-rec.last<REC_DT)return;
  rec.last=now;rec.frames.push([+((now-rec.t0)/1000).toFixed(3),...recPose().map(r2)]);
  $('#recMeta').textContent=`${fmtDur((now-rec.t0)/1000)} · ${rec.src}`;
  if(rec.frames.length>=REC_MAX)recStop();
}
function recTrim(f,ev){ // drop the still time before the first and after the last movement; cues move with the frames
  const moved=(a,b)=>a.some((v,j)=>j>0&&Math.abs(v-b[j])>0.5);
  let s=0,e=f.length-1;
  while(s<e&&!moved(f[s+1],f[0]))s++;
  while(e>s&&!moved(f[e-1],f[f.length-1]))e--;
  const t0=f[s][0],t1=f[e][0];
  return{frames:f.slice(s,e+1).map(r=>[+(r[0]-t0).toFixed(3),...r.slice(1)]),
    events:ev.map(x=>[+(THREE.MathUtils.clamp(x[0],t0,t1)-t0).toFixed(3),x[1],x[2]])};
}
export function recUI(){
  const b=$('#recBtn');b.setAttribute('aria-pressed',!!rec);$('#recBtnText').textContent=rec?'Stop recording':'Record';b.disabled=busyPlaying();
  $('#wpMake').disabled=wps.length<2||!!rec;
}
function recStart(){
  if(busyPlaying())return;
  take=null;$('#recSave').hidden=true;
  const now=performance.now();
  rec={t0:now,last:-1e9,frames:[],events:[],src:haveRealNow()?'real arm':'simulation'};recTick(now);
  recNote(rec.src==='real arm'?'Recording the real arm. Move it, then press Stop recording.':'Not connected, so this records the simulated arm. Move the target, then press Stop recording.');
  recUI();
}
function recStop(){
  const r=rec;rec=null;$('#recMeta').textContent='10 samples/s';
  const t=recTrim(r.frames,r.events);
  if(t.frames.length<2){recNote('Nothing moved, so there is nothing to save.');recUI();return;}
  offerSave(t.frames,t.events,`Recorded ${fmtDur(t.frames[t.frames.length-1][0])} (${t.frames.length} samples). Give it a name and save it.`);
}
function offerSave(frames,events,msg){
  take={frames,events};$('#recSave').hidden=false;$('#recName').value='';$('#recName').focus();
  $('#recSaveMeta').textContent=fmtDur(frames[frames.length-1][0])+(events.length?` · ${events.length} LED cue${events.length>1?'s':''}`:'');
  recNote(msg);recUI();
}
async function recSave(){
  const name=$('#recName').value.trim();
  if(!name){recNote('Type a name for the recording first.');$('#recName').focus();return;}
  $('#recSaveBtn').disabled=true;
  try{
    const r=await api('POST','/recordings',{name,frames:take.frames,events:take.events,return_zero:$('#recZero').checked});
    take=null;$('#recSave').hidden=true;recNote(`Saved "${r.name}". Find it in the Play tab.`);
    await libRefresh();selectItem({kind:'rec',id:r.id});
  }catch(e){recNote(e.message+' The recording is kept here; try saving again.');}
  finally{$('#recSaveBtn').disabled=false;}
}

/* ---- waypoints: a smooth (minimum-jerk) move between poses you add one at a time ---- */
const wps=[];
function wpRender(){
  const box=$('#wpList');box.textContent='';
  wps.forEach((q,k)=>box.appendChild(item(`Point ${k+1}`,tcpText(q),null,()=>goPose(q,$('#recNote')),()=>{wps.splice(k,1);wpRender();})));
  $('#wpMeta').textContent=`${wps.length} point${wps.length===1?'':'s'}`;recUI();
}
function wpFrames(){
  const v=+$('#wpSpeed').value,pause=$('#wpPause').value/10,out=[[0,...wps[0]]];let t=0;
  for(let i=1;i<wps.length;i++){
    const a=wps[i-1],b=wps[i],dur=Math.max(0.3,1.875*Math.max(...a.map((x,j)=>Math.abs(b[j]-x)))/v),n=Math.max(2,Math.ceil(dur*10));
    for(let k=1;k<=n;k++){const u=k/n,m=u*u*u*(10-15*u+6*u*u);out.push([+(t+dur*u).toFixed(3),...a.map((x,j)=>r2(x+(b[j]-x)*m))]);}
    t+=dur;
    if(pause>0&&i<wps.length-1){t+=pause;out.push([+t.toFixed(3),...b]);}
  }
  return out;
}
function wpSliders(){$('#wpSpeedv').textContent=$('#wpSpeed').value+'°/s';$('#wpPausev').textContent=($('#wpPause').value/10).toFixed(1)+' s';}

export function initRecord(){
  $('#recBtn').addEventListener('click',()=>rec?recStop():recStart());
  $('#recLimp').addEventListener('click',()=>setLimp(!S.limp));
  $('#recSaveBtn').addEventListener('click',recSave);
  $('#recName').addEventListener('keydown',e=>{if(e.key==='Enter')recSave();});
  $('#recDiscard').addEventListener('click',()=>{take=null;$('#recSave').hidden=true;recNote('Discarded.');});
  $('#wpSpeed').addEventListener('input',wpSliders);$('#wpPause').addEventListener('input',wpSliders);wpSliders();
  $('#wpAdd').addEventListener('click',()=>{wps.push(recPose().map(r2));wpRender();recNote(`Added point ${wps.length}${haveRealNow()?' from the real arm':' from the simulation'}.`);});
  $('#wpClear').addEventListener('click',()=>{if(wps.length&&confirm('Clear every waypoint?')){wps.length=0;wpRender();}});
  $('#wpMake').addEventListener('click',()=>{
    const f=wpFrames(),bad=checkFrames(f);
    if(bad){recNote(`That path isn't clear at ${bad[0].toFixed(1)} s: ${bad[1]}. Move or remove a point.`);return;}
    offerSave(f,[],`Made a ${fmtDur(f[f.length-1][0])} recording through ${wps.length} points. Give it a name and save it.`);
  });
  wpRender();
}
