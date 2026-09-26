/* ATOM tab: the 5x5 LED matrix on the end effector, over REST (/api/atom/*). Requests go one at a time;
   a 401/429 drops the rest of the queue so a drag can't trip the lockout. The 3D ATOM's LEDs mirror the
   panel (index row*5+x, seen from behind). */
import {atomDotMats} from './scene.js';
import {api} from './api.js';
import {recEvent} from './record.js';
import {$} from './util.js';

const atomPx=Array.from({length:25},()=>[0,0,0]);let atomBright=100;
const atomCells=[];
function atomNote(t){$('#atomNote').textContent=t;}
function hexRgb(h){const n=parseInt(h.slice(1),16);return[n>>16&255,n>>8&255,n&255];}
function atomDraw(k){
  const [r,g,b]=atomPx[k];atomCells[k].style.background=`rgb(${r},${g},${b})`;
  const m=atomDotMats[k],on=r+g+b>0;
  m.color.setRGB(on?r/255*0.4:0.05,on?g/255*0.4:0.05,on?b/255*0.4:0.05);m.emissive.setRGB(r/255,g/255,b/255);
  m.emissiveIntensity=on?0.3+0.9*atomBright/100:0;
}
let atomChain=Promise.resolve(),atomGen=0;
function atomQueue(fn){ // one request at a time, in order, so a fast drag can't flood the bus
  const gen=atomGen;
  atomChain=atomChain.then(()=>gen===atomGen?fn():null)
    .then(j=>{if(j)atomNote(j.acked===false?'Sent, but the ATOM didn\'t reply. Its LEDs may still have changed.':'');})
    .catch(e=>{
      if(e.halt)atomGen++;   // wrong password or locked out: drop the rest of the queue instead of piling up failures
      atomNote(e.message+(e.read?'':' The grid may not match the ATOM now; press "Read from the ATOM" once it\'s sorted.'));});
}
const atomPending=new Set();
function atomPaint(k){
  const c=hexRgb($('#atomColor').value);
  if(atomPx[k].every((v,i)=>v===c[i]))return;
  atomPx[k]=c;atomDraw(k);recEvent('pixel',[k%5,Math.floor(k/5),...c]);
  if(atomPending.has(k))return;   // already queued: it sends whatever colour the LED has by then
  atomPending.add(k);
  atomQueue(()=>{atomPending.delete(k);const [r,g,b]=atomPx[k];return api('POST','/atom/pixel',{x:k%5,y:Math.floor(k/5),r,g,b});});
}
function atomFill(c){
  recEvent('color',c.slice());
  for(let k=0;k<25;k++){atomPx[k]=c.slice();atomDraw(k);}
  atomQueue(()=>api('POST','/atom/color',{r:c[0],g:c[1],b:c[2]}));
}
/* an LED cue during simulated playback: shown on the panel and the 3D ATOM only */
export function ledApply([,kind,a]){
  if(kind==='color')for(let k=0;k<25;k++)atomPx[k]=a.slice();
  else if(kind==='pixel')atomPx[a[1]*5+a[0]]=a.slice(2);
  else{atomBright=a[0];$('#atomBri').value=a[0];$('#atomBriv').textContent=a[0]+'%';}
  for(let k=0;k<25;k++)atomDraw(k);
}

export function initAtom(){
  let painting=false;
  for(let k=0;k<25;k++){
    const b=document.createElement('button');b.type='button';b.setAttribute('aria-label',`LED column ${k%5+1}, row ${Math.floor(k/5)+1}`);
    b.addEventListener('pointerdown',e=>{e.preventDefault();painting=true;atomPaint(k);
      try{b.releasePointerCapture(e.pointerId);}catch(_){}});   // touch captures the pointer; release it so dragging reaches the other cells
    b.addEventListener('pointerenter',()=>{if(painting)atomPaint(k);});
    b.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();atomPaint(k);}});
    $('#atomGrid').appendChild(b);atomCells.push(b);
    b.style.background='#000';
  }
  window.addEventListener('pointerup',()=>{painting=false;});window.addEventListener('pointercancel',()=>{painting=false;});
  ['#ff0000','#ff7a00','#ffd000','#00c850','#00c8c8','#0050ff','#a000ff','#ffffff'].forEach(h=>{
    const b=document.createElement('button');b.type='button';b.style.background=h;b.setAttribute('aria-label','Paint colour '+h);b.title=h;
    b.addEventListener('click',()=>{$('#atomColor').value=h;});$('#atomPalette').appendChild(b);});
  $('#atomFill').addEventListener('click',()=>atomFill(hexRgb($('#atomColor').value)));
  $('#atomClear').addEventListener('click',()=>atomFill([0,0,0]));
  $('#atomRead').addEventListener('click',()=>atomQueue(async()=>{
    const j=await api('GET','/atom/state');
    if(Array.isArray(j.pixels))j.pixels.slice(0,25).forEach((p,k)=>{atomPx[k]=[p[0]|0,p[1]|0,p[2]|0];});
    if(typeof j.brightness==='number'){atomBright=Math.max(1,Math.round(j.brightness*100/128));$('#atomBri').value=atomBright;$('#atomBriv').textContent=atomBright+'%';}
    for(let k=0;k<25;k++)atomDraw(k);
    atomNote('Read the LEDs back from the ATOM.');
  }));
  let t=null;
  $('#atomBri').addEventListener('input',e=>{
    atomBright=+e.target.value;$('#atomBriv').textContent=atomBright+'%';for(let k=0;k<25;k++)if(atomPx[k].some(v=>v))atomDraw(k);
    clearTimeout(t);t=setTimeout(()=>{recEvent('brightness',[atomBright]);atomQueue(()=>api('POST','/atom/brightness',{percent:atomBright}));},250);});
}
