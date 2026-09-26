/* Small DOM and formatting helpers shared by the page's modules. */
import * as THREE from 'three';

export const $=s=>document.querySelector(s);
export const cssVar=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
export const V=(x,y,z)=>new THREE.Vector3(x,y,z);
export const mm=v=>Math.round(v*1000);
export const r2=v=>+v.toFixed(2);
export const fmtDur=s=>s>=60?`${Math.floor(s/60)}:${String(Math.floor(s%60)).padStart(2,'0')}`:s.toFixed(1)+' s';
// per-device UI preferences; storage may be unavailable (private windows), so failures are ignored
export const uiGet=k=>{try{return localStorage.getItem(k);}catch(_){return null;}};
export const uiSet=(k,v)=>{try{v==null?localStorage.removeItem(k):localStorage.setItem(k,v);}catch(_){}};

export function item(name,meta,selected,onClick,onDelete){ // a list row: [ name ........ meta ][x]
  const b=document.createElement('button');b.type='button';b.title=name;
  if(selected!==null){b.setAttribute('role','option');b.setAttribute('aria-selected',selected);}
  const n=document.createElement('span');n.className='n';n.textContent=name;
  const d=document.createElement('span');d.className='d';d.textContent=meta;b.append(n,d);
  b.addEventListener('click',onClick);
  if(!onDelete)return b;
  const w=document.createElement('div');w.className='item';
  const x=document.createElement('button');x.type='button';x.className='x';x.textContent='×';x.title='Remove '+name;x.setAttribute('aria-label','Remove '+name);
  x.addEventListener('click',onDelete);w.append(b,x);return w;
}
