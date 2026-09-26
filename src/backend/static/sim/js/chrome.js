/* App chrome: inspector tabs, the panel toggle, theme, camera views, the connection chip, the Stop button
   and the Joints tab's position bars. */
import {JOINTS,N,LIM} from './kinematics.js';
import {camera,orbit,applyTheme} from './scene.js';
import {S} from './state.js';
import {setStopped} from './link.js';
import {$,V,uiGet,uiSet} from './util.js';

const panel=$('#panel');
const tabs=[...document.querySelectorAll('.tab')];
function showTab(name,focus){
  tabs.forEach(t=>{const on=t.id==='tabbtn-'+name;t.setAttribute('aria-selected',on);t.tabIndex=on?0:-1;
    $('#'+t.getAttribute('aria-controls')).hidden=!on;if(on&&focus)t.focus();});
  if(panel.classList.contains('collapsed'))$('#toggle').click();
  uiSet('mycobot-tab',name);
}
const THEMES=['system','light','dark'];
function setTheme(t){
  if(t==='system')delete document.documentElement.dataset.theme;else document.documentElement.dataset.theme=t;
  const b=$('#themeBtn');b.title='Theme: '+t;b.setAttribute('aria-label','Colour theme: '+t+'. Click to change.');
  uiSet('mycobot-theme',t==='system'?null:t);applyTheme();
}
const VIEWS={persp:[0.62,0.3,0.7],front:[1,0,0],side:[0,0,1],top:[0.001,1,0]};  // three.js world: base X, base Z (up), -base Y

let linkKey='';
export function updateLinkChip(){ // derived from the link state every frame; only touches the DOM when it changes
  let state='off',text='Offline';
  if(S.ws&&!S.measured){state='connecting';text='Connecting…';}
  else if(S.ws){const n=S.measured.filter(v=>v!==null).length;
    if(S.limp){state='warn';text='Hand-guide · torque off';}
    else if(n<N){state='warn';text=`Connected · ${n}/${N} servos`;}
    else{state='live';text=S.stopped?'Connected · stopped':'Connected · live';}}
  if(state+text===linkKey)return;linkKey=state+text;
  $('#linkChip').dataset.state=state;$('#linkText').textContent=text;
}

/* Joints tab: one bar per joint (simulated servo, solved angle, real reading, zero), updated by the frame loop */
export const jointUI=[];
export const pct=(i,a)=>((a-LIM[i][0])/(LIM[i][1]-LIM[i][0])*100).toFixed(2)+'%';

export function initChrome(){
  JOINTS.forEach(j=>{
    const row=document.createElement('div');row.className='joint';
    row.innerHTML=`<span class="name">${j.name}</span><div class="bar"><div class="zero"></div><div class="tgt"></div><div class="real"></div><div class="pos"></div></div><span class="val">0.0°</span>`;
    $('#joints').appendChild(row);
    jointUI.push({real:row.querySelector('.real'),tgt:row.querySelector('.tgt'),pos:row.querySelector('.pos'),val:row.querySelector('.val')});
    row.querySelector('.zero').style.left=((0-j.min)/(j.max-j.min)*100)+'%';
  });
  $('#toggle').addEventListener('click',e=>{const c=panel.classList.toggle('collapsed'),b=e.currentTarget;
    b.setAttribute('aria-expanded',!c);b.setAttribute('aria-label',c?'Show controls':'Hide controls');b.title=b.getAttribute('aria-label');
    b.firstElementChild.style.transform=c?'rotate(180deg)':'';});
  tabs.forEach((t,i)=>{
    t.addEventListener('click',()=>showTab(t.id.slice(7)));
    t.addEventListener('keydown',e=>{const d={ArrowRight:1,ArrowLeft:-1}[e.key];if(!d)return;e.preventDefault();
      showTab(tabs[(i+d+tabs.length)%tabs.length].id.slice(7),true);});
  });
  {const t=uiGet('mycobot-tab');showTab(tabs.some(b=>b.id==='tabbtn-'+t)?t:'motion');}
  $('#linkChip').addEventListener('click',()=>showTab('robot'));
  setTheme(THEMES.includes(uiGet('mycobot-theme'))?uiGet('mycobot-theme'):'system');
  $('#themeBtn').addEventListener('click',()=>{const cur=document.documentElement.dataset.theme||'system';setTheme(THEMES[(THEMES.indexOf(cur)+1)%3]);});
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change',applyTheme);
  document.querySelectorAll('.view-tools button').forEach(b=>b.addEventListener('click',()=>{
    const d=V(...VIEWS[b.dataset.view]).normalize();orbit.target.set(0,0.2,0);
    camera.position.copy(orbit.target).addScaledVector(d,1.0);orbit.update();}));
  $('#btnStop').addEventListener('click',()=>setStopped(!S.stopped));
  window.addEventListener('keydown',e=>{if(e.key==='Escape')setStopped(true);});
}
