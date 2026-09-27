/* App chrome: inspector tabs, the panel toggle, foldable cards and their help notes, the popovers (connection,
   View), theme, camera views, the connection chip, the Stop button and the joint position overlay. */
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
    else{state='live';const who=S.armConfig&&S.armConfig.simulated?'Simulated arm':'Connected';text=who+(S.stopped?' · stopped':' · live');}}
  if(state+text===linkKey)return;linkKey=state+text;
  $('#linkChip').dataset.state=state;$('#linkText').textContent=text;
}

/* Popovers: one open at a time; a click outside or Esc closes it */
let openPop=null;
function setPop(btn,pop,on){
  if(!on&&openPop!==pop)return;
  if(on&&openPop&&openPop!==pop)setPop(openPop.btn,openPop,false);
  pop.hidden=!on;btn.setAttribute('aria-expanded',on);openPop=on?pop:null;if(on)pop.btn=btn;
}
export function showConn(){setPop($('#linkChip'),$('#connPop'),true);}

/* Cards: data-fold="open|closed" folds from the header (remembered per device), and a p.note.help inside
   is shown by an ⓘ button added to the header */
function initCards(){
  let folds={};try{folds=JSON.parse(uiGet('mycobot-fold'))||{};}catch(_){}
  document.querySelectorAll('.card').forEach(card=>{
    const head=card.querySelector('.card-head'),h2=head&&head.querySelector('h2');if(!h2)return;
    if(card.querySelector('.note.help')){
      const b=document.createElement('button');b.type='button';b.className='help-btn';b.textContent='i';
      b.title='What does this do?';b.setAttribute('aria-label','Explain '+h2.textContent);b.setAttribute('aria-expanded','false');
      b.addEventListener('click',e=>{e.stopPropagation();b.setAttribute('aria-expanded',card.classList.toggle('show-help'));});
      head.appendChild(b);}
    if(!card.dataset.fold||!card.id)return;
    h2.tabIndex=0;h2.setAttribute('role','button');
    const set=on=>{card.classList.toggle('folded',on);h2.setAttribute('aria-expanded',!on);};
    set(card.id in folds?folds[card.id]:card.dataset.fold==='closed');
    const toggle=()=>{const on=!card.classList.contains('folded');set(on);folds[card.id]=on;uiSet('mycobot-fold',JSON.stringify(folds));};
    head.addEventListener('click',e=>{if(!e.target.closest('button,input,label,a'))toggle();});
    h2.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();toggle();}});
  });
}

/* Joint overlay: one bar per joint (simulated servo, solved angle, real reading, zero), updated by the frame loop */
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
  initCards();
  [['#linkChip','#connPop'],['#viewBtn','#viewPop']].forEach(([b,p])=>{const btn=$(b),pop=$(p);
    btn.addEventListener('click',e=>{e.stopPropagation();setPop(btn,pop,pop.hidden);});});
  document.addEventListener('pointerdown',e=>{if(openPop&&!openPop.contains(e.target)&&!openPop.btn.contains(e.target))setPop(openPop.btn,openPop,false);});
  window.addEventListener('keydown',e=>{if(e.key==='Escape'&&openPop)setPop(openPop.btn,openPop,false);});
  { // the joint overlay: on unless turned off (off by default on narrow screens)
    const box=$('#optJoints'),saved=uiGet('mycobot-joints'),apply=()=>{$('#jointsHud').hidden=!box.checked;};
    box.checked=saved?saved==='1':innerWidth>760;apply();
    box.addEventListener('change',()=>{apply();uiSet('mycobot-joints',box.checked?'1':'0');});}
  setTheme(THEMES.includes(uiGet('mycobot-theme'))?uiGet('mycobot-theme'):'system');
  $('#themeBtn').addEventListener('click',()=>{const cur=document.documentElement.dataset.theme||'system';setTheme(THEMES[(THEMES.indexOf(cur)+1)%3]);});
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change',applyTheme);
  document.querySelectorAll('.view-tools button').forEach(b=>b.addEventListener('click',()=>{
    const d=V(...VIEWS[b.dataset.view]).normalize();orbit.target.set(0,0.2,0);
    camera.position.copy(orbit.target).addScaledVector(d,1.0);orbit.update();}));
  $('#btnStop').addEventListener('click',()=>setStopped(!S.stopped));
  window.addEventListener('keydown',e=>{if(e.key==='Escape')setStopped(true);});
}
