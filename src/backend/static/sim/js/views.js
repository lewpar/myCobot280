/* The top bar's view switch: Arm (the main 3D view and the inspector), Studio (studio.js) and Workspace
   (workspace.js). Only the showing view's 3D is drawn each frame (main.js asks S.view). The one showing is
   remembered on this device (mycobot-view) and restoreView goes back to it. */
import {S} from './state.js';
import {$,uiGet,uiSet} from './util.js';

const VIEWS=['arm','studio','workspace'],hooks={};
/* enter/leave: called when a view comes up or goes away */
export function onView(name,h){hooks[name]=h;}
export function setView(v){
  if(!VIEWS.includes(v))v='arm';
  if(v===S.view)return;
  const was=S.view;S.view=v;S.studio=v==='studio';S.workspace=v==='workspace';
  VIEWS.forEach(n=>{document.body.classList.toggle(n,n===v&&n!=='arm');const b=$('#view'+n[0].toUpperCase()+n.slice(1));if(b)b.setAttribute('aria-pressed',n===v);});
  $('#studio').hidden=v!=='studio';$('#workspace').hidden=v!=='workspace';
  uiSet('mycobot-view',v==='arm'?null:v);
  if(hooks[was]&&hooks[was].leave)hooks[was].leave();
  if(hooks[v]&&hooks[v].enter)hooks[v].enter();
}
/* back where the user was last time */
export function restoreView(){const v=uiGet('mycobot-view');if(v&&v!=='arm')setView(v);}
export function initViews(){
  S.view='arm';
  VIEWS.forEach(n=>{const b=$('#view'+n[0].toUpperCase()+n.slice(1));if(b)b.addEventListener('click',()=>setView(n));});
}
