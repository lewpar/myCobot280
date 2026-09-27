/* Toasts: short messages that slide in at the bottom of the screen and go away by themselves (saves, playback
   starting and ending, errors). The notes in the cards still say the same, for anyone looking there. */
import {$} from './util.js';

const ICON={good:'✓',bad:'!',info:'i'};
/* kind: 'info' | 'good' | 'bad'; ms: how long it stays (errors stay longer) */
export function toast(msg,kind='info',ms){
  const box=$('#toasts');if(!box||!msg)return;
  // the same message twice in a row: bump the one showing instead of stacking a copy
  const last=box.lastElementChild;
  if(last&&last.dataset.msg===msg&&!last.classList.contains('out')){clearTimeout(last._t);last._t=setTimeout(()=>hide(last),last._ms);return;}
  const t=document.createElement('div');t.className='toast '+kind;t.setAttribute('role',kind==='bad'?'alert':'status');t.dataset.msg=msg;
  t.innerHTML=`<span class="ti" aria-hidden="true">${ICON[kind]||'i'}</span><span class="tm"></span><button class="tx" aria-label="Dismiss">✕</button>`;
  t.querySelector('.tm').textContent=msg;t.querySelector('.tx').addEventListener('click',()=>hide(t));
  box.appendChild(t);
  while(box.children.length>4)box.firstElementChild.remove();
  t._ms=ms||(kind==='bad'?7000:3500);t._t=setTimeout(()=>hide(t),t._ms);
}
function hide(t){if(t.classList.contains('out'))return;t.classList.add('out');setTimeout(()=>t.remove(),250);}
