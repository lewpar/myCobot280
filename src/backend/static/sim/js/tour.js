/* The first-run tour: a few spotlighted stops round the page (the views, the link, Stop, the tabs, the 3D view,
   Setup). It starts once, the first time the scene opens on this device (after the calibration question or the
   wizard, so it never covers them), and again from the ? button in the top bar. */
import {S} from './state.js';
import {$,uiGet,uiSet} from './util.js';

const STOPS=[
  {el:()=>$('.view-switch'),title:'Three views',text:'Arm drives the real arm: move it, record, play. Studio is a sandbox where you build motions from blocks. Workspace is where you put in the things around the arm, so it keeps clear of them.'},
  {el:()=>$('#linkChip'),title:'The connection',text:'Shows whether the arm is connected. Click it for the backend address, the password, or to disconnect.'},
  {el:()=>$('#btnStop'),title:'Stop',text:'Stops the arm at once, from anywhere. So does the Esc key. Press it again to resume.'},
  {el:()=>document.querySelector('.tab')&&document.querySelector('.tab').parentElement,title:'What you can do',
    text:'Move the target and jog the joints, record motions, play recordings and Studio motions, set the ATOM\'s LEDs, and set up the arm.'},
  {el:()=>$('#stage'),title:'The 3D view',text:'Drag the orange target, or click the floor, to send the arm there. The green line is the real arm as it reads back; the blue one is where it\'s going.'},
  {el:()=>$('#tabbtn-setup'),title:'Setup',text:'The attachment on the flange, the work area, the stall guard, and calibration, whenever the model and the arm stop lining up.'},
];
let k=-1;
function place(){
  const stop=STOPS[k],el=stop&&stop.el();if(!el){end();return;}
  const r=el.getBoundingClientRect(),pad=6,spot=$('#tourSpot'),bub=$('#tourBubble');
  Object.assign(spot.style,{left:r.left-pad+'px',top:r.top-pad+'px',width:r.width+pad*2+'px',height:r.height+pad*2+'px'});
  $('#tourStep').textContent=`${k+1} of ${STOPS.length}`;$('#tourTitle').textContent=stop.title;$('#tourText').textContent=stop.text;
  $('#tourBack').disabled=k===0;$('#tourNext').textContent=k===STOPS.length-1?'Done':'Next';
  // below the spot if there's room, else above; kept on screen
  const bw=Math.min(320,innerWidth-24),bh=bub.offsetHeight||170;
  let top=r.bottom+14;if(top+bh>innerHeight-12)top=Math.max(12,r.top-bh-14);
  if(r.height>innerHeight*0.5)top=Math.max(12,r.top+r.height/2-bh/2);   // a big one (the 3D view): in the middle
  const left=Math.max(12,Math.min(innerWidth-bw-12,r.left+r.width/2-bw/2));
  Object.assign(bub.style,{top:top+'px',left:left+'px',width:bw+'px'});
}
export function startTour(){if(S.wizard||S.landing||S.view!=='arm')return;k=0;$('#tour').hidden=false;place();$('#tourNext').focus();}
function end(){k=-1;$('#tour').hidden=true;uiSet('mycobot-tour','done');}
/* the first time only */
export function maybeTour(){if(!uiGet('mycobot-tour'))setTimeout(startTour,500);}

export function initTour(){
  $('#tourNext').addEventListener('click',()=>{if(k>=STOPS.length-1)end();else{k++;place();}});
  $('#tourBack').addEventListener('click',()=>{if(k>0){k--;place();}});
  $('#tourSkip').addEventListener('click',end);
  $('#btnTour').addEventListener('click',startTour);
  window.addEventListener('resize',()=>{if(k>=0)place();});
}
