/* The connect screen the page opens on, and the "calibrate the arm?" question once it's connected.

   The screen takes the backend address and the password (copied into the connection menu's fields, which the
   rest of the page reads), then follows the link as it comes up: the socket opens (the backend is reachable),
   hello arrives (the password was accepted) and every servo reads back. Then it steps aside for the 3D scene
   and asks whether to calibrate. "Use the simulator without the arm" skips straight to the scene: the backend
   still solves the IK there (/ws/ik), so the password is still wanted. */
import {S} from './state.js';
import {connectArm,onArm} from './link.js';
import {openWizard} from './wizard.js';
import {solverReconnect} from './solve.js';
import {restoreView} from './views.js';
import {maybeTour} from './tour.js';
import {toast} from './toast.js';
import {$} from './util.js';

const SERVO_WAIT_MS=8000;   // not every servo answering after this long: offer to go on anyway
let phase='form',timer=null,started=0,asked=false;

const STEPS=['#lsReach','#lsAuth','#lsServos'];
function stepState(id,state,text){const li=$(id);li.dataset.state=state;if(text!==undefined)li.querySelector('small').textContent=text;}
function showForm(err){
  phase='form';clearTimeout(timer);
  $('#landForm').hidden=false;$('#landProgress').hidden=true;$('#landErr').textContent=err||'';
  $('#landing').classList.remove('connecting');
  if(err)$('#landPw').focus();
}
function fail(step,msg){
  if(phase!=='connecting')return;
  stepState(step,'bad');phase='failed';clearTimeout(timer);
  $('#landing').classList.remove('connecting');
  setTimeout(()=>{if(phase==='failed')showForm(msg);},900);
}
function go(){
  const url=$('#landUrl').value.trim(),pw=$('#landPw').value;
  if(!/^wss?:\/\//.test(url)){showForm('The address should look like ws://raspberrypi.local:8000/ws/arm.');return;}
  if(!pw){showForm('Enter the arm\'s password.');return;}
  $('#wsUrl').value=url;$('#wsPw').value=pw;$('#optRemember').checked=$('#landRemember').checked;
  solverReconnect();
  phase='connecting';started=performance.now();
  $('#landForm').hidden=true;$('#landProgress').hidden=false;$('#landContinue').hidden=true;
  $('#landing').classList.add('connecting');
  stepState('#lsReach','now','');stepState('#lsAuth','','');stepState('#lsServos','','');
  $('#landHost').textContent=url.replace(/^wss?:\/\//,'').replace(/\/ws\/arm\/?$/,'');
  clearTimeout(timer);timer=setTimeout(()=>fail('#lsReach','The backend didn\'t answer. Check it\'s running and the address is right.'),8000);
  connectArm();
}
function enter(){ // connected, or going on without the arm: into the 3D scene
  if(phase==='done')return;
  phase='done';clearTimeout(timer);
  const land=$('#landing');land.classList.add('leaving');
  setTimeout(()=>{land.hidden=true;land.classList.remove('leaving','connecting');S.landing=false;
    if(S.ws&&!asked){asked=true;askCalibrate();}else settled();},650);
}
function success(){
  if(phase!=='connecting')return;
  phase='entering';clearTimeout(timer);
  stepState('#lsServos','ok',S.armConfig&&S.armConfig.simulated?'simulated arm':'all six answering');
  toast(S.armConfig&&S.armConfig.simulated?'Connected to the simulated arm':'Connected to the arm','good');
  $('#landing').classList.add('connected');$('#landDone').hidden=false;
  setTimeout(enter,700);
}

/* ---------- "Calibrate the arm?" ---------- */
function askCalibrate(){
  const c=S.armConfig,range=S.armRange;
  let text,tag='';
  if(range){text=`${range.split(':')[0]}. Calibrating fixes that: it re-centres the servos and saves the zero pose.`;tag='Recommended';}
  else if(c&&c.calibrated===false){text='This arm hasn\'t been calibrated yet. It takes about three minutes, and the model shows you each step.';tag='Recommended';}
  else text='It was calibrated before. Calibrate again if the model and the real arm don\'t line up, or after changing a servo.';
  $('#calAskText').textContent=text;$('#calAskTag').textContent=tag;$('#calAskTag').hidden=!tag;
  $('#calAsk').hidden=false;$('#calAskGo').focus();
}
function closeAsk(){$('#calAsk').hidden=true;}
/* in the scene with nothing in the way: back where the user left off last time, and the tour the first time */
function settled(){restoreView();maybeTour();}

export function initLanding(){
  $('#landUrl').value=$('#wsUrl').value;$('#landPw').value=$('#wsPw').value;$('#landRemember').checked=$('#optRemember').checked;
  $('#landGo').addEventListener('click',go);
  ['#landUrl','#landPw'].forEach(id=>$(id).addEventListener('keydown',e=>{if(e.key==='Enter')go();}));
  $('#landSim').addEventListener('click',()=>{
    if($('#landPw').value){$('#wsPw').value=$('#landPw').value;solverReconnect();}
    enter();});
  $('#landCancel').addEventListener('click',()=>{if(S.ws)$('#btnWs').click();showForm('');});
  $('#landContinue').addEventListener('click',success);
  $('#calAskGo').addEventListener('click',()=>{closeAsk();openWizard();});
  $('#calAskSkip').addEventListener('click',()=>{closeAsk();settled();});
  window.addEventListener('mycobot:wizard-closed',()=>{if(!S.landing)maybeTour();});
  onArm(m=>{
    if(phase!=='connecting')return;
    if(m.type==='_open'){stepState('#lsReach','ok','');stepState('#lsAuth','now','');
      clearTimeout(timer);timer=setTimeout(()=>fail('#lsAuth','The backend didn\'t answer the login.'),8000);}
    else if(m.type==='hello'){stepState('#lsAuth','ok','');stepState('#lsServos','now','');clearTimeout(timer);
      timer=setTimeout(()=>{$('#landContinue').hidden=false;},SERVO_WAIT_MS);}
    else if(m.type==='error'&&m.code==='auth')fail('#lsAuth','The backend rejected that password.');
    else if(m.type==='error'&&m.code==='locked')fail('#lsAuth',m.message||'Too many wrong passwords. Wait a minute.');
    else if(m.type==='error'&&m.code==='no_arm')fail('#lsServos','The backend is running, but it has no arm on its serial port. Check the cable and power, or use the simulator without the arm.');
    else if(m.type==='_closed'){const at=STEPS.find(id=>$(id).dataset.state==='now')||'#lsReach';
      fail(at,at==='#lsReach'?'Could not reach the backend. Check it\'s running and the address is right.':(m.message||'The connection closed.'));}
    else if(m.type==='state'&&Array.isArray(m.angles)){
      const n=m.angles.filter(v=>typeof v==='number').length;
      if(n===6)success();
      else stepState('#lsServos','now',`${n} of 6 answering`);
    }
  });
}
