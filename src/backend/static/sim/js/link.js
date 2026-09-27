/* Real arm link: the WebSocket to the backend's /ws/arm (protocol 2, see ik_link.py), plus the controls
   that act on the arm through it: Stop/Resume, hand-guide mode (top bar) and calibration (Setup tab).
   The backend sends hello, then config (now and on every change) and state about 10 times a second; the
   page streams qCmd as goals, carrying the epoch of the state it last re-read the pose from. */
import {DEG,JOINTS,N,LIM,URDF_LIM,ATTACHMENTS,clampJ} from './kinematics.js';
import {area} from './collision.js';
import {S,qIK,qCmd,servo,setDemo,targetFromPose,haveRealNow} from './state.js';
import {attachment,setAttachment,setArea} from './settings.js';
import {spd,acc,poseRefresh} from './motion.js';
import {endPlay,playNote,playUI,libRefresh} from './play.js';
import {storePw,forgetPw} from './api.js';
import {showConn} from './chrome.js';
import {$} from './util.js';

const WS_PROTOCOL=2,WS_FATAL=['auth','locked','no_arm'];   // errors with these codes end the link
let wsTimer=null,lastSent='',armTorque=null,calibKey='',resync=false,armEpoch=null,armClients=1,calibMsg=false;
let dirBoxes=[];

export function send(o){if(S.ws&&S.ws.readyState===1)S.ws.send(JSON.stringify(o));}
export const linkLive=()=>!!(S.ws&&S.ws.readyState===1&&haveRealNow());
function wsNote(t){$('#wsNote').textContent=t;}
const anglesDeg=()=>qCmd.map(a=>+(a/DEG).toFixed(2));
function libAutoload(){if($('#wsPw').value){libRefresh();poseRefresh();}}

function adoptMeasured(){ // start the sim from the arm's real pose so the next command doesn't swing it
  if(S.play)endPlay('Playback stopped: re-read the arm\'s pose.');
  for(let i=0;i<N;i++){qIK[i]=clampJ(i,S.measured[i]);qCmd[i]=qIK[i];servo[i].pos=S.measured[i]*DEG;servo[i].vel=0;}
  targetFromPose();S.homeLock=true;setDemo(false);lastSent='';
}
export function setLimp(on){
  S.limp=on;$('#btnLimp').setAttribute('aria-pressed',on);$('#recLimp').setAttribute('aria-pressed',on);
  if(on&&S.play)endPlay('Playback stopped: hand-guide mode is on.');
  if(on){setDemo(false);send({type:'torque',on:false});wsNote('Torque is off. Move the arm by hand and the sim follows it.');}
  else{ // hold where the arm is now: adopt the measured pose as the new goal before torque comes back
    if(S.measured&&S.measured.every(v=>v!==null)){for(let i=0;i<N;i++){qIK[i]=qCmd[i]=S.measured[i]*DEG;servo[i].pos=qIK[i];servo[i].vel=0;}targetFromPose();}
    S.homeLock=true;send({type:'torque',on:true});wsNote('Torque is on. The arm holds its pose until you set a new target.');}
}
export function setStopped(on,fromArm){ // fromArm: the backend reported it, so don't send it back
  if(on===S.stopped)return;
  S.stopped=on;const b=$('#btnStop');b.setAttribute('aria-pressed',on);b.textContent=on?'Resume':'Stop';
  if(on){setDemo(false);for(let i=0;i<N;i++){qCmd[i]=servo[i].pos;servo[i].vel=0;}if(!fromArm)send({type:'stop'});if(S.play)endPlay('Playback stopped.');}
  else{
    if(!fromArm)send({type:'resume'});
    if(S.ws&&S.measured)resync=true;   // re-read the real pose before sending anything new
    else{for(let i=0;i<N;i++)qIK[i]=qCmd[i];targetFromPose();S.homeLock=true;}
  }
}

function wsStop(msg){
  if(wsTimer)clearInterval(wsTimer);wsTimer=null;
  if(S.ws){const w=S.ws;S.ws=null;try{w.close();}catch(_){}}
  S.measured=null;S.limp=false;calibKey='';resync=false;armEpoch=null;armClients=1;calibMsg=false;S.remoteBlocked=null;S.remotePlay=null;S.remotePending=false;S.playEndN=null;S.armFault=null;playUI();$('#calNote').textContent='';$('#btnLimp').disabled=true;$('#btnZero').disabled=true;dirBoxes.forEach(b=>b.disabled=true);
  for(let i=0;i<N;i++){LIM[i][0]=URDF_LIM[i][0];LIM[i][1]=URDF_LIM[i][1];}$('#btnLimp').setAttribute('aria-pressed','false');
  $('#recLimp').disabled=true;$('#recLimp').setAttribute('aria-pressed','false');
  $('#btnWs').textContent='Connect';$('#btnWs').setAttribute('aria-pressed','false');
  if(msg){wsNote(msg);if(!/^Disconnected/.test(msg))showConn();}   // a failure: open the menu so it's seen
}
function connect(){
  if(S.ws){wsStop('Disconnected from the backend.');return;}
  const pw=$('#wsPw').value;
  if(!pw){wsNote('Enter the arm password first.');showConn();$('#wsPw').focus();return;}
  let sock;
  try{sock=new WebSocket($('#wsUrl').value.trim());}catch(err){wsNote('That address is not a valid WebSocket URL. It should look like ws://raspberrypi.local:8000/ws/arm.');return;}
  S.ws=sock;$('#btnWs').textContent='Disconnect';$('#btnWs').setAttribute('aria-pressed','true');wsNote('Connecting…');
  let opened=false,greeted=false;
  let toolSynced=false,areaSynced=false;
  sock.onopen=()=>{opened=true;lastSent='';$('#btnLimp').disabled=false;$('#recLimp').disabled=false;
    sock.send(JSON.stringify({type:'auth',password:pw}));storePw(pw);
    wsNote('Connected. Waiting for servo readings before sending anything.');libAutoload();
    wsTimer=setInterval(()=>{
      if(sock.readyState!==1||S.limp||resync||S.stopped||S.remotePlay||S.remotePending||!$('#optSend').checked||!S.measured)return;
      const msg=JSON.stringify({type:'goal',angles:anglesDeg(),speed:+spd.value,acc:+acc.value,epoch:armEpoch});
      if(msg!==lastSent){sock.send(msg);lastSent=msg;}
    },100);};
  // settings: sent once after hello, then whenever they change
  const onConfig=m=>{
    if(Array.isArray(m.limits))m.limits.forEach((l,i)=>{if(i<N&&l[1]>l[0]){LIM[i][0]=l[0]*DEG;LIM[i][1]=l[1]*DEG;}});
    if(Array.isArray(m.dir))m.dir.forEach((d,i)=>{if(dirBoxes[i]){dirBoxes[i].checked=d<0;dirBoxes[i].disabled=false;}});
    $('#btnZero').disabled=false;
    if(typeof m.stall_guard==='boolean')$('#optStall').checked=m.stall_guard;
    if(!toolSynced&&typeof m.tool_mm==='number'){ // the backend's saved attachment wins: it's what its collision check uses
      toolSynced=true;const was=attachment;setAttachment(m.attachment||'custom',false,m.tool_mm,m.tool_d_mm);
      if(was!==attachment)wsNote(`The arm is set up with: ${ATTACHMENTS[attachment].name}. Change it in Setup if that's wrong.`);}
    if(!areaSynced&&m.area&&typeof m.area.center==='number'){ // and so does its work area
      areaSynced=true;const was=JSON.stringify(area);setArea(m.area,false);
      if(was!==JSON.stringify(area))wsNote('Using the work area saved on the arm (Setup tab).');}
    // warn when calibration is missing or a zero leaves a joint short of its travel
    const notes=[];
    if(m.calibrated===false)notes.push('Not calibrated yet, so the angles use default zeros and the arm-side collision check is only approximate.');
    if(Array.isArray(m.limits))m.limits.forEach((l,i)=>{const u=JOINTS[i];const lo=u.min/DEG,hi=u.max/DEG;
      if(l[0]>lo+10||l[1]<hi-10)notes.push(`${u.name} can only turn ${l[0].toFixed(0)}° to ${l[1].toFixed(0)}° with this zero (full travel ${lo.toFixed(0)}° to ${hi.toFixed(0)}°). A zero nearer mid-travel gives it more room.`);});
    $('#calNote').textContent=notes.join(' ');
    const key=JSON.stringify([m.zero,m.dir]);
    if(calibKey&&key!==calibKey){calibMsg=true;wsNote('Calibration changed. Re-reading the arm\'s pose.');}
    calibKey=key;
  };
  sock.onmessage=ev=>{
    let m;try{m=JSON.parse(ev.data);}catch(_){return;}
    if(m.type==='error'){
      if(WS_FATAL.includes(m.code)){if(m.code==='auth')forgetPw();wsStop(m.code==='auth'?'The backend rejected that password.':(m.message||'The backend reported an error.'));return;}
      if(!(m.code==='refused'&&m.ref==='goal'))wsNote(m.message||'The arm refused that.');   // a refused goal: the state says why
      return;}
    if(m.type==='hello'){if(m.protocol===WS_PROTOCOL)greeted=true;
      else wsStop(`The backend speaks protocol ${m.protocol} and this page ${WS_PROTOCOL}. Open the page from the backend (/sim) so they match.`);return;}
    if(!greeted){wsStop('This backend is older than the page. Update the backend or open the page it serves (/sim).');return;}
    if(m.type==='config'){onConfig(m);return;}
    if(m.type!=='state'||!Array.isArray(m.angles))return;
    const first=!S.measured;
    S.measured=m.angles.slice(0,N).map(v=>typeof v==='number'?v:null);S.measuredAt=performance.now();armTorque=m.torque;
    const complete=S.measured.every(v=>v!==null);
    S.remoteBlocked=m.blocked||null;
    S.armFault=m.fault||null;
    { // playback running on the backend: the page follows the arm and sends nothing
      const was=S.remotePlay;S.remotePlay=m.playback||null;
      if(S.remotePlay){S.remotePending=false;if(!was){S.homeLock=true;setDemo(false);S.remoteStopSent=false;if(S.play)endPlay();}}
      if(m.play_end){if(S.playEndN!==null&&m.play_end.n!==S.playEndN){S.remotePending=false;if(m.play_end.message)playNote(m.play_end.message);}S.playEndN=m.play_end.n;}
      if(was&&!S.remotePlay&&complete)adoptMeasured();
      playUI();
    }
    if(typeof m.stopped==='boolean'&&m.stopped!==S.stopped)setStopped(m.stopped,true);
    if(typeof m.clients==='number'&&m.clients!==armClients){
      if(m.clients>armClients)wsNote(`${m.clients} pages are connected to the arm. Whichever one sends a target last moves it.`);
      armClients=m.clients;}
    // the epoch goes up whenever the pose must be re-read (resume, calibration change, end of a
    // backend playback); the backend refuses goals carrying an older one
    if(armEpoch===null)armEpoch=m.epoch;
    else if(m.epoch!==armEpoch){armEpoch=m.epoch;resync=true;}
    if(resync&&complete){adoptMeasured();resync=false;
      if(calibMsg){calibMsg=false;wsNote('Calibration saved. The sim now matches the arm\'s pose.');}return;}
    if(first){
      const missing=S.measured.map((v,i)=>v===null?JOINTS[i].name:null).filter(Boolean);
      if(!missing.length)adoptMeasured();
      wsNote(missing.length?`Connected, but ${missing.join(', ')} did not answer. Check the IDs and wiring; nothing is sent until every servo reads back.`
        :'Connected. The sim now starts from the arm\'s real pose; set a target to move it.');
    }
  };
  sock.onerror=()=>{if(!opened)wsStop('Could not reach the backend. Check it is running and the address is right.');};
  sock.onclose=()=>{if(S.ws===sock)wsStop(opened?'The backend closed the connection.':undefined);};
}

export function initLink(){
  // served by the backend itself (/sim/)? then its own host is the arm link
  if(/^https?:$/.test(location.protocol)&&/\/sim(\/(index\.html)?)?$/.test(location.pathname))
    $('#wsUrl').value=(location.protocol==='https:'?'wss://':'ws://')+location.host+'/ws/arm';
  dirBoxes=JOINTS.map((j,i)=>{const l=document.createElement('label');l.className='chk';l.style.margin='0 6px 0 0';
    l.innerHTML=`<input type="checkbox" disabled> Reverse ${j.name}`;$('#dirRow').appendChild(l);
    const box=l.querySelector('input');box.addEventListener('change',()=>send({type:'set_dir',joint:i,dir:box.checked?-1:1}));return box;});
  $('#btnZero').addEventListener('click',()=>{
    if(!confirm('Make the arm\'s current pose the zero pose for every joint?'))return;send({type:'set_zero'});});
  $('#btnLimp').addEventListener('click',()=>setLimp(!S.limp));
  $('#optStall').addEventListener('change',e=>send({type:'set_stall_guard',on:e.target.checked}));
  $('#btnWs').addEventListener('click',connect);
  $('#wsPw').addEventListener('keydown',e=>{if(e.key==='Enter'&&!S.ws)connect();});
  // don't send goals until every servo reads back
  setInterval(()=>{if(S.ws&&S.measured&&S.measured.some(v=>v===null))lastSent='';},500);
}
