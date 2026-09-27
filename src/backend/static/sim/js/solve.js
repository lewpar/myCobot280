/* IK comes from the backend (ik.py); the page has no solver of its own. When the page isn't driving the arm it
   asks /ws/ik, the backend's solve-only socket (it needs the password but no arm): one request in flight at a
   time, each carrying the pose to start from (qIK) and where the simulated servos are, so the answer includes
   the route. When it is driving the arm, link.js sends `target` over /ws/arm instead and hands each state's
   `ik` to setArmIk. Either way the latest answer is `ikRes` (angles and next in degrees, as the backend sends
   them) for the frame loop in main.js. */
import {DEG,N,LIM,toolLen,toolR} from './kinematics.js';
import {area} from './collision.js';
import {$} from './util.js';

const IK_PROTOCOL=4,REPLY_MS=5000,RETRY_MS=2000;
let sock=null,ready=false,inFlight=null,settingsKey='',settledKey='',nextId=1,retryAt=0,badPw=null,note='';
export let ikRes=null;   // the latest answer: the backend's result plus basis (the qIK it started from, radians) and src

const ikUrl=()=>{const u=$('#wsUrl').value.trim();return /\/ws\/arm\/?$/.test(u)?u.replace(/\/ws\/arm\/?$/,'/ws/ik'):null;};
/* Why there's no solution to show, for the status bar ('' when the solver is up). */
export function solverNote(){
  if(ready)return '';
  if(!$('#wsPw').value)return 'Enter the arm password (connection menu): the backend solves the target';
  return note||'Connecting to the backend\'s solver…';
}

function close(){if(sock){const s=sock;sock=null;try{s.close();}catch(_){}}ready=false;inFlight=null;settledKey='';}
function open(){
  const url=ikUrl(),pw=$('#wsPw').value;
  if(!url){note='The backend address in the connection menu should end in /ws/arm.';return;}
  if(!pw||pw===badPw)return;
  let s;try{s=new WebSocket(url);}catch(_){note='The backend address in the connection menu isn\'t valid.';return;}
  sock=s;settingsKey='';retryAt=performance.now()+RETRY_MS;
  s.onopen=()=>s.send(JSON.stringify({type:'auth',password:pw}));
  s.onmessage=ev=>{
    if(s!==sock)return;
    let m;try{m=JSON.parse(ev.data);}catch(_){return;}
    if(m.type==='hello'){if(m.protocol===IK_PROTOCOL){ready=true;note='';}
      else{note=`The backend speaks protocol ${m.protocol} and this page ${IK_PROTOCOL}. Open the page the backend serves (/sim).`;close();}return;}
    if(m.type==='error'&&['auth','locked'].includes(m.code)){   // don't retry a wrong password: it counts towards the lockout
      if(m.code==='auth')badPw=pw;note=m.code==='auth'?'The backend rejected that password.':m.message;close();return;}
    if(m.type==='ik'&&inFlight&&m.id===inFlight.id){ikRes={...m,basis:inFlight.q,src:'ik',fresh:true};
      settledKey=m.settled?inFlight.key:'';inFlight=null;return;}
    if(m.type==='error'&&inFlight&&m.ref==='solve')inFlight=null;
  };
  s.onclose=()=>{if(s===sock){sock=null;ready=false;inFlight=null;retryAt=performance.now()+RETRY_MS;}};
}
/* Reconnect with a new address or password (the connection menu changed). */
export function solverReconnect(){badPw=null;note='';close();retryAt=0;}

/* Ask for the next solution, unless one is already on its way (then false). req: {xyz (THREE.Vector3, metres) or angles
   (radians), down, q (radians: where to start solving), from (radians: the servos), rescue, restart}. */
export function requestSolve(req){
  const now=performance.now();
  if(!sock){if(now>=retryAt)open();return false;}
  if(!ready||sock.readyState!==1)return false;
  if(inFlight&&now-inFlight.at<REPLY_MS)return false;
  const deg=q=>q.map(v=>+(v/DEG).toFixed(4));
  const lim=LIM.map(l=>[+(l[0]/DEG).toFixed(3),+(l[1]/DEG).toFixed(3)]);
  const settings={type:'settings',tool_mm:+(toolLen*1000).toFixed(2),tool_d_mm:+(toolR*2000).toFixed(2),area:{...area},limits:lim};
  const sKey=JSON.stringify(settings);
  const msg={type:'solve',down:!!req.down,q:deg(req.q),from:deg(req.from),rescue:req.rescue!==false,restart:!!req.restart};
  if(req.angles)msg.angles=deg(req.angles);else msg.xyz=[req.xyz.x,req.xyz.y,req.xyz.z].map(v=>+(v*1000).toFixed(2));
  // the same question as last time, whose answer was settled: asking again would get the same answer
  const key=sKey+JSON.stringify(msg);
  if(key===settledKey)return false;
  if(sKey!==settingsKey){sock.send(sKey);settingsKey=sKey;}   // its reply comes before the solve's
  msg.id=nextId++;
  inFlight={id:msg.id,at:now,q:req.q.slice(),key};
  sock.send(JSON.stringify(msg));
  return true;
}
/* The arm is solving (link.js is sending `target`): each state's ik becomes the latest answer. */
export function setArmIk(ik){ikRes=ik?{...ik,basis:null,src:'arm',fresh:true}:null;}
export function dropIk(){ikRes=null;inFlight=null;settledKey='';}

export function initSolve(){
  // a new address or password: try again (only on change, not every keystroke: a wrong password counts)
  ['#wsUrl','#wsPw'].forEach(id=>$(id).addEventListener('change',solverReconnect));
  $('#btnWs').addEventListener('click',()=>{if(!ready)solverReconnect();});
}
export const nearPose=(a,b,tol=1e-3)=>{for(let i=0;i<N;i++)if(Math.abs(a[i]-b[i])>tol)return false;return true;};
