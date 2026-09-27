/* REST calls to the backend (/api/*, X-Arm-Password header) and the stored password. The API lives on the
   same host as the /ws/arm address in the connection menu. */
import {showConn} from './chrome.js';
import {$} from './util.js';

const PW_KEY='mycobot-password';
export function storePw(pw){try{sessionStorage.setItem(PW_KEY,pw);if($('#optRemember').checked)localStorage.setItem(PW_KEY,pw);else localStorage.removeItem(PW_KEY);}catch(_){}}
export function forgetPw(){try{sessionStorage.removeItem(PW_KEY);localStorage.removeItem(PW_KEY);}catch(_){}$('#wsPw').value='';}
export function initApi(){ // fill in the password saved in this tab (or on this device, if "remember" was ticked)
  try{const saved=sessionStorage.getItem(PW_KEY)||localStorage.getItem(PW_KEY);if(saved){$('#wsPw').value=saved;$('#optRemember').checked=!!localStorage.getItem(PW_KEY);}}catch(_){}
}

function apiBase(){
  try{const u=new URL($('#wsUrl').value.trim());return (u.protocol==='wss:'?'https:':'http:')+'//'+u.host+'/api';}catch(_){return null;}
}
/* Resolves to the JSON reply; rejects with an Error whose message can be shown as is. err.halt is set for
   401/429 (stop retrying: wrong password or locked out), err.read for GETs. */
export async function api(method,path,body){
  const base=apiBase(),pw=$('#wsPw').value;
  if(!base){showConn();throw new Error('The backend address in the connection menu isn\'t valid.');}
  if(!pw){showConn();$('#wsPw').focus();throw new Error('Enter the arm password in the connection menu first.');}
  let r;
  try{r=await fetch(base+path,{method,headers:Object.assign({'X-Arm-Password':pw},body?{'Content-Type':'application/json'}:{}),body:body?JSON.stringify(body):undefined});}
  catch(_){throw new Error(`Couldn't reach the backend at ${base}.`);}
  let j={};try{j=await r.json();}catch(_){}
  if(r.ok){storePw(pw);return j;}
  const why={401:'The backend rejected that password.',502:'The ATOM didn\'t answer.',503:'The backend has no arm connected (check the serial port).'}[r.status];
  const err=new Error(why||(typeof j.detail==='string'?j.detail:`The backend answered ${r.status}.`));
  err.halt=r.status===401||r.status===429;err.read=method==='GET';throw err;
}
