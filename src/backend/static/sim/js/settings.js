/* Arm setup the collision check depends on, kept on this device and sent to the backend: the attachment
   (Attachments tab) and the work area (Robot tab). Once connected, the backend's saved values win (link.js). */
import {ATTACHMENTS,toolLen,setToolSize} from './kinematics.js';
import {DEFAULT_AREA,area,setAreaModel} from './collision.js';
import {applyTool,drawArea} from './scene.js';
import {S} from './state.js';
import {send} from './link.js';
import {$,item,uiGet,uiSet} from './util.js';

/* ---- attachment ---- */
export let attachment='none';
const toolIn=$('#tool'),toolDIn=$('#toolD');
function syncToolSliders(){$('#toolv').textContent=toolIn.value+' mm';$('#toolDv').textContent=toolDIn.value+' mm';}
function attRender(){
  const box=$('#attList');box.textContent='';
  Object.entries(ATTACHMENTS).forEach(([id,a])=>{
    const size=a.length===null?`${toolIn.value} × ⌀${toolDIn.value} mm`:a.length?`${a.length} × ⌀${a.diameter} mm`:'';
    const b=item(a.name,size,null,()=>setAttachment(id,true));b.setAttribute('role','radio');b.setAttribute('aria-checked',id===attachment);
    b.setAttribute('aria-selected',id===attachment);box.appendChild(b);});
  $('#attCustom').hidden=attachment!=='custom';
  $('#attNote').textContent=ATTACHMENTS[attachment].note;
  $('#attMeta').textContent=toolLen>0.002?`tip ${Math.round(toolLen*1000)} mm past the flange`:'flange';
}
let toolTimer=null;
export function setAttachment(id,user,len,dia){ // user: picked here (tell the backend); else adopted from it
  const a=ATTACHMENTS[id]||ATTACHMENTS.none;attachment=ATTACHMENTS[id]?id:'none';
  if(len!==undefined){toolIn.value=len;toolDIn.value=Math.max(5,Math.min(60,dia||20));}
  setToolSize((a.length===null?+toolIn.value:a.length)/1000,(a.length===null?+toolDIn.value:(a.diameter||20))/2000);
  applyTool(attachment);syncToolSliders();attRender();S.homeLock=false;S.lastRescueKey='';
  uiSet('mycobot-attachment',JSON.stringify({id:attachment,len:+toolIn.value,dia:+toolDIn.value}));
  if(user){clearTimeout(toolTimer);toolTimer=setTimeout(()=>send({type:'set_tool',attachment,mm:+toolIn.value,d_mm:+toolDIn.value}),300);}
}

/* ---- work area ---- */
const AREA_PRESETS=[['Front half',{center:0,span:180}],['Right half',{center:-90,span:180}],['Left half',{center:90,span:180}],['Full circle',{center:0,span:360}]];
const areaDirName=c=>({'-90':'right','90':'left','0':'front','180':'back','-180':'back'})[String(Math.round(c))]||'';
let areaTimer=null;
export function setArea(a,user){
  setAreaModel({enabled:!!a.enabled,center:+a.center,span:+a.span,radius_mm:+a.radius_mm||0});
  $('#areaOn').checked=area.enabled;$('#areaCenter').value=area.center;$('#areaSpan').value=area.span;$('#areaRadius').value=area.radius_mm;
  $('#areaCenterv').textContent=`${area.center}°`+(areaDirName(area.center)?` ${areaDirName(area.center)}`:'');
  $('#areaSpanv').textContent=area.span>=360?'full':area.span+'°';
  $('#areaRadiusv').textContent=area.radius_mm?area.radius_mm+' mm':'no limit';
  ['#areaCenter','#areaSpan','#areaRadius'].forEach(id=>$(id).disabled=!area.enabled);
  $('#areaPresets').querySelectorAll('button').forEach((b,k)=>{const p=AREA_PRESETS[k][1];
    b.setAttribute('aria-pressed',area.enabled&&area.span===p.span&&(p.span>=360||area.center===p.center));});
  $('#areaMeta').textContent=!area.enabled?'off':area.span>=360?(area.radius_mm?'circle':'no side limit'):`${area.span}° slice`;
  drawArea();S.homeLock=false;S.lastRescueKey='';
  uiSet('mycobot-area',JSON.stringify(area));
  if(user){clearTimeout(areaTimer);areaTimer=setTimeout(()=>send({type:'set_area',...area}),250);}
}

export function initSettings(){
  [toolIn,toolDIn].forEach(el=>el.addEventListener('input',()=>setAttachment('custom',true)));
  $('#optEnvelope').addEventListener('change',()=>applyTool(attachment));
  syncToolSliders();
  {let a=null;try{a=JSON.parse(uiGet('mycobot-attachment'));}catch(_){}   // last attachment used on this device (the backend's wins once connected)
   if(a&&ATTACHMENTS[a.id])setAttachment(a.id,false,a.len,a.dia);else setAttachment('none',false);}
  AREA_PRESETS.forEach(([name,p])=>{const b=document.createElement('button');b.type='button';b.textContent=name;
    b.addEventListener('click',()=>setArea({...area,...p,enabled:true},true));$('#areaPresets').appendChild(b);});
  $('#areaOn').addEventListener('change',e=>setArea({...area,enabled:e.target.checked},true));
  $('#areaCenter').addEventListener('input',e=>setArea({...area,center:+e.target.value},true));
  $('#areaSpan').addEventListener('input',e=>setArea({...area,span:+e.target.value},true));
  $('#areaRadius').addEventListener('input',e=>{const v=+e.target.value;setArea({...area,radius_mm:v<100?0:v},true);});
  {let a=null;try{a=JSON.parse(uiGet('mycobot-area'));}catch(_){}
   setArea(a&&typeof a.center==='number'?a:DEFAULT_AREA,false);}
}
