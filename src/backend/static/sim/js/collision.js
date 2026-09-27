/* Collision model: keep in sync with arm_model.py (metres). tests/test_page.py compares checkPose with
   check_pose on 10,000 poses, and checkFrames/checkSteps mirror player.check_frames/check_steps. */
import * as THREE from 'three';
import {DEG,JOINTS,N,URDF_LIM,makeFK,fk,toolLen,toolR} from './kinematics.js';

export const COLLISION={FLOOR_MARGIN:0.005,TCP_MIN_Z:0.003,BASE_R:0.075,BASE_TOP:0.12,COLUMN_R:0.05,COLUMN_TOP:0.19,WRIST_TO_UPPER_ARM:0.05,
  TOOL_STEP:0.015,UPPER_ARM_R:0.03,FOREARM_R:0.028,AREA_CORE:0.06,BASE_KEEPOUT_TOP:0.25,
  // the ATOM head behind the J5 body, on the J6 axis: [distance behind the flange face, radius] of its spheres
  ATOM_SPHERES:[[0.050,0.017],[0.066,0.016]]};
/* Work area: the slice of the circle around the base the tool tip must stay inside; the rest of the arm may
   cross its edges (as arm_model.DEFAULT_AREA). center: degrees, 0 = +X (where the flange points at zero),
   -90 = -Y (the arm's right); span: width in degrees (360 = no side limit); radius_mm: outer limit, 0 = none;
   base_mm: a keep-out cylinder that radius around the base axis, up to BASE_KEEPOUT_TOP, 0 = none (working that
   close in folds the wrist back onto the arm). Only setAreaModel changes it. */
export const DEFAULT_AREA={enabled:true,center:0,span:180,radius_mm:0,base_mm:150};
export let area={...DEFAULT_AREA};
export function setAreaModel(a){ // (a work area without obstacles keeps the ones there are)
  if(a.obstacles===undefined&&area.obstacles)a={...a,obstacles:area.obstacles};area=a;}
export function setObstacles(obs){area={...area,obstacles:obs};}

/* Obstacles (the Workspace view): boxes, cylinders and spheres the whole arm keeps clear of, checked whether or
   not the work area is on. Keep in sync with arm_model.py (OBST_*, BODY_*, euler_xyz, obstacle_distance,
   _hits_obstacle). Sizes and positions in mm, rotations in degrees (Euler XYZ, as three.js).
   The arm's body is checked as scene.js draws it (the links run up to ~70 mm beside the kinematic joint line):
   base frame at the zero pose, each part moving with joint j. The column (J1-J2) is left to the page's warning. */
export const OBST={MARGIN:0.01,MAX:50,SHAPES:['box','cylinder','sphere'],STEP:0.015,
  LINKS:[ // [name, joint, radius, polyline]: the tubes
    ['the upper arm',1,0.023,[[0,-0.035,0.1386],[0,-0.068,0.1386],[0,-0.068,0.249],[0,-0.031,0.249]]],
    ['the forearm',2,0.021,[[0,-0.031,0.249],[0,0,0.249],[0,0,0.345],[0,-0.029,0.345]]],
    ['the wrist',3,0.020,[[0,-0.029,0.345],[0,-0.0636,0.345],[0,-0.0636,0.381]]],
    ['the wrist',4,0.019,[[0,-0.0636,0.381],[0,-0.0636,0.4181]]]],
  HOUSINGS:[ // [name, joint, centre, axis, radius, length]: the servos
    ['the elbow',1,[0,-0.031,0.249],[0,1,0],0.028,0.032],['J4',2,[0,-0.029,0.345],[0,1,0],0.024,0.028],
    ['the wrist',3,[0,-0.0636,0.381],[0,0,1],0.0205,0.014],['J6',4,[0.023,-0.0636,0.4181],[1,0,0],0.021,0.034]]};
/* the body as points in each joint's own frame: [{name, j, r, pts, cen, reach}] (arm_model._body_parts) */
const BODY=(()=>{
  const z=makeFK();fk(new Array(N).fill(0),z);
  const parts=[];
  for(const [name,j,r,pts] of OBST.LINKS){
    const s=[];
    for(let k=1;k<pts.length;k++){const a=new THREE.Vector3(...pts[k-1]),b=new THREE.Vector3(...pts[k]);
      const n=Math.max(1,Math.ceil(a.distanceTo(b)/OBST.STEP));for(let i=0;i<n;i++)s.push(a.clone().lerp(b,i/n));}
    s.push(new THREE.Vector3(...pts[pts.length-1]));parts.push([name,j,r,s]);
  }
  for(const [name,j,c,ax,r,len] of OBST.HOUSINGS)
    parts.push([name,j,r,[-1,0,1].map(k=>new THREE.Vector3(...c).addScaledVector(new THREE.Vector3(...ax),len/2*k))]);
  return parts.map(([name,j,r,s])=>{
    const inv=z.mat[j].clone().invert(),loc=s.map(p=>p.applyMatrix4(inv));
    const cen=loc.reduce((a,p)=>a.add(p),new THREE.Vector3()).multiplyScalar(1/loc.length);
    return{name,j,r,pts:loc,cen,reach:Math.max(...loc.map(p=>p.distanceTo(cen)))+r};
  });
})();
export function eulerXYZ(rx,ry,rz){
  const a=Math.cos(rx),b=Math.sin(rx),c=Math.cos(ry),d=Math.sin(ry),e=Math.cos(rz),f=Math.sin(rz),ae=a*e,af=a*f,be=b*e,bf=b*f;
  return [[c*e,-c*f,d],[af+be*d,ae-bf*d,-b*c],[bf-ae*d,be+af*d,a*c]];
}
export function prepObstacle(o){
  const c=new THREE.Vector3(o.pos[0]/1000,o.pos[1]/1000,o.pos[2]/1000),R=eulerXYZ(...o.rot.map(v=>v*DEG));
  const [sx,sy,sz]=o.size.map(v=>v/2000);
  if(o.shape==='box')return{name:o.name,shape:'box',c,R,dims:[sx,sy,sz],bound:Math.sqrt(sx*sx+sy*sy+sz*sz)};
  if(o.shape==='cylinder')return{name:o.name,shape:'cylinder',c,R,dims:[sx,sz],bound:Math.hypot(sx,sz)};
  return{name:o.name,shape:'sphere',c,R,dims:[sx],bound:sx};
}
const prepCache=new Map();
function prepared(obs){
  const key=obs.map(o=>[o.shape,...o.pos,...o.size,...o.rot,o.name].join()).join('|');
  let got=prepCache.get(key);
  if(!got){if(prepCache.size>64)prepCache.clear();got=obs.map(prepObstacle);prepCache.set(key,got);}
  return got;
}
export function obstacleDistance(p,pr){ // signed (m): negative inside
  const dx=p.x-pr.c.x,dy=p.y-pr.c.y,dz=p.z-pr.c.z,R=pr.R,d=pr.dims;
  const lx=R[0][0]*dx+R[1][0]*dy+R[2][0]*dz,ly=R[0][1]*dx+R[1][1]*dy+R[2][1]*dz,lz=R[0][2]*dx+R[1][2]*dy+R[2][2]*dz;
  if(pr.shape==='box'){const qx=Math.abs(lx)-d[0],qy=Math.abs(ly)-d[1],qz=Math.abs(lz)-d[2];
    return Math.sqrt(Math.max(qx,0)**2+Math.max(qy,0)**2+Math.max(qz,0)**2)+Math.min(Math.max(qx,qy,qz),0);}
  if(pr.shape==='cylinder'){const rad=Math.hypot(lx,ly)-d[0],ax=Math.abs(lz)-d[1];
    return Math.hypot(Math.max(rad,0),Math.max(ax,0))+Math.min(Math.max(rad,ax),0);}
  return Math.sqrt(lx*lx+ly*ly+lz*lz)-d[0];
}
const _op=new THREE.Vector3();
function hitsObstacle(obs,mat,extra){ // mat: each joint's frame (fk's out.mat)
  for(const pr of prepared(obs)){
    for(const b of BODY){
      if(_op.copy(b.cen).applyMatrix4(mat[b.j]).distanceTo(pr.c)>pr.bound+b.reach+OBST.MARGIN)continue;   // nowhere near
      for(const p of b.pts)if(obstacleDistance(_op.copy(p).applyMatrix4(mat[b.j]),pr)<b.r+OBST.MARGIN)return `${b.name} would hit ${pr.name}`;
    }
    for(const [part,p,r] of extra)
      if(p.distanceTo(pr.c)<=pr.bound+r+OBST.MARGIN&&obstacleDistance(p,pr)<r+OBST.MARGIN)return `${part} would hit ${pr.name}`;
  }
  return null;
}
export function outsideArea(p){
  const rad=Math.hypot(p.x,p.y),lim=area.radius_mm/1000;
  if(lim>0&&rad>lim)return 'would reach past the work area';
  if(rad<(area.base_mm||0)/1000&&p.z<COLLISION.BASE_KEEPOUT_TOP)return 'would come too close to the base';
  if(area.span>=360||rad<COLLISION.AREA_CORE)return null;
  const off=Math.abs(((Math.atan2(p.y,p.x)/DEG-area.center+180)%360+360)%360-180);
  return off>area.span/2?'would leave the work area':null;
}
const colF=makeFK(),_fl=new THREE.Vector3(),_sd=new THREE.Vector3(),_sd2=new THREE.Vector3();
function segDist(p,a,b){
  _sd.subVectors(b,a);const L=_sd.lengthSq();
  const f=L===0?0:THREE.MathUtils.clamp(_sd2.subVectors(p,a).dot(_sd)/L,0,1);
  return _sd2.copy(a).addScaledVector(_sd,f).distanceTo(p);
}
export function checkPose(q){ // q in radians; null if clear, otherwise a reason
  for(let i=0;i<N;i++)if(q[i]<URDF_LIM[i][0]-0.5*DEG||q[i]>URDF_LIM[i][1]+0.5*DEG)return `${JOINTS[i].name} would pass its limit`;
  fk(q,colF);const j=colF.pos,C=COLLISION;
  _fl.copy(colF.tcp).addScaledVector(colF.dir,-toolLen);
  const body=[['the elbow (J3)',j[2],0.03,0.03,false],['the forearm',j[2].clone().lerp(j[3],0.5),0.028,0.028,false],
    ['J4',j[3],0.026,0.026,false],['the wrist (J5)',j[4],0.024,0.024,true],['J6',j[5],0.022,0.022,true],['the flange',_fl,0,0.02,true]];
  const atom=C.ATOM_SPHERES.map(([d,r])=>[_fl.clone().addScaledVector(colF.dir,-d),r]);
  atom.forEach(([p,r])=>body.push(['the ATOM',p,r,r,true]));
  // the attachment: a cylinder along the flange normal, sampled every TOOL_STEP; its lowest point is its full
  // radius when level and nothing when vertical
  const tool=[],tf=toolR*Math.sqrt(Math.max(0,1-colF.dir.z*colF.dir.z));
  if(toolLen>0.002){const n=Math.max(2,Math.ceil(toolLen/C.TOOL_STEP));
    for(let i=1;i<=n;i++)tool.push(_fl.clone().lerp(colF.tcp,i/n));
    tool.slice(0,-1).forEach(p=>body.push(['the attachment',p,tf,toolR,true]));}
  for(const [name,p,rf,rs,wrist] of body){
    if(p.z-rf<C.FLOOR_MARGIN)return `${name} would hit the table`;
    const rad=Math.hypot(p.x,p.y);
    if(rad<C.BASE_R+rs&&p.z-rs<C.BASE_TOP)return `${name} would hit the base`;
    if(wrist&&rad<C.COLUMN_R+rs&&p.z-rs<C.COLUMN_TOP)return `${name} would hit the shoulder`;
    if(wrist&&segDist(p,j[1],j[2])<C.WRIST_TO_UPPER_ARM)return `${name} would hit the upper arm`;
  }
  for(const [p,r] of atom)if(segDist(p,j[2],j[3])<C.FOREARM_R+r)return 'the ATOM would hit the forearm';
  for(const p of tool){ // the attachment folding back into the arm's own links
    if(segDist(p,j[1],j[2])<C.UPPER_ARM_R+toolR)return 'the attachment would hit the upper arm';
    if(segDist(p,j[2],j[3])<C.FOREARM_R+toolR)return 'the attachment would hit the forearm';
  }
  if(area.obstacles&&area.obstacles.length){
    const why=hitsObstacle(area.obstacles,colF.mat,[['the flange',_fl,0.02],...atom.map(([p,r])=>['the ATOM',p,r]),...tool.map(p=>['the attachment',p,toolR])]);
    if(why)return why;}
  if(tool.length&&colF.tcp.z-tf<C.TCP_MIN_Z)return 'the attachment\'s tip would go below the table';
  if(colF.tcp.z<C.TCP_MIN_Z)return 'the tool tip would go below the table';
  if(area.enabled){const why=outsideArea(colF.tcp);if(why)return `${tool.length?'the attachment\'s tip':'the flange'} ${why}`;}
  return null;
}
const _pq=new Array(N).fill(0);
export function checkPath(q0,q1,steps=16,every=3*DEG){ // straight joint-space move (the servos are synchronised to follow it)
  if(checkPose(q0))return checkPose(q1);   // already in contact: allow moving to any clear pose
  let d=0;for(let i=0;i<N;i++)d=Math.max(d,Math.abs(q1[i]-q0[i]));
  steps=Math.max(steps,Math.ceil(d/every));   // at least every 3° (radians), so a long move can't skip over a collision
  for(let s=1;s<=steps;s++){const f=s/steps;for(let i=0;i<N;i++)_pq[i]=q0[i]+(q1[i]-q0[i])*f;
    const why=checkPose(_pq);if(why)return why+(s<steps?' on the way there':'');}
  return null;
}
/* same pre-check as player.check_frames / check_steps on the backend (frames in degrees) */
export function checkFrames(frames){
  let last=null;
  for(let k=0;k<frames.length;k++){
    const q=frames[k].slice(1);
    if(last&&k<frames.length-1&&Math.max(...q.map((v,j)=>Math.abs(v-last[j])))<1)continue;
    const why=checkPose(q.map(v=>v*DEG));if(why)return [frames[k][0],why];last=q;
  }
  return null;
}
export function checkSteps(steps){
  for(let n=0;n<steps.length;n++){const s=steps[n],where=`"${s.name}"`+(steps.length>1?` (step ${n+1})`:'');
    const bad=checkFrames(s.frames);if(bad)return `${where} at ${bad[0].toFixed(1)} s: ${bad[1]}`;
    if(s.return_zero){const why=checkPose(new Array(N).fill(0));if(why)return `${where}: the zero pose is blocked (${why})`;}}
  return null;
}
