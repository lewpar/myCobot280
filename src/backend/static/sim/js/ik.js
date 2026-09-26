/* IK: task-priority damped least squares (position first, flange direction in the null space), seeded
   restarts when it's stuck, and the route the servos take to a solution (planMove). */
import * as THREE from 'three';
import {DEG,N,LIM,makeFK,fk,toolLen,toolR} from './kinematics.js';
import {checkPose,checkPath} from './collision.js';

function solveLinear(A,b,n){
  for(let c=0;c<n;c++){
    let p=c;for(let r=c+1;r<n;r++)if(Math.abs(A[r][c])>Math.abs(A[p][c]))p=r;
    [A[c],A[p]]=[A[p],A[c]];[b[c],b[p]]=[b[p],b[c]];
    const d=A[c][c]||1e-12;
    for(let r=c+1;r<n;r++){const f=A[r][c]/d;if(!f)continue;for(let k=c;k<n;k++)A[r][k]-=f*A[c][k];b[r]-=f*b[c];}
  }
  const x=new Array(n).fill(0);
  for(let r=n-1;r>=0;r--){let s=b[r];for(let k=r+1;k<n;k++)s-=A[r][k]*x[k];x[r]=s/(A[r][r]||1e-12);}
  return x;
}
function pinv(J,m,n,lam2){
  const A=[];for(let r=0;r<m;r++){A.push(new Array(m).fill(0));for(let c=0;c<m;c++){let s=0;for(let k=0;k<n;k++)s+=J[r][k]*J[c][k];A[r][c]=s+(r===c?lam2:0);}}
  const X=[];for(let r=0;r<m;r++)X.push(new Array(m).fill(0));
  for(let c=0;c<m;c++){const e=new Array(m).fill(0);e[c]=1;const col=solveLinear(A.map(r=>r.slice()),e,m);for(let r=0;r<m;r++)X[r][c]=col[r];}
  const P=[];for(let i=0;i<n;i++){P.push(new Array(m).fill(0));for(let c=0;c<m;c++){let s=0;for(let r=0;r<m;r++)s+=J[r][i]*X[r][c];P[i][c]=s;}}
  return P;
}
const ikF=makeFK(),_e=new THREE.Vector3(),_c=new THREE.Vector3(),_eo=new THREE.Vector3(),DOWN=new THREE.Vector3(0,0,-1);
/* Moves q (radians, in place) toward putting the TCP on target; returns the remaining errors. */
export function ikIterate(q,target,iters,orient){
  let posErr=0,oriErr=0;
  for(let it=0;it<iters;it++){
    fk(q,ikF);
    _e.subVectors(target,ikF.tcp);posErr=_e.length();
    if(posErr>0.03)_e.multiplyScalar(0.03/posErr);
    if(orient){
      const dot=THREE.MathUtils.clamp(ikF.dir.dot(DOWN),-1,1);oriErr=Math.acos(dot);
      _eo.crossVectors(ikF.dir,DOWN);
      if(_eo.lengthSq()<1e-8){_eo.set(1,0,0).cross(ikF.dir);if(_eo.lengthSq()<1e-8)_eo.set(0,1,0);}
      _eo.normalize().multiplyScalar(Math.min(oriErr,0.3));
    }
    if(posErr<1.5e-4&&(!orient||oriErr<2e-3))break;
    const Jp=[[],[],[]],Jo=[[],[],[]];
    for(let i=0;i<N;i++){
      _c.subVectors(ikF.tcp,ikF.pos[i]).crossVectors(ikF.axis[i],_c);
      Jp[0][i]=_c.x;Jp[1][i]=_c.y;Jp[2][i]=_c.z;
      Jo[0][i]=ikF.axis[i].x;Jo[1][i]=ikF.axis[i].y;Jo[2][i]=ikF.axis[i].z;
    }
    const Pp=pinv(Jp,3,N,0.006*0.006);
    const ep=[_e.x,_e.y,_e.z];const dq=new Array(N).fill(0);
    for(let i=0;i<N;i++)for(let r=0;r<3;r++)dq[i]+=Pp[i][r]*ep[r];
    if(orient){
      const Nm=[];for(let i=0;i<N;i++){Nm.push(new Array(N).fill(0));for(let k=0;k<N;k++){let s=0;for(let r=0;r<3;r++)s+=Pp[i][r]*Jp[r][k];Nm[i][k]=(i===k?1:0)-s;}}
      const JoN=[[],[],[]];for(let r=0;r<3;r++)for(let k=0;k<N;k++){let s=0;for(let i=0;i<N;i++)s+=Jo[r][i]*Nm[i][k];JoN[r][k]=s;}
      const eo=[_eo.x,_eo.y,_eo.z];
      for(let r=0;r<3;r++){let s=0;for(let i=0;i<N;i++)s+=Jo[r][i]*dq[i];eo[r]-=s;}
      const Po=pinv(JoN,3,N,0.05*0.05);
      for(let i=0;i<N;i++){let s=0;for(let r=0;r<3;r++)s+=Po[i][r]*eo[r];dq[i]+=s;}
    }
    let mx=0;for(let i=0;i<N;i++)mx=Math.max(mx,Math.abs(dq[i]));
    const sc=mx>0.1?0.1/mx:1;
    for(let i=0;i<N;i++)q[i]=THREE.MathUtils.clamp(q[i]+dq[i]*sc,LIM[i][0],LIM[i][1]);
  }
  fk(q,ikF);
  return{pos:ikF.tcp.distanceTo(target),ori:orient?Math.acos(THREE.MathUtils.clamp(ikF.dir.dot(DOWN),-1,1)):0};
}
/* Like ikIterate, but q never steps from a clear pose into a collision: it stops at its last clear step
   (the solver itself knows nothing about collisions, and following a target it would happily walk the elbow
   into the base). A q that already collides just iterates. */
export function ikIterateClear(q,target,iters,orient){
  if(checkPose(q))return ikIterate(q,target,iters,orient);
  let r=ikIterate(q,target,0,orient);
  for(let it=0;it<iters;it++){const prev=q.slice();r=ikIterate(q,target,1,orient);
    if(checkPose(q)){for(let i=0;i<N;i++)q[i]=prev[i];return ikIterate(q,target,0,orient);}
    if(r.pos<1.5e-4&&(!orient||r.ori<2e-3))break;}
  return r;
}

/* How the servos get from `from` to `to` (radians): [] = straight there, else the poses to pass through
   first; null if no route is clear. When the straight joint-space move would hit something or take the tip
   out of the work area, it goes through raised poses (J2-J5 at 0, the arm pointing up): out of `from` (all at
   once, or lifting the shoulder first, or straightening the elbow and wrist first), turning the base while
   raised, and down into `to` the same ways in reverse. Every leg gets the same collision and work-area check. */
const withZero=(q,js)=>q.map((v,i)=>js.includes(i)?0:v);
const raised=q=>withZero(q,[1,2,3,4]),shoulderUp=q=>withZero(q,[1]),unbent=q=>withZero(q,[2,3,4]);
let planKey='',planOut=null;
export function planMove(from,to){
  const key=from.join()+'|'+to.join();
  if(key===planKey)return planOut;
  planKey=key;planOut=null;
  const legs=new Map(),clear=(a,b)=>{const k=a.join()+'>'+b.join();if(!legs.has(k))legs.set(k,!checkPath(a,b));return legs.get(k);};
  if(clear(from,to))return planOut=[];
  const out=[[raised(from)],[shoulderUp(from),raised(from)],[unbent(from),raised(from)]];
  const into=[[raised(to)],[raised(to),shoulderUp(to)],[raised(to),unbent(to)]];
  const chains=[[raised(to)],[raised(from)],...out.flatMap(a=>into.map(b=>[...a,...b]))];
  for(const vias of chains){
    const pts=[from,...vias,to];let ok=true;
    for(let k=1;k<pts.length&&ok;k++)ok=!checkPose(pts[k])&&clear(pts[k-1],pts[k]);
    if(ok)return planOut=vias;
  }
  return null;
}
// 2: the pose collides; 1: clear, but the servos have no clear route to it from `from`; 0: fine
const trouble=(q,from)=>checkPose(q)?2:from&&!planMove(from,q)?1:0;

/* Restarts from seeded poses when the solver is stuck or colliding; the best {q, r, trouble} or null.
   `from` (where the servos are) makes poses they can't get to lose. tries > 0 uses random seeds instead of the
   fixed ones (which would only find the same pose again), different for each try but repeatable. */
export function ikRescue(qCur,target,orient,from=null,tries=0){
  const yaw=Math.atan2(target.y,target.x);
  const bends=[[0.5,0.9,0.9],[0.2,1.3,0.9],[0.9,0.4,1.2],[-0.3,1.6,1.0]];
  const seeds=[];
  if(!tries)for(const y0 of [yaw,yaw+Math.PI/2,yaw-Math.PI/2,yaw+Math.PI])for(const b of bends)for(const sgn of [1,-1]){
    let y=y0;while(y>Math.PI)y-=2*Math.PI;while(y<-Math.PI)y+=2*Math.PI;
    seeds.push([y,sgn*b[0],sgn*b[1],sgn*b[2],0,0]);
  }
  let rs=tries*7919+1;const rand=()=>(rs=(rs*1103515245+12345)%2147483648)/2147483648;
  if(tries>0)for(let k=0;k<24;k++)seeds.push(LIM.map(([lo,hi],i)=>i===0?yaw+(rand()-0.5)*1.6:(lo+(hi-lo)*rand())*0.8));
  // and from the arm pointing straight up (facing the target, and at zero), stepped clear of collisions for
  // longer: what solving from the zero pose finds, and a pose the route through raised poses always reaches
  const upright=[[yaw,0,0,0,0,0],[0,0,0,0,0,0]].map(q=>({q,clear:true}));
  const found=[...(tries?[]:upright),...seeds.map(q=>({q}))].map(s=>{
    const q=s.q,r=s.clear?ikIterateClear(q,target,240,orient):ikIterate(q,target,80,orient);
    let move=0;for(let i=0;i<N;i++)move+=Math.abs(q[i]-qCur[i]);
    const hit=!!checkPose(q);return {q,r,trouble:hit?2:0,score:r.pos*1000+r.ori*20+move*0.5+(hit?1e6:0)};
  }).sort((a,b)=>a.score-b.score);
  if(!from)return found[0]||null;
  // the best clear pose the servos have a route to; failing that, the best clear one; failing that, the best
  for(const c of found.slice(0,8)){if(c.trouble)break;if(planMove(from,c.q))return c;c.trouble=1;}
  return found[0]||null;
}

/* One frame of solving for the page (main.js): moves q (radians, in place) toward target and returns its
   errors, stepping clear of collisions (ikIterateClear). When q is stuck (short of the target,
   colliding, or with no clear route from `from`, where the servos are) it restarts from seeded poses: RESCUE_MS
   after the target changes, then up to MAX_RETRIES more times, RETRY_MS apart and with new random seeds
   each time, while it stays stuck on the same target (an unreachable one stops costing time). A restart wins
   if it's in less trouble (clear beats colliding, reachable beats unreachable), or as clear and closer.
   mem ({key, t, tries}) carries the retry state between frames; mem.key='' starts it over. */
const RESCUE_MS=250,RETRY_MS=1000,MAX_RETRIES=6;
export function solveFrame(q,target,orient,from,mem,now,rescue=true){
  let r=ikIterateClear(q,target,14,orient);
  const t=trouble(q,from),stuck=t>0||r.pos>0.002||(orient&&r.ori>2*DEG);
  const key=target.toArray().map(v=>v.toFixed(4)).join()+orient+toolLen+toolR;
  if(!stuck){mem.key='';return r;}
  const again=key===mem.key;
  if(rescue&&(again?mem.tries<MAX_RETRIES&&now-mem.t>=RETRY_MS:now-mem.t>=RESCUE_MS)){
    mem.tries=again?mem.tries+1:0;mem.key=key;mem.t=now;
    const b=ikRescue(q,target,orient,from,mem.tries);
    if(b&&(b.trouble<t||(b.trouble===t&&(b.r.pos<r.pos-0.001||(b.r.pos<0.002&&b.r.ori<r.ori-DEG))))){for(let i=0;i<N;i++)q[i]=b.q[i];r=b.r;}
  }
  return r;
}
