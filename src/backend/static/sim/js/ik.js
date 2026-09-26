/* IK: task-priority damped least squares (position first, flange direction in the null space). */
import * as THREE from 'three';
import {N,LIM,makeFK,fk} from './kinematics.js';
import {checkPose} from './collision.js';

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
/* Restarts from seeded poses when the solver is stuck or colliding; the best {q, r} or null. */
export function ikRescue(qCur,target,orient){
  const yaw=Math.atan2(target.y,target.x);
  const bends=[[0.5,0.9,0.9],[0.2,1.3,0.9],[0.9,0.4,1.2],[-0.3,1.6,1.0]];
  let best=null,bestScore=Infinity;
  for(const y0 of [yaw,yaw+Math.PI/2,yaw-Math.PI/2,yaw+Math.PI])for(const b of bends)for(const sgn of [1,-1]){
    let y=y0;while(y>Math.PI)y-=2*Math.PI;while(y<-Math.PI)y+=2*Math.PI;
    const q=[y,sgn*b[0],sgn*b[1],sgn*b[2],0,0];
    const r=ikIterate(q,target,80,orient);
    let move=0;for(let i=0;i<N;i++)move+=Math.abs(q[i]-qCur[i]);
    const score=r.pos*1000+r.ori*20+move*0.5+(checkPose(q)?1e6:0);
    if(score<bestScore){bestScore=score;best={q,r};}
  }
  return best;
}
