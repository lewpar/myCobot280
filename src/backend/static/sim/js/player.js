/* Player: a copy of src/backend/player.py (keep the two in step; tests/test_page.py compares them goal for goal).
   Offline playback ticks it in the frame loop; times in seconds, angles in degrees. */
import * as THREE from 'three';
import {N} from './kinematics.js';

export const PB={GOAL_DT:0.1,LEAD:0.15,ARRIVE:2,GRACE:3,MIN_DPS:3};
export function poseAt(fr,t,hint){
  let i=hint>=0&&hint<fr.length-1&&fr[hint][0]<=t?hint:0;
  while(i<fr.length-2&&fr[i+1][0]<=t)i++;
  const a=fr[i],b=fr[i+1],u=b[0]<=a[0]?1:THREE.MathUtils.clamp((t-a[0])/(b[0]-a[0]),0,1);
  return [a.slice(1).map((v,j)=>v+(b[j+1]-v)*u),i];
}
export class Player{
  constructor(name,steps,now,o){Object.assign(this,{name,steps,rate:o.rate,loop:o.loop,timed:o.timed,speed:o.speed});this.step=0;this.enter('approach',now);}
  enter(ph,now){this.phase=ph;this.since=now;this.t=0;this.hint=0;this.nextEv=0;this.arrivedBy=null;this.lastGoal=-1e9;}
  get cur(){return this.steps[this.step];}
  status(){const s=this.cur;return{name:this.name,recording:s.name,step:this.step,steps:this.steps.length,phase:this.phase,t:this.t,duration:s.frames[s.frames.length-1][0]};}
  travel(cur,goal,now,act){
    if(this.arrivedBy===null)this.arrivedBy=now+Math.max(0,...goal.map((g,j)=>Math.abs(g-cur[j])))/Math.max(this.speed,1)+PB.GRACE;
    if(now-this.lastGoal>=PB.GOAL_DT){act.goal=goal;act.speeds=goal.map(()=>this.speed);this.lastGoal=now;}
    return goal.every((g,j)=>Math.abs(g-cur[j])<=PB.ARRIVE)||now>this.arrivedBy;
  }
  afterStep(now,act){if((this.cur.pause||0)>0&&(this.step+1<this.steps.length||this.loop))this.enter('pause',now);else this.nextStep(now,act);}
  nextStep(now,act){if(this.step+1<this.steps.length)this.step++;else if(this.loop)this.step=0;else{act.done=true;return;}this.enter('approach',now);}
  tick(now,cur){
    const act={goal:null,speeds:null,events:[],done:false},s=this.cur,fr=s.frames,end=fr[fr.length-1][0];
    if(this.phase==='approach'){if(this.travel(cur,fr[0].slice(1),now,act))this.enter('run',now);return act;}
    if(this.phase==='run'){
      this.t=Math.min(end,(now-this.since)*this.rate);
      const ev=s.events||[];while(this.nextEv<ev.length&&ev[this.nextEv][0]<=this.t)act.events.push(ev[this.nextEv++]);
      if(now-this.lastGoal>=PB.GOAL_DT||this.t>=end){
        const ahead=Math.min(end,this.t+(this.timed?PB.LEAD*this.rate:0));let goal;[goal,this.hint]=poseAt(fr,ahead,this.hint);
        const q=poseAt(fr,this.t,this.hint)[0];
        act.goal=goal;act.speeds=this.timed?goal.map((g,j)=>Math.max(PB.MIN_DPS,1.1*Math.abs(g-q[j])/PB.LEAD,Math.abs(g-cur[j])/PB.LEAD)):goal.map(()=>this.speed);
        this.lastGoal=now;
      }
      if(this.t>=end){this.enter('finish',now);this.t=end;this.lastGoal=now;}
      return act;
    }
    if(this.phase==='finish'){this.t=end;if(this.travel(cur,fr[fr.length-1].slice(1),now,act)){if(s.return_zero)this.enter('zero',now);else this.afterStep(now,act);}return act;}
    if(this.phase==='zero'){if(this.travel(cur,new Array(N).fill(0),now,act))this.afterStep(now,act);return act;}
    if(this.phase==='pause'){this.t=now-this.since;if(this.t>=(s.pause||0))this.nextStep(now,act);return act;}
    act.done=true;return act;
  }
}
