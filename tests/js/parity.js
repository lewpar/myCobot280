// Reads {"poses": [[deg x6], ...], "tool_mm": n, "player": {...}} on stdin and prints what the page
// computes: checkPose for each pose (true = blocked) and the goals of a simulated Player run.
const { pureBlocks } = require('./page');
const K = pureBlocks();
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
K.setTool((inp.tool_mm || 0) / 1000);
const out = { blocked: inp.poses.map(q => !!K.checkPose(q.map(v => v * K.DEG))) };
if (inp.player) {
  const p = inp.player, pl = new K.Player('x', p.steps, 0, p.opts), arm = p.arm.slice(), goals = [];
  for (let t = 0; t < 60; t = Math.round((t + p.dt) * 1e6) / 1e6) {
    const act = pl.tick(t, arm.slice());
    if (act.goal) { goals.push([t, pl.phase, act.goal, act.speeds]); arm.splice(0, 6, ...act.goal); }
    if (act.done) break;
  }
  out.goals = goals;
}
process.stdout.write(JSON.stringify(out));
