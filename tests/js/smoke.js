// jsdom smoke test of the simulator page against a fake backend (REST, /ws/arm and /ws/ik). The IK is the real
// backend solver (ik.py), run through ik_stdio.py. Exits non-zero on a failed check. Run by tests/test_page.py.
const path = require('path');
const { spawn } = require('child_process');
const { loadPage } = require('./page');

// ---- the backend's solver, one JSON line each way ----
const py = spawn(process.env.PYTHON || 'python3', [path.join(__dirname, 'ik_stdio.py')], { stdio: ['pipe', 'pipe', 'inherit'] });
let pyBuf = '';
const pyWait = [];
py.stdout.on('data', d => {
  pyBuf += d;
  for (let k; (k = pyBuf.indexOf('\n')) >= 0; pyBuf = pyBuf.slice(k + 1)) pyWait.shift()(JSON.parse(pyBuf.slice(0, k)));
});
const pySolve = (sid, m) => new Promise(r => { pyWait.push(r); py.stdin.write(JSON.stringify({ _s: sid, ...m }) + '\n'); });

// ---- fake backend ----
const db = { recordings: {}, sequences: {}, poses: {} }, calls = [];
let n = 0, remote = null;
const id = () => String(++n).padStart(12, '0');
const summary = r => ({ id: r.id, name: r.name, created: r.created, return_zero: !!r.return_zero,
  duration: r.frames.at(-1)[0], frames: r.frames.length, events: (r.events || []).length });
async function fetch(url, o = {}) {
  const p = new URL(url).pathname.replace(/^\/api/, ''), m = o.method || 'GET', body = o.body ? JSON.parse(o.body) : null;
  calls.push([m, p, body]);
  const res = (s, j) => ({ ok: s < 300, status: s, json: async () => j });
  if (o.headers['X-Arm-Password'] !== 'pw') return res(401, { detail: 'Password required' });
  const [, coll, key] = p.split('/');
  if (coll === 'playback') {
    if (key === 'stop') { remote = null; playEnd('Playback stopped.'); return res(200, {}); }
    remote = { name: body.recording ? db.recordings[body.recording].name : db.sequences[body.sequence].name, until: Date.now() + 800, body };
    return res(200, { success: true });
  }
  const store = db[coll];
  if (!store) return res(404, { detail: 'nope' });
  if (!key) {
    if (m === 'GET') return res(200, Object.values(store).map(x => coll === 'recordings' ? summary(x) : x));
    const x = { id: id(), created: Date.now() / 1000, ...body };
    if (coll === 'recordings' && x.frames.some(f => f.length !== 7)) return res(422, { detail: 'bad frames' });
    store[x.id] = x; return res(200, coll === 'recordings' ? summary(x) : x);
  }
  const x = store[key];
  if (!x) return res(404, { detail: 'No such item' });
  if (m === 'GET') return res(200, x);
  if (m === 'DELETE') { delete store[key]; return res(200, { success: true }); }
  if (m === 'PUT') { Object.assign(x, body); return res(200, x); }
  if (m === 'PATCH') {
    if (body.name) x.name = body.name;
    if (body.return_zero !== undefined) x.return_zero = body.return_zero;
    if (body.trim) { const [a, b] = body.trim; x.frames = x.frames.filter(f => f[0] >= a - 1e-6 && f[0] <= b + 1e-6).map(f => [+(f[0] - a).toFixed(3), ...f.slice(1)]); }
    return res(200, summary(x));
  }
}
let sock = null, pose = [0, 20, 20, 20, 0, 0], wsSent = [], armState = { stopped: false, fault: null, epoch: 0 }, playEndN = 0, playEndMsg = null;
let ikSent = [], nSess = 0, armTarget = null, armIk = null, armSolving = false;
// torque, the "won't move" reason, raw readings that differ from zero + angle (a servo past its wrap point)
let armTorque = true, armRange = null, tickOver = {};
const ticksOf = () => pose.map((a, j) => tickOver[j] !== undefined ? tickOver[j] : Math.round(2048 + a * cfg.dir[j] * 4096 / 360));
function playEnd(msg) { playEndN++; playEndMsg = msg; armState.epoch++; armTarget = armIk = null; }
function armSolve() { // the fake arm's target: solved by the real solver from where the arm is, then jumps to the next pose
  if (!armTarget || armSolving || remote) return;
  armSolving = true;
  const t = armTarget;
  pySolve('arm', { type: 'solve', ...(t.xyz ? { xyz: t.xyz, down: t.down } : { angles: t.angles }), from: pose }).then(r => {
    armSolving = false;
    if (!armTarget || armTarget.epoch !== t.epoch) return;   // ended meanwhile (a re-sent target carries on)
    delete r._s; delete r.type; delete r.id;
    if (r.next) pose = r.next.slice();
    armIk = { ...r, arrived: !!r.next && !r.detour };
  });
}
const CONFIG = { type: 'config', calibrated: true, zero: [2048, 2048, 2048, 2048, 2048, 2048], dir: [1, 1, 1, 1, 1, 1],
  tool_mm: 0, tool_d_mm: 20, attachment: 'custom', area: { enabled: false, center: 0, span: 180, radius_mm: 0 },
  limits: [[-168, 168], [-140, 140], [-150, 150], [-150, 150], [-155, 160], [-180, 180]], stall_guard: true };
let cfg = { ...CONFIG };
class FakeWS {
  constructor(url) { this.readyState = 1;
    if (/\/ws\/ik$/.test(url)) { // solve-only: every message to the real solver, in its own session
      this.ik = true; this.sid = ++nSess;
      setTimeout(() => { this.onopen(); this.emit({ type: 'hello', protocol: 4 }); }, 5);
      return;
    }
    sock = this;
    pySolve('arm', { type: 'settings', area: CONFIG.area, tool_mm: CONFIG.tool_mm, tool_d_mm: CONFIG.tool_d_mm });
    setTimeout(() => { this.onopen(); this.emit({ type: 'hello', protocol: 4 }); this.emit(cfg); }, 5);
    this.iv = setInterval(() => {
      if (remote && Date.now() > remote.until) { remote = null; playEnd('Playback finished.'); }
      armSolve();
      this.emit({ type: 'state', angles: pose, ticks: ticksOf(), torque: armTorque, out_of_range: armRange, stopped: armState.stopped, blocked: null,
        fault: armState.fault, epoch: armState.epoch, clients: 1,
        playback: remote ? { name: remote.name, recording: remote.name, step: 0, steps: 1, phase: 'run', t: 0.4, duration: 1 } : null,
        play_end: { n: playEndN, message: playEndMsg }, ik: armTarget ? armIk : null });
    }, 100); }
  emit(m) { this.onmessage({ data: JSON.stringify(m) }); }
  send(m) { m = JSON.parse(m);
    if (this.ik) {
      if (m.type === 'auth') return;
      ikSent.push(m);
      pySolve(this.sid, m).then(r => { if (!this.closed) { delete r._s; this.emit(r); } });
      return;
    }
    wsSent.push(m);
    if (m.type === 'goal' && m.epoch === armState.epoch) { armTarget = armIk = null; pose = m.angles.slice(); }
    if (m.type === 'target' && m.epoch === armState.epoch) { if (!armTarget) armIk = null; armTarget = m; }
    if (m.type === 'set_area') pySolve('arm', { type: 'settings', area: { ...m, type: undefined } });
    if (m.type === 'set_tool') pySolve('arm', { type: 'settings', tool_mm: m.mm, tool_d_mm: m.d_mm });
    if (m.type === 'resume') { armState = { stopped: false, fault: null, epoch: armState.epoch + 1 }; armTarget = armIk = null; }
    if (m.type === 'torque') { armTorque = m.on; armTarget = armIk = null; }
    if (m.type === 'recenter') {
      if (armTorque) { this.emit({ type: 'error', code: 'refused', ref: 'recenter', message: 'Turn torque off first' }); return; }
      const results = m.joints.map(j => ({ joint: j, id: j + 1, ok: true, before: ticksOf()[j], after: 2048 }));
      m.joints.forEach(j => { delete tickOver[j]; }); armRange = null; armState.epoch++;
      setTimeout(() => this.emit({ type: 'recentered', results }), 30);
    }
    if (m.type === 'set_zero') { pose = pose.map(() => 0); cfg = { ...cfg, zero: cfg.zero.map(z => z + 1) }; armState.epoch++; this.emit(cfg); }
    if (m.type === 'set_dir') { const d = cfg.dir.slice(); d[m.joint] = m.dir; cfg = { ...cfg, dir: d }; pose[m.joint] = -pose[m.joint]; armState.epoch++; this.emit(cfg); } }
  close() { this.closed = true; clearInterval(this.iv); }
}

// ---- harness ----
let w, errors;
const $ = s => w.document.querySelector(s);
const click = s => (typeof s === 'string' ? $(s) : s).dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
const sleep = ms => new Promise(r => setTimeout(r, ms));
const txt = s => $(s).textContent;
let failed = false;
function check(cond, what, extra = '') { console.log((cond ? 'ok   ' : 'FAIL ') + what + (cond ? '' : '  ' + extra)); if (!cond) failed = true; }
async function until(fn, ms = 15000) { const t = Date.now(); while (!fn() && Date.now() - t < ms) await sleep(50); return fn(); }
const input = (s, v) => { $(s).value = v; $(s).dispatchEvent(new w.Event('input', { bubbles: true })); };

(async () => {
  ({ w, errors } = await loadPage({ fetch, WebSocket: FakeWS }));
  await sleep(200);
  check(/password/.test(txt('#statusText')) && !ikSent.length, 'no password: says the backend solves, sends nothing', txt('#statusText'));
  $('#wsPw').value = 'pw';
  await until(() => /At target/.test(txt('#statusText')), 5000);
  check(ikSent.some(m => m.type === 'settings') && ikSent.some(m => m.type === 'solve' && m.xyz && m.restart),
        'solves on the backend (/ws/ik), restarting the first time', JSON.stringify(ikSent.slice(0, 2)));
  check(/At target/.test(txt('#statusText')), 'reaches the starting target', txt('#statusText'));
  { await sleep(300); const n0 = ikSent.length; await sleep(700);
    check(ikSent.length - n0 <= 1, 'idle at the target: stops asking the solver', `${ikSent.length - n0} solves in 0.7 s`); }

  // 1. record in the simulation, painting an LED on the way
  click('#tabbtn-record');
  click('#recBtn');
  check($('#recBtn').getAttribute('aria-pressed') === 'true', 'recording starts');
  await sleep(300);
  for (let k = 0; k < 15; k++) { input('#tx', 160 + k * 4); await sleep(70); }
  $('#atomGrid').children[7].dispatchEvent(new w.MouseEvent('pointerdown', { bubbles: true }));
  w.dispatchEvent(new w.MouseEvent('pointerup'));
  await sleep(1200);
  click('#recBtn');
  check(!$('#recSave').hidden && /LED cue/.test(txt('#recSaveMeta')), 'save form offers the take with its LED cue', txt('#recSaveMeta'));
  $('#recName').value = 'Sweep';
  click('#recSaveBtn');
  await until(() => Object.keys(db.recordings).length === 1);
  const saved = Object.values(db.recordings)[0];
  check(saved && saved.events.length === 1 && saved.events[0][1] === 'pixel' && saved.frames[0][0] === 0, 'recording saved with frames and cue');

  // 2. Play tab: selected, checked, path drawn
  click('#tabbtn-play');
  await until(() => !$('#recEdit').hidden);
  check($('#recList').children.length === 1 && $('#recList').children[0].getAttribute('aria-selected') === 'true', 'new recording listed and selected');
  check(/Path clear/.test(txt('#edCheck')) && !$('#recPlay').disabled, 'pre-check passes, Play enabled', txt('#edCheck'));

  // 3. play it in the simulation (not connected)
  input('#tx', 120); await sleep(1500);   // away from the start, so playback has an approach (x=0 is beside the shoulder)
  click('#recPlay');
  check(/simulation/.test(txt('#playNote')), 'plays locally when offline', txt('#playNote'));
  await until(() => /finished|stopped/i.test(txt('#playNote')));
  check(txt('#playNote') === 'Playback finished.', 'local playback finishes', txt('#playNote'));

  // 4. edit: rename, return to zero, trim
  $('#edName').value = 'Sweep 2'; click('#edRename');
  await until(() => db.recordings[saved.id].name === 'Sweep 2');
  check(true, 'rename sent');
  $('#edZero').checked = true; $('#edZero').dispatchEvent(new w.Event('change'));
  await until(() => db.recordings[saved.id].return_zero === true);
  check(await until(() => /Sweep 2→ 0/.test(txt('#recList'))), 'renamed and return-to-zero flagged in the list', txt('#recList'));
  const dur = saved.frames.at(-1)[0];
  input('#edT0', 0.5);
  check(!$('#edTrim').disabled, 'trim enabled after moving a handle');
  click('#edTrim');
  await until(() => db.recordings[saved.id].frames[0][0] === 0 && db.recordings[saved.id].frames.at(-1)[0] < dur - 0.3);
  check(calls.some(c => c[0] === 'PATCH' && c[2].trim && c[2].trim[0] === 0.5), 'trim sent');
  click('#edExport');
  check(w.downloads.at(-1) === 'Sweep_2.json', 'export offers a download', JSON.stringify(w.downloads));

  // 5. a colliding recording can't be played
  const bad = { name: 'Bad', frames: [[0, 0, 20, 20, 20, 0, 0], [1, 0, 130, 130, 0, 0, 0]], events: [], return_zero: false };
  const bid = id(); db.recordings[bid] = { id: bid, created: Date.now() / 1000, ...bad };
  click('#recRefresh'); await until(() => $('#recList').children.length === 2);
  click([...$('#recList').children].find(b => /Bad/.test(b.textContent)));
  await until(() => /Collision/.test(txt('#edCheck')));
  check($('#recPlay').disabled, 'colliding recording: Play disabled', txt('#edCheck'));
  click('#edDel'); await until(() => !db.recordings[bid]);
  check(true, 'delete sent');

  // 6. import
  const file = new w.File([JSON.stringify({ format: 'mycobot280-recording', name: 'Imported', frames: saved.frames, events: [] })], 'x.json');
  Object.defineProperty($('#recFile'), 'files', { value: [file], configurable: true });
  $('#recFile').dispatchEvent(new w.Event('change'));
  await until(() => Object.values(db.recordings).some(r => r.name === 'Imported'));
  check(/Imported/.test(txt('#playNote')), 'import', txt('#playNote'));

  // 7. waypoints
  click('#tabbtn-record');
  input('#tx', 40); input('#ty', -190); input('#tz', 100); await sleep(1500); click('#wpAdd');
  input('#tx', 120); input('#ty', -150); await sleep(1200); click('#wpAdd');
  check(txt('#wpMeta') === '2 points' && !$('#wpMake').disabled, 'two waypoints added');
  click('#wpMake');
  check(!$('#recSave').hidden && /through 2 points/.test(txt('#recNote')), 'waypoints make a take', txt('#recNote'));
  $('#recName').value = 'Points'; click('#recSaveBtn');
  await until(() => Object.values(db.recordings).some(r => r.name === 'Points'));
  const pts = Object.values(db.recordings).find(r => r.name === 'Points').frames;
  check(pts.length > 5 && pts.every((f, k) => !k || f[0] > pts[k - 1][0]), 'waypoint recording is smooth and timed');

  // 8. sequence
  click('#tabbtn-play');
  await until(() => [...$('#seqPick').options].some(o => o.textContent === 'Points'));
  click('#seqNew');
  $('#seqName').value = 'Show';
  const opts = [...$('#seqPick').options];
  $('#seqPick').value = opts.find(o => o.textContent === 'Points').value; click('#seqAdd');
  $('#seqPick').value = opts.find(o => o.textContent === 'Imported').value; click('#seqAdd');
  const pauseIn = $('#seqSteps').querySelector('input'); pauseIn.value = '0.5'; pauseIn.dispatchEvent(new w.Event('input'));
  click('#seqSave');
  await until(() => Object.keys(db.sequences).length === 1);
  const seq = Object.values(db.sequences)[0];
  check(seq.steps.length === 2 && seq.steps[0].pause === 0.5, 'sequence saved', JSON.stringify(seq.steps));
  await until(() => !$('#recPlay').disabled);
  click('#recPlay');
  await until(() => /finished|stopped/i.test(txt('#playNote')), 25000);
  check(txt('#playNote') === 'Playback finished.', 'sequence plays locally', txt('#playNote'));

  // 9. saved poses
  click('#tabbtn-motion');
  $('#poseName').value = 'Here'; click('#poseSave');
  await until(() => Object.keys(db.poses).length === 1);
  input('#tx', 120); await sleep(1500);
  click($('#poseList').querySelector('.item button'));
  const want = Object.values(db.poses)[0].angles;
  await sleep(1500);
  const shown = [...w.document.querySelectorAll('.joint .val')].map(e => parseFloat(e.textContent));
  check(shown.every((v, j) => Math.abs(v - want[j]) < 1), 'go to a saved pose', JSON.stringify([shown, want]));

  // 10. jog
  const x0 = +$('#tx').value;
  const plus = $('#jogXYZ').querySelectorAll('button')[1];
  plus.dispatchEvent(new w.MouseEvent('pointerdown', { bubbles: true })); plus.dispatchEvent(new w.MouseEvent('pointerup', { bubbles: true }));
  check(+$('#tx').value === x0 + 5, 'jog X +5 mm', `${x0} -> ${$('#tx').value}`);
  input('#tx', 60); input('#ty', -160); input('#tz', 120); await sleep(1500);
  click('#jogModeJ');
  await sleep(300);
  const j1 = parseFloat(w.document.querySelector('.joint .val').textContent);
  const jplus = $('#jogJ').querySelectorAll('button')[1];
  jplus.dispatchEvent(new w.MouseEvent('pointerdown', { bubbles: true })); jplus.dispatchEvent(new w.MouseEvent('pointerup', { bubbles: true }));
  await sleep(800);
  check(Math.abs(parseFloat(w.document.querySelector('.joint .val').textContent) - (j1 + 5)) < 0.6, 'jog J1 +5°');

  // 10b. attachments: pick the vacuum tool; the target becomes its tip and the list shows it chosen
  click('#tabbtn-setup');
  const att = name => [...$('#attList').children].find(b => b.textContent.startsWith(name));
  check($('#attList').children.length === 3, 'three attachments listed');
  click(att('Vacuum suction'));
  check(att('Vacuum suction').getAttribute('aria-checked') === 'true' && /80 mm/.test(txt('#attMeta')), 'vacuum selected', txt('#attMeta'));
  check($('#attCustom').hidden, 'custom sliders hidden for a known attachment');
  $('#optEnvelope').checked = true; $('#optEnvelope').dispatchEvent(new w.Event('change'));
  click(att('Custom'));
  input('#tool', 40); input('#toolD', 30);
  check(!$('#attCustom').hidden && /40 × ⌀30/.test(txt('#attList')), 'custom size', txt('#attList'));
  click(att('Vacuum suction'));

  // 10c. work area: default front half (only the tip counts); the figure-8 runs inside it; presets change it
  click('#tabbtn-setup');
  check($('#areaOn').checked && txt('#areaCenterv').includes('front') && txt('#areaSpanv') === '180°', 'default work area: front half', txt('#areaCenterv'));
  click('#tabbtn-motion'); click('#btnDemo');
  let blockedSeen = '';
  for (let k = 0; k < 40; k++) { await sleep(100); if (/Blocked/.test(txt('#statusText'))) blockedSeen = txt('#statusText'); }
  click('#btnDemo');
  check(!blockedSeen, 'figure-8 stays inside the work area', blockedSeen);
  input('#tx', -60); input('#ty', -180); input('#tz', 120);       // a target just behind the right edge
  await until(() => /work area/.test(txt('#statusText')), 3000);
  check(/outside the work area/.test(txt('#statusText')), 'a target past the edge is refused', txt('#statusText'));
  input('#tx', -150); input('#ty', -100); input('#tz', 100);      // well behind: says why, not "out of reach"
  await until(() => /outside the work area/.test(txt('#statusText')), 3000);
  check(/target is outside the work area/.test(txt('#statusText')), 'a target behind is explained', txt('#statusText'));
  click('#tabbtn-setup');
  click([...$('#areaPresets').children].find(b => b.textContent === 'Right half'));
  await sleep(1500);
  check(!/work area/.test(txt('#statusText')), 'right half preset allows it', txt('#statusText'));
  click([...$('#areaPresets').children].find(b => b.textContent === 'Front half'));
  input('#tx', 60); input('#ty', -160); input('#tz', 120);

  // 11. connected: playback runs on the backend and the page sends no goals meanwhile
  click('#btnWs');
  await until(() => txt('#linkText').includes('live'));
  click('#tabbtn-play');
  click([...$('#recList').children].find(b => /Imported/.test(b.textContent)));
  await until(() => !$('#recPlay').disabled);
  check(txt('#playWhere') === 'on the arm', 'Play targets the arm when connected', txt('#playWhere') + ' / ' + txt('#linkText') + ' / ' + txt('#wsNote') + ' / ' + txt('#statusText'));
  wsSent = [];
  click('#recPlay');
  await until(() => remote);
  const pb = calls.filter(c => c[1] === '/playback').at(-1)[2];
  check(pb.recording && pb.timed === true && pb.speed <= 150, 'POST /api/playback', JSON.stringify(pb));
  await until(() => /Stop playback/.test(txt('#recPlay')));
  await sleep(300);
  check(remote && !wsSent.some(m => m.type === 'goal' || m.type === 'target'), 'no goals or targets sent during backend playback', JSON.stringify(wsSent.slice(0, 3)));
  await until(() => !remote && !/Stop playback/.test(txt('#recPlay')), 5000);   // (the note may still say so from before)
  await sleep(300);
  check(txt('#playNote') === 'Playback finished.', 'backend playback end reported', txt('#playNote'));

  // 11a. driving: the Move target goes to the arm as a point, which the arm solves and moves to
  click('#tabbtn-motion');
  wsSent = [];
  input('#tx', 180); input('#ty', -40); input('#tz', 110);
  await until(() => armIk && armIk.target && armIk.target[0] === 180 && armIk.arrived, 5000);
  const tg = wsSent.filter(m => m.type === 'target').at(-1);
  check(tg && tg.xyz && tg.xyz[0] === 180 && Number.isInteger(tg.epoch) && !wsSent.some(m => m.type === 'goal'),
        'target sent as a point, no goals', JSON.stringify(wsSent.map(m => m.type + (m.xyz ? ' xyz ' + m.xyz : m.angles ? ' ang' : ''))));
  check(armIk && armIk.reached && armIk.arrived, 'the arm solved and got there', JSON.stringify(armIk));
  await until(() => /At target/.test(txt('#statusText')), 3000);
  check(/At target/.test(txt('#statusText')), 'the page shows the arm\'s solution', txt('#statusText'));
  const nIk = ikSent.length; await sleep(500);
  check(ikSent.length === nIk, 'no /ws/ik solving while the arm solves', `${nIk} -> ${ikSent.length}`);

  // 11b. the backend's saved attachment wins on connect; picking one tells the backend
  click('#tabbtn-setup');
  click([...$('#attList').children].find(b => b.textContent.startsWith('Vacuum suction')));
  await until(() => wsSent.some(m => m.type === 'set_tool'), 2000);
  check(wsSent.some(m => m.type === 'set_tool' && m.attachment === 'vacuum'), 'set_tool sent', JSON.stringify(wsSent.filter(m => m.type === 'set_tool')));

  // 11c. changing the work area tells the backend
  click('#tabbtn-setup');
  $('#areaSpan').value = 150; $('#areaSpan').dispatchEvent(new w.Event('input'));
  await until(() => wsSent.some(m => m.type === 'set_area' && m.span === 150), 2000);
  check(wsSent.some(m => m.type === 'set_area' && m.span === 150 && m.center === 0), 'set_area sent', JSON.stringify(wsSent.filter(m => m.type === 'set_area')));

  // 12. stall guard toggle and fault display
  $('#optStall').checked = false; $('#optStall').dispatchEvent(new w.Event('change'));
  check(wsSent.some(m => m.type === 'set_stall_guard' && m.on === false), 'stall guard toggle sent');
  armState = { stopped: true, fault: 'J2 stalled 9° short of its goal', epoch: armState.epoch };
  await until(() => txt('#statusText').includes('stalled'));
  check(txt('#statusText').includes('J2 stalled'), 'fault shown in the status bar', txt('#statusText'));

  // 13. goals carry the epoch: resuming bumps it, the page re-reads the pose and only sends the new one
  check(wsSent.filter(m => m.type === 'target').every(m => Number.isInteger(m.epoch)), 'targets carry an epoch');
  const before = armState.epoch; wsSent = [];
  click('#btnStop');
  await until(() => wsSent.some(m => m.type === 'target'), 3000);
  const goals = wsSent.filter(m => m.type === 'target');
  check(wsSent[0] && wsSent[0].type === 'resume', 'resume sent', JSON.stringify(wsSent[0]));
  check(goals.length && goals.every(m => m.epoch === before + 1), 'targets after resume use the new epoch', JSON.stringify(goals.map(m => m.epoch)));
  check(goals[0].angles && !goals[0].xyz, 'after re-reading the pose it holds it (a joint target)', JSON.stringify(goals[0]));

  // 13b. hand-guide: the sim mirrors the real arm
  click('#btnLimp');
  pose = [15, 0, 0, 0, 0, 0];
  await until(() => Math.abs(parseFloat(w.document.querySelector('.joint .val').textContent) - 15) < 0.6, 3000);
  check(Math.abs(parseFloat(w.document.querySelector('.joint .val').textContent) - 15) < 0.6, 'hand-guide: the sim follows the arm',
        w.document.querySelector('.joint .val').textContent);
  click('#btnLimp');
  await until(() => armTorque, 2000);

  // 13c. what happened on the real arm: a joint past its servo's wrap point. The page sends nothing and says why
  const why = 'J4 reads 141°, outside the -150° to 46° its servo can reach, so the arm won\'t move it.';
  armState = { stopped: true, fault: why, epoch: armState.epoch }; armRange = why; tickOver = { 3: 3063 }; pose = [0, 0, 0, 141, 0, 0];
  await until(() => /J4 reads 141/.test(txt('#statusText')), 3000);
  wsSent = [];
  input('#tx', 170); await sleep(600);
  check(/J4 reads 141/.test(txt('#statusText')), 'status says why the arm won\'t move', txt('#statusText'));
  check(!wsSent.some(m => m.type === 'target' || m.type === 'goal'), 'nothing sent to move it', JSON.stringify(wsSent));
  check(!$('#btnCalib').hidden, 'the Calibrate pill appears');

  // 13d. the calibration wizard fixes it, step by step
  const title = () => txt('#wizTitle'), next = () => click('#wizNext');
  click('#btnCalib');
  check(!$('#wiz').hidden && title() === 'Set up the arm', 'wizard opens', title());
  await until(() => !$('#wizNext').disabled, 2000);
  check($('#wzReq').querySelectorAll('.wz-i.ok').length === 3, 'start: link, servos and no playback checked');
  next();
  check(title() === 'Hold the arm' && $('#wzTorqueOff').disabled, 'hold: torque-off waits for "I\'m holding it"');
  $('#wzHeld').checked = true; $('#wzHeld').dispatchEvent(new w.Event('change'));
  await until(() => !$('#wzTorqueOff').disabled, 1000);
  click('#wzTorqueOff');
  await until(() => !armTorque && !$('#wizNext').disabled, 3000);
  check(wsSent.some(m => m.type === 'torque' && m.on === false) && !$('#wizNext').disabled, 'torque off sent, Next enabled');
  pose = [0, 0, 0, 141, 0, 0]; tickOver = { 3: 3063 };
  next();
  check(title() === 'Pose it straight up' && $('#wzHints').children.length === 6, 'pose: six joint hints');
  next();
  check(title() === 'Centre the servos', 'centre step', title());
  await until(() => $('#wzC3') && /from centre/.test(txt('#wzC3')), 2000);
  const marked = [...w.document.querySelectorAll('#wzCentre input')].map(b => b.checked);
  check(marked[3] && marked.filter(Boolean).length === 1, 'only the far servo (J4) is marked', JSON.stringify(marked));
  check(txt('#wizNext') === 'Skip', 'it can be skipped', txt('#wizNext'));
  await until(() => !$('#wzRecenter').disabled, 2000);
  click('#wzRecenter');
  await until(() => /J4 centred/.test(txt('#wzCentreState')), 3000);
  check(wsSent.some(m => m.type === 'recenter' && JSON.stringify(m.joints) === '[3]'), 'recenter sent for J4');
  check(/J4 centred/.test(txt('#wzCentreState')) && txt('#wizNext') === 'Next', 'result shown', txt('#wzCentreState'));
  pose = [0, 0, 0, 1, 0, 0];
  next();
  check(title() === 'Save the zero' && $('#wizNext').disabled, 'zero: Next waits for the zero');
  click('#wzZero');
  await until(() => !$('#wizNext').disabled, 3000);
  check(wsSent.some(m => m.type === 'set_zero') && !$('#wizNext').disabled, 'set_zero sent and saved');
  next();
  check(title() === 'Check each joint\'s direction' && /J1/.test(txt('#wzDirLead')), 'directions: starts with J1', txt('#wzDirLead'));
  pose = [20, 0, 0, 0, 0, 0];                                    // J1 turned the right way
  await until(() => /Right way round/.test(txt('#wzDirState')), 3000);
  check(/Right way round/.test(txt('#wzDirState')), 'J1 counts the right way', txt('#wzDirState'));
  pose = [0, 0, 0, 0, 0, 0];
  await until(() => /J2/.test(txt('#wzDirLead')), 3000);
  pose = [0, -20, 0, 0, 0, 0];                                   // J2 counts the other way
  await until(() => wsSent.some(m => m.type === 'set_dir'), 3000);
  check(wsSent.some(m => m.type === 'set_dir' && m.joint === 1 && m.dir === -1), 'J2 reversed', JSON.stringify(wsSent.filter(m => m.type === 'set_dir')));
  await until(() => /reversed now/.test(txt('#wzDirState')), 3000);
  pose = [0, 0, 0, 0, 0, 0];
  await until(() => /J3/.test(txt('#wzDirLead')), 3000);
  for (let k = 0; k < 4; k++) { click('#wzDirSkip'); await sleep(50); }
  await until(() => !$('#wizNext').disabled, 2000);
  check(!$('#wizNext').disabled && w.document.querySelectorAll('#wzDirDots .wz-dot.good').length === 2, 'the rest skipped', txt('#wzDirDots'));
  next();
  check(title() === 'Torque back on', 'torque step');
  click('#wzTorqueOn');
  await until(() => armTorque && !$('#wzResume').hidden, 3000);
  check(!$('#wzResume').hidden, 'the arm was stopped: Resume offered');
  click('#wzResume');
  await until(() => !$('#wizNext').disabled, 3000);
  next();
  check(title() === 'All set' && /Re-centred J4/.test(txt('#wizBody')) && /Reversed J2/.test(txt('#wizBody')), 'summary', txt('#wizBody'));
  check(!wsSent.some(m => m.type === 'target' || m.type === 'goal'), 'nothing drove the arm during the wizard',
        JSON.stringify(wsSent.filter(m => m.type === 'target' || m.type === 'goal')).slice(0, 200));
  click('#wizNext');
  check($('#wiz').hidden && $('#btnCalib').hidden, 'Finish closes it');

  // 14. a refused command is shown but keeps the link; a refused goal is silent
  sock.emit({ type: 'error', code: 'refused', ref: 'goal', message: 'Stale goal' });
  sock.emit({ type: 'error', code: 'bad_request', ref: 'set_area', message: 'Needs enabled (bool)' });
  check(txt('#wsNote').includes('Needs enabled'), 'non-fatal error shown', txt('#wsNote'));
  check(txt('#btnWs') === 'Disconnect', 'non-fatal error keeps the link', txt('#btnWs'));
  sock.emit({ type: 'error', code: 'auth', message: 'nope' });
  check(txt('#btnWs') === 'Connect', 'fatal error ends the link', txt('#btnWs'));

  check(!errors.length, 'no page errors', errors.join(' | '));
  process.exit(failed ? 1 : 0);
})().catch(e => { console.error(e); process.exit(1); });
