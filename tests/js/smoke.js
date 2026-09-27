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
const db = { recordings: {}, sequences: {}, poses: {}, programs: {}, obstacles: [] }, calls = [];
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
    remote = { name: body.program ? db.programs[body.program].name : body.recording ? db.recordings[body.recording].name : db.sequences[body.sequence].name, until: Date.now() + 800, body };
    return res(200, { success: true });
  }
  if (coll === 'obstacles') {                         // the Workspace's shapes
    if (m === 'PUT') db.obstacles = JSON.parse(JSON.stringify(body.obstacles));
    return res(200, { obstacles: db.obstacles });
  }
  if (coll === 'programs' && key === 'compile') {   // the real compiler, through ik_stdio.py
    const r = await pySolve('compile', { type: 'compile', blocks: body.blocks });
    return r.type === 'error' ? res(422, { detail: r.message }) : res(200, r);
  }
  if (coll === 'programs' && m !== 'GET' && m !== 'DELETE') body.blocks = JSON.parse(JSON.stringify(body.blocks));
  const store = db[coll];
  if (!store) return res(404, { detail: 'nope' });
  if (!key) {
    if (m === 'GET') return res(200, Object.values(store).map(x => coll === 'recordings' ? summary(x) : coll === 'programs' ? { ...x, blocks: x.blocks.length } : x));
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
    setTimeout(() => {
      this.onopen();                                   // (the page sends its auth message from onopen)
      if (this.badPw) { this.emit({ type: 'error', code: 'auth', message: 'Wrong or missing password.' }); this.close(); this.onclose(); return; }
      this.emit({ type: 'hello', protocol: 4 }); this.emit(cfg); }, 5);
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
    if (m.type === 'auth') this.badPw = m.password !== 'pw';
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
  check(!$('#landing').hidden && $('#landForm').hidden === false, 'opens on the connect screen');
  click('#landSim');                                              // the simulator first, without the arm
  await until(() => $('#landing').hidden, 2000);
  check($('#landing').hidden && $('#calAsk').hidden, 'without the arm: straight to the scene, no calibration question');
  await until(() => !$('#tour').hidden, 2000);
  check(!$('#tour').hidden && txt('#tourStep') === '1 of 6' && txt('#tourTitle') === 'Three views', 'the first time, a tour starts', txt('#tourStep'));
  click('#tourNext');
  check(txt('#tourStep') === '2 of 6' && txt('#tourTitle') === 'The connection', 'the tour steps on');
  click('#tourSkip');
  check($('#tour').hidden && w.localStorage.getItem('mycobot-tour') === 'done', 'skipped, and remembered');
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

  // 7. Motion Studio: blocks, compiled by the real backend code, previewed in the sandbox, saved, reopened
  const blk = type => [...w.document.querySelectorAll('#stProg .blk')].filter(e => e.classList.contains('blk-' + type));
  const inputOn = (el, v) => { el.value = v; el.dispatchEvent(new w.Event('input', { bubbles: true })); };
  const compiledOk = () => /\d s ·/.test(txt('#stState'));
  click('#viewStudio');
  check(w.document.body.classList.contains('studio') && !$('#studio').hidden && $('#stPalette').children.length === 6, 'Studio opens with six block types');
  check(blk('home').length === 1 && $('#stName').value === 'Untitled motion', 'a new motion starts at zero');
  click($('#stPalette').querySelector('.pal-pose'));
  check(blk('pose').length === 1 && blk('pose')[0].classList.contains('sel') && blk('pose')[0].querySelectorAll('input[type=range]').length === 7,
        'Move to pose: added, selected, six joint sliders and a speed');
  inputOn(blk('pose')[0].querySelector('input[data-j="0"]'), 40);
  inputOn(blk('pose')[0].querySelector('input[data-j="1"]'), 20);
  click($('#stPalette').querySelector('.pal-wait'));
  click($('#stPalette').querySelector('.pal-led'));
  click($('#stPalette').querySelector('.pal-repeat'));
  click($('#stPalette').querySelector('.pal-point'));             // the repeat is selected: into it
  const rep = blk('repeat')[0];
  check(rep && rep.querySelector('.binner .blk-point'), 'a block added to a selected repeat goes inside it');
  click($('#stPalette').querySelector('.pal-home'));
  await until(compiledOk, 5000);
  check(compiledOk(), 'compiled on the backend: no problems', txt('#stState'));
  check(!$('#stPlay').disabled && !$('#stRun'), 'Preview on; no Run button in the Studio (motions play from the Play tab)');
  click('#stPlay');
  await sleep(700);
  check(+$('#stTime').value > 0.3 && w.document.querySelector('#stProg .blk.running'), 'preview plays and marks the running block', $('#stTime').value);
  click('#stStop');
  // a point out of reach is marked, and the program can't run
  const pt = blk('point')[0];
  click(pt.querySelector('.bhead'));
  inputOn(blk('point')[0].querySelector('input[data-k="0"]'), 900);
  await until(() => blk('point')[0].classList.contains('bad'), 5000);
  check(/reach/.test(blk('point')[0].querySelector('.bprob').textContent) && $('#stPlay').disabled === false, 'an unreachable point is marked', txt('#stState'));
  inputOn(blk('point')[0].querySelector('input[data-k="0"]'), 170);
  await until(() => !w.document.querySelector('#stProg .blk.bad') && compiledOk(), 5000);
  check(!w.document.querySelector('#stProg .blk.bad'), 'fixed again', [...w.document.querySelectorAll('#stProg .bprob')].map(e => e.textContent).join(' | ') + ' ' + JSON.stringify(blk('point')[0] && [...blk('point')[0].querySelectorAll('input[data-k]')].map(i => i.value)));
  // reorder, copy, delete
  const order0 = [...w.document.querySelectorAll('#stProg > .blk')].map(e => e.dataset.id);
  click(blk('wait')[0].querySelector('[data-act="up"]'));
  const after = [...w.document.querySelectorAll('#stProg > .blk')].map(e => e.dataset.id);
  check(after[1] === order0[2] && after[2] === order0[1], 'move up reorders', JSON.stringify([order0, after]));
  click(blk('led')[0].querySelector('[data-act="copy"]'));
  check(blk('led').length === 2, 'duplicate');
  click(blk('led')[1].querySelector('[data-act="del"]'));
  check(blk('led').length === 1, 'delete');
  // save, start again, reopen
  $('#stName').value = 'Pick demo'; $('#stName').dispatchEvent(new w.Event('input'));
  click('#stSave');
  await until(() => Object.values(db.programs).some(p => p.name === 'Pick demo'), 3000);
  const saved2 = Object.values(db.programs).find(p => p.name === 'Pick demo');
  check(saved2 && saved2.blocks.length === 5 && saved2.blocks[4].type === 'repeat' && saved2.blocks[4].blocks.map(b => b.type).join() === 'point,home',
        'saved with its blocks', JSON.stringify(saved2 && saved2.blocks.map(b => b.type)));
  await until(() => [...$('#stTabs').children].some(t => t.textContent === 'Pick demo' && t.classList.contains('on')), 3000);
  check([...$('#stTabs').children].some(t => t.textContent === 'Pick demo' && t.classList.contains('on')), 'its tab, highlighted', txt('#stTabs'));
  click([...$('#stTabs').children].find(t => t.classList.contains('st-new')));
  check(blk('pose').length === 0 && $('#stName').value === 'Untitled motion', '+ New starts another motion');
  click(blk('home')[0].querySelector('.bhead'));
  click($('#stPalette').querySelector('.pal-wait'));
  $('#stName').value = 'Second'; $('#stName').dispatchEvent(new w.Event('input'));
  click('#stSave');
  await until(() => Object.values(db.programs).some(p => p.name === 'Second'), 3000);
  await until(() => $('#stTabs').children.length === 3, 3000);
  check(Object.keys(db.programs).length === 2 && $('#stTabs').children.length === 3, 'two motions saved, a tab each (+ New)', txt('#stTabs'));
  click([...$('#stTabs').children].find(t => t.textContent === 'Pick demo'));
  await until(() => $('#stName').value === 'Pick demo', 3000);
  check(blk('pose').length === 1 && blk('point').length === 1 && $('#stName').value === 'Pick demo', 'reopened from its tab');
  check(/Saved "Pick demo"/.test(txt('#toasts')) && w.localStorage.getItem('mycobot-view') === 'studio', 'saving says so (a toast); the view is remembered', txt('#toasts'));
  // undo and redo: a new block goes away and comes back; a slider move is one step
  check($('#stUndo').disabled && $('#stRedo').disabled, 'nothing to undo right after opening');
  const nBlocks = () => w.document.querySelectorAll('#stProg .blk').length, n0 = nBlocks();
  click($('#stPalette').querySelector('.pal-wait'));
  check(nBlocks() === n0 + 1 && !$('#stUndo').disabled, 'a change can be undone');
  click('#stUndo');
  check(nBlocks() === n0 && !$('#stRedo').disabled, 'undo removes it');
  click('#stRedo');
  check(nBlocks() === n0 + 1, 'redo brings it back');
  click('#stUndo');
  click(blk('pose')[0].querySelector('.bhead'));
  const sj1 = () => +blk('pose')[0].querySelector('input[data-j="0"]').value, sj1a = sj1();
  for (const v of [10, 20, 30, 45]) inputOn(blk('pose')[0].querySelector('input[data-j="0"]'), v);
  check(sj1() === 45, 'slider moved');
  w.dispatchEvent(new w.KeyboardEvent('keydown', { key: 'z', ctrlKey: true, bubbles: true }));
  check(sj1() === sj1a, 'Ctrl+Z undoes the whole slider move in one step', `${sj1a} -> ${sj1()}`);
  w.dispatchEvent(new w.KeyboardEvent('keydown', { key: 'z', ctrlKey: true, shiftKey: true, bubbles: true }));
  check(sj1() === 45, 'Ctrl+Shift+Z redoes it');
  // a note on a block
  click(blk('point')[0].querySelector('[data-act="note"]'));
  const ni = $('#stProg .bnote-in');
  check(!!ni, 'the note button opens a field');
  ni.value = 'pick up the part'; ni.dispatchEvent(new w.KeyboardEvent('keydown', { key: 'Enter', bubbles: true }));
  check(blk('point')[0].querySelector('.bnote') && /pick up the part/.test(blk('point')[0].querySelector('.bnote').textContent), 'the note shows on the block');
  // the timeline: a segment per top-level block; clicking one goes there
  await until(compiledOk, 5000);
  const segs = [...w.document.querySelectorAll('#stSegs .sseg')];
  check(segs.length === w.document.querySelectorAll('#stProg > .blk').length && segs.some(s => /pick up the part/.test(s.title)) === false,
        'a timeline segment per top-level block', `${segs.length} segments`);
  click(segs[2]);
  check(blk('pose')[0].classList.contains('sel') || w.document.querySelector(`#stProg .blk.sel`).dataset.id === segs[2].dataset.id, 'clicking a segment selects its block');
  click('#stSave');
  await until(() => Object.values(db.programs).find(p => p.name === 'Pick demo').blocks[4].blocks[0].note === 'pick up the part', 3000);
  check(Object.values(db.programs).find(p => p.name === 'Pick demo').blocks[4].blocks[0].note === 'pick up the part', 'the note is saved');
  // duplicate, export, import (the ⋯ menu)
  click('#stMore');
  check(!$('#stMenu').hidden, 'the menu opens');
  click('#stmDuplicate');
  await until(() => $('#stName').value === 'Pick demo copy', 3000);
  check(Object.values(db.programs).some(p => p.name === 'Pick demo copy') && $('#stName').value === 'Pick demo copy', 'duplicate makes and opens a copy');
  click('#stmExport');
  check(w.downloads.at(-1) === 'Pick_demo_copy.motion.json', 'export offers a file', JSON.stringify(w.downloads));
  const mf = new w.File([JSON.stringify({ format: 'mycobot280-motion', version: 1, name: 'From a file', blocks: saved2.blocks })], 'x.motion.json');
  Object.defineProperty($('#stFile'), 'files', { value: [mf], configurable: true });
  $('#stFile').dispatchEvent(new w.Event('change'));
  await until(() => $('#stName').value === 'From a file', 3000);
  check($('#stName').value === 'From a file' && /Imported "From a file"/.test(txt('#toasts')), 'import saves and opens it', txt('#toasts'));
  click([...$('#stTabs').children].find(t => t.textContent === 'Pick demo'));
  await until(() => $('#stName').value === 'Pick demo', 3000);
  click('#viewArm');
  check(!w.document.body.classList.contains('studio') && $('#studio').hidden, 'back to the arm view');
  // the motions play from the Play tab (here in the simulation: offline)
  click('#tabbtn-play');
  await until(() => $('#progList').children.length === 4, 3000);
  check($('#progList').children.length === 4 && /Pick demo/.test(txt('#progList')), 'the Play tab lists the motions', txt('#progList'));
  click([...$('#progList').children].find(b => /Pick demo/.test(b.textContent)));
  await until(() => /Path clear/.test(txt('#playNote')) && !$('#recPlay').disabled, 5000);
  check(!$('#recPlay').disabled && txt('#playName') === 'Pick demo' && !$('#progEditBtn').hidden, 'selected: compiled, path clear, Play on', txt('#playNote'));
  click('#recPlay');
  check(/simulation/.test(txt('#playNote')), 'plays in the simulation', txt('#playNote'));
  await sleep(600);
  check(!$('#nowPlaying').hidden && txt('#npName') === 'Pick demo' && /simulation/.test(txt('#npSub')), 'the now-playing bar shows it', txt('#npName') + ' / ' + txt('#npSub'));
  click('#recPlay');                                              // (it's long: stop it)
  await until(() => /stopped/i.test(txt('#playNote')), 3000);
  check(/stopped/i.test(txt('#playNote')), 'and stops', txt('#playNote'));
  click('#progEditBtn');                                          // Edit in Studio
  await until(() => w.document.body.classList.contains('studio'), 2000);
  check(w.document.body.classList.contains('studio') && $('#stName').value === 'Pick demo', 'Edit in Studio opens it there');
  click('#viewArm');
  // 7b. the Workspace: shapes the arm keeps clear of, saved on the backend, used by every check
  click('#viewWorkspace');
  check(w.document.body.classList.contains('workspace') && !$('#workspace').hidden && $('#wsPalette').children.length === 3, 'Workspace opens with three shapes to add');
  click($('#wsPalette').querySelector('[data-shape="box"]'));
  check($('#wsList').children.length === 1 && $('#wsName').value === 'Box 1' && $('#wsProps').querySelectorAll('input[data-f="size"]').length === 3,
        'a box is added and selected, with its size fields', txt('#wsList'));
  await until(() => db.obstacles.length === 1, 3000);
  check(db.obstacles.length === 1 && db.obstacles[0].shape === 'box' && db.obstacles[0].pos[2] === 50, 'saved on the backend, sitting on the table', JSON.stringify(db.obstacles));
  const wsIn = (f, k) => $('#wsProps').querySelector(`input[data-f="${f}"][data-k="${k}"]`);
  inputOn(wsIn('size', 0), 150); wsIn('size', 0).dispatchEvent(new w.Event('change', { bubbles: true }));
  await until(() => db.obstacles[0] && db.obstacles[0].size[0] === 150, 3000);
  check(db.obstacles[0].size[0] === 150, 'its width changed and saved');
  click($('#wsPalette').querySelector('[data-shape="cylinder"]'));
  check($('#wsProps').querySelectorAll('input[data-f="size"]').length === 2, 'a cylinder has a diameter and a height');
  inputOn(wsIn('size', 0), 60);
  await until(() => db.obstacles.length === 2 && db.obstacles[1].size[0] === 60, 3000);
  check(db.obstacles[1].size[1] === 60, 'a cylinder stays round');
  // put it where the arm is (the zero pose, offline): flagged
  inputOn(wsIn('pos', 0), 0); inputOn(wsIn('pos', 1), -40); inputOn(wsIn('pos', 2), 300);
  wsIn('pos', 2).dispatchEvent(new w.Event('change', { bubbles: true }));
  check(/touches it/.test(txt('#wsProps')) && $('#wsList').querySelector('.ws-flag'), 'where the arm is: flagged', txt('#wsProps'));
  check(/in the way/.test(txt('#wsCount')), 'the count says so', txt('#wsCount'));
  click('#wsUndo');
  check(!/touches it/.test(txt('#wsProps')) && $('#wsList').children.length === 2, 'undo moves it back (the three quick edits are one step)');
  click($('#wsProps').querySelector('[data-act="dup"]'));
  check($('#wsList').children.length === 3 && /Cylinder 2/.test(txt('#wsList')), 'duplicate', txt('#wsList'));
  click($('#wsProps').querySelector('[data-act="del"]'));
  check($('#wsList').children.length === 2, 'delete');
  click('#wsMode-rotate');
  check($('#wsMode-rotate').getAttribute('aria-pressed') === 'true' && $('#wsMode-translate').getAttribute('aria-pressed') === 'false', 'handle modes');
  click([...$('#wsList').children].find(b => /Cylinder 1/.test(b.textContent)));
  inputOn(wsIn('rot', 0), 90); inputOn(wsIn('pos', 2), 500);
  click($('#wsProps').querySelector('[data-act="table"]'));
  const rad = +wsIn('size', 0).value / 2;
  check(+wsIn('pos', 2).value === rad, 'On the table: a lying cylinder rests on its side (its radius up)', `${wsIn('pos', 2).value} vs ${rad}`);
  await until(() => db.obstacles.length === 2 && db.obstacles[1].rot[0] === 90 && db.obstacles[1].pos[2] === rad, 3000);
  check(db.obstacles[1].pos[2] === rad, 'and saved');
  // the solver has them too (they're in the work area it's sent)
  await sleep(300);
  const lastSettings = ikSent.filter(m => m.type === 'settings').at(-1);
  check(lastSettings && lastSettings.area.obstacles && lastSettings.area.obstacles.length === 2, 'the solver gets the obstacles', JSON.stringify(lastSettings && lastSettings.area));
  // clear them again: the steps below play recordings that go where these are
  while ($('#wsList').querySelector('.ws-item')) { click($('#wsList').querySelector('.ws-item')); click($('#wsProps').querySelector('[data-act="del"]')); }
  await until(() => db.obstacles.length === 0, 3000);
  check(db.obstacles.length === 0, 'all deleted, and saved');
  click('#viewArm');
  // the recording the sequence step below uses (it used to come from waypoints)
  const pts = saved.frames, ptsId = id();
  db.recordings[ptsId] = { id: ptsId, created: Date.now() / 1000, name: 'Points', frames: pts, events: [], return_zero: false };
  click('#recRefresh');

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
  await until(() => txt('#playName') === 'Show' && !$('#recPlay').disabled);
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

  // 11b. a Studio motion plays on the arm from the Play tab, through the backend's playback
  click('#tabbtn-play');
  click([...$('#progList').children].find(b => /Pick demo/.test(b.textContent)));
  await until(() => !$('#recPlay').disabled && txt('#playWhere') === 'on the arm', 5000);
  click('#recPlay');
  await until(() => remote && remote.body.program, 3000);
  check(remote && remote.body.program === saved2.id && remote.name === 'Pick demo', 'POST /api/playback with the program', JSON.stringify(remote && remote.body));
  click('#tabbtn-motion');
  await until(() => !remote && !/Stop playback/.test(txt('#recPlay')), 5000);
  await sleep(300);

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
  check(title() === 'Hold the arm' && $('#wizNext').disabled && armTorque, 'hold: waits for "I\'m holding it"');
  $('#wzHeld').checked = true; $('#wzHeld').dispatchEvent(new w.Event('change'));
  await until(() => !armTorque && !$('#wizNext').disabled, 3000);
  check(wsSent.some(m => m.type === 'torque' && m.on === false) && !$('#wizNext').disabled, 'ticking it turns torque off, Next enabled');
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
  check(title() === 'Torque back on', 'torque on before the direction test', title());
  click('#wzTorqueOn');
  await until(() => armTorque && !$('#wzResume').hidden, 3000);
  check(!$('#wzResume').hidden, 'the arm was stopped: Resume offered');
  click('#wzResume');
  await until(() => !$('#wizNext').disabled, 3000);
  next();
  check(title() === 'Check which way each servo turns' && /J1/.test(txt('#wzDirLead')) && /press Test/.test(txt('#wzDirLead')),
        'directions: the arm tests J1 first', txt('#wzDirLead'));
  await until(() => !$('#wzTest').hidden, 2000);
  wsSent = [];
  click('#wzTest');                                              // the arm turns J1 +15° and back
  await until(() => !$('#wzSame').hidden, 5000);
  const g = wsSent.filter(m => m.type === 'goal');
  check(g.length === 2 && g[0].angles[0] === 15 && g[1].angles[0] === 0 && g[0].speed <= 20 && g.every(m => m.epoch === armState.epoch),
        'J1 turned 15° slowly and back', JSON.stringify(g));
  check(/same way as the model/.test(txt('#wzDirState')), 'then asks which way it went', txt('#wzDirState'));
  click('#wzSame');
  await until(() => /J2/.test(txt('#wzDirLead')), 2000);
  click('#wzTest');
  await until(() => !$('#wzOpp').hidden, 5000);
  click('#wzOpp');                                               // J2 went the other way
  await until(() => /J3/.test(txt('#wzDirLead')), 3000);
  check(wsSent.some(m => m.type === 'set_dir' && m.joint === 1 && m.dir === -1), 'J2 reversed', JSON.stringify(wsSent.filter(m => m.type === 'set_dir')));
  await until(() => /J2 ↺/.test(txt('#wzDirDots')), 1000);
  check(/J2 ↺/.test(txt('#wzDirDots')), 'J2 marked reversed', txt('#wzDirDots'));
  click('#wzModeHand');                                          // J3 by hand instead
  await until(() => !$('#wzTqOff').hidden, 2000);
  check(!$('#wzTqOff').hidden && /by hand|arrow points/.test(txt('#wzDirLead')), 'by hand: asks for torque off first', txt('#wzDirState'));
  click('#wzTqOff');
  await until(() => !armTorque && /arrow points/.test(txt('#wzDirState')), 3000);
  pose = [0, 0, 20, 0, 0, 0];
  await until(() => /Right way round/.test(txt('#wzDirState')), 3000);
  check(/Right way round/.test(txt('#wzDirState')), 'J3 turned by hand the right way', txt('#wzDirState'));
  pose = [0, 0, 0, 0, 0, 0];
  await until(() => /J4/.test(txt('#wzDirLead')), 3000);
  for (let k = 0; k < 3; k++) { click('#wzDirSkip'); await sleep(50); }
  await until(() => !$('#wzTqOn').hidden, 2000);
  check(!$('#wzTqOn').hidden && $('#wizNext').disabled, 'done by hand: torque must come back on', txt('#wzDirState'));
  click('#wzTqOn');
  await until(() => !$('#wizNext').disabled, 3000);
  check(w.document.querySelectorAll('#wzDirDots .wz-dot.good').length === 3, 'three checked, three skipped', txt('#wzDirDots'));
  next();
  check(title() === 'All set' && /Re-centred J4/.test(txt('#wizBody')) && /Reversed J2/.test(txt('#wizBody')), 'summary', txt('#wizBody'));
  check(!wsSent.some(m => m.type === 'target'), 'only the direction test moved the arm during the wizard',
        JSON.stringify(wsSent.filter(m => m.type === 'target')).slice(0, 200));
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

  // 15. a fresh page: connecting from the connect screen, then the calibration question
  armState = { stopped: false, fault: null, epoch: armState.epoch }; armRange = null; tickOver = {};
  cfg = { ...cfg, calibrated: false };
  ({ w, errors } = await loadPage({ fetch, WebSocket: FakeWS }));
  await sleep(100);
  check(!$('#landing').hidden && /\/ws\/arm$/.test($('#landUrl').value), 'the connect screen, address filled in', $('#landUrl').value);
  click('#landGo');
  check(/password/.test(txt('#landErr')), 'no password: asks for it', txt('#landErr'));
  $('#landPw').value = 'wrong'; click('#landGo');
  check(!$('#landProgress').hidden && $('#landing').classList.contains('connecting'), 'connecting: the animation runs');
  await until(() => /rejected/.test(txt('#landErr')), 3000);
  check(/rejected that password/.test(txt('#landErr')) && !$('#landForm').hidden, 'a wrong password goes back to the form', txt('#landErr'));
  $('#landPw').value = 'pw'; click('#landGo');
  await until(() => $('#lsServos').dataset.state === 'ok', 3000);
  check(['#lsReach', '#lsAuth', '#lsServos'].every(id => $(id).dataset.state === 'ok') && !$('#landDone').hidden,
        'reached, password accepted, every servo reading: done', ['#lsReach', '#lsAuth', '#lsServos'].map(id => $(id).dataset.state).join());
  await until(() => $('#landing').hidden && !$('#calAsk').hidden, 3000);
  check($('#landing').hidden && txt('#linkText').includes('Connected'), 'into the 3D scene, connected', txt('#linkText'));
  check(!$('#calAsk').hidden && /hasn't been calibrated/.test(txt('#calAskText')) && !$('#calAskTag').hidden,
        'asks to calibrate (recommended: not calibrated yet)', txt('#calAskText'));
  click('#calAskGo');
  check($('#calAsk').hidden && !$('#wiz').hidden && txt('#wizTitle') === 'Set up the arm', 'Calibrate opens the wizard');
  click('#wizClose');
  check($('#wiz').hidden, 'and it can be closed');
  check(!errors.length, 'no page errors (second page)', errors.join(' | '));
  process.exit(failed ? 1 : 0);
})().catch(e => { console.error(e); process.exit(1); });
