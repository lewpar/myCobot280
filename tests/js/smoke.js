// jsdom smoke test of the simulator page against a fake backend (REST + /ws/arm).
// Exits non-zero on the first failed check. Run by tests/test_page.py.
const { loadPage } = require('./page');

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
let sock = null, pose = [0, 20, 20, 20, 0, 0], wsSent = [], armState = { stopped: false, fault: null }, playEndN = 0, playEndMsg = null;
function playEnd(msg) { playEndN++; playEndMsg = msg; }
class FakeWS {
  constructor() { sock = this; this.readyState = 1; setTimeout(() => this.onopen(), 5);
    this.iv = setInterval(() => {
      if (remote && Date.now() > remote.until) { remote = null; playEnd('Playback finished.'); }
      this.onmessage({ data: JSON.stringify({ type: 'state', angles: pose, torque: true, stopped: armState.stopped, blocked: null,
        calibrated: true, zero: [2048, 2048, 2048, 2048, 2048, 2048], dir: [1, 1, 1, 1, 1, 1], tool_mm: 0,
        limits: [[-168, 168], [-140, 140], [-150, 150], [-150, 150], [-155, 160], [-180, 180]],
        fault: armState.fault, stall_guard: true,
        playback: remote ? { name: remote.name, recording: remote.name, step: 0, steps: 1, phase: 'run', t: 0.4, duration: 1 } : null,
        play_end: { n: playEndN, message: playEndMsg } }) });
    }, 100); }
  send(m) { m = JSON.parse(m); wsSent.push(m); if (m.type === 'goal') pose = m.angles.slice(); }
  close() { clearInterval(this.iv); }
}

// ---- harness ----
const { w, errors } = loadPage({ fetch, WebSocket: FakeWS });
const $ = s => w.document.querySelector(s);
const click = s => (typeof s === 'string' ? $(s) : s).dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
const sleep = ms => new Promise(r => setTimeout(r, ms));
const txt = s => $(s).textContent;
let failed = false;
function check(cond, what, extra = '') { console.log((cond ? 'ok   ' : 'FAIL ') + what + (cond ? '' : '  ' + extra)); if (!cond) failed = true; }
async function until(fn, ms = 15000) { const t = Date.now(); while (!fn() && Date.now() - t < ms) await sleep(50); return fn(); }
const input = (s, v) => { $(s).value = v; $(s).dispatchEvent(new w.Event('input', { bubbles: true })); };

(async () => {
  $('#wsPw').value = 'pw';
  await sleep(200);

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
  input('#tx', 0); await sleep(1500);
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
  click('#btnRandom'); await sleep(1200); click('#wpAdd');
  input('#tx', 180); input('#ty', 40); await sleep(1200); click('#wpAdd');
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
  click('#jogModeJ');
  await sleep(800);
  const j1 = parseFloat(w.document.querySelector('.joint .val').textContent);
  const jplus = $('#jogJ').querySelectorAll('button')[1];
  jplus.dispatchEvent(new w.MouseEvent('pointerdown', { bubbles: true })); jplus.dispatchEvent(new w.MouseEvent('pointerup', { bubbles: true }));
  await sleep(800);
  check(Math.abs(parseFloat(w.document.querySelector('.joint .val').textContent) - (j1 + 5)) < 0.6, 'jog J1 +5°');

  // 11. connected: playback runs on the backend and the page sends no goals meanwhile
  click('#btnWs');
  await until(() => txt('#linkText').includes('live'));
  click('#tabbtn-play');
  click([...$('#recList').children].find(b => /Imported/.test(b.textContent)));
  await until(() => !$('#recPlay').disabled);
  check(txt('#playWhere') === 'on the arm', 'Play targets the arm when connected');
  wsSent = [];
  click('#recPlay');
  await until(() => remote);
  const pb = calls.filter(c => c[1] === '/playback').at(-1)[2];
  check(pb.recording && pb.timed === true && pb.speed <= 150, 'POST /api/playback', JSON.stringify(pb));
  await until(() => /Stop playback/.test(txt('#recPlay')));
  await until(() => txt('#playNote') === 'Playback finished.', 5000);
  check(txt('#playNote') === 'Playback finished.', 'backend playback end reported', txt('#playNote'));
  check(!wsSent.some(m => m.type === 'goal'), 'no goals streamed during backend playback', JSON.stringify(wsSent.slice(0, 3)));

  // 12. stall guard toggle and fault display
  $('#optStall').checked = false; $('#optStall').dispatchEvent(new w.Event('change'));
  check(wsSent.some(m => m.type === 'set_stall_guard' && m.on === false), 'stall guard toggle sent');
  armState = { stopped: true, fault: 'J2 stalled 9° short of its goal' };
  await until(() => txt('#statusText').includes('stalled'));
  check(txt('#statusText').includes('J2 stalled'), 'fault shown in the status bar', txt('#statusText'));

  check(!errors.length, 'no page errors', errors.join(' | '));
  process.exit(failed ? 1 : 0);
})().catch(e => { console.error(e); process.exit(1); });
