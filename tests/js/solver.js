// Runs the page's per-frame solving (ik.solveFrame + ik.planMove, as main.js does, with synchronised servos)
// on a fixed set of targets in the default work area, and prints what happened as JSON (tests/test_page.py).
// Each target B is tried from the zero pose, then dragged to from zero, and jumped/dragged to from an arbitrary
// collision-free pose (as after hand-guide, a saved pose or jog). The servos must never collide on the way;
// the soft work-area limits may only be grazed (sampling can clip a boundary between two checked poses).
const { pureBlocks } = require('./page');
const { Vector3 } = require('three');
const K = pureBlocks();
const { N, DEG, JOINTS, fk, makeFK, checkPose, checkPath, solveFrame, planMove } = K;
K.setArea({ ...K.DEFAULT_AREA });   // the front half, clear of the base
const F = makeFK(), FPS = 60, VMAX = 90 * DEG / FPS;

function run(q0, B, how, orient) {
  const qIK = q0.slice(), qCmd = q0.slice(), servo = q0.slice(), mem = { key: '', t: -1e9, tries: 0 };
  fk(q0, F); const A = F.tcp.clone();
  for (let f = 0, t = 0; f < 420; f++) {
    t += 1000 / FPS;
    const tg = how === 'drag' && f < 60 ? A.clone().lerp(B, (f + 1) / 60) : B;
    solveFrame(qIK, tg, orient, servo.slice(), mem, t);
    if (checkPath(servo, qCmd, 16, DEG)) for (let i = 0; i < N; i++) qCmd[i] = servo[i];
    if (!checkPose(qIK)) {
      const route = planMove(servo, qIK);
      if (route) { const next = route.length ? route[0] : qIK; for (let i = 0; i < N; i++) qCmd[i] = next[i]; }
    }
    let far = 0; for (let i = 0; i < N; i++) far = Math.max(far, Math.abs(qCmd[i] - servo[i]));
    for (let i = 0; i < N; i++) {
      const e = qCmd[i] - servo[i], st = VMAX * (far > 0 ? Math.abs(e) / far : 1);
      servo[i] += Math.abs(e) < st ? e : Math.sign(e) * st;
    }
    const hit = checkPose(servo);
    if (hit) {
      if (!/work area|close to the base/.test(hit)) return { ok: false, collided: hit };
      out.graze_mm = Math.max(out.graze_mm, areaDepth(servo) * 1000);
    }
  }
  fk(servo, F);
  const ori = Math.acos(Math.max(-1, Math.min(1, -F.dir.z)));
  return { ok: F.tcp.distanceTo(B) < 0.003 && (!orient || ori < 3 * DEG), collided: null };
}

// how far (m) the tip is past the nearest work-area edge (default area: front half, keep-out around the base)
function areaDepth(q) {
  fk(q, F); const p = F.tcp, r = Math.hypot(p.x, p.y), A = K.area, C = K.COLLISION;
  let d = 0;
  if (r < A.base_mm / 1000 && p.z < C.BASE_KEEPOUT_TOP) d = Math.max(d, Math.min(A.base_mm / 1000 - r, C.BASE_KEEPOUT_TOP - p.z));
  const off = Math.abs(((Math.atan2(p.y, p.x) / DEG - A.center + 180) % 360 + 360) % 360 - 180) - A.span / 2;
  if (off > 0 && r >= C.AREA_CORE) d = Math.max(d, r * Math.sin(Math.min(off, 90) * DEG));
  return d;
}

let seed = 12345;
const rnd = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
const point = () => { const a = (rnd() - 0.5) * Math.PI * 0.9, r = 0.08 + rnd() * 0.2, z = 0.02 + rnd() * 0.25;
  return new Vector3(Math.cos(a) * r, Math.sin(a) * r, z); };
const randomPose = () => { for (;;) { const q = JOINTS.map(j => (rnd() * 2 - 1) * Math.min(j.max, 2.2)); if (!checkPose(q)) return q; } };

const n = Number(process.argv[2] || 30), out = { cases: {}, collided: [], graze_mm: 0 };
for (let k = 0; k < n; k++) {
  const B = point(), zero = new Array(N).fill(0), pose = randomPose();
  for (const orient of [true, false]) {
    if (!run(zero, B, 'jump', orient).ok) continue;          // only targets reachable from the zero pose
    for (const [name, q0, how] of [['zero/drag', zero, 'drag'], ['pose/jump', pose, 'jump'], ['pose/drag', pose, 'drag']]) {
      const r = run(q0, B, how, orient), c = out.cases[name] = out.cases[name] || { n: 0, ok: 0 };
      c.n++; if (r.ok) c.ok++;
      if (r.collided) out.collided.push({ name, orient, why: r.collided });
    }
  }
}
// a move whose straight joint-space path would put J6 into the table, but that can go up and over
const a = [28, -84, -89, 82, 66, 113].map(v => v * DEG), b = [19, -82, 46, -83, -87, -11].map(v => v * DEG);
const route = planMove(a, b);
out.detour = { clear: !checkPose(a) && !checkPose(b), straight: checkPath(a, b), vias: route && route.length,
               legs: route && [a, ...route, b].slice(1).map((q, k, l) => checkPath(k ? l[k - 1] : a, q)) };
process.stdout.write(JSON.stringify(out));
