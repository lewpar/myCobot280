// Loads the simulator page's code for tests: either the pure kinematics/player blocks, or the whole
// page in jsdom with fake WebGL, WebSocket and fetch.
const fs = require('fs'), path = require('path');
const PAGE = path.join(__dirname, '..', '..', 'src', 'backend', 'static', 'ik_sim.html');
const html = fs.readFileSync(PAGE, 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

// Kinematics, IK and collision (JOINTS .. checkPath) plus the Player class, evaluated with three.js.
function pureBlocks() {
  const THREE = require('three');
  const kin = script.slice(script.indexOf('/* ---------- Kinematics'), script.indexOf('/* ---------- Scene'));
  const pl = script.slice(script.indexOf('/* ---- Player:'), script.indexOf('/* ---- recorder ---- */'));
  return new Function('THREE',
    'const DEG=Math.PI/180;\n' + kin + pl +
    '\nreturn {JOINTS, N, DEG, LIM, fk, makeFK, checkPose, checkPath, ikIterate, ikRescue, Player, poseAt, PB, ATTACHMENTS,' +
    ' DEFAULT_AREA, setTool: (len, r) => { toolLen = len; toolR = r; }, setArea: a => { area = a; }};'
  )(THREE);
}

function loadPage({ fetch, WebSocket } = {}) {
  const { JSDOM } = require('jsdom');
  const bare = html.replace(/<script src=[^>]*><\/script>/g, '').replace(/<link[^>]*>/g, '');
  const dom = new JSDOM(bare, { runScripts: 'outside-only', pretendToBeVisual: true, url: 'http://pi:8000/sim' });
  const w = dom.window;
  w.confirm = () => true;
  w.matchMedia = () => ({ matches: false, addEventListener() {} });
  w.ResizeObserver = class { observe() {} };
  w.URL.createObjectURL = () => 'blob:x'; w.URL.revokeObjectURL = () => {};
  w.downloads = [];   // jsdom can't download: record what the page offers instead
  w.HTMLAnchorElement.prototype.click = function () { w.downloads.push(this.download); };
  if (fetch) w.fetch = fetch;
  if (WebSocket) w.WebSocket = WebSocket;
  const nm = path.join(__dirname, 'node_modules', 'three');
  w.eval(fs.readFileSync(path.join(nm, 'build', 'three.min.js'), 'utf8'));
  w.eval(fs.readFileSync(path.join(nm, 'examples', 'js', 'controls', 'OrbitControls.js'), 'utf8'));
  w.eval(fs.readFileSync(path.join(nm, 'examples', 'js', 'controls', 'TransformControls.js'), 'utf8'));
  w.THREE.WebGLRenderer = function () {
    this.domElement = w.document.createElement('canvas'); this.shadowMap = {};
    this.setPixelRatio = () => {}; this.setSize = () => {}; this.render = () => {};
  };
  const errors = [];
  w.addEventListener('error', e => errors.push(e.message));
  w.eval(script);
  return { w, errors };
}

module.exports = { pureBlocks, loadPage };
