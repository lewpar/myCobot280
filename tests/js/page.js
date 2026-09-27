// Loads the simulator page's ES modules (src/backend/static/sim/js) for tests: either the pure
// kinematics/collision/player modules, or the whole page in jsdom with fake WebGL, WebSocket and fetch.
// esbuild bundles the modules for node and jsdom only; the page itself has no build step.
const fs = require('fs'), path = require('path');
const esbuild = require('esbuild');
const SIM = path.join(__dirname, '..', '..', 'src', 'backend', 'static', 'sim');
const JSDIR = path.join(SIM, 'js');
const NODE_MODULES = path.join(__dirname, 'node_modules');

// Kinematics and collision plus the Player class.
function pureBlocks() {
  const code = esbuild.buildSync({
    stdin: { contents: ['kinematics', 'collision', 'player'].map(m => `export * from './${m}.js';`).join('\n'),
             resolveDir: JSDIR, sourcefile: 'pure.js' },
    bundle: true, format: 'cjs', platform: 'node', write: false, nodePaths: [NODE_MODULES], logLevel: 'silent',
  }).outputFiles[0].text;
  const m = { exports: {} };
  new Function('module', 'exports', 'require', code)(m, m.exports, require);
  const K = m.exports;
  return Object.assign(Object.create(K), { setTool: K.setToolSize, setArea: K.setAreaModel });
}

// three.js with a stand-in WebGLRenderer (jsdom has no WebGL); every `import 'three'` in the page gets this
const THREE_SHIM = path.join(__dirname, 'three-shim.js');
const shimThree = {
  name: 'three-shim',
  setup(b) { b.onResolve({ filter: /^three$/ }, a => (a.importer === THREE_SHIM ? undefined : { path: THREE_SHIM })); },
};

async function loadPage({ fetch, WebSocket } = {}) {
  const { JSDOM } = require('jsdom');
  const html = fs.readFileSync(path.join(SIM, 'index.html'), 'utf8');
  const bare = html.replace(/<script[^>]*>[\s\S]*?<\/script>/g, '').replace(/<link[^>]*>/g, '');
  const bundle = (await esbuild.build({
    entryPoints: [path.join(JSDIR, 'main.js')], bundle: true, format: 'iife', write: false,
    nodePaths: [NODE_MODULES], plugins: [shimThree], logLevel: 'silent',
  })).outputFiles[0].text;
  const dom = new JSDOM(bare, { runScripts: 'outside-only', pretendToBeVisual: true, url: 'http://pi:8000/' });
  const w = dom.window;
  w.confirm = () => true;
  w.matchMedia = () => ({ matches: false, addEventListener() {} });
  w.ResizeObserver = class { observe() {} };
  w.URL.createObjectURL = () => 'blob:x'; w.URL.revokeObjectURL = () => {};
  w.downloads = [];   // jsdom can't download: record what the page offers instead
  w.HTMLAnchorElement.prototype.click = function () { w.downloads.push(this.download); };
  if (fetch) w.fetch = fetch;
  if (WebSocket) w.WebSocket = WebSocket;
  const errors = [];
  w.addEventListener('error', e => errors.push(e.message));
  w.eval(bundle);
  return { w, errors };
}

module.exports = { pureBlocks, loadPage };
