// three.js for the jsdom smoke test: the real library, with a WebGLRenderer that draws nothing.
export * from './node_modules/three/build/three.module.js';
export class WebGLRenderer {
  constructor() { this.domElement = document.createElement('canvas'); this.shadowMap = {}; }
  setPixelRatio() {}
  setSize() {}
  render() {}
}
