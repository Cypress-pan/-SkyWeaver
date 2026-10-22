// Verify the camera fitter against Three.js projection, including portrait viewports.
import assert from 'node:assert/strict';
import * as THREE from '../../tools/stage/node_modules/three/build/three.module.js';
import Core from '../../web/core.js';

const points = [];
for (const x of [-3.5, 3.5]) for (const y of [-2, 2]) for (const z of [.45, 2.55]) points.push([x, y, z]);
for (const aspect of [.5, .95, 2]) for (const pitch of [.48, 1.047, 1.49]) for (const yaw of [0, .4]) {
  const view = Core.createView({target:[0,0,1.5],pitch,yaw});
  const distance = Core.perspectiveFitDistance(points, view, aspect);
  const camera = new THREE.PerspectiveCamera(48, aspect, .015, 200);
  camera.position.set(Math.sin(yaw)*Math.cos(pitch)*distance,
    1.5+Math.sin(pitch)*distance, Math.cos(yaw)*Math.cos(pitch)*distance);
  camera.lookAt(0,1.5,0); camera.updateMatrixWorld();
  for (const p of points) {
    const projected = new THREE.Vector3(p[0],p[2],-p[1]).project(camera);
    assert(Math.abs(projected.x) <= .85, `horizontal clipping at aspect ${aspect}`);
    assert(Math.abs(projected.y) <= .72, `vertical clipping at aspect ${aspect}`);
  }
}
console.log('STAGE_FIT_OK: 18 portrait/landscape camera configurations fit all actual points.');
