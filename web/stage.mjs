import * as THREE from '../tools/stage/node_modules/three/build/three.module.js';
import { mergeGeometries } from '../tools/stage/node_modules/three/examples/jsm/utils/BufferGeometryUtils.js';
import { EffectComposer } from '../tools/stage/node_modules/three/examples/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from '../tools/stage/node_modules/three/examples/jsm/postprocessing/RenderPass.js';
import { UnrealBloomPass } from '../tools/stage/node_modules/three/examples/jsm/postprocessing/UnrealBloomPass.js';
import { OutputPass } from '../tools/stage/node_modules/three/examples/jsm/postprocessing/OutputPass.js';

// ENU simulation coordinates -> Three.js Y-up. No target-to-actual substitution.
const enu = p => new THREE.Vector3(p[0], p[2], -p[1]);
const basis = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
const inverseBasis = basis.clone().invert();

function droneGeometry() {
  const parts = [new THREE.BoxGeometry(.066, .028, .044)];
  for (const angle of [-Math.PI / 4, Math.PI / 4]) {
    parts.push(new THREE.BoxGeometry(.17, .008, .012).rotateY(angle));
  }
  for (const x of [-.052, .052]) for (const z of [-.052, .052]) {
    parts.push(new THREE.TorusGeometry(.036, .004, 5, 18).rotateX(Math.PI / 2).translate(x, .008, z));
    parts.push(new THREE.CylinderGeometry(.009, .009, .021, 8).translate(x, 0, z));
  }
  const merged = mergeGeometries(parts);
  parts.forEach(g => g.dispose());
  return merged;
}

function create(canvas, options = {}) {
  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance',
      preserveDrawingBuffer: !!options.preserveDrawingBuffer });
  } catch (error) {
    console.warn('3D stage unavailable; using telemetry canvas.', error);
    return null;
  }
  renderer.setPixelRatio(options.pixelRatio ?? Math.min(window.devicePixelRatio || 1, 1.5));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(options.film ? '#02060d' : '#040914');
  scene.fog = new THREE.FogExp2('#040914', .016);
  const camera = new THREE.PerspectiveCamera(48, 1, .015, 200);
  scene.add(new THREE.HemisphereLight('#a6c8ff', '#0b182c', 3));
  const key = new THREE.DirectionalLight('#edf5ff', 4);
  key.position.set(0, 6, 2);
  scene.add(key);
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(70, 70),
    options.film ? new THREE.MeshBasicMaterial({color:'#02060d'}) :
      new THREE.MeshStandardMaterial({ color: '#01040a', roughness: .9, metalness: .15 }));
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -.025;
  scene.add(ground);
  const grid = new THREE.GridHelper(16, 32, '#1c3f65', '#112239');
  grid.material.transparent = true;
  grid.material.opacity = .35;
  scene.add(grid);
  const boundary = new THREE.Mesh(new THREE.RingGeometry(4.985, 5, 160),
    new THREE.MeshBasicMaterial({ color: '#31546d', transparent: true, opacity: .25, side: THREE.DoubleSide }));
  boundary.rotation.x = -Math.PI / 2;
  boundary.position.y = -.012;
  scene.add(boundary);
  const composer = new EffectComposer(renderer);
  composer.addPass(new RenderPass(scene, camera));
  const bloom = new UnrealBloomPass(new THREE.Vector2(640, 480), .35, .1, 1.3);
  composer.addPass(bloom);
  composer.addPass(new OutputPass());

  let capacity = 0, body, leds, references;
  const frame = new THREE.Object3D();
  const bodyMaterial = new THREE.MeshStandardMaterial({ color: '#8c9cac', metalness: .65, roughness: .35 });
  const ledMaterial = new THREE.MeshBasicMaterial({ color: '#ffffff' });
  const referenceMaterial = new THREE.MeshBasicMaterial({ color: '#e4b765', wireframe: true });
  const shape = droneGeometry();
  const ledShape = new THREE.SphereGeometry(.019, 10, 8).translate(0, .025, 0);
  const referenceShape = new THREE.OctahedronGeometry(.065);
  const trailGeometry = new THREE.BufferGeometry();
  const trailMaterial = new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: .22 });
  const trailLines = new THREE.LineSegments(trailGeometry, trailMaterial);
  trailLines.frustumCulled = false;
  scene.add(trailLines);
  const color = new THREE.Color();
  let trailCapacity = 0;
  function allocateTrails(vertices) {
    if (vertices <= trailCapacity) return;
    trailCapacity = Math.max(256, vertices);
    // Releasing the previous GPU attributes matters for a many-thousand-frame
    // offline render. Replacing attributes each frame leaks obsolete buffers.
    trailGeometry.dispose();
    for (const name of ['position', 'color']) trailGeometry.setAttribute(name,
      new THREE.BufferAttribute(new Float32Array(trailCapacity * 3), 3).setUsage(THREE.DynamicDrawUsage));
  }
  let lastFrame;
  let width = 0, height = 0, frames = 0, measuredAt = performance.now(), fps = 0;
  const stats = { engine: 'Three.js WebGL2', revision: THREE.REVISION, fps: 0, drones: 0, source: 'state.positions' };
  function allocate(n) {
    if (n <= capacity) return;
    for (const mesh of [body, leds, references]) if (mesh) { scene.remove(mesh); mesh.dispose(); }
    capacity = Math.max(128, n);
    body = new THREE.InstancedMesh(shape, bodyMaterial, capacity);
    leds = new THREE.InstancedMesh(ledShape, ledMaterial, capacity);
    references = new THREE.InstancedMesh(referenceShape, referenceMaterial, capacity);
    for (const mesh of [body, leds, references]) { mesh.frustumCulled = false; scene.add(mesh); }
  }
  function render(state, positions, view, trails, showTargets, connected, now) {
    lastFrame = [state, positions, view, trails, showTargets, connected, now];
    const w = Math.max(1, canvas.clientWidth), h = Math.max(1, canvas.clientHeight);
    if (w !== width || h !== height) {
      width = w; height = h;
      renderer.setSize(w, h, false); composer.setSize(w, h);
      camera.aspect = w / h; camera.updateProjectionMatrix();
    }
    const target = enu(view.target);
    camera.position.set(target.x + Math.sin(view.yaw) * Math.cos(view.pitch) * view.distance,
      target.y + Math.sin(view.pitch) * view.distance,
      target.z + Math.cos(view.yaw) * Math.cos(view.pitch) * view.distance);
    camera.lookAt(target);
    // Close inspection should reveal the airframe rather than overexpose LEDs.
    const closeView = view.distance < 2;
    const cinematic = /苏醒|A诞生|AGNES|helix|atom|bird|双螺旋|星河绽放|千机光浪|金色A|落幕/.test(state.activeSegment?.name || '');
    bloom.strength = closeView ? .08 : options.film && /千机光浪/.test(state.activeSegment?.name || '') ? .45 : cinematic ? .65 : .35;
    bloom.radius = cinematic ? .2 : .1;
    bloom.threshold = cinematic && !closeView ? .35 : 1.3;
    ground.material.color.set(options.film ? '#02060d' : cinematic ? '#000001' : '#01040a');
    grid.material.opacity = options.film ? 0 : cinematic ? .08 : .35;
    boundary.material.opacity = options.film ? 0 : cinematic ? .08 : .25;
    bodyMaterial.color.set(closeView ? '#a2b3c4' : options.film ? '#000000' : cinematic ? '#20314a' : '#8c9cac');
    const n = positions.length;
    allocate(Math.max(n, state.targetPositions.length));
    const segment = state.activeSegment;
    allocateTrails(Math.max(1, n) * 12 * 2);
    const values = trailGeometry.getAttribute('position').array;
    const colors = trailGeometry.getAttribute('color').array;
    let trailOffset = 0;
    if (body) {
      body.count = n; leds.count = n;
      for (let i = 0; i < n; i++) {
        frame.position.copy(enu(positions[i]));
        frame.quaternion.identity();
        const rpy = state.orientations[i];
        if (rpy && !state.raw.kinematic_preview) {
          // PyBullet RPY = extrinsic XYZ, equivalent to intrinsic ZYX.
          frame.quaternion.setFromEuler(new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX'));
          frame.quaternion.premultiply(basis).multiply(inverseBasis);
        }
        frame.scale.setScalar(1); frame.updateMatrix();
        body.setMatrixAt(i, frame.matrix); leds.setMatrixAt(i, frame.matrix);
        const hex = window.DroneShowCore.colorForDrone(segment, i, n);
        const idle = hex === '#26334b' || (!segment && positions[i][2] < .3);
        body.setColorAt(i, color.set(idle ? '#111922' : '#8197aa'));
        leds.setColorAt(i, color.set(hex).multiplyScalar(idle ? .04 : (connected ? (closeView ? 1.25 : cinematic ? 3.5 : 3) : .8)));
        const trail = trails[i];
        if (!idle && connected && trail) {
          color.set(hex);
          for (let j = Math.max(1, trail.length - (cinematic ? 5 : 12)); j < trail.length; j++) {
            if (Math.hypot(...trail[j].map((v, k) => v - trail[j - 1][k])) < .004) continue;
            for (const p of [trail[j - 1], trail[j]]) {
              values[trailOffset] = p[0]; values[trailOffset + 1] = p[2]; values[trailOffset + 2] = -p[1];
              colors[trailOffset] = color.r; colors[trailOffset + 1] = color.g; colors[trailOffset + 2] = color.b;
              trailOffset += 3;
            }
          }
        }
      }
      for (const mesh of [body, leds]) { mesh.instanceMatrix.needsUpdate = true; if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true; }
      references.visible = showTargets;
      references.count = showTargets ? state.targetPositions.length : 0;
      if (showTargets) state.targetPositions.forEach((p, i) => {
        frame.position.copy(enu(p)); frame.quaternion.identity(); frame.updateMatrix(); references.setMatrixAt(i, frame.matrix);
      });
      references.instanceMatrix.needsUpdate = true;
    }
    trailGeometry.setDrawRange(0, trailOffset / 3);
    for (const name of ['position', 'color']) trailGeometry.getAttribute(name).needsUpdate = true;
    composer.render();
    frames++;
    if (now - measuredAt > 1000) {
      fps = frames * 1000 / (now - measuredAt); frames = 0; measuredAt = now;
      stats.fps = Math.round(fps); stats.drones = n;
      canvas.dataset.renderer = stats.engine; canvas.dataset.fps = String(stats.fps);
    }
  }
  canvas.addEventListener('webglcontextlost', event => {
    event.preventDefault();
    document.getElementById('warning-text').textContent = '三维画面暂停，请刷新恢复。';
  });
  function capture() {
    if (!lastFrame) return Promise.resolve(null);
    render(...lastFrame);
    // Copy immediately while the WebGL drawing buffer contains this rendered frame.
    const sheet = document.createElement('canvas');
    sheet.width = canvas.width; sheet.height = canvas.height + 96;
    const ctx = sheet.getContext('2d');
    ctx.fillStyle = '#070c16'; ctx.fillRect(0, 0, sheet.width, sheet.height);
    ctx.drawImage(canvas, 0, 64);
    ctx.fillStyle = '#dbeaff'; ctx.font = '24px sans-serif';
    ctx.fillText('无人机秀 · ' + document.getElementById('show-content').textContent, 24, 40);
    ctx.fillStyle = '#849cba'; ctx.font = '14px sans-serif';
    ctx.fillText(document.getElementById('mode-badge').textContent + ' · ' + stats.drones + ' 架', 24, sheet.height - 12);
    return new Promise(resolve => sheet.toBlob(resolve, 'image/png'));
  }
  return { render, capture, stats };
}
window.DroneStage = { create };
