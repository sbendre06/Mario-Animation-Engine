/**
 * Mario FK Dance – CPSC 4870 | 3D Spatial Computing, Yale University
 *
 * Demonstrates:
 *   • Forward Kinematics (FK): joint rotations compose through the Blender
 *     hierarchy (Torso → Limb → EndEffector) – rotating a parent moves all
 *     children automatically via Three.js's scene-graph matrixWorld chain.
 *
 *   • Quaternion SLERP on SO(3): every joint rotation is driven by
 *       q(t) = SLERP(q_A, q_B, t)   via THREE.Quaternion.slerpQuaternions
 *     between consecutive keyframe quaternions.  No Euler lerp is used.
 *
 *   • SE(3) chaining: the world pose of any end-effector equals the product
 *       T_world_shoe = T_world_torso ⊗ T_torso_leftLeg ⊗ T_leftLeg_shoe
 *     read live from Three.js's matrixWorld and displayed in the UI.
 *
 *   • Spline-style keyframe sequencing: a global time t drives smooth
 *     eased interpolation through a 4-beat looping keyframe sequence.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader }    from 'three/examples/jsm/loaders/GLTFLoader.js';

// ─── Renderer / Scene ────────────────────────────────────────────────────────

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.1;
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0d1020);
scene.fog = new THREE.FogExp2(0x0d1020, 0.06);

// ─── Camera ──────────────────────────────────────────────────────────────────

const camera = new THREE.PerspectiveCamera(52, window.innerWidth / window.innerHeight, 0.01, 80);
camera.position.set(0, 1.8, 4.2);

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// ─── Orbit Controls ──────────────────────────────────────────────────────────

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.9, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.06;
controls.minDistance = 1.5;
controls.maxDistance = 10;
controls.maxPolarAngle = Math.PI * 0.88;

// ─── Lighting ────────────────────────────────────────────────────────────────

scene.add(new THREE.AmbientLight(0xcce0ff, 0.55));

const sun = new THREE.DirectionalLight(0xfff4dd, 2.2);
sun.position.set(4, 9, 5);
sun.castShadow = true;
Object.assign(sun.shadow.camera, { left: -3, right: 3, top: 5, bottom: -1 });
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.bias = -0.0008;
scene.add(sun);

const fill = new THREE.DirectionalLight(0x8ab4f8, 0.65);
fill.position.set(-5, 3, -3);
scene.add(fill);

const rim = new THREE.DirectionalLight(0xff9966, 0.35);
rim.position.set(0, 2, -6);
scene.add(rim);

// ─── Stage ───────────────────────────────────────────────────────────────────

const floor = new THREE.Mesh(
  new THREE.PlaneGeometry(14, 14),
  new THREE.MeshStandardMaterial({ color: 0x1e3a1e, roughness: 0.88, metalness: 0.02 })
);
floor.rotation.x = -Math.PI / 2;
floor.receiveShadow = true;
scene.add(floor);

// Subtle disco-ish circular platform
const platform = new THREE.Mesh(
  new THREE.CylinderGeometry(1.1, 1.2, 0.06, 32),
  new THREE.MeshStandardMaterial({ color: 0x2a5230, roughness: 0.6, metalness: 0.15 })
);
platform.position.y = 0.001;
platform.receiveShadow = true;
scene.add(platform);

const grid = new THREE.GridHelper(14, 28, 0x1a3a1a, 0x1a3a1a);
grid.position.y = 0.003;
scene.add(grid);

// ─── Quaternion SLERP Utility ─────────────────────────────────────────────────
//
// Core operation: geodesic interpolation on SO(3).
// Given two unit quaternions q_A, q_B ∈ SO(3) and t ∈ [0,1]:
//
//   SLERP(q_A, q_B, t) = q_A · (q_A⁻¹ · q_B)^t
//
// THREE.Quaternion.slerpQuaternions implements this formula exactly,
// choosing the shorter arc (negating q_B if q_A·q_B < 0).
//
// FK composition (SE(3) product):
//   node.quaternion = q_rest ⊗ q_delta
// where ⊗ is Hamilton product (quaternion multiplication).

function slerpQ(qa, qb, t) {
  const out = new THREE.Quaternion();
  out.slerpQuaternions(qa, qb, t);   // geodesic SLERP on SO(3)
  return out;
}

// Compose rest orientation with an animation delta: q_full = q_rest ⊗ q_delta
function fkCompose(restQ, deltaQ) {
  return restQ.clone().multiply(deltaQ);
}

// Axis-angle → unit quaternion helper (degrees for readability)
function aq(ax, ay, az, deg) {
  const q = new THREE.Quaternion();
  q.setFromAxisAngle(new THREE.Vector3(ax, ay, az).normalize(), deg * (Math.PI / 180));
  return q;
}

const ID_Q = new THREE.Quaternion(); // identity (zero rotation)

// ─── Dance Keyframes ──────────────────────────────────────────────────────────
//
// Four keyframes form a looping dance cycle.  Each entry stores DELTA
// quaternions (additional rotation on top of the GLB rest pose).
//
// The FK chain:
//   Torso → Left Leg → Left Shoe          (SE(3) product of 3 transforms)
//   Torso → Right Leg → Right Shoe
//   Torso → Left Arm  (no further children with visible geometry)
//   Torso → Right Arm
//   Torso → Head → Mario Hat / Eyes / …
//
// Rotating any parent propagates to all children automatically (FK guarantee).

const DANCE_KF = [
  // ── Beat 0 – idle neutral ──
  {
    'Left Arm':   aq(1, 0,    0,  12),
    'Right Arm':  aq(1, 0,    0, -12),
    'Left Leg':   aq(0, 0,    1,   8),
    'Right Leg':  aq(0, 0,    1,  -8),
    'Left Shoe':  aq(0, 1,    0,  20),
    'Right Shoe': aq(0, 1,    0, -20),
    'Mario Hat':  aq(0, 0,    1,   3),
    'Left Eye':   aq(0, 0,    1,   7),
    'Right Eye':  aq(0, 0,    1,  -7),
  },
  // ── Beat 1 – left arm forward, right arm back ──
  {
    'Left Arm':   aq(1, 0,    0,  58),
    'Right Arm':  aq(1, 0,    0, -62),
    'Left Leg':   aq(0, 0,    1,   2),
    'Right Leg':  aq(0, 0,    1, -15),
    'Left Shoe':  aq(0, 1,    0,   8),
    'Right Shoe': aq(0, 1,    0, -30),
    'Mario Hat':  aq(0, 0,    1,   8),
    'Left Eye':   aq(0, 0,    1,  16),
    'Right Eye':  aq(0, 0,    1,  -4),
  },
  // ── Beat 2 – both arms raised (victory / jump anticipation) ──
  {
    'Left Arm':   aq(1, 0.15, 0, -68),
    'Right Arm':  aq(1,-0.15, 0, -68),
    'Left Leg':   aq(0, 0,    1,  -6),
    'Right Leg':  aq(0, 0,    1,   6),
    'Left Shoe':  aq(0, 1,    0, -10),
    'Right Shoe': aq(0, 1,    0,  10),
    'Mario Hat':  aq(0, 0,    1,  -4),
    'Left Eye':   aq(0, 0,    1, -11),
    'Right Eye':  aq(0, 0,    1,  11),
  },
  // ── Beat 3 – right arm forward, left arm back ──
  {
    'Left Arm':   aq(1, 0,    0, -62),
    'Right Arm':  aq(1, 0,    0,  58),
    'Left Leg':   aq(0, 0,    1, -15),
    'Right Leg':  aq(0, 0,    1,   2),
    'Left Shoe':  aq(0, 1,    0, -30),
    'Right Shoe': aq(0, 1,    0,   8),
    'Mario Hat':  aq(0, 0,    1,  -8),
    'Left Eye':   aq(0, 0,    1,   4),
    'Right Eye':  aq(0, 0,    1, -16),
  },
];

const CYCLE_DURATION = 1.8;  // seconds for one full 4-beat cycle
const N_BEATS        = DANCE_KF.length;

// Smooth ease-in-out: slower at beat boundaries, faster through the middle
function easeIO(t) {
  return t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
}

function getDanceParams(elapsed) {
  const globalT  = (elapsed % CYCLE_DURATION) / CYCLE_DURATION;  // [0, 1)
  const beatF    = globalT * N_BEATS;
  const beatIdx  = Math.floor(beatF) % N_BEATS;
  const localT   = easeIO(beatF - Math.floor(beatF));             // eased [0,1]
  return { beatIdx, localT, globalT };
}

function applyDancePose(elapsed) {
  const { beatIdx, localT } = getDanceParams(elapsed);
  const kfA = DANCE_KF[beatIdx];
  const kfB = DANCE_KF[(beatIdx + 1) % N_BEATS];

  for (const name of Object.keys(kfA)) {
    const node = joints[name];
    if (!node) continue;

    const dA   = kfA[name];
    const dB   = kfB[name] ?? ID_Q;

    // SLERP on SO(3): geodesic path between two keyframe delta rotations
    const delta = slerpQ(dA, dB, localT);

    // FK composition: full rotation = rest pose ⊗ animated delta
    node.quaternion.copy(fkCompose(restQuats[name], delta));
  }
}

// ─── Special Poses ────────────────────────────────────────────────────────────

const POSE_WAVE = {
  'Left Arm':  aq(1, 0.25, 0, -80),
  'Right Arm': aq(1, 0,    0,  15),
};

const POSE_JUMP = {
  'Left Arm':  aq(1, 0.2,  0, -78),
  'Right Arm': aq(1,-0.2,  0, -78),
  'Left Leg':  aq(1, 0,  0.4, -22),
  'Right Leg': aq(1, 0, -0.4, -22),
};

function applySpecialPose(pose) {
  for (const [name, targetDelta] of Object.entries(pose)) {
    const node = joints[name];
    if (!node) continue;
    node.quaternion.copy(fkCompose(restQuats[name], targetDelta));
  }
}

// ─── Joint Registry ───────────────────────────────────────────────────────────

const joints    = {};   // name → THREE.Object3D
const restQuats = {};   // name → THREE.Quaternion (GLB rest pose, immutable)

const ALL_NAMES = [
  'Torso', 'Head', 'Nose',
  'Left Arm', 'Right Arm',
  'Left Leg', 'Right Leg',
  'Left Shoe', 'Right Shoe',
  'Mario Hat',
  'Left Eye White', 'Right Eye White',
  'Left Eye', 'Right Eye',
  'Left Eyebrow', 'Right Eyebrow',
];

// Joints available for interactive selection (keys 1–6)
const SELECT_JOINTS = ['Left Arm', 'Right Arm', 'Left Leg', 'Right Leg', 'Head', 'Mario Hat'];

// ─── Interactive Mode State ───────────────────────────────────────────────────

let isDancing        = true;
let selectedJointIdx = 0;
const interactiveDeltas = {};  // name → THREE.Quaternion (accumulated user rotation)

function selectedName()  { return SELECT_JOINTS[selectedJointIdx]; }
function ensureDelta(nm) {
  if (!interactiveDeltas[nm]) interactiveDeltas[nm] = new THREE.Quaternion();
  return interactiveDeltas[nm];
}

const ROTATE_DEG = 4.5;
const heldKeys   = new Set();

window.addEventListener('keydown', e => {
  heldKeys.add(e.code);
  switch (e.code) {
    case 'Space':
      e.preventDefault();
      isDancing = !isDancing;
      if (isDancing) {
        // Clearing interactive deltas lets the dance loop retake control cleanly
        for (const k of Object.keys(interactiveDeltas)) delete interactiveDeltas[k];
      } else {
        // Entering interactive: seed deltas from the current dance pose
        for (const [nm, node] of Object.entries(joints)) {
          const curQ  = node.quaternion.clone();
          const invR  = restQuats[nm].clone().invert();
          interactiveDeltas[nm] = invR.multiply(curQ);
        }
      }
      break;
    case 'Digit1': case 'Numpad1': selectedJointIdx = 0; break;
    case 'Digit2': case 'Numpad2': selectedJointIdx = 1; break;
    case 'Digit3': case 'Numpad3': selectedJointIdx = 2; break;
    case 'Digit4': case 'Numpad4': selectedJointIdx = 3; break;
    case 'Digit5': case 'Numpad5': selectedJointIdx = 4; break;
    case 'Digit6': case 'Numpad6': selectedJointIdx = 5; break;
    case 'KeyR':
      if (!isDancing) {
        const nm = selectedName();
        interactiveDeltas[nm] = new THREE.Quaternion();
        if (joints[nm]) joints[nm].quaternion.copy(restQuats[nm]);
      }
      break;
    case 'KeyW':
      if (!isDancing) applySpecialPose(POSE_WAVE);
      break;
    case 'KeyJ':
      if (!isDancing) applySpecialPose(POSE_JUMP);
      break;
  }
});

window.addEventListener('keyup', e => heldKeys.delete(e.code));

function processKeyRotation(dt) {
  if (isDancing) return;
  const nm   = selectedName();
  const node = joints[nm];
  if (!node) return;

  const speed = ROTATE_DEG * dt * 60;          // deg/frame-independent
  const rad   = speed * (Math.PI / 180);

  let changed = false;
  const tmp   = new THREE.Quaternion();
  const delta = ensureDelta(nm);

  // Arrow keys → rotate around local X / Y; Q/E → local Z
  if (heldKeys.has('ArrowUp'))    { tmp.setFromAxisAngle(new THREE.Vector3(1,0,0),  rad); delta.premultiply(tmp); changed = true; }
  if (heldKeys.has('ArrowDown'))  { tmp.setFromAxisAngle(new THREE.Vector3(1,0,0), -rad); delta.premultiply(tmp); changed = true; }
  if (heldKeys.has('ArrowLeft'))  { tmp.setFromAxisAngle(new THREE.Vector3(0,1,0),  rad); delta.premultiply(tmp); changed = true; }
  if (heldKeys.has('ArrowRight')) { tmp.setFromAxisAngle(new THREE.Vector3(0,1,0), -rad); delta.premultiply(tmp); changed = true; }
  if (heldKeys.has('KeyQ'))       { tmp.setFromAxisAngle(new THREE.Vector3(0,0,1),  rad); delta.premultiply(tmp); changed = true; }
  if (heldKeys.has('KeyE'))       { tmp.setFromAxisAngle(new THREE.Vector3(0,0,1), -rad); delta.premultiply(tmp); changed = true; }

  if (changed) node.quaternion.copy(fkCompose(restQuats[nm], delta));
}

function applyInteractivePose() {
  // Replay all interactive deltas each frame so non-selected joints stay put
  for (const [nm, delta] of Object.entries(interactiveDeltas)) {
    const node = joints[nm];
    if (node) node.quaternion.copy(fkCompose(restQuats[nm], delta));
  }
}

// ─── Model Loading ────────────────────────────────────────────────────────────

let marioRoot  = null;
let baseY      = 0;      // world-space Y after centering, for vertical bob

const loader = new GLTFLoader();
loader.load(
  './mario.glb',
  gltf => {
    marioRoot = gltf.scene;

    // Traverse hierarchy, register every named node and snapshot its rest quaternion
    marioRoot.traverse(node => {
      if (ALL_NAMES.includes(node.name)) {
        joints[node.name]    = node;
        restQuats[node.name] = node.quaternion.clone();
      }
      if (node.isMesh) {
        node.castShadow    = true;
        node.receiveShadow = true;
      }
    });

    // Auto-scale: fit the model to ≈1.8 world-units tall
    const box  = new THREE.Box3().setFromObject(marioRoot);
    const size = box.getSize(new THREE.Vector3());
    const scl  = 1.8 / Math.max(size.x, size.y, size.z);
    marioRoot.scale.setScalar(scl);

    // Centre at origin, stand on the floor (y=0)
    box.setFromObject(marioRoot);
    const centre = box.getCenter(new THREE.Vector3());
    marioRoot.position.set(-centre.x, -box.min.y, -centre.z);
    baseY = marioRoot.position.y;

    scene.add(marioRoot);
    document.getElementById('loading').style.display = 'none';

    console.log('Mario loaded. Joints found:', Object.keys(joints));
    console.log('Missing joints:', ALL_NAMES.filter(n => !joints[n]));
  },
  undefined,
  err => {
    const el = document.getElementById('loading');
    el.innerHTML = '⚠ Error loading mario.glb<br><span class="sub">' + err.message + '</span>';
    console.error(err);
  }
);

// ─── UI Updates ───────────────────────────────────────────────────────────────

const $  = id => document.getElementById(id);
const wp = new THREE.Vector3();   // reused for world-position queries

function updateUI(elapsed) {
  const { beatIdx, localT, globalT } = getDanceParams(elapsed);

  // Mode badge
  const badge = $('badge');
  if (isDancing) {
    badge.textContent = '● Dancing';
    badge.className   = 'badge badge-dance';
    $('ui-beat').textContent  = `${beatIdx + 1} / ${N_BEATS}`;
    $('ui-slerp').textContent = localT.toFixed(3);
    $('ui-cycle').textContent = (globalT * 100).toFixed(0) + '%';
  } else {
    badge.textContent = '◈ Interactive';
    badge.className   = 'badge badge-interactive';
    $('ui-beat').textContent  = '—';
    $('ui-slerp').textContent = '—';
    $('ui-cycle').textContent = '—';
  }

  // Selected joint quaternion (live SO(3) state)
  const nm   = selectedName();
  const node = joints[nm];
  $('ui-jname').textContent = nm;
  if (node) {
    const q = node.quaternion;
    $('ui-qw').textContent = q.w.toFixed(4);
    $('ui-qx').textContent = q.x.toFixed(4);
    $('ui-qy').textContent = q.y.toFixed(4);
    $('ui-qz').textContent = q.z.toFixed(4);
  }

  // SE(3) chain: world position of the Left Shoe end-effector.
  // matrixWorld of Left Shoe = M_world_torso × M_torso_leftLeg × M_leftLeg_shoe
  // Three.js computes this automatically via updateMatrixWorld() during render.
  const shoe = joints['Left Shoe'];
  if (shoe) {
    shoe.getWorldPosition(wp);
    $('ui-x').textContent = wp.x.toFixed(3);
    $('ui-y').textContent = wp.y.toFixed(3);
    $('ui-z').textContent = wp.z.toFixed(3);
  }
}

// ─── Animation Loop ───────────────────────────────────────────────────────────

const clock = new THREE.Clock();
let elapsed = 0;

function animate() {
  requestAnimationFrame(animate);
  const dt = clock.getDelta();
  elapsed += dt;

  controls.update();

  if (marioRoot) {
    if (isDancing) {
      // Apply FK dance pose via SLERP on SO(3) for each joint
      applyDancePose(elapsed);

      // Body groove: gentle sinusoidal sway on the root transform
      // Demonstrates that rotating Torso's parent moves the ENTIRE character (root FK)
      marioRoot.rotation.z = Math.sin(elapsed * Math.PI * 4 / CYCLE_DURATION) * 0.042;

      // Vertical bob – tiny hop feel synchronized with the beat
      marioRoot.position.y = baseY + Math.abs(Math.sin(elapsed * Math.PI * 2 / CYCLE_DURATION)) * 0.05;
    } else {
      processKeyRotation(dt);
      applyInteractivePose();
      // Smoothly settle root back to upright
      marioRoot.rotation.z *= 0.92;
      marioRoot.position.y  = baseY + (marioRoot.position.y - baseY) * 0.9;
    }

    // Force matrixWorld update so SE(3) world-position queries are current
    marioRoot.updateMatrixWorld(true);
  }

  renderer.render(scene, camera);

  // UI update after render (matrixWorld is fresh)
  if (marioRoot) updateUI(elapsed);
}

animate();
