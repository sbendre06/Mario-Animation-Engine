/**
 * Mario FK Dance – CPSC 4870 | 3D Spatial Computing, Yale University
 *
 * Demonstrates:
 *   FK:            Joint rotations compose through the scene-graph hierarchy.
 *                  Rotating Left Leg automatically moves Left Shoe (its child).
 *
 *   SO(3) SLERP:   Every joint rotation is driven by quaternion SLERP:
 *                    q(t) = SLERP(q_A, q_B, t)  via .slerpQuaternions()
 *                  No Euler-angle lerping anywhere.
 *
 *   SE(3) chaining: World pose of Left Shoe =
 *                    T_world_torso ⊗ T_torso_leftLeg ⊗ T_leftLeg_shoe
 *                   Read live from matrixWorld; displayed in the GUI.
 *
 *   Keyframe spline: A 4-beat looping sequence drives smooth eased SLERP
 *                    through each dance pose automatically.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader }    from 'three/examples/jsm/loaders/GLTFLoader.js';

// ─── Renderer ────────────────────────────────────────────────────────────────

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
document.body.appendChild(renderer.domElement);

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// ─── Scene / Camera ──────────────────────────────────────────────────────────

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xa0a0a0);
scene.fog = new THREE.Fog(0xa0a0a0, 10, 50);

const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 100);
camera.position.set(0, 2, 5);

// ─── Lighting (matches Three.js skinning example) ────────────────────────────

const hemi = new THREE.HemisphereLight(0xffffff, 0x8d8d8d, 3);
hemi.position.set(0, 20, 0);
scene.add(hemi);

const dirLight = new THREE.DirectionalLight(0xffffff, 3);
dirLight.position.set(3, 10, 10);
dirLight.castShadow = true;
dirLight.shadow.camera.top    =  4;
dirLight.shadow.camera.bottom = -4;
dirLight.shadow.camera.left   = -4;
dirLight.shadow.camera.right  =  4;
dirLight.shadow.camera.near   =  0.1;
dirLight.shadow.camera.far    =  40;
scene.add(dirLight);

// ─── Stage ───────────────────────────────────────────────────────────────────

const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(100, 100),
    new THREE.MeshPhongMaterial({ color: 0xcbcbcb, depthWrite: false })
);
mesh.rotation.x = -Math.PI / 2;
mesh.receiveShadow = true;
scene.add(mesh);

const grid = new THREE.GridHelper(200, 40, 0x000000, 0x000000);
grid.material.opacity = 0.1;
grid.material.transparent = true;
scene.add(grid);

// ─── OrbitControls ───────────────────────────────────────────────────────────

const controls = new OrbitControls(camera, renderer.domElement);
controls.enablePan   = false;
controls.enableZoom  = true;
controls.target.set(0, 1, 0);
controls.update();

// ─── Quaternion / FK Utilities ────────────────────────────────────────────────
//
// SO(3) SLERP: geodesic interpolation between two unit quaternions.
//   SLERP(q_A, q_B, t) = q_A · (q_A⁻¹ · q_B)^t
//
// FK composition (SE(3) product on the rotation part):
//   node.quaternion = q_rest ⊗ q_delta   (Hamilton product)
//
// The scene-graph automatically chains these up the hierarchy, so
// rotating Left Leg's quaternion updates Left Shoe's world transform.

const ID_Q = new THREE.Quaternion();  // identity

function slerpQ(qa, qb, t) {
    const out = new THREE.Quaternion();
    out.slerpQuaternions(qa, qb, t);  // geodesic SLERP on SO(3)
    return out;
}

// FK composition: final rotation = rest ⊗ delta
function fkCompose(restQ, deltaQ) {
    return restQ.clone().multiply(deltaQ);
}

// Axis-angle → unit quaternion  (degrees for readability)
function aq(ax, ay, az, deg) {
    const q = new THREE.Quaternion();
    q.setFromAxisAngle(new THREE.Vector3(ax, ay, az).normalize(), deg * Math.PI / 180);
    return q;
}

// ─── Joint Registry ───────────────────────────────────────────────────────────

const joints    = {};   // name → THREE.Object3D  (live reference)
const restQuats = {};   // name → THREE.Quaternion (GLB rest pose, immutable)

// Exact node names as Three.js receives them from GLB.
// Blender converts spaces → underscores on export (confirmed via debug output).
const ALL_JOINT_NAMES = [
    'Torso', 'Head', 'Nose',
    'Left_Arm', 'Right_Arm',
    'Left_Leg', 'Right_Leg',
    'Left_Shoe', 'Right_Shoe',
    'Mario_Hat',
    'Left_Eye_White', 'Right_Eye_White',
    'Left_Eye', 'Right_Eye',
    'Left_Eyebrow', 'Right_Eyebrow',
];

// ─── Pose State ───────────────────────────────────────────────────────────────
//
// Slider values drive SLERP parameter t on SO(3).
// Dance mode overrides sliders with the keyframe sequence.
//
// GLB rest-pose facts (extracted from mario.glb JSON chunk):
//   Left Arm  rotation ≈ R_x(+179°)  → arm hangs in −Y; X-delta swings fwd/back
//   Right Arm rotation ≈ R_x(−8.7°)  → nearly upright; X-delta works same way
//   Left Leg  rotation ≈ R_x(−10.7°) → Z-delta gives lateral sway
//   Right Leg rotation ≈ R_y(+180°)  → mirrored; same Z-delta sign = same world direction
//   Left Shoe rotation  = identity   → Y-delta rotates toe in/out
//   Right Shoe rotation ≈ R_y(180°)  → mirrored; same Y-delta sign works symmetrically

const settings = {
    dance:        false,
    danceSpeed:   1.0,

    // Arm controls
    leftArmSwing:  0.0,   // [-1 = back, 0 = rest, +1 = forward]
    leftArmRaise:  0.0,   // [ 0 = rest,                +1 = up ]
    rightArmSwing: 0.0,
    rightArmRaise: 0.0,

    // Lower body
    legSway:      0.0,    // [-1 = lean L, 0 = rest, +1 = lean R]
    shoeShuffleL: 0.0,    // [-1 = toe-in,  0 = rest, +1 = toe-out]
    shoeShuffleR: 0.0,

    // Upper body
    headTurn:     0.0,    // [-1 = look L, 0 = rest, +1 = look R]
    hatTilt:      0.0,    // [-1 = tilt L, 0 = rest, +1 = tilt R]

    // SE(3) chain readout (display only)
    shoeWorldX:   '—',
    shoeWorldY:   '—',
    shoeWorldZ:   '—',
};

// ─── Slider → Quaternion Mapping ─────────────────────────────────────────────

// Bipolar slider t ∈ [−1, 1] → SLERP between two extreme delta quaternions.
// At t=0, SLERP of symmetric rotations around the same axis → identity. ✓
function bipolarDelta(axis3, maxDeg, t) {
    const from = aq(...axis3, -maxDeg);
    const to   = aq(...axis3,  maxDeg);
    return slerpQ(from, to, (t + 1) / 2);  // remap [−1,1] → [0,1]
}

// Unipolar slider t ∈ [0, 1] → SLERP from identity to target.
function unipolarDelta(axis3, targetDeg, t) {
    return slerpQ(ID_Q, aq(...axis3, targetDeg), t);
}

function applySliderPose() {
    if (!marioRoot) return;

    // ── Left Arm ──────────────────────────────────────────────────────────────
    // Rest ≈ R_x(179°). Additional X rotation swings the arm forward/back.
    // X rotation by −160° on top of 179° → R_x(19°) → arm raises toward +Y.
    if (joints['Left_Arm']) {
        const qSwing = bipolarDelta([1,0,0], 75, settings.leftArmSwing);
        const qRaise = unipolarDelta([1,0,0], -160, settings.leftArmRaise);
        // SE(3) composition of two virtual DOFs on the same joint:
        //   delta = q_raise ⊗ q_swing  (extrinsic: swing first, then raise)
        const delta = qRaise.multiply(qSwing);
        joints['Left_Arm'].quaternion.copy(fkCompose(restQuats['Left_Arm'], delta));
    }

    // ── Right Arm ─────────────────────────────────────────────────────────────
    // Rest ≈ R_x(−8.7°). Same axis convention; X swing → forward/back.
    if (joints['Right_Arm']) {
        const qSwing = bipolarDelta([1,0,0], 75, settings.rightArmSwing);
        const qRaise = unipolarDelta([1,0,0], -160, settings.rightArmRaise);
        const delta  = qRaise.multiply(qSwing);
        joints['Right_Arm'].quaternion.copy(fkCompose(restQuats['Right_Arm'], delta));
    }

    // ── Left Leg ──────────────────────────────────────────────────────────────
    // Rest ≈ R_x(−10.7°). Z-axis delta → lateral sway.
    // FK: rotating Left_Leg automatically moves Left_Shoe (its child). ✓
    if (joints['Left_Leg']) {
        const delta = bipolarDelta([0,0,1], 18, settings.legSway);
        joints['Left_Leg'].quaternion.copy(fkCompose(restQuats['Left_Leg'], delta));
    }

    // ── Right Leg ─────────────────────────────────────────────────────────────
    // Rest ≈ R_y(170°) — mirrored leg. Same Z-delta sign gives symmetric sway.
    if (joints['Right_Leg']) {
        const delta = bipolarDelta([0,0,1], 18, settings.legSway);
        joints['Right_Leg'].quaternion.copy(fkCompose(restQuats['Right_Leg'], delta));
    }

    // ── Shoes ─────────────────────────────────────────────────────────────────
    // Y-rotation pivots the toe in/out.
    // Left_Shoe rest = identity; Right_Shoe rest ≈ R_y(180°).
    if (joints['Left_Shoe']) {
        const delta = bipolarDelta([0,1,0], 35, settings.shoeShuffleL);
        joints['Left_Shoe'].quaternion.copy(fkCompose(restQuats['Left_Shoe'], delta));
    }
    if (joints['Right_Shoe']) {
        const delta = bipolarDelta([0,1,0], 35, settings.shoeShuffleR);
        joints['Right_Shoe'].quaternion.copy(fkCompose(restQuats['Right_Shoe'], delta));
    }

    // ── Head ──────────────────────────────────────────────────────────────────
    if (joints['Head']) {
        const delta = bipolarDelta([0,1,0], 40, settings.headTurn);
        joints['Head'].quaternion.copy(fkCompose(restQuats['Head'], delta));
    }

    // ── Hat ───────────────────────────────────────────────────────────────────
    if (joints['Mario_Hat']) {
        const delta = bipolarDelta([0,0,1], 18, settings.hatTilt);
        joints['Mario_Hat'].quaternion.copy(fkCompose(restQuats['Mario_Hat'], delta));
    }
}

// ─── Dance Keyframes ─────────────────────────────────────────────────────────
//
// Four keyframes form a looping dance cycle.  Each entry stores delta
// quaternions applied via FK composition: node.q = restQ ⊗ deltaQ.
//
// SLERP on SO(3) drives smooth geodesic transitions between beats.
// The eased local-t avoids abrupt acceleration at beat boundaries.

// Helper: compose raise + swing for one arm (same math as slider code)
function armKF(swingT, raiseT, swingMax = 75) {
    const qSwing = bipolarDelta([1,0,0], swingMax, swingT);
    const qRaise = unipolarDelta([1,0,0], -160, raiseT);
    return qRaise.multiply(qSwing);
}

const DANCE_KF = [
    // ── Beat 0 – idle neutral ────────────────────────────────────────────────
    {
        'Left_Arm':   armKF( 0.15, 0.0),
        'Right_Arm':  armKF(-0.15, 0.0),
        'Left_Leg':   bipolarDelta([0,0,1], 18,  0.3),
        'Right_Leg':  bipolarDelta([0,0,1], 18,  0.3),
        'Left_Shoe':  bipolarDelta([0,1,0], 35,  0.2),
        'Right_Shoe': bipolarDelta([0,1,0], 35, -0.2),
        'Mario_Hat':  bipolarDelta([0,0,1], 18,  0.15),
        'Head':       bipolarDelta([0,1,0], 40,  0.0),
    },
    // ── Beat 1 – left arm forward, right arm back ────────────────────────────
    {
        'Left_Arm':   armKF( 0.85, 0.0),
        'Right_Arm':  armKF(-0.85, 0.0),
        'Left_Leg':   bipolarDelta([0,0,1], 18,  0.1),
        'Right_Leg':  bipolarDelta([0,0,1], 18, -0.6),
        'Left_Shoe':  bipolarDelta([0,1,0], 35,  0.1),
        'Right_Shoe': bipolarDelta([0,1,0], 35, -0.7),
        'Mario_Hat':  bipolarDelta([0,0,1], 18,  0.4),
        'Head':       bipolarDelta([0,1,0], 40,  0.3),
    },
    // ── Beat 2 – both arms raised (victory / jump) ───────────────────────────
    {
        'Left_Arm':   armKF( 0.0, 0.88),
        'Right_Arm':  armKF( 0.0, 0.88),
        'Left_Leg':   bipolarDelta([0,0,1], 18, -0.3),
        'Right_Leg':  bipolarDelta([0,0,1], 18, -0.3),
        'Left_Shoe':  bipolarDelta([0,1,0], 35, -0.3),
        'Right_Shoe': bipolarDelta([0,1,0], 35,  0.3),
        'Mario_Hat':  bipolarDelta([0,0,1], 18, -0.2),
        'Head':       bipolarDelta([0,1,0], 40,  0.0),
    },
    // ── Beat 3 – right arm forward, left arm back ────────────────────────────
    {
        'Left_Arm':   armKF(-0.85, 0.0),
        'Right_Arm':  armKF( 0.85, 0.0),
        'Left_Leg':   bipolarDelta([0,0,1], 18, -0.6),
        'Right_Leg':  bipolarDelta([0,0,1], 18,  0.1),
        'Left_Shoe':  bipolarDelta([0,1,0], 35, -0.7),
        'Right_Shoe': bipolarDelta([0,1,0], 35,  0.1),
        'Mario_Hat':  bipolarDelta([0,0,1], 18, -0.4),
        'Head':       bipolarDelta([0,1,0], 40, -0.3),
    },
];

const CYCLE_DURATION = 1.8;  // seconds per 4-beat cycle
const N_BEATS        = DANCE_KF.length;

function easeInOut(t) {
    return t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
}

function getDanceParams(elapsed) {
    const globalT  = (elapsed % CYCLE_DURATION) / CYCLE_DURATION;
    const beatF    = globalT * N_BEATS;
    const beatIdx  = Math.floor(beatF) % N_BEATS;
    const localT   = easeInOut(beatF - Math.floor(beatF));
    return { beatIdx, localT, globalT };
}

function applyDancePose(elapsed) {
    const { beatIdx, localT } = getDanceParams(elapsed);
    const kfA = DANCE_KF[beatIdx];
    const kfB = DANCE_KF[(beatIdx + 1) % N_BEATS];

    for (const name of Object.keys(kfA)) {
        const node = joints[name];
        if (!node) continue;

        const dA = kfA[name];
        const dB = kfB[name] ?? ID_Q;

        // SLERP on SO(3): geodesic interpolation between keyframe delta quaternions
        const delta = slerpQ(dA, dB, localT);

        // FK composition: full rotation = rest ⊗ animated delta
        node.quaternion.copy(fkCompose(restQuats[name], delta));
    }

    // Body groove: small sinusoidal sway on the root transform.
    // Demonstrates root-level FK: rotating the root moves every child. ✓
    if (marioRoot) {
        marioRoot.rotation.z = Math.sin(elapsed * Math.PI * 4 / CYCLE_DURATION) * 0.04;
        marioRoot.position.y = baseY + Math.abs(Math.sin(elapsed * Math.PI * 2 / CYCLE_DURATION)) * 0.04;
    }
}

function resetToRest() {
    for (const [name, node] of Object.entries(joints)) {
        node.quaternion.copy(restQuats[name]);
    }
    if (marioRoot) {
        marioRoot.rotation.z = 0;
        marioRoot.position.y = baseY;
    }
    settings.leftArmSwing  = 0; settings.leftArmRaise  = 0;
    settings.rightArmSwing = 0; settings.rightArmRaise = 0;
    settings.legSway       = 0;
    settings.shoeShuffleL  = 0; settings.shoeShuffleR = 0;
    settings.headTurn      = 0; settings.hatTilt       = 0;
    refreshGUI();
}

// ─── GUI ─────────────────────────────────────────────────────────────────────

// 'lil' is the UMD global injected by the <script> tag in index.html
const gui = new lil.GUI({ title: 'Mario FK Dance', width: 280 });

// ── Animation ────────────────────────────────────────────────────────────────
const animFolder = gui.addFolder('Animation');
animFolder.add(settings, 'dance').name('Dance').onChange(val => {
    setPoseControllersEnabled(!val);
    if (!val) resetToRest();
});
animFolder.add(settings, 'danceSpeed', 0.25, 3.0, 0.05).name('Speed');

// ── Arms ─────────────────────────────────────────────────────────────────────
const armsFolder = gui.addFolder('Arms');
const ctrlLSwing = armsFolder.add(settings, 'leftArmSwing',  -1, 1, 0.01).name('L Arm Swing');
const ctrlLRaise = armsFolder.add(settings, 'leftArmRaise',   0, 1, 0.01).name('L Arm Raise');
const ctrlRSwing = armsFolder.add(settings, 'rightArmSwing', -1, 1, 0.01).name('R Arm Swing');
const ctrlRRaise = armsFolder.add(settings, 'rightArmRaise',  0, 1, 0.01).name('R Arm Raise');

// ── Lower Body ────────────────────────────────────────────────────────────────
const lowerFolder = gui.addFolder('Legs & Shoes');
const ctrlSway   = lowerFolder.add(settings, 'legSway',      -1, 1, 0.01).name('Leg Sway');
const ctrlShoeL  = lowerFolder.add(settings, 'shoeShuffleL', -1, 1, 0.01).name('Shoe Shuffle L');
const ctrlShoeR  = lowerFolder.add(settings, 'shoeShuffleR', -1, 1, 0.01).name('Shoe Shuffle R');

// ── Head ─────────────────────────────────────────────────────────────────────
const headFolder = gui.addFolder('Head & Hat');
const ctrlHead   = headFolder.add(settings, 'headTurn', -1, 1, 0.01).name('Head Turn');
const ctrlHat    = headFolder.add(settings, 'hatTilt',  -1, 1, 0.01).name('Hat Tilt');

// ── Actions ──────────────────────────────────────────────────────────────────
const actFolder = gui.addFolder('Actions');
actFolder.add({
    wave: () => {
        if (settings.dance) return;
        settings.leftArmSwing  = -0.1;
        settings.leftArmRaise  =  0.9;
        settings.rightArmSwing =  0.0;
        settings.rightArmRaise =  0.0;
        settings.headTurn      =  0.3;
        refreshGUI();
    }
}, 'wave').name('Wave');

actFolder.add({
    jump: () => {
        if (settings.dance) return;
        settings.leftArmSwing  = 0;
        settings.leftArmRaise  = 0.85;
        settings.rightArmSwing = 0;
        settings.rightArmRaise = 0.85;
        settings.legSway       = 0;
        settings.headTurn      = 0;
        refreshGUI();
    }
}, 'jump').name('Jump Pose');

actFolder.add({
    reset: () => {
        if (settings.dance) return;
        resetToRest();
    }
}, 'reset').name('Reset All');

// ── SE(3) Chain readout (informational) ───────────────────────────────────────
const se3Folder = gui.addFolder('SE(3) Chain – Left_Shoe');
const ctrlSX = se3Folder.add(settings, 'shoeWorldX').name('World X').disable();
const ctrlSY = se3Folder.add(settings, 'shoeWorldY').name('World Y').disable();
const ctrlSZ = se3Folder.add(settings, 'shoeWorldZ').name('World Z').disable();
se3Folder.open();

// All pose controllers in one array for bulk enable/disable
const poseControllers = [
    ctrlLSwing, ctrlLRaise, ctrlRSwing, ctrlRRaise,
    ctrlSway, ctrlShoeL, ctrlShoeR,
    ctrlHead, ctrlHat,
];

function setPoseControllersEnabled(enabled) {
    poseControllers.forEach(c => enabled ? c.enable() : c.disable());
}

function refreshGUI() {
    gui.controllersRecursive().forEach(c => c.updateDisplay());
}

// ─── Model Loading ────────────────────────────────────────────────────────────

let marioRoot = null;
let baseY     = 0;

const loader = new GLTFLoader();
loader.load('./mario.glb',
    gltf => {
        marioRoot = gltf.scene;

        // ── Step 1: shadow-traverse to enable shadows on all meshes ──────────
        marioRoot.traverse(node => {
            if (node.isMesh) {
                node.castShadow    = true;
                node.receiveShadow = true;
            }
        });

        // ── Step 2: register joints via getObjectByName ───────────────────────
        // getObjectByName returns the FIRST DFS match (always the parent
        // transform node, never a same-named mesh child that may have identity
        // quaternion).  Spaces in Blender names become underscores in GLB export.
        for (const name of ALL_JOINT_NAMES) {
            const node = marioRoot.getObjectByName(name);
            if (node) {
                joints[name]    = node;
                restQuats[name] = node.quaternion.clone();
            } else {
                console.warn(`Joint not found: "${name}"`);
            }
        }
        console.log('Joints registered:', Object.keys(joints));

        // ── Step 3: scale and centre ──────────────────────────────────────────
        const box  = new THREE.Box3().setFromObject(marioRoot);
        const size = box.getSize(new THREE.Vector3());
        const scl  = 1.8 / Math.max(size.x, size.y, size.z);
        marioRoot.scale.setScalar(scl);

        box.setFromObject(marioRoot);
        const centre = box.getCenter(new THREE.Vector3());
        marioRoot.position.set(-centre.x, -box.min.y, -centre.z);
        baseY = marioRoot.position.y;

        scene.add(marioRoot);
    },
    undefined,
    err => console.error('GLB load error:', err)
);

// ─── SE(3) World-Position Readout ─────────────────────────────────────────────

const _wp = new THREE.Vector3();

function updateSE3Readout() {
    // matrixWorld of Left Shoe encodes the full SE(3) chain:
    //   T_world_torso ⊗ T_torso_leftLeg ⊗ T_leftLeg_shoe
    // Three.js computes this via updateMatrixWorld() during render. ✓
    const shoe = joints['Left_Shoe'];
    if (!shoe) return;
    shoe.getWorldPosition(_wp);
    settings.shoeWorldX = _wp.x.toFixed(3);
    settings.shoeWorldY = _wp.y.toFixed(3);
    settings.shoeWorldZ = _wp.z.toFixed(3);
    ctrlSX.updateDisplay();
    ctrlSY.updateDisplay();
    ctrlSZ.updateDisplay();
}

// ─── Animation Loop ───────────────────────────────────────────────────────────

const clock = new THREE.Clock();
let elapsed  = 0;

function animate() {
    requestAnimationFrame(animate);

    const dt   = clock.getDelta();
    elapsed   += dt * settings.danceSpeed;

    controls.update();

    if (marioRoot) {
        if (settings.dance) {
            applyDancePose(elapsed);
        } else {
            applySliderPose();
            // Smooth root back to upright when leaving dance
            marioRoot.rotation.z *= 0.9;
            marioRoot.position.y  = baseY + (marioRoot.position.y - baseY) * 0.9;
        }

        marioRoot.updateMatrixWorld(true);  // ensure SE(3) chain is current
        updateSE3Readout();
    }

    renderer.render(scene, camera);
}

animate();
