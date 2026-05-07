/**
 * Mario FK Dance – CPSC 4870 | 3D Spatial Computing, Yale University
 *
 * Demonstrates:
 *   FK:          Joint rotations chain through the scene-graph hierarchy.
 *                Rotating Left_Leg automatically moves Left_Shoe (its child).
 *
 *   SO(3) SLERP: Every joint rotation is driven by quaternion SLERP:
 *                  q(t) = SLERP(q_A, q_B, t)  via .slerpQuaternions()
 *                No Euler-angle lerping anywhere.
 *
 *   SE(3) chain: World pose of any end-effector (shoe, hand) is the product
 *                  T_world_torso ⊗ T_torso_limb ⊗ T_limb_endeffector
 *                composed automatically by Three.js matrixWorld.
 *
 *   Keyframe spline: 4-beat looping sequence drives eased SLERP through
 *                    dance poses automatically.
 *
 * Note: Blender converts object names with spaces → underscores on GLB export.
 *   "Left Arm" in Blender → "Left_Arm" in Three.js scene graph.
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
camera.position.set(0, 2, -5);  // negative Z → facing Mario's front

// ─── Lighting ────────────────────────────────────────────────────────────────

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

const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(100, 100),
    new THREE.MeshPhongMaterial({ color: 0xcbcbcb, depthWrite: false })
);
floor.rotation.x = -Math.PI / 2;
floor.receiveShadow = true;
scene.add(floor);

const grid = new THREE.GridHelper(200, 40, 0x000000, 0x000000);
grid.material.opacity = 0.1;
grid.material.transparent = true;
scene.add(grid);

// ─── OrbitControls ───────────────────────────────────────────────────────────

const controls = new OrbitControls(camera, renderer.domElement);
controls.enablePan  = false;
controls.target.set(0, 1, 0);
controls.update();

// ─── Quaternion / FK Utilities ────────────────────────────────────────────────
//
// SO(3) SLERP: geodesic interpolation — always takes the shortest arc.
//   SLERP(q_A, q_B, t) = q_A · (q_A⁻¹ · q_B)^t
//
// FK composition (SE(3) product on the rotation part):
//   node.quaternion = q_rest ⊗ q_delta   (Hamilton product)
//
// Three.js automatically chains these up the hierarchy, so rotating Left_Leg
// updates Left_Shoe's world transform (FK end-effector follows parent). ✓

const ID_Q = new THREE.Quaternion();

function slerpQ(qa, qb, t) {
    const out = new THREE.Quaternion();
    out.slerpQuaternions(qa, qb, t);
    return out;
}

function fkCompose(restQ, deltaQ) {
    return restQ.clone().multiply(deltaQ);
}

function aq(ax, ay, az, deg) {
    const q = new THREE.Quaternion();
    q.setFromAxisAngle(new THREE.Vector3(ax, ay, az).normalize(), deg * Math.PI / 180);
    return q;
}

// ─── Joint Registry ───────────────────────────────────────────────────────────

const joints    = {};
const restQuats = {};

// Exact names as exported by Blender → GLB (spaces become underscores)
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

// ─── Arm Slider Math ──────────────────────────────────────────────────────────
//
// Single [0..1] slider drives each arm through the FORWARD hemisphere only:
//
//   t = 0.0  →  arm hanging straight down   (rest)
//   t = 0.5  →  arm horizontal in front of Mario
//   t = 1.0  →  arm pointing straight up
//
// Implemented as SLERP(identity, R_x(180°), t) on SO(3).
//
// Why this works for BOTH arms:
//   Left_Arm  rest ≈ R_x(+179°), local tip in +Y  → +X delta sweeps forward hemisphere ✓
//   Right_Arm rest ≈ R_x(−8.7°), local tip in −Y  → same +X delta, symmetric result  ✓
//
// The arm can never reach behind Mario because 180° of X rotation maps the
// tip exactly from down (0°) through forward (90°) to up (180°), staying
// in the sagittal plane in front of the character.

function armDelta(t) {
    // SLERP on SO(3): geodesic from identity to R_x(-180°)
    // The negative sign selects the forward-hemisphere arc for both arms.
    return slerpQ(ID_Q, aq(1, 0, 0, -180), t);
}

// ─── Slider Helpers ───────────────────────────────────────────────────────────

// Bipolar [-1..1]: SLERP between −maxDeg and +maxDeg; at t=0 gives identity.
function bipolarDelta(axis3, maxDeg, t) {
    const from = aq(...axis3, -maxDeg);
    const to   = aq(...axis3,  maxDeg);
    return slerpQ(from, to, (t + 1) / 2);
}

// Unipolar [0..1]: SLERP from identity to target.
function unipolarDelta(axis3, targetDeg, t) {
    return slerpQ(ID_Q, aq(...axis3, targetDeg), t);
}

// ─── Pose State ───────────────────────────────────────────────────────────────

const settings = {
    dance:        false,
    danceSpeed:   1.0,

    // Arm: 0 = hanging down, 0.5 = horizontal forward, 1 = straight up
    leftArm:      0.0,
    rightArm:     0.0,

    // Lower body
    legSway:      0.0,    // [-1..1]
    shoeShuffleL: 0.0,
    shoeShuffleR: 0.0,

    // Head / hat
    headTurn:     0.0,    // [-1..1]
    hatTilt:      0.0,
};

// ─── Slider → Pose Application ───────────────────────────────────────────────

function applySliderPose() {
    if (!marioRoot) return;

    // Arms – strictly forward hemisphere, anatomically correct
    if (joints['Left_Arm']) {
        joints['Left_Arm'].quaternion.copy(
            fkCompose(restQuats['Left_Arm'], armDelta(settings.leftArm))
        );
    }
    if (joints['Right_Arm']) {
        joints['Right_Arm'].quaternion.copy(
            fkCompose(restQuats['Right_Arm'], armDelta(settings.rightArm))
        );
    }

    // Legs (FK: shoe follows leg automatically)
    if (joints['Left_Leg']) {
        joints['Left_Leg'].quaternion.copy(
            fkCompose(restQuats['Left_Leg'], bipolarDelta([0,0,1], 18, settings.legSway))
        );
    }
    if (joints['Right_Leg']) {
        joints['Right_Leg'].quaternion.copy(
            fkCompose(restQuats['Right_Leg'], bipolarDelta([0,0,1], 18, settings.legSway))
        );
    }

    // Shoes
    if (joints['Left_Shoe']) {
        joints['Left_Shoe'].quaternion.copy(
            fkCompose(restQuats['Left_Shoe'], bipolarDelta([0,1,0], 35, settings.shoeShuffleL))
        );
    }
    if (joints['Right_Shoe']) {
        joints['Right_Shoe'].quaternion.copy(
            fkCompose(restQuats['Right_Shoe'], bipolarDelta([0,1,0], 35, settings.shoeShuffleR))
        );
    }

    // Head & Hat
    if (joints['Head']) {
        joints['Head'].quaternion.copy(
            fkCompose(restQuats['Head'], bipolarDelta([0,1,0], 40, settings.headTurn))
        );
    }
    if (joints['Mario_Hat']) {
        joints['Mario_Hat'].quaternion.copy(
            fkCompose(restQuats['Mario_Hat'], bipolarDelta([0,0,1], 18, settings.hatTilt))
        );
    }
}

// ─── Dance Keyframes ─────────────────────────────────────────────────────────
//
// All arm values are in [0..1] using armDelta(t), guaranteeing the arm stays
// in the forward hemisphere.  Alternating L/R arm heights creates the
// classic pendulum swing without any backward overshoot.

const DANCE_KF = [
    // Beat 0 – idle (arms slightly raised forward)
    {
        'Left_Arm':   armDelta(0.15),
        'Right_Arm':  armDelta(0.12),
        'Left_Leg':   bipolarDelta([0,0,1], 18,  0.30),
        'Right_Leg':  bipolarDelta([0,0,1], 18,  0.30),
        'Left_Shoe':  bipolarDelta([0,1,0], 35,  0.20),
        'Right_Shoe': bipolarDelta([0,1,0], 35, -0.20),
        'Mario_Hat':  bipolarDelta([0,0,1], 18,  0.15),
        'Head':       bipolarDelta([0,1,0], 40,  0.0),
    },
    // Beat 1 – left arm forward/up, right arm low
    {
        'Left_Arm':   armDelta(0.60),
        'Right_Arm':  armDelta(0.05),
        'Left_Leg':   bipolarDelta([0,0,1], 18,  0.10),
        'Right_Leg':  bipolarDelta([0,0,1], 18, -0.55),
        'Left_Shoe':  bipolarDelta([0,1,0], 35,  0.10),
        'Right_Shoe': bipolarDelta([0,1,0], 35, -0.65),
        'Mario_Hat':  bipolarDelta([0,0,1], 18,  0.40),
        'Head':       bipolarDelta([0,1,0], 40,  0.25),
    },
    // Beat 2 – both arms raised (victory)
    {
        'Left_Arm':   armDelta(0.90),
        'Right_Arm':  armDelta(0.90),
        'Left_Leg':   bipolarDelta([0,0,1], 18, -0.25),
        'Right_Leg':  bipolarDelta([0,0,1], 18, -0.25),
        'Left_Shoe':  bipolarDelta([0,1,0], 35, -0.25),
        'Right_Shoe': bipolarDelta([0,1,0], 35,  0.25),
        'Mario_Hat':  bipolarDelta([0,0,1], 18, -0.20),
        'Head':       bipolarDelta([0,1,0], 40,  0.0),
    },
    // Beat 3 – right arm forward/up, left arm low
    {
        'Left_Arm':   armDelta(0.05),
        'Right_Arm':  armDelta(0.60),
        'Left_Leg':   bipolarDelta([0,0,1], 18, -0.55),
        'Right_Leg':  bipolarDelta([0,0,1], 18,  0.10),
        'Left_Shoe':  bipolarDelta([0,1,0], 35, -0.65),
        'Right_Shoe': bipolarDelta([0,1,0], 35,  0.10),
        'Mario_Hat':  bipolarDelta([0,0,1], 18, -0.40),
        'Head':       bipolarDelta([0,1,0], 40, -0.25),
    },
];

const CYCLE_DURATION = 1.8;
const N_BEATS        = DANCE_KF.length;

function easeInOut(t) { return t < 0.5 ? 2*t*t : -1+(4-2*t)*t; }

function getDanceParams(elapsed) {
    const globalT = (elapsed % CYCLE_DURATION) / CYCLE_DURATION;
    const beatF   = globalT * N_BEATS;
    const beatIdx = Math.floor(beatF) % N_BEATS;
    const localT  = easeInOut(beatF - Math.floor(beatF));
    return { beatIdx, localT, globalT };
}

function applyDancePose(elapsed) {
    const { beatIdx, localT } = getDanceParams(elapsed);
    const kfA = DANCE_KF[beatIdx];
    const kfB = DANCE_KF[(beatIdx + 1) % N_BEATS];

    for (const name of Object.keys(kfA)) {
        const node = joints[name];
        if (!node) continue;
        // SLERP on SO(3): geodesic between keyframe delta quaternions
        const delta = slerpQ(kfA[name], kfB[name] ?? ID_Q, localT);
        // FK composition: full rotation = rest ⊗ animated delta
        node.quaternion.copy(fkCompose(restQuats[name], delta));
    }

    // Root-level body groove: rotating the root moves ALL children (FK root) ✓
    if (marioRoot) {
        marioRoot.rotation.z =
            Math.sin(elapsed * Math.PI * 4 / CYCLE_DURATION) * 0.04;
        marioRoot.position.y =
            baseY + Math.abs(Math.sin(elapsed * Math.PI * 2 / CYCLE_DURATION)) * 0.04;
    }
}

// ─── Time-Based Action Animations ────────────────────────────────────────────

let eyebrowWiggleEnd    = 0;
let eyeEmoteEnd         = 0;
let eyeEmoteWasActive   = false;
let legShuffleEnd       = 0;
let legShuffleWasActive = false;

const BROW_DURATION     = 1.8;
const EYE_DURATION      = 2.5;
const SHUFFLE_DURATION  = 3.0;   // seconds (~4 full sway cycles at 1.3 Hz)

const EYE_JOINTS  = ['Left_Eye',  'Right_Eye'];
const BROW_JOINTS = ['Left_Eyebrow', 'Right_Eyebrow'];

function applyBrowWiggle(elapsed) {
    if (elapsed >= eyebrowWiggleEnd) return;
    const t   = elapsed - (eyebrowWiggleEnd - BROW_DURATION);
    const osc = Math.abs(Math.sin(Math.PI * 7 * t));
    const delta = unipolarDelta([0, 0, 1], 28, osc);
    for (const name of BROW_JOINTS) {
        if (joints[name]) joints[name].quaternion.copy(fkCompose(restQuats[name], delta));
    }
}

function applyEyeEmote(elapsed) {
    const active = elapsed < eyeEmoteEnd;

    // On the frame the emote finishes, snap all affected joints to rest.
    if (!active) {
        if (eyeEmoteWasActive) {
            for (const name of [...EYE_JOINTS, ...BROW_JOINTS]) {
                if (joints[name]) joints[name].quaternion.copy(restQuats[name]);
            }
            eyeEmoteWasActive = false;
        }
        return;
    }
    eyeEmoteWasActive = true;

    const t     = elapsed - (eyeEmoteEnd - EYE_DURATION);
    const phase = t / EYE_DURATION;   // 0 → 1

    // Fade-out envelope: full strength until phase 0.75, then smoothly → 0.
    // This guarantees lookVal → 0 as the animation ends, returning eyes to rest.
    const env = phase < 0.75
        ? 1.0
        : easeInOut((1.0 - phase) / 0.25);   // easeInOut(1→0) over last 25%

    // Shifty-eyes: sinusoidal left/right, modulated by the fade envelope
    const lookVal = Math.sin(Math.PI * 3.5 * t) * env;
    const eyeDelta = bipolarDelta([0, 1, 0], 35, lookVal);
    for (const name of EYE_JOINTS) {
        if (joints[name]) joints[name].quaternion.copy(fkCompose(restQuats[name], eyeDelta));
    }

    // Eyebrow raise fades out in the first half, returning to rest on its own
    const browRaise = unipolarDelta([0, 0, 1], 18, Math.max(0, 1 - phase * 2));
    for (const name of BROW_JOINTS) {
        if (joints[name]) joints[name].quaternion.copy(fkCompose(restQuats[name], browRaise));
    }
}

function applyLegShuffle(elapsed) {
    const active = elapsed < legShuffleEnd;

    // Snap all affected joints to rest on the frame the animation finishes
    if (!active) {
        if (legShuffleWasActive) {
            for (const name of ['Left_Leg', 'Right_Leg', 'Left_Shoe', 'Right_Shoe']) {
                if (joints[name]) joints[name].quaternion.copy(restQuats[name]);
            }
            legShuffleWasActive = false;
        }
        return;
    }
    legShuffleWasActive = true;

    const t     = elapsed - (legShuffleEnd - SHUFFLE_DURATION);
    const phase = t / SHUFFLE_DURATION;   // 0 → 1

    // Fade-out envelope: hold full amplitude then ease to 0 in the last 25%
    // so legs and shoes return to rest position as the animation ends.
    const env = phase < 0.75
        ? 1.0
        : easeInOut((1.0 - phase) / 0.25);

    // Sinusoidal sway at 1.3 Hz
    const osc = Math.sin(2 * Math.PI * 1.3 * t) * env;

    // Legs sway left/right together
    const legDelta = bipolarDelta([0, 0, 1], 16, osc * 0.7);
    if (joints['Left_Leg'])  joints['Left_Leg'].quaternion.copy(fkCompose(restQuats['Left_Leg'],  legDelta));
    if (joints['Right_Leg']) joints['Right_Leg'].quaternion.copy(fkCompose(restQuats['Right_Leg'], legDelta));

    // Shoes go outward/inward in sync with the sway:
    //   left shoe positive osc  → toes out;  negative → toes in
    //   right shoe negative osc → toes out (mirrored rest quaternion)
    const shoeAmt = osc * 0.9;
    if (joints['Left_Shoe'])  joints['Left_Shoe'].quaternion.copy(fkCompose(restQuats['Left_Shoe'],  bipolarDelta([0,1,0], 35,  shoeAmt)));
    if (joints['Right_Shoe']) joints['Right_Shoe'].quaternion.copy(fkCompose(restQuats['Right_Shoe'], bipolarDelta([0,1,0], 35, -shoeAmt)));
}

// ─── Reset ────────────────────────────────────────────────────────────────────

function resetToRest() {
    for (const [name, node] of Object.entries(joints)) {
        node.quaternion.copy(restQuats[name]);
    }
    if (marioRoot) { marioRoot.rotation.z = 0; marioRoot.position.y = baseY; }
    settings.leftArm      = 0; settings.rightArm     = 0;
    settings.legSway      = 0;
    settings.shoeShuffleL = 0; settings.shoeShuffleR = 0;
    settings.headTurn     = 0; settings.hatTilt       = 0;
    refreshGUI();
}

// ─── GUI ─────────────────────────────────────────────────────────────────────

const gui = new lil.GUI({ title: 'Mario FK Dance', width: 280 });

// Animation
const animFolder = gui.addFolder('Animation');
animFolder.add(settings, 'dance').name('Dance').onChange(val => {
    setPoseControllersEnabled(!val);
    if (!val) resetToRest();
});
animFolder.add(settings, 'danceSpeed', 0.25, 3.0, 0.05).name('Speed');

// Arms – single slider per arm, anatomically constrained to forward hemisphere
const armsFolder = gui.addFolder('Arms');
const ctrlLA = armsFolder.add(settings, 'leftArm',  0, 1, 0.01)
    .name('Left Arm Circle');
const ctrlRA = armsFolder.add(settings, 'rightArm', 0, 1, 0.01)
    .name('Right Arm Circle');

// Legs & Shoes
const lowerFolder = gui.addFolder('Legs & Shoes');
const ctrlSway  = lowerFolder.add(settings, 'legSway',      -1, 1, 0.01).name('Leg Sway');
const ctrlShoeL = lowerFolder.add(settings, 'shoeShuffleL', -1, 1, 0.01).name('Shoe Shuffle L');
const ctrlShoeR = lowerFolder.add(settings, 'shoeShuffleR', -1, 1, 0.01).name('Shoe Shuffle R');

// Head & Hat
const headFolder = gui.addFolder('Head & Hat');
const ctrlHead = headFolder.add(settings, 'headTurn', -1, 1, 0.01).name('Head Turn');
const ctrlHat  = headFolder.add(settings, 'hatTilt',  -1, 1, 0.01).name('Hat Tilt');

// Actions
const actFolder = gui.addFolder('Actions');

actFolder.add({
    wave: () => {
        if (settings.dance) return;
        settings.leftArm  = 0.92;   // arm high
        settings.rightArm = 0.08;   // other arm relaxed
        settings.headTurn = 0.30;
        refreshGUI();
    }
}, 'wave').name('Wave');

actFolder.add({
    eyebrowWiggle: () => {
        eyebrowWiggleEnd = elapsed + BROW_DURATION;
    }
}, 'eyebrowWiggle').name('Eyebrow Wiggle');

actFolder.add({
    eyeEmote: () => {
        eyeEmoteEnd = elapsed + EYE_DURATION;
    }
}, 'eyeEmote').name('Eye Emote');

actFolder.add({
    legShuffle: () => {
        legShuffleEnd = elapsed + SHUFFLE_DURATION;
    }
}, 'legShuffle').name('Leg Shuffle');

actFolder.add({
    reset: () => { if (!settings.dance) resetToRest(); }
}, 'reset').name('Reset All');

// Bulk enable/disable when dance toggles
const poseControllers = [
    ctrlLA, ctrlRA,
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
let elapsed   = 0;   // declare here so action callbacks can read it

const loader = new GLTFLoader();
loader.load('./mario.glb',
    gltf => {
        marioRoot = gltf.scene;

        marioRoot.traverse(node => {
            if (node.isMesh) {
                node.castShadow    = true;
                node.receiveShadow = true;
            }
        });

        // Use getObjectByName so we always get the FIRST (parent transform)
        // node, not a same-named mesh child with identity quaternion.
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

        // Auto-scale to ≈ 1.8 world-units tall
        const box  = new THREE.Box3().setFromObject(marioRoot);
        const size = box.getSize(new THREE.Vector3());
        marioRoot.scale.setScalar(1.8 / Math.max(size.x, size.y, size.z));

        // Stand on the floor and centre
        box.setFromObject(marioRoot);
        const centre = box.getCenter(new THREE.Vector3());
        marioRoot.position.set(-centre.x, -box.min.y, -centre.z);
        baseY = marioRoot.position.y;

        scene.add(marioRoot);
    },
    undefined,
    err => console.error('GLB load error:', err)
);

// ─── Animation Loop ───────────────────────────────────────────────────────────

const clock = new THREE.Clock();

function animate() {
    requestAnimationFrame(animate);
    const dt = clock.getDelta();
    elapsed += dt * settings.danceSpeed;

    controls.update();

    if (marioRoot) {
        if (settings.dance) {
            applyDancePose(elapsed);
        } else {
            applySliderPose();
            marioRoot.rotation.z *= 0.9;
            marioRoot.position.y  = baseY + (marioRoot.position.y - baseY) * 0.9;
        }

        // Overlay time-based actions (run on top of dance or sliders)
        applyBrowWiggle(elapsed);
        applyEyeEmote(elapsed);
        applyLegShuffle(elapsed);

        marioRoot.updateMatrixWorld(true);
    }

    renderer.render(scene, camera);
}

animate();
