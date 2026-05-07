# Mario Forward Kinematics Animation Engine

This project entails an animation engine inspired by forward kinematics (FK), applied to a low-poly Mario character that was modeled from scratch in Blender and rendered in Three.js. The animations are driven by the user through a suite of interactive controls on the Three.js GUI, which were all built using quaternion spherical linear interpolation (SLERP) to represent and calculate rotations in SO(3). The goal of the project is to leverage principles of Lie groups and forward kinematics to demonstrate how simple animations can be created.

---

## Rotations as Animations

A rotation in three-dimensional Euclidean space is a linear map **R** : ℝ³ → ℝ³ that preserves lengths and orientations. The set of all such maps forms the **special orthogonal group**

```
SO(3) = { R ∈ ℝ^{3×3} | R^T R = I,  det(R) = +1 }
```

### Axis-Angle Representation
All of the rotation animations are initially specified in axis-angle form, and then they are converted into quaternion form:

```javascript
// aq(ax, ay, az, deg) — axis-angle → unit quaternion
function aq(ax, ay, az, deg) {
    const q = new THREE.Quaternion();
    q.setFromAxisAngle(new THREE.Vector3(ax, ay, az).normalize(),
                       deg * Math.PI / 180);
    return q;
}
```

Examples used in the project:
- `aq(1, 0, 0, 180)` — 180° rotation about the world X axis (arm raise)
- `aq(0, 1, 0, 40)`  — 40° rotation about Y (head turn)
- `aq(0, 0, 1, 18)`  — 18° rotation about Z (hat tilt, leg sway)

---

## Unit Quaternions
Unit quaternions are very suitable for defining joint rotations, because they eliminate gimbal lock,  and allow for very efficient composition of rotations using the Hamilton product.
A quaternion is often written as a scalar-vector pair q = (w, **v**) where **v** = (x, y, z). The **Hamilton product** of two quaternions q₁ = (w₁, **v₁**) and q₂ = (w₂, **v₂**) is:

```
q₁ ⊗ q₂ = (w₁w₂ − v₁·v₂ ,  w₁v₂ + w₂v₁ + v₁ × v₂)
```
Note the double cover: q and −q encode the same physical rotation. The unit quaternion group S³ is a double cover of SO(3), and Three.js's SLERP implementation accounts for this by negating one quaternion when their dot product is negative (choosing the shorter arc).
In Three.js (and in this project), quaternions are stored in (x, y, z, w) component order. All joint rotations in the animation engine are unit quaternions.

---

## 3. The Special Euclidean Group SE(3) and FK Chains

A **rigid-body transform** (pose) in 3D space consists of a rotation *and* a translation. The set of all such transforms forms the **special Euclidean group**

```
SE(3) = { T = [R | t ; 0 | 1]  |  R ∈ SO(3),  t ∈ ℝ³ }
```

Elements of SE(3) are represented as 4×4 homogeneous matrices. The group operation — composing two transforms — is matrix multiplication, which in block form is:

```
T₁ ⊗ T₂ = [R₁R₂ | R₁t₂ + t₁ ; 0 | 1]
```

### Forward Kinematics as SE(3) Composition

**Forward kinematics (FK)** is the process of computing the world-space pose of every joint from a set of local joint angles, by composing transforms along a kinematic chain.

For Mario, the skeleton is a tree rooted at `Torso`. Each node stores a **local** SE(3) transform — position and rotation relative to its parent. The **world** transform of any node is the product of all ancestor transforms:

```
T_world_leftShoe = T_world_torso  ⊗  T_torso_leftLeg  ⊗  T_leftLeg_leftShoe
```

Three.js stores this as `node.matrixWorld` and recomputes it automatically each frame via `updateMatrixWorld(true)`. Crucially, **changing only the Left Leg's local quaternion** propagates automatically: Three.js multiplies each node's local matrix into its parent's world matrix, so the Left Shoe's world position moves with it — this is FK by definition.

### FK in the Code

Each animated joint's quaternion is set to:

```javascript
node.quaternion = restQ ⊗ deltaQ
```

implemented as:

```javascript
function fkCompose(restQ, deltaQ) {
    return restQ.clone().multiply(deltaQ);
}
```

where:
- `restQ` = the quaternion baked into the GLB by Blender (the joint's rest/bind pose)
- `deltaQ` = the animation's additional rotation, expressed in the parent's local frame
- `⊗` = Hamilton product (quaternion multiplication)

The Hamilton product `restQ.multiply(deltaQ)` applies `restQ` first and then `deltaQ` as an intrinsic (body-frame) rotation on top of it — exactly the correct convention for layering an animation delta onto a rest pose.

**Demonstrated FK chain in the project:**
When the *Leg Sway* slider moves, only `Left_Leg.quaternion` is written. The shoe mesh (`Left_Shoe`) has no code setting its quaternion — yet it moves perfectly in sync, because Three.js evaluates:

```
T_world_leftShoe = T_world_torso ⊗ T_torso_leftLeg(t) ⊗ T_leftLeg_leftShoe
```

at every frame, propagating the leg's rotation to the shoe's world position automatically.

---

## 4. Quaternion SLERP on SO(3)

Linear interpolation (lerp) of rotation matrices or Euler angles produces non-geodesic, non-constant-speed paths. The correct interpolation on SO(3) is the **Spherical Linear Interpolation (SLERP)**, which travels along the unique great-circle arc on S³ between two unit quaternions, at constant angular speed.

### Formula

Given two unit quaternions q_A, q_B ∈ S³ and a parameter t ∈ [0, 1], the angle between them on S³ is Ω = arccos(q_A · q_B). Then:

```
SLERP(q_A, q_B, t) = q_A · sin((1−t)Ω) / sin(Ω)  +  q_B · sin(tΩ) / sin(Ω)
```

When Ω → 0 (quaternions nearly identical), this degenerates to linear interpolation (handled numerically by the implementation). The double-cover issue is handled by negating q_B if q_A · q_B < 0, ensuring the shorter arc is always taken.

**Properties that make SLERP correct for SO(3):**
- Constant angular speed: the angular velocity of the interpolated rotation is uniform
- Geodesic path: the path is the shortest arc on S³ (and on SO(3))
- No gimbal lock: unlike Euler angles, quaternion SLERP is singularity-free

### Implementation

Every rotation in this project goes through one function:

```javascript
function slerpQ(qa, qb, t) {
    const out = new THREE.Quaternion();
    out.slerpQuaternions(qa, qb, t);   // Three.js r160 implementation
    return out;
}
```

`THREE.Quaternion.slerpQuaternions` implements the formula above with the short-arc correction and numerical fallback for near-identical quaternions.

### Three uses of SLERP in the project

**1. Bipolar slider** — interpolates symmetrically between two extreme poses:

```javascript
function bipolarDelta(axis, maxDeg, t) {
    // t ∈ [−1, 1]; at t=0 the midpoint of the geodesic = identity
    const from = aq(...axis, -maxDeg);
    const to   = aq(...axis,  maxDeg);
    return slerpQ(from, to, (t + 1) / 2);
}
```

Because `from` and `to` are symmetric about identity on the great circle, their midpoint (t = 0 → SLERP at 0.5) is exactly the identity quaternion, giving a clean "no rotation" at the centre of the slider.

**2. Unipolar slider** — interpolates from identity to a target rotation:

```javascript
function unipolarDelta(axis, targetDeg, t) {
    return slerpQ(ID_Q, aq(...axis, targetDeg), t);
}
```

**3. Arm hemisphere sweep** — a single slider drives the arm from hanging (t=0) through horizontal-forward (t=0.5) to overhead (t=1), constrained entirely to the forward half-space:

```javascript
function armDelta(t) {
    return slerpQ(ID_Q, aq(1, 0, 0, -180), t);
}
```

The target `R_x(−180°)` = quaternion (x=−1, y=0, z=0, w=0). SLERP from identity to this target traverses R_x(−90°) at t=0.5 — the horizontal-forward position — keeping the arm within Mario's front hemisphere for all t ∈ [0, 1]. This is an application of the geodesic structure of SO(3): the "shortest path" between rest and overhead sweeps exactly through the anatomically correct forward arc.

---

## 5. The Dance Keyframe System (Discrete Spline on SO(3))

The automatic dance animation is a **looping keyframe sequence** — a discrete spline on SO(3) — rather than a continuous mathematical curve.

### Keyframe Definition

Four keyframes define the dance cycle (indexed 0–3). Each keyframe is a dictionary mapping joint names to **pre-computed delta quaternions** in SO(3):

```javascript
const DANCE_KF = [
    {   // Beat 0 — idle
        'Left_Arm':  armDelta(0.15),
        'Left_Leg':  bipolarDelta([0,0,1], 18, 0.30),
        'Left_Shoe': bipolarDelta([0,1,0], 35, 0.20),
        ...
    },
    {   // Beat 1 — left arm forward
        'Left_Arm':  armDelta(0.60),
        ...
    },
    // Beat 2 (both arms raised), Beat 3 (right arm forward)
];
```

Each stored value is itself the output of `slerpQ(...)` — a pre-evaluated unit quaternion — so the keyframe table contains no angles, only points in SO(3).

### Time → Pose Mapping

Global elapsed time is mapped to a beat index and a local interpolation parameter:

```javascript
const globalT = (elapsed % CYCLE_DURATION) / CYCLE_DURATION;  // [0, 1)
const beatF   = globalT * N_BEATS;
const beatIdx = Math.floor(beatF) % N_BEATS;
const localT  = easeInOut(beatF - Math.floor(beatF));          // [0, 1]
```

The local parameter `localT` is passed through a **smooth-step (ease-in/ease-out) function** before being used as the SLERP parameter:

```javascript
function easeInOut(t) {
    return t < 0.5 ? 2*t*t : -1 + (4 - 2*t)*t;
}
```

This is the cubic Hermite interpolant (also called smoothstep) with zero first-derivative at both endpoints. It ensures the angular velocity of every joint is zero at beat boundaries (no abrupt acceleration), producing visually smooth motion even though the keyframes are a discrete set.

### Per-Joint Geodesic Interpolation

For each joint, the delta quaternions from the current and next keyframe are SLERPed:

```javascript
const delta = slerpQ(kfA[name], kfB[name], localT);
node.quaternion.copy(fkCompose(restQuats[name], delta));
```

This means every joint independently travels along the geodesic arc on SO(3) between its two surrounding keyframe poses. The concatenation of four such arcs — each with eased speed profile — forms a closed loop on SO(3)^N (the product manifold of all joint SO(3) spaces), producing the full looping dance.

### Comparison to B-Splines on SO(3)

A true cubic spline on SO(3) (such as a Cumulative B-Spline or a Squad curve) would use four keyframes simultaneously to compute a single smooth arc with C² continuity. This project uses the simpler piecewise-geodesic approach (SLERP between adjacent keyframes only), which gives C⁰ continuity at keyframe boundaries in the quaternion derivative. The ease-in/ease-out function restores C¹ continuity in practice (zero angular velocity at boundaries), making the motion visually indistinguishable from a true spline for this application.

---

## 6. Time-Based Overlay Animations

The eyebrow wiggle, eye emote, and leg shuffle are **time-based overlay animations** that run on top of either the dance or the slider system. Each uses:

### Sinusoidal Oscillation

The raw motion signal is a sine wave parameterised by elapsed time:

```javascript
const osc = Math.sin(2π · f · t)          // frequency f Hz
```

- **Eyebrow wiggle:** `|sin(7πt)|` — the absolute value creates a rapid "pop" pattern (always in the raised direction), at 3.5 Hz
- **Eye emote:** `sin(3.5πt)` — a ±1 oscillation driving the bipolar head-turn SLERP for shifty eyes, at ~1.75 Hz
- **Leg shuffle:** `sin(2π · 1.3 · t)` — smooth left-right pendulum at 1.3 Hz driving both the leg sway and the shoe rotation

### Fade-Out Envelope

To guarantee all joints return to their rest quaternion at animation end, a **fade-out envelope** is applied to the oscillation amplitude in the final 25% of the animation:

```javascript
const env = phase < 0.75
    ? 1.0
    : easeInOut((1.0 − phase) / 0.25);   // smoothly 1→0 over last 25%

const lookVal = Math.sin(...) * env;
```

As `phase → 1`, `env → 0`, so `lookVal → 0`, and `bipolarDelta(..., 0)` = identity. The joint quaternion converges to `fkCompose(restQ, identity) = restQ`. A hard snap to `restQuats[name]` fires on the first frame after `elapsed >= animationEnd` to eliminate any floating-point residual.

---

## 7. Loading and Integrating the Blender GLB File

### The GLB / glTF Format

The character was modelled in **Blender** and exported as a **GLB file** (binary glTF 2.0). glTF is a JSON-based standard for describing 3D scenes; GLB packs the JSON and all binary data (mesh geometry, images) into a single binary file.

Each object in the Blender scene becomes a **node** in the glTF JSON. A node specifies:
- `name` — the object name from Blender
- `translation`, `rotation`, `scale` — the local TRS transform (rotation stored as a unit quaternion in [x, y, z, w] order)
- `mesh` — an index into the mesh definitions (geometry data)
- `children` — indices of child nodes

This TRS form is critical: because the rotation is stored as a quaternion (not a matrix or Euler angles), Three.js loads it directly into `node.quaternion` without any conversion loss.

### Coordinate System Conventions

Blender uses a **Z-up, Y-forward** world convention. glTF 2.0 specifies a **Y-up, −Z-forward** convention. Blender's GLB exporter applies the transformation automatically:
- Blender +Z (up)      → GLB +Y (up)
- Blender +Y (forward) → GLB −Z (forward)
- Blender +X (right)   → GLB +X (right) (unchanged)

Three.js is also Y-up, so the loaded scene is correctly oriented. Mario faces the −Z direction in Three.js world space; the camera is placed at (0, 2, −5) looking toward the origin so that the viewer faces Mario's front.

### Name Mangling: Spaces → Underscores

Blender object names can contain spaces ("Left Arm", "Mario Hat"). The GLB exporter silently converts spaces to underscores in the exported node names:

```
"Left Arm"  (Blender) → "Left_Arm"  (GLB / Three.js)
"Mario Hat" (Blender) → "Mario_Hat" (GLB / Three.js)
```

This was the root cause of the initial joint-lookup failures. After debugging the Three.js scene tree, all joint name strings in the code were corrected to use underscore form.

### Multi-Primitive Nodes and Three.js Scene Structure

When Three.js loads a glTF node with a **single mesh primitive**, it creates a `THREE.Mesh` directly, named with the glTF node name. When a node has **multiple mesh primitives** (e.g., `Head` which has 4 primitives for its different colour regions), Three.js creates a `THREE.Group` named after the node, with one child `THREE.Mesh` per primitive.

This distinction matters for joint lookup. The `getObjectByName` method performs a depth-first search and returns the **first** match, which is always the parent `Group` or `Mesh` that holds the correct rest quaternion — not a same-named child mesh with identity quaternion. Using `traverse` instead (which visits all nodes) would cause the last match to overwrite the first, potentially storing a wrong (identity) quaternion. The project uses `getObjectByName` for all joint registration:

```javascript
for (const name of ALL_JOINT_NAMES) {
    const node = marioRoot.getObjectByName(name);
    if (node) {
        joints[name]    = node;
        restQuats[name] = node.quaternion.clone();
    }
}
```

### Rest Pose Quaternions

Each joint's rest quaternion is snapshotted at load time. These values reflect how Blender positioned and oriented each object in the scene. Key values confirmed from the GLB JSON:

| Joint | Rest quaternion (x, y, z, w) | Interpretation |
|---|---|---|
| `Left_Arm` | (1.000, 0, 0, 0.008) | ≈ R_x(179°) — arm tip in local +Y maps to world −Y (hanging down) |
| `Right_Arm` | (−0.076, 0, 0, 0.997) | ≈ R_x(−8.7°) — nearly identity; arm tip in local −Y, also hangs down |
| `Left_Leg` | (−0.094, 0, 0, 0.996) | ≈ R_x(−10.7°) |
| `Right_Leg` | (0, −0.996, 0.094, 0) | ≈ R_y(170°) — mirrored leg geometry |
| `Left_Shoe` | (0, 0, 0, 1) | identity |
| `Right_Shoe` | (0, 1, 0, 0) | ≈ R_y(180°) — mirrored shoe |
| `Head` | (0, 0, 0, 1) | identity |
| `Mario_Hat` | (0, 0, 0, 1) | identity |

The asymmetry between Left_Arm (≈180° X) and Right_Arm (≈−8.7° X) exists because Blender mirrored the arm geometry differently on each side: the left arm mesh was modelled with its local Y pointing up (requiring ~180° rotation to hang down), while the right arm's local Y was already pointing down (requiring only a small corrective tilt). Despite this asymmetry, the same `armDelta(t) = SLERP(identity, R_x(−180°), t)` formula produces anatomically correct and visually symmetric motion for both arms.

---

## 8. Summary of Mathematical Components

| Component | Mathematical structure | Used for |
|---|---|---|
| SO(3) | Lie group of 3×3 rotation matrices | Conceptual framework for all joint rotations |
| Unit quaternions (S³) | Double cover of SO(3) | Runtime representation of every joint rotation |
| Hamilton product (⊗) | Quaternion multiplication | FK composition: `restQ ⊗ deltaQ` |
| Axis-angle → quaternion | Rodrigues map | `aq()` — converting design-time angles to quaternions |
| SLERP | Geodesic interpolation on S³ | All slider motion, all dance keyframe transitions |
| SE(3) | Lie group of rigid-body transforms | Kinematic chain structure; shoe follows leg via matrixWorld |
| Smooth-step (easeInOut) | Cubic Hermite interpolant | Zero-velocity boundaries on dance beats |
| Sinusoidal oscillation | Trigonometric functions | Raw signal for eyebrow, eye, and leg shuffle overlays |
| Fade-out envelope | Piecewise smooth scalar function | Guarantees overlay animations return to rest quaternion |
| glTF TRS node | Quaternion + vector decomposition | Rest pose storage; loaded directly into Three.js node.quaternion |

---

## 9. File Structure

```
Mario-Animation-Engine/
├── index.html          # Entry point: importmap (Three.js CDN), lil-gui UMD, module script
├── js/
│   └── main.js         # Complete animation engine (~560 lines)
└── mario.glb           # Blender character export (Y-up glTF 2.0)
```

`main.js` has no dependencies beyond Three.js r160 (loaded from CDN via importmap) and `lil-gui` (loaded as a UMD global via a `<script>` tag). All mathematical operations are implemented using Three.js's built-in `Quaternion`, `Vector3`, and `Matrix4` classes.
