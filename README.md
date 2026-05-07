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

## SE(3) and FK Chains

A rigid-body transform (pose) in 3D space consists of a rotation *and* a translation. The set of all such transforms forms the **special Euclidean group**

```
SE(3) = { T = [R | t ; 0 | 1]  |  R ∈ SO(3),  t ∈ ℝ³ }
```

### Forward Kinematics as SE(3) Composition

**Forward kinematics (FK)** describes finding the world pose of every joint from a set of local joint angles, by composing transforms along a kinematic chain.

For Mario, the skeleton is a tree rooted at `Torso`. Each node stores a local SE(3) transform: position and rotation relative to its parent. The world transform of any node is the product of all parent transforms:

```
T_world_leftShoe = T_world_torso  ⊗  T_torso_leftLeg  ⊗  T_leftLeg_leftShoe
```

Three.js stores this as `node.matrixWorld` and recomputes it automatically each frame via `updateMatrixWorld(true)`.

---

## Spherical Linear Interpolation (SLERP)

Traditional linear interpolation (LERP) of rotation matrices to produce animations can result in paths that do not follow the shortest distance, or have non-constant speed. Through spherical linear interpolation (SLERP), travel occurs along the unique arc on $S^3$ that connects two unit quaternions at constant angular speed.

Given two unit quaternions q_A, q_B ∈ S³ and a parameter t ∈ [0, 1], the angle between them on S³ is Ω = arccos(q_A · q_B). Then:

```
SLERP(q_A, q_B, t) = q_A · sin((1−t)Ω) / sin(Ω)  +  q_B · sin(tΩ) / sin(Ω)
```

**Advantages of SLERP:**
- Constant angular speed: angular velocity of interpolated rotation is uniform
- Path is the shortest arc on S³
- No gimbal lock, better than Euler Angles

### Implementation

```javascript
function slerpQ(qa, qb, t) {
    const out = new THREE.Quaternion();
    out.slerpQuaternions(qa, qb, t);   // Three.js r160 implementation
    return out;
}
```

`THREE.Quaternion.slerpQuaternions` implements this formula with the short-arc correction and numerical fallback for near-identical quaternions.

---

## Dance Animation

The main dance animation is created through an animation loop applied to four different poses: idle, left arm forward, both arms raised, and right arm forward. SLERP is applied to each joint for smooth interpolation from one pose to the next.

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

---

## Time-Based Animations

The eyebrow wiggle, eye emote, and leg shuffle are time-based animations that run on top of the slider system.
The raw motion signal is a sine wave parameterised by elapsed time:

```javascript
const osc = Math.sin(2π · f · t)          // frequency f Hz
```

- **Eyebrow wiggle:** `|sin(7πt)|` — the absolute value creates a rapid "pop" pattern (always in the raised direction), at 3.5 Hz
- **Eye emote:** `sin(3.5πt)` — a ±1 oscillation driving the bipolar head-turn SLERP for shifty eyes, at ~1.75 Hz
- **Leg shuffle:** `sin(2π · 1.3 · t)` — smooth left-right pendulum at 1.3 Hz driving both the leg sway and the shoe rotation

---

## Integrating the Blender GLB File

The character was modelled in **Blender** and exported as a **GLB file** (binary glTF 2.0). glTF is a JSON-based standard for describing 3D scenes; GLB packs the JSON and all binary data (mesh geometry, images) into a single binary file.

Each object in the Blender scene becomes a **node** in the glTF JSON. A node specifies:
- `name` — the object name from Blender
- `translation`, `rotation`, `scale` — the local TRS transform (rotation stored as a unit quaternion in [x, y, z, w] order)
- `mesh` — an index into the mesh definitions (geometry data)
- `children` — indices of child nodes

This TRS form is critical: because the rotation is stored as a quaternion (not a matrix or Euler angles), Three.js loads it directly into `node.quaternion` without any conversion loss.

```javascript
for (const name of ALL_JOINT_NAMES) {
    const node = marioRoot.getObjectByName(name);
    if (node) {
        joints[name]    = node;
        restQuats[name] = node.quaternion.clone();
    }
}
```

---

## File Structure

```
Mario-Animation-Engine/
├── index.html          # Entry point: importmap (Three.js CDN), lil-gui UMD, module script
├── js/
│   └── main.js         # Complete animation engine (~560 lines)
└── mario.glb           # Blender character export (Y-up glTF 2.0)
```

`main.js` has no dependencies beyond Three.js r160 (loaded from CDN via importmap) and `lil-gui` (loaded as a UMD global via a `<script>` tag). All mathematical operations are implemented using Three.js's built-in `Quaternion`, `Vector3`, and `Matrix4` classes.
