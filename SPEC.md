# SPEC — 4-DOF Robotic Arm Virtual Laboratory

KON 445E, Fall 2026. This file is the technical specification. Every number and equation the code needs is here. If the code and this file disagree, one of them is wrong: fix the code, or change this file on purpose and say so in the commit.

Source of the model: the team's literature review (`docs/RoboticArm_LiteratureReview_v6.pdf`, Section 3). Equation numbers in brackets, e.g. [eq. 17], refer to that report.

Reference numbers for the tests are in `reference/test_vectors.json`, produced by `reference/arm_reference.py`, an independent Python implementation of this file.

---

## 1. Scope

An online virtual laboratory for a control-engineering student. The student picks an environment (planet), a payload, a task and a controller, runs the experiment, and studies plots and performance metrics. The arm is the ROBOTIS OpenMANIPULATOR-X: a base yaw joint and three parallel pitch joints.

Out of scope for the first version: contact and grasp physics, collisions between links, 3D rendering (optional later), real hardware.

## 2. Conventions

- SI units everywhere inside the core: m, kg, s, rad, N·m. Degrees and millimetres only in the user interface.
- Joint order `q = [q1, q2, q3, q4]`. Arrays are zero-based in code, so `q[0]` is joint 1.
- State `x = [q1..q4, q̇1..q̇4]` (8 values).
- Base frame: origin on the joint-1 axis, z up, x forward when q1 = 0.
- A positive pitch angle tips the arm forward and down (same as the manufacturer's URDF).
- Absolute pitch of each body below the horizontal [eq. 1]: θ1 = 0, θ2 = q2, θ3 = q2 + q3, θ4 = q2 + q3 + q4.
- Formatting of numbers in files always uses `CultureInfo.InvariantCulture`.

## 3. Parameters

### 3.1 Geometry [Table 3]

| Name | Meaning | Value |
|---|---|---|
| d1 | height of joint-2 axis above the base frame | 0.0595 m |
| a2x, a2z | joint 3 relative to joint 2 at q2 = 0 | 0.024 m forward, 0.128 m up |
| L2 | √(a2x² + a2z²) | 0.130231 m |
| β | atan2(0.128, 0.024) | 1.385448 rad (79.38°) |
| L3 | joint 3 to joint 4 | 0.124 m |
| L4 | joint 4 to tool point | 0.126 m |

### 3.2 Bodies [Table 5]

Body i moves with joint i. Body 4 includes the gripper. The centre of mass of body i lies at distance ℓi from joint i, at the angle θi + εi below the horizontal. Inertias are about the centre of mass, in the body frame (Section 4.1), in kg·m².

| Body | m (kg) | ℓ (m) | ε (deg) | Ixx | Iyy | Izz | Ixz |
|---|---|---|---|---|---|---|---|
| 1 | 0.0984 | 0.0474 | −90.4 | 34.5e-6 | 32.7e-6 | 18.9e-6 | −0.4e-6 |
| 2 | 0.1385 | 0.1022 | −84.2 | 330.6e-6 | 342.9e-6 | 60.3e-6 | −38.5e-6 |
| 3 | 0.1327 | 0.0909 | −0.1 | 30.7e-6 | 242.3e-6 | 251.6e-6 | −0.3e-6 |
| 4 | 0.1433 | 0.0451 | −11.4 | 80.9e-6 | 76.0e-6 | 93.1e-6 | −1.0e-6 |

Ixy and Iyz are taken as zero and the centres of mass lie in the arm's vertical plane. With this data the closed forms of Section 5.3 are exact, not approximations.

All masses come from the URDF. Do not mix in the specification-sheet masses (105–236 g).

### 3.3 Actuators (DYNAMIXEL XM430-W350)

| Name | Value | Status |
|---|---|---|
| Stall torque τmax | 4.1 N·m at 12 V | from data sheet |
| Gear ratio N | 353.5 | from data sheet |
| Encoder | 4096 counts per revolution | from data sheet (checked in T05 against the ROBOTIS e-manual) |
| No-load speed | about 46 rpm (4.8 rad/s) at 12 V | from data sheet (checked in T05 against the ROBOTIS e-manual) |
| Rotor inertia Jm | unknown, default 0 | experiment parameter |
| Viscous friction fv | default 0.01 N·m·s/rad | experiment parameter |
| Coulomb friction fc | default 0.02 N·m | experiment parameter |
| Smoothing speed ωs | 0.1 rad/s | numerical parameter; sets the step-size limit of §6.2 |

### 3.4 Joint limits

Copied from `open_manipulator_x.urdf` in task T02 (joint `<limit lower upper>` attributes, rad):

| Joint | Lower (rad) | Upper (rad) |
|---|---|---|
| 1 | −π | +π |
| 2 | −1.5 | +1.5 |
| 3 | −1.5 | +1.4 |
| 4 | −1.7 | +1.97 |

Source: <https://github.com/ROBOTIS-GIT/open_manipulator/blob/main/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf>, branch `main`, commit `d31000d90c679af9c982e73de8b12d777c5ff7dd` (read 2026-10-15). The same values are in `open_manipulator_x_arm.urdf.xacro` in the same folder.

The limits are data. Parameter, kinematics and dynamics code never rejects a configuration for being outside them; they are checked only by inverse kinematics (§4.3) and by fault detection (§6.4).

### 3.5 Environments

Gravity acts along −z with magnitude g.

| Key | Name in UI | g (m/s²) |
|---|---|---|
| Earth | Dünya | 9.81 |
| Moon | Ay | 1.62 |
| Mars | Mars | 3.71 |
| HighG | Yüksek g (Jüpiter düzeyi) | 24.79 |
| Orbit | Yörünge (mikro yerçekimi) | 0.0 |

A custom g between 0 and 30 m/s² is also allowed.

## 4. Kinematics

### 4.1 Kinematic chain (used by the general algorithms)

Each joint is a revolute joint. Frame Fi of body i is obtained from the parent frame:

| Joint | Parent | Origin in parent frame | Axis (local) |
|---|---|---|---|
| 1 | base | (0, 0, 0) | z |
| 2 | body 1 | (0, 0, d1) | y |
| 3 | body 2 | (0.024, 0, 0.128) | y |
| 4 | body 3 | (L3, 0, 0) | y |

So `F1 = Rz(q1)`, `F2 = F1·T(0,0,d1)·Ry(q2)`, `F3 = F2·T(0.024,0,0.128)·Ry(q3)`, `F4 = F3·T(L3,0,0)·Ry(q4)`, with

```
Rz(a) = [[c,-s,0],[s,c,0],[0,0,1]]     Ry(a) = [[c,0,s],[0,1,0],[-s,0,c]]
```

Ry with a positive angle turns the local x axis downward, which is the pitch convention of Section 2.

- Tool point in F4: (L4, 0, 0). Tool x axis is the local x axis of F4.
- Centre of mass of body i in Fi: (ℓi cos εi, 0, −ℓi sin εi).
- Inertia tensor of body i in Fi: [[Ixx, 0, Ixz], [0, Iyy, 0], [Ixz, 0, Izz]].

The chain is data, not code. The same algorithms must work for a 6-DOF chain given as a different table.

### 4.2 Forward kinematics [eqs. 2–3]

```
r = L2 cos(θ2 − β) + L3 cos θ3 + L4 cos θ4
z = d1 − L2 sin(θ2 − β) − L3 sin θ3 − L4 sin θ4
p = [r cos q1, r sin q1, z],   ψ = θ4  (tool pitch below horizontal)
```

### 4.3 Inverse kinematics [eqs. 5–7]

Input: target position p = (x, y, z), tool pitch ψ, branch (elbow up or elbow down), and an option `checkLimits` (default true).

```
q1 = atan2(y, x)
u  = √(x² + y²) − L4 cos ψ
v  = z − d1 + L4 sin ψ
cos φ = (u² + v² − L2² − L3²) / (2 L2 L3)
φ = −arccos(cos φ)  for elbow up,  +arccos(cos φ)  for elbow down
e  = atan2(v, u) − atan2(L3 sin φ, L2 + L3 cos φ)
q2 = β − e,  q3 = −β − φ,  q4 = ψ − q2 − q3
```

- Unreachable if |cos φ| > 1. Return a failure result, never NaN and never an exception.
- With `checkLimits = true` (the default), a solution outside the joint limits of §3.4 is a failure with reason `JointLimit`. With `checkLimits = false` the limits are ignored and the solution of the formulas above is returned. The angles are not wrapped; elbow-down solutions can lie outside the limits (q3 ≤ −β), which is why the option exists.
- The two "reach back over the base" solutions [report §3.3] are not needed.

### 4.4 Jacobian and singularities [eqs. 8–9]

Planar Jacobian of (r, z, ψ) with respect to (q2, q3, q4), with s2 = L2 sin(θ2−β), c2 = L2 cos(θ2−β), si = Li sin θi, ci = Li cos θi for i = 3, 4:

```
Jp = [[−(s2+s3+s4), −(s3+s4), −s4],
      [−(c2+c3+c4), −(c3+c4), −c4],
      [ 1,           1,          1 ]]
det Jp = −L2 L3 sin(q3 + β)
```

Singular when |sin(q3 + β)| is small, or when r ≈ 0 (joint 1 no longer moves the tool point). Code that inverts Jp must check the determinant against a threshold (default 1e-4) and refuse.

## 5. Dynamics

### 5.1 Equations of motion [eq. 11]

```
M(q) q̈ + C(q, q̇) q̇ + g(q) = τ − τf(q̇)
```

### 5.2 Required implementation: general algorithms

The simulator computes dynamics with general algorithms over the chain of Section 4.1, so that a 6-DOF arm needs new data only:

- Inverse dynamics by the recursive Newton–Euler algorithm (Luh, Walker and Paul). Input q, q̇, q̈, gravity vector. Output τ.
- Bias torques `h(q, q̇) = C(q, q̇) q̇ + g(q)` = RNEA(q, q̇, 0).
- Inertia matrix: column k of M = RNEA(q, 0, e_k) with gravity zero (Walker and Orin method), or the composite rigid body algorithm. Either is acceptable.
- Forward dynamics: solve `M q̈ = τ − τf − h` by Cholesky decomposition. Never invert M explicitly.
- Reflected rotor inertia N²·Jm is added to each diagonal entry of M.

### 5.3 Closed forms (used by the tests)

These are the report's eqs. (14)–(18). Tests compare the general algorithms against them. They must agree to 1e-9 relative.

Yaw entry:

```
m11 = Σi [ mi ρi² + Ixx,i sin²θi + Izz,i cos²θi − 2 Ixz,i sin θi cos θi ]
ρ1 = ℓ1 cos ε1
ρ2 = ℓ2 cos(θ2 + ε2)
ρ3 = L2 cos(θ2 − β) + ℓ3 cos(θ3 + ε3)
ρ4 = L2 cos(θ2 − β) + L3 cos θ3 + ℓ4 cos(θ4 + ε4)
```

Pitch block (Ji = Iyy,i):

```
h   = L2 ℓ3 cos(q3 + β + ε3)
k23 = L2 L3 cos(q3 + β)
k24 = L2 ℓ4 cos(q3 + q4 + β + ε4)
k34 = L3 ℓ4 cos(q4 + ε4)

M44 = J4 + m4 ℓ4²
M34 = J4 + m4 (ℓ4² + k34)
M33 = J3 + J4 + m3 ℓ3² + m4 (L3² + ℓ4² + 2 k34)
M24 = J4 + m4 (ℓ4² + k34 + k24)
M23 = J3 + J4 + m3 (ℓ3² + h) + m4 (L3² + ℓ4² + 2 k34 + k23 + k24)
M22 = J2 + J3 + J4 + m2 ℓ2² + m3 (L2² + ℓ3² + 2h) + m4 (L2² + L3² + ℓ4² + 2 k23 + 2 k24 + 2 k34)
M1k = Mk1 = 0 for k = 2, 3, 4
```

Gravity:

```
g4 = −g m4 ℓ4 cos(θ4 + ε4)
g3 = g4 − g [ m3 ℓ3 cos(θ3 + ε3) + m4 L3 cos θ3 ]
g2 = g3 − g [ m2 ℓ2 cos(θ2 + ε2) + (m3 + m4) L2 cos(θ2 − β) ]
g1 = 0
```

Coriolis matrix from the Christoffel symbols [eq. 13]:

```
C_kj = Σi c_ijk q̇i,   c_ijk = ½ ( ∂M_kj/∂q_i + ∂M_ki/∂q_j − ∂M_ij/∂q_k )
```

The test project may compute ∂M/∂q by central differences (step 1e-6).

### 5.4 Actuators and friction [eq. 20]

```
τi    = clamp(ui, −τmax, +τmax)
τf,i  = fv q̇i + fc tanh(q̇i / ωs)
```

Saturation is applied in the plant, not in the controller, because it is physics. The plant reports, for each step, which joints were saturated.

### 5.5 Payload

A payload of mass mp is a point mass rigidly attached at the tool point. When attached, body 4 is replaced by the combined body:

```
c4  = centre of mass of body 4 in F4 = (ℓ4 cos ε4, 0, −ℓ4 sin ε4)
t   = tool point in F4 = (L4, 0, 0)
m4' = m4 + mp
c'  = (m4 c4 + mp t) / m4'
I4' = I4 + m4 (|d|² 1 − d dᵀ)|d = c4 − c'  +  mp (|d|² 1 − d dᵀ)|d = t − c'
ℓ4' = √(c'x² + c'z²),  ε4' = atan2(−c'z, c'x)
```

Pick-up rule: the object is attached when the tool point is within 10 mm of the object and the tool speed is below 0.05 m/s. Release puts the object at the tool point. Both happen at a step boundary. The state (q, q̇) is continuous across the change; the parameters jump.

The payload range offered in the UI is 0 to 0.5 kg (the manufacturer's rated payload is 0.5 kg).

### 5.6 True plant versus controller model

The plant uses the true parameters. Every model-based controller holds its own copy of the parameters (its "belief"), which may differ: wrong payload, wrong g, scaled masses. This is how the model-mismatch experiments are made.

## 6. Simulation

### 6.1 State space [eq. 21]

```
ẋ = f(x, u) = [ q̇ ;  M(q)⁻¹ ( τ(u) − h(q, q̇) − τf(q̇) ) ]
```

### 6.2 Solver

- Fixed-step RK4 [eq. 22], input held constant over the step (zero-order hold).
- Default step h = 0.5 ms. Allowed range 0.1 ms to 5 ms.
- No allocations inside `Step`. Work arrays are created once.
- Euler is also provided, for the teaching comparison of Note 1.

**Step-size stability.** Near rest, friction acts like damping with coefficient d = fv + fc/ωs on each joint (the slope of §5.4 at q̇ = 0). RK4 is stable only if h·λmax < 2.78, where λmax is the largest eigenvalue of M(q)⁻¹·D and D = diag(d) (later also plus the controller's derivative gains). Above this limit a held arm chatters instead of staying still. With the defaults of §3.3 the limit is above 1 ms in every pose inside the limits; with ωs = 0.01 rad/s it was about 0.17 ms. The core provides the estimate (`StabilityEstimate.MaxStableStep`), and the UI shows a warning when the chosen h exceeds it.

### 6.3 Sampled control (software in the loop)

- The controller runs every Ts (default 2 ms), which must be an integer multiple of h.
- Between controller updates the torque command is held.
- Sensors: joint angles quantised to the encoder resolution (2π/4096). Joint speeds estimated by backward difference of the quantised angles, optionally low-pass filtered. A "perfect sensor" option exists for comparison.
- Optional measurement noise with a seeded random generator, so that runs are repeatable.

### 6.4 Faults

A fault state stops the experiment and shows why:

- a joint limit was crossed,
- the tool point went below z = 0 (floor), unless the task allows it,
- the state contains NaN or infinity.

## 7. Controllers

All controllers implement one interface: given time, measured q and q̇, and the reference (q_d, q̇_d, q̈_d), return u (4 torques). A controller never reads the true plant.

| Key | Controller | Law |
|---|---|---|
| PID | Independent-joint PID | ui = Kp,i ei + Ki,i ∫ei + Kd,i ėi, with anti-windup (integration stops while the plant reports saturation on that joint) |
| PDG | PD + gravity compensation | u = Kp e + Kd ė + ĝ(q) |
| CT | Computed torque | u = M̂(q)(q̈_d + Kd ė + Kp e) + ĥ(q, q̇) |
| ADP | Adaptive (Slotine–Li), optional | estimates the payload mass online |

e = q_d − q. Hats mean the controller's own model (Section 5.6).

Default gains are found in task T07 so that, on Earth with no payload, a 0.5 rad step on each pitch joint settles within 1 s without saturation, and are written here.

## 8. References, tasks and scenarios

### 8.1 Trajectories

- Joint space: quintic polynomial between two configurations, zero speed and acceleration at both ends, duration chosen by the user or from a speed limit.
- Cartesian straight line: the tool point moves on a straight line with a quintic time law. Joint references come from inverse kinematics at each controller sample, with fixed tool pitch and branch. Fails cleanly if any point is unreachable.

### 8.2 Tasks

| Key | Task | Success criterion |
|---|---|---|
| Reach | Move the tool to a target point and pitch | tool within 2 mm of target and stays there for 0.5 s |
| PickPlace | Pick an object at A, carry it to B, release | object released within 5 mm of B, no fault |
| Track | Follow a circle or line with the tool point | RMS tool error below a threshold set per task |
| Hold | Hold a pose while a disturbance torque pulse acts | return within 2 mm in 1 s |

### 8.3 Scenario

A scenario is the full description of one experiment: environment, payload(s), task, controller and gains, controller's model, h, Ts, sensor options, random seed, duration. It is serialisable to JSON so that any run can be repeated exactly.

## 9. Metrics

Computed by the core from the logged run, not by the UI:

- RMS joint error and maximum joint error, per joint
- tool position error: RMS, maximum, final
- settling time (tool within 2 mm and stays), overshoot
- peak |τ| per joint and percentage of time in saturation
- mechanical energy delivered ∫ Σ |τi q̇i| dt
- task success and the reason for failure

## 10. Logging

- One row per controller sample: t, q (4), q̇ (4), q_d (4), u (4), τ applied (4), saturation flags, tool x y z, payload attached.
- CSV with a header line, InvariantCulture, `.` as decimal separator.
- The scenario JSON is stored with the log.

## 11. Verification

Each item is an automated test in `RobotArm.Tests`. Tolerances in brackets.

| # | Test | Source |
|---|---|---|
| V1 | Parameters equal Section 3; L2 and β computed, not typed | — |
| V2 | FK equals `poses[*].tool_position` and `tool_pitch` in the vectors [1e-12] | §4.2 |
| V3 | IK matches `inverse_kinematics` vectors [1e-9]: `elbow_up` with `checkLimits = true`; `elbow_down` with `checkLimits = false`, and the same `elbow_down` targets return `JointLimit` with `checkLimits = true`. Round trip: q sampled inside the §3.4 limits, IK(FK(q)) on the same branch with `checkLimits = true` returns q [1e-9], for both branches. Unreachable target returns failure | §4.3 |
| V4 | Jp equals central-difference derivative of (r, z, ψ) [1e-6]; det formula [1e-12] | §4.4 |
| V5 | General M equals closed form and equals `poses[*].M` [1e-9 relative] | §5.2–5.3 |
| V6 | M symmetric and positive definite in 1000 random poses | report §3.7 |
| V7 | Bias torque with q̇ = 0 equals closed-form g and `gravity_earth` [1e-9] | §5.3 |
| V8 | zᵀ(Ṁ − 2C)z = 0 for random q, q̇, z [1e-6], using the closed-form C. V8 alone does not detect errors in the velocity terms of RNEA; the C·q̇ comparison test added in T04 does. | report §3.7 |
| V9 | Free motion (τ = 0, no friction, Jm = 0), Earth, x0 from vectors, 0.4 s at h = 2 ms: max energy drift < 1e-5 J | report §3.7 |
| V10 | Same run with joint 1 free: p1 = Σ M1j q̇j drift < 1e-7 | report §3.7 |
| V11 | Error at T = 0.4 s for h = 4, 2, 1 ms against the reference: successive ratios between 12 and 20 | report §3.7 |
| V12 | State at 0.1, 0.2, 0.4 s with h = 1 ms equals `free_motion.reference_states` [5e-5] | vectors |
| V13 | Statics: τ = g(q) holds the arm still for 1 s on every planet, payload 0 and 0.5 kg, default friction, default h (max |q̇| < 1e-9) | report §3.7 |
| V14 | Payload combination: total mass, combined centre of mass and inertia match a hand calculation; `poses` with `payload_kg = 0.5` match | §5.5 |
| V15 | Saturation: commanded 10 N·m is applied as 4.1 N·m and flagged | §5.4 |
| V16 | Determinism: same scenario and seed give identical logs | §8.3 |
| V17 | PDG, Earth, no payload: reaches a 0.5 rad step with final error < 1e-3 rad | §7 |
| V18 | CT with the exact model tracks a quintic with max error < 1e-3 rad; with the payload model wrong by 0.3 kg the error is larger | §7 |
| V19 | HighG with 0.5 kg payload at horizontal reach: gravity torque exceeds τmax (the vectors give 6.84 N·m on joint 2), and the hold test reports saturation | §3.5 |

V9 and V10 are the checks that matter most. A free arm is chaotic, so long runs are judged by conserved quantities, never by comparing trajectories.

Optional V20: cross-check against MuJoCo using the same URDF and the same torque input (Python, outside the C# solution).
