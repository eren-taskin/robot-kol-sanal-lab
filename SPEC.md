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

**Step-size stability.** Near rest, friction acts like damping with coefficient d = fv + fc/ωs on each joint (the slope of §5.4 at q̇ = 0). RK4 is stable only if h·λmax < 2.78, where λmax is the largest eigenvalue of M(q)⁻¹·D and D = diag(d). Above this limit a held arm chatters instead of staying still. Controller gains do not enter this limit: u is held constant over each step (and over each controller sample Ts ≥ h), so inside the RK4 stages the controller is a constant input and the limit depends only on the plant's inertia and friction. (Corrected in T07; the gains decide a different stability, that of the sampled closed loop, §6.3.) With the defaults of §3.3 the limit is above 1 ms in every pose inside the limits; with ωs = 0.01 rad/s it was about 0.17 ms. The core provides the estimate (`StabilityEstimate.MaxStableStep`), and the UI shows a warning when the chosen h exceeds it.

### 6.3 Sampled control (software in the loop)

- The controller runs every Ts (default 2 ms), which must be an integer multiple of h (to 1e-9 relative; otherwise the control loop throws). Sample k is at t_k = k·Ts: the sensor measures the true state, the reference is evaluated, the controller computes u.
- Between controller updates the torque command is held: the plant runs Ts/h steps with the same u (zero-order hold).
- Sensors, with the encoder resolution r = 2π/4096 = 1.53e-3 rad of each joint (§3.3):
  - Encoder: measured angle q_m = r·round((q + n)/r). Speed by backward difference of the quantised angles over the sample, v = (q_m,k − q_m,k−1)/Δt (0 at the first sample), optionally low-pass filtered: y ← y + α(v − y), α = Δt/(τ + Δt). Unfiltered, the estimate moves in steps of one count per sample, r/Ts = 0.77 rad/s at Ts = 2 ms, and differs from the true average speed over the sample by at most r/Ts.
  - Perfect sensor: the true q and q̇, for comparison.
- Optional measurement noise n ~ N(0, σ²) on each angle, from a seeded random generator, so that runs are repeatable.

**Sampled closed-loop stability.** A second kind of stability, separate from the step-size limit of §6.2: the controller's gains act on samples held for Ts, so the loop is a discrete-time system with its own limit on Ts. Linearise the plant at rest at a pose q0 for a given planet and payload, x = [δq; δq̇]:

```
A = [[0, I], [−M⁻¹G, −M⁻¹D]],   B = [[0], [M⁻¹]]
```

with M = M(q0), G = ∂g/∂q at q0 (central differences of the bias torque with q̇ = 0) and D = diag(fv + fc/ωs). Discretise with zero-order hold over Ts: Φ = e^{A·Ts} and Γ = ∫₀^Ts e^{Aτ}dτ·B, both by integrating the linear system with RK4 sub-steps (at least 200 per Ts) from the unit columns, no matrix exponential. A controller linearised at rest is u_k = −K x_k with the effective gains K of §7, so x_{k+1} = (Φ − ΓK) x_k, stable when the spectral radius ρ(Φ − ΓK) < 1. ρ is found without an eigen solver by Gelfand's formula ρ = lim ‖P^m‖^{1/m}, m = 2^k, by repeated squaring with rescaling until the estimate stops changing (about 1e-13; 12 squarings leave errors of about 1e-4). MaxStableTs is the first Ts at which ρ reaches 1, found by doubling and bisection. The core provides both (`SampledLoopStability`). The analysis is local: one pose, at rest, no saturation, no quantisation. The results for the default gains are in §7.

### 6.4 Faults

A fault state stops the experiment and shows why:

- a joint limit was crossed,
- the tool point went below z = 0 (floor), unless the task allows it,
- the state contains NaN or infinity.

## 7. Controllers

All controllers implement one interface: given time, measured q and q̇, and the reference (q_d, q̇_d, q̈_d), return u (4 torques). A controller never reads the true plant. It knows τmax from its own model (its belief, §5.6), clamps its own output to ±τmax and reports which joints it clamped (an output, for logging). It is never told whether the plant saturated.

| Key | Controller | Law |
|---|---|---|
| PID | Independent-joint PID | ui = Kp,i ei + Ki,i ∫ei dt + Kd,i ėi, clamped to ±τmax,i. Anti-windup by conditional integration: on a joint whose output it clamped, the controller keeps the old integral for that sample. The integral is a rectangle sum over the time between samples (nothing at the first sample). No gravity model. |
| PDG | PD + gravity compensation | u = Kp e + Kd ė + ĝ(q), clamped |
| CT | Computed torque | u = M̂(q̄)(q̈_d + Kd ė + Kp e) + ĥ(q̄, q̄̇) + τ̂f(q̄̇), clamped. τ̂f is the friction of §5.4 with the belief's parameters. (q̄, q̄̇) is the measured state predicted to the middle of the hold: with a = q̈_d + Kd ė + Kp e and Δt the time since the previous sample, q̄ = q + q̇·Δt/2 + a·Δt²/8, q̄̇ = q̇ + a·Δt/2. The prediction is an option, on by default; without it q̄ = q and q̄̇ = q̇, the textbook law. |
| CT+I | Optional integral computed torque | Same CT law and prediction, with a = q̈_d + Kd ė + Kp e + Ki ∫e. Off by default. Candidate rectangle integral uses elapsed sample time (zero at the first sample); a joint whose candidate output is clamped keeps its old integral, as in PID. |
| ADP | Adaptive (Slotine–Li), optional | estimates the payload mass online |

e = q_d − q and ė = q̇_d − q̇ with the measured q and q̇. Hats mean the controller's own model (Section 5.6). Gains are per joint (diagonal matrices).

Linearised at rest at q0 with the constant reference q_d = q0 (used by §6.3), each law is a PD-type feedback δu = −K [δq; δq̇], with Ĝ = ∂ĝ/∂q and D̂ = diag(fv + fc/ωs) of the belief:

- PID: K = [Kp | Kd]. The integral is left out; gravity is not compensated, so G stays in the plant.
- PDG: K = [Kp − Ĝ | Kd]. ĝ is computed from the sample and held, so Ĝ belongs to the sampled feedback.
- CT: K = [P Kp − Ĝ | P Kd − Ĝ·Ts/2 − D̂] with P = M̂ + Ĝ·Ts²/8 + D̂·Ts/2 (Ts = 0 without the prediction).

**Criterion for the default gains.** On Earth with no payload, exact belief, perfect sensor, default h and Ts: from rest at A = [0, −0.6, −0.3, −0.3] rad to the set point B = A + [0, 0.5, 0.5, 0.5] rad, and from B back to A, every joint stays within 0.01 rad (2 % of the step) of the set point from at most 1 s on, and the plant never saturates; PID and PDG also never clamp their output, while CT's A1c gains clamp briefly at the start of the discontinuous step (docs/DECISIONS.md A1c). The tool is 0.39 m above the floor at A and 0.14 m at B. A PD-type law starts with |u| = Kp·0.5 + |ĝ|, so Kp·0.5 must stay well below 4.1 N·m.

**Default gains** (T07; CT retuned in A1c, CT+I in A1b):

| Gain | Joint 1 | Joint 2 | Joint 3 | Joint 4 | Unit |
|---|---|---|---|---|---|
| PDG and PID Kp | 6 | 6 | 6 | 2 | N·m/rad |
| PDG and PID Kd | 0.3 | 0.6 | 0.3 | 0.05 | N·m·s/rad |
| PID Ki | 5 | 20 | 20 | 8 | N·m/(rad·s) |
| CT Kp | 1600 | 1600 | 1800 | 1800 | 1/s² |
| CT Kd | 350 | 350 | 350 | 350 | 1/s |
| CT+I Kp | 3600 | 3600 | 3600 | 3600 | 1/s² (ωn = 60 rad/s) |
| CT+I Kd | 360 | 360 | 360 | 360 | 1/s (ζ = 3) |
| CT+I Ki | 10000 | 10000 | 10000 | 10000 | 1/s³ |

**Measured with these gains** (T07, A1c; perfect sensor, default h = 0.5 ms and Ts = 2 ms). Settling time in s for A→B / B→A; C = the controller clamped its output; F = fault (§6.4); ">2" = not settled after 2 s.

| Planet | Payload | PID | PDG | CT |
|---|---|---|---|---|
| Earth | 0 | 0.83 / 0.75 | 0.29 / 0.33 | 0.87 C / 0.85 C |
| Earth | 0.5 kg | F floor / >2 C | 1.68 / 1.97 C | 0.96 C / 1.07 C |
| Moon | 0 | 0.83 / 0.79 | 0.29 / 0.33 | 0.87 C / 0.85 C |
| Moon | 0.5 kg | F floor / >2 | 1.68 / 1.96 | 0.96 C / 0.99 C |
| Mars | 0 | 0.83 / 0.78 | 0.29 / 0.33 | 0.87 C / 0.85 C |
| Mars | 0.5 kg | F floor / >2 | 1.68 / 1.96 | 0.96 C / 1.00 C |
| HighG | 0 | 1.14 / 1.20 | 0.29 / 0.33 | 0.87 C / 0.86 C |
| HighG | 0.5 kg | F floor / F floor | F floor / F floor | F limit j3 / F limit j4 |
| Orbit | 0 | 0.83 / 0.81 | 0.29 / 0.33 | 0.87 C / 0.85 C |
| Orbit | 0.5 kg | F floor / >2 | 1.68 / 1.96 | 0.96 C / 0.98 C |

Peak |u| on Earth without payload: PID 3.70, PDG 3.58, CT 4.10 N·m (clamped at the initial sample of the discontinuous step; on smooth quintics such as V18(a) and the Earth reach scenario, peak |u| is 1.31 N·m without clamping). With 0.5 kg the PID's integral winds up during the slow rise and overshoots into the floor even in orbit; CT settles without fault on Earth, Moon, Mars and Orbit with both 0 and 0.5 kg, and faults only on HighG with 0.5 kg where the static gravity torque exceeds 4.1 N·m.

Sampled-loop stability (§6.3) at the poses A, B and C = [0.5, −1.0, 0.3, 0.7], every planet, 0 and 0.5 kg:

| Controller | ρ at Ts = 2 ms | MaxStableTs |
|---|---|---|
| PID (PD part) | 0.9845–0.9944 | 6.5–29.7 ms; HighG, 0.5 kg, pose A: unstable at any Ts (ρ = 1.008), Kp cannot hold the gravity gradient |
| PDG | 0.9845–0.9945 | 6.5–29.5 ms |
| CT | 0.9908–0.9908 | 5.7–5.7 ms |

The smallest MaxStableTs for PID/PDG, 6.5 ms at pose A without payload, is set by the yaw joint, whose inertia m11 = 8.1e-4 kg·m² is small when the arm leans back over the base; for CT, Kd = 350 1/s sets MaxStableTs ≈ 5.7 ms across poses A, B, C. ρ close to 1 at 2 ms means slow modes, not a stability problem: ρ = e^{−σ·Ts} with σ ≈ 3–11 1/s, the decay rate of the slowest mode.

**CT and CT+I robustness (T08, A1b, A1c, D8, D9).** With the original T07 plain CT defaults (Kp = 144 1/s², Kd = 24 1/s), the unloaded wrist at Pose B in Orbit had only 0.34 dB gain margin and 15.91° phase margin (and 10.31° on joint 3 at HighG, 0.5 kg, reach start pose) because P44·Kd = 0.0139 < d4 = 0.21 N·m·s/rad left the net linearised velocity feedback negative (friction cancellation dominating Kd). Retuning plain CT in A1c to Kp = [1600, 1600, 1800, 1800] 1/s² and Kd = [350, 350, 350, 350] 1/s gives ρ ≤ 0.9908 < 1, gain margin ≥ 7.19 dB ([9.15, 9.36, 10.75, 7.19] dB worst case per joint) and phase margin ≥ 62.51° ([69.24°, 71.83°, 89.68°, 62.51°] worst case per joint) across all five planets, payloads 0 and 0.5 kg, and all library scenario start poses. Enabling CT+I in a scenario selects the separately tuned defaults above (Kp = 3600 1/s², Kd = 360 1/s, Ki = 10000 1/s³), unless gains are explicitly supplied: across the same 30 cases its 12-state augmented closed loop (§12.2) has ρ ≤ 0.9898 < 1, loop-at-a-time gain margin ≥ 7.43 dB (≥ 6 dB) and phase margin ≥ 46.57° (≥ 40°) on every joint (9.21 dB and 51.94° on the wrist at the reach start pose). In V18(b), with plant 0.3 kg and belief 0 kg, the same 1 s quintic and 0.5 s hold (end t = 1.5 s) give maximum final joint errors: ordinary CT **0.5542987672 rad**, CT+I **0.0069754717 rad** (< 1e-2), and CT with CT+I's Kp/Kd but Ki = 0 **0.3376327568 rad**. All are fault-free. Peak errors over the run are 0.559830798 rad for ordinary CT and 0.2483847706 rad for CT+I: this is rejection of a persistent disturbance, not elimination of the initial transient. The library runs the same move to 10 s to demonstrate task-level settling (settling at 1.28 s). The PD-only linearisation of §6.3 excludes the integral state, whereas §12.2–§12.4 include it.

## 8. References, tasks and scenarios

### 8.1 Trajectories

- Joint space: quintic polynomial between two configurations, zero speed and acceleration at both ends, duration chosen by the user or from a speed limit.
- Cartesian line or horizontal XY circle: quintic time law (one turn for a circle), zero endpoint speed/acceleration and fixed tool pitch/branch. IK supplies joint positions at every sample. Analytic Cartesian velocity and acceleration are transformed using Jp⁻¹ and analytic J̇p q̇; near-singular samples are refused using §4.4's threshold, including the yaw-axis singularity. Unreachable, limit-violating, singular or yaw-branch-cut paths produce a named task failure.
- Reach and both PickPlace legs always use joint quintics. Track uses a Cartesian line or circle and holds the last point after completion. Hold starts at its reference pose. No task introduces a step reference; the discontinuous criterion experiments in §7 remain separate controller tests.

### 8.2 Tasks

| Key | Task | Success criterion |
|---|---|---|
| Reach | Move the tool to a target point and pitch | tool within 2 mm of target and stays there for 0.5 s |
| PickPlace | Pick an object at A, carry it to B, release | object released within 5 mm of B, no fault |
| Track | Follow a circle or line with the tool point | RMS tool error below a threshold set per task |
| Hold | Hold a pose while a disturbance torque pulse acts | return within 2 mm in 1 s |

PickPlace begins with a detached object at A. It approaches A over `MotionDuration`, then holds until the measured true tool satisfies §5.5's 10 mm / 0.05 m/s pickup gate. Attachment occurs at a controller sample boundary (also a plant step boundary), and a second quintic starts from the first leg's exact endpoint with zero reference velocity/acceleration. After this leg, release requires distance to B ≤ 5 mm and speed < 0.05 m/s. Each event has a configurable timeout (default 2 s); the object remains at its actual release position. Zero-mass objects still have an attachment state. No fault is ignored after release; the arm keeps holding until the run ends.

Hold applies a configurable external joint torque pulse **after motor saturation**, separately from actuator torque. It succeeds only when the pose returns within 2 mm no later than 1 s after the pulse ends and stays there to the run end; the run must cover that deadline. With the pulse disabled, the pose must stay within 2 mm throughout the logged run.

### 8.3 Scenario

A `Scenario` record contains environment (including custom g), payload, task geometry and timing, controller and optional gain overrides, independent belief (g, body mass/inertia scale, initial payload), h, Ts, sensor options, seed, duration, Turkish name and explanation. Null gain overrides select the documented law-specific defaults; a null belief g/payload selects the initial true value. `ScenarioJson` uses System.Text.Json with named enums, vector support and validation. Record/value data cannot retain caller-owned mutable arrays.

Payload knowledge is selectable: **Informed** updates the controller's payload belief at PickPlace attachment/release, without resetting integral or sample history; **Unaware** retains its initial payload belief throughout. Other belief parameters never change. PickPlace's plant initially has no payload; the scenario's payload is the object mass. Other tasks start with that payload attached.

Duration must be finite, in **(0, 60] simulated seconds**, and an integer multiple of h. h obeys §6.2; Ts must be a positive integer multiple of h. Validate initial joint limits/floor, finite settings, enum values, nonnegative gains/noise/filter settings and payload in 0..0.5 kg before running. Unreachable task targets/paths return a failed result with a reason, rather than a physics exception.

`ExperimentRunner.Run(scenario)` and `Start(scenario)`, `AdvanceBy(simSeconds)`, `Current`, `IsFinished` use the same state machine. Start computes the t=0 sample. AdvanceBy accumulates fractional h remainders across calls and advances whole h steps only, never beyond duration; zero is a no-op and invalid/nonfinite increments are rejected. Current is an allocation-free state snapshot for animation; Log is a read-only live view and Result is available on completion. Restarting creates new storage, preserving prior results. No wall clock or UI dependency enters Core.

ScenarioLibrary provides eight Turkish-named experiments (tested against their declared outcomes): Dünya – boş kol – noktaya git; Ay – 0,3 kg – al ve bırak; Mars – çember izleme; Yüksek g – 0,5 kg – tutma (doyum); Yörünge – noktaya git; Yanlış model – CT, 0,3 kg habersiz yük; Yanlış model düzeltildi – CT+I, 0,3 kg habersiz yük; Örnekleme – Ts sınırına yakın. The last uses exactly 0.9 × MaxStableTs at its initial pose, with h chosen to divide Ts and a small disturbance at that pose. Explanations state the expected successes and the overload/model-mismatch failures.

## 9. Metrics

Computed by `RunMetrics` from the logged true states (not sensor readings). Each sample is weighted by its actual held interval, including a shortened final interval or a fault between samples:

- RMS and maximum absolute joint-reference error, per joint (rad).
- Tool-reference distance: RMS, maximum, final (m); final means the last logged sample. Track also reports RMS over its moving interval only, so a long final hold cannot dilute tracking error.
- Settling time is an absolute time from run start: the first logged sample at/after the final reference hold begins from which tool distance to the fixed final target remains ≤ 2 mm until the end. Null means no settling. Reach additionally requires 0.5 s of this final hold. A fault always makes the task fail even if a settling number exists for its truncated log.
- Overshoot (m): largest positive projection beyond the final target onto the unit direction from initial tool position to final target; zero for a zero-length move (including a circle returning to its start).
- Peak |τ| per joint; time percentages of controller clamping and physical motor saturation separately, per joint; percentage with any controller joint clamped. These are not interchangeable (D6).
- Mechanical energy delivered: left-rectangle sample estimate Σk Δtk Σi |τi,k q̇i,k| (J), using motor torque only, excluding external disturbance torque.
- Payload attachment/release transition counts and maximum attached mass, derived from the log.
- `RunResult` includes task success, the task/fault failure reason, the physical fault (if present), and the actual terminal state. A failed target before the first sample has an empty log and zero aggregate metrics, with no settling time.

The Report-category test runs Reach and PickPlace for PID, PDG, CT and CT+I on every planet with 0, 0.1, 0.3 and 0.5 kg (160 runs, informed belief), writing `docs/results/robustness.csv`. CI excludes Category=Report; run it explicitly in Release to regenerate the report and measure a warmed 10 s Earth PickPlace CT run.

## 10. Logging

- Exactly one `LogRow` per controller sample at t=k·Ts < duration, including t=0. All state/reference columns describe that instant, **before** integration with the new command. No synthetic terminal/controller row is appended at a fault or at duration.
- Columns: sample time and actual held interval; q, q̇, q_d, q̇_d, q̈_d; controller command u and applied motor τ; separate controller clamp and plant saturation flags per joint; true and reference tool xyz; attachment flag, true and believed payload kg; task phase, object xyz, and external disturbance torque at the sample.
- Rows are value types in storage reserved at Start. Normal controller samples, trajectory evaluation, logging and plant steps allocate nothing; payload events rebuild model buffers, and completion allocates the result/metrics.
- `CsvLogWriter.Write(TextWriter, log)` writes a header and one row per sample with InvariantCulture and round-trip numeric formatting (`.` decimal separator); no file path API in Core. The complete scenario JSON is stored in RunResult beside the log for paired export/replay.

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
| V16 | Full Scenario JSON round trip and identical seed produce bit-identical logs, including PickPlace events with noisy filtered encoders; synchronous and irregular frame batches agree, while a different seed changes the log | §8.3 |
| V17 | PDG, Earth, no payload, perfect sensor: the 0.5 rad step A → B of §7 ends with final error < 1e-3 rad | §7 |
| V18 | CT (default gains and prediction), perfect sensor, quintic [0, −0.4, −0.3, 1.1] → [0, 0.1, −0.8, 0.6] rad in 1 s, held to 1.5 s (tool at least 0.15 m above the floor and inside the joint limits along the path): (a) exact model, max error < 1e-3 rad; (b) plant 0.3 kg, belief 0 kg and (c) plant 0 kg, belief 0.3 kg, max error at least 5 times (a); all three without a fault. The same runs with the textbook CT are reported | §7 |
| V19 | HighG, 0.5 kg, Hold at test_vectors.json horizontal_reach_pose (verified inside all joint limits), pulse off: joint-2 gravity torque magnitude ≈ 6.84 N·m > 4.1; PDG reports controller clamping on joint 2 from t=0. Task fails (pose not held within 2 mm, or floor fault) with a recorded failure reason | §7–§9 |

V9 and V10 are the checks that matter most. A free arm is chaotic, so long runs are judged by conserved quantities, never by comparing trajectories.

Added in T07:

| # | Test | Source |
|---|---|---|
| V21 | Default gains meet the criterion of §7 for PID, PDG and CT (both directions) | §7 |
| V22 | Ts not a positive integer multiple of h throws; Ts/h plant steps per sample; the command is held between samples | §6.3 |
| V23 | Each controller clamps to τmax of its own model; PID keeps its integral on a clamped joint and integrates again once unclamped | §7 |
| V24 | Encoder: quantisation r = 2π/4096; the §7 step ends within 2 counts (PDG and PID with and without filter, CT with τ = 5 ms); the speed estimate differs from the true average speed over the sample by at most r/Ts and reaches more than half of it (visible); the filter lowers its RMS error; the noise is seeded | §6.3 |
| V25 | Sampled loop: Φ, Γ of a 1-DOF mass–damper–spring equal the closed form [1e-12] and ρ equals the closed form [1e-6]; the arm's A, B equal the plant's Jacobian [1e-5]; each controller's K equals −∂u/∂x of its law [1e-6] | §6.3, §7 |
| V26 | Default gains: ρ < 1 at Ts = 2 ms and MaxStableTs > 2 ms for every planet × payload (0, 0.5 kg) × poses A, B, C (PID wherever its continuous loop is stable); on Earth without payload at A, B, C, ρ < 1 at 0.99 × and ρ > 1 at 1.01, 2 and 4 × MaxStableTs | §6.3 |
| V27 | Prediction against simulation: holding pose B from a 1e-4 rad offset, the simulated error grows at 1.1 × MaxStableTs and decays at 0.9 × MaxStableTs (PDG 0 and 0.5 kg, CT) | §6.3 |
| V28 | Compute, Measure and one control-loop sample allocate nothing | PROJECT §3 |

Added in T08:

| # | Test | Source |
|---|---|---|
| V29 | V18(b), same 1 s move + 0.5 s hold: CT+I with its §7 defaults has final max joint error < 1e-2 rad without fault; report ordinary CT and same-PD-gain CT without I | §7 |
| V30 | Earth CT success for all four tasks; Track line and circle; PickPlace attachment/release and informed/unaware belief logs; every library scenario matches its Turkish explanation | §8 |
| V31 | Quintic endpoint conditions and input copying; Cartesian path, analytic derivatives vs central differences, unreachable/singular/branch-cut failures; continuous PickPlace references across events | §8.1 |
| V32 | Log-derived metrics agree with hand calculations, including partial sample weights, moving-only Track RMS, clamping vs saturation and payload transitions | §9 |
| V33 | Exactly one row per controller sample, partial final interval, validated duration ≤60 s; complete CSV numeric round trip under Turkish culture; custom scenario/gain/belief JSON round trip | §8.3, §10 |
| V34 | CT+I conditional integration, reset, payload-update state preservation, off-by-default regression; normal runner steps including logging allocate nothing | §7, §10 |
| V35 | Report category (excluded from CI): 160-run informed robustness CSV and Release 10 s Earth PickPlace CT wall-clock measurement | §9 |

Added in A1 (linear analysis, §12):

| # | Test | Source |
|---|---|---|
| V36 | Eigenvalues: diagonal, companion matrix of 9 known roots [1e-9], rotation block (exact conjugate pair), 16 × 16 similarity V D V⁻¹ [1e-10]; on 50 random 12 × 12 matrices Σλ = trace [1e-12·‖A‖F] and Πλ = det [1e-9 relative]; max \|λ\| equals the §6.3 spectral radius on the arm for PID, PDG, CT × every planet × 0, 0.5 kg × poses A, B, C [1e-9] | §12.1 |
| V37 | Closed-loop model: 8 poles for PDG and CT, 12 for PID and CT+I; ρ equals §6.3 for the PD-type laws [1e-9]; for a 1e-6 step and offset the augmented model reproduces each controller's own `Compute` driving x_{k+1} = Φx + Γ(u − u0) over 200 samples [1e-3 of the signal]; s = ln(z)/Ts round trip | §12.2 |
| V38 | Root locus: 1-DOF mass–spring–damper with sampled PD, poles equal the roots of the closed-form characteristic polynomial [1e-12] and the critical scale factor equals the Jury closed form [1e-8 relative], also with Φ, Γ by RK4 sub-steps; on the arm, tracks are continuous (relative pole step ≤ 0.05) and each crossing separates ρ < 1 from ρ > 1 at ±1e-6 | §12.3 |
| V39 | Frequency response: 1-DOF L(e^{jωTs}) equals the analytic discrete loop [1e-9 dB, 1e-7°]; gain margin (dB) = 20·log10 of the critical scale factor of V38 [1e-6 dB]; \|L(ω_gc)\| = 1; on the arm (PDG, CT+I) L is real and negative at ω_pc, and scaling joint i's row of K_ξ by the gain margin puts ρ on 1 [1e-6]; across all five planets × payloads 0 and 0.5 kg × library scenario start poses, the default gains of all four controllers keep ρ < 1 and both CT and CT+I keep gain margin ≥ 6 dB and phase margin ≥ 40° on every joint | §12.4 |
| V40 | Linear step response against the nonlinear run at the start pose of "Dünya – boş kol – noktaya git": CT and CT+I at 0.01 rad and PDG at 0.003 rad on every joint agree within 2 % of the step (and PDG at 0.01 rad with friction replaced by its slope); the 0.01 rad PDG step on the true plant (3.26 %, within 5 %) is kept as a documented observation of the Coulomb friction band (D9); 0.5 rad reported; linear PDG ends at q0 + δr [1e-8] | §12.5 |
| V41 | LQR: scalar DARE against its closed form [1e-12]; arm DARE residual < 1e-9 and LQR loop stable on every planet × 0, 0.5 kg × poses A, B, C; the projection is the diagonal of K | §12.6 |
| V42 | Gain tuner: optimises over a scenario or a list of scenarios (`Sum` or `Max` cost), optionally evaluating each candidate with both perfect and encoder sensors (`PerfectAndEncoder`), with minimum gain margin and phase margin enforced as hard constraints via `LoopFrequencyResponse` at each scenario's start pose; tuning on "Dünya – boş kol – noktaya git" with both sensors (10 evaluations in the normal suite, 60 evaluations in the Report category) gives gains that pass that scenario with the encoder; same seed gives the same history; cancellation returns the evaluations done | §12.7 |
| V43 | Report category (excluded from CI): every analysis call on the arm (except the tuner) takes less than 50 ms natively, median of 7 warm calls | §12.8 |

Optional V20: cross-check against MuJoCo using the same URDF and the same torque input (Python, outside the C# solution).

## 12. Linear analysis

A toolkit in `RobotArm.Core/Analysis` (task A1) for the student to study the sampled loop as a linear system: poles, root locus, frequency response, a linear step response to overlay on a run, LQR, and a gain tuner. It uses the linearisation of §6.3 (`LinearSystem.ArmAtRest`, Φ and Γ by RK4 sub-steps, the effective gains of §7). No package is used; `System.Numerics.Complex` is. The calls allocate; they are analysis, not once per step. Decisions and measurements are in `docs/DECISIONS.md` D9.

**What is approximated** (everything here, for the arm). One pose q0, at rest; friction replaced by its slope at q̇ = 0, d = fv + fc/ωs (valid only for |q̇| well below ωs = 0.1 rad/s); gravity by its gradient G; Coriolis terms dropped (quadratic in q̇); no saturation or clamping, no conditional integration; a perfect sensor (no quantisation, noise or filter); every sample exactly Ts after the previous one.

### 12.1 Eigenvalues

`EigenSolver.Eigenvalues(A, n)`: balancing by powers of 2, reduction to upper Hessenberg form, Francis double-shift QR with deflation (EISPACK hqr). Sorted by decreasing |λ|; complex pairs are exactly conjugate. Used for every pole set below; tested up to 16 × 16.

### 12.2 Closed-loop model and poles

Each controller supplies, besides K of §7 (`IAugmentedLinearisableController`), how a step δr of the constant reference and the integral enter its law, linearised at rest at q0:

```
δu_k = −K x_k + L δr + Kz w_k,    w_k = w_{k−1} + Ts (δr − δq_k)
```

| Law | L | Kz |
|---|---|---|
| PID | diag(Kp) | diag(Ki) |
| PDG | diag(Kp) | — |
| CT | P diag(Kp) | — |
| CT+I | P diag(Kp) | P diag(Ki) |

with P = M̂ + Ĝ·Ts²/8 + D̂·Ts/2 of §7 (Ts = 0 without the prediction; the integral always uses the elapsed Ts). w is the deviation of ∫e dt from the value that holds the equilibrium (for PID, Ki·∫e = g(q0)).

PD-type laws (PDG, CT): ξ = x, 8 states, ξ_{k+1} = (Φ − ΓK) ξ_k + ΓL δr. With an integral (PID, CT+I): ξ = [x; w_{k−1}], 12 states,

```
A_ξ = [[Φ, 0], [−Ts S, I]],  B_ξ = [[Γ], [0]],  E_ξ = [[0], [Ts I]],  S = [I 0]
K_ξ = [K + Ts Kz S, −Kz],    L_ξ = L + Ts Kz
ξ_{k+1} = (A_ξ − B_ξ K_ξ) ξ_k + (B_ξ L_ξ + E_ξ) δr,    u_k = u0 − K_ξ ξ_k + L_ξ δr
```

`LinearAnalysis.ClosedLoopPoles(arm, planet, payload, q0, controller, Ts)` returns the z-poles, s = ln(z)/Ts (principal branch, |Im s| ≤ π/Ts), ωn = |s| and ζ = −Re(s)/|s| of each, and ρ = max |z|.

### 12.3 Root locus

`RootLocus.Sweep` varies one parameter on a logarithmic grid: a common factor k on every gain (Kp, Kd, Ki of every joint), or one gain of one joint. A step is split at its geometric midpoint while a pole moves more than 0.05·max(1, |z|); poles are matched between steps by nearest pairs, so each track is continuous. Every sign change of ρ − 1 is bisected on log(value) (relative 1e-10); the first stable-to-unstable crossing is the critical value.

### 12.4 Loop-at-a-time frequency response

The loop of joint i is opened at its input with the other loops closed: A_i = A_ξ − Σ_{j≠i} b_j k_j and L_i(z) = k_i (zI − A_i)⁻¹ b_i at z = e^{jωTs}, ω from 1e-4·π/Ts to the Nyquist frequency π/Ts (included). Returned: magnitude (dB), phase (deg, unwrapped), gain margin at the phase crossover (an odd multiple of −180°) with the smallest |margin|, phase margin at the gain crossover with the smallest margin, both crossover frequencies (bisection), and the number of open-loop poles outside the unit circle (when not 0, the Bode margins alone do not decide stability). The gain margin is the factor on joint i's whole row of K_ξ (gains, integral and the model terms). For a single loop it equals 20·log10 of the critical scale factor of §12.3. Loop-at-a-time margins do not cover simultaneous changes in several joints.

### 12.5 Linear step response

`LinearStepResponse.Compute` runs ξ_{k+1} = A_cl ξ_k + B_cl δr from ξ0 = 0 for a step δr of the joint reference at t = 0 and returns q, q̇ and δu at every controller sample, like the log rows of §10. It agrees with the nonlinear run within 2 % for small steps (V40): 0.01 rad for CT (0.42 %) and CT+I (0.79 %), and 0.003 rad for PDG (0.32 %, or 0.08 % at 0.01 rad when Coulomb friction is replaced by its slope). On the true plant at 0.01 rad, PDG's yaw joint reaches 0.12 rad/s > ωs = 0.1 rad/s and the prediction differs by 3.26 %, kept as a documented observation of the Coulomb friction band. CT cancels friction and inertia and stays close to linear even at 0.5 rad (5.39 %, with brief controller clamping at t = 0).

### 12.6 Discrete LQR

`DiscreteLqr` solves the DARE P = ΦᵀPΦ − ΦᵀPΓ(R + ΓᵀPΓ)⁻¹ΓᵀPΦ + Q for diagonal Q (positions, velocities) and R by structure-preserving doubling, then two plain Riccati iterations, and returns the 4 × 8 gain K = (R + ΓᵀPΓ)⁻¹ΓᵀPΦ, the poles of Φ − ΓK, the relative residual, and the diagonal projection Kp,i = K[i, i], Kd,i = K[i, 4 + i] with the poles and ρ of Φ − Γ[diag(Kp) | diag(Kd)] (the same plant under an independent-joint PD), so the student sees what the decentralised structure loses.

### 12.7 Gain tuner

`GainTuner.Tune(scenarios, options, progress, token)` (and the single-scenario overload): Nelder–Mead over per-joint (ωn, ζ) or one shared pair, bounded (ωn on a log scale, ζ linear, trial points clamped to the box), seeded and deterministic. Gains from (ωn, ζ): CT and CT+I Kp = ωn², Kd = 2ζωn; PDG and PID Kp = M̂ii ωn², Kd = max(0, 2ζωn M̂ii − d̂i) with M̂ii at the primary scenario's start pose of the controller's own model; Ki is kept. At each scenario's start pose, `SampledLoopModel` and `LoopFrequencyResponse` check ρ < 1, minimum gain margin (`GainMarginLimitDb`, default 6 dB) and minimum phase margin (`PhaseMarginLimitDeg`, default 40°) as hard constraints (infeasible candidates are rejected with a barrier penalty without running the nonlinear simulation). Feasible candidates run the nonlinear `ExperimentRunner` on each scenario, either with the scenario's own sensor (`TuningSensorMode.Scenario`) or with both the perfect sensor and the encoder (`TuningSensorMode.PerfectAndEncoder`), combining per-run costs (ITAE Σ t·|p_tool − p_ref|·Δt plus fault and clamping penalties) by sum (`TuningCostAggregation.Sum`) or worst-case maximum (`TuningCostAggregation.Max`). Evaluation 1 is the primary scenario's own gains. Budget (default 60 evaluations), a cancellation token (checked between runs and every 0.25 s of simulated time) and a progress callback after every evaluation.

### 12.8 Speed

Each call on the arm takes less than 50 ms natively, the tuner excepted (V43). Measured (Release, Apple Silicon, median of 7 warm calls): eigenvalues of a 12 × 12 matrix 0.05 ms; closed-loop poles 3.4–3.5 ms (mostly the discretisation); root locus 20–32 ms; frequency response of one loop, 400 points, 3.9–5.5 ms; step response over 2 s 3.9 ms; LQR 3.8 ms. One tuner evaluation of the 10 s reach takes about 70 ms in Release and 1.5 s in Debug; the 60-evaluation tuner test is in the Report category while a 10-evaluation test runs in the normal suite.

### 12.9 Default PDG gains, Earth, start pose of "Dünya – boş kol – noktaya git"

q0 = [0, −0.4, −0.3, 1.1] rad, no payload, Ts = 2 ms. ρ = 0.984494.

| z | ωn (rad/s) | ζ |
|---|---|---|
| 0.984494 | 7.81 | 1 |
| 0.982959 | 8.59 | 1 |
| 0.975141 | 12.59 | 1 |
| 0.968845 | 15.83 | 1 |
| 0.945996 | 27.76 | 1 |
| 0.641218 | 222.2 | 1 |
| 0.535141 | 312.6 | 1 |
| −0.358365 | 1652 (s = −513 + 1571j) | 0.31 |

| Joint | Gain margin | ω_pc | Phase margin | ω_gc | Open-loop unstable poles |
|---|---|---|---|---|---|
| 1 | 18.34 dB | 1571 rad/s (Nyquist) | 115.3° | 90.9 rad/s | 0 |
| 2 | 16.34 dB | 1571 rad/s | 101.7° | 36.4 rad/s | 1 |
| 3 | 10.70 dB | 1571 rad/s | 120.5° | 42.3 rad/s | 1 |
| 4 | 12.64 dB | 1571 rad/s | 102.6° | 9.75 rad/s | 0 |

Critical scale factor on all gains: k = 2.3279 (7.34 dB), smaller than every loop-at-a-time margin: raising every joint's gains together is less safe than raising one. With loop 2 or 3 opened, the others closed, the arm has one unstable pole (gravity), so for those joints the Bode margins are read with the Nyquist criterion.
