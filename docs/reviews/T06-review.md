# T06 — Independent Review of Tasks T02–T05

Reviewer: Antigravity (Gemini)  
Scope: `src/RobotArm.Core` (Model, Kinematics, Dynamics, Simulation) and `tests/RobotArm.Tests` against `PROJECT.md`, `SPEC.md` (§2–§6, §11), and `docs/DECISIONS.md` (D1–D4).  
Build & test status: `dotnet build` succeeds with 0 warnings and 0 errors; `dotnet test` passes all 230 tests.

---

## 1. Line-by-Line Verification Against SPEC §3–§6

### 1.1 SPEC §3 — Parameters (`src/RobotArm.Core/Model`)

- **§3.1 Geometry (`OpenManipulatorXGeometry.cs`):**
  - `D1 = 0.0595`, `A2X = 0.024`, `A2Z = 0.128`, `L3 = 0.124`, `L4 = 0.126` (lines 10–22) match Table 3 exactly.
  - `L2 = Math.Sqrt(A2X * A2X + A2Z * A2Z)` (line 25) and `Beta = Math.Atan2(A2Z, A2X)` (line 28) are computed, never typed.
- **§3.2 Bodies (`BodyParameters.cs`, `ArmModel.cs`):**
  - Masses (`0.0984, 0.1385, 0.1327, 0.1433` kg), CoM distances $\ell_i$ (`0.0474, 0.1022, 0.0909, 0.0451` m), angles $\varepsilon_i$ (`-90.4°, -84.2°, -0.1°, -11.4°` converted to radians via `Deg`), and inertia components $(I_{xx}, I_{yy}, I_{zz}, I_{xz})$ in `ArmModel.OpenManipulatorX()` (lines 84–89) match Table 5 row for row.
  - `BodyParameters.CenterOfMass` (line 86) returns $(\ell_i \cos\varepsilon_i,\, 0,\, -\ell_i \sin\varepsilon_i)$ and `BodyParameters.Inertia` (line 89) returns $\begin{bmatrix} I_{xx} & 0 & I_{xz} \\ 0 & I_{yy} & 0 \\ I_{xz} & 0 & I_{zz} \end{bmatrix}$, matching SPEC §4.1.
  - Validation checks $m > 0$, $\ell \ge 0$, finiteness, and positive definiteness via Sylvester's criterion (`Matrix3d.IsSymmetricPositiveDefinite`, lines 95–99).
- **§3.3 Actuators (`ActuatorParameters.cs`):**
  - Defaults: `StallTorque = 4.1`, `GearRatio = 353.5`, `EncoderCountsPerRevolution = 4096`, `NoLoadSpeed = 4.8`, `RotorInertia = 0.0`, `ViscousFriction = 0.01`, `CoulombFriction = 0.02`, `SmoothingSpeed = 0.1` (lines 9–16).
  - `ReflectedRotorInertia` computes $N^2 J_m$ (line 87); `EncoderResolution` computes $2\pi / 4096$ (line 90).
- **§3.4 Joint limits (`OpenManipulatorXGeometry.cs`, `JointDefinition.cs`):**
  - Limits $[-\pi, \pi]$, $[-1.5, 1.5]$, $[-1.5, 1.4]$, $[-1.7, 1.97]$ rad match the ROBOTIS URDF and SPEC §3.4.
  - Zero-based indexing (`Joints[0]` is joint 1) is consistent throughout.
- **§3.5 Environments (`Planet.cs`):**
  - `Earth` (9.81), `Moon` (1.62), `Mars` (3.71), `HighG` (24.79), `Orbit` (0.0), and `Custom(g)` with $0 \le g \le 30\text{ m/s}^2$.
  - `GravityVector` returns $(0, 0, -g)$ (line 58).

### 1.2 SPEC §4 — Kinematics (`src/RobotArm.Core/Kinematics`)

- **§4.1 Kinematic chain (`Frame.cs`, `ForwardKinematics.cs`):**
  - `Rotations.Rz(a)` (lines 57–62) and `Rotations.Ry(a)` (lines 65–70) match SPEC §4.1:
    $$R_z(a) = \begin{bmatrix} c & -s & 0 \\ s & c & 0 \\ 0 & 0 & 1 \end{bmatrix}, \qquad R_y(a) = \begin{bmatrix} c & 0 & s \\ 0 & 1 & 0 \\ -s & 0 & c \end{bmatrix}$$
  - `Frame.Then(offset, axis, angle)` (lines 34–35) computes `Rotation * Rotations.About(axis, angle)` and `Origin + Rotation * offset`, which is the exact homogeneous product $F_{i-1} \cdot T(\text{origin}_i) \cdot R(\text{axis}_i, q_i)$.
- **§4.2 Forward kinematics (`ForwardKinematics.cs`):**
  - `Planar` (lines 31–40) and `ClosedForm` (lines 47–52) implement eqs. (1)–(3) with $\theta_2 = q_2$, $\theta_3 = q_2 + q_3$, $\theta_4 = q_2 + q_3 + q_4$, $r = L_2\cos(\theta_2 - \beta) + L_3\cos\theta_3 + L_4\cos\theta_4$, $z = d_1 - L_2\sin(\theta_2 - \beta) - L_3\sin\theta_3 - L_4\sin\theta_4$, $p = [r\cos q_1, r\sin q_1, z]$, $\psi = \theta_4$.
- **§4.3 Inverse kinematics (`InverseKinematics.cs`):**
  - `Solve` (lines 135–160) implements eqs. (5)–(7) with $\varphi = -\arccos(\cos\varphi)$ for `ElbowBranch.Up` and $+\arccos(\cos\varphi)$ for `ElbowBranch.Down`, $e = \operatorname{atan2}(v, u) - \operatorname{atan2}(L_3\sin\varphi, L_2 + L_3\cos\varphi)$, $q_2 = \beta - e$, $q_3 = -\beta - \varphi$, $q_4 = \psi - q_2 - q_3$.
  - Non-finite inputs and $|\cos\varphi| > 1$ return `FailureReason.Unreachable` without throwing or returning `NaN`.
- **§4.4 Planar Jacobian (`PlanarJacobian.cs`):**
  - `Compute` (lines 26–43) and `Determinant` (lines 48–53) match eqs. (8)–(9): $\det J_p = -L_2 L_3 \sin(q_3 + \beta)$.
  - `TryInvert` (lines 68–85) checks $|\det J_p| \ge \text{threshold}$ (default $10^{-4}$) before computing the adjugate inverse.

### 1.3 SPEC §5 — Dynamics (`src/RobotArm.Core/Dynamics`, `tests/RobotArm.Tests/ClosedFormArm.cs`)

- **§5.1–§5.2 General algorithms (`RecursiveNewtonEuler.cs`, `Cholesky.cs`):**
  - `RecursiveNewtonEuler` implements Luh–Walker–Paul RNEA in body frames $F_i$, Walker–Orin column construction of $M(q)$, and $h(q, \dot{q}) = \operatorname{RNEA}(q, \dot{q}, 0)$.
  - `Cholesky.Factor` and `Cholesky.Solve` implement in-place $A = L L^T$ decomposition and forward/backward substitution without allocation, throwing `ArgumentException` naming the offending pivot if a pivot is $\le 0$ or non-finite.
- **§5.3 Closed forms and Christoffel indices (`ClosedFormArm.cs`):**
  - `InertiaMatrix` (lines 28–74) and `Gravity` (lines 77–88) implement eqs. (14)–(18) with zero-based body indices `_b[0..3]` mapped to bodies $1..4$.
  - `Coriolis` (lines 121–134) implements eq. (13):
    ```csharp
    s += 0.5 * (dM[i][k, j] + dM[j][k, i] - dM[k][i, j]) * qd[i];
    ```
    where `dM[p][r, c]` is $\partial M_{rc}/\partial q_p$ by central differences with step $10^{-6}$, matching $c_{ijk} = \frac{1}{2}(\partial M_{kj}/\partial q_i + \partial M_{ki}/\partial q_j - \partial M_{ij}/\partial q_k)$ and $C_{kj} = \sum_i c_{ijk}\dot{q}_i$.
- **§5.4 Actuators and friction (`ArmPlant.cs`):**
  - Lines 145–150 compute $\tau_i = \operatorname{clamp}(u_i, -\tau_{\max}, +\tau_{\max})$, $\text{saturated}_i = |u_i| > \tau_{\max}$, and $\tau_{f,i} = f_v \dot{q}_i + f_c \tanh(\dot{q}_i / \omega_s)$, then solve $M(q)\ddot{q} = \tau - \tau_f - h(q, \dot{q})$ via `Cholesky.FactorAndSolve`.
- **§5.5 Payload (`ArmModel.cs`, `Matrix3d.cs`):**
  - `ArmModel.WithPayload` (lines 101–122) combines body 4 with point mass $m_p$ at `ToolPoint` using the parallel-axis theorem `Matrix3d.PointMassInertia(mass, d)` $= m(|d|^2 I - d d^T)$ and recovers $\ell_4' = \sqrt{c_x'^2 + c_z'^2}$, $\varepsilon_4' = \operatorname{atan2}(-c_z', c_x')$.

### 1.4 SPEC §6 — Simulation (`src/RobotArm.Core/Simulation`)

- **§6.1–§6.2 State space and solvers (`ArmPlant.cs`, `Rk4Solver.cs`, `EulerSolver.cs`, `Simulation.cs`):**
  - State $x = [q_1..q_4, \dot{q}_1..\dot{q}_4]$ (length $2n = 8$), input $u$ (length $n = 4$) held constant across all four RK4 stages.
  - Step bounds $0.1\text{ ms} \le h \le 5\text{ ms}$ with default $h = 0.5\text{ ms}$ are enforced in `Simulation.Advance`.

---

## 2. Specific Review Questions

### 2.1 `RecursiveNewtonEuler`: Frame Conventions, Gravity Sign, and $N^2 J_m$ (D2)

1. **Frame conventions ($R_y$ with positive angle tips forward-down):**
   - In the base frame ($x$ forward, $y$ left, $z$ up), a standard right-hand rotation by $+q_i$ about $+y$ maps the local $x$ axis $(1, 0, 0)^T$ to $(\cos q_i, 0, -\sin q_i)^T$, which has a negative $z$ component—tipping the link forward and down.
   - Because $R_y(q_i) = \begin{bmatrix} \cos q_i & 0 & \sin q_i \\ 0 & 1 & 0 \\ -\sin q_i & 0 & \cos q_i \end{bmatrix}$ is the standard right-hand rotation matrix about $+y$, the angular velocity vector of frame $F_i$ relative to $F_{i-1}$ is $+\dot{q}_i \hat{y}$, and the time derivative of any body-fixed vector satisfies $\dot{r} = \omega \times r$ with the standard right-hand cross product (`Vector3d.Cross`).
   - In `RecursiveNewtonEuler.RigidBodyPass` (lines 199–205), the translation `p = _origin[i]` occurs in the parent frame $F_{i-1}$ before the joint rotation `_rotation[i] = R(axis_i, q_i)`, matching `Frame.Then` ($F_i = F_{i-1} T(\text{origin}_i) R(\text{axis}_i, q_i)$). `TransposeTimes(r, ...)` applies $R_i^T$ to map vectors from $F_{i-1}$ into $F_i$, and the backward pass (lines 231–233) applies `r * f` and `r * nMoment` ($R_i$) to map forces and moments from $F_i$ back into $F_{i-1}$.
2. **Gravity sign:**
   - `Planet.GravityVector` is $\mathbf{g} = (0, 0, -g)$ in the base frame.
   - `Inverse` (line 107) and `BiasTorque` (line 127) pass `baseAcceleration = -gravity = (0, 0, +g)` into `RigidBodyPass`. Accelerating the base upward by $+g\hat{z}$ produces the same internal reaction torques as supporting the arm at rest against a downward gravitational field $-g\hat{z}$.
   - Why $g_k(q) < 0$ in stretched horizontal poses (SPEC §5.3 eq. 18): since positive pitch $q_k > 0$ moves the links downward (decreasing $z_i$), potential energy $V(q) = \sum m_i g z_i$ decreases as $q_k$ increases, so $g_k(q) = \partial V / \partial q_k = -g(\dots)\cos(\dots) < 0$. In RNEA, for a horizontal link with CoM at $(\ell, 0, 0)$ and upward base acceleration $(0, 0, +g)$, the moment is $(\ell, 0, 0) \times (0, 0, m g) = (0, -m g \ell, 0)$, whose projection onto $\hat{y} = (0, 1, 0)$ is $-m g \ell < 0$. The signs in RNEA and the closed-form eq. (18) agree completely.
3. **How $N^2 J_m$ enters both $M$ and `Inverse` (D2):**
   - `RigidBodyPass` knows nothing about motor rotors.
   - `InertiaMatrix` (line 171) adds `_reflectedRotorInertia` ($N^2 J_m$) to each diagonal entry `m[k * _n + k]`.
   - `Inverse` (lines 108–109) adds `_reflectedRotorInertia * qdd[i]` ($N^2 J_m \ddot{q}_i$) to `tau[i]`.
   - `BiasTorque` sets $\ddot{q} = 0$, so rotor inertia does not enter $h(q, \dot{q})$.
   - This makes the identity $\operatorname{Inverse}(q, \dot{q}, \ddot{q}, \mathbf{g}) = M(q)\ddot{q} + h(q, \dot{q}, \mathbf{g})$ hold by construction for all $J_m \ge 0$.

### 2.2 `StabilityEstimate` and D3: Is $2.78 / \lambda_{\max}(M^{-1} D)$ the Right Criterion for RK4, and What Is Missing?

1. **Why $2.78 / \lambda_{\max}(M(q)^{-1} D)$ is the right local criterion for smoothed Coulomb friction:**
   - Near rest ($\dot{q} \approx 0$), the smoothed Coulomb friction $f_c \tanh(\dot{q}_i / \omega_s)$ has its maximum slope $f_c / \omega_s$ (since $\operatorname{sech}^2(\dot{q}_i / \omega_s) \le 1$ is maximized at $\dot{q}_i = 0$).
   - When the fast dissipation time scale $D = \operatorname{diag}(f_v + f_c/\omega_s)$ dominates stiffness, the velocity perturbation obeys $\delta\ddot{q} \approx -M(q)^{-1} D \delta\dot{q}$.
   - Because $M(q) \succ 0$ and $D \succeq 0$, $M(q)^{-1} D$ is similar to $D^{1/2} M(q)^{-1} D^{1/2} \succeq 0$, so all its eigenvalues are real and non-negative, placing the poles of the linearized velocity dynamics on the negative real axis at $s_i = -\lambda_i(M(q)^{-1} D)$.
   - For explicit RK4, the stability polynomial $R(z) = 1 + z + \frac{z^2}{2} + \frac{z^3}{6} + \frac{z^4}{24}$ satisfies $|R(z)| \le 1$ for real $z \in [-2.78529356\dots, 0]$, so $h \cdot \lambda_{\max}(M(q)^{-1} D) < 2.78$ is the exact linear stability boundary for real negative poles.
2. **What is missing from `StabilityEstimate.MaxStableStep`:**
   - **Configuration dependence of $M(q)$ along a trajectory:** `MaxStableStep(model, q, extraDamping)` evaluates $M(q)$ at a **single pose $q$**. However, $M(q)$ varies strongly with $q$: in particular, $m_{11}(q) = \sum_i (m_i \rho_i^2 + \dots)$ shrinks dramatically when the arm folds toward the base $z$-axis ($\rho_i \to 0$). A step size $h$ that is stable at an extended initial pose $q(0)$ can violate $h \lambda_{\max}(M(q(t))^{-1} D) < 2.78$ mid-trajectory when the arm passes near a low-inertia configuration. If the UI only checks `MaxStableStep` at the initial pose, it can miss mid-run instability.
   - **Full $2n \times 2n$ state-space Jacobian (gravity gradient $\partial g / \partial q$ and proportional gain $K_p$):** Linearizing $\dot{x} = f(x, u)$ around an equilibrium gives the $2n \times 2n$ block matrix
     $$A = \begin{bmatrix} 0 & I \\ -M(q)^{-1} K & -M(q)^{-1} D \end{bmatrix}, \qquad K = \frac{\partial g(q)}{\partial q} + K_p - \frac{\partial \hat{g}(q)}{\partial q}.$$
     `StabilityEstimate` assumes $K = 0$.
     - When $D = 0$ (e.g., frictionless arm with `extraDamping = 0`), `MaxStableStep` returns `double.PositiveInfinity`, even though undamped gravity/stiffness modes ($s = \pm \sqrt{-\lambda(M^{-1}K)}$) require a finite step for accuracy and stability (on the imaginary axis RK4 requires $h \omega_n < 2\sqrt{2} \approx 2.828$).
     - When $K$ has negative eigenvalues (an uncompensated inverted/upright pose where $\partial g/\partial q \prec 0$), a mode with damping rate $d/m$ and negative stiffness $k/m < 0$ has its negative real eigenvalue pushed further left to $-\frac{1}{2}(d/m + \sqrt{(d/m)^2 + 4|k|/m}) < -d/m$. With default friction ($d/m \sim 200\text{–}2000\text{ s}^{-1}$ vs $|k/m| \sim g/\ell \sim 100\text{–}250\text{ s}^{-2}$) this shift is small ($\lesssim 1\%$), but for very small friction or high $g$ it reduces the stable step.
     - When a controller uses a large proportional gain $K_p$ with small $K_d$, the eigenvalues become complex conjugate ($s = -\sigma \pm i\omega_d$), where the RK4 stability boundary curves inward from $2.785$ toward $2\sqrt{2}$ and is governed by $\sqrt{\lambda(M^{-1}K_p)}$ rather than $\lambda(M^{-1}D)$ alone.
   - **Sampled-data zero-order hold (ZOH) of controller derivative gains (`extraDamping`):** D3 notes that T07 should pass the controller's derivative gains $K_d$ as `extraDamping`. However, in the sampled control loop (SPEC §6.3), the controller torque $u$ is updated only every $T_s$ (default $2\text{ ms}$) and is **held constant** across the RK4 stages (`Rk4Solver.Step` passes the same `u` to all four stages). Thus $K_d \dot{e}$ does not act as continuous-time damping inside the RK4 stages; instead, it is a discrete-time ZOH feedback with sample period $T_s$, whose stability limit depends on $T_s \cdot \lambda_{\max}(M^{-1} K_d)$ (roughly $< 2$ for discrete derivative feedback) rather than $h \cdot \lambda_{\max} < 2.78$.
   - **Coriolis/centrifugal coupling at non-zero speed ($\dot{q} \neq 0$):** At high joint speeds, $C(q, \dot{q})\dot{q}$ adds gyroscopic/centrifugal terms of order $\|\dot{q}\|$ to the Jacobian, whereas `StabilityEstimate` is strictly a rest ($\dot{q} = 0$) estimate.
   - **Non-finite `q` bug in `StabilityEstimate`:** See Finding #1 in the table below (`MaxStableStep` returns `+Infinity` or throws from `Cholesky` if `q` contains `NaN`).

### 2.3 NaN Handling (D4) and `FaultChecks` Logic in `Simulation`: Can Any Path Produce an Exception Instead of a Reported Fault?

1. **Normal finite run where a torque command or state becomes `NaN`:**
   - When `u` contains `NaN`, `Math.Clamp(double.NaN, -4.1, 4.1)` yields `NaN`, `Cholesky.Solve` propagates `NaN` into $\ddot{q}$ (`dx[4..7]`), stage 2 of RK4 puts `NaN` into $\dot{q}$ (`_stage[4..7]`), and stages 3–4 put `NaN` into $q$ (`_stage[0..3]`), which hits the `!double.IsFinite(qi)` guard in `ArmPlant.Derivatives` (line 131). After the step, `Simulation.DetectFault()` reports `FaultReason.NonFinite` without throwing.
2. **Paths that CAN produce an exception or wrong fault instead of `FaultReason.NonFinite`:**
   - **Misclassification as `FaultReason.JointLimit` when `FaultChecks.NonFinite` is disabled:** In `Simulation.DetectFault()` (lines 153–158), if `Checks` includes `FaultChecks.JointLimit` but not `FaultChecks.NonFinite`, `_state[i]` being `double.NaN` causes `model.Joints[i].IsWithinLimits(double.NaN)` (`angle >= LowerLimit && angle <= UpperLimit`) to evaluate to `false`. `DetectFault()` then returns `FaultReason.JointLimit` on joint `0` with `Value = NaN` instead of ignoring non-finite values in the joint-limit check.
   - **`Cholesky.Factor` throwing `ArgumentException` on overflow of `ReflectedRotorInertia`:** `ActuatorParameters` checks that `GearRatio` and `RotorInertia` are individually finite, but does not check that `ReflectedRotorInertia = _gearRatio * _gearRatio * _rotorInertia` is finite. If `ReflectedRotorInertia` overflows to `+Infinity`, `RecursiveNewtonEuler.InertiaMatrix` puts `+Infinity` on the diagonal of $M(q)$ for finite $q$, and `Cholesky.Factor` (line 31) throws `ArgumentException` ("The matrix is not positive definite: pivot 0 is Infinity") during `Simulation.Advance`.
   - **Unchecked `qd` in `ArmPlant.Derivatives`:** `ArmPlant.Derivatives` (lines 129–136) only checks `q` for finiteness, not `qd`. When `q` is finite and `qd` is `NaN` or `±Infinity` (which happens in stage 2 of RK4 whenever `u` is `NaN`, or if `initialState` has non-finite `qd`, or if `qd` overflows before `q` does), `Derivatives` still executes `BiasTorque`, `InertiaMatrix`, and `Cholesky.FactorAndSolve`. It avoids throwing only because `InertiaMatrix` depends solely on `q` and `Cholesky.Solve` does not check the right-hand side for finiteness. Checking all of `x` (both `q` and `qd`) would avoid running RNEA and Cholesky on `NaN` velocities.
   - **Stale `AppliedTorque` and `Saturated` on non-finite `q`:** Because `ArmPlant.Derivatives` returns at line 134 before lines 145–147 when `q` is non-finite, `_appliedTorque` and `_saturated` are not updated if a step starts with non-finite `q`.

### 2.4 Payload Attach/Detach: State Continuity and Controller Model Separation

1. **Is the state really unchanged?**
   - Yes. `Simulation.AttachPayload(massKg)` and `Simulation.DetachPayload()` call `_plant.SetPayload(...)`, which executes:
     ```csharp
     var model = BaseModel.WithPayload(massKg);
     _dynamics = new RecursiveNewtonEuler(model);
     PayloadKg = massKg;
     ```
   - `Simulation._state` ($[q; \dot{q}]$), `Simulation.Time`, and `Simulation.StepCount` are untouched. Because `SetPayload` always calls `BaseModel.WithPayload(massKg)` on the unloaded `BaseModel`, attaching a new payload replaces any previous payload cleanly rather than compounding masses.
2. **Is the controller model separate from the true plant?**
   - Yes. `ArmModel`, `BodyParameters`, `JointDefinition`, `ActuatorParameters`, and `Planet` are deeply immutable (`sealed` classes/records with private cloned arrays exposed via `Array.AsReadOnly`).
   - Calling `plant.SetPayload(massKg)` constructs a new `ArmModel` and a new `RecursiveNewtonEuler` inside `ArmPlant` without mutating `BaseModel` or any `ArmModel` reference held by a controller.

### 2.5 Test Suite Audit: V5, V7, V9 Analysis and Tests That Would Pass Even If the Code Were Wrong (V8 Weakness & Others)

#### What V5, V7, and V9 Catch and Miss

| Verification item | What bugs it catches | What bugs it misses |
|---|---|---|
| **V5** (`InertiaMatrixTests.cs`: general $M(q)$ vs closed form and reference vectors) | Wrong frame rotations $R_z, R_y$; wrong joint origins; wrong CoM positions $(\ell_i, \varepsilon_i)$; wrong body masses $m_i$ or planar inertia components $(I_{xx}, I_{yy}, I_{zz}, I_{xz})$; sign errors in acceleration kinematics for $\ddot{q} = e_k$ or moment lever arms in backward recursion. | 1. Any error in **velocity-dependent (Coriolis/centrifugal) terms** of RNEA ($\dot{q} = 0$ in `InertiaMatrix`).<br>2. Any error in **gravity handling** ($\mathbf{g} = \mathbf{0}$ in `InertiaMatrix`).<br>3. Any error in **reflected rotor inertia $N^2 J_m$** (default $J_m = 0$; tested separately).<br>4. Any error in **upper-triangle entries `_column[0..k-1]`** computed by RNEA, because `InertiaMatrix` overwrites the upper triangle by mirroring the lower triangle `_column[k..n-1]`.<br>5. Any 3D out-of-plane error ($c_y \neq 0, I_{xy} \neq 0, I_{yz} \neq 0$), since the 4-DOF data is planar-symmetric. |
| **V7** (`GravityTests.cs`: bias torque at $\dot{q} = 0$ vs closed-form $g(q)$ and `gravity_earth`) | Wrong gravity sign (`-gravity` vs `+gravity`); wrong masses $m_2, m_3, m_4$; wrong link lengths $L_2, \beta, L_3$; wrong CoM $(\ell_i, \varepsilon_i)$ for $i = 2, 3, 4$; wrong pitch rotation $R_y$ sign; wrong static moment cross products in RNEA backward recursion. | 1. Any error in **inertia tensors $I_i$**, angular velocities $\omega_i$, angular accelerations $\dot{\omega}_i$, or rotor inertia $N^2 J_m$ (since $\dot{q} = \ddot{q} = 0 \implies \omega_i = \dot{\omega}_i = 0$).<br>2. Any error in **body 1's mass $m_1$, CoM $(\ell_1, \varepsilon_1)$, or base height $d_1$**, because gravity is parallel to joint 1's $z$-axis ($g_1(q) \equiv 0$) and $d_1$ is a constant vertical offset that drops out of $\partial V/\partial q$.<br>3. Any error in **tool length $L_4$** in the unloaded test (only enters when a payload is attached). |
| **V9** (`FreeMotionTests.cs`: max energy drift $< 10^{-5}\text{ J}$ over $0.4\text{ s}$ at $h = 2\text{ ms}$) | Non-conservative errors in forward dynamics or RK4 integration: errors in $C(q, \dot{q})\dot{q}$ that violate skew-symmetry of $\dot{M} - 2C$, inconsistency between $g(q)$ in RNEA and $\nabla_q V(q)$, or corrupted RK4 stage weights. | 1. **A no-op solver/step**: if `Rk4Solver.Step` or `Simulation.Advance` left the state unchanged at $x_0$, energy drift would be identically $0 < 10^{-5}\text{ J}$ and V9 would pass.<br>2. **Any wrong parameter in `ArmModel` or wrong $g$**: `SimulationTestHelpers.Energy` computes $E = \frac{1}{2}\dot{q}^T M(q)\dot{q} + V(q)$ using the *same* `ArmModel`, `RecursiveNewtonEuler.InertiaMatrix`, and `ForwardKinematics.ComputeFrames` as `ArmPlant`—any conservative Lagrangian system conserves its own Hamiltonian.<br>3. **Any skew-symmetric error in $C(q, \dot{q})$** (a fictitious gyroscopic term $S(q, \dot{q})\dot{q}$ with $S^T = -S$ does zero work: $\dot{q}^T S \dot{q} = 0$, so energy is still conserved).<br>4. Any error in **friction, actuator saturation, rotor inertia $J_m$, or fault checks** (all zero/off in V9). |

#### Other Tests in the Suite That Would Pass Even If the Code They Test Were Wrong

1. **`CoriolisTests.MdotMinusTwoC_IsSkewSymmetric_In200RandomStates` (`CoriolisTests.cs:23–55`, V8):**
   - Beyond the SPEC §11 note that V8 never calls RNEA with $\dot{q} \neq 0$, V8 has a second mathematical blind spot: for $C_{kj} = \frac{1}{2}\sum_i (\partial_i M_{kj} + \partial_j M_{ki} - \partial_k M_{ij})\dot{q}_i$, the first term $\frac{1}{2}\sum_i \partial_i M_{kj}\dot{q}_i = \frac{1}{2}\dot{M}_{kj}$ cancels $\dot{M}_{kj}$, while the remaining two terms $\frac{1}{2}\sum_i (\partial_j M_{ki} - \partial_k M_{ij})\dot{q}_i$ are **anti-symmetric in $(k, j)$ for any coefficient**. If `ClosedFormArm.Coriolis` dropped both $\partial_j M_{ki} - \partial_k M_{ij}$ terms (or multiplied them by 99), $\dot{M} - 2C$ would *still* be skew-symmetric ($z^T(\dot{M} - 2C)z = 0$) and V8 would pass!
2. **`CoriolisTests.BiasTorque_WithoutGravity_EqualsClosedFormCoriolisTimesSpeed` (`CoriolisTests.cs:62–87`):**
   - Because this test only checks the product $C(q, \dot{q})\dot{q}$ (where $\dot{q}_i \dot{q}_j$ is symmetric in $i, j$), $\sum_{i,j} \partial_i M_{kj} \dot{q}_i \dot{q}_j = \sum_{i,j} \partial_j M_{ki} \dot{q}_i \dot{q}_j$. If `ClosedFormArm.Coriolis` had an index bug that replaced `0.5 * (dM[i][k, j] + dM[j][k, i] - dM[k][i, j])` with `dM[i][k, j] - 0.5 * dM[k][i, j]`, **both** tests in `CoriolisTests.cs` (`MdotMinusTwoC_IsSkewSymmetric` fails only if the $(k,j)$-skew part is broken, or if `dM[j][k, i] - dM[k][i, j]` is scaled, wait: if `ClosedFormArm.Coriolis` added any $(k,j)$-skew-symmetric matrix $S(q, \dot{q})$ to $C$, $z^T(\dot{M}-2C)z = 0$ AND $\dot{q}^T S \dot{q} = 0$), or if $C(q, \dot{q})$ is later used in a Slotine–Li controller where $C(q, \dot{q})\dot{q}_r$ requires the full matrix $C(q, \dot{q})$ on $\dot{q}_r \neq \dot{q}$, neither test checks $C(q, \dot{q})w$ for $w \neq \dot{q}$.
3. **`FreeMotionTests.V9_FreeMotion_EnergyDrift_Below1e5J_In04sAt2ms` and `V10_FreeMotion_YawMomentumDrift_Below1e7_In04sAt2ms` (`FreeMotionTests.cs:34–73`):**
   - Neither V9 nor V10 asserts that the state actually changed from `x0`! If `Simulation.Advance` or `Rk4Solver.Step` were a no-op, energy drift and $p_1$ drift would both be `0.0`, passing V9 and V10.
4. **`SimulationTests.V13_GravityTorque_HoldsArmStillFor1s_WithFriction` (`SimulationTests.cs:46–81`):**
   - Would pass even if **friction in `ArmPlant.Derivatives` were completely broken or deleted** (`friction = 0` or wrong sign for viscous vs Coulomb): because the test starts at exact rest $\dot{q} = 0$ and commands $\tau = g_{\text{closed}}(q)$, the net acceleration $\ddot{q} = M^{-1}(g_{\text{closed}}(q) - g_{\text{RNEA}}(q))$ is at machine epsilon ($\sim 10^{-14}\text{ rad/s}^2$), so $|\dot{q}|$ over $1\text{ s}$ stays $\sim 10^{-14} \ll 10^{-9}\text{ rad/s}$ even with zero friction. Conversely, if `g(q)` had a small error up to $|\Delta g| < f_c = 0.02\text{ N·m}$, Coulomb friction would hold $|\dot{q}| \approx \omega_s \operatorname{artanh}(|\Delta g|/f_c)$, which for errors $< 10^{-10}\text{ N·m}$ also stays below $10^{-9}\text{ rad/s}$. And if `Advance` were a no-op, $|\dot{q}| = 0 < 10^{-9}$ would also pass.
5. **`InertiaMatrixTests.InertiaMatrix_IsSymmetricPositiveDefinite_In1000RandomPoses` (`InertiaMatrixTests.cs:70–95`, V6):**
   - Lines 81–83 assert `Assert.Equal(m[r * 4 + c], m[c * 4 + r])` on the output of `RecursiveNewtonEuler.InertiaMatrix`. However, `InertiaMatrix` (lines 166–170) explicitly copies `_column[i]` into both `m[i * _n + k]` and `m[k * _n + i]`, discarding `_column[0..k-1]`. Thus the symmetry assertion in V6 is a tautology that would pass even if the raw columns returned by `RigidBodyPass` were asymmetric.
6. **`ParametersTests.JointLimits_AreDataOnly_PosesOutsideThemAreReportedNotRejected` (`ParametersTests.cs:154–164`):**
   - Despite its name and comment ("The model only reports this; nothing here throws"), this test only calls `s_arm.Joints[i].IsWithinLimits(q[i])` on arrays from `TestVectors`—it never passes an out-of-limits pose to `ArmModel`, `ForwardKinematics`, `PlanarJacobian`, `RecursiveNewtonEuler`, or `ArmPlant` to verify they do not reject it.
7. **`ChainTests.Chain_IsDataNotCode_SixJointChainCanBeBuilt` (`ChainTests.cs:74–81`):**
   - Only constructs a 6-DOF `ArmModel` and asserts `Assert.Equal(6, model.DegreesOfFreedom)`. It never calls `ForwardKinematics.ComputeFrames`, `RecursiveNewtonEuler`, `ArmPlant`, `Rk4Solver`, `Simulation`, or `StabilityEstimate` on the 6-DOF model, so any hardcoded `4` or `8` in kinematics, dynamics, or simulation would still pass this test.
8. **`DependencyRuleTests.CoreAssembly_ReferencesNoOtherRobotArmAssembly` (`DependencyRuleTests.cs:9–21`):**
   - Only checks for referenced assemblies whose name starts with `"RobotArm."`. If `RobotArm.Core` referenced a forbidden third-party NuGet package or UI/IO assembly, this test would still pass.

### 2.6 Would the General Algorithms Work Unchanged for a 6-DOF Chain? What Would Break?

- **What works unchanged:** `ForwardKinematics.ComputeFrames`, `RecursiveNewtonEuler` (`Inverse`, `BiasTorque`, `InertiaMatrix`), `Cholesky`, `ArmPlant`, `Rk4Solver`, `EulerSolver`, `Simulation`, and `StabilityEstimate` all size their buffers from `model.DegreesOfFreedom` ($n$) and `plant.StateCount` ($2n$) without hardcoding 4 or 8.
- **What would break for a general 6-DOF arm:**
  1. **No `JointAxis.X` (roll axis) or fixed joint-frame rotation (`JointDefinition.cs:4–11`, `Frame.cs:74–79`):** `JointAxis` only supports `Z` and `Y`. Most 6-DOF industrial arms (e.g., spherical-wrist arms with forearm roll on joint 4 and tool roll on joint 6) rotate about the link's local $x$-axis (`Rx`), or require a fixed parent-to-joint rotation $R_{\text{fixed}}$ in addition to the translation `Origin`.
  2. **Planar body restriction ($c_y = 0, I_{xy} = I_{yz} = 0$) in `BodyParameters` (`BodyParameters.cs:37–90`):** `BodyParameters` parameterizes the CoM only by $(\ell, \varepsilon)$ in the body's $x\text{–}z$ plane (`CenterOfMass.Y` is hardcoded to `0.0`) and only accepts $(I_{xx}, I_{yy}, I_{zz}, I_{xz})$ (`Ixy = Iyz = 0`). A 6-DOF arm with offset links or asymmetric wrist bodies having $c_y \neq 0$ or $I_{xy}, I_{yz} \neq 0$ cannot be represented, even though `RecursiveNewtonEuler` itself uses full 3D `Vector3d` and `Matrix3d` arithmetic.
  3. **Planar tool-point restriction in `ArmModel.WithPayload` (`ArmModel.cs:107–121`):** `WithPayload` throws `InvalidOperationException` if `ToolPoint.Y != 0.0` and calls `BodyParameters.FromCenterOfMassAndInertia`, which throws `ArgumentOutOfRangeException` if the combined body has $c_y \neq 0$, $I_{xy} \neq 0$, or $I_{yz} \neq 0$.
  4. **Single shared `ActuatorParameters` across all joints (`ArmModel.cs:27`, `RecursiveNewtonEuler.cs:71`, `ArmPlant.cs:47–51`):** `ArmModel.Actuator` is a single record shared by every joint. A 6-DOF arm typically has heavy shoulder/elbow actuators (joints 1–3) and lighter wrist actuators (joints 4–6) with different $\tau_{\max}, N, J_m, f_v, f_c$.

---

## 3. PROJECT.md Rules Check

| Rule | Status | Notes |
|---|---|---|
| Dependency rule (Core references nothing; no screen, file path, timer, browser, clock) | Pass | `RobotArm.Core.csproj` has no project or package references; no I/O or clock APIs used. |
| `IPlant` and `ISolver` signatures (`PROJECT.md` §2 rule 2, D1) | Pass | Exact match in `IPlant.cs` and `ISolver.cs`. |
| One source of truth for numbers (`PROJECT.md` §2 rule 5) | Pass | All constants live in `RobotArm.Core/Model` and are verified against `reference/test_vectors.json`. |
| C# 14, `net10.0`, `Nullable=enable`, `TreatWarningsAsErrors=true` in Core | Pass | Configured in `RobotArm.Core.csproj`; builds with 0 warnings. |
| Classes `sealed` unless there is a reason | Pass | All non-static classes and records in Core and Tests are `sealed`. |
| Defensive array cloning (`Clone()` / `ToArray()`) | Pass | `ArmModel`, `Simulation`, and `ClosedFormArm` copy all input sequences/spans. |
| Zero allocation in per-step methods (`Derivatives`, `Step`, `Advance`, RNEA, Cholesky) | Pass | Verified by inspection and by `DynamicsAllocationTests` and `SimulationAllocationTests` (0 bytes over 10,000 calls). |
| `CultureInfo.InvariantCulture` for every number written or parsed | **Minor issue** | `BodyParameters.cs:50`, `JointDefinition.cs:51`, and `Planet.cs:53` format `double` values in exception messages using default string interpolation instead of `string.Create(CultureInfo.InvariantCulture, ...)`. |
| XML doc comments with SPEC section on every public type and member | Pass | Enforced by `<GenerateDocumentationFile>true</GenerateDocumentationFile>` with `TreatWarningsAsErrors`. |

---

## 4. Table of Findings

| File | Line | Severity | Explanation | Suggested fix |
|---|---|---|---|---|
| `src/RobotArm.Core/Simulation/StabilityEstimate.cs` | 41–45, 60–95, 184–186 | `bug` | `MaxDampingRate` does not validate that `q` entries are finite. If `q` contains `NaN` or `±Infinity`: (a) when all joints are damped, `SmallestEigenvalue` returns `NaN` (after 100 Jacobi sweeps), and `MaxStableStep` evaluates `lambda > 0.0` (`false` for `NaN`) and returns `double.PositiveInfinity`, falsely reporting that any step size is stable; (b) when any joint is undamped (`u > 0`), `ScaledSchurComplement` calls `Cholesky.Factor` on a `NaN` submatrix and throws `ArgumentException` with parameter name `"a"`. | Validate that every element of `q` is finite in `MaxDampingRate` (throwing `ArgumentOutOfRangeException(nameof(q), ...)`), and in `MaxStableStep` check `if (lambda == 0.0) return double.PositiveInfinity; return Rk4StabilityLimit / lambda;` so `NaN` is never turned into `+Infinity`. |
| `src/RobotArm.Core/Simulation/Simulation.cs` | 145–158 | `bug` | In `DetectFault()`, if `FaultChecks.JointLimit` is enabled without `FaultChecks.NonFinite`, a `NaN` joint angle `_state[i]` causes `JointDefinition.IsWithinLimits(double.NaN)` (`angle >= LowerLimit && angle <= UpperLimit`, `JointDefinition.cs:66`) to return `false`, reporting a spurious `FaultReason.JointLimit` on joint 0 with `Value = NaN`. | Guard the joint-limit check in `DetectFault()` with `if (double.IsFinite(_state[i]) && !model.Joints[i].IsWithinLimits(_state[i]))` so `JointLimit` only triggers on finite angles outside the limits. |
| `src/RobotArm.Core/Simulation/ArmPlant.cs` | 129–151 | `risk` | When `q` contains a non-finite angle, `Derivatives` fills `dx` with `NaN` and returns at line 134 *before* updating `_appliedTorque` and `_saturated` from `u` (lines 145–147) and without checking `qd` for finiteness. If a step begins with non-finite `q` (e.g., under `FaultChecks.None`), `AppliedTorque` and `Saturated` retain stale values from an earlier step; if `q` is finite and `qd` is `NaN`/`±Infinity` (e.g., RK4 stage 2 after a `NaN` input), RNEA and Cholesky still run unnecessarily. | Update `_appliedTorque` and `_saturated` from `u` first, then check all of `x` (both `q` and `qd`) for finiteness and fill `dx` with `NaN` on early return. |
| `src/RobotArm.Core/Model/ActuatorParameters.cs` | 25–30, 55–60, 87 | `risk` | `GearRatio` and `RotorInertia` are validated individually as finite, but `ReflectedRotorInertia` (`_gearRatio * _gearRatio * _rotorInertia`) can overflow to `+Infinity` for large finite inputs, which causes `Cholesky.Factor` to throw `ArgumentException` during `Simulation.Advance` instead of reporting `FaultReason.NonFinite`. | Validate `double.IsFinite(ReflectedRotorInertia)` in the `GearRatio` and `RotorInertia` `init` setters. |
| `src/RobotArm.Core/Kinematics/PlanarJacobian.cs` | 58–85 | `risk` | `IsNearSingular` and `TryInvert` do not validate `threshold`. If a caller passes `threshold <= 0.0` (such as `0.0`), `Math.Abs(det) >= threshold` is `true` even when `det == 0.0`, causing `TryInvert` to divide by zero and return `true` with `NaN`/`±Infinity` in `inverse`. | Validate `double.IsFinite(threshold) && threshold > 0.0` (throwing `ArgumentOutOfRangeException(nameof(threshold), ...)`). |
| `src/RobotArm.Core/Kinematics/InverseKinematics.cs` | 33–70, 144 | `risk` | (1) `InverseKinematicsResult` is a `readonly struct` with `Success => Failure == FailureReason.None` where `FailureReason.None = 0`, so `default(InverseKinematicsResult)` reports `Success == true` and `LimitJointIndex == 0`. (2) At line 144, `!(Math.Abs(cosPhi) <= 1.0)` rejects exact workspace boundary poses (e.g., stretched elbow $q_3 = -\beta$, which lies inside joint 3's limits $[-1.5, 1.4]$) whenever floating-point roundoff in `FK` $\to$ `IK` makes $|\cos\varphi| = 1 + 10^{-16}$. | Store a `private readonly bool _succeeded;` field in `InverseKinematicsResult` (so `default` has `Success == false`), and consider clamping `cosPhi` within $[-(1 + 10^{-12}), 1 + 10^{-12}]$ to $[-1, 1]$ before `Math.Acos`. |
| `src/RobotArm.Core/Model/JointDefinition.cs`, `BodyParameters.cs`, `ArmModel.cs` | `JointDefinition.cs:4–11`, `BodyParameters.cs:69–90`, `ArmModel.cs:27, 107–121` | `risk` | General 6-DOF chains are limited by: (1) no `JointAxis.X` roll axis in `JointAxis`/`Rotations.About`, (2) `BodyParameters` restricting CoM to $c_y = 0$ and inertia to $I_{xy} = I_{yz} = 0$, (3) `ArmModel.WithPayload` requiring `ToolPoint.Y == 0`, and (4) a single shared `ActuatorParameters` for all joints. | Add `JointAxis.X` / `Rotations.Rx` and general 3D CoM/inertia support if 6-DOF arms are added in T14, or document these constraints in `ArmModel`. |
| `src/RobotArm.Core/Model/BodyParameters.cs`, `JointDefinition.cs`, `Planet.cs` | `BodyParameters.cs:50`, `JointDefinition.cs:51`, `Planet.cs:53` | `style` | Exception messages interpolate `double` values (`ixx, ixz, iyy, izz`, `upperLimit`, `MaxCustomG`) without `CultureInfo.InvariantCulture`, violating `PROJECT.md` §3. On a Turkish (`tr-TR`) locale, decimal commas make `[[0,1, 0, -0,4], ...]` ambiguous. | Use `string.Create(CultureInfo.InvariantCulture, $"...")` as in `Cholesky.cs:33` and `Simulation.cs:118`. |
| `tests/RobotArm.Tests/CoriolisTests.cs`, `FreeMotionTests.cs`, `SimulationTests.cs`, `InertiaMatrixTests.cs`, `ChainTests.cs` | `CoriolisTests.cs:23–87`, `FreeMotionTests.cs:34–73`, `SimulationTests.cs:46–81`, `InertiaMatrixTests.cs:81–83`, `ChainTests.cs:74–81` | `risk` | Test blind spots detailed in §2.5: V8 and $C\dot{q}$ do not test $C(q, \dot{q})w$ for $w \neq \dot{q}$; V9/V10 do not assert that state moved from $x_0$; V13 passes even if plant friction is zero; V6 tests symmetry after mirroring; `ChainTests` never runs kinematics/dynamics/simulation on the 6-DOF model. | Add assertions that `sim.State` moves in V9/V10, test raw RNEA column symmetry $e_i^T \operatorname{RNEA}(q,0,e_k) = e_k^T \operatorname{RNEA}(q,0,e_i)$, and run `ComputeFrames`, `RecursiveNewtonEuler`, and `Simulation` on the 6-DOF model in `ChainTests`. |

---

## 5. Three Things We Are Least Sure Are Correct (TASKS.md T06)

1. **Whether `StabilityEstimate.MaxStableStep` with `extraDamping = Kd` will accurately predict stability in T07's sampled control loop (`SPEC §6.2–§6.3`, D3):**
   Because the controller runs at sample period $T_s$ (default $2\text{ ms}$) and holds $u$ constant across the RK4 step $h$ (default $0.5\text{ ms}$), the derivative term $K_d \dot{e}$ is a discrete-time zero-order-held feedback at rate $1/T_s$, not continuous-time physical damping evaluated inside the four RK4 stages. Furthermore, `MaxStableStep` evaluates $M(q)$ at only one pose $q$ and omits $K_p$ and $\partial g/\partial q$.
2. **Exact workspace boundary behavior of `InverseKinematics.Solve` at $q_3 = -\beta$ (`InverseKinematics.cs:144`):**
   The stretched-elbow singularity $q_3 = -\beta \approx -1.3854\text{ rad}$ lies strictly inside joint 3's URDF limits $[-1.5, 1.4]\text{ rad}$. For poses on this boundary, `FK(q)` followed by `IK` can produce $\cos\varphi = 1 + O(10^{-16})$ from IEEE 754 rounding, which `!(Math.Abs(cosPhi) <= 1.0)` rejects as `Unreachable`. In T08, a Cartesian straight-line trajectory touching or starting on the stretched boundary could fail unexpectedly.
3. **Whether `BodyParameters` and `JointAxis` are intended to support general 6-DOF arms without code changes (`SPEC §4.1`, `§5.2`):**
   While `RecursiveNewtonEuler` and `ForwardKinematics.ComputeFrames` operate on general 3D vectors and matrices for any $n$, `JointAxis` lacks `X` (roll) and `BodyParameters` rejects $c_y \neq 0$ and $I_{xy}, I_{yz} \neq 0$, which most 6-DOF arms with a 3-axis wrist require.

---

## 6. Three Most Important Findings

1. **`StabilityEstimate` non-finite `q` bug and sampled-data / configuration-dependence limitations (`StabilityEstimate.cs:41–95`):**
   Passing a `NaN` angle to `StabilityEstimate.MaxStableStep` returns `double.PositiveInfinity` (when all joints are damped) or throws an internal `Cholesky` `ArgumentException` (when any joint is undamped). Beyond input validation, `MaxStableStep` evaluates $M(q)$ at a single pose $q$ (missing lower-inertia poses along a trajectory, especially as $m_{11}(q)$ drops near the base axis) and models controller derivative gains `extraDamping` as continuous-time RK4 damping rather than zero-order-held sampled feedback at period $T_s$.
2. **Fault-detection and non-finite state edge cases in `Simulation` and `ArmPlant` (`Simulation.cs:145–158`, `ArmPlant.cs:129–151`):**
   When `FaultChecks.JointLimit` is active without `FaultChecks.NonFinite`, `JointDefinition.IsWithinLimits(double.NaN)` returns `false`, misreporting a `NaN` state as `FaultReason.JointLimit`. In `ArmPlant.Derivatives`, the non-finite check only inspects `q` (not `qd`) and returns before updating `_appliedTorque` and `_saturated`, leaving stale torque/saturation data when `q` is non-finite.
3. **6-DOF generality constraints in `Model` and test blind spots in `RobotArm.Tests` (`JointDefinition.cs:4–11`, `BodyParameters.cs:69–90`, `CoriolisTests.cs`, `SimulationTests.cs`, `ChainTests.cs`):**
   Although the RNEA, Cholesky, plant, and solver algorithms are written for $n$ degrees of freedom, `JointAxis` lacks an `X` roll axis, `BodyParameters` and `WithPayload` enforce planar $y = 0$ symmetry, and `ActuatorParameters` is shared across all joints. In the test suite, `ChainTests` never runs kinematics, dynamics, or simulation on its 6-DOF model, `V13` passes even if plant friction is zero, and `V9`/`V10` do not assert that the state actually advances from $x_0$.
