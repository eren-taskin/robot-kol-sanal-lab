# Design decisions

Each entry records a decision that departs from, or extends, the lecture notes, and why. New entries are appended with the next number.

---

## D1 – IPlant takes a vector input

**Context.** Lecture Note 2 defines the plant interface with a scalar input:

```csharp
void Derivatives(double t, double[] x, double u, double[] dx);
```

That fits a single-input plant. The OpenMANIPULATOR-X has four actuated joints, so the input is a vector of four torques (SPEC §2, §6.1).

**Decision.** Our interfaces (PROJECT.md §2, rule 2) are:

```csharp
public interface IPlant
{
    int StateCount { get; }
    int InputCount { get; }
    void Derivatives(double t, ReadOnlySpan<double> x, ReadOnlySpan<double> u, Span<double> dx);
}
public interface ISolver
{
    void Step(IPlant plant, double t, Span<double> x, ReadOnlySpan<double> u, double h);
}
```

**Why.**

- *Vector input.* `u` becomes `ReadOnlySpan<double>` of length `InputCount` (4 for this arm). The course's scalar case is the special case `InputCount = 1`, so nothing in Note 2 is lost.
- *Sizes are declared.* `StateCount` (8) and `InputCount` (4) let a solver size its work buffers once, in its constructor, and let callers check array lengths. The same solver then works for a 6-DOF arm (12 states, 6 inputs) without change.
- *Spans instead of arrays.* `ReadOnlySpan` makes the read-only arguments (`x`, `u`) read-only at compile time, and `Span` makes the outputs (`dx`, the stepped `x`) explicit. Spans let callers pass slices of preallocated buffers, which supports the rule that nothing called once per step allocates (PROJECT.md §3).
- *Input held over the step.* `ISolver.Step` takes `u` once, so the zero-order hold of SPEC §6.2 is part of the contract: every stage of RK4 sees the same input.

**Consequences.** Code from the lecture notes cannot be pasted unchanged; the `u` argument and array types must be adapted. The deviation is also explained in the report.

---

## D2 – Inertia matrix by columns of RNEA; rotor inertia outside the rigid-body recursion

**Context.** SPEC §5.2 allows either the Walker–Orin column method (column k of M = RNEA(q, 0, e_k) with gravity zero) or the composite rigid body algorithm (CRBA). It also requires N²·Jm on each diagonal entry of M.

**Decision.**

- `RecursiveNewtonEuler.InertiaMatrix` uses the **Walker–Orin column method**. The joint rotations are computed once per call and shared by the n column passes. The entries on and below the diagonal are taken from the columns and mirrored above it, so M is exactly symmetric, as the Cholesky solver expects.
- The rigid-body recursion knows nothing about the motors. The reflected rotor inertia N²·Jm is added afterwards: to the diagonal of M, and as N²·Jm·q̈i to `Inverse`. `BiasTorque` does not depend on Jm. So `Inverse(q, q̇, q̈) = M(q) q̈ + h(q, q̇)` holds exactly, with or without rotor inertia.
- Gravity enters the recursion as a base acceleration a0 = −gravity, with gravity = (0, 0, −g) from a `Planet` (SPEC §3.5).

**Why.** The column method reuses the one recursion that has to be right anyway (RNEA), so there is one algorithm to explain and defend instead of two. It costs n RNEA passes (about 0.5 µs for the 4-DOF arm in Release), which is negligible at h = 1 ms. CRBA would be faster for long chains but adds a second algorithm with its own bookkeeping. Keeping Jm outside the recursion makes "Jm adds exactly N²·Jm to each diagonal entry and changes nothing else" true by construction, and it is tested.

**Consequences.** If a long chain ever makes M the bottleneck, CRBA can replace the column method behind the same `InertiaMatrix` signature.

---

## D3 – Friction smoothing ωs = 0.1 rad/s, default step 0.5 ms, and a stability estimate

**Context.** SPEC §5.4 smooths Coulomb friction as fc tanh(q̇/ωs). Near q̇ = 0 this is linear damping with d = fv + fc/ωs on each joint. With the first defaults (fv = 0.01, fc = 0.02, ωs = 0.01) d = 2.01 N·m·s/rad. The speeds then obey q̈ ≈ −M(q)⁻¹ D q̇, and for the unloaded arm the fastest mode decays at λmax ≈ 1.7e4 1/s. RK4 is stable on the negative real axis only for h·λmax < 2.78 (2.785 exactly), so h had to be below about 0.17 ms. The default step was 1 ms.

Task T05 found this with V13. An arm held with τ = g(q) and friction on should stay still. Instead, rounding noise of about 1e-17 grew by a factor of about 19 per step until the speed left the tanh band, and the arm chattered. This is the lesson of Lecture Note 1 about step size: the solver moved the arm, not the physics. Measured worst |q̇| over 1 s, unloaded and 0.5 kg, Earth and HighG, five poses inside the limits:

| h | worst \|q̇\| (rad/s), ωs = 0.01 |
|---|---|
| 1 ms | 3.5e-2 |
| 0.5 ms | 1.9e-2 |
| 0.4 ms | 1.3e-2 |
| 0.2 ms | 2.6e-3 |
| 0.1 ms | 3.4e-16 |

Predicted limit 2.78/λmax with ωs = 0.01, unloaded: 0.164 ms at q = 0, 0.165–0.295 ms over the five poses; with 0.5 kg: 0.58–1.99 ms.

**Decision** (conductor, after T05).

- ωs = 0.1 rad/s (SPEC §3.3). Then d = 0.21 N·m·s/rad. The friction torque still reaches 76 % of fc at 0.1 rad/s and 99.9 % at 0.4 rad/s, so the model stays a Coulomb model at the speeds of the tasks.
- Default step h = 0.5 ms (SPEC §6.2); the allowed range stays 0.1 to 5 ms.
- `StabilityEstimate.MaxStableStep(model, q, extraDamping)` returns 2.78/λmax(M(q)⁻¹D) with D = diag(fv + fc/ωs + extra damping). ~~The extra damping is for later controllers (derivative gains).~~ Corrected in T07 (D6): controller gains do not enter this limit; the extra damping is only for continuous damping that the actuator parameters leave out. M⁻¹D is similar to D^½M⁻¹D^½, so λmax = 1/λmin(D^−½ M D^−½), found by the Jacobi method; joints without damping are handled through the Schur complement of M. The UI warns when the chosen h exceeds this limit (SPEC §6.2).

**Result with the new defaults.** The smallest MaxStableStep over 1000 random poses inside the limits is 1.27 ms unloaded and 2.78 ms with 0.5 kg. So the default 0.5 ms has a margin of at least 2.5, and even 1 ms is stable. V13 at h = 0.5 ms: max |q̇| ≤ 3.2e-15 rad/s on all five planets, 0 and 0.5 kg. A payload can only raise the limit, because it adds a positive semidefinite term to M.

**Consequences.**
- The old case is kept as a deliberate regression test (`LectureNote1Lesson_StepAboveStabilityLimit_MakesHeldArmChatter_Regression`): ωs = 0.01, h = 1 ms. The estimate calls h unstable and the held arm's speed grows above 1e-3 rad/s.
- The estimate is a local, linear one at rest. ~~Controller gains add damping and stiffness, and a large Kd can lower the limit again; T07 should pass its derivative gains as `extraDamping`.~~ Corrected in T07 (D6): u is held over each step, so controller gains never enter the RK4 limit; they decide the stability of the sampled loop instead.
- Steps up to 5 ms remain allowed, so the warning in the UI is needed.

**Limitations** (corrected in T07). The estimate is evaluated at one pose q. M(q) changes along a trajectory (m11 shrinks when the arm folds towards the base axis), so a step that is stable at the start pose can be too large later in the run. It ignores the gravity gradient ∂g/∂q, so it models only the damped velocity modes (with no damping at all it returns +∞, although undamped oscillating modes still limit h). Controller gains are not a limitation of this estimate: the solver holds u constant over each step, and a sampled controller holds it over Ts ≥ h, so in the RK4 stages the controller is a constant input and the step-size limit depends on the plant's inertia and friction only. ~~It also treats extra damping as continuous damping inside the RK4 stages … T07 will extend the estimate to the linearised closed loop.~~ The gains decide a different stability, that of the sampled closed loop, which T07 computes separately (`SampledLoopStability`, D6). Friction is now per joint (D5): d_i = fv,i + fc,i/ωs,i.

---

## D4 – A non-finite angle gives NaN derivatives, not an exception

**Context.** SPEC §6.4 says a state with NaN or infinity is a fault that stops the experiment and shows why. Within one RK4 step, a NaN input makes the stage state NaN. The next stage then builds M(q) at a NaN angle, and the Cholesky solver throws "not positive definite". So the run ended with an exception from deep inside the dynamics instead of the fault.

**Decision.** `ArmPlant.Derivatives` checks the joint angles first. If any is NaN or infinite, it fills ẋ with NaN and returns without building M. A NaN torque command passes through as NaN (clamping keeps NaN). An infinite command is clamped to ±τmax and flagged as saturated, which is physics, not a fault. The state then becomes NaN and `Simulation` reports `FaultReason.NonFinite` after the step.

**Why.** The fault path of SPEC §6.4 is the one place that decides how a run ends. The solver and plant stay simple and do not throw for a bad number. A matrix that is not positive definite for finite q is still an exception, because that is a model error, not a run-time fault.

**Consequences.** ~~With fault checks off (`FaultChecks.None`) a NaN state keeps going as NaN without an exception.~~ Changed in T06b (D5): a non-finite state is always reported as `FaultReason.NonFinite`, whatever `FaultChecks` says. The plant now checks q̇ as well as q, and on a non-finite state it sets `AppliedTorque` to NaN and clears the saturation flags.

---

## D5 – T06 review: what was fixed (T06b) and what was deferred

**Context.** The independent review of T02–T05 (`docs/reviews/T06-review.md`, Antigravity/Gemini) confirmed the parameters, kinematics, dynamics, solver and the PROJECT.md rules, and found bugs in non-finite handling, limits on 6-DOF generality and blind spots in the tests. The conductor chose which to fix in T06b.

**Fixed.**

- *StabilityEstimate, NaN pose.* `MaxDampingRate` and `MaxStableStep` throw `ArgumentException` (parameter `q`, naming the joint) when an angle is not finite. Before, a NaN pose returned +∞ ("any step is stable") or threw from inside Cholesky.
- *Fault detection.* A non-finite state is always `FaultReason.NonFinite`, whatever `FaultChecks` says, and it is checked before the joint limits. Before, with `JointLimit` on and `NonFinite` off, a NaN angle failed `IsWithinLimits` and was reported as a `JointLimit` fault on joint 1. NaN can never be a valid state, so the check is not optional; `FaultChecks.NonFinite` is kept so existing code compiles, and `FaultChecks.None` now means "no limit or floor checks". This changes the consequence of D4.
- *ArmPlant, non-finite state.* `Derivatives` checks the whole state, q and q̇. On a non-finite value it fills ẋ with NaN, sets `AppliedTorque` to NaN and clears `Saturated`, so no value from an earlier call remains, and RNEA and Cholesky are not run.
- *6-DOF readiness.*
  - `JointAxis.X` (roll, `Rotations.Rx`) is supported by the frame chain and RNEA.
  - `BodyParameters` has a general constructor (mass, centre-of-mass vector, full symmetric positive definite inertia tensor) next to the planar one of SPEC §3.2. The dynamics always used the full vector and tensor. `IsPlanar` says whether Cy = Ixy = Iyz = 0.
  - Actuator parameters are per joint: `ArmModel.Actuators` (one `ActuatorParameters` per joint) replaces `ArmModel.Actuator`. `OpenManipulatorX()` gives all four joints the XM430-W350 defaults of SPEC §3.3. A constructor overload that takes one `ActuatorParameters` for every joint is kept. `RecursiveNewtonEuler` (N²·Jm per joint), `ArmPlant` (τmax, fv, fc, ωs per joint) and `StabilityEstimate` (dᵢ per joint) use them.
  - `WithPayload` stays planar (SPEC §5.5). It throws `NotSupportedException` when the last body is not planar or the tool point has y ≠ 0, instead of building a wrong combined body.
  - A test-only 6-DOF chain (`tests/RobotArm.Tests/SixJointChain.cs`, axes Z, Y, Y, X, Y, X, off-axis centres of mass, rotated inertia tensors with nonzero products, different actuators per joint) is run through the frame chain, RNEA, the inertia matrix, the plant, RK4 and the stability estimate. Results: kinetic energy ½q̇ᵀMq̇ against Σ(½mv² + ½ωᵀIω) + Σ½N²Jm q̇² from the frames by central differences, 100 random states: largest relative mismatch 5.1e-10; τ = Mq̈ + h to 1e-15 relative; g(q) = ∂V/∂q to 8e-10 and Cq̇ against the Lagrange form to 2e-9 relative; free motion without friction, 0.4 s at h = 0.5 ms, E0 ≈ 15 J: drift 5.6e-11 J; M symmetric (raw columns to 1.1e-16) and positive definite in 1000 poses.
- *Test blind spots.*
  - V9 and V10 also assert that the state moved (max |q − q0| > 0.1 rad; it is 3.4 rad), so a frozen simulation cannot pass.
  - Friction: with τ = 0 and friction on, energy decreases in every step and E(t) − E(0) equals ∫ −q̇ᵀτf dt (Simpson's rule) to 8e-8 of the dissipated energy. V13 could not see friction at all.
  - V6 checks the symmetry of the raw Walker–Orin columns (internal `RecursiveNewtonEuler.InertiaColumns`, visible to the tests through `InternalsVisibleTo`) to 1e-12, before `InertiaMatrix` mirrors them; measured 3.5e-18.

**Deferred.**

- The stability estimate's limits (one pose, no stiffness, continuous instead of sampled damping): see the Limitations paragraph of D3; T07 extends it to the linearised closed loop.
- `ReflectedRotorInertia` = N²·Jm can overflow to +∞ for absurd finite inputs; not validated yet.
- `PlanarJacobian.TryInvert` and `IsNearSingular` do not validate `threshold` (0 or negative would allow a division by zero).
- `InverseKinematicsResult`: `default` reports success, and |cos φ| = 1 + 1e-16 from rounding on the stretched-elbow boundary is reported as unreachable. To be handled with the Cartesian trajectories of T08.
- Exception messages in `BodyParameters`, `JointDefinition` and `Planet` format numbers without `CultureInfo.InvariantCulture` (style).
- V8 and the C·q̇ test do not test C(q, q̇)·w for w ≠ q̇; needed only if the optional Slotine–Li controller is built.
- `WithPayload` for non-planar bodies (general 3D parallel-axis combination) and fixed joint-frame rotations beyond the three elementary axes, for T14 if a real 6-DOF arm is added.

---

## D6 – Anti-windup without reading the plant, and the two kinds of stability

**Context.** SPEC §5.6 says a controller never reads the true plant, but SPEC §7 said the PID stops integrating "while the plant reports saturation", and T07's prompt asked for the plant's saturation flags to be passed into the PID. Separately, the step-size paragraph of SPEC §6.2 and D3 (written in T05) said the controller's derivative gains enter the RK4 limit.

**Decision 1: anti-windup from the controller's own model.**

- Every controller knows τmax from its own model (its belief, `ControllerBelief`), clamps its output to ±τmax and records which joints it clamped (`IController.Clamped`, an output for logging). Nothing about saturation goes into a controller.
- PID anti-windup is conditional integration: at each sample the candidate integral Ii + ei·Δt is formed and the output computed with it; if that output exceeds ±τmax, it is clamped and the old integral is kept for that joint, otherwise the candidate is kept.
- With an exact belief the plant never saturates, because the command never exceeds the true τmax. The plant still reports saturation (SPEC §5.4) for the case where the belief's τmax is wrong. The control loop records both.

**Why.** The controller is the part that would run on the real arm. On the real arm it would not get a saturation flag from the physics either, only its own knowledge of the motor's limit. Reading the plant would also make a mismatch experiment (wrong τmax in the belief) impossible to express.

**Decision 2: two kinds of stability, kept apart.**

1. *RK4 step size* (SPEC §6.2, `StabilityEstimate`). The solver holds u over each step h, and the controller holds it over Ts ≥ h. Inside the RK4 stages the controller is a constant input, so the step-size limit h·λmax(M⁻¹D) < 2.78 depends on the plant's inertia and friction only. Controller gains, Kd included, do not enter it. SPEC §6.2 and D3 were corrected.
2. *Sampled closed loop* (SPEC §6.3, `SampledLoopStability`). The gains act on samples that are held for Ts, so the loop is a discrete-time system x_{k+1} = (Φ − ΓK) x_k. The plant is linearised at rest at a pose q0 (A = [[0, I], [−M⁻¹G, −M⁻¹D]], B = [[0], [M⁻¹]], G = ∂g/∂q by central differences). Φ and Γ come from integrating the linear system over Ts with RK4 sub-steps from the unit columns (at least 200 per Ts, more if Ts·‖A‖∞ needs it), so no package and no matrix exponential are needed. Each controller supplies its effective K (`ILinearisableController`, SPEC §7): PID [Kp | Kd] (integral left out, gravity uncompensated), PDG [Kp − Ĝ | Kd], CT [P Kp − Ĝ | P Kd − Ĝ Ts/2 − D̂] with P = M̂ + Ĝ Ts²/8 + D̂ Ts/2 (D7). Ĝ and D̂ are in K because they are computed from the sample and held.

**Spectral radius without an eigen solver.** ρ = lim ‖P^m‖^{1/m} with m = 2^k by repeated squaring. P^(2^k) is kept as e^{L_k}·B_k with ‖B_k‖∞ = 1 (B_{k+1} = B_k²/s, L_{k+1} = 2L_k + ln s), so nothing overflows, and ln ρ ≈ L_k/2^k. The error is about ln(C)/2^k, where C measures how far P is from normal. The conductor suggested k up to about 12, but that leaves about 1e-4: for the 1-DOF test, 12 squarings give an error of 1.1e-4, more than the 1e-6 the test asks for. The loop therefore continues until two estimates differ by less than 1e-14 (at most 60 squarings), which gives 5e-15 on the same example. Rounding is harmless: the 2^k-th root divides the accumulated error by 2^k. MaxStableTs is the first Ts where ρ reaches 1, found by doubling from 0.1 ms and then bisection to 1e-4 relative.

**Results** (SPEC §7 has the tables). With the default gains at Ts = 2 ms, ρ < 1 for PDG and CT on every planet × payload (0, 0.5 kg) at three poses: ρ = 0.9845–0.9945 for PDG and 0.9785–0.9818 for CT. MaxStableTs is 6.5–29.5 ms for PDG and 80.7–86.2 ms for CT. The smallest value, 6.5 ms, is set by the yaw joint at pose A, where m11 = 8.1e-4 kg·m² (halving its Kd raises it to 8.3 ms). The PID's PD part is unstable at any Ts in one case, HighG with 0.5 kg leaning back (pose A): there Kp = 6 N·m/rad is smaller than the destabilising gravity gradient, and only gravity compensation helps. The prediction agrees with the nonlinear simulation: holding pose B from a 1e-4 rad offset, at 1.1 × MaxStableTs the error grows until a joint limit or the floor stops the run (ρ = 1.09–1.27); at 0.9 × it decays by 1e-3 or more (ρ = 0.74–0.92).

**Consequences.**

- `ILinearisableController.LinearFeedbackGain(q0, Ts, K)` takes Ts because the CT law with prediction depends on it. `MaxStableSamplePeriod` has an overload that recomputes K for each Ts it tries.
- The analysis is local (one pose, at rest, no saturation, no quantisation), like the step-size estimate. The UI can show both limits: h against `StabilityEstimate.MaxStableStep`, and Ts against `SampledLoopStability.MaxStableSamplePeriod`.
- SPEC §11 V19 says "the hold test reports saturation". With Decision 1 and an exact belief, the PDG controller clamps, so the plant never saturates. When V19 is written it should check the controller's clamping, or use a belief with a larger τmax.

---

## D7 – Computed torque: friction in the model, and half-sample prediction

**Context.** SPEC §7's CT law u = M̂(q)(q̈_d + Kd ė + Kp e) + ĥ(q, q̇) did not meet V18 (track a quintic to 1e-3 rad with the exact model) or the settling criterion with this arm at Ts = 2 ms. T07 found two causes, both on the light wrist (M44 ≈ 3.7e-4 kg·m²).

1. *Friction left in the loop.* With ĥ = Cq̇ + g only, the smoothed friction stays in the plant. Near rest it is damping d = fv + fc/ωs = 0.21 N·m·s/rad, and d/M44 ≈ 570 1/s swamps Kd. The designed loop ë + Kd ė + Kp e = 0 becomes a creeping mode with a time constant of seconds: the step still had 0.33–0.39 rad of error after 2 s. Feeding friction forward on q̇_d instead of q̇ tracks well, but it leaves the same creep at the end of every move.
2. *The held torque goes stale.* u is held for Ts = 2 ms, but the torque the model asks for changes during the hold. The wrist's gravity torque depends on θ4 = q2 + q3 + q4, which turns at 2–3 rad/s during the quintic, and its friction changes within M44/d = 1.8 ms. With friction compensated at the measured speed, the V18 error was 4.9e-3 rad. Without friction at all it was still 1.8e-3 rad, which identified gravity as the main part. Raising Kp to 400 1/s² gave 2.6e-3 rad and saturated on the criterion step.

**Decision.**

- CT includes the belief's friction τ̂f (SPEC §5.4) in the terms it cancels. The full model is then M̂ q̈ + ĥ + τ̂f.
- CT evaluates the model at the state predicted for the middle of the hold. With a = q̈_d + Kd ė + Kp e and Δt the time since the previous sample (0 at the first sample after a reset): q̄ = q + q̇·Δt/2 + a·Δt²/8 and q̄̇ = q̇ + a·Δt/2, and u = M̂(q̄) a + ĥ(q̄, q̄̇) + τ̂f(q̄̇). The prediction is an option (`predictHalfSample`, on by default); off gives the textbook law, so the lab can show the difference.

**Result.** On the first V18 path (A → B of SPEC §7) the exact model gave 5.5e-4 rad without payload and 1.0e-4 rad with 0.3 kg; the textbook law gave 4.9e-3 rad. A forgotten 0.3 kg dropped the arm to the floor there, so V18 now uses a path that stays high: [0, −0.4, −0.3, 1.1] → [0, 0.1, −0.8, 0.6] rad, with the tool at least 0.218 m up. Max errors there, with the mid-hold law and with the textbook law: (a) exact model 3.9e-4 and 3.4e-3 rad; (b) plant 0.3 kg, belief 0 kg: 1.12 and 1.13 rad; (c) plant 0 kg, belief 0.3 kg: 0.65 and 0.67 rad. None of them faults. In (b) the wrist sags until it nearly hangs, because CT's wrist stiffness with a 0 kg belief is only M̂44·Kp ≈ 0.05 N·m/rad. The gains could stay at ωn = 12 rad/s, ζ = 1 (Kp = 144 1/s², Kd = 24 1/s, peak 1.9 N·m on the criterion step). Exactly averaging the friction over the hold, assuming linear speed, gave the same numbers as the midpoint, so the simpler midpoint is used.

**Consequences.**

- The effective gain K of CT depends on Ts (D6). The finite-difference test of the law covers it at Ts = 2 ms and 30 ms.
- The friction cancellation needs a speed estimate better than the friction band ωs = 0.1 rad/s. The unfiltered encoder speed moves in steps of 0.77 rad/s (one count per 2 ms), so with it the wrist creeps again: 15 counts of error after 3 s (kept as a regression test). With the first-order filter at τ = 5 ms the step ends within 0.1 count. PDG and PID do not have this problem: they end within 0.3 count with or without the filter.
- CT's stiffness is M̂·Kp, so the wrist is soft: about 0.05 N·m/rad. A wrong payload in the belief therefore shows up mostly as a wrist error.

