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


---

## D8 – Optional CT integral feedback, payload knowledge, and an explicit overload test

**Context.** V18(b) exposes the light wrist's low physical stiffness M̂·Kp when the controller forgets a 0.3 kg payload. A task runner also needs a deliberate policy for what the controller learns at a pickup/release. The original V19 wording asked for plant saturation although D6's controller clamps first.

**Decision.**

- CT remains unchanged and integral-free by default. CT+I adds Ki ∫e to the outer-loop acceleration before the existing model/friction cancellation and midpoint prediction. Integral candidates use the elapsed sample time even when prediction is disabled, with no integration on the first sample. As in PID, each joint retains its old integral whenever its candidate torque is clamped. Reset clears the integral. No plant saturation information enters the controller.
- CT+I has separately tuned scenario defaults: Kp = 3600 1/s², Kd = 360 1/s, Ki = 10000 1/s³ on all joints (retuned in A1b from T08's 1600 / 80 / 6400). CT retains Kp = 144 and Kd = 24. Enabling an integral alone with a large Ki and those original CT gains is unstable in the forgotten-payload experiment, while Kd = 80 left the unloaded wrist with only 1.28 dB gain margin and 15.1° phase margin because P44·Kd = 0.047 < d4 = 0.21 N·m·s/rad made the net linearised velocity feedback negative (friction cancellation dominating Kd). With Kp = 3600, Kd = 360 and Ki = 10000, Pii·Kd > di on every joint and across all five planets, payloads 0 and 0.5 kg, and all library scenario start poses the 12-state closed loop has ρ ≤ 0.989816 < 1, gain margin ≥ 7.43 dB ([8.91, 9.10, 10.43, 7.43] dB worst case per joint) and phase margin ≥ 46.57° ([67.81°, 70.07°, 87.69°, 46.57°] worst case per joint). All gains remain overrideable. The PD-only MaxStableTs calculation of D6 excludes integral dynamics; §12.2–§12.4 include them.
- V18(b)'s exact timing stays 1 s moving plus 0.5 s holding. At t=1.5 s the largest joint errors are CT 0.7287002454 rad, CT+I 0.0069754717 rad, and CT using the retuned Kp/Kd (3600 / 360) with Ki=0 0.3376327568 rad. None faults. The latter comparison separates integral rejection from PD retuning. CT+I has a 0.2483847706 rad initial transient (down from 0.6293 rad with 1600 / 80 / 6400); its final-error result is not a claim of accurate tracking throughout an unknown-load run. The library extends the hold to a 10 s run so its Reach success can be judged independently (settling at 1.280 s).
- PickPlace's **Informed** policy changes the belief payload to the object's mass on attachment and zero on release. **Unaware** keeps the initial belief, including an explicitly specified nonzero value. The initial PickPlace plant is unloaded. Events happen at controller sample boundaries, which are also integration boundaries. Model changes preserve q, q̇, integral state and controller sample history; they do not silently restart the controller. PID has no payload dynamics, but its reported belief follows the same selected policy.
- V19 is a pulse-free PDG Hold on HighG with 0.5 kg at the vectors' `horizontal_reach_pose`, reconstructed by IK at (0.38, 0, d1), pitch 0 and checked against the vector and all limits. |g2| ≈ 6.84 N·m exceeds 4.1 N·m, so joint 2 must report **controller clamping at the first sample**. The pose must fail to be held, or a floor fault must stop it, and the result must explain the failure. In the measured run it loses the 2 mm pose tolerance before joint 2 crosses its upper limit at 0.0395 s; the physical saturation flag stays false, consistently with D6.

**Task/run semantics.** All task moves are smooth: joint quintics for Reach/PickPlace, quintic Cartesian line/circle phase for Track. Hold starts at its reference and applies an external torque after the motor clamp. Cartesian velocities/accelerations use analytic Jp and J̇p q̇, with singularity checks. The IK issues deferred in D5 are fixed here: a default result is a failure, and the cosine is clamped only within 1e-14 of the workspace boundary to absorb roundoff. Yaw branch-cut paths fail cleanly instead of emitting a 2π reference step. These are numerical safeguards, not a change to the geometry.

**Logging and replay.** One row represents the true state and reference at a controller sample, together with the new command and its following actual held interval. A fault truncates that interval; it does not add a fictitious controller sample. Rows include separate controller/plant flags, true/believed payload and object/phase data. Log storage is reserved at Start; normal steps, reference evaluation and logging allocate nothing. Events may allocate model buffers. Metrics are sample estimates with actual-interval weights, and failure status lives in RunResult. The stepwise API accumulates fractional h budgets, making frame scheduling irrelevant to the logged experiment. Duration is validated in (0,60] s on the h grid. The same serialized scenario and seed yield identical full logs in synchronous and irregular frame-batch runs.

**Consequences.** The ordinary CT tests and default gains remain intact; CT+I's V29 test supplements V18. All eight library outcomes are asserted. The 160-run informed robustness grid is generated by an explicit Report-category test, which CI skips. The benchmark clock and output-file handling are in Tests; Core only exposes data and writes CSV to a TextWriter. Details and measured results are in `docs/results/T08-report.md`.

---

## D9 – Linear analysis toolkit (A1, A1b): what is linearised, loop at a time, LQR projection, tuner cost

**Context.** T07 gave the sampled loop's stability as one number, ρ(Φ − ΓK), found without an eigen solver (D6). A1 adds the rest of the linear toolbox for the student (SPEC §12): poles with ωn and ζ, root locus, Bode margins, a linear step response to overlay on a run, LQR and a gain tuner. No package; `System.Numerics.Complex` only.

**Decision 1: one linearisation, stated once.** Everything uses the plant of D6 (`LinearSystem.ArmAtRest`, Φ and Γ by RK4 sub-steps) and the effective gains of SPEC §7. What is approximated is listed once, in `SampledLoopModel` and SPEC §12: one pose at rest; friction by its slope d = fv + fc/ωs; gravity by its gradient; no Coriolis terms, saturation, clamping, conditional integration or sensor effects; every sample exactly Ts after the previous one.

- *Reference and integral.* K alone cannot give a step response or the integrator states, so the controllers now also implement `IAugmentedLinearisableController` (Control): δu = −Kx + L δr + Kz w, w_k = w_{k−1} + Ts(δr − δq_k). PID: L = diag(Kp), Kz = diag(Ki); PDG: L = diag(Kp); CT: L = P diag(Kp), and CT+I adds Kz = P diag(Ki), with P = M̂ + ĜTs²/8 + D̂Ts/2 of D7. The integrator state is ∫e before the current sample, so the closed loop has 8 states for PDG and CT and 12 for PID and CT+I. PID always keeps its 12 states, even with Ki = 0. The model is checked against the controllers themselves: driving x_{k+1} = Φx + Γ(u − u0) with each controller's own `Compute` for a 1e-6 step agrees with the model over 200 samples; the largest difference, relative to the 1e-6 signal, is 1e-9 for PID, 1e-6 for PDG, 2e-5 for CT and 5e-5 for CT+I (the laws' own second-order terms).
- *Eigenvalues.* Balancing, Hessenberg reduction by stabilised elimination, Francis double-shift QR (EISPACK hqr). On the arm max |λ| agrees with T07's repeated squaring to 2.2e-14 over every controller, planet, payload and pose A, B, C.
- *A small refactor.* `ExperimentRunner.CreateController(Scenario)` (internal) builds the scenario's belief and controller, so the tuner linearises exactly the controller a run uses. `ComputedTorque.IntegralGains` exposes Ki.

**Decision 2: loop at a time.** The arm has four inputs. The margins are classical single-loop margins: open input i, keep the other three loops closed, L_i(z) = k_i(zI − A_i)⁻¹b_i, with k_i the whole row i of K_ξ (gains, integral and the controller's model terms Ĝ, P). Why: it is what a student can relate to the Bode plots of the course, and for one joint it is exactly the 1-DOF case (gain margin = critical scale factor of the root locus, tested to 1e-6 dB). Consequences, reported rather than hidden: loop-at-a-time margins do not cover simultaneous changes. With the default PDG gains at the reach start pose the joints have 10.7–18.3 dB of gain margin each, but scaling all gains together destabilises at k = 2.33 (7.34 dB). With loop 2 or 3 opened the arm has an unstable pole (gravity), so the result carries the number of open-loop unstable poles: when it is not 0 the margins are read with the Nyquist criterion. T08's CT+I defaults (1600 / 80 / 6400) had small wrist margins at the reach start pose (1.28 dB, 15.1°); A1b retuned them to 3600 / 360 / 10000 (D8), raising the reach-pose wrist margins to 9.21 dB and 51.94° and keeping ρ < 1, GM ≥ 6 dB and PM ≥ 40° on every joint across all five planets, payloads 0 and 0.5 kg, and all library start poses (checked by `DefaultGains_MarginsAtLibraryStartPoses_DoNotRegress`).

**Decision 3: LQR and its diagonal projection.** The DARE is solved by structure-preserving doubling (quadratic convergence; 10 steps on the arm), then two plain Riccati iterations; residual 2e-15 relative at the start pose, at most 2.3e-14 over all planets × payloads × poses A, B, C. The LQR is designed on the plant as linearised, gravity gradient included, so K couples the joints and contains its own gravity-gradient compensation. The projection keeps Kp,i = K[i, i] and Kd,i = K[i, 4 + i] and applies them as an independent-joint PD to the same plant. Example (Q = diag(100, …, 0.1, …), R = I, Earth, reach start pose): LQR ρ = 0.9598, slowest pair ωn = 23.9 rad/s, ζ = 0.86; projection ρ = 0.9665, ωn = 23.6 rad/s, ζ = 0.72, and one fast pole goes from z = 0.016 to −0.216. The coupling the projection drops is mostly the off-diagonal gravity-gradient terms of joints 2–4.

**Decision 4: tuner cost and robustness (A1b).** Each candidate is first checked against the linearised loop at every scenario's start pose (`SampledLoopModel` and `LoopFrequencyResponse`): ρ < 1, minimum loop-at-a-time gain margin (`GainMarginLimitDb`, default 6 dB) and minimum phase margin (`PhaseMarginLimitDeg`, default 40°) are enforced as **hard constraints** (infeasible candidates receive a barrier penalty and skip the nonlinear simulation). Feasible candidates run the real nonlinear `ExperimentRunner` over one scenario or a list of scenarios, either with each scenario's own sensor (`TuningSensorMode.Scenario`) or with both the perfect sensor and the encoder (`TuningSensorMode.PerfectAndEncoder`), combining per-run costs (ITAE Σ t·|p_tool − p_ref|·Δt plus fault penalty 10 and clamping penalty 1e-3 per % above 5 %) by sum (`TuningCostAggregation.Sum`) or worst-case maximum (`TuningCostAggregation.Max`). Gains from (ωn, ζ) by a decoupled single-joint design: CT Kp = ωn², Kd = 2ζωn; PDG and PID Kp = M̂ii ωn², Kd = max(0, 2ζωn M̂ii − d̂i), the friction slope d̂i counted as damping. Nelder–Mead in the box [0, 1]^d (ωn logarithmic), seeded, with evaluation 1 = the primary scenario's own gains, so the result is never worse than the default.

**Result** ("Dünya – boş kol – noktaya git", per joint):
- *A1 without GM constraint, perfect sensor only (60 evaluations):* cost 2.83e-3 → 3.02e-4 (0.107 ×), Kp [6, 6, 6, 2] → [1.09, 43.6, 27.9, 28.7] N·m/rad, Kd [0.3, 0.6, 0.3, 0.05] → [0, 1.01, 0, 0] N·m·s/rad. Failed with the encoder (RMS 42.3 mm, clamping 85.7 % of the time).
- *A1b with GM ≥ 6 dB and PM ≥ 40° hard constraints, perfect sensor only (60 evaluations):* cost 2.8292e-3 → 1.3918e-3 (0.4919 ×), Kp [4.95, 6.45, 9.19, 3.66] N·m/rad, Kd [0.261, 0.788, 0.139, 0.103] N·m·s/rad, min GM 6.99 dB, min PM 83.1°, ρ = 0.98570, no clamping.
- *A1b with both sensors (`PerfectAndEncoder`), 10 evaluations (normal test suite, `Sum`):* cost 1.3853e-2 → 1.0253e-2 (0.7402 ×), Kp [6.00, 6.00, 6.00, 2.00] N·m/rad, Kd [0.300, 1.186, 0.300, 0.050] N·m·s/rad, min GM 9.81 dB, min PM 102.6°, ρ = 0.99103; passes with both perfect (RMS 0.621 mm) and encoder (RMS 0.559 mm, final 0.092 mm, clamping 0 %) sensors.
- *A1b with both sensors (`PerfectAndEncoder`), 60 evaluations (Report suite):* `Sum` gives cost 1.3853e-2 → 9.3841e-3 (0.6774 ×), Kp [6.81, 4.91, 6.01, 2.33] N·m/rad, Kd [0.529, 0.697, 0.393, 0.050] N·m·s/rad, min GM 8.22 dB, min PM 97.8°, ρ = 0.98816, passing with both perfect (RMS 0.521 mm) and encoder (RMS 0.481 mm, final 0.138 mm, clamping 0 %) sensors; `Max` gives cost 1.1023e-2 → 6.6000e-3 (0.5987 ×), Kp [9.36, 4.08, 6.35, 2.10] N·m/rad, Kd [0.485, 0.692, 0.490, 0.019] N·m·s/rad, min GM 6.91 dB, min PM 94.0°, ρ = 0.99025, passing with both perfect (RMS 0.540 mm) and encoder (RMS 0.495 mm, final 0.089 mm, clamping 0 %) sensors.

**Finding: the friction band limits the linear step response.** The linear prediction agrees with the nonlinear run within 2 % for a 0.01 rad step under CT (0.30 %) and CT+I (0.79 %), and for a 0.003 rad step under PDG (0.32 %), as well as at 0.01 rad when Coulomb friction is replaced by its slope (fv' = fv + fc/ωs, fc' = 0: 0.075 %). On the true plant a 0.01 rad step on every joint drives PDG's yaw joint to 0.12 rad/s, above the smoothing speed ωs = 0.1 rad/s, where fc·tanh(q̇/ωs) is well below its slope fc/ωs, giving a 3.26 % difference (kept as a documented observation of the Coulomb friction band in V40). CT cancels the friction with its model, so it stays near-linear even at 0.5 rad (1.87 %), while PDG is off by 44 % there.

**Speed** (SPEC §12.8). Every call on the arm except the tuner stays under 50 ms natively, the slowest being a root locus at 17–32 ms (Report test V43). One tuner evaluation of the 10 s reach takes about 70 ms in Release and 1.5 s in Debug; the 60-evaluation tuner test is in the Report category while a 10-evaluation test runs in the normal suite so `dotnet test` in Debug stays well under a minute.

**Consequences.**

- The analysis is local and linear; the UI should show it next to the nonlinear run, never instead of it.
- The root-locus refinement measures pole movement relative to max(1, |z|): with absolute steps, poles far outside the unit circle at large gains used up all 400 points.
- Not in A1: Web pages for these tools, multivariable margins (disk margins, singular values), and an LQR on the gravity-compensated plant to suggest PDG gains directly.

---

## D10 – WebAssembly AOT for the published Release build

**Context.** On WebAssembly, .NET normally runs the IL in the Mono interpreter. T09 asked whether the interpreter is fast enough for Normal mode to run at real time, and whether AOT compilation is worth its cost.

**Decision.**

- `RunAOTCompilation` is true only when publishing in Release. The condition is `Configuration == Release` and either `_IsPublishing` (set by `dotnet publish`) or `WasmBuildingForNestedPublish`. The second is needed because the wasm SDK compiles in a nested publish that clears `_IsPublishing`. With `Configuration == Release` alone, `dotnet build -c Release` (the CI build) would also ask for the wasm-tools workload. Debug builds, `dotnet build`, `dotnet test` and `dotnet run` do not change.
- CI keeps build and test as they were. A separate step installs wasm-tools and runs `dotnet publish src/RobotArm.Web -c Release`, so a broken AOT build fails CI.
- The Normal-mode run loop is unchanged. It already advances by the elapsed wall time per frame, with a 50 ms cap on catch-up after a stall. Both published builds run at a real-time factor of 1.00, so the cap does not limit Normal mode.

**Measurements** (MacBook, Apple Silicon, built-in browser pane, Release publish served as static files by `python3 -m http.server`). AdvanceBy, render and RTF are medians of the "Performans" overlay's 1 s averages, sampled every 0.5 s from 1.5 s into a 10 s run. "Cold stability" is the "Son Kararlılık Hesabı" value after a fresh page load: on load for Earth, then on the first switch to Moon. Fast is the "Tüm senaryo … ms" total over two runs.

| | Interpreter | AOT |
|---|---|---|
| Earth, empty arm, reach (PDG): AdvanceBy / frame | 5.20 ms | 0.62 ms |
| render / frame | 1.07 ms | 1.26 ms |
| real-time factor | 1.00 | 1.00 |
| cold stability panel | 109–110 ms | 20–36 ms |
| Fast mode, 10 s scenario | 3268–3281 ms | 503–536 ms |
| Moon, 0.3 kg, pick and place (CT): AdvanceBy / frame | 5.23 ms | 0.62 ms |
| render / frame | 1.04 ms | 1.28 ms |
| real-time factor | 1.00 | 1.00 |
| cold stability panel | 1055–1177 ms | 105–202 ms |
| Fast mode, 10 s scenario | 3297–3301 ms | 517–530 ms |
| `dotnet publish -c Release` (warm `obj`) | 37 s | 102 s |
| `dotnet publish -c Release` (clean copy) | — | 107 s |
| `_framework` on disk (all files) | 15.7 MB | 32.8 MB |
| `_framework` uncompressed / Brotli / gzip | 9.2 / 2.8 / 3.5 MB | 20.8 / 4.8 / 7.0 MB |

The `dotnet.native.wasm` grows from 2.9 MB to 15.0 MB because it now holds the compiled code. The IL assemblies are still shipped for their metadata. A browser downloads the Brotli files, so the first load grows by about 2.1 MB.

**What the numbers say.**

- The published interpreter build already reaches real time on this machine. AdvanceBy takes 5.2 ms of the 16.7 ms frame. AOT cuts this to 0.6 ms, 8.4× faster, so a slower student laptop keeps a large margin. Fast mode is 6.3× faster and the stability panel 3–10× faster. Rendering does not change, about 1.0–1.3 ms per frame.
- With AOT, Fast mode is no longer limited by physics. At 0.62 ms per 16.7 ms of simulated time, the 10 s run needs about 370 ms of AdvanceBy. The remaining 130–160 ms is the per-chunk redraw and `Task.Delay(1)` between the 40 chunks of 0.25 s.
- `dotnet run -c Release` is not representative. It serves the build output, not the publish output, and that build output has `debugLevel: -1` (debugger attached), under which the interpreter runs without its optimisations. There, Earth reach takes 265 ms of AdvanceBy and 11–13 ms of rendering per frame. The 50 ms cap then limits it to a real-time factor of 0.18. Fast mode takes 51.7 s and the cold stability panel 1021 ms. AOT does not apply to `dotnet run`. Use the published site to judge speed.

**Consequences.**

- A Release publish takes about a minute longer on this machine (37 s → 102 s) and needs the wasm-tools workload, locally and in CI. The CI step has not run yet (nothing is pushed), so its time on GitHub's runner is still to be measured. GitHub Pages (T11) must publish the same way.
- The first page load downloads about 2.1 MB more (Brotli).
- Stability timings vary by up to 2× between page loads (GC and browser scheduling), so compare ranges, not single values.

---

## D11 – Plain CT default gains retuned for loop-at-a-time margins (A1c)

**Context.** A1b retuned CT+I's defaults (`Kp = 3600`, `Kd = 360`, `Ki = 10000`) so that every joint keeps gain margin ≥ 6 dB and phase margin ≥ 40° across all five planets, payloads 0 and 0.5 kg, and the three distinct library scenario start poses (`q0 = [0, −0.4, −0.3, 1.1]` rad from `"Dünya – boş kol – noktaya git"`, `q0 = [0, 1.3439, −1.3002, −0.0436]` rad from `"Yüksek g – 0,5 kg – tutma (doyum)"`, and `q0 = [0, −0.1, 0.2, 0.2]` rad from `"Örnekleme – Ts sınırına yakın"`). Plain CT remained at T07's `Kp = 144 1/s²`, `Kd = 24 1/s` ($\omega_n = 12\text{ rad/s}$, $\zeta = 1$), which gave the wrist only 0.34 dB gain margin and 10.3° phase margin at the worst library start pose.

**Why `Kp = 144, Kd = 24` had 0.34 dB / 10.3° margins ($P_{ii} K_{d,i}$ vs. friction slope $d_i$).** Linearised at rest (`SPEC.md` §7, §12.2), CT's velocity feedback matrix is $K_{\text{vel}} = P\operatorname{diag}(K_d) - \hat{G}\frac{T_s}{2} - \hat{D}$, where $P = \hat{M}(q_0) + \hat{G}\frac{T_s^2}{8} + \hat{D}\frac{T_s}{2}$ and $\hat{D} = \operatorname{diag}(d_i)$ with friction slope $d_i = f_v + f_c/\omega_s = 0.01 + 0.02/0.1 = 0.21\text{ N}\cdot\text{m}\cdot\text{s/rad}$. On the unloaded wrist (joint 4, 0 kg), $\hat{M}_{44} = 3.6747 \times 10^{-4}\text{ kg}\cdot\text{m}^2$ and $P_{44} \approx 5.7747 \times 10^{-4}\text{ kg}\cdot\text{m}^2$. With $K_{d,4} = 24\text{ s}^{-1}$, $P_{44} K_{d,4} = 0.01386\text{ N}\cdot\text{m}\cdot\text{s/rad} \ll d_4 = 0.21\text{ N}\cdot\text{m}\cdot\text{s/rad}$, so the diagonal velocity feedback entry is negative ($P_{44} K_{d,4} - d_4 = -0.19614 < 0$): the friction cancellation term $-\hat{D}\dot{q}$ dominates the outer-loop damping $P K_d \dot{q}$. When loop $i$ is scaled by a factor $k > 1$ in `LoopFrequencyResponse`, the entire row $k_i$ of $K_\xi$ is scaled by $k$, over-cancelling the plant's passive damping $d_i$ and driving the closed loop unstable at $k = 1.0401$ ($0.3417\text{ dB}$):
- **Plain CT worst GM = 0.34 dB (`0.3417 dB`)** occurs on **joint 4 (wrist) at Orbit ($g = 0$), 0 kg payload, Pose 2 (`"Örnekleme – Ts sınırına yakın"`, $q_0 = [0, -0.1, 0.2, 0.2]$ rad)**, with $\text{PM} = 15.91^\circ$, $\omega_{pc} = 10.62\text{ rad/s}$, $\omega_{gc} = 0.84\text{ rad/s}$, $\rho = 0.981827$ (and $0.3417\text{–}0.3419\text{ dB}$ across all five planets at 0 kg, Pose 2; on joint 4 the worst PM is $11.36^\circ$ with $\text{GM} = 0.35\text{ dB}$ at HighG, 0 kg, Pose 1 `"Yüksek g – 0,5 kg – tutma (doyum)"`).
- **Plain CT overall worst PM = 10.3° (`10.3138°`)** occurs on **joint 3 at HighG ($g = 24.79\text{ m/s}^2$), 0.5 kg payload, Pose 0 (`"Dünya – boş kol – noktaya git"`, $q_0 = [0, -0.4, -0.3, 1.1]$ rad)**, with $\text{GM} = 1.87\text{ dB}$ (at 0 kg on Pose 2, joint 3 has $P_{33} = 5.114 \times 10^{-3}\text{ kg}\cdot\text{m}^2$ and $P_{33} K_{d,3} = 0.1227 < d_3 = 0.21$, giving $\text{GM} = 0.80\text{ dB}$ and $\text{PM} = 14.37^\circ$).
- **PDG 2.69 dB (`2.6869 dB`) worst case on joint 3** occurs at **Orbit ($g = 0$), 0.5 kg payload, Pose 1 (`"Yüksek g – 0,5 kg – tutma (doyum)"`, `horizontal_reach_pose`, $q_0 = [0, 1.3439, -1.3002, -0.0436]$ rad)**, with $\text{PM} = 118.65^\circ$, $\omega_{pc} = 1570.80\text{ rad/s}$ ($\pi/T_s$), $\omega_{gc} = 13.51\text{ rad/s}$, $\rho = 0.995192$. At that outstretched pose with 0.5 kg, inertial coupling through $M^{-1}$ with loops 2 and 4 closed reduces joint 3's Nyquist-frequency gain margin; at the other two start poses PDG keeps $\text{GM} \ge 6\text{ dB}$ and $\text{PM} \ge 40^\circ$ on all joints. PDG gains are unchanged.

**Decision.**
- Retune `ComputedTorque.DefaultGains` to `Kp = [1600, 1600, 1800, 1800] 1/s²`, `Kd = [350, 350, 350, 350] 1/s`. Across the 30 cases, the minimum $P_{ii}$ is $[2.685 \times 10^{-3}, 1.206 \times 10^{-2}, 5.114 \times 10^{-3}, 5.775 \times 10^{-4}]\text{ kg}\cdot\text{m}^2$, so $P_{44} K_{d,4} \approx d_4$ requires $K_{d,4} \approx 350\text{ s}^{-1}$, and strong inertial coupling $M_{ir}(M^{-1})_{ri}$ at `horizontal_reach_pose` with 0.5 kg requires $K_d$ on joints 2–3 to match joint 4 so the off-diagonal terms of $M\operatorname{diag}(K_d)$ do not introduce phase lag when one loop is opened.
- Because `Kp = [1600, 1600, 1800, 1800]` and `Kd = 350` are overdamped ($\zeta \approx 4.1\text{–}4.4$, dominant decay rate $\lambda_1 \approx K_p/K_d \approx 4.6\text{–}5.1\text{ s}^{-1}$ so the 0.5 rad step settles in $0.87\text{ s} / 0.85\text{ s} \le 1\text{ s}$), the discontinuous 0.5 rad step of `SPEC.md` §7 commands $|u(0)| > 4.1\text{ N}\cdot\text{m}$ for the first few samples, so the controller clamps briefly at $t = 0$ while the plant never saturates (`PlantSaturated = false`). On smooth quintic trajectories (`V18`, `ScenarioLibrary`), peak $|u|$ on Earth without payload is $1.31\text{ N}\cdot\text{m}$ with zero clamping.
- `LoopFrequencyResponseTests.DefaultGains_MarginsAtLibraryStartPoses_DoNotRegress` now asserts $\text{GM} \ge 6\text{ dB}$ and $\text{PM} \ge 40^\circ$ for both `"CT"` and `"CT+I"`.

**Worst-case margins across all 5 planets × {0, 0.5} kg × 3 library start poses (30 cases):**

| Controller | max ρ | Joint 1 GM / PM | Joint 2 GM / PM | Joint 3 GM / PM | Joint 4 GM / PM |
|---|---|---|---|---|---|
| PID (unchanged) | 0.999320 | 18.34 dB / 31.73° | −23.70 dB (\|GM\| 1.07 dB) / 10.91° | −4.04 dB (\|GM\| 2.69 dB) / 60.92° | 11.26 dB / 38.90° |
| PDG (unchanged) | 0.995192 | 18.34 dB / 37.12° | 6.08 dB / 65.13° | 2.69 dB / 82.20° | 11.26 dB / 82.54° |
| CT (before A1c: 144 / 24) | 0.982309 | 3.16 dB / 44.28° | 4.16 dB / 34.74° | 0.80 dB / 10.31° | 0.34 dB / 11.36° |
| CT (after A1c: 1600,1600,1800,1800 / 350) | 0.990779 | 9.15 dB / 69.24° | 9.36 dB / 71.83° | 10.75 dB / 89.68° | 7.19 dB / 62.51° |
| CT+I (unchanged: 3600 / 360 / 10000) | 0.989816 | 8.91 dB / 67.81° | 9.10 dB / 70.07° | 10.43 dB / 87.69° | 7.43 dB / 46.57° |

**CI-fix (V18(c) on Linux).** GitHub CI (ubuntu-latest, x64) failed V18 with a JointLimit fault in case (c) (plant 0 kg, belief 0.3 kg) at t = 1.403 s, value −1.50256 rad; macOS arm64 passed. Measured on macOS arm64 with the A1c gains, case (c) is clamped in [0.0, 91.1, 92.9, 96.1] % of the 750 samples (joints 1–4), reaches min q = [0, −0.755, −1.282, −0.823] rad, max error 1.423 rad, no fault; with the old 144 / 24 gains it is never clamped, min q = [0, −0.421, −0.990, 0.099] rad, max error 0.651 rad. Cause: the 0.3 kg belief makes M̂/M at the start pose [2.23, 3.78, 13.96] on joints 2–4, so the effective wrist derivative gain per sample Kd·Ts·M̂44/M44 is 350 · 0.002 · 13.96 ≈ 9.8 (old gains: 0.67), far above the ≈ 2 at which the sampled loop stops being stable; the loop then chatters between ±4.1 N·m, and the resulting nonlinear motion amplifies last-bit differences between platforms. Smaller mismatches do not avoid it: belief 0.05 kg is already clamped 44–55 % of the time and 0.1 kg faults on joint 4 at t = 0.360 s on macOS; 0.03 kg and below stay unclamped. **Decision:** V18 keeps the symmetric ±0.3 kg mismatch and asserts for (c) only that its max error, taken up to 1.5 s or the fault, is at least 5 times (a); (c) may fault. This cannot depend on rounding: (a) is 1.73e-5 rad against 1e-3 rad, (c) is 1.42 rad against 8.6e-5 rad without a fault, and a joint-limit fault implies an error of at least 0.7 rad because the path stays at least 0.7 rad inside every limit. (a) and (b) are still asserted fault-free (no clamping; (b) max q4 = 1.454 rad against 1.97 rad).

---

## D12 – Student-written JavaScript controller ("Kendi kontrolcün", T10b)

**Context.** T10b adds a fifth controller option (`"Kendi kontrolcün (JavaScript)"`) so a student can write `function kontrol(t, q, qd, ref, model)` directly in the browser without rebuilding the .NET WebAssembly app, while keeping `RobotArm.Core` free of any browser/JS dependency.

**Decision.**
- `JsController` (`src/RobotArm.Web/Models/JsController.cs`) implements Core's `IController` in `RobotArm.Web` and invokes the compiled JS function synchronously at each sample period $T_s$ via `IJSInProcessRuntime`.
- `model.M(q)`, `model.gravity(q)` and `model.bias(q, qd)` call back into `[JSInvokable]` methods on `JsController` (`ComputeM`, `ComputeGravity`, `ComputeBias`), which evaluate `RecursiveNewtonEuler` (`InertiaMatrix` and `BiasTorque`) on the controller's `ControllerBelief` (never on the true plant). When a `PickPlace` event attaches or releases an object under `PayloadKnowledge.Informed`, `SetPayloadBelief` updates `JsController.Belief` just like built-in controllers.
- **Safety before the run:** `RobotArmCustomController.dryRunInWorker` (`src/RobotArm.Web/wwwroot/js/custom-controller.js`) runs 2000 calls with representative inputs inside a dedicated Web Worker with a hard 500 ms wall-clock timeout. If the code has a syntax error, throws, returns anything other than 4 finite numbers, or exceeds 500 ms (e.g., `while (true) {}`), the worker is terminated (`worker.terminate()`) and the run is refused with a Turkish message without freezing the UI thread.
- **Safety during the run:** `invokeKontrol` returns `[u0, u1, u2, u3, okFlag]` (using finite numbers so `System.Text.Json` never fails to deserialize `NaN`), and `JsController.Compute` verifies `okFlag == 1` and `double.IsFinite(u[i])`, clamping finite outputs to $\pm\tau_{\max}$ (`±4.1 N·m`). If any output is non-finite or invalid, `CustomExperimentRunner` stops the run cleanly with `Fault(FaultReason.NonFinite, t, -1, double.NaN)` and the Turkish message `"Kontrolcün sayı olmayan bir değer döndürdü"`. Any call slower than 5 ms records a Turkish warning in `SlowCallWarning`.
- **Persistence:** `LabState.SerializeScenarioWithCustomController` writes `"customController"` into the exported `.scenario.json`, and `LabState.DeserializeScenarioWithCustomController` extracts and strips `"customController"` before passing the JSON to `ScenarioJson.Deserialize` (preserving Core's `JsonUnmappedMemberHandling.Disallow`).
- **Verification:** On `"Dünya – boş kol – noktaya git"`, the prefilled PD + gravity law (`CustomControllerExamples.DefaultPdgCode`) produces the exact same 5000-row log as built-in `PDG` (`maxDiffAll = 0.0 <= 1e-9`). All three Turkish-commented example laws (`pdg`, `ct`, `smc`) complete the reach task with `Görev Başarılı` in 482–628 ms in Fast mode.

