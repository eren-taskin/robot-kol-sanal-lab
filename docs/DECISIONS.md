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
- `StabilityEstimate.MaxStableStep(model, q, extraDamping)` returns 2.78/λmax(M(q)⁻¹D) with D = diag(fv + fc/ωs + extra damping). The extra damping is for later controllers (derivative gains). M⁻¹D is similar to D^½M⁻¹D^½, so λmax = 1/λmin(D^−½ M D^−½), found by the Jacobi method; joints without damping are handled through the Schur complement of M. The UI warns when the chosen h exceeds this limit (SPEC §6.2).

**Result with the new defaults.** The smallest MaxStableStep over 1000 random poses inside the limits is 1.27 ms unloaded and 2.78 ms with 0.5 kg. So the default 0.5 ms has a margin of at least 2.5, and even 1 ms is stable. V13 at h = 0.5 ms: max |q̇| ≤ 3.2e-15 rad/s on all five planets, 0 and 0.5 kg. A payload can only raise the limit, because it adds a positive semidefinite term to M.

**Consequences.**
- The old case is kept as a deliberate regression test (`LectureNote1Lesson_StepAboveStabilityLimit_MakesHeldArmChatter_Regression`): ωs = 0.01, h = 1 ms. The estimate calls h unstable and the held arm's speed grows above 1e-3 rad/s.
- The estimate is a local, linear one at rest. Controller gains add damping and stiffness, and a large Kd can lower the limit again; T07 should pass its derivative gains as `extraDamping`.
- Steps up to 5 ms remain allowed, so the warning in the UI is needed.

---

## D4 – A non-finite angle gives NaN derivatives, not an exception

**Context.** SPEC §6.4 says a state with NaN or infinity is a fault that stops the experiment and shows why. Within one RK4 step, a NaN input makes the stage state NaN. The next stage then builds M(q) at a NaN angle, and the Cholesky solver throws "not positive definite". So the run ended with an exception from deep inside the dynamics instead of the fault.

**Decision.** `ArmPlant.Derivatives` checks the joint angles first. If any is NaN or infinite, it fills ẋ with NaN and returns without building M. A NaN torque command passes through as NaN (clamping keeps NaN). An infinite command is clamped to ±τmax and flagged as saturated, which is physics, not a fault. The state then becomes NaN and `Simulation` reports `FaultReason.NonFinite` after the step.

**Why.** The fault path of SPEC §6.4 is the one place that decides how a run ends. The solver and plant stay simple and do not throw for a bad number. A matrix that is not positive definite for finite q is still an exception, because that is a model error, not a run-time fault.

**Consequences.** With fault checks off (`FaultChecks.None`) a NaN state keeps going as NaN without an exception. Tests cover both cases.
