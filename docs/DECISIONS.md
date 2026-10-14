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
