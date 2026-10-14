namespace RobotArm.Core.Simulation;

/// <summary>
/// A fixed-step integrator. SPEC §6.2; PROJECT.md §2 rule 2; docs/DECISIONS.md D1.
/// </summary>
/// <remarks>
/// The input u of <see cref="Step"/> is passed once, so it is held over the whole step
/// (zero-order hold, SPEC §6.2): every stage of the method sees the same input.
/// </remarks>
public interface ISolver
{
    /// <summary>
    /// Advances <paramref name="x"/> in place from t to t + h with input <paramref name="u"/> held constant.
    /// SPEC §6.2. Must not allocate.
    /// </summary>
    /// <param name="plant">The plant ẋ = f(t, x, u).</param>
    /// <param name="t">Time at the start of the step, s.</param>
    /// <param name="x">State at t; on return the state at t + h.</param>
    /// <param name="u">Input held over the step.</param>
    /// <param name="h">Step length, s.</param>
    void Step(IPlant plant, double t, Span<double> x, ReadOnlySpan<double> u, double h);
}
