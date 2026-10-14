namespace RobotArm.Core.Simulation;

/// <summary>
/// A continuous-time plant ẋ = f(t, x, u) with a vector input. SPEC §6.1; PROJECT.md §2 rule 2;
/// docs/DECISIONS.md D1 (the course's scalar-input interface extended to <see cref="InputCount"/> inputs).
/// </summary>
public interface IPlant
{
    /// <summary>Length of the state vector x.</summary>
    int StateCount { get; }

    /// <summary>Length of the input vector u.</summary>
    int InputCount { get; }

    /// <summary>Writes ẋ = f(t, x, u) into <paramref name="dx"/>. SPEC §6.1. Must not allocate.</summary>
    /// <param name="t">Time, s.</param>
    /// <param name="x">State, length <see cref="StateCount"/>.</param>
    /// <param name="u">Input, length <see cref="InputCount"/>.</param>
    /// <param name="dx">Output: state derivative, length <see cref="StateCount"/>.</param>
    void Derivatives(double t, ReadOnlySpan<double> x, ReadOnlySpan<double> u, Span<double> dx);
}
