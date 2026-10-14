namespace RobotArm.Core.Simulation;

/// <summary>
/// Explicit (forward) Euler with a fixed step, input held over the step, for the teaching comparison of
/// Lecture Note 1. SPEC §6.2. The work buffer is created once, so <see cref="Step"/> does not allocate.
/// </summary>
public sealed class EulerSolver : ISolver
{
    private readonly int _n;
    private readonly double[] _dx;

    /// <summary>Creates the solver for plants with <paramref name="stateCount"/> states. SPEC §6.2.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="stateCount"/> is not greater than 0.</exception>
    public EulerSolver(int stateCount)
    {
        _n = SolverArguments.CheckStateCount(stateCount);
        _dx = new double[_n];
    }

    /// <summary>Number of states the work buffer is sized for.</summary>
    public int StateCount => _n;

    /// <summary>x(t + h) = x + h f(t, x, u). SPEC §6.2. Does not allocate.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="plant"/> is null.</exception>
    /// <exception cref="ArgumentException">A size does not match the plant or the solver.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="h"/> is not a finite number greater than 0.</exception>
    public void Step(IPlant plant, double t, Span<double> x, ReadOnlySpan<double> u, double h)
    {
        SolverArguments.Check(plant, _n, x, u, h);
        plant.Derivatives(t, x, u, _dx);
        for (var i = 0; i < _n; i++)
            x[i] += h * _dx[i];
    }
}
