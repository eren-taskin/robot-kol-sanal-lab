namespace RobotArm.Core.Simulation;

/// <summary>
/// Classical fourth-order Runge–Kutta with a fixed step [eq. 22], input held over the step. SPEC §6.2.
/// </summary>
/// <remarks>
/// The course version allocates its stage arrays in every step (Lecture Note 2, caution). Here they are
/// fields created once in the constructor, so <see cref="Step"/> does not allocate (PROJECT.md §3).
/// An instance holds work buffers, so it is not safe to use from two threads at once.
/// </remarks>
public sealed class Rk4Solver : ISolver
{
    private readonly int _n;
    private readonly double[] _k1;
    private readonly double[] _k2;
    private readonly double[] _k3;
    private readonly double[] _k4;
    private readonly double[] _stage;

    /// <summary>Creates the solver for plants with <paramref name="stateCount"/> states. SPEC §6.2.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="stateCount"/> is not greater than 0.</exception>
    public Rk4Solver(int stateCount)
    {
        _n = SolverArguments.CheckStateCount(stateCount);
        _k1 = new double[_n];
        _k2 = new double[_n];
        _k3 = new double[_n];
        _k4 = new double[_n];
        _stage = new double[_n];
    }

    /// <summary>Number of states the work buffers are sized for.</summary>
    public int StateCount => _n;

    /// <summary>
    /// x(t + h) = x + h/6 (k1 + 2 k2 + 2 k3 + k4), with the same <paramref name="u"/> in all four stages.
    /// SPEC §6.2 [eq. 22]. Does not allocate.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="plant"/> is null.</exception>
    /// <exception cref="ArgumentException">A size does not match the plant or the solver.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="h"/> is not a finite number greater than 0.</exception>
    public void Step(IPlant plant, double t, Span<double> x, ReadOnlySpan<double> u, double h)
    {
        SolverArguments.Check(plant, _n, x, u, h);
        var half = 0.5 * h;

        plant.Derivatives(t, x, u, _k1);

        for (var i = 0; i < _n; i++) _stage[i] = x[i] + half * _k1[i];
        plant.Derivatives(t + half, _stage, u, _k2);

        for (var i = 0; i < _n; i++) _stage[i] = x[i] + half * _k2[i];
        plant.Derivatives(t + half, _stage, u, _k3);

        for (var i = 0; i < _n; i++) _stage[i] = x[i] + h * _k3[i];
        plant.Derivatives(t + h, _stage, u, _k4);

        var sixth = h / 6.0;
        for (var i = 0; i < _n; i++)
            x[i] += sixth * (_k1[i] + 2.0 * _k2[i] + 2.0 * _k3[i] + _k4[i]);
    }
}
