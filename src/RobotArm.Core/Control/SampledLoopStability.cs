using System.Globalization;

namespace RobotArm.Core.Control;

/// <summary>
/// Stability of the sampled closed loop: a continuous linear plant ẋ = A x + B u under a sampled state feedback
/// u_k = −K x_k held over Ts (zero-order hold). SPEC §6.3, §7; docs/DECISIONS.md D6.
/// </summary>
/// <remarks>
/// <para>
/// This is a different stability from the RK4 step-size limit of SPEC §6.2. The solver holds u over each step h,
/// so that limit depends on the plant (friction) only. Here the controller's gains matter: the loop
/// x_{k+1} = (Φ − ΓK) x_k is stable when the spectral radius ρ(Φ − ΓK) &lt; 1.
/// </para>
/// <para>
/// Φ = e^{A·Ts} and Γ = ∫₀^Ts e^{Aτ}dτ B are found without a matrix exponential: the linear system is integrated
/// over Ts with RK4 sub-steps from each unit state (u = 0) for the columns of Φ, and from x = 0 with each unit
/// input held for the columns of Γ. ρ is found without an eigen solver, by Gelfand's formula
/// ρ = lim ‖Φcl^m‖^{1/m} with m = 2^k by repeated squaring, rescaled at each squaring (see
/// <see cref="SpectralRadius(ReadOnlySpan{double}, int, int)"/>).
/// </para>
/// <para>The methods allocate; they are for analysis and the user interface, not for once per step.</para>
/// </remarks>
public static class SampledLoopStability
{
    /// <summary>Default number of RK4 sub-steps per Ts for the discretisation.</summary>
    public const int DefaultSubsteps = 200;

    /// <summary>Largest number of squarings in <see cref="SpectralRadius(ReadOnlySpan{double}, int, int)"/>.</summary>
    public const int MaxSquarings = 60;

    /// <summary>
    /// Φ and Γ of the zero-order-hold discretisation over <paramref name="samplePeriod"/>, both row-major
    /// (Φ: nx × nx, Γ: nx × nu). SPEC §6.3; docs/DECISIONS.md D6.
    /// </summary>
    /// <param name="system">The continuous plant.</param>
    /// <param name="samplePeriod">Ts, s.</param>
    /// <param name="substeps">
    /// RK4 sub-steps per Ts, at least 1. More are used when needed to keep (Ts/substeps)·‖A‖∞ ≤ 0.1, so that each
    /// sub-step resolves the fastest plant mode.
    /// </param>
    /// <exception cref="ArgumentNullException"><paramref name="system"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">Ts is not positive and finite, or substeps is less than 1.</exception>
    public static (double[] Phi, double[] Gamma) Discretise(LinearSystem system, double samplePeriod, int substeps = DefaultSubsteps)
    {
        ArgumentNullException.ThrowIfNull(system);
        if (!double.IsFinite(samplePeriod) || samplePeriod <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(samplePeriod), samplePeriod, "Sample period must be a finite number greater than 0 s.");
        if (substeps < 1)
            throw new ArgumentOutOfRangeException(nameof(substeps), substeps, "At least one sub-step is needed.");

        var nx = system.StateCount;
        var nu = system.InputCount;
        var a = system.A;
        var b = system.B;

        var normA = 0.0;
        for (var r = 0; r < nx; r++)
        {
            var row = 0.0;
            for (var c = 0; c < nx; c++) row += Math.Abs(a[r * nx + c]);
            normA = Math.Max(normA, row);
        }
        var count = (int)Math.Min(int.MaxValue, Math.Max(substeps, Math.Ceiling(samplePeriod * normA / 0.1)));
        var h = samplePeriod / count;

        var phi = new double[nx * nx];
        var gamma = new double[nx * nu];
        var x = new double[nx];
        var bu = new double[nx];
        var work = new Rk4Work(nx);

        for (var j = 0; j < nx; j++)
        {
            Array.Clear(x);
            x[j] = 1.0;
            Array.Clear(bu);
            for (var s = 0; s < count; s++) work.Step(a, nx, bu, x, h);
            for (var i = 0; i < nx; i++) phi[i * nx + j] = x[i];
        }
        for (var j = 0; j < nu; j++)
        {
            Array.Clear(x);
            for (var i = 0; i < nx; i++) bu[i] = b[i * nu + j];
            for (var s = 0; s < count; s++) work.Step(a, nx, bu, x, h);
            for (var i = 0; i < nx; i++) gamma[i * nu + j] = x[i];
        }
        return (phi, gamma);
    }

    /// <summary>
    /// Φcl = Φ − Γ K of the sampled loop u_k = −K x_k, nx × nx row-major. SPEC §6.3; docs/DECISIONS.md D6.
    /// </summary>
    /// <param name="system">The continuous plant.</param>
    /// <param name="gain">K, nu × nx row-major, e.g. from <see cref="ILinearisableController.LinearFeedbackGain"/>.</param>
    /// <param name="samplePeriod">Ts, s.</param>
    /// <param name="substeps">RK4 sub-steps per Ts (see <see cref="Discretise"/>).</param>
    /// <exception cref="ArgumentException">K has the wrong number of entries or an entry that is not finite.</exception>
    public static double[] ClosedLoop(LinearSystem system, ReadOnlySpan<double> gain, double samplePeriod, int substeps = DefaultSubsteps)
    {
        ArgumentNullException.ThrowIfNull(system);
        var nx = system.StateCount;
        var nu = system.InputCount;
        if (gain.Length != nu * nx)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"K must have {nu * nx} entries ({nu} × {nx}), got {gain.Length}."), nameof(gain));
        foreach (var v in gain)
            if (!double.IsFinite(v)) throw new ArgumentException("K must be finite.", nameof(gain));

        var (phi, gamma) = Discretise(system, samplePeriod, substeps);
        for (var r = 0; r < nx; r++)
            for (var c = 0; c < nx; c++)
            {
                var sum = 0.0;
                for (var k = 0; k < nu; k++) sum += gamma[r * nu + k] * gain[k * nx + c];
                phi[r * nx + c] -= sum;
            }
        return phi;
    }

    /// <summary>
    /// ρ(Φ − ΓK), the spectral radius of the sampled closed loop: below 1 means stable. SPEC §6.3; docs/DECISIONS.md D6.
    /// </summary>
    public static double SpectralRadius(LinearSystem system, ReadOnlySpan<double> gain, double samplePeriod, int substeps = DefaultSubsteps) =>
        SpectralRadius(ClosedLoop(system, gain, samplePeriod, substeps), system.StateCount);

    /// <summary>
    /// Spectral radius of the n × n row-major <paramref name="matrix"/> by Gelfand's formula
    /// ρ = lim ‖P^m‖^{1/m}, m = 2^k, without an eigen solver. docs/DECISIONS.md D6.
    /// </summary>
    /// <remarks>
    /// <para>
    /// P^(2^k) is kept as e^{L_k}·B_k with ‖B_k‖∞ = 1: B_{k+1} = B_k²/s and L_{k+1} = 2 L_k + ln s with s = ‖B_k²‖∞,
    /// so nothing overflows or underflows, and ln ρ ≈ L_k / 2^k. The error of that estimate is about ln(C)/2^k,
    /// where C depends on how far P is from normal (C = 1 for a symmetric P). After 12 squarings it is therefore
    /// still about 1e-4 for a typical P; the loop goes on until two estimates differ by less than 1e-14 or
    /// <see cref="MaxSquarings"/> squarings are done, which gives about 1e-13. Rounding adds only about ε·cond to
    /// ln ρ, because taking the 2^k-th root divides the accumulated error by 2^k.
    /// </para>
    /// <para>Returns 0 for a nilpotent matrix (some power is exactly zero).</para>
    /// </remarks>
    /// <param name="matrix">n × n, row-major.</param>
    /// <param name="n">Size.</param>
    /// <param name="maxSquarings">Largest number of squarings k, between 1 and <see cref="MaxSquarings"/>.</param>
    /// <exception cref="ArgumentException">The matrix does not have n × n entries, or an entry is not finite.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="maxSquarings"/> is out of range.</exception>
    public static double SpectralRadius(ReadOnlySpan<double> matrix, int n, int maxSquarings = MaxSquarings)
    {
        if (maxSquarings < 1 || maxSquarings > MaxSquarings)
            throw new ArgumentOutOfRangeException(nameof(maxSquarings), maxSquarings, "Between 1 and 60 squarings.");
        if (n <= 0 || matrix.Length != n * n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"Expected {n} × {n} entries, got {matrix.Length}."), nameof(matrix));
        foreach (var v in matrix)
            if (!double.IsFinite(v)) throw new ArgumentException("The matrix must be finite.", nameof(matrix));

        var p = matrix.ToArray();
        var square = new double[n * n];
        var norm = InfinityNorm(p, n);
        if (norm == 0.0)
            return 0.0;
        Scale(p, 1.0 / norm);
        var logNorm = Math.Log(norm);   // L_k
        var power = 1.0;                // 2^k
        var estimate = logNorm;

        for (var k = 0; k < maxSquarings; k++)
        {
            Multiply(p, p, square, n);
            var s = InfinityNorm(square, n);
            if (s == 0.0)
                return 0.0;
            Scale(square, 1.0 / s);
            (p, square) = (square, p);
            logNorm = 2.0 * logNorm + Math.Log(s);
            power *= 2.0;
            var next = logNorm / power;
            if (Math.Abs(next - estimate) < 1e-14)
                return Math.Exp(next);
            estimate = next;
        }
        return Math.Exp(estimate);
    }

    /// <summary>
    /// The largest Ts for which the sampled loop is stable (ρ &lt; 1), by bisection. SPEC §6.3; docs/DECISIONS.md D6.
    /// </summary>
    /// <remarks>
    /// ρ is evaluated from Ts = <paramref name="startPeriod"/> upward, doubling until ρ ≥ 1, then the bracket is
    /// bisected until its width is below <paramref name="relativeTolerance"/> times its lower end, and the lower
    /// (stable) end is returned. If ρ ≥ 1 already at the start period, 0 is returned (the loop is unstable even
    /// with fast sampling, so the continuous closed loop is unstable). If ρ &lt; 1 up to
    /// <paramref name="maxPeriod"/>, +∞ is returned. Between the doubling points the first crossing is found;
    /// ρ(Ts) need not be monotonic beyond it.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">A period or the tolerance is not positive and finite, or start ≥ max.</exception>
    public static double MaxStableSamplePeriod(
        LinearSystem system, ReadOnlySpan<double> gain,
        double startPeriod = 1e-4, double maxPeriod = 1.0, double relativeTolerance = 1e-4, int substeps = DefaultSubsteps)
    {
        ArgumentNullException.ThrowIfNull(system);
        var k = gain.ToArray();
        return FirstUnstablePeriod(ts => SpectralRadius(system, k, ts, substeps) < 1.0, startPeriod, maxPeriod, relativeTolerance);
    }

    /// <summary>
    /// ρ of the sampled loop of <paramref name="controller"/> linearised at rest at <paramref name="q0"/>, with K
    /// from <see cref="ILinearisableController.LinearFeedbackGain"/> at this Ts. SPEC §6.3, §7; docs/DECISIONS.md D6.
    /// </summary>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    public static double SpectralRadius(
        LinearSystem system, ILinearisableController controller, ReadOnlySpan<double> q0, double samplePeriod,
        int substeps = DefaultSubsteps)
    {
        ArgumentNullException.ThrowIfNull(system);
        ArgumentNullException.ThrowIfNull(controller);
        var k = new double[system.InputCount * system.StateCount];
        controller.LinearFeedbackGain(q0, samplePeriod, k);
        return SpectralRadius(system, k, samplePeriod, substeps);
    }

    /// <summary>
    /// The largest stable Ts of <paramref name="controller"/> linearised at rest at <paramref name="q0"/>, with K
    /// evaluated anew at every Ts tried (it depends on Ts for computed torque with half-sample prediction).
    /// Same search as <see cref="MaxStableSamplePeriod(LinearSystem, ReadOnlySpan{double}, double, double, double, int)"/>.
    /// SPEC §6.3, §7; docs/DECISIONS.md D6.
    /// </summary>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    public static double MaxStableSamplePeriod(
        LinearSystem system, ILinearisableController controller, ReadOnlySpan<double> q0,
        double startPeriod = 1e-4, double maxPeriod = 1.0, double relativeTolerance = 1e-4, int substeps = DefaultSubsteps)
    {
        ArgumentNullException.ThrowIfNull(system);
        ArgumentNullException.ThrowIfNull(controller);
        var pose = q0.ToArray();
        var k = new double[system.InputCount * system.StateCount];
        return FirstUnstablePeriod(ts =>
        {
            controller.LinearFeedbackGain(pose, ts, k);
            return SpectralRadius(system, k, ts, substeps) < 1.0;
        }, startPeriod, maxPeriod, relativeTolerance);
    }

    private static double FirstUnstablePeriod(Func<double, bool> stable, double startPeriod, double maxPeriod, double relativeTolerance)
    {
        if (!double.IsFinite(startPeriod) || startPeriod <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(startPeriod), startPeriod, "Must be a finite number greater than 0 s.");
        if (!double.IsFinite(maxPeriod) || maxPeriod <= startPeriod)
            throw new ArgumentOutOfRangeException(nameof(maxPeriod), maxPeriod, "Must be finite and greater than the start period.");
        if (!double.IsFinite(relativeTolerance) || relativeTolerance <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(relativeTolerance), relativeTolerance, "Must be a finite number greater than 0.");

        if (!stable(startPeriod))
            return 0.0;
        var lo = startPeriod;
        var hi = 2.0 * startPeriod;
        while (stable(hi))
        {
            if (hi >= maxPeriod)
                return double.PositiveInfinity;
            lo = hi;
            hi = Math.Min(2.0 * hi, maxPeriod);
        }
        while (hi - lo > relativeTolerance * lo)
        {
            var mid = 0.5 * (lo + hi);
            if (stable(mid)) lo = mid;
            else hi = mid;
        }
        return lo;
    }

    private static double InfinityNorm(double[] m, int n)
    {
        var norm = 0.0;
        for (var r = 0; r < n; r++)
        {
            var row = 0.0;
            for (var c = 0; c < n; c++) row += Math.Abs(m[r * n + c]);
            norm = Math.Max(norm, row);
        }
        return norm;
    }

    private static void Scale(double[] m, double factor)
    {
        for (var i = 0; i < m.Length; i++) m[i] *= factor;
    }

    private static void Multiply(double[] left, double[] right, double[] result, int n)
    {
        for (var r = 0; r < n; r++)
            for (var c = 0; c < n; c++)
            {
                var sum = 0.0;
                for (var k = 0; k < n; k++) sum += left[r * n + k] * right[k * n + c];
                result[r * n + c] = sum;
            }
    }

    /// <summary>RK4 for ẋ = A x + b with b constant (the held input), in place.</summary>
    private sealed class Rk4Work(int n)
    {
        private readonly double[] _k1 = new double[n];
        private readonly double[] _k2 = new double[n];
        private readonly double[] _k3 = new double[n];
        private readonly double[] _k4 = new double[n];
        private readonly double[] _stage = new double[n];

        public void Step(ReadOnlySpan<double> a, int nx, double[] b, double[] x, double h)
        {
            Derivative(a, nx, b, x, _k1);
            for (var i = 0; i < nx; i++) _stage[i] = x[i] + 0.5 * h * _k1[i];
            Derivative(a, nx, b, _stage, _k2);
            for (var i = 0; i < nx; i++) _stage[i] = x[i] + 0.5 * h * _k2[i];
            Derivative(a, nx, b, _stage, _k3);
            for (var i = 0; i < nx; i++) _stage[i] = x[i] + h * _k3[i];
            Derivative(a, nx, b, _stage, _k4);
            for (var i = 0; i < nx; i++) x[i] += h / 6.0 * (_k1[i] + 2.0 * _k2[i] + 2.0 * _k3[i] + _k4[i]);
        }

        private static void Derivative(ReadOnlySpan<double> a, int nx, double[] b, double[] x, double[] dx)
        {
            for (var r = 0; r < nx; r++)
            {
                var sum = b[r];
                for (var c = 0; c < nx; c++) sum += a[r * nx + c] * x[c];
                dx[r] = sum;
            }
        }
    }
}
