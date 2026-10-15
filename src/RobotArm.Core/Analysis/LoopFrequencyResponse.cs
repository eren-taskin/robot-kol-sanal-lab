using System.Globalization;
using System.Numerics;
using RobotArm.Core.Control;
using RobotArm.Core.Model;

namespace RobotArm.Core.Analysis;

/// <summary>
/// Frequency response of one loop of the sampled arm, opened at its input with the other loops closed, and its
/// margins. SPEC §12.4.
/// </summary>
/// <param name="Joint">Zero-based joint whose loop is opened.</param>
/// <param name="Frequencies">ω, rad/s, logarithmic up to the Nyquist frequency π/Ts (included).</param>
/// <param name="MagnitudeDb">20·log10 |L_i(e^{jωTs})|.</param>
/// <param name="PhaseDeg">∠L_i, degrees, unwrapped from the principal value at the lowest frequency.</param>
/// <param name="GainMarginDb">−20·log10 |L_i| at the phase crossover with the smallest |margin|; +∞ if the phase never reaches an odd multiple of −180°.</param>
/// <param name="PhaseCrossoverFrequency">ω of that phase crossover, rad/s; NaN if none.</param>
/// <param name="PhaseMarginDeg">180° + ∠L_i (wrapped to (−180°, 180°]) at the gain crossover with the smallest margin; +∞ if |L_i| never crosses 1.</param>
/// <param name="GainCrossoverFrequency">ω of that gain crossover, rad/s; NaN if none.</param>
/// <param name="PhaseCrossovers">All phase crossover frequencies in the range, rad/s.</param>
/// <param name="GainCrossovers">All gain crossover frequencies in the range, rad/s.</param>
/// <param name="OpenLoopUnstablePoles">
/// Poles of the loop with input i opened that lie outside the unit circle. When this is not 0 the Bode margins do not
/// decide stability alone (the Nyquist criterion needs that many encirclements).
/// </param>
public sealed record LoopResponse(
    int Joint, IReadOnlyList<double> Frequencies, IReadOnlyList<double> MagnitudeDb, IReadOnlyList<double> PhaseDeg,
    double GainMarginDb, double PhaseCrossoverFrequency, double PhaseMarginDeg, double GainCrossoverFrequency,
    IReadOnlyList<double> PhaseCrossovers, IReadOnlyList<double> GainCrossovers, int OpenLoopUnstablePoles);

/// <summary>
/// Loop-at-a-time frequency response L_i(e^{jωTs}) of a sampled closed loop. SPEC §12.4; docs/DECISIONS.md D9.
/// </summary>
/// <remarks>
/// <para>
/// The loop is broken at input i: u_i becomes an external input v while every other input keeps its feedback, and
/// the return is what the controller would command on that input, −(−K_ξ,i ξ). With A_i = A_ξ − Σ_{j≠i} b_j k_j
/// (b_j the columns of B_ξ, k_j the rows of K_ξ of <see cref="SampledLoopModel"/>),
/// <code>
/// L_i(z) = k_i (z I − A_i)⁻¹ b_i,   z = e^{jωTs},
/// </code>
/// and the closed loop is 1 + L_i in negative feedback. Scaling L_i by k scales every gain of joint i's row of K_ξ
/// (including its integral and the controller's gravity and inertia terms) by k, so the gain margin is the factor on
/// that whole row; for a single-loop system it equals 20·log10 of the critical scale factor of the root locus.
/// </para>
/// <para>
/// Loop-at-a-time margins do not cover simultaneous changes in several loops (the coupling through M(q)⁻¹ and G).
/// The linearisation is that of <see cref="SampledLoopModel"/>.
/// </para>
/// </remarks>
public static class LoopFrequencyResponse
{
    /// <summary>Default number of logarithmic frequency points.</summary>
    public const int DefaultPoints = 400;

    /// <summary>
    /// L_i on <paramref name="points"/> logarithmic frequencies from <paramref name="minFrequency"/> (default
    /// 1e-4 times the Nyquist frequency) to π/Ts, with gain and phase margins and their crossover frequencies, each
    /// crossover located by bisection. SPEC §12.4.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The joint, the number of points or the minimum frequency is invalid.</exception>
    public static LoopResponse Compute(SampledLoopModel model, int joint, int points = DefaultPoints, double? minFrequency = null)
    {
        ArgumentNullException.ThrowIfNull(model);
        var nu = model.InputCount;
        if (joint < 0 || joint >= nu)
            throw new ArgumentOutOfRangeException(nameof(joint), joint, string.Create(CultureInfo.InvariantCulture, $"Joint must be 0 to {nu - 1}."));
        if (points < 2)
            throw new ArgumentOutOfRangeException(nameof(points), points, "At least 2 points.");
        var ts = model.SamplePeriod;
        var nyquist = Math.PI / ts;
        var wMin = minFrequency ?? 1e-4 * nyquist;
        if (!double.IsFinite(wMin) || wMin <= 0.0 || wMin >= nyquist)
            throw new ArgumentOutOfRangeException(nameof(minFrequency), wMin, "Must be above 0 and below π/Ts.");

        var loop = new OpenedLoop(model, joint);
        var w = new double[points];
        var mag = new double[points];
        var phase = new double[points];
        for (var p = 0; p < points; p++)
        {
            w[p] = p == points - 1 ? nyquist : wMin * Math.Pow(nyquist / wMin, (double)p / (points - 1));
            var l = loop.Evaluate(w[p]);
            mag[p] = 20.0 * Math.Log10(Complex.Abs(l));
            var raw = Degrees(l);
            phase[p] = p == 0 ? raw : phase[p - 1] + Wrap(raw - phase[p - 1]);
        }

        // Phase crossovers: the unwrapped phase passes an odd multiple of 180°.
        var phaseCrossovers = new List<double>();
        for (var p = 0; p < points - 1; p++)
        {
            var (a, b) = (phase[p], phase[p + 1]);
            var kLo = (int)Math.Ceiling((Math.Min(a, b) - 180.0) / 360.0) - 1;
            var kHi = (int)Math.Floor((Math.Max(a, b) - 180.0) / 360.0) + 1;
            for (var k = kLo; k <= kHi; k++)
            {
                var c = 180.0 + 360.0 * k;
                if (p == points - 2 && Math.Abs(b - c) < 1e-7)
                {
                    phaseCrossovers.Add(nyquist); // at z = −1 the loop is real
                    continue;
                }
                if ((a - c) * (b - c) >= 0.0) continue;
                phaseCrossovers.Add(Bisect(w[p], w[p + 1], x => a + Wrap(Degrees(loop.Evaluate(x)) - a) - c, a - c));
            }
        }

        // Gain crossovers: |L| passes 1.
        var gainCrossovers = new List<double>();
        for (var p = 0; p < points - 1; p++)
        {
            if (mag[p + 1] == 0.0) gainCrossovers.Add(w[p + 1]);
            else if (mag[p] * mag[p + 1] < 0.0)
                gainCrossovers.Add(Bisect(w[p], w[p + 1], x => Complex.Abs(loop.Evaluate(x)) - 1.0, mag[p]));
        }

        var gm = double.PositiveInfinity;
        var wpc = double.NaN;
        foreach (var x in phaseCrossovers)
        {
            var margin = -20.0 * Math.Log10(Complex.Abs(loop.Evaluate(x)));
            if (Math.Abs(margin) < Math.Abs(gm)) (gm, wpc) = (margin, x);
        }

        var pm = double.PositiveInfinity;
        var wgc = double.NaN;
        foreach (var x in gainCrossovers)
        {
            var margin = Wrap(180.0 + Degrees(loop.Evaluate(x)));
            if (margin < pm) (pm, wgc) = (margin, x);
        }

        return new LoopResponse(joint, Array.AsReadOnly(w), Array.AsReadOnly(mag), Array.AsReadOnly(phase),
            gm, wpc, pm, wgc, phaseCrossovers.AsReadOnly(), gainCrossovers.AsReadOnly(), loop.UnstablePoles());
    }

    /// <summary>
    /// L_i of the arm's sampled loop under <paramref name="controller"/> linearised at rest at <paramref name="q0"/>.
    /// SPEC §12.4.
    /// </summary>
    public static LoopResponse Compute(ArmModel arm, Planet planet, double payloadKg, ReadOnlySpan<double> q0,
        ILinearisableController controller, double samplePeriod, int joint, int points = DefaultPoints, double? minFrequency = null) =>
        Compute(SampledLoopModel.ForArm(arm, planet, payloadKg, q0, controller, samplePeriod), joint, points, minFrequency);

    /// <summary>Margins of every joint's loop (loop at a time), with the default grid. SPEC §12.4.</summary>
    public static IReadOnlyList<LoopResponse> ComputeAll(SampledLoopModel model, int points = DefaultPoints)
    {
        ArgumentNullException.ThrowIfNull(model);
        var all = new LoopResponse[model.InputCount];
        for (var i = 0; i < all.Length; i++) all[i] = Compute(model, i, points);
        return Array.AsReadOnly(all);
    }

    /// <summary>The value L_i(e^{jωTs}) at one frequency. SPEC §12.4.</summary>
    public static Complex Evaluate(SampledLoopModel model, int joint, double frequency) =>
        new OpenedLoop(model, joint).Evaluate(frequency);

    private static double Degrees(Complex z) => Math.Atan2(z.Imaginary, z.Real) * 180.0 / Math.PI;

    /// <summary>Wraps an angle in degrees to (−180, 180].</summary>
    private static double Wrap(double deg)
    {
        var r = Math.IEEERemainder(deg, 360.0);
        return r <= -180.0 ? r + 360.0 : r;
    }

    /// <summary>Bisection on log ω for a sign change of f; <paramref name="fLo"/> is f at the lower end.</summary>
    private static double Bisect(double lo, double hi, Func<double, double> f, double fLo)
    {
        for (var it = 0; it < 100 && hi / lo - 1.0 > 1e-13; it++)
        {
            var mid = Math.Sqrt(lo * hi);
            var fm = f(mid);
            if (fm == 0.0) return mid;
            if (fm * fLo > 0.0) { lo = mid; fLo = fm; }
            else hi = mid;
        }
        return Math.Sqrt(lo * hi);
    }

    /// <summary>A_i, b_i and k_i of the loop opened at one input.</summary>
    private sealed class OpenedLoop
    {
        private readonly int _n;
        private readonly double _ts;
        private readonly double[] _a;
        private readonly double[] _b;
        private readonly double[] _k;

        public OpenedLoop(SampledLoopModel model, int joint)
        {
            _n = model.StateCount;
            _ts = model.SamplePeriod;
            var nu = model.InputCount;
            var a = model.OpenLoopMatrix();
            var b = model.InputMatrix();
            var k = model.AugmentedFeedbackGain();
            for (var j = 0; j < nu; j++)
            {
                if (j == joint) continue;
                for (var r = 0; r < _n; r++)
                    for (var c = 0; c < _n; c++) a[r * _n + c] -= b[r * nu + j] * k[j * _n + c];
            }
            _a = a;
            _b = new double[_n];
            _k = new double[_n];
            for (var r = 0; r < _n; r++)
            {
                _b[r] = b[r * nu + joint];
                _k[r] = k[joint * _n + r];
            }
        }

        public Complex Evaluate(double frequency)
        {
            var z = Complex.FromPolarCoordinates(1.0, frequency * _ts);
            var m = new Complex[_n * _n];
            var rhs = new Complex[_n];
            for (var r = 0; r < _n; r++)
            {
                for (var c = 0; c < _n; c++) m[r * _n + c] = -_a[r * _n + c];
                m[r * _n + r] += z;
                rhs[r] = _b[r];
            }
            var x = DenseMatrix.Solve(m, rhs, _n);
            var sum = Complex.Zero;
            for (var r = 0; r < _n; r++) sum += _k[r] * x[r];
            return sum;
        }

        public int UnstablePoles() => EigenSolver.Eigenvalues(_a, _n).Count(z => Complex.Abs(z) > 1.0 + 1e-9);
    }
}
