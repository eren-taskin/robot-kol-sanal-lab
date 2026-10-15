using System.Globalization;
using System.Numerics;
using RobotArm.Core.Control;
using RobotArm.Core.Model;

namespace RobotArm.Core.Analysis;

/// <summary>Which gain a root locus varies. SPEC §12.3.</summary>
public enum LocusGain
{
    /// <summary>A common factor k on every gain of every joint (Kp, Kd and Ki); the nominal law is k = 1.</summary>
    Scale,
    /// <summary>Kp of one joint, in the controller's own unit.</summary>
    Kp,
    /// <summary>Kd of one joint.</summary>
    Kd,
    /// <summary>Ki of one joint (PID, CT+I).</summary>
    Ki,
}

/// <summary>The parameter of a root locus: a common scale factor, or one gain of one joint (zero-based). SPEC §12.3.</summary>
/// <param name="Gain">Which gain.</param>
/// <param name="Joint">Zero-based joint; ignored for <see cref="LocusGain.Scale"/>.</param>
public readonly record struct LocusParameter(LocusGain Gain, int Joint)
{
    /// <summary>The common scale factor k on all gains.</summary>
    public static LocusParameter ScaleAll => new(LocusGain.Scale, -1);
}

/// <summary>Settings of a root-locus sweep. SPEC §12.3.</summary>
public sealed record RootLocusOptions
{
    /// <summary>Points of the initial logarithmic grid (at least 2).</summary>
    public int InitialPoints { get; init; } = 41;

    /// <summary>Largest number of points after refinement.</summary>
    public int MaxPoints { get; init; } = 400;

    /// <summary>
    /// A step is refined (its geometric midpoint inserted) while a pole moves more than this in the z-plane, relative
    /// to max(1, |z|) so that poles far outside the unit circle do not use up the points.
    /// </summary>
    public double MaxPoleStep { get; init; } = 0.05;

    /// <summary>Relative width at which the bisection for a crossing of |z| = 1 stops.</summary>
    public double RelativeTolerance { get; init; } = 1e-10;
}

/// <summary>A parameter value at which the largest |z| crosses 1. SPEC §12.3.</summary>
/// <param name="Value">The parameter value.</param>
/// <param name="BecomesUnstable">True when the loop is stable just below the value and unstable above it.</param>
public readonly record struct LocusCrossing(double Value, bool BecomesUnstable);

/// <summary>Result of a root-locus sweep. SPEC §12.3.</summary>
/// <param name="Values">Parameter values, increasing.</param>
/// <param name="Tracks">Tracks[j][i]: pole j at Values[i]; each track is continuous from one value to the next.</param>
/// <param name="SpectralRadii">ρ = max |z| at each value.</param>
/// <param name="Crossings">Every value in the range where ρ crosses 1, found by bisection.</param>
public sealed record RootLocusResult(IReadOnlyList<double> Values, IReadOnlyList<IReadOnlyList<Complex>> Tracks,
    IReadOnlyList<double> SpectralRadii, IReadOnlyList<LocusCrossing> Crossings)
{
    /// <summary>The first value, going up, at which the stable loop becomes unstable; null if none in the range.</summary>
    public double? CriticalValue => Crossings.Where(c => c.BecomesUnstable).Select(c => (double?)c.Value).FirstOrDefault();
}

/// <summary>
/// Root locus of a sampled closed loop: the poles as one parameter goes over a logarithmic grid. SPEC §12.3;
/// docs/DECISIONS.md D9.
/// </summary>
/// <remarks>
/// <para>
/// The grid starts with <see cref="RootLocusOptions.InitialPoints"/> logarithmically spaced values. A step is then
/// split at its geometric midpoint while some pole moves more than <see cref="RootLocusOptions.MaxPoleStep"/>
/// (relative to max(1, |z|)) between its ends, up to <see cref="RootLocusOptions.MaxPoints"/> points. Poles are matched from one value to the
/// next by the nearest-pair rule (all pairs sorted by distance, the closest unmatched pair taken first); with the
/// refined steps a pole moves much less than the distance between poles, so the tracks are continuous.
/// </para>
/// <para>
/// Each sign change of ρ − 1 between neighbouring values is bisected on log(value). A crossing that enters and
/// leaves the unit circle between two grid values (a tangent touch) can be missed; the refinement makes that unlikely.
/// The linearisation is that of <see cref="SampledLoopModel"/>.
/// </para>
/// </remarks>
public static class RootLocus
{
    /// <summary>
    /// The locus of the N × N closed-loop matrix returned by <paramref name="closedLoop"/> for each parameter value
    /// in [<paramref name="min"/>, <paramref name="max"/>]. SPEC §12.3.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The range is not 0 &lt; min &lt; max, finite, or the options are invalid.</exception>
    public static RootLocusResult Sweep(Func<double, double[]> closedLoop, int n, double min, double max, RootLocusOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(closedLoop);
        options ??= new RootLocusOptions();
        if (!double.IsFinite(min) || min <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(min), min, "The range must start above 0.");
        if (!double.IsFinite(max) || max <= min)
            throw new ArgumentOutOfRangeException(nameof(max), max, "The range must end above its start.");
        if (options.InitialPoints < 2 || options.MaxPoints < options.InitialPoints)
            throw new ArgumentOutOfRangeException(nameof(options), "Need at least 2 initial points and MaxPoints ≥ InitialPoints.");
        if (!(options.MaxPoleStep > 0.0) || !(options.RelativeTolerance > 0.0))
            throw new ArgumentOutOfRangeException(nameof(options), "MaxPoleStep and RelativeTolerance must be greater than 0.");

        Complex[] Poles(double v) => EigenSolver.Eigenvalues(closedLoop(v), n);

        var values = new List<double>();
        var poles = new List<Complex[]>();
        for (var i = 0; i < options.InitialPoints; i++)
        {
            var v = i == options.InitialPoints - 1 ? max : min * Math.Pow(max / min, (double)i / (options.InitialPoints - 1));
            values.Add(v);
            poles.Add(Poles(v));
        }

        // Adaptive refinement: split steps in which a pole jumps.
        var index = 0;
        while (index < values.Count - 1 && values.Count < options.MaxPoints)
        {
            var (lo, hi) = (values[index], values[index + 1]);
            if (MaxDisplacement(poles[index], poles[index + 1]) > options.MaxPoleStep && hi / lo > 1.0 + 1e-12)
            {
                var mid = Math.Sqrt(lo * hi);
                values.Insert(index + 1, mid);
                poles.Insert(index + 1, Poles(mid));
            }
            else
            {
                index++;
            }
        }

        // Tracks: order the poles at each value by matching them to the previous value.
        var ordered = new Complex[values.Count][];
        ordered[0] = poles[0];
        for (var i = 1; i < values.Count; i++) ordered[i] = Match(ordered[i - 1], poles[i]);
        var tracks = new IReadOnlyList<Complex>[n];
        for (var j = 0; j < n; j++)
        {
            var track = new Complex[values.Count];
            for (var i = 0; i < values.Count; i++) track[i] = ordered[i][j];
            tracks[j] = Array.AsReadOnly(track);
        }
        var radii = poles.Select(Radius).ToArray();

        var crossings = new List<LocusCrossing>();
        for (var i = 0; i < values.Count - 1; i++)
        {
            var stableLo = radii[i] < 1.0;
            if (stableLo == radii[i + 1] < 1.0) continue;
            var (lo, hi) = (values[i], values[i + 1]);
            while (hi / lo - 1.0 > options.RelativeTolerance)
            {
                var mid = Math.Sqrt(lo * hi);
                if (Radius(Poles(mid)) < 1.0 == stableLo) lo = mid;
                else hi = mid;
            }
            crossings.Add(new LocusCrossing(Math.Sqrt(lo * hi), stableLo));
        }

        return new RootLocusResult(values.AsReadOnly(), Array.AsReadOnly(tracks), Array.AsReadOnly(radii), crossings.AsReadOnly());
    }

    /// <summary>
    /// The locus of the arm's sampled loop under <paramref name="controller"/> linearised at rest at
    /// <paramref name="q0"/>, varying <paramref name="parameter"/> from <paramref name="min"/> to <paramref name="max"/>.
    /// Φ and Γ are computed once; each value builds a controller with the same belief and the changed gain.
    /// SPEC §12.3. For <see cref="LocusGain.Scale"/> the values are factors on the controller's gains.
    /// </summary>
    /// <exception cref="ArgumentException">The joint is out of range, or Ki is varied on a law without an integral.</exception>
    /// <exception cref="NotSupportedException">The controller is not PID, PDG or CT.</exception>
    public static RootLocusResult Sweep(ArmModel arm, Planet planet, double payloadKg, ReadOnlySpan<double> q0,
        ILinearisableController controller, double samplePeriod, LocusParameter parameter, double min, double max,
        RootLocusOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(controller);
        var plant = LinearSystem.ArmAtRest(arm, planet, payloadKg, q0);
        var (phi, gamma) = SampledLoopStability.Discretise(plant, samplePeriod);
        var pose = q0.ToArray();
        var probe = SampledLoopModel.ForController(phi, gamma, plant.StateCount, plant.InputCount, pose,
            ControllerGains.With(controller, parameter, 1.0), samplePeriod);
        return Sweep(v => SampledLoopModel.ForController(phi, gamma, plant.StateCount, plant.InputCount, pose,
            ControllerGains.With(controller, parameter, v), samplePeriod).ClosedLoopMatrix(), probe.StateCount, min, max, options);
    }

    private static double Radius(Complex[] poles)
    {
        var r = 0.0;
        foreach (var z in poles) r = Math.Max(r, Complex.Abs(z));
        return r;
    }

    private static double MaxDisplacement(Complex[] previous, Complex[] next)
    {
        var matched = Match(previous, next);
        var d = 0.0;
        for (var j = 0; j < previous.Length; j++)
            d = Math.Max(d, Complex.Abs(matched[j] - previous[j]) / Math.Max(1.0, Math.Max(Complex.Abs(matched[j]), Complex.Abs(previous[j]))));
        return d;
    }

    /// <summary>Reorders <paramref name="next"/> so that next[j] continues previous[j] (nearest pairs first).</summary>
    internal static Complex[] Match(Complex[] previous, Complex[] next)
    {
        var n = previous.Length;
        var pairs = new (double Distance, int From, int To)[n * n];
        for (var i = 0; i < n; i++)
            for (var j = 0; j < n; j++) pairs[i * n + j] = (Complex.Abs(previous[i] - next[j]), i, j);
        Array.Sort(pairs, (a, b) => a.Distance != b.Distance ? a.Distance.CompareTo(b.Distance)
            : a.From != b.From ? a.From.CompareTo(b.From) : a.To.CompareTo(b.To));
        var result = new Complex[n];
        var fromUsed = new bool[n];
        var toUsed = new bool[n];
        var left = n;
        foreach (var (_, from, to) in pairs)
        {
            if (fromUsed[from] || toUsed[to]) continue;
            result[from] = next[to];
            fromUsed[from] = toUsed[to] = true;
            if (--left == 0) break;
        }
        return result;
    }
}

/// <summary>Builds a controller like a given one (same belief and options) with changed gains. SPEC §12.3, §12.7.</summary>
internal static class ControllerGains
{
    /// <summary>
    /// For <see cref="LocusGain.Scale"/>: every gain times <paramref name="value"/>; otherwise the chosen gain of the
    /// chosen joint set to <paramref name="value"/>.
    /// </summary>
    public static ILinearisableController With(ILinearisableController controller, LocusParameter parameter, double value)
    {
        var (kp, kd, ki) = Get(controller);
        var n = kp.Length;
        if (parameter.Gain == LocusGain.Scale)
        {
            for (var i = 0; i < n; i++)
            {
                kp[i] *= value;
                kd[i] *= value;
                if (ki is not null) ki[i] *= value;
            }
        }
        else
        {
            if (parameter.Joint < 0 || parameter.Joint >= n)
                throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Joint must be 0 to {n - 1}."), nameof(parameter));
            var target = parameter.Gain switch
            {
                LocusGain.Kp => kp,
                LocusGain.Kd => kd,
                LocusGain.Ki => ki ?? throw new ArgumentException("This law has no integral gain.", nameof(parameter)),
                _ => throw new ArgumentOutOfRangeException(nameof(parameter)),
            };
            target[parameter.Joint] = value;
        }
        return Build(controller, kp, kd, ki);
    }

    /// <summary>Copies of Kp, Kd and Ki (null for a law without an integral).</summary>
    public static (double[] Kp, double[] Kd, double[]? Ki) Get(ILinearisableController controller) => controller switch
    {
        Pid pid => ([.. pid.Gains.Kp], [.. pid.Gains.Kd], [.. pid.Gains.Ki]),
        PdGravity pdg => ([.. pdg.Gains.Kp], [.. pdg.Gains.Kd], null),
        ComputedTorque ct => ([.. ct.Gains.Kp], [.. ct.Gains.Kd], ct.IntegralEnabled ? [.. ct.IntegralGains] : null),
        null => throw new ArgumentNullException(nameof(controller)),
        _ => throw new NotSupportedException("Gains can be changed for PID, PDG and CT only."),
    };

    /// <summary>A controller of the same kind, belief and options with the given gains.</summary>
    public static ILinearisableController Build(ILinearisableController controller, double[] kp, double[] kd, double[]? ki) => controller switch
    {
        Pid pid => new Pid(pid.Belief, new PidGains(kp, ki!, kd)),
        PdGravity pdg => new PdGravity(pdg.Belief, new PdGains(kp, kd)),
        ComputedTorque ct => new ComputedTorque(ct.Belief, new PdGains(kp, kd), ct.PredictHalfSample, ct.IntegralEnabled ? ki : null),
        _ => throw new NotSupportedException("Gains can be changed for PID, PDG and CT only."),
    };
}
