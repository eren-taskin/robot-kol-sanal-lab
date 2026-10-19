using RobotArm.Core.Control;
using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using RobotArm.Core.Tasks;

namespace RobotArm.Core.Analysis;

/// <summary>Whether the tuner searches one (ωn, ζ) per joint or one pair shared by all joints. SPEC §12.7.</summary>
public enum TuningMode
{
    /// <summary>(ωn, ζ) per joint: 8 parameters for the arm.</summary>
    PerJoint,
    /// <summary>One (ωn, ζ) for every joint: 2 parameters.</summary>
    Shared,
}

/// <summary>How the tuner combines per-run costs across scenarios and sensors. SPEC §12.7.</summary>
public enum TuningCostAggregation
{
    /// <summary>Sum of the per-run costs.</summary>
    Sum,
    /// <summary>Largest (worst-case) per-run cost.</summary>
    Max,
}

/// <summary>Whether each scenario is evaluated with its own sensor or with both perfect and encoder sensors. SPEC §12.7.</summary>
public enum TuningSensorMode
{
    /// <summary>Use each scenario's <see cref="Scenario.UseEncoder"/> as configured.</summary>
    Scenario,
    /// <summary>Evaluate each scenario twice: once with the perfect sensor and once with the encoder.</summary>
    PerfectAndEncoder,
}

/// <summary>Search space, budget, robustness options and constraints of the gain tuner. SPEC §12.7.</summary>
public sealed record GainTunerOptions
{
    /// <summary>Per-joint or shared parameters.</summary>
    public TuningMode Mode { get; init; } = TuningMode.PerJoint;

    /// <summary>How costs across scenarios and sensors are combined (sum or max).</summary>
    public TuningCostAggregation CostAggregation { get; init; } = TuningCostAggregation.Sum;

    /// <summary>Sensor evaluation mode (scenario's own sensor, or both perfect and encoder).</summary>
    public TuningSensorMode Sensor { get; init; } = TuningSensorMode.Scenario;

    /// <summary>Lower bound of ωn, rad/s.</summary>
    public double MinNaturalFrequency { get; init; } = 1.0;

    /// <summary>Upper bound of ωn, rad/s.</summary>
    public double MaxNaturalFrequency { get; init; } = 300.0;

    /// <summary>Lower bound of ζ.</summary>
    public double MinDampingRatio { get; init; } = 0.2;

    /// <summary>Upper bound of ζ.</summary>
    public double MaxDampingRatio { get; init; } = 10.0;

    /// <summary>Largest number of candidate evaluations, the baseline included.</summary>
    public int MaxEvaluations { get; init; } = 60;

    /// <summary>Seed of the generator that spreads the initial simplex.</summary>
    public int Seed { get; init; } = 445;

    /// <summary>Edge of the initial simplex in the normalised parameter box [0, 1]^d.</summary>
    public double InitialStep { get; init; } = 0.1;

    /// <summary>Added when a run ends in a fault (SPEC §6.4).</summary>
    public double FaultPenalty { get; init; } = 10.0;

    /// <summary>Percentage of time with any controller clamping allowed without penalty.</summary>
    public double ClampLimitPercent { get; init; } = 5.0;

    /// <summary>Penalty per percentage point of clamping above <see cref="ClampLimitPercent"/>.</summary>
    public double ClampPenalty { get; init; } = 1e-3;

    /// <summary>Added times ρ when the linearised loop at a start pose has ρ ≥ 1.</summary>
    public double SpectralRadiusPenalty { get; init; } = 10.0;

    /// <summary>Minimum loop-at-a-time gain margin required as a hard constraint at each scenario's start pose, dB.</summary>
    public double GainMarginLimitDb { get; init; } = 6.0;

    /// <summary>Minimum loop-at-a-time phase margin required as a hard constraint at each scenario's start pose, degrees.</summary>
    public double PhaseMarginLimitDeg { get; init; } = 40.0;

    /// <summary>Additional slope per degree below <see cref="PhaseMarginLimitDeg"/>.</summary>
    public double PhaseMarginPenalty { get; init; } = 1e-3;
}

/// <summary>One candidate evaluation of the tuner. SPEC §12.7.</summary>
/// <param name="Index">1-based evaluation number (1 is the baseline).</param>
/// <param name="Kp">Kp per joint, in the controller's unit.</param>
/// <param name="Kd">Kd per joint.</param>
/// <param name="NaturalFrequency">ωn per joint, rad/s (the design values the gains came from).</param>
/// <param name="DampingRatio">ζ per joint.</param>
/// <param name="Cost">Combined ITAE + penalties and hard-constraint barriers.</param>
/// <param name="Itae">Combined Σ t·|tool − reference tool|·Δt over the log, m·s².</param>
/// <param name="Faulted">Any run ended in a fault.</param>
/// <param name="ClampedPercent">Largest time percentage with any controller clamping (SPEC §9).</param>
/// <param name="SpectralRadius">Largest ρ of the linearised loop across scenario start poses.</param>
/// <param name="MinPhaseMarginDeg">Smallest loop-at-a-time phase margin across scenario start poses, degrees.</param>
/// <param name="MinGainMarginDb">Smallest loop-at-a-time gain margin across scenario start poses, dB.</param>
public sealed record GainTunerEvaluation(int Index, IReadOnlyList<double> Kp, IReadOnlyList<double> Kd,
    IReadOnlyList<double> NaturalFrequency, IReadOnlyList<double> DampingRatio, double Cost, double Itae, bool Faulted,
    double ClampedPercent, double SpectralRadius, double MinPhaseMarginDeg, double MinGainMarginDb = double.PositiveInfinity);

/// <summary>Reported after every evaluation. SPEC §12.7.</summary>
/// <param name="Evaluation">Evaluations done.</param>
/// <param name="MaxEvaluations">The budget.</param>
/// <param name="Latest">The evaluation just finished.</param>
/// <param name="Best">The best so far.</param>
public sealed record GainTunerProgress(int Evaluation, int MaxEvaluations, GainTunerEvaluation Latest, GainTunerEvaluation Best);

/// <summary>Result of a tuning run. SPEC §12.7.</summary>
/// <param name="Baseline">The scenario's own gains (evaluation 1).</param>
/// <param name="Best">The lowest cost found (possibly the baseline).</param>
/// <param name="History">Every evaluation, in order.</param>
/// <param name="Cancelled">The token was cancelled; the result holds what was found until then.</param>
/// <param name="BestScenario">The primary scenario with the best gains as explicit overrides.</param>
public sealed record GainTunerResult(GainTunerEvaluation Baseline, GainTunerEvaluation Best,
    IReadOnlyList<GainTunerEvaluation> History, bool Cancelled, Scenario BestScenario);

/// <summary>
/// Gain tuning by Nelder–Mead over (ωn, ζ), each evaluation running the nonlinear runner over one or more scenarios
/// and optionally both sensors, with minimum gain and phase margins enforced as hard constraints at each start pose.
/// SPEC §12.7; docs/DECISIONS.md D9.
/// </summary>
public static class GainTuner
{
    private const double ChunkSeconds = 0.25;
    private const int MarginPoints = 200;

    /// <summary>Tunes the PD gains of <paramref name="scenario"/>'s controller. SPEC §12.7.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The options are invalid.</exception>
    public static GainTunerResult Tune(Scenario scenario, GainTunerOptions? options = null,
        Action<GainTunerProgress>? progress = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(scenario);
        return Tune([scenario], options, progress, cancellationToken);
    }

    /// <summary>Tunes the PD gains across a list of <paramref name="scenarios"/>. SPEC §12.7.</summary>
    /// <exception cref="ArgumentException"><paramref name="scenarios"/> is empty.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The options are invalid.</exception>
    public static GainTunerResult Tune(IReadOnlyList<Scenario> scenarios, GainTunerOptions? options = null,
        Action<GainTunerProgress>? progress = null, CancellationToken cancellationToken = default)
    {
        ValidateScenarios(scenarios);
        options ??= new GainTunerOptions();
        Validate(options);

        var primary = scenarios[0];
        var design = new Design(primary, options);
        var history = new List<GainTunerEvaluation>();
        GainTunerEvaluation? best = null;
        var cancelled = false;

        bool IsFeasible(GainTunerEvaluation e) =>
            !e.Faulted && e.SpectralRadius < 1.0 && e.MinGainMarginDb >= options.GainMarginLimitDb && e.MinPhaseMarginDeg >= options.PhaseMarginLimitDeg;

        bool IsBetter(GainTunerEvaluation candidate, GainTunerEvaluation incumbent)
        {
            var cf = IsFeasible(candidate);
            var incf = IsFeasible(incumbent);
            if (cf != incf) return cf;
            return candidate.Cost < incumbent.Cost;
        }

        GainTunerEvaluation Run(double[] kp, double[] kd, double[] wn, double[] zeta)
        {
            if (history.Count >= options.MaxEvaluations) throw new BudgetExhausted();
            if (cancellationToken.IsCancellationRequested) { cancelled = true; throw new BudgetExhausted(); }
            var evaluation = Evaluate(scenarios, kp, kd, wn, zeta, history.Count + 1, options, cancellationToken)
                ?? throw new CancelledDuringRun();
            history.Add(evaluation);
            if (best is null || IsBetter(evaluation, best)) best = evaluation;
            progress?.Invoke(new GainTunerProgress(history.Count, options.MaxEvaluations, evaluation, best));
            return evaluation;
        }

        var (kp0, kd0) = design.ScenarioGains();
        var baselineDesign = design.ToDesign(kp0, kd0);
        GainTunerEvaluation baseline;
        try
        {
            baseline = Run(kp0, kd0, baselineDesign.Wn, baselineDesign.Zeta);
        }
        catch (Exception e) when (e is BudgetExhausted or CancelledDuringRun)
        {
            throw new OperationCanceledException("Cancelled before the baseline run finished.", cancellationToken);
        }

        try
        {
            var u0 = design.Normalise(baselineDesign.Wn, baselineDesign.Zeta);
            double Cost(double[] u)
            {
                var (wn, zeta) = design.Denormalise(u);
                var (kp, kd) = design.ToGains(wn, zeta);
                if (SameGains(kp, kd, kp0, kd0)) return baseline.Cost;
                return Run(kp, kd, wn, zeta).Cost;
            }
            NelderMead(Cost, u0, options, new Random(options.Seed));
        }
        catch (BudgetExhausted) { }
        catch (CancelledDuringRun) { cancelled = true; }

        var winner = best!;
        var bestScenario = primary with
        {
            Controller = primary.Controller with { Kp = JointVector.From(winner.Kp.ToArray()), Kd = JointVector.From(winner.Kd.ToArray()) }
        };
        return new GainTunerResult(baseline, winner, history.AsReadOnly(), cancelled, bestScenario);
    }

    /// <summary>The tuner's cost of <paramref name="scenario"/> with its own gains, as evaluation 1 would compute it. SPEC §12.7.</summary>
    public static GainTunerEvaluation Evaluate(Scenario scenario, GainTunerOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(scenario);
        return Evaluate([scenario], options);
    }

    /// <summary>The tuner's cost of <paramref name="scenarios"/> with the primary scenario's gains, as evaluation 1 would compute it. SPEC §12.7.</summary>
    public static GainTunerEvaluation Evaluate(IReadOnlyList<Scenario> scenarios, GainTunerOptions? options = null)
    {
        ValidateScenarios(scenarios);
        options ??= new GainTunerOptions();
        Validate(options);
        var design = new Design(scenarios[0], options);
        var (kp, kd) = design.ScenarioGains();
        var d = design.ToDesign(kp, kd);
        return Evaluate(scenarios, kp, kd, d.Wn, d.Zeta, 1, options, CancellationToken.None)!;
    }

    private static GainTunerEvaluation? Evaluate(IReadOnlyList<Scenario> scenarios, double[] kp, double[] kd, double[] wn, double[] zeta,
        int index, GainTunerOptions options, CancellationToken token)
    {
        var arm = ArmModel.OpenManipulatorX();
        var maxRho = 0.0;
        var minGm = double.PositiveInfinity;
        var minPm = double.PositiveInfinity;

        foreach (var scenario in scenarios)
        {
            if (token.IsCancellationRequested) return null;
            var linearTrial = scenario with
            {
                Controller = scenario.Controller with { Kp = JointVector.From(kp), Kd = JointVector.From(kd) }
            };
            var controller = (ILinearisableController)ExperimentRunner.CreateController(linearTrial);
            var q0 = linearTrial.Task.Start.ToArray();
            var plantPayload = linearTrial.Task.Kind == TaskKind.PickPlace ? 0.0 : linearTrial.PayloadKg;
            var model = SampledLoopModel.ForArm(arm, linearTrial.GetPlanet(), plantPayload, q0, controller, linearTrial.SamplePeriod);
            maxRho = Math.Max(maxRho, LinearAnalysis.ClosedLoopPoles(model).SpectralRadius);
            for (var i = 0; i < model.InputCount; i++)
            {
                var bode = LoopFrequencyResponse.Compute(model, i, MarginPoints);
                minGm = Math.Min(minGm, bode.GainMarginDb);
                minPm = Math.Min(minPm, bode.PhaseMarginDeg);
            }
        }

        var hardViolation = 0.0;
        if (maxRho >= 1.0) hardViolation += options.SpectralRadiusPenalty * maxRho;
        if (minGm < options.GainMarginLimitDb)
            hardViolation += Math.Max(options.FaultPenalty, 1.0) + (options.GainMarginLimitDb - minGm);
        if (minPm < options.PhaseMarginLimitDeg)
            hardViolation += Math.Max(options.FaultPenalty, 1.0) + (1.0 + options.PhaseMarginPenalty) * (options.PhaseMarginLimitDeg - minPm);

        if (index > 1 && hardViolation > 0.0)
        {
            return new GainTunerEvaluation(index, Array.AsReadOnly(kp), Array.AsReadOnly(kd), Array.AsReadOnly(wn), Array.AsReadOnly(zeta),
                hardViolation, 0.0, false, 0.0, maxRho, minPm, minGm);
        }

        var aggRunCost = 0.0;
        var aggItae = 0.0;
        var anyFaulted = false;
        var maxClamped = 0.0;

        foreach (var scenario in scenarios)
        {
            var sensors = options.Sensor == TuningSensorMode.PerfectAndEncoder
                ? new[] { false, true }
                : new[] { scenario.UseEncoder };
            foreach (var useEncoder in sensors)
            {
                var trial = scenario with
                {
                    UseEncoder = useEncoder,
                    Controller = scenario.Controller with { Kp = JointVector.From(kp), Kd = JointVector.From(kd) }
                };
                var runner = new ExperimentRunner();
                runner.Start(trial);
                while (!runner.IsFinished)
                {
                    if (token.IsCancellationRequested) return null;
                    runner.AdvanceBy(ChunkSeconds);
                }
                var result = runner.Result!;
                var runItae = 0.0;
                foreach (var row in result.Log) runItae += row.Time * (row.Tool - row.ReferenceTool).Length * row.Interval;
                var runFaulted = result.Fault is not null;
                var runClamped = result.Metrics.AnyClampedPercent;
                var runCost = runItae;
                if (runFaulted) runCost += options.FaultPenalty;
                runCost += options.ClampPenalty * Math.Max(0.0, runClamped - options.ClampLimitPercent);

                if (options.CostAggregation == TuningCostAggregation.Max)
                {
                    aggRunCost = Math.Max(aggRunCost, runCost);
                    aggItae = Math.Max(aggItae, runItae);
                }
                else
                {
                    aggRunCost += runCost;
                    aggItae += runItae;
                }
                anyFaulted |= runFaulted;
                maxClamped = Math.Max(maxClamped, runClamped);
            }
        }

        var cost = aggRunCost;
        if (maxRho >= 1.0) cost += options.SpectralRadiusPenalty * maxRho;
        if (minGm < options.GainMarginLimitDb)
            cost += Math.Max(options.FaultPenalty, 1.0) + (options.GainMarginLimitDb - minGm);
        if (minPm < options.PhaseMarginLimitDeg)
            cost += Math.Max(options.FaultPenalty, 1.0) + (1.0 + options.PhaseMarginPenalty) * (options.PhaseMarginLimitDeg - minPm);

        return new GainTunerEvaluation(index, Array.AsReadOnly(kp), Array.AsReadOnly(kd), Array.AsReadOnly(wn), Array.AsReadOnly(zeta),
            cost, aggItae, anyFaulted, maxClamped, maxRho, minPm, minGm);
    }

    private static void ValidateScenarios(IReadOnlyList<Scenario> scenarios)
    {
        ArgumentNullException.ThrowIfNull(scenarios);
        if (scenarios.Count == 0) throw new ArgumentException("At least one scenario is required.", nameof(scenarios));
        foreach (var s in scenarios)
        {
            ArgumentNullException.ThrowIfNull(s);
            s.Validate();
            if (s.Controller.Method == ControlMethod.STSMC)
                throw new NotSupportedException("STSMC is not linearisable at s = 0, so the tuner's margin constraints do not apply (SPEC §12.7, docs/DECISIONS.md D13).");
        }
    }

    /// <summary>Nelder–Mead on [0, 1]^d with clamping; the cost function throws to stop it.</summary>
    private static void NelderMead(Func<double[], double> cost, double[] start, GainTunerOptions options, Random random)
    {
        var d = start.Length;
        var vertices = new double[d + 1][];
        var values = new double[d + 1];
        vertices[0] = Clamp(start);
        for (var j = 0; j < d; j++)
        {
            var v = (double[])vertices[0].Clone();
            var step = options.InitialStep * (1.0 + 0.5 * (random.NextDouble() - 0.5));
            v[j] = v[j] + step <= 1.0 ? v[j] + step : v[j] - step;
            vertices[j + 1] = Clamp(v);
        }
        for (var j = 0; j <= d; j++) values[j] = cost(vertices[j]);

        while (true)
        {
            var order = Enumerable.Range(0, d + 1).OrderBy(i => values[i]).ThenBy(i => i).ToArray();
            vertices = [.. order.Select(i => vertices[i])];
            values = [.. order.Select(i => values[i])];

            var size = 0.0;
            for (var j = 1; j <= d; j++)
                for (var c = 0; c < d; c++) size = Math.Max(size, Math.Abs(vertices[j][c] - vertices[0][c]));
            if (size < 1e-6) return;

            var centroid = new double[d];
            for (var j = 0; j < d; j++)
                for (var c = 0; c < d; c++) centroid[c] += vertices[j][c] / d;
            var worst = vertices[d];

            var reflected = Clamp(Combine(centroid, worst, -1.0));
            var fr = cost(reflected);
            if (fr < values[0])
            {
                var expanded = Clamp(Combine(centroid, worst, -2.0));
                var fe = cost(expanded);
                (vertices[d], values[d]) = fe < fr ? (expanded, fe) : (reflected, fr);
                continue;
            }
            if (fr < values[d - 1])
            {
                (vertices[d], values[d]) = (reflected, fr);
                continue;
            }
            var contracted = fr < values[d]
                ? Clamp(Combine(centroid, worst, -0.5))   // outside contraction
                : Clamp(Combine(centroid, worst, 0.5));   // inside contraction
            var fc = cost(contracted);
            if (fc < Math.Min(fr, values[d]))
            {
                (vertices[d], values[d]) = (contracted, fc);
                continue;
            }
            for (var j = 1; j <= d; j++)
            {
                for (var c = 0; c < d; c++) vertices[j][c] = vertices[0][c] + 0.5 * (vertices[j][c] - vertices[0][c]);
                values[j] = cost(vertices[j]);
            }
        }
    }

    /// <summary>centroid + t (point − centroid): t = −1 reflects, −2 expands, ∓½ contracts.</summary>
    private static double[] Combine(double[] centroid, double[] point, double t)
    {
        var r = new double[centroid.Length];
        for (var c = 0; c < r.Length; c++) r[c] = centroid[c] + t * (point[c] - centroid[c]);
        return r;
    }

    private static double[] Clamp(double[] u)
    {
        for (var c = 0; c < u.Length; c++) u[c] = Math.Clamp(u[c], 0.0, 1.0);
        return u;
    }

    private static bool SameGains(double[] kp, double[] kd, double[] kp0, double[] kd0)
    {
        for (var i = 0; i < kp.Length; i++)
            if (Math.Abs(kp[i] - kp0[i]) > 1e-12 * Math.Max(1.0, Math.Abs(kp0[i]))
                || Math.Abs(kd[i] - kd0[i]) > 1e-12 * Math.Max(1.0, Math.Abs(kd0[i])))
                return false;
        return true;
    }

    private static void Validate(GainTunerOptions o)
    {
        if (!(o.MinNaturalFrequency > 0.0) || !(o.MaxNaturalFrequency > o.MinNaturalFrequency) || !double.IsFinite(o.MaxNaturalFrequency))
            throw new ArgumentOutOfRangeException(nameof(o), "Need 0 < min ωn < max ωn.");
        if (!(o.MinDampingRatio > 0.0) || !(o.MaxDampingRatio > o.MinDampingRatio) || !double.IsFinite(o.MaxDampingRatio))
            throw new ArgumentOutOfRangeException(nameof(o), "Need 0 < min ζ < max ζ.");
        if (o.MaxEvaluations < 1)
            throw new ArgumentOutOfRangeException(nameof(o), "At least one evaluation.");
        if (!(o.InitialStep > 0.0 && o.InitialStep <= 0.5))
            throw new ArgumentOutOfRangeException(nameof(o), "Initial step must be in (0, 0.5].");
        foreach (var w in new[] { o.FaultPenalty, o.ClampPenalty, o.SpectralRadiusPenalty, o.PhaseMarginPenalty, o.ClampLimitPercent })
            if (!double.IsFinite(w) || w < 0.0) throw new ArgumentOutOfRangeException(nameof(o), "Weights and limits must be finite and at least 0.");
        if (!double.IsFinite(o.GainMarginLimitDb) || !double.IsFinite(o.PhaseMarginLimitDeg))
            throw new ArgumentOutOfRangeException(nameof(o), "The gain and phase margin limits must be finite.");
        if (!Enum.IsDefined(o.Mode) || !Enum.IsDefined(o.CostAggregation) || !Enum.IsDefined(o.Sensor))
            throw new ArgumentOutOfRangeException(nameof(o), "Invalid enum option.");
    }

    private sealed class BudgetExhausted : Exception;

    private sealed class CancelledDuringRun : Exception;

    /// <summary>The map between (ωn, ζ) and the gains for one scenario's controller.</summary>
    private sealed class Design
    {
        private readonly Scenario _scenario;
        private readonly GainTunerOptions _options;
        private readonly bool _accelerationGains;
        private readonly double[] _inertia;
        private readonly double[] _damping;
        private readonly int _n;

        public Design(Scenario scenario, GainTunerOptions options)
        {
            _scenario = scenario;
            _options = options;
            var controller = ExperimentRunner.CreateController(scenario);
            _n = controller.DegreesOfFreedom;
            _accelerationGains = controller is ComputedTorque;
            var belief = controller switch
            {
                Pid pid => pid.Belief,
                PdGravity pdg => pdg.Belief,
                ComputedTorque ct => ct.Belief,
                _ => throw new NotSupportedException("PID, PDG or CT only."),
            };
            var m = new double[_n * _n];
            new RecursiveNewtonEuler(belief.Model).InertiaMatrix(scenario.Task.Start.ToArray(), m);
            _inertia = new double[_n];
            _damping = new double[_n];
            for (var i = 0; i < _n; i++)
            {
                _inertia[i] = m[i * _n + i];
                var a = belief.Model.Actuators[i];
                _damping[i] = a.ViscousFriction + a.CoulombFriction / a.SmoothingSpeed;
            }
        }

        public (double[] Kp, double[] Kd) ScenarioGains()
        {
            var (kp, kd, _) = ControllerGains.Get((ILinearisableController)ExperimentRunner.CreateController(_scenario));
            return (kp, kd);
        }

        public (double[] Kp, double[] Kd) ToGains(double[] wn, double[] zeta)
        {
            var kp = new double[_n];
            var kd = new double[_n];
            for (var i = 0; i < _n; i++)
            {
                if (_accelerationGains)
                {
                    kp[i] = wn[i] * wn[i];
                    kd[i] = 2.0 * zeta[i] * wn[i];
                }
                else
                {
                    kp[i] = _inertia[i] * wn[i] * wn[i];
                    kd[i] = Math.Max(0.0, 2.0 * zeta[i] * wn[i] * _inertia[i] - _damping[i]);
                }
            }
            return (kp, kd);
        }

        public (double[] Wn, double[] Zeta) ToDesign(double[] kp, double[] kd)
        {
            var wn = new double[_n];
            var zeta = new double[_n];
            for (var i = 0; i < _n; i++)
            {
                if (_accelerationGains)
                {
                    wn[i] = Math.Sqrt(kp[i]);
                    zeta[i] = wn[i] > 0.0 ? kd[i] / (2.0 * wn[i]) : 0.0;
                }
                else
                {
                    wn[i] = Math.Sqrt(kp[i] / _inertia[i]);
                    zeta[i] = wn[i] > 0.0 ? (kd[i] + _damping[i]) / (2.0 * wn[i] * _inertia[i]) : 0.0;
                }
            }
            return (wn, zeta);
        }

        public double[] Normalise(double[] wn, double[] zeta)
        {
            double W(double w) => (Math.Log(Math.Clamp(w, _options.MinNaturalFrequency, _options.MaxNaturalFrequency)) - Math.Log(_options.MinNaturalFrequency))
                / (Math.Log(_options.MaxNaturalFrequency) - Math.Log(_options.MinNaturalFrequency));
            double Z(double z) => (Math.Clamp(z, _options.MinDampingRatio, _options.MaxDampingRatio) - _options.MinDampingRatio)
                / (_options.MaxDampingRatio - _options.MinDampingRatio);
            if (_options.Mode == TuningMode.Shared)
            {
                var logMean = wn.Select(w => Math.Log(Math.Max(w, _options.MinNaturalFrequency))).Average();
                return [W(Math.Exp(logMean)), Z(zeta.Average())];
            }
            var u = new double[2 * _n];
            for (var i = 0; i < _n; i++)
            {
                u[2 * i] = W(wn[i]);
                u[2 * i + 1] = Z(zeta[i]);
            }
            return u;
        }

        public (double[] Wn, double[] Zeta) Denormalise(double[] u)
        {
            double W(double x) => Math.Exp(Math.Log(_options.MinNaturalFrequency) + x * (Math.Log(_options.MaxNaturalFrequency) - Math.Log(_options.MinNaturalFrequency)));
            double Z(double x) => _options.MinDampingRatio + x * (_options.MaxDampingRatio - _options.MinDampingRatio);
            var wn = new double[_n];
            var zeta = new double[_n];
            for (var i = 0; i < _n; i++)
            {
                var j = _options.Mode == TuningMode.Shared ? 0 : i;
                wn[i] = W(u[2 * j]);
                zeta[i] = Z(u[2 * j + 1]);
            }
            return (wn, zeta);
        }
    }
}
