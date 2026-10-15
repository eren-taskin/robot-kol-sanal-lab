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

/// <summary>Search space, budget and cost weights of the gain tuner. SPEC §12.7.</summary>
public sealed record GainTunerOptions
{
    /// <summary>Per-joint or shared parameters.</summary>
    public TuningMode Mode { get; init; } = TuningMode.PerJoint;

    /// <summary>Lower bound of ωn, rad/s.</summary>
    public double MinNaturalFrequency { get; init; } = 1.0;

    /// <summary>Upper bound of ωn, rad/s.</summary>
    public double MaxNaturalFrequency { get; init; } = 300.0;

    /// <summary>Lower bound of ζ.</summary>
    public double MinDampingRatio { get; init; } = 0.2;

    /// <summary>Upper bound of ζ.</summary>
    public double MaxDampingRatio { get; init; } = 10.0;

    /// <summary>Largest number of cost evaluations (nonlinear runs), the baseline included.</summary>
    public int MaxEvaluations { get; init; } = 60;

    /// <summary>Seed of the generator that spreads the initial simplex.</summary>
    public int Seed { get; init; } = 445;

    /// <summary>Edge of the initial simplex in the normalised parameter box [0, 1]^d.</summary>
    public double InitialStep { get; init; } = 0.1;

    /// <summary>Added when the run ends in a fault (SPEC §6.4).</summary>
    public double FaultPenalty { get; init; } = 10.0;

    /// <summary>Percentage of time with any controller clamping allowed without penalty.</summary>
    public double ClampLimitPercent { get; init; } = 5.0;

    /// <summary>Penalty per percentage point of clamping above <see cref="ClampLimitPercent"/>.</summary>
    public double ClampPenalty { get; init; } = 1e-3;

    /// <summary>Added times ρ when the linearised loop at the start pose has ρ ≥ 1.</summary>
    public double SpectralRadiusPenalty { get; init; } = 10.0;

    /// <summary>Smallest loop-at-a-time phase margin allowed without penalty, degrees.</summary>
    public double PhaseMarginLimitDeg { get; init; } = 30.0;

    /// <summary>Penalty per degree of the smallest phase margin below <see cref="PhaseMarginLimitDeg"/>.</summary>
    public double PhaseMarginPenalty { get; init; } = 1e-3;
}

/// <summary>One cost evaluation of the tuner. SPEC §12.7.</summary>
/// <param name="Index">1-based evaluation number (1 is the baseline).</param>
/// <param name="Kp">Kp per joint, in the controller's unit.</param>
/// <param name="Kd">Kd per joint.</param>
/// <param name="NaturalFrequency">ωn per joint, rad/s (the design values the gains came from).</param>
/// <param name="DampingRatio">ζ per joint.</param>
/// <param name="Cost">ITAE + penalties.</param>
/// <param name="Itae">Σ t·|tool − reference tool|·Δt over the log, m·s².</param>
/// <param name="Faulted">The run ended in a fault.</param>
/// <param name="ClampedPercent">Time percentage with any controller clamping (SPEC §9).</param>
/// <param name="SpectralRadius">ρ of the linearised loop at the start pose.</param>
/// <param name="MinPhaseMarginDeg">Smallest loop-at-a-time phase margin at the start pose, degrees.</param>
public sealed record GainTunerEvaluation(int Index, IReadOnlyList<double> Kp, IReadOnlyList<double> Kd,
    IReadOnlyList<double> NaturalFrequency, IReadOnlyList<double> DampingRatio, double Cost, double Itae, bool Faulted,
    double ClampedPercent, double SpectralRadius, double MinPhaseMarginDeg);

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
/// <param name="BestScenario">The scenario with the best gains as explicit overrides.</param>
public sealed record GainTunerResult(GainTunerEvaluation Baseline, GainTunerEvaluation Best,
    IReadOnlyList<GainTunerEvaluation> History, bool Cancelled, Scenario BestScenario);

/// <summary>
/// Gain tuning by Nelder–Mead over (ωn, ζ), each evaluation a real nonlinear run of the scenario. SPEC §12.7;
/// docs/DECISIONS.md D9.
/// </summary>
/// <remarks>
/// <para>
/// <b>Parameters to gains.</b> CT and CT+I: Kp = ωn², Kd = 2ζωn (the law cancels inertia and friction). PDG and
/// PID: Kp,i = M̂ii ωn,i², Kd,i = max(0, 2ζi ωn,i M̂ii − d̂i), with M̂ii(q_start) of the controller's own model and
/// d̂i = fv + fc/ωs, its friction slope: a decoupled single-joint design that ignores the coupling of M and, for PID,
/// gravity. Ki (PID, CT+I) is kept as in the scenario. The search runs in the box [0, 1]^d, ωn on a logarithmic and
/// ζ on a linear scale between the bounds; every trial point is clamped to the box.
/// </para>
/// <para>
/// <b>Cost.</b> ITAE = Σ t·|p_tool − p_ref|·Δt over the logged samples (SPEC §9 weights), plus: a fault penalty;
/// a penalty per percentage point of controller clamping above a limit; a penalty times ρ when the loop linearised at
/// the start pose (<see cref="SampledLoopModel"/>) has ρ ≥ 1; a penalty per degree of the smallest loop-at-a-time
/// phase margin below a limit (<see cref="LoopFrequencyResponse"/>). The linear terms are evaluated at the start
/// pose only.
/// </para>
/// <para>
/// <b>Search.</b> Evaluation 1 is the scenario's own gains. The simplex starts at their (ωn, ζ) (clamped to the
/// bounds; if that reproduces the gains, evaluation 1 is reused), with edges of <see cref="GainTunerOptions.InitialStep"/>
/// spread ±25 % by a seeded generator, so a run is deterministic. Standard coefficients: reflection 1, expansion 2,
/// contraction ½, shrink ½. It stops at the evaluation budget, when the simplex is smaller than 1e-6, or when the
/// token is cancelled (checked between runs and every 0.25 s of simulated time).
/// </para>
/// </remarks>
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
        scenario.Validate();
        options ??= new GainTunerOptions();
        Validate(options);

        var design = new Design(scenario, options);
        var history = new List<GainTunerEvaluation>();
        GainTunerEvaluation? best = null;
        var cancelled = false;

        GainTunerEvaluation Run(double[] kp, double[] kd, double[] wn, double[] zeta)
        {
            if (history.Count >= options.MaxEvaluations) throw new BudgetExhausted();
            if (cancellationToken.IsCancellationRequested) { cancelled = true; throw new BudgetExhausted(); }
            var evaluation = Evaluate(scenario, kp, kd, wn, zeta, history.Count + 1, options, cancellationToken)
                ?? throw new CancelledDuringRun();
            history.Add(evaluation);
            if (best is null || evaluation.Cost < best.Cost) best = evaluation;
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
        var bestScenario = scenario with
        {
            Controller = scenario.Controller with { Kp = JointVector.From(winner.Kp.ToArray()), Kd = JointVector.From(winner.Kd.ToArray()) }
        };
        return new GainTunerResult(baseline, winner, history.AsReadOnly(), cancelled, bestScenario);
    }

    /// <summary>The tuner's cost of <paramref name="scenario"/> with its own gains, as evaluation 1 would compute it. SPEC §12.7.</summary>
    public static GainTunerEvaluation Evaluate(Scenario scenario, GainTunerOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(scenario);
        scenario.Validate();
        options ??= new GainTunerOptions();
        Validate(options);
        var design = new Design(scenario, options);
        var (kp, kd) = design.ScenarioGains();
        var d = design.ToDesign(kp, kd);
        return Evaluate(scenario, kp, kd, d.Wn, d.Zeta, 1, options, CancellationToken.None)!;
    }

    private static GainTunerEvaluation? Evaluate(Scenario scenario, double[] kp, double[] kd, double[] wn, double[] zeta,
        int index, GainTunerOptions options, CancellationToken token)
    {
        var trial = scenario with
        {
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
        var itae = 0.0;
        foreach (var row in result.Log) itae += row.Time * (row.Tool - row.ReferenceTool).Length * row.Interval;
        var faulted = result.Fault is not null;
        var clamped = result.Metrics.AnyClampedPercent;

        var controller = (ILinearisableController)ExperimentRunner.CreateController(trial);
        var q0 = trial.Task.Start.ToArray();
        var plantPayload = trial.Task.Kind == TaskKind.PickPlace ? 0.0 : trial.PayloadKg;
        var model = SampledLoopModel.ForArm(ArmModel.OpenManipulatorX(), trial.GetPlanet(), plantPayload, q0, controller, trial.SamplePeriod);
        var rho = LinearAnalysis.ClosedLoopPoles(model).SpectralRadius;
        var minPm = double.PositiveInfinity;
        for (var i = 0; i < model.InputCount; i++)
            minPm = Math.Min(minPm, LoopFrequencyResponse.Compute(model, i, MarginPoints).PhaseMarginDeg);

        var cost = itae;
        if (faulted) cost += options.FaultPenalty;
        cost += options.ClampPenalty * Math.Max(0.0, clamped - options.ClampLimitPercent);
        if (rho >= 1.0) cost += options.SpectralRadiusPenalty * rho;
        if (double.IsFinite(minPm)) cost += options.PhaseMarginPenalty * Math.Max(0.0, options.PhaseMarginLimitDeg - minPm);

        return new GainTunerEvaluation(index, Array.AsReadOnly(kp), Array.AsReadOnly(kd), Array.AsReadOnly(wn), Array.AsReadOnly(zeta),
            cost, itae, faulted, clamped, rho, minPm);
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
        if (!double.IsFinite(o.PhaseMarginLimitDeg))
            throw new ArgumentOutOfRangeException(nameof(o), "The phase margin limit must be finite.");
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
