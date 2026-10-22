using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using RobotArm.Core.Tasks;

namespace RobotArm.Web.Models;

/// <summary>
/// Asynchronous wrapper around <see cref="GainTuner"/> that yields to the browser event loop
/// between candidate evaluations so the Blazor UI, progress bar, and Cancel button stay responsive.
/// Produces the exact same deterministic Nelder–Mead sequence and <see cref="GainTunerResult"/> as
/// <see cref="GainTuner.Tune(IReadOnlyList{Scenario}, GainTunerOptions?, Action{GainTunerProgress}?, CancellationToken)"/>.
/// </summary>
public static class AsyncGainTuner
{
    private const int MarginPoints = 200;

    public static async Task<GainTunerResult> TuneAsync(
        IReadOnlyList<Scenario> scenarios,
        GainTunerOptions options,
        Action<GainTunerProgress>? progress = null,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(scenarios);
        if (scenarios.Count == 0)
            throw new ArgumentException("En az bir senaryo seçilmelidir.", nameof(scenarios));
        foreach (var s in scenarios)
        {
            s.Validate();
        }

        var primary = scenarios[0];
        var design = new Design(primary, options);
        var history = new List<GainTunerEvaluation>();
        GainTunerEvaluation? best = null;
        var cancelled = false;

        bool IsFeasible(GainTunerEvaluation e) =>
            !e.Faulted &&
            e.SpectralRadius < 1.0 &&
            e.MinGainMarginDb >= options.GainMarginLimitDb &&
            e.MinPhaseMarginDeg >= options.PhaseMarginLimitDeg;

        bool IsBetter(GainTunerEvaluation candidate, GainTunerEvaluation incumbent)
        {
            var cf = IsFeasible(candidate);
            var incf = IsFeasible(incumbent);
            if (cf != incf) return cf;
            return candidate.Cost < incumbent.Cost;
        }

        async Task<GainTunerEvaluation> RunAsync(double[] kp, double[] kd, double[] wn, double[] zeta)
        {
            if (history.Count >= options.MaxEvaluations) throw new BudgetExhausted();
            if (cancellationToken.IsCancellationRequested) { cancelled = true; throw new BudgetExhausted(); }

            // Yield to Blazor UI thread before each evaluation so progress bar and Cancel button stay responsive
            await Task.Delay(1, CancellationToken.None);
            if (cancellationToken.IsCancellationRequested) { cancelled = true; throw new BudgetExhausted(); }

            var evaluation = EvaluateCandidate(scenarios, kp, kd, wn, zeta, history.Count + 1, options, cancellationToken)
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
            baseline = await RunAsync(kp0, kd0, baselineDesign.Wn, baselineDesign.Zeta);
        }
        catch (Exception e) when (e is BudgetExhausted or CancelledDuringRun)
        {
            throw new OperationCanceledException("Otomatik ayar başlangıç değerlendirmesi tamamlanmadan iptal edildi.", cancellationToken);
        }

        try
        {
            var u0 = design.Normalise(baselineDesign.Wn, baselineDesign.Zeta);
            async Task<double> CostAsync(double[] u)
            {
                var (wn, zeta) = design.Denormalise(u);
                var (kp, kd) = design.ToGains(wn, zeta);
                if (SameGains(kp, kd, kp0, kd0)) return baseline.Cost;
                var eval = await RunAsync(kp, kd, wn, zeta);
                return eval.Cost;
            }
            await NelderMeadAsync(CostAsync, u0, options, new Random(options.Seed));
        }
        catch (BudgetExhausted) { }
        catch (CancelledDuringRun) { cancelled = true; }

        var winner = best!;
        var bestScenario = primary with
        {
            Controller = primary.Controller with
            {
                Kp = JointVector.From(winner.Kp.ToArray()),
                Kd = JointVector.From(winner.Kd.ToArray())
            }
        };
        return new GainTunerResult(baseline, winner, history.AsReadOnly(), cancelled, bestScenario);
    }

    private static GainTunerEvaluation? EvaluateCandidate(
        IReadOnlyList<Scenario> scenarios,
        double[] kp,
        double[] kd,
        double[] wn,
        double[] zeta,
        int index,
        GainTunerOptions options,
        CancellationToken token)
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
            var controller = CreateLinearisableController(linearTrial);
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
            return new GainTunerEvaluation(
                index,
                Array.AsReadOnly((double[])kp.Clone()),
                Array.AsReadOnly((double[])kd.Clone()),
                Array.AsReadOnly((double[])wn.Clone()),
                Array.AsReadOnly((double[])zeta.Clone()),
                hardViolation,
                0.0,
                false,
                0.0,
                maxRho,
                minPm,
                minGm);
        }

        if (token.IsCancellationRequested) return null;

        // Delegate full nonlinear evaluation to Core's GainTuner.Evaluate
        var trialScenarios = scenarios
            .Select(s => s with
            {
                Controller = s.Controller with { Kp = JointVector.From(kp), Kd = JointVector.From(kd) }
            })
            .ToArray();

        var coreEval = GainTuner.Evaluate(trialScenarios, options);
        return coreEval with
        {
            Index = index,
            Kp = Array.AsReadOnly((double[])kp.Clone()),
            Kd = Array.AsReadOnly((double[])kd.Clone()),
            NaturalFrequency = Array.AsReadOnly((double[])wn.Clone()),
            DampingRatio = Array.AsReadOnly((double[])zeta.Clone())
        };
    }

    internal static ILinearisableController CreateLinearisableController(Scenario scenario)
    {
        var model = ArmModel.OpenManipulatorX();
        var planet = scenario.GetPlanet();
        var payload = scenario.Task.Kind == TaskKind.PickPlace ? 0.0 : scenario.PayloadKg;
        var scale = scenario.Belief.MassScale;
        var believedModel = scale == 1.0
            ? model
            : new ArmModel(
                model.Joints,
                model.Bodies.Select(b => new BodyParameters(b.Mass * scale, b.CenterOfMass, scale * b.Inertia)),
                model.ToolPoint,
                model.Actuators);
        var belief = new ControllerBelief(
            believedModel,
            scenario.Belief.Gravity is double g ? Planet.Custom(g) : planet,
            scenario.Belief.PayloadKg ?? payload);

        var c = scenario.Controller;
        var defaults = c.Method == ControlMethod.CT
            ? c.IntegralEnabled ? ComputedTorque.DefaultIntegralPdGains : ComputedTorque.DefaultGains
            : PdGravity.DefaultGains;
        var kp = c.Kp?.ToArray() ?? defaults.Kp.ToArray();
        var kd = c.Kd?.ToArray() ?? defaults.Kd.ToArray();
        return c.Method switch
        {
            ControlMethod.PID => new Pid(belief, new PidGains(kp, c.Ki?.ToArray() ?? Pid.DefaultGains.Ki.ToArray(), kd)),
            ControlMethod.PDG => new PdGravity(belief, new PdGains(kp, kd)),
            ControlMethod.CT => new ComputedTorque(belief, new PdGains(kp, kd), c.PredictHalfSample,
                c.IntegralEnabled ? c.Ki?.ToArray() ?? ComputedTorque.DefaultIntegralGains.ToArray() : null),
            _ => throw new ArgumentOutOfRangeException(nameof(scenario))
        };
    }

    private static async Task NelderMeadAsync(
        Func<double[], Task<double>> costAsync,
        double[] start,
        GainTunerOptions options,
        Random random)
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
        for (var j = 0; j <= d; j++) values[j] = await costAsync(vertices[j]);

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
            var fr = await costAsync(reflected);
            if (fr < values[0])
            {
                var expanded = Clamp(Combine(centroid, worst, -2.0));
                var fe = await costAsync(expanded);
                (vertices[d], values[d]) = fe < fr ? (expanded, fe) : (reflected, fr);
                continue;
            }
            if (fr < values[d - 1])
            {
                (vertices[d], values[d]) = (reflected, fr);
                continue;
            }
            var contracted = fr < values[d]
                ? Clamp(Combine(centroid, worst, -0.5))
                : Clamp(Combine(centroid, worst, 0.5));
            var fc = await costAsync(contracted);
            if (fc < Math.Min(fr, values[d]))
            {
                (vertices[d], values[d]) = (contracted, fc);
                continue;
            }
            for (var j = 1; j <= d; j++)
            {
                for (var c = 0; c < d; c++) vertices[j][c] = vertices[0][c] + 0.5 * (vertices[j][c] - vertices[0][c]);
                values[j] = await costAsync(vertices[j]);
            }
        }
    }

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

    private sealed class BudgetExhausted : Exception;
    private sealed class CancelledDuringRun : Exception;

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
            var controller = CreateLinearisableController(scenario);
            _n = ((IController)controller).DegreesOfFreedom;
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
            var controller = CreateLinearisableController(_scenario);
            return controller switch
            {
                Pid pid => (pid.Gains.Kp.ToArray(), pid.Gains.Kd.ToArray()),
                PdGravity pdg => (pdg.Gains.Kp.ToArray(), pdg.Gains.Kd.ToArray()),
                ComputedTorque ct => (ct.Gains.Kp.ToArray(), ct.Gains.Kd.ToArray()),
                _ => throw new NotSupportedException()
            };
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
