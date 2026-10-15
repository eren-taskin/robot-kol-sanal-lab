using System.Diagnostics;
using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using RobotArm.Core.Tasks;
using Xunit.Abstractions;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>SPEC §12.7: Nelder–Mead gain tuning on the nonlinear runner (V42).</summary>
public sealed class GainTunerTests(ITestOutputHelper output)
{
    /// <summary>
    /// Slow 60-evaluation tuner runs on "Dünya – boş kol – noktaya git" (PDG), moved to the Report category so
    /// <c>dotnet test</c> in Debug stays under a minute. Covers single-sensor and both-sensors (Sum and Max) tuning.
    /// </summary>
    [Fact]
    [Trait("Category", "Report")]
    public void EarthReach_FindsCostNoWorseThanDefaultGains_Within60Evaluations()
    {
        var scenario = AnalysisFixture.EarthReach;
        var options = new GainTunerOptions { MaxEvaluations = 60 };
        var reports = 0;
        var watch = Stopwatch.StartNew();
        var result = GainTuner.Tune(scenario, options, _ => reports++);
        watch.Stop();
        var reference = GainTuner.Evaluate(scenario, options);

        output.WriteLine(Invariant($"{result.History.Count} evaluations in {watch.Elapsed.TotalSeconds:F2} s ({watch.Elapsed.TotalMilliseconds / result.History.Count:F0} ms each)"));
        Describe("default", result.Baseline);
        Describe("best   ", result.Best);
        output.WriteLine(Invariant($"cost ratio best/default = {result.Best.Cost / result.Baseline.Cost:F4}"));

        Assert.False(result.Cancelled);
        Assert.True(result.History.Count <= 60);
        Assert.Equal(result.History.Count, reports);
        Assert.Equal(reference.Cost, result.Baseline.Cost, 1e-12);
        Assert.Equal(PdGravity.DefaultGains.Kp, result.Baseline.Kp);
        Assert.Equal(PdGravity.DefaultGains.Kd, result.Baseline.Kd);
        Assert.True(result.Best.Cost <= result.Baseline.Cost);
        Assert.Equal(result.History.Min(e => e.Cost), result.Best.Cost);
        Assert.True(result.Best.MinGainMarginDb >= options.GainMarginLimitDb);
        Assert.True(result.Best.MinPhaseMarginDeg >= options.PhaseMarginLimitDeg);
        Assert.Equal(result.Best.Kp.ToArray(), result.BestScenario.Controller.Kp!.Value.ToArray());

        foreach (var aggregation in new[] { TuningCostAggregation.Sum, TuningCostAggregation.Max })
        {
            var robustOptions = new GainTunerOptions
            {
                MaxEvaluations = 60,
                Sensor = TuningSensorMode.PerfectAndEncoder,
                CostAggregation = aggregation,
            };
            var robust = GainTuner.Tune(scenario, robustOptions);
            Describe($"both-{aggregation}-default", robust.Baseline);
            Describe($"both-{aggregation}-best   ", robust.Best);
            var encoderRun = new ExperimentRunner().Run(robust.BestScenario with { UseEncoder = true });
            output.WriteLine(Invariant($"both-{aggregation} encoder run: success {encoderRun.Success}, RMS {encoderRun.Metrics.RmsToolError * 1e3:F3} mm, clamped {encoderRun.Metrics.AnyClampedPercent:F2} %"));
            Assert.True(robust.Best.Cost < robust.Baseline.Cost);
            Assert.True(robust.Best.MinGainMarginDb >= robustOptions.GainMarginLimitDb);
            Assert.True(robust.Best.MinPhaseMarginDeg >= robustOptions.PhaseMarginLimitDeg);
            Assert.True(encoderRun.Success, encoderRun.FailureReason);
        }
    }

    /// <summary>
    /// Short tuner test (10 evaluations) in the normal suite: tuning on "Dünya – boş kol – noktaya git" with both
    /// sensors (perfect and encoder) and hard gain/phase margin constraints improves cost and gives gains that pass
    /// that scenario with the encoder (whereas A1's single-scenario perfect-sensor result did not).
    /// </summary>
    [Fact]
    public void EarthReach_WithBothSensors_PassesScenarioWithEncoder_Within10Evaluations()
    {
        var scenario = AnalysisFixture.EarthReach;
        var options = new GainTunerOptions
        {
            MaxEvaluations = 10,
            Sensor = TuningSensorMode.PerfectAndEncoder,
            CostAggregation = TuningCostAggregation.Sum,
            GainMarginLimitDb = 6.0,
            PhaseMarginLimitDeg = 40.0,
        };
        var reports = 0;
        var result = GainTuner.Tune(scenario, options, _ => reports++);
        var reference = GainTuner.Evaluate(scenario, options);

        Describe("default (both sensors)", result.Baseline);
        Describe("best    (10 evals)    ", result.Best);

        Assert.False(result.Cancelled);
        Assert.Equal(10, result.History.Count);
        Assert.Equal(10, reports);
        Assert.Equal(reference.Cost, result.Baseline.Cost, 1e-12);
        Assert.Equal(PdGravity.DefaultGains.Kp, result.Baseline.Kp);
        Assert.Equal(PdGravity.DefaultGains.Kd, result.Baseline.Kd);
        Assert.True(result.Best.Cost < result.Baseline.Cost);
        Assert.True(result.Best.SpectralRadius < 1.0);
        Assert.True(result.Best.MinGainMarginDb >= options.GainMarginLimitDb);
        Assert.True(result.Best.MinPhaseMarginDeg >= options.PhaseMarginLimitDeg);

        var perfectRun = new ExperimentRunner().Run(result.BestScenario with { UseEncoder = false });
        var encoderRun = new ExperimentRunner().Run(result.BestScenario with { UseEncoder = true });
        output.WriteLine(Invariant($"perfect sensor: success {perfectRun.Success}, RMS {perfectRun.Metrics.RmsToolError * 1e3:F3} mm; encoder: success {encoderRun.Success}, RMS {encoderRun.Metrics.RmsToolError * 1e3:F3} mm, clamped {encoderRun.Metrics.AnyClampedPercent:F2} %"));
        Assert.True(perfectRun.Success, perfectRun.FailureReason);
        Assert.True(encoderRun.Success, encoderRun.FailureReason);
        Assert.Null(encoderRun.Fault);
    }

    /// <summary>
    /// Multi-scenario tuning supports Sum and Max cost aggregation and enforces hard gain and phase margin constraints
    /// across every scenario's start pose.
    /// </summary>
    [Fact]
    public void MultiScenario_SumAndMax_AndHardMarginConstraints_AreEnforced()
    {
        var s0 = AnalysisFixture.EarthReach with { Duration = 2.5 };
        var s1 = s0 with { PayloadKg = 0.3, Task = s0.Task with { Start = new(0, -0.1, 0.2, 0.2) } };
        Scenario[] scenarios = [s0, s1];

        var eval0 = GainTuner.Evaluate(s0);
        var eval1 = GainTuner.Evaluate(s1);
        var sumEval = GainTuner.Evaluate(scenarios, new GainTunerOptions { CostAggregation = TuningCostAggregation.Sum });
        var maxEval = GainTuner.Evaluate(scenarios, new GainTunerOptions { CostAggregation = TuningCostAggregation.Max });

        Assert.Equal(eval0.Itae + eval1.Itae, sumEval.Itae, 1e-12);
        Assert.Equal(Math.Max(eval0.Itae, eval1.Itae), maxEval.Itae, 1e-12);
        Assert.Equal(Math.Min(eval0.MinGainMarginDb, eval1.MinGainMarginDb), sumEval.MinGainMarginDb, 1e-12);
        Assert.Equal(Math.Min(eval0.MinPhaseMarginDeg, eval1.MinPhaseMarginDeg), sumEval.MinPhaseMarginDeg, 1e-12);

        var tunedMax = GainTuner.Tune(scenarios, new GainTunerOptions
        {
            Mode = TuningMode.Shared,
            CostAggregation = TuningCostAggregation.Max,
            MaxEvaluations = 5,
            GainMarginLimitDb = 6.0,
            PhaseMarginLimitDeg = 40.0,
        });
        Assert.Equal(5, tunedMax.History.Count);
        Assert.True(tunedMax.Best.Cost <= tunedMax.Baseline.Cost);
        Assert.True(tunedMax.Best.MinGainMarginDb >= 6.0);
        Assert.True(tunedMax.Best.MinPhaseMarginDeg >= 40.0);

        Assert.Throws<ArgumentException>(() => GainTuner.Tune(Array.Empty<Scenario>()));
    }

    /// <summary>Same options and seed give the same history; cancellation returns the work done so far.</summary>
    [Fact]
    public void Tuner_IsDeterministic_AndCancellable()
    {
        var scenario = AnalysisFixture.EarthReach with { Duration = 3.0 };
        var options = new GainTunerOptions { Mode = TuningMode.Shared, MaxEvaluations = 6 };
        var a = GainTuner.Tune(scenario, options);
        var b = GainTuner.Tune(scenario, options);
        Assert.Equal(a.History.Select(e => e.Cost), b.History.Select(e => e.Cost));
        Assert.Equal(6, a.History.Count);
        foreach (var e in a.History.Skip(1))
            Assert.True(e.NaturalFrequency.Distinct().Count() == 1 && e.DampingRatio.Distinct().Count() == 1, "shared mode uses one (ωn, ζ)");

        using var source = new CancellationTokenSource();
        var cancelled = GainTuner.Tune(scenario, options with { MaxEvaluations = 50 }, p => { if (p.Evaluation == 3) source.Cancel(); }, source.Token);
        Assert.True(cancelled.Cancelled);
        Assert.Equal(3, cancelled.History.Count);

        Assert.Throws<ArgumentOutOfRangeException>(() => GainTuner.Tune(scenario, options with { MinDampingRatio = 2, MaxDampingRatio = 1 }));
    }

    private void Describe(string label, GainTunerEvaluation e) => output.WriteLine(Invariant(
        $"{label}: cost {e.Cost:E4} (ITAE {e.Itae:E4}), fault {e.Faulted}, clamped {e.ClampedPercent:F2} %, ρ {e.SpectralRadius:F5}, min GM {e.MinGainMarginDb:F2} dB, min PM {e.MinPhaseMarginDeg:F1}°; Kp [{TestFormat.Join(e.Kp.ToArray())}], Kd [{TestFormat.Join(e.Kd.ToArray())}]; ωn [{string.Join(", ", e.NaturalFrequency.Select(v => v.ToString("F2", System.Globalization.CultureInfo.InvariantCulture)))}], ζ [{string.Join(", ", e.DampingRatio.Select(v => v.ToString("F3", System.Globalization.CultureInfo.InvariantCulture)))}]"));
}
