using System.Diagnostics;
using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using Xunit.Abstractions;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>SPEC §12.7: Nelder–Mead gain tuning on the nonlinear runner (V42).</summary>
public sealed class GainTunerTests(ITestOutputHelper output)
{
    /// <summary>
    /// "Dünya – boş kol – noktaya git" (PDG), per-joint (ωn, ζ), 60 evaluations: the best cost is no worse than the
    /// default gains' cost, evaluation 1 is exactly the default gains, and progress is reported once per evaluation.
    /// </summary>
    [Fact]
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
        Assert.Equal(result.Best.Kp.ToArray(), result.BestScenario.Controller.Kp!.Value.ToArray());
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
        $"{label}: cost {e.Cost:E4} (ITAE {e.Itae:E4}), fault {e.Faulted}, clamped {e.ClampedPercent:F2} %, ρ {e.SpectralRadius:F5}, min PM {e.MinPhaseMarginDeg:F1}°; Kp [{TestFormat.Join(e.Kp.ToArray())}], Kd [{TestFormat.Join(e.Kd.ToArray())}]; ωn [{string.Join(", ", e.NaturalFrequency.Select(v => v.ToString("F2", System.Globalization.CultureInfo.InvariantCulture)))}], ζ [{string.Join(", ", e.DampingRatio.Select(v => v.ToString("F3", System.Globalization.CultureInfo.InvariantCulture)))}]"));
}
