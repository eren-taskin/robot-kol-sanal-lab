using System.Diagnostics;
using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using RobotArm.Core.Model;
using Xunit.Abstractions;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §12.8: each analysis call on the arm takes less than 50 ms natively (the gain tuner excepted). Wall-clock,
/// so it is in the Report category that CI skips; run it explicitly (Release for representative numbers).
/// </summary>
public sealed class AnalysisTimingTests(ITestOutputHelper output)
{
    private const double Ts = ControlLoop.DefaultSamplePeriod;
    private const double LimitMs = 50.0;

    [Fact]
    [Trait("Category", "Report")]
    public void EveryAnalysisCall_TakesLessThan50ms()
    {
        var arm = ArmModel.OpenManipulatorX();
        var q0 = AnalysisFixture.StartPose;
        var pdg = AnalysisFixture.DefaultPdg();
        var pid = new Pid(ControllerBelief.OpenManipulatorX(Planet.Earth), Pid.DefaultGains);
        var cti = AnalysisFixture.CtPlusI();
        var random = new Random(1);
        var m12 = Enumerable.Range(0, 144).Select(_ => random.NextDouble() - 0.5).ToArray();
        var pidModel = SampledLoopModel.ForArm(arm, Planet.Earth, 0.0, q0, pid, Ts);

        (string Name, Action Call)[] calls =
        [
            ("EigenSolver.Eigenvalues, 12 × 12", () => EigenSolver.Eigenvalues(m12, 12)),
            ("ClosedLoopPoles, PDG (8 states)", () => LinearAnalysis.ClosedLoopPoles(arm, Planet.Earth, 0.0, q0, pdg, Ts)),
            ("ClosedLoopPoles, PID (12 states)", () => LinearAnalysis.ClosedLoopPoles(arm, Planet.Earth, 0.0, q0, pid, Ts)),
            ("ClosedLoopPoles, CT+I (12 states)", () => LinearAnalysis.ClosedLoopPoles(arm, Planet.Earth, 0.0, q0, cti, Ts)),
            ("RootLocus, PDG scale 0.05–50", () => RootLocus.Sweep(arm, Planet.Earth, 0.0, q0, pdg, Ts, LocusParameter.ScaleAll, 0.05, 50.0)),
            ("RootLocus, PID Kd joint 1, 0.01–100", () => RootLocus.Sweep(arm, Planet.Earth, 0.0, q0, pid, Ts, new LocusParameter(LocusGain.Kd, 0), 0.01, 100.0)),
            ("LoopFrequencyResponse, PDG joint 2 (400 points)", () => LoopFrequencyResponse.Compute(arm, Planet.Earth, 0.0, q0, pdg, Ts, 1)),
            ("LoopFrequencyResponse, PID joint 2 (12 states)", () => LoopFrequencyResponse.Compute(pidModel, 1)),
            ("LinearStepResponse, PDG 2 s", () => LinearStepResponse.Compute(arm, Planet.Earth, 0.0, q0, pdg, Ts, [0.01, 0.01, 0.01, 0.01], 2.0)),
            ("DiscreteLqr.ForArm", () => DiscreteLqr.ForArm(arm, Planet.Earth, 0.0, q0, Ts, [100, 100, 100, 100], [0.1, 0.1, 0.1, 0.1], [1, 1, 1, 1])),
        ];

        var config = typeof(AnalysisTimingTests).Assembly.GetCustomAttributes(typeof(System.Reflection.AssemblyConfigurationAttribute), false)
            .Cast<System.Reflection.AssemblyConfigurationAttribute>().FirstOrDefault()?.Configuration ?? "?";
        output.WriteLine($"configuration {config}; median of 7 warm calls, and the worst of them");
        foreach (var (name, call) in calls)
        {
            call(); // warm-up (JIT)
            var times = new double[7];
            for (var i = 0; i < times.Length; i++)
            {
                var watch = Stopwatch.StartNew();
                call();
                times[i] = watch.Elapsed.TotalMilliseconds;
            }
            Array.Sort(times);
            output.WriteLine(Invariant($"  {name,-50} median {times[3],8:F3} ms   max {times[^1],8:F3} ms"));
            Assert.True(times[3] < LimitMs, Invariant($"{name}: {times[3]:F1} ms"));
        }
    }
}
