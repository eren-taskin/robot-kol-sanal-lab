using System.Globalization;
using System.Text.Json;
using RobotArm.Core.Control;
using RobotArm.Core.Dynamics;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Logging;
using RobotArm.Core.Model;
using RobotArm.Core.Tasks;
using Xunit.Abstractions;

namespace RobotArm.Tests;

public sealed class ExperimentTests(ITestOutputHelper output)
{
    [Theory]
    [InlineData(TaskKind.Reach)]
    [InlineData(TaskKind.PickPlace)]
    [InlineData(TaskKind.Track)]
    [InlineData(TaskKind.Hold)]
    public void EarthCt_EachTaskSucceeds(TaskKind kind)
    {
        var s = kind == TaskKind.PickPlace ? ScenarioLibrary.EarthPickPlace
            : new Scenario { Task = new TaskSettings { Kind = kind, MotionDuration = 4 } };
        var result = new ExperimentRunner().Run(s);
        Print(s, result);
        Assert.True(result.Success, result.FailureReason);
        Assert.Null(result.Fault);
        if (kind == TaskKind.Hold) Assert.True(result.Metrics.MaxToolError > 1e-4, "The pulse must actually displace the arm.");
        if (kind == TaskKind.PickPlace)
        {
            Assert.Equal(1, result.Metrics.PayloadAttachments); Assert.Equal(1, result.Metrics.PayloadReleases);
            Assert.Equal(0.3, result.Metrics.MaxPayloadKg);
            Assert.Equal(TaskPhase.Released, result.Final.Phase);
        }
    }

    [Fact]
    public void Library_AllNineOutcomesMatchTheirExplanations()
    {
        Assert.Equal(9, ScenarioLibrary.All.Count);
        foreach (var s in ScenarioLibrary.All)
        {
            Assert.False(string.IsNullOrWhiteSpace(s.Description));
            Assert.Contains(s.ExpectedSuccess ? "başarıyla" : "başarısız", s.Description);
            var result = new ExperimentRunner().Run(s);
            Print(s, result);
            Assert.True(s.ExpectedSuccess == result.Success, s.Name + ": " + result.FailureReason);
            if (!result.Success) Assert.False(string.IsNullOrWhiteSpace(result.FailureReason));
        }
    }

    [Fact]
    public void V19_HorizontalReachWithinLimits_ClampsJoint2FromFirstSample_AndFails()
    {
        using var vectors = JsonDocument.Parse(File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "test_vectors.json")));
        var q = vectors.RootElement.GetProperty("horizontal_reach_pose").EnumerateArray().Select(v => v.GetDouble()).ToArray();
        var model = ArmModel.OpenManipulatorX();
        for (var i = 0; i < 4; i++) Assert.True(model.Joints[i].IsWithinLimits(q[i]));
        var tau = new double[4];
        new RecursiveNewtonEuler(model.WithPayload(0.5)).BiasTorque(q, new double[4], Planet.HighG.GravityVector, tau);
        Assert.InRange(Math.Abs(tau[1]), 6.84, 6.85);
        Assert.True(Math.Abs(tau[1]) > model.Actuators[1].StallTorque);
        var s = ScenarioLibrary.All.Single(s => s.PlanetKey == "HighG");
        for (var i = 0; i < 4; i++) Assert.Equal(q[i], s.Task.Start[i], 12);
        var result = new ExperimentRunner().Run(s);
        Assert.Equal(0, result.Log[0].Time); Assert.NotEqual(0, result.Log[0].ClampedMask & 2);
        Assert.Equal(-model.Actuators[1].StallTorque, result.Log[0].Command.J2);
        Assert.False(result.Success); Assert.NotNull(result.FailureReason);
        Assert.True(result.Fault?.Reason == RobotArm.Core.Simulation.FaultReason.Floor || result.Metrics.MaxToolError > 0.002);
        Print(s, result);
    }

    [Fact]
    public void V29_CtIntegral_RemovesForgottenPayloadFinalError_AfterHold()
    {
        var s = ScenarioLibrary.All.Single(s => s.Name == "Yanlış model – CT, 0,3 kg habersiz yük") with { Duration = 1.5 };
        var ct = new ExperimentRunner().Run(s);
        var integral = new ExperimentRunner().Run(s with { Controller = s.Controller with { IntegralEnabled = true } });
        var retunedCt = new ExperimentRunner().Run(s with { Controller = s.Controller with
        { Kp = JointVector.From(ComputedTorque.DefaultIntegralPdGains.Kp.ToArray()), Kd = JointVector.From(ComputedTorque.DefaultIntegralPdGains.Kd.ToArray()) } });
        Assert.Null(ct.Fault); Assert.Null(integral.Fault); Assert.Null(retunedCt.Fault);
        var end = TaskTarget(s);
        var ctError = MaxError(end, ct.Final.Q); var integralError = MaxError(end, integral.Final.Q);
        var retunedError = MaxError(end, retunedCt.Final.Q);
        output.WriteLine(FormattableString.Invariant($"V18(b), 1 s move + 0.5 s hold: CT final {ctError:E9}; CT+I final {integralError:E9}; CT with the same retuned PD gains final {retunedError:E9}; max CT {Max(ct.Metrics.MaxJointError):E9}; max CT+I {Max(integral.Metrics.MaxJointError):E9}"));
        Assert.True(ctError > 0.5); Assert.True(retunedError > 1e-2); Assert.True(integralError < 1e-2);
    }

    [Theory]
    [InlineData(PayloadKnowledge.Informed)]
    [InlineData(PayloadKnowledge.Unaware)]
    public void PickPlace_BeliefChangesOnlyWhenInformed(PayloadKnowledge knowledge)
    {
        // Low gravity makes both variants complete, so both attachment and release are observed.
        var s = ScenarioLibrary.EarthPickPlace with { PlanetKey = "Orbit", PayloadKg = 0.1, Belief = new BeliefSettings { Knowledge = knowledge } };
        var result = new ExperimentRunner().Run(s);
        Assert.True(result.Success, result.FailureReason);
        Assert.Contains(result.Log, r => r.PayloadAttached && r.PayloadKg == 0.1);
        Assert.Contains(result.Log, r => r.Phase == TaskPhase.Released && r.PayloadKg == 0);
        Assert.All(result.Log, row => Assert.Equal(knowledge == PayloadKnowledge.Informed ? row.PayloadKg : 0, row.BeliefPayloadKg));
    }

    [Fact]
    public void V16_ScenarioJsonAndSeed_ProduceBitIdenticalFullLogs_RegardlessOfBatches()
    {
        var s = ScenarioLibrary.EarthPickPlace with { UseEncoder = true, NoiseStdDev = 0.0001, Seed = 123 };
        var json = ScenarioJson.Serialize(s); var restored = ScenarioJson.Deserialize(json);
        Assert.Equal(s, restored); Assert.Equal(json, ScenarioJson.Serialize(restored));
        var one = new ExperimentRunner().Run(s); var two = new ExperimentRunner().Run(restored);
        var stepped = new ExperimentRunner(); stepped.Start(restored);
        var chunks = new[] { 0.00003, 0.016, 0.0037, 0.041, 0.00007 };
        for (var i = 0; !stepped.IsFinished; i++) stepped.AdvanceBy(chunks[i % chunks.Length]);
        Assert.Equal(one.Log, two.Log); Assert.Equal(one.Log, stepped.Result!.Log);
        Assert.Equal(one.Metrics, stepped.Result.Metrics); Assert.Equal(one.Final, stepped.Current);
        var different = new ExperimentRunner().Run(s with { Seed = 124 });
        Assert.NotEqual(Export(one.Log), Export(different.Log));
    }

    [Fact]
    public void Logging_OneRowPerSample_NoEndpointSample_AndFractionalBatchBudget()
    {
        var s = new Scenario { Duration = 1.001, Task = new TaskSettings { Kind = TaskKind.Hold, PulseDuration = 0 } };
        var runner = new ExperimentRunner(); runner.Start(s);
        Assert.Single(runner.Log); Assert.Equal(s.Task.Start, runner.Log[0].Q);
        runner.AdvanceBy(0.0002); Assert.Equal(0, runner.Current.Time);
        runner.AdvanceBy(0.0003); Assert.Equal(s.Step, runner.Current.Time);
        runner.AdvanceBy(2);
        Assert.True(runner.IsFinished); Assert.Equal(s.Duration, runner.Current.Time);
        Assert.Equal(501, runner.Log.Count); Assert.Equal(0.001, runner.Log[^1].Interval, 12);
        for (var i = 0; i < runner.Log.Count; i++) Assert.Equal(i * s.SamplePeriod, runner.Log[i].Time, 12);
        Assert.Equal(s.Duration, runner.Log.Sum(r => r.Interval), 12);
        var result = runner.Result; runner.AdvanceBy(100); Assert.Same(result, runner.Result);
    }

    [Theory]
    [InlineData(0)] [InlineData(-1)] [InlineData(60.001)] [InlineData(double.NaN)] [InlineData(double.PositiveInfinity)]
    public void Duration_OutsideZeroToSixty_IsRejected(double duration) =>
        Assert.Throws<ArgumentOutOfRangeException>(() => new ExperimentRunner().Start(new Scenario { Duration = duration }));

    [Fact]
    public void Validation_RejectsInvalidTimingAndNonfiniteParameters()
    {
        (new Scenario { Duration = 60 }).Validate();
        Assert.Throws<ArgumentOutOfRangeException>(() => (new Scenario { SamplePeriod = 0.0021 }).Validate());
        Assert.Throws<ArgumentOutOfRangeException>(() => (new Scenario { Duration = 1.0001 }).Validate());
        Assert.Throws<ArgumentOutOfRangeException>(() => (new Scenario { PayloadKg = 0.51 }).Validate());
        Assert.Throws<ArgumentOutOfRangeException>(() => (new Scenario { Task = new TaskSettings { Start = new(0, 2, 0, 0) } }).Validate());
        Assert.Throws<ArgumentOutOfRangeException>(() => (new Scenario { Controller = new ControllerSettings { Kp = new(double.NaN, 0, 0, 0) } }).Validate());
    }

    [Fact]
    public void ScenarioJson_RoundTripsCustomEnvironmentBeliefAndAllGainOverrides()
    {
        var s = new Scenario
        {
            PlanetKey = Planet.CustomKey, CustomGravity = 12.3, PayloadKg = 0.1,
            Belief = new BeliefSettings { Gravity = 8.7, PayloadKg = 0.3, MassScale = 1.2, Knowledge = PayloadKnowledge.Unaware },
            Controller = new ControllerSettings { IntegralEnabled = true, Kp = new(1, 2, 3, 4), Kd = new(5, 6, 7, 8), Ki = new(9, 10, 11, 12), PredictHalfSample = false },
            Task = new TaskSettings { Kind = TaskKind.PickPlace, Target = new(0.2, 0.1, 0.2), ObjectPosition = new(0.21, 0.02, 0.3) },
            UseEncoder = true, NoiseStdDev = 0.001, Seed = -123
        };
        Assert.Equal(s, ScenarioJson.Deserialize(ScenarioJson.Serialize(s)));
    }

    [Fact]
    public void UnreachableTarget_ReturnsFailedResultWithReason()
    {
        var result = new ExperimentRunner().Run(new Scenario { Task = new TaskSettings { Target = new(9, 0, 9) } });
        Assert.False(result.Success); Assert.Contains("Unreachable", result.FailureReason); Assert.Empty(result.Log);
    }

    [Theory]
    [InlineData(TrackShape.Line)] [InlineData(TrackShape.Circle)]
    public void Track_BothShapesSucceed(TrackShape shape)
    {
        var start = new JointVector(0, 0.1, -0.8, 0.6);
        var target = ForwardKinematics.ClosedForm(start.ToArray()).Position + new Vector3d(-0.02, 0.02, 0);
        var result = new ExperimentRunner().Run(new Scenario { Task = new TaskSettings { Kind = TaskKind.Track, Shape = shape, Start = start, Target = target, MotionDuration = 4 } });
        Assert.True(result.Success, result.FailureReason);
    }

    [Fact]
    public void Csv_RoundTripsEveryNumericField_InTurkishCulture()
    {
        var result = new ExperimentRunner().Run(new Scenario { Duration = 2, Task = new TaskSettings { MotionDuration = 1 } });
        var old = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("tr-TR");
            var csv = Export(result.Log); var lines = csv.Split('\n', StringSplitOptions.RemoveEmptyEntries);
            Assert.Equal(result.Log.Count + 1, lines.Length);
            var headers = lines[0].TrimEnd('\r').Split(',');
            for (var i = 0; i < result.Log.Count; i++)
            {
                var cells = lines[i + 1].TrimEnd('\r').Split(','); Assert.Equal(headers.Length, cells.Length);
                var row = result.Log[i];
                var expected = new List<double> { row.Time, row.Interval };
                foreach (var values in new[] { row.Q, row.QDot, row.Reference, row.ReferenceVelocity, row.ReferenceAcceleration, row.Command, row.AppliedTorque }) expected.AddRange(values.ToArray());
                for (var j = 0; j < 4; j++) expected.Add((row.ClampedMask >> j) & 1);
                for (var j = 0; j < 4; j++) expected.Add((row.SaturatedMask >> j) & 1);
                expected.AddRange([row.Tool.X, row.Tool.Y, row.Tool.Z, row.ReferenceTool.X, row.ReferenceTool.Y, row.ReferenceTool.Z, row.PayloadAttached ? 1 : 0, row.PayloadKg, row.BeliefPayloadKg]);
                Assert.Equal(row.Phase.ToString(), cells[47]);
                expected.AddRange([row.ObjectPosition.X, row.ObjectPosition.Y, row.ObjectPosition.Z]); expected.AddRange(row.Disturbance.ToArray());
                var numeric = cells.Where((_, j) => j != 47).Select(v => double.Parse(v, CultureInfo.InvariantCulture)).ToArray();
                Assert.Equal(expected, numeric);
            }
            Assert.Equal(result.ScenarioJson, ScenarioJson.Serialize(ScenarioJson.Deserialize(result.ScenarioJson)));
        }
        finally { CultureInfo.CurrentCulture = old; }
    }

    private void Print(Scenario s, RunResult r) => output.WriteLine(FormattableString.Invariant($"{s.Name}: success={r.Success}, settling={r.Metrics.SettlingTime}, rms={r.Metrics.RmsToolError:E6}, final={r.Metrics.FinalToolError:E6}, peak={Max(r.Metrics.PeakTorque):F6}, clamped={r.Metrics.AnyClampedPercent:F3}%, reason={r.FailureReason}"));
    internal static string Export(IReadOnlyList<LogRow> log) { using var writer = new StringWriter(); CsvLogWriter.Write(writer, log); return writer.ToString(); }
    internal static double Max(JointVector v) => v.ToArray().Max();
    private static JointVector TaskTarget(Scenario s) { var q = InverseKinematics.Solve(s.Task.Target, s.Task.Pitch, s.Task.Branch).Q; return new(q[0], q[1], q[2], q[3]); }
    private static double MaxError(JointVector a, JointVector b) => Enumerable.Range(0, 4).Max(i => Math.Abs(a[i] - b[i]));
}
