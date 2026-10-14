using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.SimulationTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §6.2 "Step-size stability" (docs/DECISIONS.md D3): <see cref="StabilityEstimate.MaxStableStep(ArmModel, ReadOnlySpan{double}, ReadOnlySpan{double})"/>
/// = 2.78 / λmax(M⁻¹D), D = diag(fv + fc/ωs + extra damping).
/// </summary>
public sealed class StepStabilityTests(ITestOutputHelper output)
{
    /// <summary>The smoothing speed before D3, which made RK4 unstable at the old default h = 1 ms.</summary>
    private const double OldSmoothingSpeed = 0.01;

    private static readonly double[] s_heldPose = [0.0, -0.4, 0.2, 1.0];

    private static ArmModel WithSmoothingSpeed(ArmModel arm, double omegaS) =>
        new(arm.Joints, arm.Bodies, arm.ToolPoint, arm.Actuator with { SmoothingSpeed = omegaS });

    private static double[] RandomPoseInsideLimits(Random random, ArmModel arm)
    {
        var q = new double[arm.DegreesOfFreedom];
        for (var i = 0; i < q.Length; i++)
        {
            var j = arm.Joints[i];
            q[i] = j.LowerLimit + (j.UpperLimit - j.LowerLimit) * random.NextDouble();
        }
        return q;
    }

    [Fact]
    public void MaxStableStep_MatchesIndependentEigenvalue_WithUnequalDamping()
    {
        // Independent route: λmax(M⁻¹D) by power iteration with the closed-form M and Gaussian elimination.
        var arm = ArmModel.OpenManipulatorX().WithPayload(0.2);
        var closed = new ClosedFormArm(arm);
        var a = arm.Actuator;
        var friction = a.ViscousFriction + a.CoulombFriction / a.SmoothingSpeed;
        var random = new Random(5);
        var worst = 0.0;

        for (var trial = 0; trial < 50; trial++)
        {
            var q = RandomPoseInsideLimits(random, arm);
            double[] extra = [0.0, 3.0 * random.NextDouble(), 0.5 * random.NextDouble(), 0.1 * random.NextDouble()];
            var m = closed.InertiaMatrix(q);
            var v = new double[] { 1.0, 0.7, -0.4, 0.2 };
            var lambda = 0.0;
            for (var it = 0; it < 5000; it++)
            {
                var dv = new double[4];
                for (var i = 0; i < 4; i++) dv[i] = (friction + extra[i]) * v[i];
                var w = ArmPlantTests.GaussSolve(m, dv);
                lambda = Math.Sqrt(w.Sum(x => x * x)) / Math.Sqrt(v.Sum(x => x * x));
                var norm = Math.Sqrt(w.Sum(x => x * x));
                for (var i = 0; i < 4; i++) v[i] = w[i] / norm;
            }

            var expected = StabilityEstimate.Rk4StabilityLimit / lambda;
            var actual = StabilityEstimate.MaxStableStep(arm, q, extra);
            worst = Math.Max(worst, Math.Abs(actual - expected) / expected);
        }

        output.WriteLine(Invariant($"MaxStableStep vs power iteration, 50 poses with unequal damping: max relative difference {worst:E3}"));
        Assert.True(worst < 1e-6, Invariant($"{worst:E3}"));
    }

    [Fact]
    public void OldSmoothingSpeed_UnloadedArm_PredictsAbout017ms()
    {
        // The measurement of T05: with ωs = 0.01 a held arm stays still at 0.1 ms and chatters at 0.2 ms and above;
        // the smallest predicted limit over the measured poses was 0.165 ms.
        var arm = WithSmoothingSpeed(ArmModel.OpenManipulatorX(), OldSmoothingSpeed);
        double[][] poses = [s_heldPose, [0.3, -0.5, 0.9, -0.3], [-1.2, -0.8, 0.6, 0.8], [1.0, -0.6, -0.4, 1.2], [-2.8, -1.2, 0.4, 1.5], [0.0, 0.0, 0.0, 0.0]];
        var min = poses.Min(q => StabilityEstimate.MaxStableStep(arm, q));
        var atZero = StabilityEstimate.MaxStableStep(arm, new double[4]);

        output.WriteLine(Invariant($"ωs = 0.01, unloaded: MaxStableStep at q = 0 {atZero * 1e3:F4} ms, smallest over T05 poses {min * 1e3:F4} ms"));
        Assert.InRange(atZero, 0.16e-3, 0.18e-3);
        Assert.InRange(min, 0.1e-3, 0.2e-3);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(500)]
    public void NewDefaults_MaxStableStep_Above1ms_In1000RandomPosesInsideLimits(int payloadGrams)
    {
        var arm = ArmModel.OpenManipulatorX().WithPayload(payloadGrams / 1000.0);
        Assert.Equal(0.1, arm.Actuator.SmoothingSpeed);
        var random = new Random(445);
        var min = double.PositiveInfinity;
        double[] argMin = [];

        for (var trial = 0; trial < 1000; trial++)
        {
            var q = RandomPoseInsideLimits(random, arm);
            var h = StabilityEstimate.MaxStableStep(arm, q);
            if (h < min) (min, argMin) = (h, q);
        }

        output.WriteLine(Invariant($"Defaults, {payloadGrams / 1000.0:F1} kg, 1000 random poses: smallest MaxStableStep {min * 1e3:F4} ms at q = ({TestFormat.Join(argMin)})"));
        Assert.True(min > 1e-3, Invariant($"{min * 1e3:F4} ms"));
        Assert.True(min > 2.0 * Simulation.DefaultStep);
    }

    [Fact]
    public void HeavierPayload_GivesLargerStableStep()
    {
        // A payload adds a positive semidefinite term to M, so M⁻¹D and λmax can only shrink.
        var arm = ArmModel.OpenManipulatorX();
        var random = new Random(9);
        for (var trial = 0; trial < 100; trial++)
        {
            var q = RandomPoseInsideLimits(random, arm);
            var previous = 0.0;
            foreach (var mp in new[] { 0.0, 0.1, 0.25, 0.5 })
            {
                var h = StabilityEstimate.MaxStableStep(arm.WithPayload(mp), q);
                Assert.True(h > previous, Invariant($"q = ({TestFormat.Join(q)}), {mp} kg: {h:E4} s is not above {previous:E4} s"));
                previous = h;
            }
        }
    }

    [Fact]
    public void ExtraDamping_GivesSmallerStableStep_AndNoDampingGivesInfinity()
    {
        var arm = ArmModel.OpenManipulatorX();
        var friction = StabilityEstimate.MaxStableStep(arm, s_heldPose);
        var withGains = StabilityEstimate.MaxStableStep(arm, s_heldPose, [0.5, 0.5, 0.5, 0.5]);
        Assert.True(withGains < friction);

        var frictionless = FrictionlessArm();
        Assert.Equal(double.PositiveInfinity, StabilityEstimate.MaxStableStep(frictionless, s_heldPose));
        Assert.True(double.IsFinite(StabilityEstimate.MaxStableStep(frictionless, s_heldPose, [0.0, 0.5, 0.0, 0.0])));
        Assert.Throws<ArgumentOutOfRangeException>(() => StabilityEstimate.MaxStableStep(arm, s_heldPose, [0.0, -1.0, 0.0, 0.0]));
        Assert.Throws<ArgumentException>(() => StabilityEstimate.MaxStableStep(arm, s_heldPose, [0.0, 1.0]));
    }

    [Fact]
    public void LectureNote1Lesson_StepAboveStabilityLimit_MakesHeldArmChatter_Regression()
    {
        // Deliberate regression test for Lecture Note 1: a step that is too large for the fastest mode makes
        // the solver, not the physics, move the arm. ωs = 0.01 rad/s and h = 1 ms were the defaults before D3.
        const double h = 1e-3;
        var arm = WithSmoothingSpeed(ArmModel.OpenManipulatorX(), OldSmoothingSpeed);
        var g = new ClosedFormArm(arm).Gravity(s_heldPose, Planet.Earth.G);

        var limit = StabilityEstimate.MaxStableStep(arm, s_heldPose);
        Assert.True(h > limit, Invariant($"the estimate {limit * 1e3:F4} ms should call h = 1 ms unstable"));

        var sim = new Simulation(new ArmPlant(arm, Planet.Earth), new Rk4Solver(8), [.. s_heldPose, 0.0, 0.0, 0.0, 0.0]);
        var worst = 0.0;
        for (var step = 0; step < Steps(1.0, h); step++)
        {
            sim.Advance(g, h);
            worst = Math.Max(worst, DynamicsTestHelpers.MaxAbs(sim.QDot));
        }

        output.WriteLine(Invariant($"ωs = 0.01, h = 1 ms (limit {limit * 1e3:F4} ms), arm held by τ = g(q) on Earth: max |q̇| = {worst:E3} rad/s"));
        Assert.True(worst > 1e-3, Invariant($"max |q̇| = {worst:E3}"));
    }
}
