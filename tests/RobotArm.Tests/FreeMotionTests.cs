using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.SimulationTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §6.1–§6.2 (V9–V12 of SPEC §11): free motion of the arm, τ = 0, no friction, Jm = 0, Earth, from
/// <c>free_motion.x0</c> of the vectors, integrated by <see cref="Rk4Solver"/> through <see cref="Simulation"/>.
/// </summary>
/// <remarks>
/// Fault detection is off in these runs: the free arm leaves the joint limits (q2 reaches about 3.6 rad)
/// and the tool goes below the floor within 0.4 s. The conserved quantities are what is tested, not the
/// physical plausibility of the motion.
/// </remarks>
public sealed class FreeMotionTests(ITestOutputHelper output)
{
    private static readonly double[] s_zeroTorque = new double[4];
    private static FreeMotionVectors Vectors => TestVectors.Instance.FreeMotion;

    [Fact]
    public void Energy_OfInitialState_MatchesVectors()
    {
        var arm = FrictionlessArm();
        var e0 = Energy(arm, new RecursiveNewtonEuler(arm), Vectors.G, Vectors.X0);
        output.WriteLine(Invariant($"E(x0) = {e0:R} J, vectors {Vectors.Energy0:R} J"));
        Assert.Equal(Vectors.Energy0, e0, 1e-12);
    }

    [Fact]
    public void V9_FreeMotion_EnergyDrift_Below1e5J_In04sAt2ms()
    {
        var sim = FreeMotion();
        var arm = sim.Plant.Model;
        var rnea = new RecursiveNewtonEuler(arm);
        var e0 = Energy(arm, rnea, Vectors.G, sim.State);
        var worst = 0.0;
        var moved = 0.0;

        for (var i = 0; i < Steps(0.4, 2e-3); i++)
        {
            sim.Advance(s_zeroTorque, 2e-3);
            worst = Math.Max(worst, Math.Abs(Energy(arm, rnea, Vectors.G, sim.State) - e0));
            moved = Math.Max(moved, MaxAbsDiff(sim.Q, Vectors.X0.AsSpan(0, 4)));
        }

        output.WriteLine(Invariant($"V9 max |E − E0| over 0.4 s at h = 2 ms: {worst:E3} J (Python reference about 2.3e-6 J); max |q − q0| = {moved:F3} rad"));
        Assert.Null(sim.Fault);
        // A frozen simulation would conserve energy too, so the arm must also have moved.
        Assert.True(moved > 0.1, Invariant($"the state hardly moved: {moved:E3} rad"));
        Assert.True(worst < 1e-5, Invariant($"energy drift {worst:E3} J"));
    }

    [Fact]
    public void V10_FreeMotion_YawMomentumDrift_Below1e7_In04sAt2ms()
    {
        var sim = FreeMotion();
        var rnea = new RecursiveNewtonEuler(sim.Plant.Model);
        var min = double.PositiveInfinity;
        var max = double.NegativeInfinity;
        var moved = 0.0;

        // Same sampling as the reference script: p1 after each step.
        for (var i = 0; i < Steps(0.4, 2e-3); i++)
        {
            sim.Advance(s_zeroTorque, 2e-3);
            var p1 = YawMomentum(rnea, sim.State);
            min = Math.Min(min, p1);
            max = Math.Max(max, p1);
            moved = Math.Max(moved, MaxAbsDiff(sim.Q, Vectors.X0.AsSpan(0, 4)));
        }

        var drift = max - min;
        output.WriteLine(Invariant($"V10 p1 = Σ M1j q̇j: max − min over 0.4 s at h = 2 ms = {drift:E3} (Python reference about 8e-9), p1 ≈ {max:R}; max |q − q0| = {moved:F3} rad"));
        Assert.True(moved > 0.1, Invariant($"the state hardly moved: {moved:E3} rad"));
        Assert.True(drift < 1e-7, Invariant($"p1 drift {drift:E3}"));
    }

    [Fact]
    public void V11_Rk4ErrorAt04s_ShrinksByFourthOrder_RatiosBetween12And20()
    {
        var reference = Vectors.ReferenceStates["0.4"];
        double[] steps = [4e-3, 2e-3, 1e-3];
        var errors = new double[steps.Length];
        for (var k = 0; k < steps.Length; k++)
        {
            var sim = FreeMotion();
            Run(sim, s_zeroTorque, steps[k], Steps(0.4, steps[k]));
            errors[k] = MaxAbsDiff(sim.State, reference);
        }

        var ratio1 = errors[0] / errors[1];
        var ratio2 = errors[1] / errors[2];
        output.WriteLine(Invariant($"V11 max state error at 0.4 s: h = 4 ms {errors[0]:E3}, 2 ms {errors[1]:E3}, 1 ms {errors[2]:E3}"));
        output.WriteLine(Invariant($"V11 ratios {ratio1:F2}, {ratio2:F2} (Python reference about 17.9, 17.5; RK4 ideal 16)"));
        Assert.InRange(ratio1, 12.0, 20.0);
        Assert.InRange(ratio2, 12.0, 20.0);
    }

    [Fact]
    public void V12_StatesAt01_02_04s_At1ms_MatchReferenceStates()
    {
        const double h = 1e-3;
        var sim = FreeMotion();
        var worst = 0.0;
        var done = 0;
        foreach (var (time, expected) in new[] { (0.1, "0.1"), (0.2, "0.2"), (0.4, "0.4") })
        {
            var target = Steps(time, h);
            Run(sim, s_zeroTorque, h, target - done);
            done = target;

            var diff = MaxAbsDiff(sim.State, Vectors.ReferenceStates[expected]);
            worst = Math.Max(worst, diff);
            output.WriteLine(Invariant($"V12 t = {time:F1} s: max |x − x_ref| = {diff:E3}"));
            Assert.True(diff < 5e-5, Invariant($"state at {time} s differs by {diff:E3}"));
        }

        output.WriteLine(Invariant($"V12 worst {worst:E3} (Python reference at h = 1 ms: {Vectors.ErrorH1msVsReference:E3}, tolerance 5e-5)"));
    }

    [Fact]
    public void FreeMotion_WithFaultDetectionOn_StopsWithinTheRun()
    {
        // The reason V9–V12 switch fault detection off: the same run stops by itself.
        var sim = FreeMotion(FaultChecks.All);
        for (var i = 0; i < Steps(0.4, 2e-3) && !sim.IsStopped; i++)
            sim.Advance(s_zeroTorque, 2e-3);

        Assert.NotNull(sim.Fault);
        output.WriteLine(Invariant($"Free motion with faults on: {sim.Fault!.Reason} at t = {sim.Fault.Time:F3} s (joint {sim.Fault.Joint}, value {sim.Fault.Value:F4})"));
    }

    [Fact]
    public void Euler_IsFirstOrder_ErrorRatioAboutTwo()
    {
        // SPEC §6.2: Euler for the teaching comparison. Short run so that Euler stays in its asymptotic range.
        var fm = Vectors;
        var reference = new double[8];
        {
            var sim = FreeMotion();
            Run(sim, s_zeroTorque, 1e-4, Steps(0.02, 1e-4));
            sim.State.CopyTo(reference);
        }

        double Error(double h)
        {
            var plant = new ArmPlant(FrictionlessArm(), Planet.Earth);
            var sim = new Simulation(plant, new EulerSolver(plant.StateCount), fm.X0, FaultChecks.None);
            Run(sim, s_zeroTorque, h, Steps(0.02, h));
            return MaxAbsDiff(sim.State, reference);
        }

        var ratio = Error(1e-3) / Error(5e-4);
        output.WriteLine(Invariant($"Euler error ratio h = 1 ms / 0.5 ms at 0.02 s: {ratio:F2} (first order: 2)"));
        Assert.InRange(ratio, 1.7, 2.3);
    }
}
