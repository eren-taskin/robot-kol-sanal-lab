using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>SPEC §12.5: linear step response against the nonlinear simulation (V40).</summary>
public sealed class LinearStepResponseTests(ITestOutputHelper output)
{
    private const double Ts = ControlLoop.DefaultSamplePeriod;
    private const double Duration = 2.0;

    private enum Law { Pdg, Ct, CtPlusI }

    /// <summary>The OpenMANIPULATOR-X with Coulomb friction replaced by its slope at rest: fv' = fv + fc/ωs, fc' = 0.</summary>
    private static ArmModel LinearFrictionArm()
    {
        var arm = ArmModel.OpenManipulatorX();
        return new ArmModel(arm.Joints, arm.Bodies, arm.ToolPoint, arm.Actuators.Select(a =>
            a with { ViscousFriction = a.ViscousFriction + a.CoulombFriction / a.SmoothingSpeed, CoulombFriction = 0.0 }));
    }

    private static IController Make(Law law, ArmModel arm)
    {
        var belief = new ControllerBelief(arm, Planet.Earth);
        return law switch
        {
            Law.Pdg => new PdGravity(belief, PdGravity.DefaultGains),
            Law.Ct => new ComputedTorque(belief, ComputedTorque.DefaultGains),
            _ => new ComputedTorque(belief, ComputedTorque.DefaultIntegralPdGains, integralGains: ComputedTorque.DefaultIntegralGains),
        };
    }

    /// <summary>
    /// Step of every joint's reference by δ from rest at the library start pose (Earth, exact belief, perfect
    /// sensor, default h and Ts), compared sample by sample with the nonlinear RK4 control loop over 2 s. Returns the
    /// largest |q_nonlinear − q_linear| over joints and samples, relative to δ.
    /// </summary>
    private double Compare(Law law, double delta, ArmModel? arm = null)
    {
        arm ??= ArmModel.OpenManipulatorX();
        var q0 = AnalysisFixture.StartPose;
        var step = new[] { delta, delta, delta, delta };
        var linear = LinearStepResponse.Compute(arm, Planet.Earth, 0.0, q0, (ILinearisableController)Make(law, arm), Ts, step, Duration);
        var plant = new ArmPlant(arm, Planet.Earth);
        var x0 = new double[8];
        q0.CopyTo(x0, 0);
        var sim = new Simulation(plant, new Rk4Solver(8), x0, FaultChecks.All);
        var loop = new ControlLoop(sim, Make(law, arm), new PerfectSensor(4), new ConstantReference(q0.Zip(step, (a, b) => a + b).ToArray()));

        var worst = 0.0;
        var worstTime = 0.0;
        var peakSpeed = 0.0;
        var clamped = false;
        for (var s = 0; s < linear.SampleCount; s++)
        {
            for (var i = 0; i < 4; i++)
            {
                var e = Math.Abs(sim.Q[i] - linear.Position(s, i));
                if (e > worst) (worst, worstTime) = (e, s * Ts);
                peakSpeed = Math.Max(peakSpeed, Math.Abs(sim.QDot[i]));
            }
            if (s == linear.SampleCount - 1) break;
            Assert.True(loop.AdvanceSample(), "the nonlinear run faulted");
            for (var i = 0; i < 4; i++) clamped |= loop.Controller.Clamped[i];
        }
        var relative = worst / delta;
        output.WriteLine(Invariant(
            $"{law}{(arm.Actuators[0].CoulombFriction == 0 ? " (linear friction)" : "")}, step {delta} rad on every joint: largest |q_nl − q_lin| = {worst:E3} rad at t = {worstTime:F3} s = {100 * relative:F3} % of the step; peak |q̇| {peakSpeed:F4} rad/s{(clamped ? "; controller clamped" : "")}"));
        return relative;
    }

    /// <summary>
    /// Computed torque cancels inertia, gravity and friction with its model, so its loop stays close to linear: a
    /// 0.01 rad step agrees within 2 % for CT (8 states) and CT+I (12 states with integrators). The 0.5 rad step is
    /// reported.
    /// </summary>
    [Fact]
    public void SmallStep_ComputedTorqueAndCtPlusI_AgreeWithin2Percent()
    {
        var ct = Compare(Law.Ct, 0.01);
        var cti = Compare(Law.CtPlusI, 0.01);
        var ctLarge = Compare(Law.Ct, 0.5);
        Assert.True(ct < 0.02, Invariant($"CT 0.01 rad: {100 * ct:F3} %"));
        Assert.True(cti < 0.02, Invariant($"CT+I 0.01 rad: {100 * cti:F3} %"));
        Assert.True(ctLarge > ct);
    }

    /// <summary>
    /// PDG leaves the plant's friction in the loop. The linear model replaces fc·tanh(q̇/ωs) by its slope fc/ωs, which
    /// holds only while |q̇| ≪ ωs = 0.1 rad/s. A 0.01 rad step on every joint reaches 0.12 rad/s on the yaw joint,
    /// where tanh is already well below its slope, so the prediction is off by about 3 % (reported, docs/DECISIONS.md
    /// D9). With the friction made linear (same slope) the same step agrees within 2 %, and so does a 0.003 rad step
    /// on the true plant: the linearisation is right, the friction band limits its range. The 0.5 rad step is reported.
    /// </summary>
    [Fact]
    public void SmallStep_Pdg_AgreesWithin2Percent_InsideTheFrictionBand()
    {
        var linearFriction = Compare(Law.Pdg, 0.01, LinearFrictionArm());
        var smaller = Compare(Law.Pdg, 0.003);
        var trueSmall = Compare(Law.Pdg, 0.01);
        var trueLarge = Compare(Law.Pdg, 0.5);
        Assert.True(linearFriction < 0.02, Invariant($"PDG, linear friction, 0.01 rad: {100 * linearFriction:F3} %"));
        Assert.True(smaller < 0.02, Invariant($"PDG 0.003 rad: {100 * smaller:F3} %"));
        Assert.True(trueSmall < 0.05, Invariant($"PDG 0.01 rad: {100 * trueSmall:F3} %"));
        Assert.True(trueSmall > 10 * linearFriction, "the remaining error at 0.01 rad comes from the friction band");
        Assert.True(trueLarge > trueSmall);
    }

    /// <summary>With gravity compensated exactly, PDG's linear loop has zero static error: the response ends at q0 + δr.</summary>
    [Fact]
    public void LinearPdg_EndsAtTheReference()
    {
        var q0 = AnalysisFixture.StartPose;
        double[] step = [0.01, -0.02, 0.03, -0.01];
        var r = LinearStepResponse.Compute(ArmModel.OpenManipulatorX(), Planet.Earth, 0.0, q0, AnalysisFixture.DefaultPdg(), Ts, step, 10.0);
        Assert.Equal(5001, r.SampleCount);
        for (var i = 0; i < 4; i++)
        {
            Assert.Equal(q0[i], r.Position(0, i));
            Assert.Equal(q0[i] + step[i], r.Position(r.SampleCount - 1, i), 1e-8);
        }
        Assert.Throws<ArgumentException>(() => LinearStepResponse.Compute(OneDof.Model(), [0.0, 0.0], [0.1], 1.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => LinearStepResponse.Compute(OneDof.Model(), [0.0], [0.1], 0.0));
    }
}
