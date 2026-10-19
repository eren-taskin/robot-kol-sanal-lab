using RobotArm.Core.Control;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using RobotArm.Core.Tasks;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.ControlTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §7 (STSMC) and V44–V48 of SPEC §11: the super-twisting sliding mode controller of docs/DECISIONS.md D13.
/// </summary>
public sealed class SuperTwistingTests(ITestOutputHelper output)
{
    /// <summary>A 1-DOF arm: one yaw joint, no friction, no rotor inertia: M = Izz + m·r² = 0.02 kg·m², h = 0.</summary>
    private static ControllerBelief OneJointBelief()
    {
        var joint = new JointDefinition(new Vector3d(0, 0, 0), JointAxis.Z, -100.0, 100.0);
        var body = new BodyParameters(1.0, new Vector3d(0.1, 0, 0), Matrix3d.Diagonal(0.01, 0.01, 0.01));
        var actuator = new ActuatorParameters { StallTorque = 1000.0, RotorInertia = 0.0, ViscousFriction = 0.0, CoulombFriction = 0.0 };
        return new ControllerBelief(new ArmModel([joint], [body], new Vector3d(0.1, 0, 0), [actuator]), Planet.Orbit);
    }

    private const double OneJointInertia = 0.02;

    /// <summary>
    /// The 1-DOF double integrator J ẍ = u + d(t), d/J = A sin(ωt), regulated to x_d = 0 by STSMC with an exact model,
    /// u held over Ts and the plant integrated by RK4 with 20 sub-steps per sample. Then ṡ = −v − d/J, so the
    /// perturbation's derivative is bounded by L = A·ω. Returns max |s| and max |x| over the last half of the run.
    /// </summary>
    private static (double S, double X) RegulateDoubleIntegrator(double samplePeriod, double duration = 6.0)
    {
        const double a = 5.0, omega = 2.0, lambda = 5.0, l = a * omega;
        var ctl = new SuperTwistingSlidingMode(OneJointBelief(), new SuperTwistingGains([lambda], [1.5 * Math.Sqrt(l)], [1.1 * l]));
        double x = 0.0, xd = 0.0, bandS = 0.0, bandX = 0.0;
        double[] zero = [0.0], q = new double[1], qd = new double[1], u = new double[1];
        var steps = (int)Math.Round(duration / samplePeriod);
        const int sub = 20;
        var h = samplePeriod / sub;
        for (var k = 0; k < steps; k++)
        {
            var t = k * samplePeriod;
            q[0] = x; qd[0] = xd;
            ctl.Compute(t, q, qd, new JointReference(zero, zero, zero), u);
            if (t >= duration / 2)
            {
                bandS = Math.Max(bandS, Math.Abs(ctl.SlidingVariable[0]));
                bandX = Math.Max(bandX, Math.Abs(x));
            }
            var acceleration = u[0] / OneJointInertia;
            for (var j = 0; j < sub; j++)
            {
                var tj = t + j * h;
                double F(double time) => acceleration + a * Math.Sin(omega * time);
                var k1x = xd; var k1v = F(tj);
                var k2x = xd + 0.5 * h * k1v; var k2v = F(tj + 0.5 * h);
                var k3x = xd + 0.5 * h * k2v; var k3v = k2v;
                var k4x = xd + h * k3v; var k4v = F(tj + h);
                x += h / 6.0 * (k1x + 2 * k2x + 2 * k3x + k4x);
                xd += h / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v);
            }
        }
        return (bandS, bandX);
    }

    /// <summary>
    /// V44: on the 1-DOF double integrator with a Lipschitz disturbance (L = 10 rad/s³), STSMC keeps s in a band
    /// that shrinks with Ts. Sampled super-twisting gives |s| = O(Ts²) (Levant): halving Ts should divide the band
    /// by about 4.
    /// </summary>
    [Fact]
    public void V44_DoubleIntegrator_SlidingBandShrinksWithTsSquared()
    {
        var coarse = RegulateDoubleIntegrator(0.002);
        var fine = RegulateDoubleIntegrator(0.001);
        var ratioS = coarse.S / fine.S;
        var ratioX = coarse.X / fine.X;
        output.WriteLine(Invariant($"V44 Ts = 2 ms: max|s| {coarse.S:E3} rad/s, max|x| {coarse.X:E3} rad; Ts = 1 ms: max|s| {fine.S:E3} rad/s, max|x| {fine.X:E3} rad; ratios s {ratioS:F3}, x {ratioX:F3}"));
        Assert.True(coarse.S < 0.01, Invariant($"{coarse.S}"));
        Assert.InRange(ratioS, 3.0, 5.0);
    }

    private static readonly double[] s_q = [0.1, -0.4, 0.3, 0.2];
    private static readonly double[] s_qd = [0.2, -0.1, 0.4, -0.3];
    private static readonly double[] s_ref = [0.15, -0.3, 0.25, 0.4];
    private static readonly double[] s_refVel = [0.1, 0.2, -0.1, 0.0];
    private static readonly double[] s_refAcc = [1.0, -2.0, 0.5, 0.3];

    /// <summary>The law against an independent route (closed-form M, g, C of SPEC §5.3), and w after one more sample.</summary>
    [Theory]
    [InlineData(0.0)] [InlineData(0.5)]
    public void Law_EqualsInverseDynamicsOfTheSuperTwistingAcceleration_AndIntegratesWOncePerSample(double epsilon)
    {
        var belief = ControllerBelief.OpenManipulatorX(Planet.Earth, 0.3);
        var gains = SuperTwistingSlidingMode.DefaultGains;
        var ctl = new SuperTwistingSlidingMode(belief, gains, epsilon);
        var u = new double[4];
        ctl.Compute(0.0, s_q, s_qd, new JointReference(s_ref, s_refVel, s_refAcc), u);
        var closed = new ClosedFormArm(belief.Model);
        var m = closed.InertiaMatrix(s_q); var g = closed.Gravity(s_q, Planet.Earth.G); var c = closed.Coriolis(s_q, s_qd);
        var a = new double[4]; var sliding = new double[4];
        for (var i = 0; i < 4; i++)
        {
            var e = s_ref[i] - s_q[i]; var eDot = s_refVel[i] - s_qd[i];
            sliding[i] = eDot + gains.Lambda[i] * e;
            var phi = epsilon > 0 ? sliding[i] / (Math.Abs(sliding[i]) + epsilon) : Math.Sign(sliding[i]);
            a[i] = s_refAcc[i] + gains.Lambda[i] * eDot + gains.K1[i] * Math.Sqrt(Math.Abs(sliding[i])) * phi; // w = 0 at the first sample
        }
        var actuator = belief.Model.Actuators[0];
        for (var i = 0; i < 4; i++)
        {
            var expected = g[i] + actuator.ViscousFriction * s_qd[i] + actuator.CoulombFriction * Math.Tanh(s_qd[i] / actuator.SmoothingSpeed);
            for (var j = 0; j < 4; j++) expected += m[i, j] * a[j] + c[i, j] * s_qd[j];
            Assert.Equal(expected, u[i], 1e-6);
            Assert.Equal(sliding[i], ctl.SlidingVariable[i], 12);
            Assert.Equal(0.0, ctl.W[i]);
        }
        Assert.DoesNotContain(true, ctl.Clamped.ToArray());
        ctl.Compute(0.002, s_q, s_qd, new JointReference(s_ref, s_refVel, s_refAcc), u);
        for (var i = 0; i < 4; i++)
        {
            var phi = epsilon > 0 ? sliding[i] / (Math.Abs(sliding[i]) + epsilon) : Math.Sign(sliding[i]);
            Assert.Equal(0.002 * gains.K2[i] * phi, ctl.W[i], 12);
        }
        ctl.Reset();
        Assert.All(ctl.W.ToArray(), w => Assert.Equal(0.0, w));
    }

    /// <summary>Anti-windup as CT+I: a clamped joint keeps its old w; it integrates again once unclamped. SPEC §7.</summary>
    [Fact]
    public void AntiWindup_ClampedJointKeepsW_AndIntegratesAgainWhenUnclamped()
    {
        var ctl = new SuperTwistingSlidingMode(ControllerBelief.OpenManipulatorX(Planet.Orbit), SuperTwistingSlidingMode.DefaultGains);
        // e1 = 1e4 rad: k1·|s|^½ ≈ 6500 rad/s², about 16 N·m on the yaw joint, far above τmax = 4.1 N·m.
        double[] q = [0, -0.4, -0.3, 1.1], target = [1e4, -0.4, -0.3, 1.1], zero = new double[4], u = new double[4];
        for (var k = 0; k < 10; k++)
        {
            ctl.Compute(k * 0.002, q, zero, new JointReference(target, zero, zero), u);
            Assert.True(ctl.Clamped[0]); Assert.Equal(0.0, ctl.W[0]);
            Assert.Equal(ctl.Belief.Model.Actuators[0].StallTorque, u[0]);
        }
        target[0] = 1e-4;
        ctl.Compute(0.02, q, zero, new JointReference(target, zero, zero), u);
        Assert.False(ctl.Clamped[0]);
        Assert.Equal(0.002 * SuperTwistingSlidingMode.DefaultGains.K2[0], ctl.W[0], 12);
        var w = ctl.W.ToArray(); ctl.SetPayloadBelief(0.3); Assert.Equal(w, ctl.W.ToArray());
    }

    /// <summary>STSMC has no linearisation at s = 0: it is not an ILinearisableController and the tuner refuses it. SPEC §12.</summary>
    [Fact]
    public void NotLinearisable_AndTheTunerRefusesIt()
    {
        var ctl = new SuperTwistingSlidingMode(ControllerBelief.OpenManipulatorX(Planet.Earth), SuperTwistingSlidingMode.DefaultGains);
        Assert.False(typeof(ILinearisableController).IsAssignableFrom(ctl.GetType()));
        Assert.Throws<NotSupportedException>(() => RobotArm.Core.Analysis.GainTuner.Tune(StsmcExperiment.Library));
    }

    /// <summary>
    /// V45: with the exact model, STSMC (default gains, mid-hold prediction) tracks the V18 quintic (1 s, held to
    /// 1.5 s) within 1e-3 rad, without clamping or fault. Without the prediction it is reported.
    /// </summary>
    [Fact]
    public void V45_ExactBelief_TracksV18QuinticWithin1e3()
    {
        foreach (var predict in new[] { true, false })
        {
            var ctl = new SuperTwistingSlidingMode(ControllerBelief.OpenManipulatorX(Planet.Earth), SuperTwistingSlidingMode.DefaultGains, 0.0, predict);
            double[] start = [0, -0.4, -0.3, 1.1], end = [0, 0.1, -0.8, 0.6];
            var reference = new QuinticReference(start, end, 1.0);
            var loop = Loop(ctl, new PerfectSensor(4), reference, Planet.Earth, 0.0, start);
            double[] qd = new double[4], v = new double[4], a = new double[4];
            var worst = 0.0; var clamped = false;
            while (loop.Time < 1.5 - 1e-9)
            {
                var running = loop.AdvanceSample();
                for (var i = 0; i < 4; i++) clamped |= ctl.Clamped[i];
                reference.Evaluate(loop.Simulation.Time, qd, v, a);
                worst = Math.Max(worst, MaxError(qd, loop.Simulation.Q));
                if (!running) break;
            }
            output.WriteLine(Invariant($"V45 {(predict ? "mid-hold" : "textbook")}: max error {worst:E3} rad, clamped {clamped}, fault {loop.Simulation.Fault?.Reason}"));
            if (!predict) continue;
            Assert.Null(loop.Simulation.Fault); Assert.False(clamped);
            Assert.True(worst < 1e-3, Invariant($"{worst}"));
        }
    }

    private void Print(StsmcComparisonRow r) => output.WriteLine(Invariant(
        $"{r.Case} {r.Controller}: RMS moving [{TestFormat.Join(r.Rms)}]; peak [{TestFormat.Join(r.Peak)}]; final [{TestFormat.Join(r.Final)}]; chatter [{TestFormat.Join(r.Chatter)}]; clamped % [{TestFormat.Join(r.ClampedPercent)}]; fault {r.Fault?.Reason} {r.Fault?.Time} joint {r.Fault?.Joint + 1}"));

    /// <summary>
    /// V46, underestimate (plant 0.3 kg, belief 0 kg), V29 path. Measured (D13): with the standard gains from the
    /// measured L, STSMC does <b>not</b> beat CT+I: w4 ramps at k2 = 1114 rad/s³ while the wrist needs about 1500 rad/s²
    /// from t = 0, so the wrist sags into its upper limit (JointLimit fault on joint 4 at 0.2475 s, a steady drift,
    /// not a chattering knife edge). Its wrist RMS error while moving is 3.3 × CT+I's; asserted ≥ 2 ×.
    /// </summary>
    [Fact]
    public void V46_Underestimate_StandardGainsSagIntoWristLimit_WhileCtIntegralHolds()
    {
        var integral = StsmcExperiment.Compare(true, "CT+I");
        var pure = StsmcExperiment.Compare(true, "STSMC (ε = 0)");
        var smooth = StsmcExperiment.Compare(true, "STSMC (ε > 0)");
        Print(integral); Print(pure); Print(smooth);
        Assert.Null(integral.Fault);
        foreach (var r in new[] { pure, smooth })
        {
            output.WriteLine(Invariant($"{r.Controller}: wrist RMS ratio STSMC / CT+I = {r.Rms[3] / integral.Rms[3]:F3}"));
            Assert.True(r.Rms[3] > 2.0 * integral.Rms[3], Invariant($"{r.Rms[3]} vs {integral.Rms[3]}"));
            Assert.Equal(FaultReason.JointLimit, r.Fault?.Reason);
            Assert.Equal(3, r.Fault!.Joint);
            Assert.True(r.Fault.Time < 0.5, Invariant($"{r.Fault.Time}"));
        }
    }

    /// <summary>
    /// V47, overestimate (plant 0 kg, belief 0.3 kg), V29 path. CT and CT+I chatter between the torque limits
    /// (clamped in most samples, D11 CI-fix); STSMC never clamps and its largest RMS error while moving is about
    /// 2 % of CT+I's (asserted &lt; 10 %). Its fault-freedom is asserted because it never reaches the limits (peak
    /// |u| reported); CT and CT+I faults are not asserted (knife edge, D11).
    /// </summary>
    [Fact]
    public void V47_Overestimate_StstmcTracksWithoutClamping_FarBetterThanCtIntegral()
    {
        var integral = StsmcExperiment.Compare(false, "CT+I");
        Print(StsmcExperiment.Compare(false, "CT")); Print(integral);
        foreach (var name in new[] { "STSMC (ε = 0)", "STSMC (ε > 0)" })
        {
            var r = StsmcExperiment.Compare(false, name);
            var run = new ExperimentRunner().Run(StsmcExperiment.Case(false, name));
            var peak = run.Log.Max(row => Enumerable.Range(0, 4).Max(i => Math.Abs(row.Command[i])));
            Print(r);
            output.WriteLine(Invariant($"{name}: max RMS ratio STSMC / CT+I = {r.Rms.Max() / integral.Rms.Max():F4}; peak |u| {peak:F3} N·m"));
            Assert.Null(r.Fault);
            Assert.Equal(0.0, r.AnyClampedPercent);
            Assert.True(peak < 3.5, Invariant($"{peak}"));
            Assert.True(r.Rms.Max() < 0.1 * integral.Rms.Max(), Invariant($"{r.Rms.Max()} vs {integral.Rms.Max()}"));
        }
    }

    /// <summary>
    /// The default k1, k2 follow from L measured on the library path: δ = M̂⁻¹(τ − ĥ − τ̂f) − q̈ evaluated on the
    /// reference with the torque the true (0.3 kg) plant needs, L = max |Δδ|/Ts (D13). Joint 1 does not move, so it
    /// takes joint 2's L.
    /// </summary>
    [Fact]
    public void DefaultGains_AreTheStandardChoiceForTheMeasuredL()
    {
        var run = new ExperimentRunner().Run(StsmcExperiment.Case(true, "CT+I"));
        var l = StsmcExperiment.Rate(run.Log, StsmcExperiment.Perturbation(run.Log, 0.3, 0.0, onReference: true));
        output.WriteLine(Invariant($"L on the reference [{TestFormat.Join(l)}] rad/s³"));
        Assert.True(l[0] < 1e-9);
        l[0] = l[1];
        var gains = SuperTwistingSlidingMode.DefaultGains;
        for (var i = 0; i < 4; i++)
        {
            Assert.InRange(gains.K1[i] / (1.5 * Math.Sqrt(l[i])), 0.998, 1.002);
            Assert.InRange(gains.K2[i] / (1.1 * l[i]), 0.998, 1.002);
        }
    }

    /// <summary>V48: Compute and normal runner steps allocate nothing; scenario JSON and seed give bit-identical logs.</summary>
    [Fact]
    public void V48_NoAllocation_AndDeterministicWithSeed()
    {
        var ctl = new SuperTwistingSlidingMode(ControllerBelief.OpenManipulatorX(Planet.Earth), SuperTwistingSlidingMode.DefaultGains, 0.5);
        double[] q = [0, -0.4, -0.3, 1.1], target = [0, -0.39, -0.31, 1.11], zero = new double[4], u = new double[4];
        for (var k = 0; k < 100; k++) ctl.Compute(k * 0.002, q, zero, new JointReference(target, zero, zero), u);
        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var k = 100; k < 200; k++) ctl.Compute(k * 0.002, q, zero, new JointReference(target, zero, zero), u);
        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());

        var s = StsmcExperiment.Library with
        {
            PlanetKey = "Orbit", UseEncoder = true, NoiseStdDev = 1e-4, Seed = 7, Duration = 1.0,
            Controller = new ControllerSettings { Method = ControlMethod.STSMC, BoundaryLayer = 0.5, Lambda = new(15, 15, 15, 15) }
        };
        var runner = new ExperimentRunner(); runner.Start(s); runner.AdvanceBy(0.1);
        before = GC.GetAllocatedBytesForCurrentThread();
        runner.AdvanceBy(0.1);
        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());

        var json = ScenarioJson.Serialize(s); var restored = ScenarioJson.Deserialize(json);
        Assert.Equal(s, restored);
        var one = new ExperimentRunner().Run(s); var two = new ExperimentRunner().Run(restored);
        Assert.Equal(one.Log, two.Log);
        Assert.NotEqual(one.Log, new ExperimentRunner().Run(s with { Seed = 8 }).Log);
    }
}
