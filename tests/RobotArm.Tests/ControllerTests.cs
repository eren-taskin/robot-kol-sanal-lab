using RobotArm.Core.Control;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.ControlTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §7 (controllers) and V17, V18 of SPEC §11: the laws, the clamping and anti-windup of docs/DECISIONS.md D6,
/// the default-gain criterion and the settling table asked for in T07.
/// </summary>
public sealed class ControllerTests(ITestOutputHelper output)
{
    private static readonly double[] s_q = [0.1, -0.4, 0.3, 0.2];
    private static readonly double[] s_qd = [0.2, -0.1, 0.4, -0.3];
    private static readonly double[] s_ref = [0.15, -0.3, 0.25, 0.4];
    private static readonly double[] s_refVel = [0.1, 0.2, -0.1, 0.0];
    private static readonly double[] s_refAcc = [1.0, -2.0, 0.5, 0.3];

    private static JointReference Reference() => new(s_ref, s_refVel, s_refAcc);

    [Fact]
    public void PdGravity_ComputesKpEPlusKdEDotPlusGravityOfItsBelief()
    {
        var belief = ControllerBelief.OpenManipulatorX(Planet.Mars, 0.2);
        var ctl = new PdGravity(belief, PdGravity.DefaultGains);
        var u = new double[4];
        ctl.Compute(0.0, s_q, s_qd, Reference(), u);

        var g = new ClosedFormArm(belief.Model).Gravity(s_q, Planet.Mars.G);
        for (var i = 0; i < 4; i++)
        {
            var expected = PdGravity.DefaultGains.Kp[i] * (s_ref[i] - s_q[i]) + PdGravity.DefaultGains.Kd[i] * (s_refVel[i] - s_qd[i]) + g[i];
            Assert.Equal(expected, u[i], 1e-12);
        }
        Assert.DoesNotContain(true, ctl.Clamped.ToArray());
    }

    [Fact]
    public void ComputedTorque_ComputesInverseDynamicsOfTheCommandedAccelerationPlusFriction()
    {
        var belief = ControllerBelief.OpenManipulatorX(Planet.Earth, 0.3);
        var gains = new PdGains([144.0, 144.0, 144.0, 144.0], [24.0, 24.0, 24.0, 24.0]);
        var ctl = new ComputedTorque(belief, gains);
        var u = new double[4];
        ctl.Compute(0.0, s_q, s_qd, Reference(), u);

        // Independent route: closed-form M and g, and Cq̇ from the Christoffel symbols (SPEC §5.3).
        var closed = new ClosedFormArm(belief.Model);
        var m = closed.InertiaMatrix(s_q);
        var g = closed.Gravity(s_q, Planet.Earth.G);
        var c = closed.Coriolis(s_q, s_qd);
        var a = new double[4];
        for (var i = 0; i < 4; i++)
            a[i] = s_refAcc[i] + gains.Kd[i] * (s_refVel[i] - s_qd[i]) + gains.Kp[i] * (s_ref[i] - s_q[i]);
        var actuator = belief.Model.Actuators[0];
        for (var i = 0; i < 4; i++)
        {
            var expected = g[i] + actuator.ViscousFriction * s_qd[i] + actuator.CoulombFriction * Math.Tanh(s_qd[i] / actuator.SmoothingSpeed);
            for (var j = 0; j < 4; j++)
                expected += m[i, j] * a[j] + c[i, j] * s_qd[j];
            Assert.Equal(expected, u[i], 1e-6);
        }
    }

    [Fact]
    public void Pid_IntegratesTheErrorBetweenSamples_AndHasNoGravityModel()
    {
        var gains = new PidGains([1.0, 2.0, 3.0, 4.0], [0.5, 0.5, 0.5, 0.5], [0.1, 0.1, 0.1, 0.1]);
        var ctl = new Pid(ControllerBelief.OpenManipulatorX(Planet.HighG), gains);
        var u = new double[4];
        ctl.Compute(1.0, s_q, s_qd, Reference(), u); // first sample: Δt = 0, no integral yet
        for (var i = 0; i < 4; i++)
            Assert.Equal(gains.Kp[i] * (s_ref[i] - s_q[i]) + gains.Kd[i] * (s_refVel[i] - s_qd[i]), u[i], 1e-15);

        ctl.Compute(1.002, s_q, s_qd, Reference(), u);
        for (var i = 0; i < 4; i++)
        {
            var e = s_ref[i] - s_q[i];
            Assert.Equal(0.002 * e, ctl.Integral[i], 1e-15);
            Assert.Equal(gains.Kp[i] * e + gains.Ki[i] * 0.002 * e + gains.Kd[i] * (s_refVel[i] - s_qd[i]), u[i], 1e-14);
        }

        ctl.Reset();
        Assert.All(ctl.Integral.ToArray(), v => Assert.Equal(0.0, v));
    }

    /// <summary>
    /// docs/DECISIONS.md D6: the controller clamps to τmax of its own model and stops integrating on a joint it
    /// clamped; it is never told whether the plant saturated.
    /// </summary>
    [Fact]
    public void Pid_ClampsToItsBeliefTorqueLimit_AndStopsIntegratingWhileClamped()
    {
        var arm = ArmModel.OpenManipulatorX();
        var weak = new ArmModel(arm.Joints, arm.Bodies, arm.ToolPoint, arm.Actuators[0] with { StallTorque = 1.0 });
        var ctl = new Pid(new ControllerBelief(weak, Planet.Earth), new PidGains([10.0, 10.0, 10.0, 10.0], [5.0, 5.0, 5.0, 5.0], [0.0, 0.0, 0.0, 0.0]));
        double[] zero = [0.0, 0.0, 0.0, 0.0];
        double[] q = [0.0, 0.0, 0.0, 0.0];
        double[] target = [0.5, 0.05, -0.5, 0.0]; // joints 1 and 3 clamp (|10·0.5| > 1), joint 2 does not
        var reference = new JointReference(target, zero, zero);
        var u = new double[4];

        for (var k = 0; k < 100; k++)
            ctl.Compute(k * 0.002, q, zero, reference, u);

        Assert.Equal(1.0, u[0]);
        Assert.Equal(-1.0, u[2]);
        Assert.Equal([true, false, true, false], ctl.Clamped.ToArray());
        Assert.Equal(0.0, ctl.Integral[0]);
        Assert.Equal(0.0, ctl.Integral[2]);
        Assert.Equal(99 * 0.002 * 0.05, ctl.Integral[1], 1e-14);
        Assert.Equal(10.0 * 0.05 + 5.0 * 99 * 0.002 * 0.05, u[1], 1e-12);

        // Once the error is small enough the joint unclamps and integrates again.
        double[] near = [0.05, 0.05, -0.05, 0.0];
        ctl.Compute(0.2, q, zero, new JointReference(near, zero, zero), u);
        Assert.False(ctl.Clamped[0]);
        Assert.Equal(0.002 * 0.05, ctl.Integral[0], 1e-15);
    }

    [Theory]
    [InlineData(ControllerKind.PdGravity)]
    [InlineData(ControllerKind.ComputedTorque)]
    public void ModelBasedControllers_ClampToTheirBeliefTorqueLimit(ControllerKind kind)
    {
        var arm = ArmModel.OpenManipulatorX();
        var weak = new ArmModel(arm.Joints, arm.Bodies, arm.ToolPoint, arm.Actuators[0] with { StallTorque = 0.5 });
        var ctl = Create(kind, new ControllerBelief(weak, Planet.Earth));
        double[] zero = [0.0, 0.0, 0.0, 0.0];
        double[] q = [0.0, 0.0, 0.0, 0.0];
        double[] target = [1.0, -1.0, 1.0, -1.0];
        var u = new double[4];
        ctl.Compute(0.0, q, zero, new JointReference(target, zero, zero), u);

        Assert.All(u, v => Assert.True(Math.Abs(v) <= 0.5));
        Assert.Contains(true, ctl.Clamped.ToArray());
        ctl.Reset();
        Assert.DoesNotContain(true, ctl.Clamped.ToArray());
    }

    /// <summary>
    /// The effective gains that <see cref="SampledLoopStability"/> uses equal −∂u/∂x of the controller itself,
    /// by central differences at rest with the reference fixed at q0, at the second sample (so that the time since
    /// the previous sample is Ts, which the half-sample prediction of CT uses) (docs/DECISIONS.md D6).
    /// </summary>
    [Theory]
    [InlineData(ControllerKind.Pid, 2e-3)]
    [InlineData(ControllerKind.PdGravity, 2e-3)]
    [InlineData(ControllerKind.ComputedTorque, 2e-3)]
    [InlineData(ControllerKind.ComputedTorque, 0.03)]
    public void LinearFeedbackGain_EqualsDerivativeOfTheControlLaw(ControllerKind kind, double ts)
    {
        var belief = ControllerBelief.OpenManipulatorX(Planet.HighG, 0.4);
        // K of the PID is its PD part (Ki is left out), so the check uses a PID with Ki = 0.
        var ctl = kind == ControllerKind.Pid
            ? new Pid(belief, new PidGains(Pid.DefaultGains.Kp, [0.0, 0.0, 0.0, 0.0], Pid.DefaultGains.Kd))
            : Create(kind, belief);
        var q0 = PoseC;
        var k = Gain(ctl, q0, ts);
        var previous = new double[4];
        double[] zero = [0.0, 0.0, 0.0, 0.0];
        var reference = new JointReference(q0, zero, zero);
        const double delta = 1e-6;
        var plus = new double[4];
        var minus = new double[4];
        var worst = 0.0;

        for (var j = 0; j < 8; j++)
        {
            var q = (double[])q0.Clone();
            var qd = new double[4];
            var x = j < 4 ? q : qd;
            var index = j % 4;
            x[index] += delta;
            ctl.Reset();
            ctl.Compute(0.0, q0, zero, reference, previous);
            ctl.Compute(ts, q, qd, reference, plus);
            x[index] -= 2.0 * delta;
            ctl.Reset();
            ctl.Compute(0.0, q0, zero, reference, previous);
            ctl.Compute(ts, q, qd, reference, minus);
            for (var i = 0; i < 4; i++)
            {
                var derivative = -(plus[i] - minus[i]) / (2.0 * delta);
                worst = Math.Max(worst, Math.Abs(derivative - k[i * 8 + j]) / Math.Max(1.0, Math.Abs(k[i * 8 + j])));
            }
        }
        output.WriteLine(Invariant($"{kind}, Ts = {ts} s: largest relative difference {worst:E2}"));
        Assert.True(worst < 1e-6, Invariant($"{kind}: {worst:E2}"));
    }

    [Fact]
    public void Controllers_RejectGainsForAnotherNumberOfJoints_AndNegativeGains()
    {
        var belief = ControllerBelief.OpenManipulatorX(Planet.Earth);
        Assert.Throws<ArgumentException>(() => new PdGravity(belief, new PdGains([1.0, 1.0], [0.1, 0.1])));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PdGains([1.0, -1.0], [0.1, 0.1]));
        Assert.Throws<ArgumentException>(() => new PidGains([1.0, 1.0], [1.0], [0.1, 0.1]));
    }

    /// <summary>
    /// SPEC §7 criterion for the default gains: Earth, no payload, a 0.5 rad step on each pitch joint (A → B and
    /// B → A) settles within 1 s into the 2 % band (0.01 rad) without plant saturation; PID and PDG also never
    /// clamp their output, while CT's A1c gains clamp briefly at the start of the discontinuous step (docs/DECISIONS.md A1c).
    /// Perfect sensor, default h and Ts.
    /// </summary>
    [Theory]
    [InlineData(ControllerKind.Pid)]
    [InlineData(ControllerKind.PdGravity)]
    [InlineData(ControllerKind.ComputedTorque)]
    public void Spec7_DefaultGains_SettleWithin1s_WithoutSaturation_OnEarthWithoutPayload(ControllerKind kind)
    {
        foreach (var (start, target, name) in new[] { (PoseA, PoseB, "A→B"), (PoseB, PoseA, "B→A") })
        {
            var r = StepResponse(kind, Planet.Earth, 0.0, start, target);
            output.WriteLine(Invariant($"{kind} {name}: settled at {r.SettlingTime:F3} s, max |u| {r.MaxCommand:F2} N·m, final error {r.FinalError:E1} rad"));
            Assert.Null(r.Fault);
            Assert.True(r.Settled);
            Assert.True(r.SettlingTime <= 1.0, Invariant($"{kind} {name}: {r.SettlingTime} s"));
            if (kind != ControllerKind.ComputedTorque)
                Assert.False(r.Clamped);
            Assert.False(r.PlantSaturated);
        }
    }

    /// <summary>V17: PDG, Earth, no payload, 0.5 rad step: final error below 1e-3 rad (perfect sensor, D6).</summary>
    [Fact]
    public void V17_PdGravity_ReachesHalfRadianStep_WithFinalErrorBelow1e3()
    {
        var r = StepResponse(ControllerKind.PdGravity, Planet.Earth, 0.0, PoseA, PoseB, duration: 2.0);
        output.WriteLine(Invariant($"V17: final error after 2 s {r.FinalError:E2} rad, settled at {r.SettlingTime:F3} s"));
        Assert.Null(r.Fault);
        Assert.True(r.FinalError < 1e-3, Invariant($"{r.FinalError}"));
    }

    /// <summary>Start of the V18 quintic: tool well above the floor, 0.3 rad inside every joint limit.</summary>
    private static readonly double[] s_trackStart = [0.0, -0.4, -0.3, 1.1];

    /// <summary>End of the V18 quintic: start + [0, 0.5, −0.5, −0.5] rad.</summary>
    private static readonly double[] s_trackEnd = [0.0, 0.1, -0.8, 0.6];

    /// <summary>
    /// V18: CT with the exact model tracks a quintic (1 s, then hold to 1.5 s) with max error below 1e-3 rad.
    /// With the payload model wrong by 0.3 kg (plant 0.3 kg with belief 0, and plant 0 with belief 0.3 kg) the
    /// error is at least 5 times larger. The path keeps the tool at least 0.15 m above the floor and inside the
    /// joint limits, so every run finishes without a fault. The same three runs with the textbook law (no
    /// half-sample prediction, docs/DECISIONS.md D7) are reported.
    /// </summary>
    [Fact]
    public void V18_ComputedTorque_TracksQuinticWithin1e3_AndPayloadMismatchMakesItClearlyWorse()
    {
        var lowest = LowestToolHeight(s_trackStart, s_trackEnd);
        output.WriteLine(Invariant($"V18 path: lowest tool height {lowest:F3} m"));
        Assert.True(lowest >= 0.15, Invariant($"{lowest}"));
        var arm = ArmModel.OpenManipulatorX();
        for (var i = 0; i < 4; i++)
            foreach (var q in new[] { s_trackStart[i], s_trackEnd[i] })
                Assert.True(arm.Joints[i].IsWithinLimits(q));

        foreach (var predict in new[] { true, false })
        {
            var exact = TrackQuintic(truePayload: 0.0, believedPayload: 0.0, predict);
            var forgotten = TrackQuintic(truePayload: 0.3, believedPayload: 0.0, predict);
            var imagined = TrackQuintic(truePayload: 0.0, believedPayload: 0.3, predict);
            var law = predict ? "mid-hold (default)" : "textbook";
            output.WriteLine(Invariant($"V18 {law}, (a) plant 0 kg,   belief 0 kg:   max error {exact.Error:E3} rad{Stop(exact)}"));
            output.WriteLine(Invariant($"V18 {law}, (b) plant 0.3 kg, belief 0 kg:   max error {forgotten.Error:E3} rad ({forgotten.Error / exact.Error:F0}×){Stop(forgotten)}"));
            output.WriteLine(Invariant($"V18 {law}, (c) plant 0 kg,   belief 0.3 kg: max error {imagined.Error:E3} rad ({imagined.Error / exact.Error:F0}×){Stop(imagined)}"));
            if (!predict)
                continue;

            Assert.Null(exact.Fault);
            Assert.Null(forgotten.Fault);
            Assert.Null(imagined.Fault);
            Assert.True(exact.Error < 1e-3, Invariant($"{exact.Error}"));
            Assert.True(forgotten.Error >= 5.0 * exact.Error, Invariant($"{forgotten.Error} vs {exact.Error}"));
            Assert.True(imagined.Error >= 5.0 * exact.Error, Invariant($"{imagined.Error} vs {exact.Error}"));
        }

        static string Stop((double Error, Fault? Fault) r) =>
            r.Fault is null ? "" : Invariant($" until a {r.Fault.Reason} fault at t = {r.Fault.Time:F3} s");
    }

    /// <summary>Largest joint error while tracking the V18 quintic, up to 1.5 s or the fault that stopped the run.</summary>
    private static (double Error, Fault? Fault) TrackQuintic(double truePayload, double believedPayload, bool predictHalfSample)
    {
        var ctl = new ComputedTorque(ControllerBelief.OpenManipulatorX(Planet.Earth, believedPayload), ComputedTorque.DefaultGains, predictHalfSample);
        var reference = new QuinticReference(s_trackStart, s_trackEnd, 1.0);
        var loop = Loop(ctl, new PerfectSensor(4), reference, Planet.Earth, truePayload, s_trackStart);
        var qd = new double[4];
        var v = new double[4];
        var a = new double[4];
        var worst = 0.0;
        while (loop.Time < 1.5 - 1e-9)
        {
            var running = loop.AdvanceSample();
            reference.Evaluate(loop.Simulation.Time, qd, v, a);
            worst = Math.Max(worst, MaxError(qd, loop.Simulation.Q));
            if (!running) break;
        }
        return (worst, loop.Simulation.Fault);
    }

    /// <summary>Lowest tool height along the quintic from start to end, checked at 201 points (SPEC §4.2).</summary>
    private static double LowestToolHeight(double[] start, double[] end)
    {
        var arm = ArmModel.OpenManipulatorX();
        var reference = new QuinticReference(start, end, 1.0);
        var frames = new Frame[4];
        var q = new double[4];
        var v = new double[4];
        var a = new double[4];
        var lowest = double.PositiveInfinity;
        for (var k = 0; k <= 200; k++)
        {
            reference.Evaluate(k / 200.0, q, v, a);
            lowest = Math.Min(lowest, ForwardKinematics.ComputeFrames(arm, q, frames).Z);
        }
        return lowest;
    }

    /// <summary>
    /// The table asked for in T07 (conductor's item E): settling time of the §7 step with the default gains on
    /// every planet with 0 and 0.5 kg, both directions, and which runs saturate or fault. Printed, and the
    /// Earth/0 kg row asserted by the criterion test above.
    /// </summary>
    [Fact]
    public void Report_SettlingTimes_AllPlanetsAndPayloads()
    {
        output.WriteLine("controller      planet  payload  A→B settle  B→A settle  (C = controller clamped, F = fault)");
        foreach (var kind in Enum.GetValues<ControllerKind>())
            foreach (var planet in Planet.All)
                foreach (var payload in Payloads)
                {
                    var down = StepResponse(kind, planet, payload, PoseA, PoseB);
                    var up = StepResponse(kind, planet, payload, PoseB, PoseA);
                    output.WriteLine(Invariant($"{kind,-15} {planet.Key,-7} {payload,4:F1} kg  {Cell(down),-11} {Cell(up),-11}"));
                }

        static string Cell(StepResult r) =>
            r.Fault is not null ? Invariant($"F {r.Fault.Reason}{(r.Fault.Joint >= 0 ? $" j{r.Fault.Joint + 1}" : "")}")
            : (r.Settled ? Invariant($"{r.SettlingTime:F2}") : ">2") + (r.Clamped ? " C" : "") + (r.PlantSaturated ? " S" : "");
    }
}
