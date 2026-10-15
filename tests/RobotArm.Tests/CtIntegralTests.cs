using RobotArm.Core.Control;
using RobotArm.Core.Model;
using RobotArm.Core.Tasks;

namespace RobotArm.Tests;

public sealed class CtIntegralTests
{
    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void OptionalIntegral_ConditionalIntegrationAndReset_WorkWithOrWithoutPrediction(bool prediction)
    {
        var c = new ComputedTorque(ControllerBelief.OpenManipulatorX(Planet.Orbit), ComputedTorque.DefaultGains, prediction, ComputedTorque.DefaultIntegralGains);
        double[] q = [0, -0.4, -0.3, 1.1], target = [100, -0.4, -0.3, 1.1], zero = new double[4], u = new double[4];
        for (var k = 0; k < 10; k++)
        { c.Compute(k * 0.002, q, zero, new(target, zero, zero), u); Assert.True(c.Clamped[0]); Assert.Equal(0, c.Integral[0]); }
        target[0] = 0.001;
        c.Compute(0.02, q, zero, new(target, zero, zero), u);
        Assert.False(c.Clamped[0]); Assert.Equal(0.000002, c.Integral[0], 12);
        var old = c.Integral.ToArray();
        c.SetPayloadBelief(0.3); Assert.Equal(old, c.Integral.ToArray());
        c.Reset(); Assert.All(c.Integral.ToArray(), x => Assert.Equal(0, x));
        c.Compute(1, q, zero, new(target, zero, zero), u); Assert.All(c.Integral.ToArray(), x => Assert.Equal(0, x));
    }

    [Fact]
    public void IntegralOffByDefault_PreservesCtLaw_AndComputeAllocatesNothing()
    {
        var belief = ControllerBelief.OpenManipulatorX(Planet.Earth);
        var plain = new ComputedTorque(belief, ComputedTorque.DefaultGains);
        var zeroGain = new ComputedTorque(belief, ComputedTorque.DefaultGains, integralGains: new double[4]);
        var integral = new ComputedTorque(belief, ComputedTorque.DefaultGains, integralGains: ComputedTorque.DefaultIntegralGains);
        Assert.False(plain.IntegralEnabled);
        double[] q = [0, -0.4, -0.3, 1.1], target = [0, -0.39, -0.31, 1.11], zero = new double[4], u = new double[4], other = new double[4];
        for (var k = 0; k < 100; k++)
        {
            plain.Compute(k * 0.002, q, zero, new(target, zero, zero), u);
            zeroGain.Compute(k * 0.002, q, zero, new(target, zero, zero), other); Assert.Equal(u, other);
            integral.Compute(k * 0.002, q, zero, new(target, zero, zero), u);
        }
        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var k = 100; k < 200; k++) integral.Compute(k * 0.002, q, zero, new(target, zero, zero), u);
        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }

    [Fact]
    public void Pid_InformedPayloadUpdatePreservesIntegralAndSampleHistory()
    {
        var pid = new Pid(ControllerBelief.OpenManipulatorX(Planet.Orbit), Pid.DefaultGains);
        var q = new double[4]; var target = new[] { 0.001, 0.0, 0.0, 0.0 }; var u = new double[4];
        pid.Compute(0, q, q, new(target, q, q), u);
        pid.Compute(0.002, q, q, new(target, q, q), u);
        var integral = pid.Integral[0];
        pid.SetPayloadBelief(0.3); Assert.Equal(0.3, pid.Belief.PayloadKg); Assert.Equal(integral, pid.Integral[0]);
        pid.Compute(0.004, q, q, new(target, q, q), u);
        Assert.Equal(2 * integral, pid.Integral[0], 12);
    }

    [Fact]
    public void Runner_NormalStepsAndLoggingAllocateNothingAfterStart()
    {
        var r = new ExperimentRunner(); r.Start(new Scenario { Controller = new ControllerSettings { IntegralEnabled = true } });
        r.AdvanceBy(0.1);
        var before = GC.GetAllocatedBytesForCurrentThread();
        r.AdvanceBy(0.1);
        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }
}
