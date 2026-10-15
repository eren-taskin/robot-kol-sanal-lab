using RobotArm.Core.Logging;
using RobotArm.Core.Model;
using RobotArm.Core.Tasks;

namespace RobotArm.Tests;

public sealed class MetricsTests
{
    [Fact]
    public void Metrics_UseTrueStateAndActualIntervals_MatchHandCalculation()
    {
        var target = new Vector3d(1, 0, 0);
        var baseRow = new LogRow { Reference = new(2, 0, 0, 0), QDot = new(2, 0, 0, 0), AppliedTorque = new(3, 0, 0, 0), ReferenceTool = target };
        LogRow[] log =
        [
            baseRow with { Time = 0, Interval = 0.2, Q = new(0, 0, 0, 0), Tool = new(0.996, 0, 0), ClampedMask = 1 },
            baseRow with { Time = 0.2, Interval = 0.3, Q = new(1, 0, 0, 0), Tool = new(1.001, 0, 0), SaturatedMask = 2, PayloadAttached = true, PayloadKg = 0.3 },
            baseRow with { Time = 0.5, Interval = 0.5, Q = new(2, 0, 0, 0), Tool = new(1.001, 0, 0), ClampedMask = 2 }
        ];
        var m = RunMetrics.Compute(log, target, 0, 0.5);
        Assert.Equal(Math.Sqrt(1.1), m.RmsJointError.J1, 12); Assert.Equal(2, m.MaxJointError.J1);
        Assert.Equal(0.002, m.RmsToolError, 12); Assert.Equal(0.004, m.MaxToolError, 12); Assert.Equal(0.001, m.FinalToolError, 12);
        Assert.Equal(0.2, m.SettlingTime); Assert.Equal(0.001, m.Overshoot, 12); Assert.Equal(3, m.PeakTorque.J1);
        Assert.Equal(20, m.ClampedPercent.J1); Assert.Equal(50, m.ClampedPercent.J2); Assert.Equal(30, m.SaturatedPercent.J2);
        Assert.Equal(70, m.AnyClampedPercent); Assert.Equal(6, m.MechanicalEnergy);
        Assert.Equal(Math.Sqrt((0.004 * 0.004 * 0.2 + 0.001 * 0.001 * 0.3) / 0.5), m.TrackingRmsToolError, 12);
        Assert.Equal(1, m.PayloadAttachments); Assert.Equal(1, m.PayloadReleases); Assert.Equal(0.3, m.MaxPayloadKg);
        log[2] = log[2] with { Tool = new(1.003, 0, 0) };
        Assert.Null(RunMetrics.Compute(log, target, 0, 0.5).SettlingTime);
    }

    [Fact]
    public void Settling_ExcludesMovingReference_AndZeroLengthOvershootIsZero()
    {
        LogRow[] log = [ new() { Time = 0, Interval = 1, Tool = new(0, 0, 0) }, new() { Time = 1, Interval = 1, Tool = new(0, 0, 0) } ];
        Assert.Equal(1, RunMetrics.Compute(log, Vector3d.Zero, 1, 1).SettlingTime);
        Assert.Equal(0, RunMetrics.Compute(log, Vector3d.Zero, 1, 1).Overshoot);
    }
}
