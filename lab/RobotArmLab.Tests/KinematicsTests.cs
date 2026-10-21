using System.Text.Json;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;

namespace RobotArmLab.Tests;

public class KinematicsTests
{
    private readonly ArmParameters _p = ArmParameters.Default();

    [Fact]
    public void L2AndBetaAreComputedFromA2()
    {
        Assert.Equal(0.130231, _p.L2M, 6);
        Assert.Equal(1.385448, _p.BetaRad, 6);
    }

    [Fact]
    public void ForwardKinematicsMatchesTestVectors()
    {
        foreach (JsonElement pose in TestVectors.Root.GetProperty("poses").EnumerateArray())
        {
            double[] q = TestVectors.Vector(pose.GetProperty("q"));
            double[] expected = TestVectors.Vector(pose.GetProperty("tool_position"));
            Point3 tool = ForwardKinematics.ToolPosition(_p, q);
            Assert.Equal(expected[0], tool.X, 1e-12);
            Assert.Equal(expected[1], tool.Y, 1e-12);
            Assert.Equal(expected[2], tool.Z, 1e-12);
            Assert.Equal(pose.GetProperty("tool_pitch").GetDouble(), ForwardKinematics.ToolPitch(q), 1e-12);
        }
    }

    [Fact]
    public void JointPositionsEndAtToolAndStartAtShoulderHeight()
    {
        double[] q = { 0.4, -0.3, 0.5, 0.2 };
        Point3[] pts = ForwardKinematics.JointPositions(_p, q);
        Assert.Equal(5, pts.Length);
        Assert.Equal(0.0595, pts[1].Z, 12);
        Point3 tool = ForwardKinematics.ToolPosition(_p, q);
        Assert.Equal(tool.X, pts[4].X, 15);
        // Link lengths are preserved: |p3 − p2| = L2, |p4 − p3| = L3, |p5 − p4| = L4.
        Assert.Equal(_p.L2M, Distance(pts[1], pts[2]), 12);
        Assert.Equal(_p.L3M, Distance(pts[2], pts[3]), 12);
        Assert.Equal(_p.L4M, Distance(pts[3], pts[4]), 12);
    }

    [Fact]
    public void InverseKinematicsMatchesTestVectors()
    {
        foreach (JsonElement c in TestVectors.Root.GetProperty("inverse_kinematics").EnumerateArray())
        {
            double[] t = TestVectors.Vector(c.GetProperty("target_position"));
            double pitch = c.GetProperty("target_pitch").GetDouble();

            IkResult up = InverseKinematics.Solve(_p, t[0], t[1], t[2], pitch, true, true);
            Assert.True(up.Success, up.Reason);
            Check.RelativeClose(TestVectors.Vector(c.GetProperty("elbow_up")), up.Q, 1e-9, "elbow up");

            IkResult down = InverseKinematics.Solve(_p, t[0], t[1], t[2], pitch, false, false);
            Assert.True(down.Success, down.Reason);
            Check.RelativeClose(TestVectors.Vector(c.GetProperty("elbow_down")), down.Q, 1e-9, "elbow down");

            IkResult downLimited = InverseKinematics.Solve(_p, t[0], t[1], t[2], pitch, false, true);
            Assert.False(downLimited.Success);
            Assert.Equal(IkFailure.JointLimit, downLimited.Failure);
        }
    }

    [Fact]
    public void InverseThenForwardRoundTripOn200RandomReachableTargets()
    {
        // Targets come from FK of random joint angles inside the limits. Poses that reach back over the
        // base (planar r < 0) are skipped: SPEC §4.3 does not solve that branch.
        Random rng = new Random(1);
        int accepted = 0;
        while (accepted < 200)
        {
            double[] q = Check.RandomQInLimits(rng, _p);
            Point3 target = ForwardKinematics.ToolPosition(_p, q);
            if (target.X * Math.Cos(q[0]) + target.Y * Math.Sin(q[0]) <= 1e-3)
                continue;
            accepted++;
            double pitch = ForwardKinematics.ToolPitch(q);
            foreach (bool elbowUp in new[] { true, false })
            {
                IkResult r = InverseKinematics.Solve(_p, target.X, target.Y, target.Z, pitch, elbowUp, false);
                Assert.True(r.Success, r.Reason);
                Point3 back = ForwardKinematics.ToolPosition(_p, r.Q);
                Assert.True(Distance(target, back) < 1e-9, $"round trip error {Distance(target, back):E3} m");
                Assert.Equal(pitch, ForwardKinematics.ToolPitch(r.Q), 1e-9);
            }
        }
    }

    [Fact]
    public void UnreachableTargetFailsWithReasonAndNoNaN()
    {
        IkResult r = InverseKinematics.Solve(_p, 1.0, 0.0, 0.0, 0.0, true, true);
        Assert.False(r.Success);
        Assert.Equal(IkFailure.Unreachable, r.Failure);
        Assert.False(string.IsNullOrEmpty(r.Reason));
        Assert.Null(r.Q);
    }

    private static double Distance(Point3 a, Point3 b)
    {
        double dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }
}
