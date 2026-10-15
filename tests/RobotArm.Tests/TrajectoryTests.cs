using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using RobotArm.Core.Tasks;

namespace RobotArm.Tests;

public sealed class TrajectoryTests
{
    [Fact]
    public void Quintic_ClonesInputs_AndHasZeroEndpointSpeedAndAcceleration()
    {
        double[] start = [0, -0.4, -0.3, 1.1], end = [0, 0.1, -0.8, 0.6];
        var path = new QuinticJointTrajectory(start, end, 2);
        start[0] = 99; end[0] = 99;
        var q = new double[4]; var v = new double[4]; var a = new double[4];
        foreach (var t in new[] { -1.0, 0.0, 2.0, 3.0 })
        {
            path.Evaluate(t, q, v, a);
            Assert.Equal(0, q[0]); Assert.All(v, x => Assert.Equal(0, x)); Assert.All(a, x => Assert.Equal(0, x));
            Assert.Equal(t <= 0 ? -0.4 : 0.1, q[1], 12);
        }
        path.Evaluate(1, q, v, a);
        Assert.Equal(-0.15, q[1], 12); Assert.Equal(0.46875, v[1], 12); Assert.Equal(0, a[1], 12);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void Cartesian_IKPositionAndAnalyticDerivativesMatchPathAndFiniteDifferences(bool circle)
    {
        var start = new Vector3d(0.25, 0.01, 0.22); var end = new Vector3d(0.23, 0.03, 0.23);
        RobotArm.Core.Control.IReferenceSource path = circle
            ? new CartesianCircleTrajectory(start - new Vector3d(0.01, 0, 0), 0.01, 0, 3)
            : new CartesianLineTrajectory(start, end, 0, 3);
        var q = new double[4]; var v = new double[4]; var a = new double[4];
        var qm = new double[4]; var qp = new double[4]; var vm = new double[4]; var vp = new double[4]; var scratch = new double[4];
        for (var k = 1; k < 30; k++)
        {
            var t = k * 0.1; const double eps = 1e-5;
            path.Evaluate(t, q, v, a); path.Evaluate(t - eps, qm, vm, scratch); path.Evaluate(t + eps, qp, vp, scratch);
            for (var i = 0; i < 4; i++)
            {
                Assert.InRange(Math.Abs(v[i] - (qp[i] - qm[i]) / (2 * eps)), 0, 1e-7);
                Assert.InRange(Math.Abs(a[i] - (vp[i] - vm[i]) / (2 * eps)), 0, 1e-7);
            }
            var p = ForwardKinematics.ClosedForm(q);
            Assert.Equal(0, p.Pitch, 12);
            if (circle) Assert.Equal(0.01, (p.Position - (start - new Vector3d(0.01, 0, 0))).Length, 12);
            else Assert.InRange(Vector3d.Cross(p.Position - start, end - start).Length, 0, 1e-12);
        }
        foreach (var t in new[] { 0.0, 3.0, 4.0 })
        { path.Evaluate(t, q, v, a); Assert.All(v, x => Assert.Equal(0, x, 12)); Assert.All(a, x => Assert.Equal(0, x, 12)); }
    }

    [Fact]
    public void Cartesian_UnreachableAndSingularPointsFailCleanly()
    {
        var q = new double[4]; var v = new double[4]; var a = new double[4];
        var line = new CartesianLineTrajectory(new(0.25, 0, 0.22), new(4, 0, 0.22), 0, 1);
        line.Evaluate(0, q, v, a);
        Assert.Throws<TrajectoryException>(() => line.Evaluate(1, q, v, a));
        var g = OpenManipulatorXGeometry.L2 + OpenManipulatorXGeometry.L3 + OpenManipulatorXGeometry.L4;
        var singular = new CartesianLineTrajectory(new(g, 0, OpenManipulatorXGeometry.D1), new(g, 0, OpenManipulatorXGeometry.D1), 0, 1);
        Assert.Throws<TrajectoryException>(() => singular.Evaluate(0, q, v, a));
    }

    [Fact]
    public void Cartesian_YawBranchCutIsRejectedInsteadOfAReferenceStep()
    {
        Assert.Throws<TrajectoryException>(() => new CartesianLineTrajectory(new(-0.2, 0.01, 0.2), new(-0.2, -0.01, 0.2), 0, 1));
        Assert.Throws<TrajectoryException>(() => new CartesianCircleTrajectory(new(-0.2, 0, 0.2), 0.02, 0, 1));
    }

    [Fact]
    public void Ik_DefaultIsFailure_AndRoundoffAtReachBoundaryIsAccepted()
    {
        Assert.False(default(InverseKinematicsResult).Success);
        var q = new[] { 0.0, OpenManipulatorXGeometry.Beta, -OpenManipulatorXGeometry.Beta, 0.0 };
        var p = ForwardKinematics.ClosedForm(q);
        var result = InverseKinematics.Solve(p.Position, p.Pitch, ElbowBranch.Up);
        Assert.True(result.Success);
        Assert.False(InverseKinematics.Solve(p.Position + new Vector3d(1e-6, 0, 0), p.Pitch, ElbowBranch.Up).Success);
    }

    [Fact]
    public void ReachAndPickPlace_ReferencesStayContinuousAtLegAndPayloadBoundaries()
    {
        var result = new ExperimentRunner().Run(ScenarioLibrary.EarthPickPlace);
        Assert.True(result.Success, result.FailureReason);
        for (var k = 1; k < result.Log.Count; k++)
        {
            var previous = result.Log[k - 1]; var current = result.Log[k];
            for (var i = 0; i < 4; i++) Assert.InRange(Math.Abs(current.Reference[i] - previous.Reference[i]), 0, 0.003);
            if (previous.PayloadAttached != current.PayloadAttached)
                for (var i = 0; i < 4; i++)
                {
                    Assert.InRange(Math.Abs(current.ReferenceVelocity[i]), 0, 1e-10);
                    Assert.InRange(Math.Abs(current.ReferenceAcceleration[i]), 0, 1e-10);
                }
        }
    }
}
