using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using G = RobotArm.Core.Model.OpenManipulatorXGeometry;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>SPEC §4.3 (V3 of SPEC §11): closed-form inverse kinematics and the joint-limit option.</summary>
public sealed class InverseKinematicsTests
{
    private const double Tol = 1e-9;
    private const int RandomPoses = 1000;
    private const double MinReach = 0.01; // m; below this q1 is ill-conditioned (SPEC §4.4, r ≈ 0).

    public static TheoryData<int> CaseIndices()
    {
        var data = new TheoryData<int>();
        for (var i = 0; i < TestVectors.Instance.InverseKinematics.Count; i++) data.Add(i);
        return data;
    }

    private static Vector3d Target(InverseKinematicsVectors c) =>
        new(c.TargetPosition[0], c.TargetPosition[1], c.TargetPosition[2]);

    private static void AssertAngles(double[] expected, InverseKinematicsResult result, double tol)
    {
        for (var i = 0; i < 4; i++)
            Assert.True(Math.Abs(result.Q[i] - expected[i]) <= tol,
                Invariant($"q{i + 1}: expected {expected[i]:R}, actual {result.Q[i]:R}, difference {Math.Abs(result.Q[i] - expected[i]):E2}"));
    }

    [Theory]
    [MemberData(nameof(CaseIndices))]
    public void ElbowUp_MatchesVectors_WithLimitsChecked(int index)
    {
        var c = TestVectors.Instance.InverseKinematics[index];
        var result = InverseKinematics.Solve(Target(c), c.TargetPitch, ElbowBranch.Up, checkLimits: true);

        Assert.True(result.Success, Invariant($"failure {result.Failure}"));
        Assert.Equal(FailureReason.None, result.Failure);
        AssertAngles(c.ElbowUp, result, Tol);
    }

    [Theory]
    [MemberData(nameof(CaseIndices))]
    public void ElbowDown_MatchesVectors_WithLimitsIgnored(int index)
    {
        var c = TestVectors.Instance.InverseKinematics[index];
        var result = InverseKinematics.Solve(Target(c), c.TargetPitch, ElbowBranch.Down, checkLimits: false);

        Assert.True(result.Success, Invariant($"failure {result.Failure}"));
        AssertAngles(c.ElbowDown, result, Tol);
    }

    [Theory]
    [MemberData(nameof(CaseIndices))]
    public void ElbowDown_FromVectors_ReportsJointLimit_WithLimitsChecked(int index)
    {
        var c = TestVectors.Instance.InverseKinematics[index];
        var result = InverseKinematics.Solve(Target(c), c.TargetPitch, ElbowBranch.Down);

        Assert.False(result.Success);
        Assert.Equal(FailureReason.JointLimit, result.Failure);
        Assert.InRange(result.LimitJointIndex, 0, 3);
        // The violating solution is still reported, finite, for display.
        AssertAngles(c.ElbowDown, result, Tol);
    }

    [Fact]
    public void CheckLimits_DefaultsToTrue()
    {
        var c = TestVectors.Instance.InverseKinematics[0];
        Assert.Equal(FailureReason.JointLimit, InverseKinematics.Solve(Target(c), c.TargetPitch, ElbowBranch.Down).Failure);
    }

    [Theory]
    [InlineData(1.0, 0.0, 0.0, 0.0)]      // far beyond L2 + L3 + L4
    [InlineData(0.0, 0.0, 2.0, -1.0)]     // far above
    [InlineData(0.126, 0.0, 0.0595, 0.0)] // wrist point on the joint-2 axis: closer than |L2 − L3|
    public void UnreachableTarget_ReturnsFailure_NotNaN(double x, double y, double z, double pitch)
    {
        foreach (var branch in new[] { ElbowBranch.Up, ElbowBranch.Down })
        foreach (var check in new[] { true, false })
        {
            var result = InverseKinematics.Solve(new Vector3d(x, y, z), pitch, branch, check);
            Assert.False(result.Success);
            Assert.Equal(FailureReason.Unreachable, result.Failure);
            for (var i = 0; i < 4; i++) Assert.True(double.IsFinite(result.Q[i]));
        }
    }

    [Theory]
    [InlineData(double.NaN, 0.0, 0.1, 0.0)]
    [InlineData(0.2, double.PositiveInfinity, 0.1, 0.0)]
    [InlineData(0.2, 0.0, 0.1, double.NaN)]
    public void NonFiniteTarget_ReturnsUnreachable_DoesNotThrow(double x, double y, double z, double pitch)
    {
        var result = InverseKinematics.Solve(new Vector3d(x, y, z), pitch, ElbowBranch.Up);
        Assert.Equal(FailureReason.Unreachable, result.Failure);
        for (var i = 0; i < 4; i++) Assert.Equal(0.0, result.Q[i]);
    }

    [Theory]
    [InlineData(ElbowBranch.Up, 1)]
    [InlineData(ElbowBranch.Down, 2)]
    public void RoundTrip_IkOfFk_ReturnsQ_InRandomPosesInsideLimits(ElbowBranch branch, int seed)
    {
        var rng = new Random(seed);
        // Elbow up means q3 ≥ −β, elbow down q3 ≤ −β (SPEC §4.3); sample q3 on the branch's side.
        var q3Lower = branch == ElbowBranch.Up ? -G.Beta : G.Joint3Lower;
        var q3Upper = branch == ElbowBranch.Up ? G.Joint3Upper : -G.Beta;
        double[] q = new double[4];
        var tested = 0;
        while (tested < RandomPoses)
        {
            q[0] = Uniform(rng, G.Joint1Lower, G.Joint1Upper);
            q[1] = Uniform(rng, G.Joint2Lower, G.Joint2Upper);
            q[2] = Uniform(rng, q3Lower, q3Upper);
            q[3] = Uniform(rng, G.Joint4Lower, G.Joint4Upper);

            // Poses that reach back over the base (r ≤ 0) are not IK solutions (SPEC §4.3).
            if (ForwardKinematics.Planar(q).R < MinReach) continue;
            tested++;

            var tool = ForwardKinematics.ClosedForm(q);
            var result = InverseKinematics.Solve(tool.Position, tool.Pitch, branch, checkLimits: true);

            Assert.True(result.Success, Invariant($"q = [{TestFormat.Join(q)}]: failure {result.Failure}"));
            for (var i = 0; i < 4; i++)
                Assert.True(Math.Abs(result.Q[i] - q[i]) <= Tol,
                    Invariant($"q = [{TestFormat.Join(q)}]: q{i + 1} = {result.Q[i]:R}, difference {Math.Abs(result.Q[i] - q[i]):E2}"));
        }
    }

    [Theory]
    [InlineData(ElbowBranch.Up, 3)]
    [InlineData(ElbowBranch.Down, 4)]
    public void OtherBranch_ReachesTheSameTarget_WithLimitsIgnored(ElbowBranch branch, int seed)
    {
        var rng = new Random(seed);
        var other = branch == ElbowBranch.Up ? ElbowBranch.Down : ElbowBranch.Up;
        double[] q = new double[4];
        for (var n = 0; n < RandomPoses; n++)
        {
            q[0] = Uniform(rng, G.Joint1Lower, G.Joint1Upper);
            q[1] = Uniform(rng, G.Joint2Lower, G.Joint2Upper);
            q[2] = Uniform(rng, G.Joint3Lower, G.Joint3Upper);
            q[3] = Uniform(rng, G.Joint4Lower, G.Joint4Upper);
            if (ForwardKinematics.Planar(q).R < MinReach) continue;

            var tool = ForwardKinematics.ClosedForm(q);
            var result = InverseKinematics.Solve(tool.Position, tool.Pitch, other, checkLimits: false);
            Assert.True(result.Success, Invariant($"failure {result.Failure}"));

            var back = ForwardKinematics.ClosedForm(result.Q.ToArray());
            Assert.True((back.Position - tool.Position).Length <= Tol, Invariant($"position error {(back.Position - tool.Position).Length:E2}"));
            Assert.True(Math.Abs(Math.IEEERemainder(back.Pitch - tool.Pitch, 2.0 * Math.PI)) <= Tol);
        }
    }

    [Fact]
    public void JointAngles_CopyToAndIndexer_Agree()
    {
        var c = TestVectors.Instance.InverseKinematics[1];
        var result = InverseKinematics.Solve(Target(c), c.TargetPitch, ElbowBranch.Up);
        Span<double> q = stackalloc double[4];
        result.Q.CopyTo(q);
        for (var i = 0; i < 4; i++) Assert.Equal(result.Q[i], q[i]);
        Assert.Throws<ArgumentOutOfRangeException>(() => result.Q[4]);
    }

    private static double Uniform(Random rng, double lower, double upper) => lower + (upper - lower) * rng.NextDouble();
}
