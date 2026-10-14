using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using G = RobotArm.Core.Model.OpenManipulatorXGeometry;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>SPEC §4.1–§4.2 (V2 of SPEC §11): closed-form and frame-chain forward kinematics.</summary>
public sealed class ForwardKinematicsTests
{
    private const double Tol = 1e-12;
    private static readonly ArmModel s_arm = ArmModel.OpenManipulatorX();

    public static TheoryData<int> PoseIndices()
    {
        var data = new TheoryData<int>();
        for (var i = 0; i < TestVectors.Instance.Poses.Count; i++) data.Add(i);
        return data;
    }

    private static void AssertClose(double expected, double actual, double tol, string what) =>
        Assert.True(Math.Abs(actual - expected) <= tol,
            Invariant($"{what}: expected {expected:R}, actual {actual:R}, difference {Math.Abs(actual - expected):E2}"));

    [Theory]
    [MemberData(nameof(PoseIndices))]
    public void ClosedForm_MatchesVectorToolPositionAndPitch(int index)
    {
        var pose = TestVectors.Instance.Poses[index];
        var tool = ForwardKinematics.ClosedForm(pose.Q);

        AssertClose(pose.ToolPosition[0], tool.Position.X, Tol, "x");
        AssertClose(pose.ToolPosition[1], tool.Position.Y, Tol, "y");
        AssertClose(pose.ToolPosition[2], tool.Position.Z, Tol, "z");
        AssertClose(pose.ToolPitch, tool.Pitch, Tol, "pitch");
    }

    [Theory]
    [MemberData(nameof(PoseIndices))]
    public void FrameChain_MatchesVectorToolPosition(int index)
    {
        var pose = TestVectors.Instance.Poses[index];
        var (_, tool) = ForwardKinematics.ComputeFrames(s_arm, pose.Q);

        AssertClose(pose.ToolPosition[0], tool.X, Tol, "x");
        AssertClose(pose.ToolPosition[1], tool.Y, Tol, "y");
        AssertClose(pose.ToolPosition[2], tool.Z, Tol, "z");
    }

    [Fact]
    public void ClosedForm_ZeroPose_IsArmStretchedForwardWithElbowRaised()
    {
        // q = 0: r = a2x + L3 + L4 = 0.274, z = d1 + a2z = 0.1875 (SPEC §3.1, §4.2).
        var tool = ForwardKinematics.ClosedForm([0, 0, 0, 0]);
        AssertClose(0.274, tool.Position.X, Tol, "x");
        AssertClose(0.0, tool.Position.Y, Tol, "y");
        AssertClose(0.1875, tool.Position.Z, Tol, "z");
        Assert.Equal(0.0, tool.Pitch);
    }

    [Fact]
    public void FrameChain_AgreesWithClosedForm_InRandomPoses()
    {
        var rng = new Random(20261015);
        Span<double> q = stackalloc double[4];
        Span<Frame> frames = stackalloc Frame[4];
        for (var n = 0; n < 1000; n++)
        {
            for (var i = 0; i < 4; i++) q[i] = (rng.NextDouble() * 2.0 - 1.0) * Math.PI;

            var closed = ForwardKinematics.ClosedForm(q);
            var tool = ForwardKinematics.ComputeFrames(s_arm, q, frames);

            AssertClose(closed.Position.X, tool.X, Tol, "x");
            AssertClose(closed.Position.Y, tool.Y, Tol, "y");
            AssertClose(closed.Position.Z, tool.Z, Tol, "z");

            // F4 rotation is Rz(q1)·Ry(θ4): the tool x axis points along q1, pitched down by ψ = θ4.
            var expected = Rotations.Rz(q[0]) * Rotations.Ry(closed.Pitch);
            for (var r = 0; r < 3; r++)
                for (var c = 0; c < 3; c++)
                    AssertClose(expected[r, c], frames[3].Rotation[r, c], Tol, Invariant($"R4[{r},{c}]"));
        }
    }

    [Fact]
    public void FrameChain_FrameOriginsFollowTheChainTable()
    {
        double[] q = [0.3, 0.2, -0.4, 0.5];
        var (frames, _) = ForwardKinematics.ComputeFrames(s_arm, q);

        Assert.Equal(4, frames.Length);
        // F1 and F2 origins lie on the joint-1 axis (SPEC §4.1).
        AssertClose(0.0, frames[0].Origin.Length, Tol, "F1 origin");
        AssertClose(0.0, frames[1].Origin.X, Tol, "F2 origin x");
        AssertClose(0.0, frames[1].Origin.Y, Tol, "F2 origin y");
        AssertClose(G.D1, frames[1].Origin.Z, Tol, "F2 origin z");
        // F3 origin is joint 3: at distance L2 from joint 2. F4 origin is joint 4: at distance L3 from joint 3.
        AssertClose(G.L2, (frames[2].Origin - frames[1].Origin).Length, Tol, "|F3 − F2|");
        AssertClose(G.L3, (frames[3].Origin - frames[2].Origin).Length, Tol, "|F4 − F3|");
        // Every rotation is orthonormal.
        foreach (var f in frames)
        {
            var rtr = f.Rotation.Transpose() * f.Rotation;
            for (var r = 0; r < 3; r++)
                for (var c = 0; c < 3; c++)
                    AssertClose(r == c ? 1.0 : 0.0, rtr[r, c], Tol, "RᵀR");
        }
    }

    [Fact]
    public void PositivePitch_TipsTheToolDown()
    {
        var level = ForwardKinematics.ClosedForm([0, 0, 0, 0]);
        var tipped = ForwardKinematics.ClosedForm([0, 0, 0, 0.3]);
        Assert.True(tipped.Position.Z < level.Position.Z);
        Assert.Equal(0.3, tipped.Pitch);
    }

    [Fact]
    public void WrongNumberOfAngles_Throws()
    {
        Assert.Throws<ArgumentException>(() => ForwardKinematics.ClosedForm([0, 0, 0]));
        Assert.Throws<ArgumentException>(() => ForwardKinematics.ComputeFrames(s_arm, [0, 0, 0, 0, 0]));
        Assert.Throws<ArgumentException>(() => ForwardKinematics.ComputeFrames(s_arm, [0, 0, 0, 0], new Frame[3]));
    }

    [Fact]
    public void FrameChain_IgnoresJointLimits()
    {
        // SPEC §3.4: kinematics never rejects a configuration outside the limits.
        double[] q = [0, 3.0, -3.0, 3.0];
        var (_, tool) = ForwardKinematics.ComputeFrames(s_arm, q);
        Assert.True(tool.IsFinite);
    }
}
