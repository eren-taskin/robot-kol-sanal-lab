using RobotArm.Core.Model;
using G = RobotArm.Core.Model.OpenManipulatorXGeometry;

namespace RobotArm.Core.Kinematics;

/// <summary>Tool position and pitch in the arm's vertical plane: reach r, height z, pitch ψ. SPEC §4.2.</summary>
/// <param name="R">Signed horizontal reach r of the tool point along the direction q1, m.</param>
/// <param name="Z">Height z of the tool point, m.</param>
/// <param name="Pitch">Tool pitch ψ = θ4 below the horizontal, rad (not wrapped).</param>
public readonly record struct PlanarPose(double R, double Z, double Pitch);

/// <summary>Tool position in the base frame and tool pitch. SPEC §4.2.</summary>
/// <param name="Position">Tool point p = (r cos q1, r sin q1, z), m.</param>
/// <param name="Pitch">Tool pitch ψ = θ4 below the horizontal, rad (not wrapped).</param>
public readonly record struct ToolPose(Vector3d Position, double Pitch);

/// <summary>
/// Forward kinematics in two independent forms that must agree: the closed form of SPEC §4.2 for the
/// OpenMANIPULATOR-X, and the general frame chain of SPEC §4.1 over any <see cref="ArmModel"/>.
/// No method allocates except <see cref="ComputeFrames(ArmModel, ReadOnlySpan{double})"/>.
/// Joint limits are never checked here (SPEC §3.4).
/// </summary>
public static class ForwardKinematics
{
    /// <summary>
    /// Closed form of SPEC §4.2 in the vertical plane: r, z and ψ = θ4, with θ2 = q2, θ3 = q2 + q3,
    /// θ4 = q2 + q3 + q4 (SPEC §2). Uses the OpenMANIPULATOR-X geometry of SPEC §3.1.
    /// </summary>
    /// <param name="q">Joint angles q1..q4, rad. q1 does not enter the planar pose.</param>
    /// <exception cref="ArgumentException"><paramref name="q"/> does not hold 4 angles.</exception>
    public static PlanarPose Planar(ReadOnlySpan<double> q)
    {
        RequireFour(q);
        var theta2 = q[1];
        var theta3 = theta2 + q[2];
        var theta4 = theta3 + q[3];
        var r = G.L2 * Math.Cos(theta2 - G.Beta) + G.L3 * Math.Cos(theta3) + G.L4 * Math.Cos(theta4);
        var z = G.D1 - G.L2 * Math.Sin(theta2 - G.Beta) - G.L3 * Math.Sin(theta3) - G.L4 * Math.Sin(theta4);
        return new PlanarPose(r, z, theta4);
    }

    /// <summary>
    /// Closed form of SPEC §4.2: p = [r cos q1, r sin q1, z] and ψ = θ4. OpenMANIPULATOR-X only.
    /// </summary>
    /// <param name="q">Joint angles q1..q4, rad.</param>
    /// <exception cref="ArgumentException"><paramref name="q"/> does not hold 4 angles.</exception>
    public static ToolPose ClosedForm(ReadOnlySpan<double> q)
    {
        var planar = Planar(q);
        var position = new Vector3d(planar.R * Math.Cos(q[0]), planar.R * Math.Sin(q[0]), planar.Z);
        return new ToolPose(position, planar.Pitch);
    }

    /// <summary>
    /// General frame chain of SPEC §4.1: F1 = Rz(q1), Fi = F(i−1)·T(origin i)·R(axis i, qi), for any
    /// <see cref="ArmModel"/>. Writes frame Fi of body i into <paramref name="frames"/>[i − 1] and returns
    /// the tool point in the base frame. Does not allocate.
    /// </summary>
    /// <param name="model">The arm; its joint table is the chain.</param>
    /// <param name="q">Joint angles, one per joint, rad.</param>
    /// <param name="frames">Output, one frame per joint.</param>
    /// <exception cref="ArgumentNullException"><paramref name="model"/> is null.</exception>
    /// <exception cref="ArgumentException">A span length differs from the number of joints.</exception>
    public static Vector3d ComputeFrames(ArmModel model, ReadOnlySpan<double> q, Span<Frame> frames)
    {
        ArgumentNullException.ThrowIfNull(model);
        var n = model.DegreesOfFreedom;
        if (q.Length != n)
            throw new ArgumentException($"Expected {n} joint angles, got {q.Length} (SPEC §4.1).", nameof(q));
        if (frames.Length != n)
            throw new ArgumentException($"Expected room for {n} frames, got {frames.Length} (SPEC §4.1).", nameof(frames));

        var parent = Frame.Identity;
        for (var i = 0; i < n; i++)
        {
            var joint = model.Joints[i];
            parent = parent.Then(joint.Origin, joint.Axis, q[i]);
            frames[i] = parent;
        }
        return parent.TransformPoint(model.ToolPoint);
    }

    /// <summary>
    /// Allocating convenience form of <see cref="ComputeFrames(ArmModel, ReadOnlySpan{double}, Span{Frame})"/>
    /// for drawing: returns a new array of frames and the tool point. SPEC §4.1.
    /// </summary>
    public static (Frame[] Frames, Vector3d ToolPosition) ComputeFrames(ArmModel model, ReadOnlySpan<double> q)
    {
        ArgumentNullException.ThrowIfNull(model);
        var frames = new Frame[model.DegreesOfFreedom];
        var tool = ComputeFrames(model, q, frames);
        return (frames, tool);
    }

    private static void RequireFour(ReadOnlySpan<double> q)
    {
        if (q.Length != 4)
            throw new ArgumentException($"The closed form needs 4 joint angles, got {q.Length} (SPEC §4.2).", nameof(q));
    }
}
