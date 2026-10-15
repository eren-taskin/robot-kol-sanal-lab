using RobotArm.Core.Model;
using G = RobotArm.Core.Model.OpenManipulatorXGeometry;

namespace RobotArm.Core.Kinematics;

/// <summary>Which of the two planar solutions inverse kinematics returns. SPEC §4.3.</summary>
public enum ElbowBranch
{
    /// <summary>Elbow up: φ = −arccos(cos φ), so q3 ≥ −β.</summary>
    Up,

    /// <summary>Elbow down: φ = +arccos(cos φ), so q3 ≤ −β.</summary>
    Down,
}

/// <summary>Why inverse kinematics failed. SPEC §4.3.</summary>
public enum FailureReason
{
    /// <summary>No failure.</summary>
    None,

    /// <summary>The target is outside the workspace (|cos φ| &gt; 1) or not a finite number.</summary>
    Unreachable,

    /// <summary>A solution exists but violates a joint limit of SPEC §3.4.</summary>
    JointLimit,
}

/// <summary>
/// Result of <see cref="InverseKinematics.Solve"/>: success flag, joint angles q1..q4 and the reason for
/// failure. SPEC §4.3. A value type, so solving allocates nothing. Angles are never NaN.
/// </summary>
public readonly struct InverseKinematicsResult
{
    private readonly double _q1, _q2, _q3, _q4;
    private readonly bool _initialized;
    private readonly FailureReason _failure;

    private InverseKinematicsResult(FailureReason reason, double q1, double q2, double q3, double q4, int limitJoint)
    {
        _failure = reason;
        _initialized = true;
        _q1 = q1; _q2 = q2; _q3 = q3; _q4 = q4;
        LimitJointIndex = limitJoint;
    }

    /// <summary>True when <see cref="Q"/> is a valid solution.</summary>
    public bool Success => Failure == FailureReason.None;

    /// <summary>Why the solve failed, or <see cref="FailureReason.None"/>.</summary>
    public FailureReason Failure => _initialized ? _failure : FailureReason.Unreachable;

    /// <summary>
    /// Zero-based index of the first joint outside its limits when <see cref="Failure"/> is
    /// <see cref="FailureReason.JointLimit"/>, otherwise −1.
    /// </summary>
    public int LimitJointIndex { get; }

    /// <summary>
    /// Joint angles q1..q4 in rad, zero-based (Q[0] is joint 1). On success the solution. On
    /// <see cref="FailureReason.JointLimit"/> the computed solution that violates the limits, for display.
    /// On <see cref="FailureReason.Unreachable"/> all zero.
    /// </summary>
    public JointAngles4 Q => new(_q1, _q2, _q3, _q4);

    internal static InverseKinematicsResult Solved(double q1, double q2, double q3, double q4) =>
        new(FailureReason.None, q1, q2, q3, q4, -1);

    internal static InverseKinematicsResult OutsideLimits(double q1, double q2, double q3, double q4, int joint) =>
        new(FailureReason.JointLimit, q1, q2, q3, q4, joint);

    internal static InverseKinematicsResult Unreachable => new(FailureReason.Unreachable, 0, 0, 0, 0, -1);
}

/// <summary>Four joint angles in rad, zero-based by index. Value type; does not allocate. SPEC §2.</summary>
public readonly struct JointAngles4
{
    /// <summary>Number of angles, 4.</summary>
    public const int Length = 4;

    private readonly double _q1, _q2, _q3, _q4;

    /// <summary>Creates the angles q1..q4.</summary>
    public JointAngles4(double q1, double q2, double q3, double q4)
    {
        _q1 = q1; _q2 = q2; _q3 = q3; _q4 = q4;
    }

    /// <summary>Angle of joint index + 1 (zero-based, SPEC §2).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is not 0..3.</exception>
    public double this[int index] => index switch
    {
        0 => _q1,
        1 => _q2,
        2 => _q3,
        3 => _q4,
        _ => throw new ArgumentOutOfRangeException(nameof(index), index, "Joint index must be 0..3."),
    };

    /// <summary>Copies the angles into <paramref name="destination"/>, which needs room for 4.</summary>
    /// <exception cref="ArgumentException">The destination is shorter than 4.</exception>
    public void CopyTo(Span<double> destination)
    {
        if (destination.Length < Length)
            throw new ArgumentException($"Destination needs room for {Length} angles.", nameof(destination));
        destination[0] = _q1;
        destination[1] = _q2;
        destination[2] = _q3;
        destination[3] = _q4;
    }

    /// <summary>A new array [q1, q2, q3, q4].</summary>
    public double[] ToArray() => [_q1, _q2, _q3, _q4];
}

/// <summary>
/// Closed-form inverse kinematics of the OpenMANIPULATOR-X, SPEC §4.3 [eqs. 5–7]. Uses the geometry of
/// SPEC §3.1 and the joint limits of SPEC §3.4. Never returns NaN and never throws for an unreachable or
/// non-finite target. Does not allocate.
/// </summary>
public static class InverseKinematics
{
    /// <summary>
    /// Joint angles that put the tool point at <paramref name="position"/> with tool pitch
    /// <paramref name="pitch"/> on the chosen <paramref name="branch"/>. SPEC §4.3.
    /// </summary>
    /// <remarks>
    /// The angles are the formulas of SPEC §4.3 as written, not wrapped to (−π, π], like the reference
    /// model. The "reach back over the base" solutions are not returned (SPEC §4.3).
    /// </remarks>
    /// <param name="position">Target tool point (x, y, z) in the base frame, m.</param>
    /// <param name="pitch">Target tool pitch ψ below the horizontal, rad.</param>
    /// <param name="branch">Elbow up or elbow down.</param>
    /// <param name="checkLimits">
    /// True (default): a solution outside the joint limits of SPEC §3.4 is a failure with
    /// <see cref="FailureReason.JointLimit"/>. False: limits are ignored.
    /// </param>
    public static InverseKinematicsResult Solve(Vector3d position, double pitch, ElbowBranch branch, bool checkLimits = true)
    {
        if (!position.IsFinite || !double.IsFinite(pitch) || !Enum.IsDefined(branch))
            return InverseKinematicsResult.Unreachable;

        var q1 = Math.Atan2(position.Y, position.X);
        var u = Math.Sqrt(position.X * position.X + position.Y * position.Y) - G.L4 * Math.Cos(pitch);
        var v = position.Z - G.D1 + G.L4 * Math.Sin(pitch);
        var cosPhi = (u * u + v * v - G.L2 * G.L2 - G.L3 * G.L3) / (2.0 * G.L2 * G.L3);
        // Permit only floating-point roundoff at the stretched/folded boundary (D8).
        if (!(Math.Abs(cosPhi) <= 1.0 + 1e-14))
            return InverseKinematicsResult.Unreachable;

        cosPhi = Math.Clamp(cosPhi, -1.0, 1.0);
        var phi = branch == ElbowBranch.Up ? -Math.Acos(cosPhi) : Math.Acos(cosPhi);
        var e = Math.Atan2(v, u) - Math.Atan2(G.L3 * Math.Sin(phi), G.L2 + G.L3 * Math.Cos(phi));
        var q2 = G.Beta - e;
        var q3 = -G.Beta - phi;
        var q4 = pitch - q2 - q3;

        if (checkLimits)
        {
            var joint = FirstOutsideLimits(q1, q2, q3, q4);
            if (joint >= 0)
                return InverseKinematicsResult.OutsideLimits(q1, q2, q3, q4, joint);
        }
        return InverseKinematicsResult.Solved(q1, q2, q3, q4);
    }

    private static int FirstOutsideLimits(double q1, double q2, double q3, double q4)
    {
        if (q1 < G.Joint1Lower || q1 > G.Joint1Upper) return 0;
        if (q2 < G.Joint2Lower || q2 > G.Joint2Upper) return 1;
        if (q3 < G.Joint3Lower || q3 > G.Joint3Upper) return 2;
        if (q4 < G.Joint4Lower || q4 > G.Joint4Upper) return 3;
        return -1;
    }
}
