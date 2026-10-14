namespace RobotArm.Core.Model;

/// <summary>Local rotation axis of a revolute joint. SPEC §4.1.</summary>
public enum JointAxis
{
    /// <summary>Rotation about the local z axis (yaw), Rz(q).</summary>
    Z,

    /// <summary>Rotation about the local y axis (pitch), Ry(q); positive tips the arm down.</summary>
    Y,
}

/// <summary>
/// One revolute joint of the chain: where it sits in the parent frame, its axis and its limits.
/// SPEC §4.1 (origin, axis) and §3.4 (limits).
/// </summary>
/// <remarks>
/// The limits are data only. Nothing in Model, Kinematics or Dynamics rejects a configuration for being
/// outside them; they are checked by inverse kinematics (SPEC §4.3) and by fault detection (SPEC §6.4).
/// </remarks>
public sealed record JointDefinition
{
    /// <summary>Origin of the joint in the parent frame, in m. SPEC §4.1.</summary>
    public Vector3d Origin { get; }

    /// <summary>Local rotation axis. SPEC §4.1.</summary>
    public JointAxis Axis { get; }

    /// <summary>Lower joint limit in rad. SPEC §3.4.</summary>
    public double LowerLimit { get; }

    /// <summary>Upper joint limit in rad. SPEC §3.4.</summary>
    public double UpperLimit { get; }

    /// <summary>Creates a joint. SPEC §4.1, §3.4.</summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Origin or limits not finite, an undefined axis, or lower limit not below upper limit.
    /// </exception>
    public JointDefinition(Vector3d origin, JointAxis axis, double lowerLimit, double upperLimit)
    {
        if (!origin.IsFinite)
            throw new ArgumentOutOfRangeException(nameof(origin), origin, "Joint origin must be finite (SPEC §4.1).");
        if (!Enum.IsDefined(axis))
            throw new ArgumentOutOfRangeException(nameof(axis), axis, "Joint axis must be Z or Y (SPEC §4.1).");
        if (!double.IsFinite(lowerLimit))
            throw new ArgumentOutOfRangeException(nameof(lowerLimit), lowerLimit, "Lower joint limit must be finite (SPEC §3.4).");
        if (!double.IsFinite(upperLimit))
            throw new ArgumentOutOfRangeException(nameof(upperLimit), upperLimit, "Upper joint limit must be finite (SPEC §3.4).");
        if (lowerLimit >= upperLimit)
            throw new ArgumentOutOfRangeException(nameof(lowerLimit), lowerLimit,
                $"Lower joint limit must be below the upper limit {upperLimit} rad (SPEC §3.4).");

        Origin = origin;
        Axis = axis;
        LowerLimit = lowerLimit;
        UpperLimit = upperLimit;
    }

    /// <summary>Unit vector of <see cref="Axis"/> in the joint's own frame. SPEC §4.1.</summary>
    public Vector3d AxisVector => Axis == JointAxis.Z ? Vector3d.UnitZ : Vector3d.UnitY;

    /// <summary>
    /// True when <paramref name="angle"/> (rad) lies within [LowerLimit, UpperLimit]. A query only, for
    /// inverse kinematics (SPEC §4.3) and fault detection (SPEC §6.4); it is never used to reject input.
    /// </summary>
    public bool IsWithinLimits(double angle) => angle >= LowerLimit && angle <= UpperLimit;
}
