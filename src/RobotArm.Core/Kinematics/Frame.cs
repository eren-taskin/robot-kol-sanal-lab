using RobotArm.Core.Model;

namespace RobotArm.Core.Kinematics;

/// <summary>
/// Pose of a body frame Fi in the base frame: its rotation and origin. SPEC §4.1.
/// Immutable value type; no operation allocates.
/// </summary>
public readonly struct Frame : IEquatable<Frame>
{
    /// <summary>Rotation of the frame: column k is the frame's local axis k expressed in the base frame.</summary>
    public Matrix3d Rotation { get; }

    /// <summary>Origin of the frame in the base frame, m.</summary>
    public Vector3d Origin { get; }

    /// <summary>Creates the frame from its rotation and origin. SPEC §4.1.</summary>
    public Frame(Matrix3d rotation, Vector3d origin)
    {
        Rotation = rotation;
        Origin = origin;
    }

    /// <summary>The base frame: identity rotation, zero origin.</summary>
    public static Frame Identity => new(Matrix3d.Identity, Vector3d.Zero);

    /// <summary>Maps a point given in this frame to the base frame: Origin + Rotation·p. SPEC §4.1.</summary>
    public Vector3d TransformPoint(Vector3d local) => Origin + Rotation * local;

    /// <summary>
    /// The child frame F·T(offset)·R(axis, angle): translate by <paramref name="offset"/> in this frame,
    /// then rotate about the local <paramref name="axis"/>. One row of the chain table of SPEC §4.1.
    /// </summary>
    public Frame Then(Vector3d offset, JointAxis axis, double angle) =>
        new(Rotation * Rotations.About(axis, angle), TransformPoint(offset));

    /// <summary>Exact equality of rotation and origin.</summary>
    public bool Equals(Frame other) => Rotation.Equals(other.Rotation) && Origin.Equals(other.Origin);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Frame other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(Rotation, Origin);

    /// <summary>Exact equality.</summary>
    public static bool operator ==(Frame a, Frame b) => a.Equals(b);

    /// <summary>Exact inequality.</summary>
    public static bool operator !=(Frame a, Frame b) => !a.Equals(b);
}

/// <summary>Elementary rotations of SPEC §4.1.</summary>
public static class Rotations
{
    /// <summary>Rz(a) = [[c,−s,0],[s,c,0],[0,0,1]]. SPEC §4.1.</summary>
    public static Matrix3d Rz(double a)
    {
        var c = Math.Cos(a);
        var s = Math.Sin(a);
        return new Matrix3d(c, -s, 0, s, c, 0, 0, 0, 1);
    }

    /// <summary>Ry(a) = [[c,0,s],[0,1,0],[−s,0,c]]; a positive angle turns local x downward. SPEC §4.1.</summary>
    public static Matrix3d Ry(double a)
    {
        var c = Math.Cos(a);
        var s = Math.Sin(a);
        return new Matrix3d(c, 0, s, 0, 1, 0, -s, 0, c);
    }

    /// <summary>Rx(a) = [[1,0,0],[0,c,−s],[0,s,c]], roll about the local x axis. SPEC §4.1 (longer chains).</summary>
    public static Matrix3d Rx(double a)
    {
        var c = Math.Cos(a);
        var s = Math.Sin(a);
        return new Matrix3d(1, 0, 0, 0, c, -s, 0, s, c);
    }

    /// <summary>Rotation by <paramref name="angle"/> about a joint axis. SPEC §4.1.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The axis is not defined.</exception>
    public static Matrix3d About(JointAxis axis, double angle) => axis switch
    {
        JointAxis.Z => Rz(angle),
        JointAxis.Y => Ry(angle),
        JointAxis.X => Rx(angle),
        _ => throw new ArgumentOutOfRangeException(nameof(axis), axis, "Joint axis must be Z, Y or X (SPEC §4.1)."),
    };
}
