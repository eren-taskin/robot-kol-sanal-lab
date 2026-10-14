namespace RobotArm.Core.Model;

/// <summary>
/// A 3-vector of doubles, used for points, offsets and axes in a body frame (SPEC §4.1).
/// Immutable value type; no operation allocates.
/// </summary>
public readonly struct Vector3d : IEquatable<Vector3d>
{
    /// <summary>x component. SPEC §2: x forward when q1 = 0.</summary>
    public double X { get; }

    /// <summary>y component.</summary>
    public double Y { get; }

    /// <summary>z component. SPEC §2: z up.</summary>
    public double Z { get; }

    /// <summary>Creates the vector (x, y, z).</summary>
    public Vector3d(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    /// <summary>The zero vector.</summary>
    public static Vector3d Zero => default;

    /// <summary>Unit vector along x.</summary>
    public static Vector3d UnitX => new(1.0, 0.0, 0.0);

    /// <summary>Unit vector along y (axis of the pitch joints, SPEC §4.1).</summary>
    public static Vector3d UnitY => new(0.0, 1.0, 0.0);

    /// <summary>Unit vector along z (axis of the yaw joint, SPEC §4.1).</summary>
    public static Vector3d UnitZ => new(0.0, 0.0, 1.0);

    /// <summary>Component by zero-based index: 0 = x, 1 = y, 2 = z.</summary>
    public double this[int index] => index switch
    {
        0 => X,
        1 => Y,
        2 => Z,
        _ => throw new ArgumentOutOfRangeException(nameof(index), index, "Vector3d index must be 0, 1 or 2."),
    };

    /// <summary>Squared Euclidean length.</summary>
    public double LengthSquared => X * X + Y * Y + Z * Z;

    /// <summary>Euclidean length.</summary>
    public double Length => Math.Sqrt(LengthSquared);

    /// <summary>True when every component is finite (not NaN, not infinity).</summary>
    public bool IsFinite => double.IsFinite(X) && double.IsFinite(Y) && double.IsFinite(Z);

    /// <summary>Dot product a·b.</summary>
    public static double Dot(Vector3d a, Vector3d b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    /// <summary>Cross product a×b.</summary>
    public static Vector3d Cross(Vector3d a, Vector3d b) =>
        new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);

    /// <summary>Component-wise sum.</summary>
    public static Vector3d operator +(Vector3d a, Vector3d b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    /// <summary>Component-wise difference.</summary>
    public static Vector3d operator -(Vector3d a, Vector3d b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    /// <summary>Negation.</summary>
    public static Vector3d operator -(Vector3d a) => new(-a.X, -a.Y, -a.Z);

    /// <summary>Scaling by a scalar.</summary>
    public static Vector3d operator *(double s, Vector3d a) => new(s * a.X, s * a.Y, s * a.Z);

    /// <summary>Scaling by a scalar.</summary>
    public static Vector3d operator *(Vector3d a, double s) => new(s * a.X, s * a.Y, s * a.Z);

    /// <summary>Division by a scalar.</summary>
    public static Vector3d operator /(Vector3d a, double s) => new(a.X / s, a.Y / s, a.Z / s);

    /// <summary>Exact component-wise equality.</summary>
    public static bool operator ==(Vector3d a, Vector3d b) => a.Equals(b);

    /// <summary>Exact component-wise inequality.</summary>
    public static bool operator !=(Vector3d a, Vector3d b) => !a.Equals(b);

    /// <summary>Exact component-wise equality.</summary>
    public bool Equals(Vector3d other) => X.Equals(other.X) && Y.Equals(other.Y) && Z.Equals(other.Z);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Vector3d other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(X, Y, Z);

    /// <summary>Text form "(x, y, z)" in invariant culture (SPEC §2).</summary>
    public override string ToString() =>
        string.Create(System.Globalization.CultureInfo.InvariantCulture, $"({X}, {Y}, {Z})");
}
