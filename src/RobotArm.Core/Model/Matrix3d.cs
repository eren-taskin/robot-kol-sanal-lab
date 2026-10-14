namespace RobotArm.Core.Model;

/// <summary>
/// A 3×3 matrix of doubles, used for inertia tensors and rotations (SPEC §4.1, §5.5).
/// Elements are named by zero-based row and column, M01 = row 0, column 1.
/// Immutable value type; no operation allocates.
/// </summary>
public readonly struct Matrix3d : IEquatable<Matrix3d>
{
    /// <summary>Row 0, column 0.</summary>
    public double M00 { get; }
    /// <summary>Row 0, column 1.</summary>
    public double M01 { get; }
    /// <summary>Row 0, column 2.</summary>
    public double M02 { get; }
    /// <summary>Row 1, column 0.</summary>
    public double M10 { get; }
    /// <summary>Row 1, column 1.</summary>
    public double M11 { get; }
    /// <summary>Row 1, column 2.</summary>
    public double M12 { get; }
    /// <summary>Row 2, column 0.</summary>
    public double M20 { get; }
    /// <summary>Row 2, column 1.</summary>
    public double M21 { get; }
    /// <summary>Row 2, column 2.</summary>
    public double M22 { get; }

    /// <summary>Creates the matrix from its elements in row order.</summary>
    public Matrix3d(
        double m00, double m01, double m02,
        double m10, double m11, double m12,
        double m20, double m21, double m22)
    {
        M00 = m00; M01 = m01; M02 = m02;
        M10 = m10; M11 = m11; M12 = m12;
        M20 = m20; M21 = m21; M22 = m22;
    }

    /// <summary>The zero matrix.</summary>
    public static Matrix3d Zero => default;

    /// <summary>The identity matrix.</summary>
    public static Matrix3d Identity => new(1, 0, 0, 0, 1, 0, 0, 0, 1);

    /// <summary>Element by zero-based row and column.</summary>
    public double this[int row, int column] => (row, column) switch
    {
        (0, 0) => M00, (0, 1) => M01, (0, 2) => M02,
        (1, 0) => M10, (1, 1) => M11, (1, 2) => M12,
        (2, 0) => M20, (2, 1) => M21, (2, 2) => M22,
        _ => throw new ArgumentOutOfRangeException(nameof(row), $"Matrix3d index ({row}, {column}) must be in 0..2."),
    };

    /// <summary>Diagonal matrix diag(a, b, c).</summary>
    public static Matrix3d Diagonal(double a, double b, double c) => new(a, 0, 0, 0, b, 0, 0, 0, c);

    /// <summary>Outer product a bᵀ.</summary>
    public static Matrix3d Outer(Vector3d a, Vector3d b) => new(
        a.X * b.X, a.X * b.Y, a.X * b.Z,
        a.Y * b.X, a.Y * b.Y, a.Y * b.Z,
        a.Z * b.X, a.Z * b.Y, a.Z * b.Z);

    /// <summary>
    /// Parallel-axis term m (|d|² 1 − d dᵀ) of SPEC §5.5: the inertia of a point mass m at offset d.
    /// </summary>
    public static Matrix3d PointMassInertia(double mass, Vector3d d) =>
        mass * (d.LengthSquared * Identity - Outer(d, d));

    /// <summary>Transpose.</summary>
    public Matrix3d Transpose() => new(M00, M10, M20, M01, M11, M21, M02, M12, M22);

    /// <summary>Sum of the diagonal.</summary>
    public double Trace => M00 + M11 + M22;

    /// <summary>Determinant.</summary>
    public double Determinant =>
        M00 * (M11 * M22 - M12 * M21)
        - M01 * (M10 * M22 - M12 * M20)
        + M02 * (M10 * M21 - M11 * M20);

    /// <summary>True when every element is finite.</summary>
    public bool IsFinite =>
        double.IsFinite(M00) && double.IsFinite(M01) && double.IsFinite(M02) &&
        double.IsFinite(M10) && double.IsFinite(M11) && double.IsFinite(M12) &&
        double.IsFinite(M20) && double.IsFinite(M21) && double.IsFinite(M22);

    /// <summary>True when the matrix equals its transpose exactly.</summary>
    public bool IsSymmetric => M01 == M10 && M02 == M20 && M12 == M21;

    /// <summary>
    /// True when the matrix is symmetric and positive definite (Sylvester's criterion: all leading
    /// principal minors positive). Used to validate inertia tensors (SPEC §3.2).
    /// </summary>
    public bool IsSymmetricPositiveDefinite =>
        IsFinite && IsSymmetric &&
        M00 > 0.0 &&
        M00 * M11 - M01 * M10 > 0.0 &&
        Determinant > 0.0;

    /// <summary>Element-wise sum.</summary>
    public static Matrix3d operator +(Matrix3d a, Matrix3d b) => new(
        a.M00 + b.M00, a.M01 + b.M01, a.M02 + b.M02,
        a.M10 + b.M10, a.M11 + b.M11, a.M12 + b.M12,
        a.M20 + b.M20, a.M21 + b.M21, a.M22 + b.M22);

    /// <summary>Element-wise difference.</summary>
    public static Matrix3d operator -(Matrix3d a, Matrix3d b) => new(
        a.M00 - b.M00, a.M01 - b.M01, a.M02 - b.M02,
        a.M10 - b.M10, a.M11 - b.M11, a.M12 - b.M12,
        a.M20 - b.M20, a.M21 - b.M21, a.M22 - b.M22);

    /// <summary>Scaling by a scalar.</summary>
    public static Matrix3d operator *(double s, Matrix3d a) => new(
        s * a.M00, s * a.M01, s * a.M02,
        s * a.M10, s * a.M11, s * a.M12,
        s * a.M20, s * a.M21, s * a.M22);

    /// <summary>Matrix–vector product A v.</summary>
    public static Vector3d operator *(Matrix3d a, Vector3d v) => new(
        a.M00 * v.X + a.M01 * v.Y + a.M02 * v.Z,
        a.M10 * v.X + a.M11 * v.Y + a.M12 * v.Z,
        a.M20 * v.X + a.M21 * v.Y + a.M22 * v.Z);

    /// <summary>Matrix product A B.</summary>
    public static Matrix3d operator *(Matrix3d a, Matrix3d b) => new(
        a.M00 * b.M00 + a.M01 * b.M10 + a.M02 * b.M20,
        a.M00 * b.M01 + a.M01 * b.M11 + a.M02 * b.M21,
        a.M00 * b.M02 + a.M01 * b.M12 + a.M02 * b.M22,
        a.M10 * b.M00 + a.M11 * b.M10 + a.M12 * b.M20,
        a.M10 * b.M01 + a.M11 * b.M11 + a.M12 * b.M21,
        a.M10 * b.M02 + a.M11 * b.M12 + a.M12 * b.M22,
        a.M20 * b.M00 + a.M21 * b.M10 + a.M22 * b.M20,
        a.M20 * b.M01 + a.M21 * b.M11 + a.M22 * b.M21,
        a.M20 * b.M02 + a.M21 * b.M12 + a.M22 * b.M22);

    /// <summary>Exact element-wise equality.</summary>
    public static bool operator ==(Matrix3d a, Matrix3d b) => a.Equals(b);

    /// <summary>Exact element-wise inequality.</summary>
    public static bool operator !=(Matrix3d a, Matrix3d b) => !a.Equals(b);

    /// <summary>Exact element-wise equality.</summary>
    public bool Equals(Matrix3d other) =>
        M00.Equals(other.M00) && M01.Equals(other.M01) && M02.Equals(other.M02) &&
        M10.Equals(other.M10) && M11.Equals(other.M11) && M12.Equals(other.M12) &&
        M20.Equals(other.M20) && M21.Equals(other.M21) && M22.Equals(other.M22);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Matrix3d other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        var hash = new HashCode();
        hash.Add(M00); hash.Add(M01); hash.Add(M02);
        hash.Add(M10); hash.Add(M11); hash.Add(M12);
        hash.Add(M20); hash.Add(M21); hash.Add(M22);
        return hash.ToHashCode();
    }

    /// <summary>Text form "[[..], [..], [..]]" in invariant culture (SPEC §2).</summary>
    public override string ToString() => string.Create(
        System.Globalization.CultureInfo.InvariantCulture,
        $"[[{M00}, {M01}, {M02}], [{M10}, {M11}, {M12}], [{M20}, {M21}, {M22}]]");
}
