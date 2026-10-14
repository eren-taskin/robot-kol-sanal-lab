using RobotArm.Core.Model;
using G = RobotArm.Core.Model.OpenManipulatorXGeometry;

namespace RobotArm.Core.Kinematics;

/// <summary>
/// Planar Jacobian Jp of (r, z, ψ) with respect to (q2, q3, q4) for the OpenMANIPULATOR-X, its
/// determinant and a guarded inverse. SPEC §4.4 [eqs. 8–9]. Does not allocate.
/// </summary>
/// <remarks>
/// Jp does not involve q1, so the second singularity of SPEC §4.4, r ≈ 0 (joint 1 no longer moves the tool
/// point), is not visible here; code that uses the full spatial Jacobian must check r separately.
/// </remarks>
public static class PlanarJacobian
{
    /// <summary>Default threshold on |det Jp| below which the Jacobian is treated as singular. SPEC §4.4.</summary>
    public const double DefaultDeterminantThreshold = 1e-4;

    /// <summary>
    /// Jp = [[−(s2+s3+s4), −(s3+s4), −s4], [−(c2+c3+c4), −(c3+c4), −c4], [1, 1, 1]] with
    /// s2 = L2 sin(θ2−β), c2 = L2 cos(θ2−β), si = Li sin θi, ci = Li cos θi. Rows r, z, ψ; columns q2, q3, q4.
    /// SPEC §4.4.
    /// </summary>
    /// <param name="q">Joint angles q1..q4, rad.</param>
    /// <exception cref="ArgumentException"><paramref name="q"/> does not hold 4 angles.</exception>
    public static Matrix3d Compute(ReadOnlySpan<double> q)
    {
        if (q.Length != 4)
            throw new ArgumentException($"The planar Jacobian needs 4 joint angles, got {q.Length} (SPEC §4.4).", nameof(q));
        var theta2 = q[1];
        var theta3 = theta2 + q[2];
        var theta4 = theta3 + q[3];
        var s2 = G.L2 * Math.Sin(theta2 - G.Beta);
        var c2 = G.L2 * Math.Cos(theta2 - G.Beta);
        var s3 = G.L3 * Math.Sin(theta3);
        var c3 = G.L3 * Math.Cos(theta3);
        var s4 = G.L4 * Math.Sin(theta4);
        var c4 = G.L4 * Math.Cos(theta4);
        return new Matrix3d(
            -(s2 + s3 + s4), -(s3 + s4), -s4,
            -(c2 + c3 + c4), -(c3 + c4), -c4,
            1.0, 1.0, 1.0);
    }

    /// <summary>det Jp = −L2 L3 sin(q3 + β). SPEC §4.4.</summary>
    /// <param name="q">Joint angles q1..q4, rad.</param>
    /// <exception cref="ArgumentException"><paramref name="q"/> does not hold 4 angles.</exception>
    public static double Determinant(ReadOnlySpan<double> q)
    {
        if (q.Length != 4)
            throw new ArgumentException($"The planar Jacobian needs 4 joint angles, got {q.Length} (SPEC §4.4).", nameof(q));
        return -G.L2 * G.L3 * Math.Sin(q[2] + G.Beta);
    }

    /// <summary>
    /// True when |det Jp| is below <paramref name="threshold"/>, i.e. |sin(q3 + β)| is small. SPEC §4.4.
    /// </summary>
    public static bool IsNearSingular(ReadOnlySpan<double> q, double threshold = DefaultDeterminantThreshold) =>
        !(Math.Abs(Determinant(q)) >= threshold);

    /// <summary>
    /// Inverse of Jp, refused (returns false and a zero matrix) when |det Jp| is below
    /// <paramref name="threshold"/>. SPEC §4.4: code that inverts Jp must check the determinant first.
    /// </summary>
    /// <param name="q">Joint angles q1..q4, rad.</param>
    /// <param name="inverse">Jp⁻¹ on success, otherwise zero.</param>
    /// <param name="threshold">Threshold on |det Jp|, default 1e-4.</param>
    public static bool TryInvert(ReadOnlySpan<double> q, out Matrix3d inverse, double threshold = DefaultDeterminantThreshold)
    {
        var det = Determinant(q);
        if (!(Math.Abs(det) >= threshold))
        {
            inverse = Matrix3d.Zero;
            return false;
        }

        // Adjugate divided by the determinant (closed-form SPEC value of det Jp).
        var j = Compute(q);
        var adjugate = new Matrix3d(
            j.M11 * j.M22 - j.M12 * j.M21, j.M02 * j.M21 - j.M01 * j.M22, j.M01 * j.M12 - j.M02 * j.M11,
            j.M12 * j.M20 - j.M10 * j.M22, j.M00 * j.M22 - j.M02 * j.M20, j.M02 * j.M10 - j.M00 * j.M12,
            j.M10 * j.M21 - j.M11 * j.M20, j.M01 * j.M20 - j.M00 * j.M21, j.M00 * j.M11 - j.M01 * j.M10);
        inverse = (1.0 / det) * adjugate;
        return true;
    }
}
