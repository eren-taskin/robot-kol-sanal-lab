namespace RobotArm.Core.Model;

/// <summary>
/// Mass properties of one rigid body of the chain, in the frame of the joint that moves it.
/// SPEC §3.2 (table and meaning of ℓ, ε) and §4.1 (centre of mass and inertia in Fi).
/// Immutable; validated on construction.
/// </summary>
public sealed record BodyParameters
{
    /// <summary>Mass m in kg. SPEC §3.2.</summary>
    public double Mass { get; }

    /// <summary>Distance ℓ from the joint to the centre of mass, in m. SPEC §3.2.</summary>
    public double Ell { get; }

    /// <summary>Angle ε of the centre of mass below the body's x axis, in rad. SPEC §3.2.</summary>
    public double Eps { get; }

    /// <summary>Ixx about the centre of mass, body frame, kg·m². SPEC §3.2.</summary>
    public double Ixx { get; }

    /// <summary>Iyy about the centre of mass, body frame, kg·m². SPEC §3.2.</summary>
    public double Iyy { get; }

    /// <summary>Izz about the centre of mass, body frame, kg·m². SPEC §3.2.</summary>
    public double Izz { get; }

    /// <summary>Ixz about the centre of mass, body frame, kg·m². Ixy = Iyz = 0. SPEC §3.2.</summary>
    public double Ixz { get; }

    /// <summary>
    /// Creates a body. SPEC §3.2.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Mass not positive, ℓ negative, a value not finite, or the inertia tensor not positive definite.
    /// </exception>
    public BodyParameters(double mass, double ell, double eps, double ixx, double iyy, double izz, double ixz)
    {
        if (!double.IsFinite(mass) || mass <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(mass), mass, "Body mass must be a finite number greater than 0 kg (SPEC §3.2).");
        if (!double.IsFinite(ell) || ell < 0.0)
            throw new ArgumentOutOfRangeException(nameof(ell), ell, "Centre-of-mass distance ℓ must be a finite number of at least 0 m (SPEC §3.2).");
        if (!double.IsFinite(eps))
            throw new ArgumentOutOfRangeException(nameof(eps), eps, "Centre-of-mass angle ε must be finite (SPEC §3.2).");

        var inertia = new Matrix3d(ixx, 0.0, ixz, 0.0, iyy, 0.0, ixz, 0.0, izz);
        if (!inertia.IsSymmetricPositiveDefinite)
            throw new ArgumentOutOfRangeException(
                nameof(ixx),
                $"Inertia tensor [[{ixx}, 0, {ixz}], [0, {iyy}, 0], [{ixz}, 0, {izz}]] must be finite and positive definite (SPEC §3.2).");

        Mass = mass;
        Ell = ell;
        Eps = eps;
        Ixx = ixx;
        Iyy = iyy;
        Izz = izz;
        Ixz = ixz;
    }

    /// <summary>
    /// Creates a body from its centre of mass and inertia tensor in the body frame, the inverse of
    /// <see cref="CenterOfMass"/> and <see cref="Inertia"/>: ℓ = √(cx² + cz²), ε = atan2(−cz, cx). SPEC §5.5.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// The centre of mass is not in the x–z plane, the tensor has Ixy or Iyz terms or is not symmetric,
    /// or the constructor's checks fail.
    /// </exception>
    public static BodyParameters FromCenterOfMassAndInertia(double mass, Vector3d centerOfMass, Matrix3d inertia)
    {
        if (centerOfMass.Y != 0.0)
            throw new ArgumentOutOfRangeException(nameof(centerOfMass), centerOfMass,
                "The centre of mass must lie in the body's x–z plane, y = 0 (SPEC §3.2).");
        if (!inertia.IsSymmetric || inertia.M01 != 0.0 || inertia.M12 != 0.0)
            throw new ArgumentOutOfRangeException(nameof(inertia), inertia,
                "The inertia tensor must be symmetric with Ixy = Iyz = 0 (SPEC §3.2).");

        return new BodyParameters(
            mass,
            Math.Sqrt(centerOfMass.X * centerOfMass.X + centerOfMass.Z * centerOfMass.Z),
            Math.Atan2(-centerOfMass.Z, centerOfMass.X),
            inertia.M00, inertia.M11, inertia.M22, inertia.M02);
    }

    /// <summary>Centre of mass in the body frame Fi: (ℓ cos ε, 0, −ℓ sin ε). SPEC §4.1.</summary>
    public Vector3d CenterOfMass => new(Ell * Math.Cos(Eps), 0.0, -Ell * Math.Sin(Eps));

    /// <summary>Inertia tensor about the centre of mass in Fi: [[Ixx, 0, Ixz], [0, Iyy, 0], [Ixz, 0, Izz]]. SPEC §4.1.</summary>
    public Matrix3d Inertia => new(Ixx, 0.0, Ixz, 0.0, Iyy, 0.0, Ixz, 0.0, Izz);
}
