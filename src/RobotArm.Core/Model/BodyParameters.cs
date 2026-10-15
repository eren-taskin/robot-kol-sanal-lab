namespace RobotArm.Core.Model;

/// <summary>
/// Mass properties of one rigid body of the chain, in the frame of the joint that moves it.
/// SPEC §3.2 (table and meaning of ℓ, ε) and §4.1 (centre of mass and inertia in Fi).
/// Immutable; validated on construction.
/// </summary>
/// <remarks>
/// The OpenMANIPULATOR-X bodies are planar: centre of mass in the body's x–z plane and Ixy = Iyz = 0
/// (SPEC §3.2). A general body for other chains, such as a 6-DOF arm, is made with
/// <see cref="BodyParameters(double, Vector3d, Matrix3d)"/>; the dynamics use the full vector and tensor either way.
/// </remarks>
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

    /// <summary>Ixz about the centre of mass, body frame, kg·m². SPEC §3.2.</summary>
    public double Ixz { get; }

    /// <summary>y coordinate of the centre of mass in the body frame, m; 0 for planar bodies. SPEC §4.1.</summary>
    public double Cy { get; }

    /// <summary>Ixy about the centre of mass, body frame, kg·m²; 0 for planar bodies. SPEC §4.1.</summary>
    public double Ixy { get; }

    /// <summary>Iyz about the centre of mass, body frame, kg·m²; 0 for planar bodies. SPEC §4.1.</summary>
    public double Iyz { get; }

    /// <summary>
    /// Centre of mass in the body frame Fi: (ℓ cos ε, Cy, −ℓ sin ε), m. Cy = 0 for the planar bodies of SPEC §4.1.
    /// </summary>
    public Vector3d CenterOfMass { get; }

    /// <summary>
    /// Inertia tensor about the centre of mass in Fi, kg·m²: [[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]].
    /// For the planar bodies of SPEC §4.1 Ixy = Iyz = 0.
    /// </summary>
    public Matrix3d Inertia { get; }

    /// <summary>
    /// True when the centre of mass lies in the body's x–z plane and Ixy = Iyz = 0, as for the
    /// OpenMANIPULATOR-X (SPEC §3.2). The payload combination of SPEC §5.5 needs this.
    /// </summary>
    public bool IsPlanar => Cy == 0.0 && Ixy == 0.0 && Iyz == 0.0;

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
        CenterOfMass = new Vector3d(ell * Math.Cos(eps), 0.0, -ell * Math.Sin(eps));
        Inertia = inertia;
    }

    /// <summary>
    /// Creates a general body from its centre of mass and its full inertia tensor about the centre of mass,
    /// both in the body frame. SPEC §4.1 ("the same algorithms must work for a 6-DOF chain"). The vector and
    /// tensor are stored as given; ℓ = √(cx² + cz²) and ε = atan2(−cz, cx) describe their x–z part.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Mass not positive, a value not finite, or the tensor not symmetric and positive definite.
    /// </exception>
    public BodyParameters(double mass, Vector3d centerOfMass, Matrix3d inertia)
    {
        if (!double.IsFinite(mass) || mass <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(mass), mass, "Body mass must be a finite number greater than 0 kg (SPEC §3.2).");
        if (!centerOfMass.IsFinite)
            throw new ArgumentOutOfRangeException(nameof(centerOfMass), centerOfMass, "Centre of mass must be finite (SPEC §4.1).");
        if (!inertia.IsSymmetricPositiveDefinite)
            throw new ArgumentOutOfRangeException(nameof(inertia), inertia,
                "Inertia tensor must be finite, symmetric and positive definite (SPEC §4.1).");

        Mass = mass;
        Ell = Math.Sqrt(centerOfMass.X * centerOfMass.X + centerOfMass.Z * centerOfMass.Z);
        Eps = Math.Atan2(-centerOfMass.Z, centerOfMass.X);
        Cy = centerOfMass.Y;
        Ixx = inertia.M00;
        Iyy = inertia.M11;
        Izz = inertia.M22;
        Ixz = inertia.M02;
        Ixy = inertia.M01;
        Iyz = inertia.M12;
        CenterOfMass = centerOfMass;
        Inertia = inertia;
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

}
