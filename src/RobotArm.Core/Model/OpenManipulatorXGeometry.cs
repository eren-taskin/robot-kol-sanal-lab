namespace RobotArm.Core.Model;

/// <summary>
/// Geometry of the OpenMANIPULATOR-X, the single place where its lengths and joint limits are typed.
/// SPEC §3.1 and §3.4. L2 and β are computed from a2x and a2z, never typed.
/// </summary>
public static class OpenManipulatorXGeometry
{
    /// <summary>d1: height of the joint-2 axis above the base frame, m. SPEC §3.1.</summary>
    public const double D1 = 0.0595;

    /// <summary>a2x: joint 3 forward of joint 2 at q2 = 0, m. SPEC §3.1.</summary>
    public const double A2X = 0.024;

    /// <summary>a2z: joint 3 above joint 2 at q2 = 0, m. SPEC §3.1.</summary>
    public const double A2Z = 0.128;

    /// <summary>L3: joint 3 to joint 4, m. SPEC §3.1.</summary>
    public const double L3 = 0.124;

    /// <summary>L4: joint 4 to the tool point, m. SPEC §3.1.</summary>
    public const double L4 = 0.126;

    /// <summary>L2 = √(a2x² + a2z²), m. SPEC §3.1.</summary>
    public static readonly double L2 = Math.Sqrt(A2X * A2X + A2Z * A2Z);

    /// <summary>β = atan2(a2z, a2x), rad. SPEC §3.1.</summary>
    public static readonly double Beta = Math.Atan2(A2Z, A2X);

    /// <summary>Joint 1 lower limit, rad (−π). SPEC §3.4.</summary>
    public const double Joint1Lower = -Math.PI;

    /// <summary>Joint 1 upper limit, rad (+π). SPEC §3.4.</summary>
    public const double Joint1Upper = Math.PI;

    /// <summary>Joint 2 lower limit, rad. SPEC §3.4.</summary>
    public const double Joint2Lower = -1.5;

    /// <summary>Joint 2 upper limit, rad. SPEC §3.4.</summary>
    public const double Joint2Upper = 1.5;

    /// <summary>Joint 3 lower limit, rad. SPEC §3.4.</summary>
    public const double Joint3Lower = -1.5;

    /// <summary>Joint 3 upper limit, rad. SPEC §3.4.</summary>
    public const double Joint3Upper = 1.4;

    /// <summary>Joint 4 lower limit, rad. SPEC §3.4.</summary>
    public const double Joint4Lower = -1.7;

    /// <summary>Joint 4 upper limit, rad. SPEC §3.4.</summary>
    public const double Joint4Upper = 1.97;
}
