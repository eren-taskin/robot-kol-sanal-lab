using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;

namespace RobotArm.Tests;

/// <summary>
/// A made-up 6-DOF chain for the tests of SPEC §4.1 ("the same algorithms must work for a 6-DOF chain given as
/// a different table") and §5.2. Joint axes Z, Y, Y, X, Y, X; joint origins, centres of mass and the tool point
/// off the axes; full inertia tensors with nonzero products of inertia; different actuators on every joint.
/// The numbers are invented but physically valid (positive masses, principal moments obeying the triangle
/// inequality). Test data only, not a real arm.
/// </summary>
internal static class SixJointChain
{
    private static readonly JointAxis[] s_axes = [JointAxis.Z, JointAxis.Y, JointAxis.Y, JointAxis.X, JointAxis.Y, JointAxis.X];

    private static readonly Vector3d[] s_origins =
    [
        new(0.0, 0.0, 0.0),
        new(0.02, 0.01, 0.18),
        new(0.30, -0.03, 0.02),
        new(0.06, 0.02, -0.01),
        new(0.22, -0.01, 0.015),
        new(0.05, 0.008, -0.004),
    ];

    // m (kg), centre of mass in Fi (m), principal moments (kg·m²), and the angles (rad) of Rz·Ry·Rx that turn
    // the principal axes away from the body axes, so that every product of inertia is nonzero.
    private static readonly (double Mass, Vector3d Com, Vector3d Principal, Vector3d Angles)[] s_bodies =
    [
        (2.4, new(0.015, -0.02, 0.07), new(0.012, 0.010, 0.008), new(0.3, -0.2, 0.5)),
        (1.9, new(0.14, 0.025, 0.03), new(0.004, 0.018, 0.017), new(0.7, 0.1, -0.4)),
        (1.1, new(0.03, -0.012, 0.02), new(0.0025, 0.0030, 0.0018), new(-0.5, 0.35, 0.2)),
        (0.8, new(0.11, 0.015, -0.01), new(0.0009, 0.0042, 0.0040), new(0.25, -0.6, 0.9)),
        (0.45, new(0.025, -0.008, 0.012), new(0.0006, 0.0005, 0.0004), new(1.1, 0.4, -0.3)),
        (0.25, new(0.04, 0.01, -0.006), new(0.00015, 0.00022, 0.00019), new(-0.8, 0.5, 0.6)),
    ];

    /// <summary>Tool point in F6, m.</summary>
    public static readonly Vector3d ToolPoint = new(0.09, 0.012, 0.006);

    /// <summary>
    /// The chain. <paramref name="friction"/> false sets fv = fc = 0 on every joint; <paramref name="rotorInertia"/>
    /// gives every joint a different Jm (and so a different N²·Jm).
    /// </summary>
    public static ArmModel Create(bool friction = true, bool rotorInertia = true)
    {
        var joints = new JointDefinition[6];
        var bodies = new BodyParameters[6];
        var actuators = new ActuatorParameters[6];
        double[] stall = [60.0, 60.0, 30.0, 10.0, 10.0, 5.0];
        double[] gear = [100.0, 100.0, 120.0, 80.0, 80.0, 50.0];
        double[] jm = [2e-6, 2e-6, 1e-6, 5e-7, 5e-7, 2e-7];
        double[] fv = [0.05, 0.05, 0.03, 0.01, 0.01, 0.005];
        double[] fc = [0.10, 0.10, 0.06, 0.02, 0.02, 0.01];

        for (var i = 0; i < 6; i++)
        {
            joints[i] = new JointDefinition(s_origins[i], s_axes[i], -3.0, 3.0);
            var (mass, com, principal, angles) = s_bodies[i];
            bodies[i] = new BodyParameters(mass, com, RotatedInertia(principal, angles));
            actuators[i] = new ActuatorParameters
            {
                StallTorque = stall[i],
                GearRatio = gear[i],
                RotorInertia = rotorInertia ? jm[i] : 0.0,
                ViscousFriction = friction ? fv[i] : 0.0,
                CoulombFriction = friction ? fc[i] : 0.0,
            };
        }
        return new ArmModel(joints, bodies, ToolPoint, actuators);
    }

    /// <summary>R·diag(principal)·Rᵀ with R = Rz·Ry·Rx, made exactly symmetric from its upper triangle.</summary>
    private static Matrix3d RotatedInertia(Vector3d principal, Vector3d angles)
    {
        var r = Rotations.Rz(angles.X) * Rotations.Ry(angles.Y) * Rotations.Rx(angles.Z);
        var a = r * Matrix3d.Diagonal(principal.X, principal.Y, principal.Z) * r.Transpose();
        return new Matrix3d(a.M00, a.M01, a.M02, a.M01, a.M11, a.M12, a.M02, a.M12, a.M22);
    }
}
