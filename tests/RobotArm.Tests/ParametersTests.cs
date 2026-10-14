using RobotArm.Core.Model;
using Geo = RobotArm.Core.Model.OpenManipulatorXGeometry;

namespace RobotArm.Tests;

/// <summary>SPEC §3 (V1 of SPEC §11): parameters equal SPEC and the reference vectors; L2 and β are computed.</summary>
public sealed class ParametersTests
{
    private static readonly ArmModel s_arm = ArmModel.OpenManipulatorX();
    private static readonly TestVectors s_vectors = TestVectors.Instance;

    [Fact]
    public void Geometry_EqualsSpecTable3()
    {
        Assert.Equal(0.0595, Geo.D1);
        Assert.Equal(0.024, Geo.A2X);
        Assert.Equal(0.128, Geo.A2Z);
        Assert.Equal(0.124, Geo.L3);
        Assert.Equal(0.126, Geo.L4);
        // SPEC §3.1 prints L2 and β rounded to 6 decimals and β also as 79.38°.
        Assert.Equal(0.130231, Geo.L2, 6);
        Assert.Equal(1.385448, Geo.Beta, 6);
        Assert.Equal(79.38, Geo.Beta * 180.0 / Math.PI, 2);
    }

    [Fact]
    public void L2AndBeta_AreComputedFromA2xAndA2z()
    {
        Assert.Equal(Math.Sqrt(0.024 * 0.024 + 0.128 * 0.128), Geo.L2);
        Assert.Equal(Math.Atan2(0.128, 0.024), Geo.Beta);
    }

    [Fact]
    public void Geometry_EqualsReferenceVectors()
    {
        var g = s_vectors.Geometry;
        Assert.Equal(Geo.D1, g.D1, 1e-15);
        Assert.Equal(g.L2, Geo.L2, 1e-15);
        Assert.Equal(g.Beta, Geo.Beta, 1e-15);
        Assert.Equal(Geo.L3, g.L3, 1e-15);
        Assert.Equal(Geo.L4, g.L4, 1e-15);
    }

    [Fact]
    public void Bodies_EqualSpecTable5()
    {
        double[,] table =
        {
            // m, ℓ, ε (deg), Ixx, Iyy, Izz, Ixz
            { 0.0984, 0.0474, -90.4, 34.5e-6, 32.7e-6, 18.9e-6, -0.4e-6 },
            { 0.1385, 0.1022, -84.2, 330.6e-6, 342.9e-6, 60.3e-6, -38.5e-6 },
            { 0.1327, 0.0909, -0.1, 30.7e-6, 242.3e-6, 251.6e-6, -0.3e-6 },
            { 0.1433, 0.0451, -11.4, 80.9e-6, 76.0e-6, 93.1e-6, -1.0e-6 },
        };

        Assert.Equal(4, s_arm.Bodies.Count);
        for (var i = 0; i < 4; i++)
        {
            var b = s_arm.Bodies[i];
            Assert.Equal(table[i, 0], b.Mass);
            Assert.Equal(table[i, 1], b.Ell);
            Assert.Equal(table[i, 2], b.Eps * 180.0 / Math.PI, 12);
            Assert.Equal(table[i, 3], b.Ixx);
            Assert.Equal(table[i, 4], b.Iyy);
            Assert.Equal(table[i, 5], b.Izz);
            Assert.Equal(table[i, 6], b.Ixz);
        }
    }

    [Fact]
    public void Bodies_EqualReferenceVectors()
    {
        Assert.Equal(s_vectors.Bodies.Count, s_arm.Bodies.Count);
        for (var i = 0; i < s_arm.Bodies.Count; i++)
        {
            var expected = s_vectors.Bodies[i];
            var b = s_arm.Bodies[i];
            Assert.Equal(expected.Mass, b.Mass, 1e-15);
            Assert.Equal(expected.Ell, b.Ell, 1e-15);
            Assert.Equal(expected.Eps, b.Eps, 1e-15);
            Assert.Equal(expected.Ixx, b.Ixx, 1e-15);
            Assert.Equal(expected.Iyy, b.Iyy, 1e-15);
            Assert.Equal(expected.Izz, b.Izz, 1e-15);
            Assert.Equal(expected.Ixz, b.Ixz, 1e-15);
        }
    }

    [Fact]
    public void Actuator_DefaultsEqualSpecSection3_3()
    {
        var a = s_arm.Actuator;
        Assert.Equal(4.1, a.StallTorque);
        Assert.Equal(s_vectors.StallTorque, a.StallTorque);
        Assert.Equal(353.5, a.GearRatio);
        Assert.Equal(4096, a.EncoderCountsPerRevolution);
        Assert.Equal(4.8, a.NoLoadSpeed);
        Assert.Equal(0.0, a.RotorInertia);
        Assert.Equal(0.01, a.ViscousFriction);
        Assert.Equal(0.02, a.CoulombFriction);
        Assert.Equal(0.1, a.SmoothingSpeed);
        Assert.Equal(2.0 * Math.PI / 4096.0, a.EncoderResolution);
        Assert.Equal(0.0, a.ReflectedRotorInertia);
    }

    [Fact]
    public void Actuator_ReflectedRotorInertia_IsGearRatioSquaredTimesJm()
    {
        var a = new ActuatorParameters { RotorInertia = 1e-7 };
        Assert.Equal(353.5 * 353.5 * 1e-7, a.ReflectedRotorInertia, 1e-15);
    }

    [Theory]
    [InlineData(nameof(ActuatorParameters.StallTorque), 0.0)]
    [InlineData(nameof(ActuatorParameters.GearRatio), -1.0)]
    [InlineData(nameof(ActuatorParameters.NoLoadSpeed), double.NaN)]
    [InlineData(nameof(ActuatorParameters.RotorInertia), -1e-9)]
    [InlineData(nameof(ActuatorParameters.ViscousFriction), -0.01)]
    [InlineData(nameof(ActuatorParameters.CoulombFriction), double.PositiveInfinity)]
    [InlineData(nameof(ActuatorParameters.SmoothingSpeed), 0.0)]
    public void Actuator_InvalidValue_Throws(string property, double value)
    {
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => property switch
        {
            nameof(ActuatorParameters.StallTorque) => new ActuatorParameters { StallTorque = value },
            nameof(ActuatorParameters.GearRatio) => new ActuatorParameters { GearRatio = value },
            nameof(ActuatorParameters.NoLoadSpeed) => new ActuatorParameters { NoLoadSpeed = value },
            nameof(ActuatorParameters.RotorInertia) => new ActuatorParameters { RotorInertia = value },
            nameof(ActuatorParameters.ViscousFriction) => new ActuatorParameters { ViscousFriction = value },
            nameof(ActuatorParameters.CoulombFriction) => new ActuatorParameters { CoulombFriction = value },
            nameof(ActuatorParameters.SmoothingSpeed) => new ActuatorParameters { SmoothingSpeed = value },
            _ => throw new ArgumentException(property),
        });
        Assert.Equal(property, ex.ParamName);
    }

    [Fact]
    public void Actuator_ZeroEncoderCounts_Throws()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new ActuatorParameters { EncoderCountsPerRevolution = 0 });
    }

    [Fact]
    public void JointLimits_EqualSpecSection3_4()
    {
        double[,] limits = { { -Math.PI, Math.PI }, { -1.5, 1.5 }, { -1.5, 1.4 }, { -1.7, 1.97 } };
        for (var i = 0; i < 4; i++)
        {
            Assert.Equal(limits[i, 0], s_arm.Joints[i].LowerLimit);
            Assert.Equal(limits[i, 1], s_arm.Joints[i].UpperLimit);
        }
    }

    [Fact]
    public void JointLimits_AreDataOnly_PosesOutsideThemAreReportedNotRejected()
    {
        // The elbow-down IK vectors are mathematical test points outside the limits on purpose.
        // The model only reports this; nothing here throws.
        var configurations = s_vectors.InverseKinematics.SelectMany(c => new[] { c.ElbowUp, c.ElbowDown }).ToList();
        var outside = configurations
            .Count(q => Enumerable.Range(0, 4).Any(i => !s_arm.Joints[i].IsWithinLimits(q[i])));
        Assert.Equal(s_vectors.InverseKinematics.Count, outside);
        Assert.True(s_arm.Joints[1].IsWithinLimits(1.5));
        Assert.False(s_arm.Joints[1].IsWithinLimits(1.6));
    }

    [Fact]
    public void Body_NonPositiveMass_Throws()
    {
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => new BodyParameters(0.0, 0.05, 0.0, 1e-5, 1e-5, 1e-5, 0.0));
        Assert.Equal("mass", ex.ParamName);
        Assert.Throws<ArgumentOutOfRangeException>(() => new BodyParameters(-0.1, 0.05, 0.0, 1e-5, 1e-5, 1e-5, 0.0));
    }

    [Theory]
    [InlineData(0.0, 1e-5, 1e-5, 0.0)]      // Ixx = 0
    [InlineData(1e-5, -1e-5, 1e-5, 0.0)]    // Iyy < 0
    [InlineData(1e-5, 1e-5, 1e-5, 2e-5)]    // Ixx Izz − Ixz² < 0
    [InlineData(1e-5, 1e-5, double.NaN, 0.0)]
    public void Body_InertiaNotPositiveDefinite_Throws(double ixx, double iyy, double izz, double ixz)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new BodyParameters(0.1, 0.05, 0.0, ixx, iyy, izz, ixz));
    }

    [Fact]
    public void Body_CentreOfMassAndInertia_FollowSpecSection4_1()
    {
        var b = new BodyParameters(0.1, 0.05, 0.3, 1e-5, 2e-5, 3e-5, -1e-6);
        Assert.Equal(0.05 * Math.Cos(0.3), b.CenterOfMass.X);
        Assert.Equal(0.0, b.CenterOfMass.Y);
        Assert.Equal(-0.05 * Math.Sin(0.3), b.CenterOfMass.Z);
        Assert.Equal(new Matrix3d(1e-5, 0, -1e-6, 0, 2e-5, 0, -1e-6, 0, 3e-5), b.Inertia);
    }

    [Fact]
    public void Body_FromCentreOfMassAndInertia_RoundTrips()
    {
        var b = s_arm.Bodies[3];
        var back = BodyParameters.FromCenterOfMassAndInertia(b.Mass, b.CenterOfMass, b.Inertia);
        Assert.Equal(b.Mass, back.Mass);
        Assert.Equal(b.Ell, back.Ell, 1e-15);
        Assert.Equal(b.Eps, back.Eps, 1e-14);
        Assert.Equal(b.Inertia, back.Inertia);
    }
}
