using RobotArm.Core.Model;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §5.5 (V14 of SPEC §11): payload combination.
/// The comparison with the <c>payload_kg = 0.5</c> poses of test_vectors.json needs the inertia matrix
/// and is added in task T04.
/// </summary>
public sealed class PayloadTests
{
    private const double RelTol = 1e-12;
    private static readonly ArmModel s_arm = ArmModel.OpenManipulatorX();

    // Hand calculation for mp = 0.5 kg, body 4 of SPEC §3.2 (m4 = 0.1433, ℓ4 = 0.0451, ε4 = −11.4°), L4 = 0.126:
    //   c4  = (ℓ4 cos ε4, 0, −ℓ4 sin ε4)             = (0.044210229975439656, 0, 0.00891434605109859)
    //   m4' = 0.1433 + 0.5                            = 0.6433
    //   c'x = (0.1433·0.0442102300 + 0.5·0.126)/0.6433 = 0.10778070255787425
    //   c'z = (0.1433·0.0089143461)/0.6433            = 0.0019857388296633427
    //   d4  = c4 − c' = (−0.0635704726, 0, 0.0069286072),  dt = t − c' = (0.0182192974, 0, −0.0019857388)
    //   Iyy' = Iyy + m4 (d4x² + d4z²) + mp (dtx² + dtz²)  = 8.299268554476241e-4
    //   Ixx' = Ixx + m4 d4z² + mp dtz²                    = 8.975078154736128e-5
    //   Izz' = Izz + m4 d4x² + mp dtx²                    = 8.381760739002628e-4
    //   Ixz' = Ixz − m4 d4x d4z − mp dtx dtz              = 8.020656110350219e-5
    //   ℓ4'  = √(c'x² + c'z²) = 0.10779899351370863,  ε4' = atan2(−c'z, c'x) = −0.018421796749938543 rad
    private const double HandMass = 0.6433;
    private const double HandCx = 0.10778070255787425;
    private const double HandCz = 0.0019857388296633427;
    private const double HandIyy = 8.299268554476241e-4;
    private const double HandIxx = 8.975078154736128e-5;
    private const double HandIzz = 8.381760739002628e-4;
    private const double HandIxz = 8.020656110350219e-5;
    private const double HandEll = 0.10779899351370863;
    private const double HandEps = -0.018421796749938543;

    private static void AssertRel(double expected, double actual) =>
        Assert.True(Math.Abs(actual - expected) <= RelTol * Math.Abs(expected),
            $"expected {expected:R}, actual {actual:R}, relative difference {Math.Abs(actual - expected) / Math.Abs(expected):E2}");

    [Fact]
    public void WithPayload_HalfKilogram_MassCentreOfMassAndIyyMatchHandCalculation()
    {
        var b = s_arm.WithPayload(0.5).Bodies[3];

        AssertRel(HandMass, b.Mass);
        AssertRel(HandCx, b.CenterOfMass.X);
        Assert.Equal(0.0, b.CenterOfMass.Y);
        AssertRel(HandCz, b.CenterOfMass.Z);
        AssertRel(HandIyy, b.Iyy);
    }

    [Fact]
    public void WithPayload_HalfKilogram_RestOfTensorAndEllEpsMatchHandCalculation()
    {
        var b = s_arm.WithPayload(0.5).Bodies[3];

        AssertRel(HandIxx, b.Ixx);
        AssertRel(HandIzz, b.Izz);
        AssertRel(HandIxz, b.Ixz);
        AssertRel(HandEll, b.Ell);
        AssertRel(HandEps, b.Eps);
    }

    [Fact]
    public void WithPayload_TotalMassAndFirstMomentAreConserved()
    {
        const double mp = 0.37;
        var before = s_arm.Bodies[3];
        var after = s_arm.WithPayload(mp).Bodies[3];

        Assert.Equal(before.Mass + mp, after.Mass, 1e-15);
        var moment = before.Mass * before.CenterOfMass + mp * s_arm.ToolPoint;
        var combined = after.Mass * after.CenterOfMass;
        Assert.Equal(moment.X, combined.X, 1e-15);
        Assert.Equal(moment.Z, combined.Z, 1e-15);
    }

    [Fact]
    public void WithPayload_ChangesOnlyTheLastBody()
    {
        var loaded = s_arm.WithPayload(0.5);

        Assert.NotEqual(s_arm, loaded);
        Assert.Equal(s_arm.Joints, loaded.Joints);
        Assert.Equal(s_arm.Bodies.Take(3), loaded.Bodies.Take(3));
        Assert.Equal(s_arm.ToolPoint, loaded.ToolPoint);
        Assert.Equal(s_arm.Actuator, loaded.Actuator);
        // The original model is unchanged.
        Assert.Equal(ArmModel.OpenManipulatorX(), s_arm);
    }

    [Fact]
    public void WithPayload_Massless_ReturnsEqualModel()
    {
        Assert.Equal(s_arm, s_arm.WithPayload(0.0));
    }

    [Theory]
    [InlineData(-0.1)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void WithPayload_InvalidMass_Throws(double massKg)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => s_arm.WithPayload(massKg));
    }

    [Fact]
    public void WithPayload_TwoSteps_EqualOneStepWithTheSum()
    {
        var twoSteps = s_arm.WithPayload(0.2).WithPayload(0.3).Bodies[3];
        var oneStep = s_arm.WithPayload(0.5).Bodies[3];

        AssertRel(oneStep.Mass, twoSteps.Mass);
        AssertRel(oneStep.Iyy, twoSteps.Iyy);
        AssertRel(oneStep.Ell, twoSteps.Ell);
        Assert.Equal(oneStep.Eps, twoSteps.Eps, 1e-13);
    }
}
