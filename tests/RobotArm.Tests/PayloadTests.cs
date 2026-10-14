using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using Xunit.Abstractions;
using static RobotArm.Tests.DynamicsTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §5.5 (V14 of SPEC §11): payload combination, checked by hand and, through the general dynamics
/// of SPEC §5.2, against the <c>payload_kg = 0.5</c> poses of test_vectors.json (Jm = 0, as in the vectors).
/// </summary>
public sealed class PayloadTests(ITestOutputHelper output)
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

    public static TheoryData<int> LoadedPoseIndices()
    {
        var data = new TheoryData<int>();
        var poses = TestVectors.Instance.Poses;
        for (var i = 0; i < poses.Count; i++)
            if (poses[i].PayloadKg == 0.5) data.Add(i);
        return data;
    }

    [Fact]
    public void Vectors_ContainHalfKilogramPoses()
    {
        Assert.NotEmpty(LoadedPoseIndices());
    }

    [Theory]
    [MemberData(nameof(LoadedPoseIndices))]
    public void WithPayload_HalfKilogram_GeneralInertiaMatchesVectors(int index)
    {
        var pose = TestVectors.Instance.Poses[index];
        var loaded = s_arm.WithPayload(pose.PayloadKg);
        Assert.Equal(0.0, loaded.Actuator.RotorInertia);
        var expected = ToArray(pose.M);
        var actual = GeneralInertia(new RecursiveNewtonEuler(loaded), pose.Q);

        var rel = MaxAbsDifference(expected, actual) / MaxAbs(expected);
        output.WriteLine($"V14 pose {index}: general M vs vectors {rel:E3}");
        Assert.True(rel <= DynamicsTestHelpers.RelTol, $"relative difference {rel:E3}");
    }

    [Theory]
    [MemberData(nameof(LoadedPoseIndices))]
    public void WithPayload_HalfKilogram_GeneralGravityMatchesVectors(int index)
    {
        var pose = TestVectors.Instance.Poses[index];
        var h = new double[4];
        new RecursiveNewtonEuler(s_arm.WithPayload(pose.PayloadKg)).BiasTorque(pose.Q, new double[4], Planet.Earth, h);

        var rel = MaxAbsDifference(pose.GravityEarth, h) / MaxAbs(pose.GravityEarth);
        output.WriteLine($"V14 pose {index}: general g vs vectors {rel:E3}");
        Assert.True(rel <= DynamicsTestHelpers.RelTol, $"relative difference {rel:E3}");
    }

    [Fact]
    public void WithPayload_HalfKilogram_GeneralDynamicsMatchClosedForm_InRandomPoses()
    {
        var loaded = s_arm.WithPayload(0.5);
        var rnea = new RecursiveNewtonEuler(loaded);
        var closed = new ClosedFormArm(loaded);
        var random = new Random(14);
        var h = new double[4];
        var worstM = 0.0;
        var worstG = 0.0;

        for (var trial = 0; trial < 1000; trial++)
        {
            var q = RandomVector(random, 4, Math.PI);
            var expectedM = closed.InertiaMatrix(q);
            var relM = MaxAbsDifference(expectedM, GeneralInertia(rnea, q)) / MaxAbs(expectedM);
            rnea.BiasTorque(q, new double[4], Planet.Earth, h);
            var expectedG = closed.Gravity(q, Planet.Earth.G);
            var relG = MaxAbsDifference(expectedG, h) / MaxAbs(expectedG);
            worstM = Math.Max(worstM, relM);
            worstG = Math.Max(worstG, relG);
            Assert.True(relM <= DynamicsTestHelpers.RelTol, $"M: relative difference {relM:E3}");
            Assert.True(relG <= DynamicsTestHelpers.RelTol, $"g: relative difference {relG:E3}");
        }
        output.WriteLine($"V14 0.5 kg, 1000 random poses: max relative difference M {worstM:E3}, g {worstG:E3}");
    }
}
