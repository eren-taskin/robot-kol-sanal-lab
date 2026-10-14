using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.DynamicsTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §5.2–§5.3 (V5 and V6 of SPEC §11): the general inertia matrix against the closed form and the
/// reference vectors, symmetry and positive definiteness, and the reflected rotor inertia N²·Jm.
/// The vector tests use Jm = 0, the default, as the vectors do.
/// </summary>
public sealed class InertiaMatrixTests(ITestOutputHelper output)
{
    private static readonly ArmModel s_arm = ArmModel.OpenManipulatorX();

    public static TheoryData<int> UnloadedPoseIndices()
    {
        var data = new TheoryData<int>();
        var poses = TestVectors.Instance.Poses;
        for (var i = 0; i < poses.Count; i++)
            if (poses[i].PayloadKg == 0.0) data.Add(i);
        return data;
    }

    [Fact]
    public void DefaultModel_HasZeroRotorInertia()
    {
        Assert.Equal(0.0, s_arm.Actuator.RotorInertia);
    }

    [Fact]
    public void InertiaMatrix_MatchesClosedForm_InRandomPoses()
    {
        var rnea = new RecursiveNewtonEuler(s_arm);
        var closed = new ClosedFormArm(s_arm);
        var random = new Random(52);
        var worst = 0.0;

        for (var trial = 0; trial < 1000; trial++)
        {
            var q = RandomVector(random, 4, Math.PI);
            var expected = closed.InertiaMatrix(q);
            var actual = GeneralInertia(rnea, q);
            var rel = MaxAbsDifference(expected, actual) / MaxAbs(expected);
            worst = Math.Max(worst, rel);
            Assert.True(rel <= RelTol, Invariant($"q = [{TestFormat.Join(q)}]: relative difference {rel:E3}"));
        }
        output.WriteLine(Invariant($"V5 general M vs closed form, 1000 random poses: max |ΔM|/max|M| = {worst:E3}"));
    }

    [Theory]
    [MemberData(nameof(UnloadedPoseIndices))]
    public void InertiaMatrix_MatchesVectors_UnloadedPoses(int index)
    {
        var pose = TestVectors.Instance.Poses[index];
        var expected = ToArray(pose.M);
        var general = GeneralInertia(new RecursiveNewtonEuler(s_arm), pose.Q);
        var closed = new ClosedFormArm(s_arm).InertiaMatrix(pose.Q);

        var relGeneral = MaxAbsDifference(expected, general) / MaxAbs(expected);
        var relClosed = MaxAbsDifference(expected, closed) / MaxAbs(expected);
        output.WriteLine(Invariant($"V5 pose {index}: general vs vectors {relGeneral:E3}, closed form vs vectors {relClosed:E3}"));
        Assert.True(relGeneral <= RelTol, Invariant($"general M vs vectors: relative difference {relGeneral:E3}"));
        Assert.True(relClosed <= RelTol, Invariant($"closed-form M vs vectors: relative difference {relClosed:E3}"));
    }

    [Fact]
    public void InertiaMatrix_IsSymmetricPositiveDefinite_In1000RandomPoses()
    {
        var rnea = new RecursiveNewtonEuler(s_arm);
        var random = new Random(37);
        var m = new double[16];

        for (var trial = 0; trial < 1000; trial++)
        {
            var q = RandomVector(random, 4, Math.PI);
            rnea.InertiaMatrix(q, m);

            for (var r = 0; r < 4; r++)
                for (var c = 0; c < r; c++)
                    Assert.Equal(m[r * 4 + c], m[c * 4 + r]);

            // xᵀ M x > 0 for random x, and Cholesky succeeds (which holds exactly for SPD matrices).
            var x = RandomVector(random, 4, 1.0);
            var quadratic = 0.0;
            for (var r = 0; r < 4; r++)
                for (var c = 0; c < 4; c++)
                    quadratic += x[r] * m[r * 4 + c] * x[c];
            Assert.True(quadratic > 0.0, Invariant($"xᵀMx = {quadratic:E3}"));

            Cholesky.Factor(m, 4);
        }
    }

    [Fact]
    public void RotorInertia_AddsNSquaredJmToEachDiagonalEntry_AndChangesNothingElse()
    {
        const double jm = 1.0e-7; // kg·m², before the gear
        var plain = new RecursiveNewtonEuler(s_arm);
        var withRotor = new RecursiveNewtonEuler(WithRotorInertia(s_arm, jm));
        var n = s_arm.Actuator.GearRatio;
        var reflected = n * n * jm;
        Assert.Equal(reflected, withRotor.Model.Actuator.ReflectedRotorInertia);

        var random = new Random(19);
        var m0 = new double[16];
        var m1 = new double[16];
        var h0 = new double[4];
        var h1 = new double[4];
        for (var trial = 0; trial < 200; trial++)
        {
            var q = RandomVector(random, 4, Math.PI);
            var qd = RandomVector(random, 4, 3.0);
            plain.InertiaMatrix(q, m0);
            withRotor.InertiaMatrix(q, m1);

            for (var r = 0; r < 4; r++)
                for (var c = 0; c < 4; c++)
                {
                    if (r == c)
                        Assert.Equal(m0[r * 4 + c] + reflected, m1[r * 4 + c]);
                    else
                        Assert.Equal(m0[r * 4 + c], m1[r * 4 + c]);
                }

            // The bias torques do not depend on Jm.
            plain.BiasTorque(q, qd, Planet.Earth, h0);
            withRotor.BiasTorque(q, qd, Planet.Earth, h1);
            Assert.Equal(h0, h1);
        }
    }
}
