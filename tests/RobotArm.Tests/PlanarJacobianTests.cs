using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using G = RobotArm.Core.Model.OpenManipulatorXGeometry;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>SPEC §4.4 (V4 of SPEC §11): planar Jacobian, determinant and singularity guard.</summary>
public sealed class PlanarJacobianTests
{
    private const double DerivativeTol = 1e-6;
    private const double DeterminantTol = 1e-12;
    private const double Step = 1e-6;

    private static double[] RandomPose(Random rng)
    {
        var q = new double[4];
        for (var i = 0; i < 4; i++) q[i] = (rng.NextDouble() * 2.0 - 1.0) * Math.PI;
        return q;
    }

    [Fact]
    public void Jacobian_EqualsCentralDifferenceOfPlanarPose_InRandomPoses()
    {
        var rng = new Random(44);
        for (var n = 0; n < 1000; n++)
        {
            var q = RandomPose(rng);
            var jp = PlanarJacobian.Compute(q);
            for (var col = 0; col < 3; col++)
            {
                var plus = (double[])q.Clone();
                var minus = (double[])q.Clone();
                plus[col + 1] += Step;
                minus[col + 1] -= Step;
                var a = ForwardKinematics.Planar(plus);
                var b = ForwardKinematics.Planar(minus);
                double[] numeric = [(a.R - b.R) / (2 * Step), (a.Z - b.Z) / (2 * Step), (a.Pitch - b.Pitch) / (2 * Step)];
                for (var row = 0; row < 3; row++)
                    Assert.True(Math.Abs(jp[row, col] - numeric[row]) <= DerivativeTol,
                        Invariant($"Jp[{row},{col}] = {jp[row, col]:R}, numeric {numeric[row]:R}"));
            }
        }
    }

    [Fact]
    public void DeterminantFormula_EqualsDeterminantOfJacobian_InRandomPoses()
    {
        var rng = new Random(45);
        for (var n = 0; n < 1000; n++)
        {
            var q = RandomPose(rng);
            var formula = PlanarJacobian.Determinant(q);
            var direct = PlanarJacobian.Compute(q).Determinant;
            Assert.True(Math.Abs(formula - direct) <= DeterminantTol, Invariant($"formula {formula:R}, direct {direct:R}"));
        }
    }

    [Fact]
    public void TryInvert_GivesInverse_AwayFromSingularity()
    {
        double[] q = [0.3, 0.3, 0.25, 0.45];
        Assert.False(PlanarJacobian.IsNearSingular(q));
        Assert.True(PlanarJacobian.TryInvert(q, out var inverse));

        var product = PlanarJacobian.Compute(q) * inverse;
        for (var r = 0; r < 3; r++)
            for (var c = 0; c < 3; c++)
                Assert.True(Math.Abs(product[r, c] - (r == c ? 1.0 : 0.0)) <= 1e-12, Invariant($"(Jp·Jp⁻¹)[{r},{c}] = {product[r, c]:R}"));
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(1e-4)]
    [InlineData(-1e-3)]
    public void TryInvert_Refuses_NearStretchedElbow(double offset)
    {
        // q3 = −β stretches link 2 and link 3 into a line: sin(q3 + β) = 0 (SPEC §4.4).
        double[] q = [0.1, 0.2, -G.Beta + offset, 0.3];
        Assert.True(Math.Abs(PlanarJacobian.Determinant(q)) < PlanarJacobian.DefaultDeterminantThreshold);
        Assert.True(PlanarJacobian.IsNearSingular(q));
        Assert.False(PlanarJacobian.TryInvert(q, out var inverse));
        Assert.Equal(Matrix3d.Zero, inverse);
    }

    [Fact]
    public void Threshold_IsAppliedToAbsoluteDeterminant()
    {
        Assert.Equal(1e-4, PlanarJacobian.DefaultDeterminantThreshold);
        // Elbow folded either way: |det| is the same, both sides invertible.
        double[] up = [0, 0, -G.Beta + 0.5, 0];
        double[] down = [0, 0, -G.Beta - 0.5, 0];
        Assert.Equal(-PlanarJacobian.Determinant(up), PlanarJacobian.Determinant(down), 15);
        Assert.True(PlanarJacobian.TryInvert(up, out _));
        Assert.True(PlanarJacobian.TryInvert(down, out _));
        // A larger threshold refuses what the default accepts.
        Assert.False(PlanarJacobian.TryInvert(up, out _, threshold: 1.0));
    }
}
