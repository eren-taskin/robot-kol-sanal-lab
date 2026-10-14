using RobotArm.Core.Dynamics;
using static RobotArm.Tests.DynamicsTestHelpers;

namespace RobotArm.Tests;

/// <summary>SPEC §5.2: Cholesky decomposition and solve for symmetric positive definite matrices.</summary>
public sealed class CholeskyTests
{
    [Fact]
    public void Factor_KnownMatrix_GivesKnownFactor()
    {
        // A = L Lᵀ with L = [[2,0,0],[1,3,0],[-1,2,4]].
        double[] a = [4, 2, -2, 2, 10, 5, -2, 5, 21];
        Cholesky.Factor(a, 3);
        Assert.Equal([2, 0, 0, 1, 3, 0, -1, 2, 4], a);
    }

    [Fact]
    public void FactorAndSolve_RandomSpdMatrices_SolveTheSystem()
    {
        var random = new Random(23);
        for (var trial = 0; trial < 200; trial++)
        {
            const int n = 6;
            // A = B Bᵀ + n·1 is symmetric positive definite.
            var b = RandomVector(random, n * n, 1.0);
            var a = new double[n * n];
            for (var r = 0; r < n; r++)
                for (var c = 0; c < n; c++)
                {
                    for (var k = 0; k < n; k++) a[r * n + c] += b[r * n + k] * b[c * n + k];
                    if (r == c) a[r * n + c] += n;
                }
            var x = RandomVector(random, n, 5.0);
            var rhs = new double[n];
            for (var r = 0; r < n; r++)
                for (var c = 0; c < n; c++)
                    rhs[r] += a[r * n + c] * x[c];

            Cholesky.FactorAndSolve(a, n, rhs);
            for (var i = 0; i < n; i++)
                Assert.Equal(x[i], rhs[i], 1e-11);
        }
    }

    [Fact]
    public void Factor_IndefiniteMatrix_ThrowsNamingThePivot()
    {
        double[] a = [1, 2, 2, 1];
        var ex = Assert.Throws<ArgumentException>(() => Cholesky.Factor(a, 2));
        Assert.Contains("not positive definite", ex.Message);
        Assert.Contains("pivot 1", ex.Message);
    }

    [Fact]
    public void Factor_NaN_Throws()
    {
        double[] a = [double.NaN, 0, 0, 1];
        Assert.Throws<ArgumentException>(() => Cholesky.Factor(a, 2));
    }

    [Fact]
    public void WrongLengths_Throw()
    {
        Assert.Throws<ArgumentException>(() => Cholesky.Factor(new double[8], 3));
        Assert.Throws<ArgumentException>(() => Cholesky.Solve(new double[9], 3, new double[2]));
    }
}
