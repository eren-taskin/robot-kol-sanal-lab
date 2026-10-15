using System.Numerics;
using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using RobotArm.Core.Model;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.ControlTestHelpers;

namespace RobotArm.Tests;

/// <summary>SPEC §12.1: eigenvalues of a real matrix by Hessenberg reduction and shifted QR (V36).</summary>
public sealed class EigenSolverTests(ITestOutputHelper output)
{
    [Fact]
    public void Eigenvalues_OfDiagonalMatrix_AreItsDiagonal()
    {
        double[] d = [3.0, -1.0, 0.5, 2e-3, -7.0, 0.0];
        var n = d.Length;
        var a = new double[n * n];
        for (var i = 0; i < n; i++) a[i * n + i] = d[i];
        var values = EigenSolver.Eigenvalues(a, n);
        AssertSameSet(d.Select(v => new Complex(v, 0.0)).ToArray(), values, 1e-15);
    }

    [Fact]
    public void Eigenvalues_OfCompanionMatrix_AreThePolynomialRoots()
    {
        Complex[] roots =
        [
            -3.0, -1.5, 0.5, 2.0, 4.0,
            new(1.0, 2.0), new(1.0, -2.0), new(-0.2, 0.7), new(-0.2, -0.7),
        ];
        var n = roots.Length;
        // Monic coefficients c_0 … c_{n−1} of Π (x − r), real because the complex roots come in pairs.
        var coeffs = new Complex[n + 1];
        coeffs[0] = 1.0;
        foreach (var r in roots)
            for (var k = n; k >= 0; k--)
                coeffs[k] = (k > 0 ? coeffs[k - 1] : Complex.Zero) - r * coeffs[k];
        // coeffs[k] multiplies x^k after the loop above (coeffs[n] = 1).
        var a = new double[n * n];
        for (var i = 1; i < n; i++) a[i * n + i - 1] = 1.0;
        for (var i = 0; i < n; i++) a[i * n + n - 1] = -coeffs[i].Real;
        Assert.Equal(1.0, coeffs[n].Real, 1e-15);

        var values = EigenSolver.Eigenvalues(a, n);
        AssertSameSet(roots, values, 1e-9);
    }

    [Fact]
    public void Eigenvalues_OfRotationBlock_AreAnExactConjugatePair()
    {
        const double radius = 0.97;
        const double angle = 0.4;
        var (c, s) = (radius * Math.Cos(angle), radius * Math.Sin(angle));
        var values = EigenSolver.Eigenvalues([c, -s, s, c], 2);
        Assert.Equal(2, values.Length);
        Assert.Equal(values[0].Real, values[1].Real);
        Assert.Equal(values[0].Imaginary, -values[1].Imaginary);
        Assert.True(values[0].Imaginary > 0.0);
        Assert.Equal(radius, Complex.Abs(values[0]), 1e-15);
        Assert.Equal(angle, values[0].Phase, 1e-14);

        // The same block with a non-normal coupling and a real eigenvalue beside it.
        var b = new double[] { c, -20.0 * s, 1.0, s / 20.0, c, 2.0, 0.0, 0.0, 0.3 };
        var v3 = EigenSolver.Eigenvalues(b, 3);
        AssertSameSet([Complex.FromPolarCoordinates(radius, angle), Complex.FromPolarCoordinates(radius, -angle), 0.3], v3, 1e-14);
    }

    /// <summary>A = V D V⁻¹ with D of rotation blocks and reals, 16 × 16: the known spectrum comes back.</summary>
    [Fact]
    public void Eigenvalues_Of16x16Similarity_MatchKnownSpectrum()
    {
        const int n = 16;
        var random = new Random(16);
        var d = new double[n * n];
        var expected = new List<Complex>();
        for (var b = 0; b < 6; b++)
        {
            var r = 0.2 + 0.15 * b;
            var phi = 0.3 + 0.4 * b;
            var (c, s) = (r * Math.Cos(phi), r * Math.Sin(phi));
            var i = 2 * b;
            d[i * n + i] = c; d[i * n + i + 1] = -s; d[(i + 1) * n + i] = s; d[(i + 1) * n + i + 1] = c;
            expected.Add(Complex.FromPolarCoordinates(r, phi));
            expected.Add(Complex.FromPolarCoordinates(r, -phi));
        }
        double[] reals = [-1.7, 0.05, 1.3, 2.2];
        for (var k = 0; k < reals.Length; k++)
        {
            d[(12 + k) * n + 12 + k] = reals[k];
            expected.Add(reals[k]);
        }
        var v = Enumerable.Range(0, n * n).Select(_ => random.NextDouble() - 0.5).ToArray();
        for (var i = 0; i < n; i++) v[i * n + i] += 2.0;
        var a = DenseMatrix.Multiply(DenseMatrix.Multiply(v, d, n, n, n), DenseMatrix.Inverse(v, n), n, n, n);

        var values = EigenSolver.Eigenvalues(a, n);
        var error = AssertSameSet([.. expected], values, 1e-10);
        output.WriteLine(Invariant($"16 × 16: largest eigenvalue error {error:E2}"));
    }

    /// <summary>Σλ = trace and Πλ = det on random 12 × 12 matrices (det by LU).</summary>
    [Fact]
    public void Eigenvalues_OfRandom12x12_SumToTraceAndMultiplyToDeterminant()
    {
        const int n = 12;
        var random = new Random(12);
        var worstSum = 0.0;
        var worstProduct = 0.0;
        for (var trial = 0; trial < 50; trial++)
        {
            var a = Enumerable.Range(0, n * n).Select(_ => 2.0 * random.NextDouble() - 1.0).ToArray();
            var values = EigenSolver.Eigenvalues(a, n);
            var trace = 0.0;
            for (var i = 0; i < n; i++) trace += a[i * n + i];
            var sum = values.Aggregate(Complex.Zero, (x, y) => x + y);
            var product = values.Aggregate(Complex.One, (x, y) => x * y);
            var det = DenseMatrix.Determinant(a, n);
            var norm = Math.Sqrt(a.Sum(x => x * x));

            worstSum = Math.Max(worstSum, Complex.Abs(sum - trace) / norm);
            worstProduct = Math.Max(worstProduct, Complex.Abs(product - det) / Math.Abs(det));
            Assert.True(Complex.Abs(sum - trace) <= 1e-12 * norm, Invariant($"trial {trial}: Σλ − trace = {Complex.Abs(sum - trace)}"));
            Assert.True(Complex.Abs(product - det) <= 1e-9 * Math.Abs(det), Invariant($"trial {trial}: Πλ = {product}, det = {det}"));
            Assert.True(Math.Abs(sum.Imaginary) <= 1e-12 * norm);
        }
        output.WriteLine(Invariant($"50 random 12 × 12: |Σλ − trace|/‖A‖F ≤ {worstSum:E2}, |Πλ − det|/|det| ≤ {worstProduct:E2}"));
    }

    /// <summary>
    /// max |λ| of the arm's sampled closed loop equals the spectral radius from repeated squaring of
    /// <see cref="SampledLoopStability"/> (T07), for every controller, planet, payload (0, 0.5 kg) and pose.
    /// </summary>
    [Fact]
    public void SpectralRadius_AgreesWithRepeatedSquaring_OnTheArm()
    {
        var worst = 0.0;
        foreach (var kind in Enum.GetValues<ControllerKind>())
            foreach (var planet in Planet.All)
                foreach (var payload in Payloads)
                    foreach (var q0 in StabilityPoses)
                    {
                        var system = LinearSystem.ArmAtRest(ArmModel.OpenManipulatorX(), planet, payload, q0);
                        var ctl = (ILinearisableController)Create(kind, ControllerBelief.OpenManipulatorX(planet, payload));
                        var k = Gain((IController)ctl, q0);
                        var cl = SampledLoopStability.ClosedLoop(system, k, ControlLoop.DefaultSamplePeriod);
                        var gelfand = SampledLoopStability.SpectralRadius(cl, 8);
                        var qr = EigenSolver.SpectralRadius(cl, 8);
                        worst = Math.Max(worst, Math.Abs(gelfand - qr));
                        Assert.Equal(gelfand, qr, 1e-9);
                    }
        output.WriteLine(Invariant($"largest |ρ_QR − ρ_squaring| = {worst:E2}"));
    }

    [Fact]
    public void Eigenvalues_RejectBadInput()
    {
        Assert.Throws<ArgumentException>(() => EigenSolver.Eigenvalues(new double[3], 2));
        Assert.Throws<ArgumentException>(() => EigenSolver.Eigenvalues([1.0, double.NaN, 0.0, 1.0], 2));
        Assert.Throws<ArgumentException>(() => EigenSolver.Eigenvalues([], 0));
        Assert.Equal(new Complex(-4.0, 0.0), EigenSolver.Eigenvalues([-4.0], 1)[0]);
    }

    /// <summary>Each expected value is matched to a distinct computed one within the tolerance; returns the largest error.</summary>
    private static double AssertSameSet(Complex[] expected, Complex[] actual, double tolerance)
    {
        Assert.Equal(expected.Length, actual.Length);
        var used = new bool[actual.Length];
        var worst = 0.0;
        foreach (var e in expected)
        {
            var best = -1;
            for (var j = 0; j < actual.Length; j++)
                if (!used[j] && (best < 0 || Complex.Abs(actual[j] - e) < Complex.Abs(actual[best] - e))) best = j;
            used[best] = true;
            var error = Complex.Abs(actual[best] - e);
            worst = Math.Max(worst, error);
            Assert.True(error <= tolerance * Math.Max(1.0, Complex.Abs(e)), Invariant($"expected {e}, closest {actual[best]}, error {error:E2}"));
        }
        return worst;
    }
}
