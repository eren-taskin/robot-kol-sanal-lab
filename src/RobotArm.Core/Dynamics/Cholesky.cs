namespace RobotArm.Core.Dynamics;

/// <summary>
/// In-place Cholesky decomposition A = L Lᵀ of a symmetric positive definite matrix and the solution of
/// A x = b, used for forward dynamics M q̈ = τ − τf − h. SPEC §5.2: M is never inverted explicitly.
/// Matrices are row-major n×n spans. No method allocates.
/// </summary>
public static class Cholesky
{
    /// <summary>
    /// Overwrites the lower triangle of <paramref name="a"/> with L, where A = L Lᵀ. Only the lower triangle
    /// of A is read; the strict upper triangle is set to zero. SPEC §5.2.
    /// </summary>
    /// <param name="a">Row-major n×n symmetric matrix; on return L.</param>
    /// <param name="n">Matrix size.</param>
    /// <exception cref="ArgumentException">
    /// <paramref name="a"/> is not n×n, or the matrix is not positive definite (a pivot is not greater than 0
    /// or not finite); the message names the pivot.
    /// </exception>
    public static void Factor(Span<double> a, int n)
    {
        RequireSquare(a.Length, n, nameof(a));
        for (var j = 0; j < n; j++)
        {
            var row = j * n;
            var d = a[row + j];
            for (var k = 0; k < j; k++)
                d -= a[row + k] * a[row + k];
            if (!(d > 0.0) || !double.IsFinite(d))
                throw new ArgumentException(
                    $"The matrix is not positive definite: pivot {j} is {d:R} after elimination (SPEC §5.2, Cholesky).",
                    nameof(a));

            var ljj = Math.Sqrt(d);
            a[row + j] = ljj;
            for (var i = j + 1; i < n; i++)
            {
                var s = a[i * n + j];
                for (var k = 0; k < j; k++)
                    s -= a[i * n + k] * a[row + k];
                a[i * n + j] = s / ljj;
            }
            for (var k = j + 1; k < n; k++)
                a[row + k] = 0.0;
        }
    }

    /// <summary>
    /// Solves L Lᵀ x = b in place, given the factor from <see cref="Factor"/>. SPEC §5.2.
    /// </summary>
    /// <param name="l">Row-major n×n factor L (lower triangle).</param>
    /// <param name="n">Matrix size.</param>
    /// <param name="b">Right-hand side; on return x.</param>
    /// <exception cref="ArgumentException">A span has the wrong length.</exception>
    public static void Solve(ReadOnlySpan<double> l, int n, Span<double> b)
    {
        RequireSquare(l.Length, n, nameof(l));
        if (b.Length != n)
            throw new ArgumentException($"Expected {n} right-hand-side values, got {b.Length}.", nameof(b));

        // Forward substitution L y = b.
        for (var i = 0; i < n; i++)
        {
            var s = b[i];
            for (var k = 0; k < i; k++)
                s -= l[i * n + k] * b[k];
            b[i] = s / l[i * n + i];
        }
        // Back substitution Lᵀ x = y.
        for (var i = n - 1; i >= 0; i--)
        {
            var s = b[i];
            for (var k = i + 1; k < n; k++)
                s -= l[k * n + i] * b[k];
            b[i] = s / l[i * n + i];
        }
    }

    /// <summary>
    /// Factors <paramref name="a"/> in place and solves A x = b in place: on return <paramref name="a"/>
    /// holds L and <paramref name="b"/> holds x. SPEC §5.2.
    /// </summary>
    /// <exception cref="ArgumentException">Wrong lengths, or the matrix is not positive definite.</exception>
    public static void FactorAndSolve(Span<double> a, int n, Span<double> b)
    {
        Factor(a, n);
        Solve(a, n, b);
    }

    private static void RequireSquare(int length, int n, string name)
    {
        if (n <= 0 || length != n * n)
            throw new ArgumentException($"Expected a {n}×{n} matrix ({n * n} entries), got {length} entries.", name);
    }
}
