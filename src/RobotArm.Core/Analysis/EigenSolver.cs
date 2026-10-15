using System.Numerics;

namespace RobotArm.Core.Analysis;

/// <summary>
/// Eigenvalues of a real square matrix: balancing, reduction to upper Hessenberg form, then the shifted QR
/// algorithm (Francis double shift). SPEC §12.1; docs/DECISIONS.md D9.
/// </summary>
/// <remarks>
/// <para>
/// The steps are the classical ones (Parlett–Reinsch balancing; Hessenberg reduction by stabilised elementary
/// similarity transformations; the double-shift QR iteration of EISPACK hqr, as in Numerical Recipes): each
/// similarity transformation keeps the eigenvalues; balancing scales rows and columns by powers of two (exact in
/// floating point) so that badly scaled matrices, such as the arm's closed loop where the wrist inertia is 1e-4 of the
/// gains, lose no accuracy. Two consecutive exceptional shifts are used when a block stalls.
/// </para>
/// <para>
/// Written for the small matrices of this project (closed loops of 2 to 16 states, tested up to 16 × 16); it works
/// for any size but costs O(n³). Eigenvectors are not computed. Allocates; not for once per step.
/// </para>
/// </remarks>
public static class EigenSolver
{
    /// <summary>Iterations allowed per eigenvalue before the solver gives up.</summary>
    public const int MaxIterationsPerEigenvalue = 60;

    /// <summary>
    /// The n eigenvalues of the n × n row-major <paramref name="matrix"/>, sorted by decreasing modulus (complex
    /// pairs with the positive imaginary part first). A real matrix has real eigenvalues and complex conjugate pairs;
    /// a pair is returned exactly conjugate. SPEC §12.1.
    /// </summary>
    /// <exception cref="ArgumentException">The matrix does not have n × n entries, or an entry is not finite.</exception>
    /// <exception cref="InvalidOperationException">The QR iteration did not converge (not seen in practice).</exception>
    public static Complex[] Eigenvalues(ReadOnlySpan<double> matrix, int n)
    {
        DenseMatrix.CheckSquare(matrix, n, nameof(matrix));
        var a = matrix.ToArray();
        Balance(a, n);
        ReduceToHessenberg(a, n);
        var values = HessenbergQr(a, n);
        Array.Sort(values, (x, y) =>
        {
            var byModulus = Complex.Abs(y).CompareTo(Complex.Abs(x));
            if (byModulus != 0) return byModulus;
            var byReal = y.Real.CompareTo(x.Real);
            return byReal != 0 ? byReal : y.Imaginary.CompareTo(x.Imaginary);
        });
        return values;
    }

    /// <summary>max |λ|, the spectral radius. SPEC §12.1.</summary>
    public static double SpectralRadius(ReadOnlySpan<double> matrix, int n)
    {
        var rho = 0.0;
        foreach (var z in Eigenvalues(matrix, n)) rho = Math.Max(rho, Complex.Abs(z));
        return rho;
    }

    /// <summary>Parlett–Reinsch balancing by powers of 2 (a similarity transformation D⁻¹ A D, exact).</summary>
    private static void Balance(double[] a, int n)
    {
        const double radix = 2.0;
        const double radixSquared = radix * radix;
        var done = false;
        while (!done)
        {
            done = true;
            for (var i = 0; i < n; i++)
            {
                double r = 0.0, c = 0.0;
                for (var j = 0; j < n; j++)
                {
                    if (j == i) continue;
                    c += Math.Abs(a[j * n + i]);
                    r += Math.Abs(a[i * n + j]);
                }
                if (c == 0.0 || r == 0.0) continue;
                var g = r / radix;
                var f = 1.0;
                var s = c + r;
                while (c < g) { f *= radix; c *= radixSquared; }
                g = r * radix;
                while (c > g) { f /= radix; c /= radixSquared; }
                if ((c + r) / f < 0.95 * s)
                {
                    done = false;
                    g = 1.0 / f;
                    for (var j = 0; j < n; j++) a[i * n + j] *= g;
                    for (var j = 0; j < n; j++) a[j * n + i] *= f;
                }
            }
        }
    }

    /// <summary>Reduction to upper Hessenberg form by elimination with pivoting (similarity transformations).</summary>
    private static void ReduceToHessenberg(double[] a, int n)
    {
        for (var m = 1; m < n - 1; m++)
        {
            var x = 0.0;
            var pivot = m;
            for (var j = m; j < n; j++)
                if (Math.Abs(a[j * n + m - 1]) > Math.Abs(x)) { x = a[j * n + m - 1]; pivot = j; }
            if (pivot != m)
            {
                for (var j = m - 1; j < n; j++) (a[pivot * n + j], a[m * n + j]) = (a[m * n + j], a[pivot * n + j]);
                for (var j = 0; j < n; j++) (a[j * n + pivot], a[j * n + m]) = (a[j * n + m], a[j * n + pivot]);
            }
            if (x == 0.0) continue;
            for (var i = m + 1; i < n; i++)
            {
                var y = a[i * n + m - 1];
                if (y == 0.0) continue;
                y /= x;
                a[i * n + m - 1] = 0.0;
                for (var j = m; j < n; j++) a[i * n + j] -= y * a[m * n + j];
                for (var j = 0; j < n; j++) a[j * n + m] += y * a[j * n + i];
            }
        }
        for (var i = 2; i < n; i++)
            for (var j = 0; j < i - 1; j++) a[i * n + j] = 0.0;
    }

    /// <summary>
    /// Eigenvalues of an upper Hessenberg matrix by the Francis double-shift QR iteration with deflation
    /// (EISPACK hqr). Indices are 1-based inside, as in the published algorithm, through <c>H(i, j)</c>.
    /// </summary>
    private static Complex[] HessenbergQr(double[] a, int n)
    {
        ref double H(int i, int j) => ref a[(i - 1) * n + (j - 1)];

        var wr = new double[n + 1];
        var wi = new double[n + 1];
        var anorm = 0.0;
        for (var i = 1; i <= n; i++)
            for (var j = Math.Max(i - 1, 1); j <= n; j++) anorm += Math.Abs(H(i, j));

        var nn = n;
        var t = 0.0;
        double p = 0, q = 0, r = 0, s, w = 0, x, y = 0, z = 0;
        while (nn >= 1)
        {
            var its = 0;
            int l;
            do
            {
                // Look for a single small subdiagonal element to split the matrix.
                for (l = nn; l >= 2; l--)
                {
                    s = Math.Abs(H(l - 1, l - 1)) + Math.Abs(H(l, l));
                    if (s == 0.0) s = anorm;
                    if (Math.Abs(H(l, l - 1)) + s == s)
                    {
                        H(l, l - 1) = 0.0;
                        break;
                    }
                }
                if (l < 1) l = 1;
                x = H(nn, nn);
                if (l == nn)
                {
                    // One root found.
                    wr[nn] = x + t;
                    wi[nn] = 0.0;
                    nn--;
                }
                else
                {
                    y = H(nn - 1, nn - 1);
                    w = H(nn, nn - 1) * H(nn - 1, nn);
                    if (l == nn - 1)
                    {
                        // Two roots found.
                        p = 0.5 * (y - x);
                        q = p * p + w;
                        z = Math.Sqrt(Math.Abs(q));
                        x += t;
                        if (q >= 0.0)
                        {
                            z = p + (p >= 0.0 ? Math.Abs(z) : -Math.Abs(z));
                            wr[nn - 1] = wr[nn] = x + z;
                            if (z != 0.0) wr[nn] = x - w / z;
                            wi[nn - 1] = wi[nn] = 0.0;
                        }
                        else
                        {
                            wr[nn - 1] = wr[nn] = x + p;
                            wi[nn - 1] = -z;
                            wi[nn] = z;
                        }
                        nn -= 2;
                    }
                    else
                    {
                        if (its == MaxIterationsPerEigenvalue)
                            throw new InvalidOperationException("The QR iteration did not converge.");
                        if (its == 10 || its == 20 || its == 40)
                        {
                            // Exceptional shift.
                            t += x;
                            for (var i = 1; i <= nn; i++) H(i, i) -= x;
                            s = Math.Abs(H(nn, nn - 1)) + Math.Abs(H(nn - 1, nn - 2));
                            y = x = 0.75 * s;
                            w = -0.4375 * s * s;
                        }
                        its++;

                        // Form the shift and look for two consecutive small subdiagonal elements.
                        int m;
                        for (m = nn - 2; m >= l; m--)
                        {
                            z = H(m, m);
                            r = x - z;
                            s = y - z;
                            p = (r * s - w) / H(m + 1, m) + H(m, m + 1);
                            q = H(m + 1, m + 1) - z - r - s;
                            r = H(m + 2, m + 1);
                            s = Math.Abs(p) + Math.Abs(q) + Math.Abs(r);
                            p /= s;
                            q /= s;
                            r /= s;
                            if (m == l) break;
                            var u = Math.Abs(H(m, m - 1)) * (Math.Abs(q) + Math.Abs(r));
                            var v = Math.Abs(p) * (Math.Abs(H(m - 1, m - 1)) + Math.Abs(z) + Math.Abs(H(m + 1, m + 1)));
                            if (u + v == v) break;
                        }
                        for (var i = m + 2; i <= nn; i++)
                        {
                            H(i, i - 2) = 0.0;
                            if (i != m + 2) H(i, i - 3) = 0.0;
                        }

                        // Double QR step on rows l..nn and columns m..nn.
                        for (var k = m; k <= nn - 1; k++)
                        {
                            if (k != m)
                            {
                                p = H(k, k - 1);
                                q = H(k + 1, k - 1);
                                r = 0.0;
                                if (k != nn - 1) r = H(k + 2, k - 1);
                                x = Math.Abs(p) + Math.Abs(q) + Math.Abs(r);
                                if (x != 0.0)
                                {
                                    p /= x;
                                    q /= x;
                                    r /= x;
                                }
                            }
                            var norm = Math.Sqrt(p * p + q * q + r * r);
                            s = p >= 0.0 ? norm : -norm;
                            if (s == 0.0) continue;
                            if (k == m)
                            {
                                if (l != m) H(k, k - 1) = -H(k, k - 1);
                            }
                            else
                            {
                                H(k, k - 1) = -s * x;
                            }
                            p += s;
                            x = p / s;
                            y = q / s;
                            z = r / s;
                            q /= p;
                            r /= p;
                            for (var j = k; j <= nn; j++)
                            {
                                p = H(k, j) + q * H(k + 1, j);
                                if (k != nn - 1)
                                {
                                    p += r * H(k + 2, j);
                                    H(k + 2, j) -= p * z;
                                }
                                H(k + 1, j) -= p * y;
                                H(k, j) -= p * x;
                            }
                            var mmin = nn < k + 3 ? nn : k + 3;
                            for (var i = l; i <= mmin; i++)
                            {
                                p = x * H(i, k) + y * H(i, k + 1);
                                if (k != nn - 1)
                                {
                                    p += z * H(i, k + 2);
                                    H(i, k + 2) -= p * r;
                                }
                                H(i, k + 1) -= p * q;
                                H(i, k) -= p;
                            }
                        }
                    }
                }
            } while (l < nn - 1);
        }

        var values = new Complex[n];
        for (var i = 0; i < n; i++) values[i] = new Complex(wr[i + 1], wi[i + 1]);
        return values;
    }
}
