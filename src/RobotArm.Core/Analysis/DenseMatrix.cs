using System.Globalization;
using System.Numerics;

namespace RobotArm.Core.Analysis;

/// <summary>
/// Small dense row-major matrix helpers for the linear analysis (SPEC §12). Every method allocates its result;
/// they are for analysis, not for once per simulation step.
/// </summary>
internal static class DenseMatrix
{
    public static double[] Identity(int n)
    {
        var m = new double[n * n];
        for (var i = 0; i < n; i++) m[i * n + i] = 1.0;
        return m;
    }

    /// <summary>a (rows × inner) times b (inner × cols).</summary>
    public static double[] Multiply(ReadOnlySpan<double> a, ReadOnlySpan<double> b, int rows, int inner, int cols)
    {
        var c = new double[rows * cols];
        for (var r = 0; r < rows; r++)
            for (var k = 0; k < inner; k++)
            {
                var v = a[r * inner + k];
                if (v == 0.0) continue;
                for (var j = 0; j < cols; j++) c[r * cols + j] += v * b[k * cols + j];
            }
        return c;
    }

    public static double[] Transpose(ReadOnlySpan<double> a, int rows, int cols)
    {
        var t = new double[rows * cols];
        for (var r = 0; r < rows; r++)
            for (var c = 0; c < cols; c++) t[c * rows + r] = a[r * cols + c];
        return t;
    }

    public static double[] Add(ReadOnlySpan<double> a, ReadOnlySpan<double> b, double scaleB = 1.0)
    {
        var c = a.ToArray();
        for (var i = 0; i < c.Length; i++) c[i] += scaleB * b[i];
        return c;
    }

    public static double MaxAbs(ReadOnlySpan<double> a)
    {
        var m = 0.0;
        foreach (var v in a) m = Math.Max(m, Math.Abs(v));
        return m;
    }

    /// <summary>Solves a X = b for X (n × cols) by LU with partial pivoting. Throws if a is singular.</summary>
    public static double[] Solve(ReadOnlySpan<double> a, ReadOnlySpan<double> b, int n, int cols)
    {
        var lu = a.ToArray();
        var x = b.ToArray();
        for (var k = 0; k < n; k++)
        {
            var pivot = k;
            for (var i = k + 1; i < n; i++)
                if (Math.Abs(lu[i * n + k]) > Math.Abs(lu[pivot * n + k])) pivot = i;
            if (lu[pivot * n + k] == 0.0)
                throw new InvalidOperationException("The matrix is singular.");
            if (pivot != k)
            {
                for (var j = 0; j < n; j++) (lu[k * n + j], lu[pivot * n + j]) = (lu[pivot * n + j], lu[k * n + j]);
                for (var j = 0; j < cols; j++) (x[k * cols + j], x[pivot * cols + j]) = (x[pivot * cols + j], x[k * cols + j]);
            }
            for (var i = k + 1; i < n; i++)
            {
                var f = lu[i * n + k] / lu[k * n + k];
                if (f == 0.0) continue;
                for (var j = k; j < n; j++) lu[i * n + j] -= f * lu[k * n + j];
                for (var j = 0; j < cols; j++) x[i * cols + j] -= f * x[k * cols + j];
            }
        }
        for (var k = n - 1; k >= 0; k--)
            for (var j = 0; j < cols; j++)
            {
                var sum = x[k * cols + j];
                for (var m = k + 1; m < n; m++) sum -= lu[k * n + m] * x[m * cols + j];
                x[k * cols + j] = sum / lu[k * n + k];
            }
        return x;
    }

    public static double[] Inverse(ReadOnlySpan<double> a, int n) => Solve(a, Identity(n), n, n);

    /// <summary>det a by LU with partial pivoting (0 for an exactly singular matrix).</summary>
    public static double Determinant(ReadOnlySpan<double> a, int n)
    {
        var lu = a.ToArray();
        var det = 1.0;
        for (var k = 0; k < n; k++)
        {
            var pivot = k;
            for (var i = k + 1; i < n; i++)
                if (Math.Abs(lu[i * n + k]) > Math.Abs(lu[pivot * n + k])) pivot = i;
            if (lu[pivot * n + k] == 0.0) return 0.0;
            if (pivot != k)
            {
                for (var j = 0; j < n; j++) (lu[k * n + j], lu[pivot * n + j]) = (lu[pivot * n + j], lu[k * n + j]);
                det = -det;
            }
            det *= lu[k * n + k];
            for (var i = k + 1; i < n; i++)
            {
                var f = lu[i * n + k] / lu[k * n + k];
                for (var j = k; j < n; j++) lu[i * n + j] -= f * lu[k * n + j];
            }
        }
        return det;
    }

    /// <summary>Solves a x = b for one complex right-hand side by Gaussian elimination with partial pivoting.</summary>
    public static Complex[] Solve(Complex[] a, Complex[] b, int n)
    {
        var lu = (Complex[])a.Clone();
        var x = (Complex[])b.Clone();
        for (var k = 0; k < n; k++)
        {
            var pivot = k;
            for (var i = k + 1; i < n; i++)
                if (Complex.Abs(lu[i * n + k]) > Complex.Abs(lu[pivot * n + k])) pivot = i;
            if (lu[pivot * n + k] == Complex.Zero)
                throw new InvalidOperationException("The matrix is singular.");
            if (pivot != k)
            {
                for (var j = 0; j < n; j++) (lu[k * n + j], lu[pivot * n + j]) = (lu[pivot * n + j], lu[k * n + j]);
                (x[k], x[pivot]) = (x[pivot], x[k]);
            }
            for (var i = k + 1; i < n; i++)
            {
                var f = lu[i * n + k] / lu[k * n + k];
                if (f == Complex.Zero) continue;
                for (var j = k; j < n; j++) lu[i * n + j] -= f * lu[k * n + j];
                x[i] -= f * x[k];
            }
        }
        for (var k = n - 1; k >= 0; k--)
        {
            var sum = x[k];
            for (var m = k + 1; m < n; m++) sum -= lu[k * n + m] * x[m];
            x[k] = sum / lu[k * n + k];
        }
        return x;
    }

    public static void CheckSquare(ReadOnlySpan<double> a, int n, string name)
    {
        if (n <= 0 || a.Length != n * n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"Expected {n} × {n} entries, got {a.Length}."), name);
        foreach (var v in a)
            if (!double.IsFinite(v)) throw new ArgumentException("The matrix must be finite.", name);
    }
}
