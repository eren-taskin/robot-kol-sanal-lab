using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;

namespace RobotArm.Tests;

/// <summary>Shared helpers for the dynamics tests of SPEC §5.</summary>
internal static class DynamicsTestHelpers
{
    /// <summary>Relative tolerance of SPEC §5.3 / V5 and V7, measured against the largest absolute entry.</summary>
    public const double RelTol = 1e-9;

    /// <summary>General M(q) as a 4×4 array.</summary>
    public static double[,] GeneralInertia(RecursiveNewtonEuler rnea, ReadOnlySpan<double> q)
    {
        var n = rnea.DegreesOfFreedom;
        var flat = new double[n * n];
        rnea.InertiaMatrix(q, flat);
        var m = new double[n, n];
        for (var r = 0; r < n; r++)
            for (var c = 0; c < n; c++)
                m[r, c] = flat[r * n + c];
        return m;
    }

    /// <summary>The vector's M as a 4×4 array.</summary>
    public static double[,] ToArray(double[][] rows)
    {
        var m = new double[rows.Length, rows.Length];
        for (var r = 0; r < rows.Length; r++)
            for (var c = 0; c < rows.Length; c++)
                m[r, c] = rows[r][c];
        return m;
    }

    /// <summary>Largest |a_rc − b_rc|.</summary>
    public static double MaxAbsDifference(double[,] a, double[,] b)
    {
        var d = 0.0;
        for (var r = 0; r < a.GetLength(0); r++)
            for (var c = 0; c < a.GetLength(1); c++)
                d = Math.Max(d, Math.Abs(a[r, c] - b[r, c]));
        return d;
    }

    /// <summary>Largest |a_i − b_i|.</summary>
    public static double MaxAbsDifference(ReadOnlySpan<double> a, ReadOnlySpan<double> b)
    {
        var d = 0.0;
        for (var i = 0; i < a.Length; i++)
            d = Math.Max(d, Math.Abs(a[i] - b[i]));
        return d;
    }

    /// <summary>Largest |a_rc|.</summary>
    public static double MaxAbs(double[,] a)
    {
        var d = 0.0;
        foreach (var x in a)
            d = Math.Max(d, Math.Abs(x));
        return d;
    }

    /// <summary>Largest |a_i|.</summary>
    public static double MaxAbs(ReadOnlySpan<double> a)
    {
        var d = 0.0;
        foreach (var x in a)
            d = Math.Max(d, Math.Abs(x));
        return d;
    }

    /// <summary>Uniform random vector with entries in [−range, range].</summary>
    public static double[] RandomVector(Random random, int n, double range)
    {
        var v = new double[n];
        for (var i = 0; i < n; i++)
            v[i] = range * (2.0 * random.NextDouble() - 1.0);
        return v;
    }

    /// <summary>The OpenMANIPULATOR-X with rotor inertia Jm, everything else as SPEC §3.</summary>
    public static ArmModel WithRotorInertia(ArmModel arm, double jm) =>
        new(arm.Joints, arm.Bodies, arm.ToolPoint, arm.Actuators[0] with { RotorInertia = jm });
}
