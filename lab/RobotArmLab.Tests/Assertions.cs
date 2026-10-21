namespace RobotArmLab.Tests;

internal static class Check
{
    /// <summary>|a − b| ≤ tol · max(1e-300, max |b|) for every entry: relative to the size of the reference.</summary>
    public static void RelativeClose(double[] expected, double[] actual, double tol, string what)
    {
        double scale = 0.0;
        for (int i = 0; i < expected.Length; i++)
            scale = Math.Max(scale, Math.Abs(expected[i]));
        for (int i = 0; i < expected.Length; i++)
        {
            double err = Math.Abs(expected[i] - actual[i]);
            Assert.True(err <= tol * Math.Max(scale, 1e-300),
                $"{what}[{i}]: expected {expected[i]:R}, got {actual[i]:R}, error {err:E3}, scale {scale:E3}");
        }
    }

    public static void RelativeClose(double[,] expected, double[,] actual, double tol, string what)
    {
        double[] e = new double[16], a = new double[16];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) { e[4 * i + j] = expected[i, j]; a[4 * i + j] = actual[i, j]; }
        RelativeClose(e, a, tol, what);
    }

    public static double[] RandomQInLimits(Random rng, RobotArmLab.Core.Model.ArmParameters p)
    {
        double[] q = new double[4];
        for (int i = 0; i < 4; i++)
            q[i] = p.LowerLimitRad[i] + rng.NextDouble() * (p.UpperLimitRad[i] - p.LowerLimitRad[i]);
        return q;
    }

    public static double[] Uniform(Random rng, double half)
    {
        double[] v = new double[4];
        for (int i = 0; i < 4; i++)
            v[i] = (2.0 * rng.NextDouble() - 1.0) * half;
        return v;
    }
}
