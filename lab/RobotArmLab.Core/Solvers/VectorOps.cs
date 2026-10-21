namespace RobotArmLab.Core.Solvers;

/// <summary>Çözücülerin kullandığı küçük vektör işlemi.</summary>
public static class VectorOps
{
    /// <summary>x + a·k vektörünü yeni bir dizi olarak döndürür (Runge–Kutta ara durumları için).</summary>
    public static double[] AddScaled(double[] x, double a, double[] k)
    {
        double[] result = new double[x.Length];
        for (int i = 0; i < x.Length; i++)
            result[i] = x[i] + a * k[i];
        return result;
    }
}
