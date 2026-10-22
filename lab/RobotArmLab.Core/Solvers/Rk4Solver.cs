namespace RobotArmLab.Core.Solvers;

/// <summary>
/// Klasik dördüncü mertebe Runge–Kutta (RK4), rapor eş. (22). Adım başına dört eğim değerlendirir
/// (başta, iki kez ortada, sonda) ve bunların 1-2-2-1 ağırlıklı ortalamasını kullanır. Hatası h⁴ ile orantılıdır.
/// </summary>
public class Rk4Solver : IOdeSolver
{
    /// <summary>Bir RK4 adımı: x + h/6 · (k1 + 2k2 + 2k3 + k4).</summary>
    public double[] Step(OdeFunction f, double t, double[] x, double h)
    {
        double[] k1 = f(t, x);
        double[] k2 = f(t + 0.5 * h, VectorOps.AddScaled(x, 0.5 * h, k1));
        double[] k3 = f(t + 0.5 * h, VectorOps.AddScaled(x, 0.5 * h, k2));
        double[] k4 = f(t + h, VectorOps.AddScaled(x, h, k3));

        double[] result = new double[x.Length];
        for (int i = 0; i < x.Length; i++)
            result[i] = x[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        return result;
    }
}
