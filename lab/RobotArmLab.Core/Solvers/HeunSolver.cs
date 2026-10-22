namespace RobotArmLab.Core.Solvers;

/// <summary>
/// Heun yöntemi (RK2, düzeltilmiş Euler), 2. mertebe. Önce Euler ile adımın sonunu tahmin eder, sonra
/// baştaki ve sondaki eğimlerin ortalamasıyla adımı tekrar atar. Hatası h² ile orantılıdır.
/// </summary>
public class HeunSolver : IOdeSolver
{
    /// <summary>Bir Heun adımı: x + h/2 · (k1 + k2), k2 Euler tahmini noktasındaki eğim.</summary>
    public double[] Step(OdeFunction f, double t, double[] x, double h)
    {
        double[] k1 = f(t, x);
        double[] k2 = f(t + h, VectorOps.AddScaled(x, h, k1));

        double[] result = new double[x.Length];
        for (int i = 0; i < x.Length; i++)
            result[i] = x[i] + 0.5 * h * (k1[i] + k2[i]);
        return result;
    }
}
