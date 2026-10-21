using System;

namespace RobotArmLab.Core.Numerics;

/// <summary>
/// 4×4 matrislerle yapılan küçük doğrusal cebir işlemleri. En önemlisi Cholesky çözücüsüdür: M q̈ = b
/// denklemi M'nin tersini almadan çözülür (SPEC §5.2). M simetrik ve pozitif tanımlı olduğundan
/// M = L Lᵀ biçiminde ayrışır; L alt üçgen matristir.
/// </summary>
public static class Small4x4
{
    /// <summary>
    /// Cholesky ayrışımı M = L·Lᵀ. L alt üçgen matrisi döndürür. M pozitif tanımlı değilse (köşegende
    /// sıfır veya negatif değer çıkarsa) istisna fırlatır: fiziksel bir kütle matrisi için bu olmamalıdır.
    /// </summary>
    public static double[,] Cholesky(double[,] m)
    {
        double[,] l = new double[4, 4];
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                double sum = m[i, j];
                for (int k = 0; k < j; k++)
                    sum -= l[i, k] * l[j, k];

                if (i == j)
                {
                    if (!(sum > 0.0))
                        throw new InvalidOperationException("Matris pozitif tanımlı değil (Cholesky başarısız).");
                    l[i, i] = Math.Sqrt(sum);
                }
                else
                {
                    l[i, j] = sum / l[j, j];
                }
            }
        }
        return l;
    }

    /// <summary>
    /// M·x = b denklemini Cholesky ile çözer: önce L·y = b (ileri yerine koyma), sonra Lᵀ·x = y (geri
    /// yerine koyma). M'nin tersi hiç hesaplanmaz.
    /// </summary>
    public static double[] SolveSymmetricPositiveDefinite(double[,] m, double[] b)
    {
        double[,] l = Cholesky(m);

        double[] y = new double[4];
        for (int i = 0; i < 4; i++)
        {
            double sum = b[i];
            for (int k = 0; k < i; k++)
                sum -= l[i, k] * y[k];
            y[i] = sum / l[i, i];
        }

        double[] x = new double[4];
        for (int i = 3; i >= 0; i--)
        {
            double sum = y[i];
            for (int k = i + 1; k < 4; k++)
                sum -= l[k, i] * x[k];
            x[i] = sum / l[i, i];
        }
        return x;
    }

    /// <summary>Matris–vektör çarpımı A·x (4×4 çarpı 4).</summary>
    public static double[] Multiply(double[,] a, double[] x)
    {
        double[] result = new double[4];
        for (int i = 0; i < 4; i++)
        {
            double sum = 0.0;
            for (int j = 0; j < 4; j++)
                sum += a[i, j] * x[j];
            result[i] = sum;
        }
        return result;
    }
}
