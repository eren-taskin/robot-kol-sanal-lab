namespace RobotArmLab.Core.Solvers;

/// <summary>
/// Açık Euler yöntemi, 1. mertebe: x(t+h) = x(t) + h·f(t, x). En basit yöntemdir; adım başına bir
/// değerlendirme yapar. Hatası h ile orantılıdır ve sürtünmesiz kolda enerjiyi korumaz.
/// </summary>
public class EulerSolver : IOdeSolver
{
    /// <summary>Bir Euler adımı: eğimi adımın başında ölçer ve h boyunca sabit kabul eder.</summary>
    public double[] Step(OdeFunction f, double t, double[] x, double h)
    {
        double[] k1 = f(t, x);
        return VectorOps.AddScaled(x, h, k1);
    }
}
