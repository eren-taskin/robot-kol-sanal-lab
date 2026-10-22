using System;
using RobotArmLab.Core.Dynamics;

namespace RobotArmLab.Core.Controllers;

/// <summary>
/// Süper-büküm kayan kip kontrolü (STSMC), ölçülen durumda hesaplanan ters dinamiğin üstünde:
/// τ = M̂(q)(q̈_ref + Λė + u_st) + ĉ(q, q̇) + ĝ(q) + f̂(q̇), s = ė + Λe, u_st = k1·|s|^½·sign(s) + v, v̇ = k2·sign(s).
/// e = q_ref − q, ė = q̇_ref − q̇. Çıkış ±τmax'a kırpılır.
/// </summary>
/// <remarks>
/// İşaret kuralı eski çekirdekteki SuperTwistingSlidingMode ile aynıdır. Model doğruysa iç döngü q̈ = q̈_ref + Λė + u_st
/// verir, bu da ṡ = −u_st demektir: klasik süper-büküm döngüsü. Model hatası (ör. habersiz yük) ṡ'ye bozucu olarak
/// girer; türevi L ile sınırlıysa ve k2 &gt; L ise s ve ṡ sonlu sürede sıfıra iner (Levant 1998, Automatica 34(3)).
/// s = 0 olunca hata ė = −Λe ile sönümlenir. Model, CT'deki gibi isteğe bağlı yarım örnek öngörülü durumda
/// hesaplanır; burada a = q̈_ref + Λė + u_st'dir.
/// </remarks>
public class SuperTwistingController : IController
{
    private readonly SuperTwistingTerm twisting = new SuperTwistingTerm();
    private double lastTimeS;
    private bool started;

    /// <summary>Kontrolcünün kendi (tahmini) dinamik modeli.</summary>
    public ArmDynamics Model { get; }

    /// <summary>Kazançlar.</summary>
    public SuperTwistingGains Gains { get; }

    /// <summary>Süper-büküm terimi (v ve s grafikler için okunabilir).</summary>
    public SuperTwistingTerm Twisting { get { return twisting; } }

    /// <summary>Arayüzde gösterilen ad.</summary>
    public string Name { get { return "Süper-büküm kayan kip (STSMC)"; } }

    /// <summary>Tahmini model ve kazançlarla kontrolcüyü kurar.</summary>
    public SuperTwistingController(ArmDynamics model, SuperTwistingGains gains)
    {
        Model = model;
        Gains = gains;
    }

    /// <summary>v, s ve örnek zamanını sıfırlar.</summary>
    public void Reset()
    {
        twisting.Reset();
        started = false;
        lastTimeS = 0.0;
    }

    /// <summary>STSMC torku, N·m (±τmax'a kırpılmış).</summary>
    public double[] Compute(double t, double[] q, double[] qd, double[] qRef, double[] qdRef, double[] qddRef)
    {
        double elapsed = started ? t - lastTimeS : 0.0;
        double dt = Gains.PredictHalfSample ? elapsed : 0.0;
        started = true;
        lastTimeS = t;

        double[] a = new double[4];   // istenen ivme, rad/s²
        for (int i = 0; i < 4; i++)
        {
            double e = qRef[i] - q[i];
            double eDot = qdRef[i] - qd[i];
            double s = eDot + Gains.Lambda[i] * e;
            a[i] = qddRef[i] + Gains.Lambda[i] * eDot + twisting.Evaluate(i, s, elapsed, Gains.K1[i], Gains.K2[i]);
        }

        double[] qBar = new double[4], qdBar = new double[4];
        HalfSample.Predict(q, qd, a, dt, qBar, qdBar);
        double[] tau = Model.InverseDynamics(qBar, qdBar, a);   // M̂(q̄) a + ĉ + ĝ + f̂
        CommitUnclamped(twisting, tau, Model.Parameters.TorqueLimitNm);
        return HalfSample.Clamp(tau, Model.Parameters.TorqueLimitNm);
    }

    /// <summary>Kırpılmayacak eklemlerin v adayını kabul eder (sargı önleme). Kırpmadan önce çağrılır.</summary>
    internal static void CommitUnclamped(SuperTwistingTerm term, double[] tau, double limit)
    {
        for (int i = 0; i < 4; i++)
        {
            if (Math.Abs(tau[i]) <= limit)
                term.Commit(i);
        }
    }
}
