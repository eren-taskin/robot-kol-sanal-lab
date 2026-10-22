using System;
using RobotArmLab.Core.Dynamics;

namespace RobotArmLab.Core.Controllers;

/// <summary>
/// Hesaplanmış tork (klasik ters dinamik kontrolü): τ = M̂(q)(q̈_ref + Kd ė + Kp e) + ĉ(q, q̇) + ĝ(q) + f̂(q̇),
/// e = q_ref − q, ė = q̇_ref − q̇. Çıkış ±τmax'a kırpılır.
/// </summary>
/// <remarks>
/// Şapkalı terimler kontrolcünün KENDİ modelinden (model alanı) gelir; bu model tahmini yükle
/// kurulur. Model doğruysa doğrusal olmayan terimler tam sökülür ve her eklem ë + Kd ė + Kp e = 0 gibi davranır.
/// Model yanlışsa (gerçek yük ≠ tahmini yük) kalan fark bozucu olarak döngüde kalır.
///
/// Kaynak: klasik hesaplanmış tork yasası; Şen ve Öke Günel 2023, Transactions of the Institute of Measurement
/// and Control, DOI 10.1177/01423312231153255, bu yasayı temel kontrol yasası olarak kullanır.
///
/// Yarım örnek öngörüsü (<see cref="ComputedTorqueGains.PredictHalfSample"/>): tork Ts boyunca tutulur; model,
/// eklemler a ivmesiyle hızlanırsa tutmanın ortasında varılacak durumda hesaplanır:
/// q̄ = q + q̇ Δt/2 + a Δt²/8, q̄̇ = q̇ + a Δt/2 (Δt: önceki örnekten beri geçen süre, ilk örnekte 0).
/// </remarks>
public class ComputedTorqueController : IController
{
    private readonly ArmDynamics model;   // kontrolcünün kendi (tahmini) dinamik modeli
    private readonly ComputedTorqueGains gains;
    private double lastTimeS;
    private bool started;

    /// <summary>Tahmini model ve kazançlarla kontrolcüyü kurar. τmax modelin parametrelerinden alınır.</summary>
    public ComputedTorqueController(ArmDynamics model, ComputedTorqueGains gains)
    {
        this.model = model;
        this.gains = gains;
    }

    /// <summary>Örnek zamanını sıfırlar (bu kontrolcünün başka iç durumu yoktur).</summary>
    public void Reset()
    {
        started = false;
        lastTimeS = 0.0;
    }

    /// <summary>Hesaplanmış tork, N·m (±τmax'a kırpılmış).</summary>
    public double[] Compute(double t, double[] q, double[] qd, double[] qRef, double[] qdRef, double[] qddRef)
    {
        double dt = started && gains.PredictHalfSample ? t - lastTimeS : 0.0;
        started = true;
        lastTimeS = t;

        double[] a = new double[4];   // istenen ivme, rad/s²
        for (int i = 0; i < 4; i++)
            a[i] = qddRef[i] + gains.Kd[i] * (qdRef[i] - qd[i]) + gains.Kp[i] * (qRef[i] - q[i]);

        double[] qBar = new double[4], qdBar = new double[4];
        HalfSample.Predict(q, qd, a, dt, qBar, qdBar);
        double[] tau = model.InverseDynamics(qBar, qdBar, a);   // M̂(q̄) a + ĉ + ĝ + f̂
        return HalfSample.Clamp(tau, model.Parameters.TorqueLimitNm);
    }
}

/// <summary>Model tabanlı kontrolcülerin ortak iki küçük yardımcısı: yarım örnek öngörüsü ve tork kırpma.</summary>
internal static class HalfSample
{
    /// <summary>q̄ = q + q̇ Δt/2 + a Δt²/8, q̄̇ = q̇ + a Δt/2. Δt = 0 ise q̄ = q, q̄̇ = q̇ (ders kitabı yasası).</summary>
    public static void Predict(double[] q, double[] qd, double[] a, double dt, double[] qBar, double[] qdBar)
    {
        for (int i = 0; i < 4; i++)
        {
            qBar[i] = q[i] + qd[i] * dt / 2.0 + a[i] * dt * dt / 8.0;   // q + q̇·(Δt/2) + ½·a·(Δt/2)²
            qdBar[i] = qd[i] + a[i] * dt / 2.0;
        }
    }

    /// <summary>Her elemanı ±limit aralığına kırpar (yerinde) ve aynı diziyi döndürür.</summary>
    public static double[] Clamp(double[] tau, double limit)
    {
        for (int i = 0; i < 4; i++)
            tau[i] = Math.Max(-limit, Math.Min(limit, tau[i]));
        return tau;
    }
}
