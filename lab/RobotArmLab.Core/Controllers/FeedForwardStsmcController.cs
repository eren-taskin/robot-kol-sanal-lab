using RobotArmLab.Core.Dynamics;

namespace RobotArmLab.Core.Controllers;

/// <summary>
/// İleri beslemeli süper-büküm kayan kip kontrolü (FF-STSMC), Saied ve diğ., IEEE Transactions on Robotics 2023,
/// DOI 10.1109/TRO.2023.3255586. Geri besleme STSMC ile aynıdır, ama model terimi ölçülen durumda değil İSTENEN
/// yörüngede hesaplanır:
/// τ = M̂(q_ref)(q̈_ref + Λė + u_st) + ĉ(q_ref, q̇_ref) + ĝ(q_ref) + f̂(q̇_ref),
/// s = ė + Λe, u_st = k1·|s|^½·sign(s) + v, v̇ = k2·sign(s). Çıkış ±τmax'a kırpılır.
/// </summary>
/// <remarks>
/// Denklem numarası makaleden teyit edilecek. Makaleye erişilemediği için yapı görev tanımına göre kurulmuştur;
/// geri besleme teriminin de M̂(q_ref) ile çarpılması (M̂(q) yerine) makaleden teyit edilecektir.
///
/// Neden: ölçüm (kodlayıcı nicemlemesi, gürültü, sonlu farkla bulunan hız) modelin içinden geçmez; ĉ, ĝ ve f̂
/// pürüzsüz referanstan hesaplanır, bu yüzden tork komutu daha az gürültülüdür. Ölçüm yalnız e ve ė üzerinden,
/// süper-büküm geri beslemesine girer. Bedeli: kol referanstan uzaktayken model terimi kolun bulunduğu yerin
/// değil referansın dinamiğidir; aradaki fark geri beslemeye bozucu olarak yüklenir. Model referansa bağlı
/// olduğundan yarım örnek öngörüsü gerekmez (bayat ölçülmüş hız bir geri besleme döngüsü oluşturmaz).
/// </remarks>
public class FeedForwardStsmcController : IController
{
    private readonly SuperTwistingTerm twisting = new SuperTwistingTerm();
    private double lastTimeS;
    private bool started;

    /// <summary>Kontrolcünün kendi (tahmini) dinamik modeli.</summary>
    public ArmDynamics Model { get; }

    /// <summary>Kazançlar (STSMC ile aynı sınıf; yarım örnek öngörüsü kullanılmaz).</summary>
    public SuperTwistingGains Gains { get; }

    /// <summary>Süper-büküm terimi (v ve s grafikler için okunabilir).</summary>
    public SuperTwistingTerm Twisting { get { return twisting; } }

    /// <summary>Arayüzde gösterilen ad.</summary>
    public string Name { get { return "İleri beslemeli süper-büküm (FF-STSMC)"; } }

    /// <summary>Tahmini model ve kazançlarla kontrolcüyü kurar.</summary>
    public FeedForwardStsmcController(ArmDynamics model, SuperTwistingGains gains)
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

    /// <summary>FF-STSMC torku, N·m (±τmax'a kırpılmış).</summary>
    public double[] Compute(double t, double[] q, double[] qd, double[] qRef, double[] qdRef, double[] qddRef)
    {
        double dt = started ? t - lastTimeS : 0.0;
        started = true;
        lastTimeS = t;

        double[] a = new double[4];   // istenen ivme, rad/s²
        for (int i = 0; i < 4; i++)
        {
            double e = qRef[i] - q[i];
            double eDot = qdRef[i] - qd[i];
            double s = eDot + Gains.Lambda[i] * e;
            a[i] = qddRef[i] + Gains.Lambda[i] * eDot + twisting.Evaluate(i, s, dt, Gains.K1[i], Gains.K2[i]);
        }

        double[] tau = Model.InverseDynamics(qRef, qdRef, a);   // M̂(q_ref) a + ĉ(q_ref, q̇_ref) + ĝ(q_ref) + f̂(q̇_ref)
        SuperTwistingController.CommitUnclamped(twisting, tau, Model.Parameters.TorqueLimitNm);
        return HalfSample.Clamp(tau, Model.Parameters.TorqueLimitNm);
    }
}
