using System;

namespace RobotArmLab.Core.Controllers;

/// <summary>
/// Bağımsız eklem PID'i (taban çizgisi): τi = Kp,i ei + Ki,i ∫ei dt + Kd,i ėi, e = q_ref − q, ė = q̇_ref − q̇.
/// Yerçekimi modeli yoktur (yerçekimi torkunu integral karşılar). Çıkış ±τmax'a kırpılır.
/// </summary>
/// <remarks>
/// Kırpma tipi integral sargı önleme (anti-windup): her örnekte aday integral I + e·Δt ile çıkış hesaplanır;
/// çıkış ±τmax'ı aşıyorsa o eklemin integrali bu örnekte güncellenmez. Böylece motor doyumdayken integral
/// büyümeye devam etmez ve doyumdan çıkınca aşırı aşma olmaz. Reset'ten sonraki ilk örnekte Δt = 0'dır.
/// </remarks>
public class PidController : IController
{
    private readonly double torqueLimitNm;
    private readonly double[] integral = new double[4];   // ∫e dt, rad·s
    private double lastTimeS;
    private bool started;

    /// <summary>Kazançlar (kurucuda verilen nesne; koşu sırasında değiştirilmemelidir).</summary>
    public PidGains Gains { get; }

    /// <summary>Arayüzde gösterilen ad.</summary>
    public string Name { get { return "PID (taban çizgisi)"; } }

    /// <summary>Kazançlar ve tork sınırı τmax (N·m) ile kontrolcüyü kurar.</summary>
    public PidController(PidGains gains, double torqueLimitNm)
    {
        Gains = gains;
        this.torqueLimitNm = torqueLimitNm;
    }

    /// <summary>İntegrali ve örnek zamanını sıfırlar.</summary>
    public void Reset()
    {
        Array.Clear(integral);
        started = false;
        lastTimeS = 0.0;
    }

    /// <summary>PID torku, N·m (±τmax'a kırpılmış).</summary>
    public double[] Compute(double t, double[] q, double[] qd, double[] qRef, double[] qdRef, double[] qddRef)
    {
        double dt = started ? t - lastTimeS : 0.0;
        started = true;
        lastTimeS = t;

        double[] tau = new double[4];
        for (int i = 0; i < 4; i++)
        {
            double e = qRef[i] - q[i];
            double eDot = qdRef[i] - qd[i];
            double candidate = integral[i] + e * dt;
            double u = Gains.Kp[i] * e + Gains.Ki[i] * candidate + Gains.Kd[i] * eDot;
            if (Math.Abs(u) <= torqueLimitNm)
                integral[i] = candidate;   // doyumda değilse integrali kabul et
            tau[i] = Math.Max(-torqueLimitNm, Math.Min(torqueLimitNm, u));
        }
        return tau;
    }
}
