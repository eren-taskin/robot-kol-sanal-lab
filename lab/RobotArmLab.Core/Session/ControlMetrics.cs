using System;
using System.Collections.Generic;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Core.Session;

/// <summary>
/// Kapalı çevrim koşusunun başarım ölçütleri. Eklem hatası e = q_ref − q (rad), takım hatası |p − p_ref| (mm;
/// p_ref, q_ref'in ileri kinematiğidir). Bütün değerler kaydedilmiş örneklerden (çözücü adımları) hesaplanır.
/// </summary>
public class ControlMetrics
{
    /// <summary>Oturma bandının varsayılanı, rad (≈ 0,29°).</summary>
    public const double DefaultSettlingBandRad = 0.005;

    /// <summary>Bütün eklemler ve örnekler üzerinden eklem hatasının karesel ortalamasının kökü (RMS), rad.</summary>
    public double RmsErrorRad;

    /// <summary>En büyük |e_i|, rad.</summary>
    public double MaxErrorRad;

    /// <summary>Son örnekte en büyük |e_i|, rad.</summary>
    public double FinalErrorRad;

    /// <summary>En büyük takım hatası, mm.</summary>
    public double MaxToolErrorMm;

    /// <summary>Son örnekteki takım hatası, mm.</summary>
    public double FinalToolErrorMm;

    /// <summary>Oturma süresi: en büyük |e_i|'nin banttan son kez taştığı an, s. Hiç taşmadıysa 0; sonda hâlâ dışarıdaysa NaN (oturmadı).</summary>
    public double SettlingTimeS;

    /// <summary>Aşma: her eklemde (q − q_son)·işaret(Δ)/|Δ|'nin en büyüğü, %, Δ = q_son − q_başlangıç (|Δ| &gt; 1 mrad olan eklemler).</summary>
    public double OvershootPercent;

    /// <summary>Uygulanan en büyük |τ_i|, N·m.</summary>
    public double MaxTorqueNm;

    /// <summary>Herhangi bir eklemin tork komutunun ±τmax'ta (doyumda) kaldığı toplam süre, s.</summary>
    public double SaturationTimeS;

    /// <summary>Kontrol eforu ∫ Σ τ_i² dt, N²·m²·s (uygulanan tork).</summary>
    public double TorqueSquaredIntegral;

    /// <summary>Titreşim (chattering) göstergesi: tork komutunun toplam değişimi Σ_k Σ_i |Δτ_cmd,i| bölü süre, N·m/s.</summary>
    public double ChatteringNmPerS;

    /// <summary>
    /// Ölçütleri hesaplar. samples kapalı çevrim örnekleri olmalıdır (QRef ve TauCommand dolu); ilk örnek t = 0.
    /// torqueLimitNm doyum süresi için, settlingBandRad oturma bandı için kullanılır.
    /// </summary>
    public static ControlMetrics Compute(List<SimulationSample> samples, ArmParameters parameters, double torqueLimitNm, double settlingBandRad)
    {
        ControlMetrics m = new ControlMetrics();
        if (samples.Count == 0)
            return m;
        ErrorMeasures(m, samples, parameters, settlingBandRad);
        TorqueMeasures(m, samples, torqueLimitNm);
        m.OvershootPercent = Overshoot(samples);
        return m;
    }

    /// <summary>Bir örneğin takım hatası |p − FK(q_ref)|, mm.</summary>
    public static double ToolErrorMm(ArmParameters parameters, SimulationSample s)
    {
        Point3 r = ForwardKinematics.ToolPosition(parameters, s.QRef);
        double dx = s.ToolX - r.X, dy = s.ToolY - r.Y, dz = s.ToolZ - r.Z;
        return 1000.0 * Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    // RMS, en büyük ve son hata, takım hatası, oturma süresi.
    private static void ErrorMeasures(ControlMetrics m, List<SimulationSample> samples, ArmParameters p, double band)
    {
        double sumSquares = 0.0, lastOutsideS = 0.0;
        bool outsideAtEnd = false;
        for (int n = 0; n < samples.Count; n++)
        {
            SimulationSample s = samples[n];
            double worst = 0.0;
            for (int i = 0; i < 4; i++)
            {
                double e = s.QRef[i] - s.Q[i];
                sumSquares += e * e;
                worst = Math.Max(worst, Math.Abs(e));
            }
            m.MaxErrorRad = Math.Max(m.MaxErrorRad, worst);
            m.MaxToolErrorMm = Math.Max(m.MaxToolErrorMm, ToolErrorMm(p, s));
            outsideAtEnd = worst > band;
            if (outsideAtEnd)
                lastOutsideS = s.TimeS;
            m.FinalErrorRad = worst;
        }
        m.RmsErrorRad = Math.Sqrt(sumSquares / (4.0 * samples.Count));
        m.FinalToolErrorMm = ToolErrorMm(p, samples[samples.Count - 1]);
        m.SettlingTimeS = outsideAtEnd ? double.NaN : lastOutsideS;
    }

    // En büyük tork, doyum süresi, ∫τ² dt ve titreşim göstergesi (dikdörtgen kuralı, örnek aralığı dt ile).
    private static void TorqueMeasures(ControlMetrics m, List<SimulationSample> samples, double limit)
    {
        double variation = 0.0;
        for (int n = 0; n < samples.Count; n++)
        {
            SimulationSample s = samples[n];
            double dt = n > 0 ? s.TimeS - samples[n - 1].TimeS : 0.0;
            bool saturated = false;
            for (int i = 0; i < 4; i++)
            {
                m.MaxTorqueNm = Math.Max(m.MaxTorqueNm, Math.Abs(s.Tau[i]));
                m.TorqueSquaredIntegral += s.Tau[i] * s.Tau[i] * dt;
                saturated = saturated || Math.Abs(s.TauCommand[i]) >= limit * (1.0 - 1e-9);
                if (n > 0)
                    variation += Math.Abs(s.TauCommand[i] - samples[n - 1].TauCommand[i]);
            }
            if (saturated)
                m.SaturationTimeS += dt;
        }
        double duration = samples[samples.Count - 1].TimeS - samples[0].TimeS;
        m.ChatteringNmPerS = duration > 0.0 ? variation / duration : 0.0;
    }

    // Her eklemin yüzde aşması; en büyüğü döner.
    private static double Overshoot(List<SimulationSample> samples)
    {
        SimulationSample first = samples[0], last = samples[samples.Count - 1];
        double worst = 0.0;
        for (int i = 0; i < 4; i++)
        {
            double delta = last.QRef[i] - first.Q[i];
            if (Math.Abs(delta) <= 1e-3)
                continue;
            for (int n = 0; n < samples.Count; n++)
            {
                double beyond = (samples[n].Q[i] - last.QRef[i]) * Math.Sign(delta) / Math.Abs(delta);
                worst = Math.Max(worst, 100.0 * beyond);
            }
        }
        return worst;
    }
}
