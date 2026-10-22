using System;
using System.Collections.Generic;
using System.Globalization;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Core.Session;

/// <summary>
/// Kapalı çevrim ayarlarını denetler (SettingsValidator'ın devamı) ve her sorunu Türkçe bir cümleyle bildirir.
/// Parametre, ortam ve başlangıç durumu geçerliyken çağrılır.
/// </summary>
public static class ClosedLoopValidator
{
    /// <summary>Motor eyleyicisinde sabit adımlı çözücülerin en büyük adım boyu, s (zıt EMK sistemi sertleştirir).</summary>
    public const double MaxMotorStepS = 0.0002;

    private static readonly CultureInfo Turkish = CultureInfo.GetCultureInfo("tr-TR");

    /// <summary>Bütün kapalı çevrim denetimlerini yapar ve hataları listeye ekler.</summary>
    public static void Check(LabSession s, List<string> errors)
    {
        ClosedLoopSettings c = s.Closed;
        if (c == null)
        {
            errors.Add("Kapalı çevrim ayarları tanımlı değil.");
            return;
        }
        CheckSampling(s, c, errors);
        CheckModelAndSensor(c, errors);
        CheckGains(c, errors);
        if (errors.Count == 0)
            CheckTask(s, c, errors);
    }

    // Ts > 0, Ts ≤ 50 ms, sabit adımlıda Ts = n·h; motor eyleyicisinde h ≤ 0,2 ms.
    private static void CheckSampling(LabSession s, ClosedLoopSettings c, List<string> errors)
    {
        double ts = c.SamplePeriodS;
        if (Bad(ts) || ts <= 0.0 || ts > 0.05)
        {
            errors.Add("Örnekleme süresi Ts 0 ile 50 ms arasında olmalıdır (şu an " + Fmt(ts * 1000.0) + " ms).");
            return;
        }
        if (!SolverFactory.IsFixedStep(s.Solver) || !(s.StepSizeS > 0.0))
            return;
        double ratio = ts / s.StepSizeS;
        if (ratio < 1.0 - 1e-9 || Math.Abs(ratio - Math.Round(ratio)) > 1e-6)
            errors.Add("Ts, çözücü adımı h'nin tam katı olmalıdır (Ts = " + Fmt(ts * 1000.0) + " ms, h = "
                + Fmt(s.StepSizeS * 1000.0) + " ms).");
        if (c.Actuator == ActuatorKind.Motor && s.StepSizeS > MaxMotorStepS + 1e-12)
            errors.Add("Motor eyleyicisinde h en çok 0,2 ms olabilir: zıt EMK hafif bileği çok sertleştirir, "
                + "daha büyük adımda çözücü kararlılık sınırına takılır.");
    }

    // Tahmini yük, kodlayıcı ve gürültü.
    private static void CheckModelAndSensor(ClosedLoopSettings c, List<string> errors)
    {
        if (Bad(c.EstimatedPayloadKg) || c.EstimatedPayloadKg < 0.0 || c.EstimatedPayloadKg > 0.5)
            errors.Add("Tahmini yük 0 ile 0,5 kg arasında olmalıdır.");
        if (c.EncoderCounts < 16)
            errors.Add("Kodlayıcı sayımı en az 16 olmalıdır (XM430-W350: 4096).");
        if (Bad(c.NoiseStdDevRad) || c.NoiseStdDevRad < 0.0)
            errors.Add("Gürültü standart sapması negatif olamaz.");
    }

    // Yalnız seçilen kontrolcünün kazançları denetlenir: 4'er sonlu, negatif olmayan sayı.
    private static void CheckGains(ClosedLoopSettings c, List<string> errors)
    {
        bool ok;
        switch (c.Controller)
        {
            case ControllerKind.Pid:
                ok = c.Pid != null && Gains(c.Pid.Kp) && Gains(c.Pid.Ki) && Gains(c.Pid.Kd);
                break;
            case ControllerKind.ComputedTorque:
                ok = c.ComputedTorque != null && Gains(c.ComputedTorque.Kp) && Gains(c.ComputedTorque.Kd);
                break;
            case ControllerKind.SuperTwisting:
                ok = c.SuperTwisting != null && Gains(c.SuperTwisting.Lambda) && Gains(c.SuperTwisting.K1) && Gains(c.SuperTwisting.K2);
                break;
            default:
                ok = c.FeedForward != null && Gains(c.FeedForward.Lambda) && Gains(c.FeedForward.K1) && Gains(c.FeedForward.K2);
                break;
        }
        if (!ok)
            errors.Add("Kontrolcü kazançlarının her biri tam 4 sonlu ve negatif olmayan sayı olmalıdır.");
    }

    // Görev sayıları, hedef açıların sınırları ve görevin kurulabilmesi (ters kinematik).
    private static void CheckTask(LabSession s, ClosedLoopSettings c, List<string> errors)
    {
        if (c.PickAndPlace && (Bad(c.PickTimeS) || c.PickTimeS < 0.0))
            errors.Add("Al-bırak anı negatif olmayan bir sayı olmalıdır.");
        if (Bad(c.StartTimeS) || c.StartTimeS < 0.0)
            errors.Add("Görevin başlama anı t0 negatif olamaz.");
        if (c.Task == TaskKind.JointQuintic || c.Task == TaskKind.Cartesian)
        {
            if (Bad(c.MotionDurationS) || c.MotionDurationS <= 0.0)
                errors.Add("Hareket süresi T sıfırdan büyük olmalıdır.");
        }
        if (c.Task == TaskKind.JointStep || c.Task == TaskKind.JointQuintic)
            CheckTarget(s, c.TargetQRad, errors);
        if (c.Task == TaskKind.Circle && (Bad(c.CircleRadiusM) || c.CircleRadiusM <= 0.0 || Bad(c.CirclePeriodS)
            || c.CirclePeriodS <= 0.0 || Bad(c.CircleApproachS) || c.CircleApproachS <= 0.0 || Bad(c.CircleRampS) || c.CircleRampS < 0.0))
            errors.Add("Çemberin yarıçapı, tur süresi ve yaklaşma süresi sıfırdan büyük, yumuşak başlama süresi negatif olmayan sayı olmalıdır.");
        if (errors.Count > 0)
            return;
        string error;
        if (ClosedLoopBuilder.CreateTask(c, s.Parameters, s.InitialQRad, out error) == null)
            errors.Add(error);
    }

    private static void CheckTarget(LabSession s, double[] target, List<string> errors)
    {
        if (target == null || target.Length != 4)
        {
            errors.Add("Hedef eklem açıları 4 eleman olmalıdır.");
            return;
        }
        for (int i = 0; i < 4; i++)
        {
            if (Bad(target[i]) || target[i] < s.Parameters.LowerLimitRad[i] || target[i] > s.Parameters.UpperLimitRad[i])
                errors.Add("Eklem " + (i + 1) + " hedef açısı eklem sınırlarının dışında veya geçersiz.");
        }
    }

    private static bool Gains(double[] g)
    {
        if (g == null || g.Length != 4)
            return false;
        for (int i = 0; i < 4; i++)
        {
            if (Bad(g[i]) || g[i] < 0.0)
                return false;
        }
        return true;
    }

    private static bool Bad(double v) { return double.IsNaN(v) || double.IsInfinity(v); }

    private static string Fmt(double v) { return v.ToString("0.###", Turkish); }
}
