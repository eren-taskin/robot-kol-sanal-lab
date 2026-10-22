using System;
using System.Collections.Generic;
using System.Globalization;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Core.Session;

/// <summary>
/// Oturum ayarlarını simülasyon kurulmadan önce denetler ve her sorunu öğrencinin anlayacağı bir Türkçe
/// cümleyle bildirir. Hiçbir zaman istisna atmaz: geçersiz ayar programı çökertmemeli, düzeltilmelidir.
/// </summary>
public static class SettingsValidator
{
    /// <summary>Oynatma hızının üst sınırı (kat). Karşılaştırma koşuları bu en yüksek hızda yapılır.</summary>
    public const double MaxSpeedFactor = 100.0;

    // Ayar sınırları. Fiziksel bir zorunluluk değil; yazım hatasını (ör. 1000 yerine 10000) yakalamak içindir.
    private const double MaxGravityMps2 = 100.0;   // Jüpiter (24,79) bile rahatça içinde kalır
    private const double MaxStepS = 0.05;           // 50 ms; daha büyük adımda hiçbir çözücü kolu doğru izleyemez
    private const double MaxDurationS = 600.0;      // 10 dakika

    private static readonly CultureInfo Turkish = CultureInfo.GetCultureInfo("tr-TR");

    /// <summary>Bütün denetimleri yapar; boş liste ayarların geçerli olduğu anlamına gelir.</summary>
    public static List<string> Validate(LabSession s)
    {
        List<string> errors = new List<string>();
        if (s.Parameters == null || s.Environment == null || s.Payload == null || s.Inputs == null
            || s.InitialQRad == null || s.InitialQdRadPerS == null)
        {
            errors.Add("Ayarlar eksik: parametre, ortam, yük, giriş veya başlangıç durumu tanımlı değil.");
            return errors;
        }
        CheckParameters(s.Parameters, errors);
        CheckEnvironmentAndPayload(s, errors);
        CheckSolverAndRun(s, errors);
        CheckInputs(s.Inputs, errors);
        if (errors.Count == 0)
            CheckInitialState(s, errors);   // poz denetimi geçerli parametre ister
        if (errors.Count == 0 && s.ClosedLoop)
            ClosedLoopValidator.Check(s, errors);   // görev denetimi geçerli başlangıç pozu ister
        return errors;
    }

    // Kütle, boy, eylemsizlik, motor ve sürtünme değerleri fiziksel olarak anlamlı mı.
    private static void CheckParameters(ArmParameters p, List<string> errors)
    {
        if (!HasFour(p.MassKg) || !HasFour(p.ComDistanceM) || !HasFour(p.ComAngleRad) || !HasFour(p.IxxKgM2)
            || !HasFour(p.IyyKgM2) || !HasFour(p.IzzKgM2) || !HasFour(p.IxzKgM2)
            || !HasFour(p.LowerLimitRad) || !HasFour(p.UpperLimitRad))
        {
            errors.Add("Uzuv ve eklem dizilerinin her biri tam 4 sayı içermelidir.");
            return;
        }
        for (int i = 0; i < 4; i++)
        {
            string link = "Uzuv " + (i + 1);
            if (!IsPositive(p.MassKg[i]))
                errors.Add(link + " kütlesi sıfırdan büyük olmalıdır (şu an " + Fmt(p.MassKg[i]) + " kg).");
            if (Bad(p.ComDistanceM[i]) || p.ComDistanceM[i] < 0.0)
                errors.Add(link + " kütle merkezi uzaklığı negatif olamaz.");
            if (Bad(p.IxxKgM2[i]) || Bad(p.IyyKgM2[i]) || Bad(p.IzzKgM2[i]) || Bad(p.IxzKgM2[i])
                || p.IxxKgM2[i] < 0.0 || p.IyyKgM2[i] < 0.0 || p.IzzKgM2[i] < 0.0)
                errors.Add(link + " eylemsizlikleri (Ixx, Iyy, Izz) negatif olamaz.");
            if (Bad(p.LowerLimitRad[i]) || Bad(p.UpperLimitRad[i]) || p.LowerLimitRad[i] >= p.UpperLimitRad[i])
                errors.Add("Eklem " + (i + 1) + " alt sınırı üst sınırından küçük olmalıdır.");
        }
        if (Bad(p.D1M) || p.D1M < 0.0 || !IsPositive(p.L3M) || !IsPositive(p.L4M) || Bad(p.A2xM) || Bad(p.A2zM) || p.L2M <= 0.0)
            errors.Add("Uzuv boyları (d1, a2x, a2z, L3, L4) geçersiz: boylar pozitif, d1 negatif olmayan bir sayı olmalıdır.");
        CheckMotorAndFriction(p, errors);
    }

    // Dişli, sürtünme ve tork sınırı.
    private static void CheckMotorAndFriction(ArmParameters p, List<string> errors)
    {
        if (!IsPositive(p.GearRatio))
            errors.Add("Dişli oranı N sıfırdan büyük olmalıdır.");
        if (Bad(p.RotorInertiaKgM2) || p.RotorInertiaKgM2 < 0.0)
            errors.Add("Rotor eylemsizliği Jm negatif olamaz.");
        if (Bad(p.ViscousFrictionNmsPerRad) || p.ViscousFrictionNmsPerRad < 0.0
            || Bad(p.CoulombFrictionNm) || p.CoulombFrictionNm < 0.0)
            errors.Add("Sürtünme katsayıları (fv, fc) negatif olamaz.");
        if (!IsPositive(p.FrictionSmoothingRadPerS))
            errors.Add("Sürtünme yumuşatma hızı ωs sıfırdan büyük olmalıdır.");
        if (!IsPositive(p.TorqueLimitNm))
            errors.Add("Tork sınırı τmax sıfırdan büyük olmalıdır.");
    }

    // Yerçekimi ve yük.
    private static void CheckEnvironmentAndPayload(LabSession s, List<string> errors)
    {
        double g = s.Environment.GravityMps2;
        if (Bad(g) || g < 0.0 || g > MaxGravityMps2)
            errors.Add("Yerçekimi ivmesi g 0 ile 100 m/s² arasında olmalıdır (şu an " + Fmt(g) + ").");
        double mp = s.Payload.MassKg;
        if (Bad(mp) || mp < 0.0)
            errors.Add("Yük kütlesi negatif olamaz.");
        else if (mp > Payload.MaxMassKg)
            errors.Add("Yük kütlesi en çok 0,5 kg olabilir (üreticinin taşıma kapasitesi).");
    }

    // Çözücü, süre ve oynatma hızı.
    private static void CheckSolverAndRun(LabSession s, List<string> errors)
    {
        if (SolverFactory.IsFixedStep(s.Solver))
        {
            if (!IsPositive(s.StepSizeS))
                errors.Add("Adım boyu h sıfırdan büyük olmalıdır (şu an " + Fmt(s.StepSizeS * 1000.0) + " ms).");
            else if (s.StepSizeS > MaxStepS)
                errors.Add("Adım boyu h en çok 50 ms olabilir.");
        }
        else if (!IsPositive(s.RelativeTolerance) || !IsPositive(s.AbsoluteTolerance))
            errors.Add("Dormand–Prince toleransları rtol ve atol sıfırdan büyük olmalıdır.");

        if (!IsPositive(s.DurationS) || s.DurationS > MaxDurationS)
            errors.Add("Simülasyon süresi 0 ile 600 s arasında olmalıdır.");
        if (!IsPositive(s.SpeedFactor) || s.SpeedFactor > MaxSpeedFactor)
            errors.Add("Oynatma hızı 0 ile 100 kat arasında olmalıdır.");
    }

    // Her eklemin giriş sinyali.
    private static void CheckInputs(JointInput[] inputs, List<string> errors)
    {
        if (inputs.Length != 4)
        {
            errors.Add("Her eklem için bir giriş olmalıdır (4 satır).");
            return;
        }
        for (int i = 0; i < 4; i++)
        {
            JointInput u = inputs[i];
            if (u == null || Bad(u.Amplitude) || Bad(u.StepTimeS) || Bad(u.FrequencyHz))
                errors.Add("Eklem " + (i + 1) + " girişinde geçersiz sayı var.");
            else if (u.StepTimeS < 0.0 || u.FrequencyHz < 0.0)
                errors.Add("Eklem " + (i + 1) + " girişinde t0 ve frekans negatif olamaz.");
        }
    }

    // Başlangıç pozu eklem sınırlarının içinde ve takım zeminin üstünde mi.
    private static void CheckInitialState(LabSession s, List<string> errors)
    {
        ArmParameters p = s.Parameters;
        if (s.InitialQRad.Length != 4 || s.InitialQdRadPerS.Length != 4)
        {
            errors.Add("Başlangıç açısı ve hızı 4'er eleman olmalıdır.");
            return;
        }
        for (int i = 0; i < 4; i++)
        {
            double q = s.InitialQRad[i];
            if (Bad(q) || Bad(s.InitialQdRadPerS[i]))
                errors.Add("Eklem " + (i + 1) + " başlangıç açısı veya hızı geçersiz.");
            else if (q < p.LowerLimitRad[i] || q > p.UpperLimitRad[i])
                errors.Add("Eklem " + (i + 1) + " başlangıç açısı " + Fmt(Deg(q)) + "° eklem sınırlarının dışında ["
                    + Fmt(Deg(p.LowerLimitRad[i])) + "°, " + Fmt(Deg(p.UpperLimitRad[i])) + "°].");
        }
        if (errors.Count == 0 && ForwardKinematics.ToolPosition(p, s.InitialQRad).Z < 0.0)
            errors.Add("Başlangıç pozunda takım noktası zeminin altında; açıları değiştirin.");
    }

    private static bool HasFour(double[] a) { return a != null && a.Length == 4; }

    private static bool Bad(double v) { return double.IsNaN(v) || double.IsInfinity(v); }

    private static bool IsPositive(double v) { return !Bad(v) && v > 0.0; }

    private static double Deg(double rad) { return rad * 180.0 / Math.PI; }

    private static string Fmt(double v) { return v.ToString("0.###", Turkish); }
}
