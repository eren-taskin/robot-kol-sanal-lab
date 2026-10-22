using System;
using System.Collections.Generic;
using RobotArmLab.Core.Model;

namespace RobotArmLab.Core.Session;

/// <summary>Bir kontrolcünün karşılaştırma koşusunun sonucu: ölçütler ve takım hatası eğrisi.</summary>
public class ComparisonResult
{
    /// <summary>Kontrolcü türü.</summary>
    public ControllerKind Kind;

    /// <summary>Kontrolcünün arayüzdeki adı.</summary>
    public string Name;

    /// <summary>Ölçütler (ayarlar geçersizse null).</summary>
    public ControlMetrics Metrics;

    /// <summary>Takım hatası eğrisinin zamanları, s (grafiğe 5 ms'de bir nokta).</summary>
    public List<double> TimeS = new List<double>();

    /// <summary>Takım hatası eğrisi, mm.</summary>
    public List<double> ToolErrorMm = new List<double>();

    /// <summary>Durma nedeni veya ayar hataları; sorunsuz koşuda boş.</summary>
    public string Message = "";

    /// <summary>Koşu bir hatayla (eklem sınırı, zemin, NaN) durdu mu.</summary>
    public bool Faulted;
}

/// <summary>
/// Aynı deneyi (kol, ortam, yük, çözücü, görev, algılayıcı, eyleyici) seçilen her kontrolcüyle baştan sona
/// koşturur ve sonuçları yan yana verir. Her koşu oturumun ayarlarının kopyasıyla yapılır; oturum değişmez.
/// Aynı ayarlar ve aynı tohumla sonuçlar her seferinde aynıdır.
/// </summary>
public static class ControllerComparison
{
    private const double CurveIntervalS = 0.005;   // takım hatası eğrisine 5 ms'de bir nokta
    private const double TimeToleranceS = 1e-9;    // örnek zamanlarındaki kayan nokta payı, s

    /// <summary>Her kontrolcüyü sırayla koşturur.</summary>
    public static List<ComparisonResult> Run(LabSession template, List<ControllerKind> kinds)
    {
        List<ComparisonResult> results = new List<ComparisonResult>();
        for (int k = 0; k < kinds.Count; k++)
            results.Add(RunOne(template, kinds[k]));
        return results;
    }

    private static ComparisonResult RunOne(LabSession template, ControllerKind kind)
    {
        LabSession s = CopyFor(template, kind);
        ComparisonResult r = new ComparisonResult();
        r.Kind = kind;
        r.Name = ClosedLoopBuilder.DisplayNames()[(int)kind];
        List<string> errors = s.Reset();
        if (errors.Count > 0)
        {
            r.Message = string.Join(" ", errors);
            return r;
        }
        while (s.CanContinue)
            s.Advance(LabSession.MaxWallClockPerCallS);
        r.Faulted = s.IsStopped;
        r.Message = s.IsStopped ? s.StatusMessage : "";
        r.Metrics = ControlMetrics.Compute(s.History, s.Parameters, s.Parameters.TorqueLimitNm, ControlMetrics.DefaultSettlingBandRad);
        double next = 0.0;
        for (int n = 0; n < s.History.Count; n++)
        {
            if (s.History[n].TimeS < next - TimeToleranceS)
                continue;
            next = s.History[n].TimeS + CurveIntervalS;
            r.TimeS.Add(s.History[n].TimeS);
            r.ToolErrorMm.Add(ControlMetrics.ToolErrorMm(s.Parameters, s.History[n]));
        }
        return r;
    }

    // Oturumun bütün ayarlarının kopyası; kapalı çevrim açık, kontrolcü verilen tür, oynatma hızı en yüksek.
    private static LabSession CopyFor(LabSession t, ControllerKind kind)
    {
        LabSession s = new LabSession();
        s.Parameters = t.Parameters.Clone();
        s.Environment = new GravityEnvironment(t.Environment.Name, t.Environment.GravityMps2);
        s.Payload = new Payload(t.Payload.MassKg);
        s.Solver = t.Solver;
        s.StepSizeS = t.StepSizeS;
        s.RelativeTolerance = t.RelativeTolerance;
        s.AbsoluteTolerance = t.AbsoluteTolerance;
        s.InitialQRad = (double[])t.InitialQRad.Clone();
        s.InitialQdRadPerS = (double[])t.InitialQdRadPerS.Clone();
        s.DurationS = t.DurationS;
        s.SpeedFactor = SettingsValidator.MaxSpeedFactor;
        s.ClosedLoop = true;
        s.Closed = t.Closed.Clone();
        s.Closed.Controller = kind;
        return s;
    }
}
