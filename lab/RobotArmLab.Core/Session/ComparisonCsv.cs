using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace RobotArmLab.Core.Session;

/// <summary>Karşılaştırma tablosunu CSV olarak yazar (InvariantCulture: nokta ondalık, virgül ayırıcı).</summary>
public static class ComparisonCsv
{
    /// <summary>Başlık satırı: sütun adları ve birimleri, tablodaki sırayla.</summary>
    public const string Header =
        "controller,rms_rad,max_rad,final_rad,max_tool_mm,final_tool_mm,settling_s,overshoot_pct," +
        "max_tau_Nm,saturation_s,tau2_integral,chattering_Nmps,message";

    /// <summary>Başlığı ve her kontrolcü için bir satırı yazar. Ölçütü olmayan (ayarı geçersiz) satırlarda sayılar boştur.</summary>
    public static void Write(List<ComparisonResult> results, TextWriter writer)
    {
        writer.WriteLine(Header);
        for (int k = 0; k < results.Count; k++)
            writer.WriteLine(Line(results[k]));
    }

    /// <summary>Bir sonucun CSV satırı.</summary>
    public static string Line(ComparisonResult r)
    {
        StringBuilder sb = new StringBuilder();
        sb.Append(Quote(r.Name));
        double[] v = Values(r.Metrics);
        for (int i = 0; i < 11; i++)
            sb.Append(',').Append(v == null ? "" : v[i].ToString("G6", CultureInfo.InvariantCulture));
        sb.Append(',').Append(Quote(r.Message));
        return sb.ToString();
    }

    /// <summary>Ölçütleri tablo sütun sırasıyla verir (metrics null ise null). Form tablosu da bunu kullanır.</summary>
    public static double[] Values(ControlMetrics m)
    {
        if (m == null)
            return null;
        return new double[] { m.RmsErrorRad, m.MaxErrorRad, m.FinalErrorRad, m.MaxToolErrorMm, m.FinalToolErrorMm,
            m.SettlingTimeS, m.OvershootPercent, m.MaxTorqueNm, m.SaturationTimeS, m.TorqueSquaredIntegral, m.ChatteringNmPerS };
    }

    // Metin hücreleri tırnak içine alınır; içteki tırnak ikilenir (CSV kuralı). Adlarda virgül olabilir.
    private static string Quote(string text)
    {
        return "\"" + (text ?? "").Replace("\"", "\"\"") + "\"";
    }
}
