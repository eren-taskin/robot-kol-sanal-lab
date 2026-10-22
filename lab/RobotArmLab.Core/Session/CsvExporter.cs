using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Core.Session;

/// <summary>
/// Simülasyon örneklerini CSV (virgülle ayrılmış değerler) olarak yazar; Excel, MATLAB veya Python ile açılabilir.
/// Sayılar her zaman InvariantCulture ile yazılır: ondalık ayırıcı noktadır. Türkçe Windows'ta ondalık ayırıcı
/// virgül olduğundan, kültür belirtilmeseydi "0,5" yazılır ve sütunlar birbirine karışırdı.
/// </summary>
public static class CsvExporter
{
    /// <summary>Başlık satırı: her sütunun adı ve birimi.</summary>
    public const string Header =
        "t_s,q1_rad,q2_rad,q3_rad,q4_rad,qd1_radps,qd2_radps,qd3_radps,qd4_radps," +
        "tau1_Nm,tau2_Nm,tau3_Nm,tau4_Nm,x_m,y_m,z_m,T_J,V_J,E_J";

    /// <summary>Başlığı ve her örnek için bir satırı yazar.</summary>
    public static void Write(List<SimulationSample> samples, TextWriter writer)
    {
        writer.WriteLine(Header);
        for (int n = 0; n < samples.Count; n++)
            writer.WriteLine(Line(samples[n]));
    }

    /// <summary>Bir örneğin CSV satırı (başlıktaki sütun sırasıyla).</summary>
    public static string Line(SimulationSample s)
    {
        StringBuilder sb = new StringBuilder();
        sb.Append(Number(s.TimeS));
        AppendAll(sb, s.Q);
        AppendAll(sb, s.Qd);
        AppendAll(sb, s.Tau);
        sb.Append(',').Append(Number(s.ToolX));
        sb.Append(',').Append(Number(s.ToolY));
        sb.Append(',').Append(Number(s.ToolZ));
        sb.Append(',').Append(Number(s.KineticEnergyJ));
        sb.Append(',').Append(Number(s.PotentialEnergyJ));
        sb.Append(',').Append(Number(s.TotalEnergyJ));
        return sb.ToString();
    }

    private static void AppendAll(StringBuilder sb, double[] values)
    {
        for (int i = 0; i < values.Length; i++)
            sb.Append(',').Append(Number(values[i]));
    }

    // 10 anlamlı basamak; "G" biçimi gerekirse 1E-05 gibi bilimsel gösterim kullanır.
    private static string Number(double v)
    {
        return v.ToString("G10", CultureInfo.InvariantCulture);
    }
}
