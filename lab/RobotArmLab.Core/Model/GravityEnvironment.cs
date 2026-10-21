using System.Collections.Generic;

namespace RobotArmLab.Core.Model;

/// <summary>
/// Deneyin yapıldığı ortam: bir ad ve yerçekimi ivmesi g. Yerçekimi taban çerçevesinin −z yönünde etki eder
/// (SPEC §3.5). Sınıfın adı "Environment" değildir, çünkü System.Environment ile çakışırdı.
/// </summary>
public class GravityEnvironment
{
    /// <summary>Arayüzde gösterilen ad (Türkçe).</summary>
    public string Name { get; set; }

    /// <summary>Yerçekimi ivmesinin büyüklüğü g, m/s². −z yönünde etki eder.</summary>
    public double GravityMps2 { get; set; }

    /// <summary>Verilen ad ve g (m/s²) ile bir ortam oluşturur.</summary>
    public GravityEnvironment(string name, double gravityMps2)
    {
        Name = name;
        GravityMps2 = gravityMps2;
    }

    /// <summary>Dünya, g = 9,81 m/s².</summary>
    public static GravityEnvironment Earth() { return new GravityEnvironment("Dünya", 9.81); }

    /// <summary>Ay, g = 1,62 m/s².</summary>
    public static GravityEnvironment Moon() { return new GravityEnvironment("Ay", 1.62); }

    /// <summary>Mars, g = 3,71 m/s².</summary>
    public static GravityEnvironment Mars() { return new GravityEnvironment("Mars", 3.71); }

    /// <summary>Jüpiter düzeyinde yüksek yerçekimi, g = 24,79 m/s².</summary>
    public static GravityEnvironment HighG() { return new GravityEnvironment("Yüksek g (Jüpiter düzeyi)", 24.79); }

    /// <summary>Yörünge (mikro yerçekimi), g = 0. Kol kendi ağırlığını hissetmez.</summary>
    public static GravityEnvironment Orbit() { return new GravityEnvironment("Yörünge (mikro yerçekimi)", 0.0); }

    /// <summary>Beş hazır ortamı SPEC §3.5 sırasıyla verir (form açılır listesi için).</summary>
    public static List<GravityEnvironment> Presets()
    {
        List<GravityEnvironment> list = new List<GravityEnvironment>();
        list.Add(Earth());
        list.Add(Moon());
        list.Add(Mars());
        list.Add(HighG());
        list.Add(Orbit());
        return list;
    }

    /// <summary>Açılır listede görünen metin: ad ve g değeri.</summary>
    public override string ToString()
    {
        return Name + " (g = " + GravityMps2.ToString("0.00", System.Globalization.CultureInfo.InvariantCulture) + " m/s²)";
    }
}
