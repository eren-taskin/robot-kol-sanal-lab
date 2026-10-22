using System.ComponentModel;

namespace RobotArmLab.Core.Controllers;

/// <summary>
/// Hesaplanmış tork kazançları, eklem başına. Kapalı çevrim hata denklemi ë + Kd ė + Kp e = 0 olur, bu yüzden
/// Kp 1/s², Kd 1/s birimindedir (kütle matrisi zaten M̂ ile çarpılır).
/// </summary>
public class ComputedTorqueGains
{
    /// <summary>Konum kazancı Kp, 1/s².</summary>
    [Category("Hesaplanmış tork"), DisplayName("Kp (1/s²)"), Description("Konum hatası kazancı; ωn = √Kp doğal frekanstır.")]
    public double[] Kp { get; set; }

    /// <summary>Hız kazancı Kd, 1/s.</summary>
    [Category("Hesaplanmış tork"), DisplayName("Kd (1/s)"), Description("Hız hatası kazancı; sönüm oranı ζ = Kd / (2√Kp).")]
    public double[] Kd { get; set; }

    /// <summary>Modelin tutma aralığının ortasındaki tahmini durumda hesaplanıp hesaplanmayacağı.</summary>
    [Category("Örnekleme"), DisplayName("Yarım örnek öngörüsü"),
     Description("Açıksa model, tork tutulurken kolun Ts/2 sonra varacağı tahmini durumda hesaplanır (eski çekirdek D7). Hafif bilek ekleminde bayat sürtünme ve yerçekimi telafisinin yarattığı hatayı azaltır.")]
    public bool PredictHalfSample { get; set; }

    /// <summary>
    /// Eski çekirdekte (A1c) doğrulanmış kazançlar: Kp = [1600, 1600, 1800, 1800] 1/s², Kd = 350 1/s; bütün
    /// başlangıç pozları, gezegenler ve 0/0,5 kg yük için en az 6 dB kazanç ve 40° faz payı. Öngörü açık.
    /// </summary>
    public static ComputedTorqueGains Default()
    {
        ComputedTorqueGains g = new ComputedTorqueGains();
        g.Kp = new double[] { 1600.0, 1600.0, 1800.0, 1800.0 };
        g.Kd = new double[] { 350.0, 350.0, 350.0, 350.0 };
        g.PredictHalfSample = true;
        return g;
    }

    /// <summary>Derin kopya.</summary>
    public ComputedTorqueGains Clone()
    {
        ComputedTorqueGains c = new ComputedTorqueGains();
        c.Kp = (double[])Kp.Clone();
        c.Kd = (double[])Kd.Clone();
        c.PredictHalfSample = PredictHalfSample;
        return c;
    }
}
