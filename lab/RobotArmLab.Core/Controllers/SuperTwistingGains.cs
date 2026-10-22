using System.ComponentModel;

namespace RobotArmLab.Core.Controllers;

/// <summary>
/// Süper-büküm kayan kip kazançları, eklem başına: kayma yüzeyi eğimi Λ (1/s), karekök kazancı k1 ve anahtarlama
/// kazancı k2. STSMC ve FF-STSMC aynı kazanç sınıfını kullanır.
/// </summary>
public class SuperTwistingGains
{
    /// <summary>Kayma yüzeyi eğimi Λ, s = ė + Λe, 1/s.</summary>
    [Category("Süper-büküm"), DisplayName("Λ (1/s)"), Description("Kayma yüzeyi s = ė + Λe. s = 0 olunca hata ė = −Λe ile sönümlenir (zaman sabiti 1/Λ).")]
    public double[] Lambda { get; set; }

    /// <summary>k1·|s|^½·sign(s) teriminin kazancı.</summary>
    [Category("Süper-büküm"), DisplayName("k1"), Description("Karekök terimi kazancı. Levant (1998) kuralı: k1 = 1,5·√L.")]
    public double[] K1 { get; set; }

    /// <summary>v̇ = k2·sign(s) integral teriminin kazancı, rad/s³.</summary>
    [Category("Süper-büküm"), DisplayName("k2 (rad/s³)"), Description("İntegral (anahtarlama) terimi kazancı. Levant (1998) kuralı: k2 = 1,1·L; L bozucunun türevinin üst sınırıdır.")]
    public double[] K2 { get; set; }

    /// <summary>Modelin tutma aralığının ortasındaki tahmini durumda hesaplanıp hesaplanmayacağı (yalnız STSMC).</summary>
    [Category("Örnekleme"), DisplayName("Yarım örnek öngörüsü"),
     Description("Yalnız STSMC: model, tork tutulurken kolun Ts/2 sonra varacağı tahmini durumda hesaplanır. FF-STSMC modeli referansta hesapladığından bu ayarı kullanmaz.")]
    public bool PredictHalfSample { get; set; }

    /// <summary>
    /// Eski çekirdekte (D13) doğrulanmış kazançlar: Λ = 20 1/s; L = [93,8; 93,8; 199,0; 1013,0] rad/s³ ("0,3 kg
    /// habersiz yük" durumunda ölçülen bozucu türevi sınırı) ile Levant 1998 kuralı k1 = 1,5√L, k2 = 1,1L:
    /// k1 = [14,53; 14,53; 21,16; 47,74], k2 = [103,2; 103,2; 218,9; 1114,3]. Öngörü açık.
    /// </summary>
    public static SuperTwistingGains Default()
    {
        SuperTwistingGains g = new SuperTwistingGains();
        g.Lambda = new double[] { 20.0, 20.0, 20.0, 20.0 };
        g.K1 = new double[] { 14.53, 14.53, 21.16, 47.74 };
        g.K2 = new double[] { 103.2, 103.2, 218.9, 1114.3 };
        g.PredictHalfSample = true;
        return g;
    }

    /// <summary>Derin kopya.</summary>
    public SuperTwistingGains Clone()
    {
        SuperTwistingGains c = new SuperTwistingGains();
        c.Lambda = (double[])Lambda.Clone();
        c.K1 = (double[])K1.Clone();
        c.K2 = (double[])K2.Clone();
        c.PredictHalfSample = PredictHalfSample;
        return c;
    }
}
