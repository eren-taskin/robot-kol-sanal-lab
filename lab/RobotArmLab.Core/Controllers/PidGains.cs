using System.ComponentModel;

namespace RobotArmLab.Core.Controllers;

/// <summary>
/// PID kazançları, eklem başına (4 eleman). Varsayılanlar eski çekirdekte (T07) SPEC §7 ölçütünü sağlayacak
/// biçimde bulunmuştur. Form bu nesneyi PropertyGrid ile düzenler.
/// </summary>
public class PidGains
{
    /// <summary>Oransal kazanç Kp, N·m/rad.</summary>
    [Category("PID"), DisplayName("Kp (N·m/rad)"), Description("Oransal kazanç: hata 1 rad iken üretilen tork.")]
    public double[] Kp { get; set; }

    /// <summary>İntegral kazancı Ki, N·m/(rad·s).</summary>
    [Category("PID"), DisplayName("Ki (N·m/(rad·s))"), Description("İntegral kazancı: yerçekimi modeli olmadığından kalıcı hatayı integral giderir.")]
    public double[] Ki { get; set; }

    /// <summary>Türev kazancı Kd, N·m·s/rad.</summary>
    [Category("PID"), DisplayName("Kd (N·m·s/rad)"), Description("Türev kazancı: hız hatasına karşı sönüm.")]
    public double[] Kd { get; set; }

    /// <summary>Eski çekirdeğin doğrulanmış kazançları: Kp = [6, 6, 6, 2], Ki = [5, 20, 20, 8], Kd = [0,3; 0,6; 0,3; 0,05].</summary>
    public static PidGains Default()
    {
        PidGains g = new PidGains();
        g.Kp = new double[] { 6.0, 6.0, 6.0, 2.0 };
        g.Ki = new double[] { 5.0, 20.0, 20.0, 8.0 };
        g.Kd = new double[] { 0.3, 0.6, 0.3, 0.05 };
        return g;
    }

    /// <summary>Derin kopya (diziler de kopyalanır).</summary>
    public PidGains Clone()
    {
        PidGains c = new PidGains();
        c.Kp = (double[])Kp.Clone();
        c.Ki = (double[])Ki.Clone();
        c.Kd = (double[])Kd.Clone();
        return c;
    }
}
