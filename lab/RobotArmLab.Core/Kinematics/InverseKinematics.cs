using System;
using RobotArmLab.Core.Model;

namespace RobotArmLab.Core.Kinematics;

/// <summary>Ters kinematiğin neden başarısız olduğunu söyler.</summary>
public enum IkFailure
{
    /// <summary>Başarılı, çözüm bulundu.</summary>
    None,

    /// <summary>Hedef, kolun erişebileceği bölgenin dışında (|cos φ| &gt; 1).</summary>
    Unreachable,

    /// <summary>Çözüm var ama bir eklem açısı sınırların dışında.</summary>
    JointLimit,
}

/// <summary>Ters kinematik sonucu: başarı bayrağı, neden ve (varsa) eklem açıları.</summary>
public class IkResult
{
    /// <summary>Çözüm kullanılabilir mi.</summary>
    public bool Success;

    /// <summary>Başarısızlık nedeni (başarılıysa None).</summary>
    public IkFailure Failure;

    /// <summary>Kullanıcıya gösterilecek Türkçe açıklama.</summary>
    public string Reason;

    /// <summary>Eklem açıları q1..q4, rad. Erişilemeyen hedefte null.</summary>
    public double[] Q;
}

/// <summary>
/// Ters kinematik: takım noktasının hedef konumu (x, y, z, m) ve eğimi ψ (rad) verilince eklem açılarını bulur.
/// Rapor eş. (5)–(7), SPEC §4.3. Taban açısı doğrudan bulunur, kalan üç eklem düzlemsel iki uzuvlu kol gibi çözülür.
/// </summary>
public static class InverseKinematics
{
    // cos φ, yuvarlama hatası yüzünden erişim sınırındaki bir hedefte 1'i bu kadar aşabilir; bu pay erişilebilir sayılır.
    private const double CosRoundingTolerance = 1e-14;

    /// <summary>
    /// Hedefe giden eklem açılarını hesaplar. elbowUp: dirsek yukarı (true) veya aşağı (false) dalı.
    /// checkLimits: true ise sınır dışı çözüm JointLimit hatasıdır. Hiçbir zaman NaN veya istisna döndürmez.
    /// </summary>
    public static IkResult Solve(ArmParameters p, double x, double y, double z, double pitchRad, bool elbowUp, bool checkLimits)
    {
        double L2 = p.L2M, L3 = p.L3M, beta = p.BetaRad;

        // Eş. (5): taban açısı ve bilek noktasının düzlemdeki konumu (u ileri, v yukarı), m.
        double q1 = Math.Atan2(y, x);
        double u = Math.Sqrt(x * x + y * y) - p.L4M * Math.Cos(pitchRad);
        double v = z - p.D1M + p.L4M * Math.Sin(pitchRad);

        // Eş. (6): kosinüs teoremi ile dirsek açısı φ. |cos φ| > 1 ise üçgen kurulamaz: hedef erişilemez.
        double cosPhi = (u * u + v * v - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
        if (double.IsNaN(cosPhi) || Math.Abs(cosPhi) > 1.0 + CosRoundingTolerance)
            return Fail(IkFailure.Unreachable, "Hedef erişim alanının dışında.", null);
        cosPhi = Math.Max(-1.0, Math.Min(1.0, cosPhi));   // yalnız yuvarlama hatasını kırpar
        double phi = elbowUp ? -Math.Acos(cosPhi) : Math.Acos(cosPhi);

        // Eş. (7): omuz, dirsek ve bilek açıları. q4 takım eğimini tamamlar.
        double e = Math.Atan2(v, u) - Math.Atan2(L3 * Math.Sin(phi), L2 + L3 * Math.Cos(phi));
        double q2 = beta - e;
        double q3 = -beta - phi;
        double q4 = pitchRad - q2 - q3;
        double[] q = new double[] { q1, q2, q3, q4 };

        if (checkLimits)
        {
            for (int i = 0; i < 4; i++)
            {
                if (q[i] < p.LowerLimitRad[i] || q[i] > p.UpperLimitRad[i])
                    return Fail(IkFailure.JointLimit, "Eklem " + (i + 1) + " sınırların dışında kalıyor.", q);
            }
        }

        IkResult ok = new IkResult();
        ok.Success = true;
        ok.Failure = IkFailure.None;
        ok.Reason = "Çözüm bulundu.";
        ok.Q = q;
        return ok;
    }

    private static IkResult Fail(IkFailure failure, string reason, double[] q)
    {
        IkResult r = new IkResult();
        r.Success = false;
        r.Failure = failure;
        r.Reason = reason;
        r.Q = q;
        return r;
    }
}
