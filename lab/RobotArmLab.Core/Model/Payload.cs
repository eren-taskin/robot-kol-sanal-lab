using System;

namespace RobotArmLab.Core.Model;

/// <summary>
/// Takım (uç) noktasına sıkıca bağlı noktasal yük. Kütlesi 0..0,5 kg (üreticinin taşıma kapasitesi 0,5 kg).
/// Yük dinamiğe ayrı bir cisim olarak girmez: uzuv 4 ile birleştirilir (SPEC §5.5).
/// </summary>
public class Payload
{
    /// <summary>Yükün kütlesi mp, kg. 0 ise yük yoktur.</summary>
    public double MassKg { get; set; }

    /// <summary>Verilen kütlede (kg) bir yük oluşturur.</summary>
    public Payload(double massKg)
    {
        MassKg = massKg;
    }

    /// <summary>
    /// Yükü uzuv 4'e katar ve yeni bir parametre takımı döndürür (asıl parametreler değişmez).
    /// Uzuv 4'ün kütlesi, kütle merkezi ve eylemsizliği, "uzuv 4 + uçtaki nokta kütle" bileşik cismininki olur.
    /// SPEC §5.5'teki formüller adım adım uygulanır.
    /// </summary>
    public ArmParameters ApplyTo(ArmParameters arm)
    {
        ArmParameters p = arm.Clone();
        if (MassKg <= 0.0)
            return p;

        // 1) Uzuv 4'ün kütle merkezi c4 ve takım noktası t, uzuv 4 çerçevesinde (x ileri, z yukarı), m.
        double m4 = p.MassKg[3];
        double c4x = p.ComDistanceM[3] * Math.Cos(p.ComAngleRad[3]);
        double c4z = -p.ComDistanceM[3] * Math.Sin(p.ComAngleRad[3]);
        double tx = p.L4M;
        double tz = 0.0;

        // 2) Bileşik kütle m4' = m4 + mp (kg) ve bileşik kütle merkezi c' = (m4·c4 + mp·t) / m4'.
        double mp = MassKg;
        double mNew = m4 + mp;
        double cx = (m4 * c4x + mp * tx) / mNew;
        double cz = (m4 * c4z + mp * tz) / mNew;

        // 3) Paralel eksen (Steiner) teoremi: her kütle, yeni merkeze uzaklığı d kadar eylemsizlik ekler,
        //    I += m (|d|² 1 − d dᵀ). d'nin y bileşeni sıfır olduğundan yalnız dört terim değişir.
        double d4x = c4x - cx, d4z = c4z - cz;   // uzuv 4 merkezinden yeni merkeze, m
        double dpx = tx - cx, dpz = tz - cz;     // yükten yeni merkeze, m
        p.IxxKgM2[3] += m4 * d4z * d4z + mp * dpz * dpz;
        p.IyyKgM2[3] += m4 * (d4x * d4x + d4z * d4z) + mp * (dpx * dpx + dpz * dpz);
        p.IzzKgM2[3] += m4 * d4x * d4x + mp * dpx * dpx;
        p.IxzKgM2[3] += -m4 * d4x * d4z - mp * dpx * dpz;

        // 4) Yeni kütle ve kütle merkezi, tabloyla aynı biçimde (ℓ, ε) yazılır.
        p.MassKg[3] = mNew;
        p.ComDistanceM[3] = Math.Sqrt(cx * cx + cz * cz);
        p.ComAngleRad[3] = Math.Atan2(-cz, cx);
        return p;
    }
}
