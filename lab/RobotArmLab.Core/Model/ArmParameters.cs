using System;
using System.ComponentModel;

namespace RobotArmLab.Core.Model;

/// <summary>
/// OpenMANIPULATOR-X kolunun bütün fiziksel parametreleri: uzuvların boyları, kütleleri, kütle merkezleri,
/// eylemsizlikleri, motor ve sürtünme değerleri, eklem sınırları. Değerler SPEC.md §3'ten (rapor Tablo 3 ve
/// Tablo 5) alınır. Sınıf bilerek düz tutulmuştur: öğrenci her alanı formdan değiştirebilir.
/// </summary>
/// <remarks>
/// Diziler eklem sırasıyla tutulur: indis 0 = eklem 1 (taban dönüşü), indis 3 = eklem 4 (bilek).
/// Uzuv i, eklem i ile birlikte döner. Uzuv 4 tutucuyu (gripper) da içerir.
/// </remarks>
public class ArmParameters
{
    // ---------------- Geometri (rapor Tablo 3, SPEC §3.1) ----------------

    /// <summary>Eklem 2 ekseninin taban çerçevesinden yüksekliği d1, m.</summary>
    [Category("1 Geometri"), DisplayName("d1 (m)"), Description("Eklem 2 ekseninin tabandan yüksekliği.")]
    public double D1M { get; set; }

    /// <summary>q2 = 0 iken eklem 3'ün eklem 2'ye göre ileri uzaklığı a2x, m.</summary>
    [Category("1 Geometri"), DisplayName("a2x (m)"), Description("q2 = 0 iken eklem 3'ün eklem 2'ye göre ileri uzaklığı.")]
    public double A2xM { get; set; }

    /// <summary>q2 = 0 iken eklem 3'ün eklem 2'ye göre yukarı uzaklığı a2z, m.</summary>
    [Category("1 Geometri"), DisplayName("a2z (m)"), Description("q2 = 0 iken eklem 3'ün eklem 2'ye göre yukarı uzaklığı.")]
    public double A2zM { get; set; }

    /// <summary>Eklem 3'ten eklem 4'e uzunluk L3, m.</summary>
    [Category("1 Geometri"), DisplayName("L3 (m)"), Description("Eklem 3'ten eklem 4'e uzunluk.")]
    public double L3M { get; set; }

    /// <summary>Eklem 4'ten takım (uç) noktasına uzunluk L4, m.</summary>
    [Category("1 Geometri"), DisplayName("L4 (m)"), Description("Eklem 4'ten takım noktasına uzunluk.")]
    public double L4M { get; set; }

    /// <summary>Eklem 2–eklem 3 arası düz uzaklık L2 = √(a2x² + a2z²), m. Hesaplanır, elle yazılmaz.</summary>
    [Category("1 Geometri"), DisplayName("L2 (m, hesaplanır)"), Description("√(a2x² + a2z²); a2x ve a2z'den hesaplanır, elle yazılmaz.")]
    public double L2M { get { return Math.Sqrt(A2xM * A2xM + A2zM * A2zM); } }

    /// <summary>Uzuv 2'nin kırık açısı β = atan2(a2z, a2x), rad (yaklaşık 79,38°).</summary>
    [Category("1 Geometri"), DisplayName("β (rad, hesaplanır)"), Description("Uzuv 2'nin kırık açısı atan2(a2z, a2x).")]
    public double BetaRad { get { return Math.Atan2(A2zM, A2xM); } }

    // ---------------- Uzuvlar (rapor Tablo 5, SPEC §3.2), indis 0..3 ----------------

    /// <summary>Uzuv kütleleri m_i, kg.</summary>
    [Category("2 Uzuvlar"), DisplayName("Kütle m (kg)"), Description("Uzuv 1..4 kütleleri. Sıfırdan büyük olmalı.")]
    public double[] MassKg { get; set; }

    /// <summary>Eklem i'den uzuv i'nin kütle merkezine uzaklık ℓ_i, m.</summary>
    [Category("2 Uzuvlar"), DisplayName("Kütle merkezi uzaklığı ℓ (m)"), Description("Eklem i'den uzuv i'nin kütle merkezine uzaklık.")]
    public double[] ComDistanceM { get; set; }

    /// <summary>Kütle merkezinin uzuv doğrultusundan sapma açısı ε_i, rad (yatayın altında θ_i + ε_i).</summary>
    [Category("2 Uzuvlar"), DisplayName("Kütle merkezi açısı ε (rad)"), Description("Kütle merkezinin uzuv doğrultusundan sapma açısı.")]
    public double[] ComAngleRad { get; set; }

    /// <summary>Kütle merkezi etrafında Ixx eylemsizliği (uzuv çerçevesinde), kg·m².</summary>
    [Category("2 Uzuvlar"), DisplayName("Ixx (kg·m²)"), Description("Kütle merkezi etrafında Ixx eylemsizliği.")]
    public double[] IxxKgM2 { get; set; }

    /// <summary>Kütle merkezi etrafında Iyy eylemsizliği (eğim eksenine göre, rapordaki J_i), kg·m².</summary>
    [Category("2 Uzuvlar"), DisplayName("Iyy = J (kg·m²)"), Description("Eğim eksenine göre eylemsizlik (rapordaki J_i).")]
    public double[] IyyKgM2 { get; set; }

    /// <summary>Kütle merkezi etrafında Izz eylemsizliği, kg·m².</summary>
    [Category("2 Uzuvlar"), DisplayName("Izz (kg·m²)"), Description("Kütle merkezi etrafında Izz eylemsizliği.")]
    public double[] IzzKgM2 { get; set; }

    /// <summary>Çarpım eylemsizliği Ixz, kg·m². Ixy ve Iyz sıfır kabul edilir (SPEC §3.2).</summary>
    [Category("2 Uzuvlar"), DisplayName("Ixz (kg·m²)"), Description("Çarpım eylemsizliği Ixz.")]
    public double[] IxzKgM2 { get; set; }

    // ---------------- Motor ve sürtünme (SPEC §3.3), her eklem için aynı ----------------

    /// <summary>Motor rotorunun eylemsizliği Jm (dişliden önce), kg·m². Bilinmiyor, varsayılan 0.</summary>
    [Category("3 Motor ve sürtünme"), DisplayName("Rotor eylemsizliği Jm (kg·m²)"), Description("Dişliden önceki rotor eylemsizliği; bilinmiyor, varsayılan 0.")]
    public double RotorInertiaKgM2 { get; set; }

    /// <summary>Dişli oranı N (boyutsuz). XM430-W350 için 353,5.</summary>
    [Category("3 Motor ve sürtünme"), DisplayName("Dişli oranı N"), Description("XM430-W350 için 353,5.")]
    public double GearRatio { get; set; }

    /// <summary>Viskoz sürtünme katsayısı fv, N·m·s/rad.</summary>
    [Category("3 Motor ve sürtünme"), DisplayName("Viskoz sürtünme fv (N·m·s/rad)"), Description("τf = fv q̇ + fc tanh(q̇/ωs). Enerji denemesi için 0 yapın.")]
    public double ViscousFrictionNmsPerRad { get; set; }

    /// <summary>Coulomb sürtünme büyüklüğü fc, N·m.</summary>
    [Category("3 Motor ve sürtünme"), DisplayName("Coulomb sürtünmesi fc (N·m)"), Description("τf = fv q̇ + fc tanh(q̇/ωs). Enerji denemesi için 0 yapın.")]
    public double CoulombFrictionNm { get; set; }

    /// <summary>Coulomb sürtünmesini yumuşatan tanh hızı ωs, rad/s. Küçüldükçe sistem sertleşir.</summary>
    [Category("3 Motor ve sürtünme"), DisplayName("Yumuşatma hızı ωs (rad/s)"), Description("Coulomb sürtünmesini yumuşatan tanh hızı.")]
    public double FrictionSmoothingRadPerS { get; set; }

    /// <summary>Eklem çıkışındaki tork sınırı (doyum) τmax, N·m. 12 V'ta tutulma (stall) torku 4,1 N·m.</summary>
    [Category("3 Motor ve sürtünme"), DisplayName("Tork sınırı τmax (N·m)"), Description("Eklem çıkışındaki doyum; 12 V tutulma torku 4,1 N·m.")]
    public double TorqueLimitNm { get; set; }

    // ---------------- Eklem sınırları (SPEC §3.4, URDF) ----------------

    /// <summary>Eklem alt sınırları, rad.</summary>
    [Category("4 Eklem sınırları"), DisplayName("Alt sınırlar (rad)"), Description("Eklem 1..4 alt sınırları.")]
    public double[] LowerLimitRad { get; set; }

    /// <summary>Eklem üst sınırları, rad.</summary>
    [Category("4 Eklem sınırları"), DisplayName("Üst sınırlar (rad)"), Description("Eklem 1..4 üst sınırları.")]
    public double[] UpperLimitRad { get; set; }

    /// <summary>
    /// SPEC.md §3'teki varsayılan değerlerle yeni bir parametre takımı oluşturur.
    /// Bütün kütleler URDF'ten alınmıştır; açılar dereceden radyana çevrilir.
    /// </summary>
    public static ArmParameters Default()
    {
        ArmParameters p = new ArmParameters();
        p.D1M = 0.0595;
        p.A2xM = 0.024;
        p.A2zM = 0.128;
        p.L3M = 0.124;
        p.L4M = 0.126;

        // Tablo 5: m (kg), ℓ (m), ε (derece), Ixx, Iyy, Izz, Ixz (kg·m²)
        p.MassKg = new double[] { 0.0984, 0.1385, 0.1327, 0.1433 };
        p.ComDistanceM = new double[] { 0.0474, 0.1022, 0.0909, 0.0451 };
        p.ComAngleRad = new double[] { Deg(-90.4), Deg(-84.2), Deg(-0.1), Deg(-11.4) };
        p.IxxKgM2 = new double[] { 34.5e-6, 330.6e-6, 30.7e-6, 80.9e-6 };
        p.IyyKgM2 = new double[] { 32.7e-6, 342.9e-6, 242.3e-6, 76.0e-6 };
        p.IzzKgM2 = new double[] { 18.9e-6, 60.3e-6, 251.6e-6, 93.1e-6 };
        p.IxzKgM2 = new double[] { -0.4e-6, -38.5e-6, -0.3e-6, -1.0e-6 };

        p.RotorInertiaKgM2 = 0.0;
        p.GearRatio = 353.5;
        p.ViscousFrictionNmsPerRad = 0.01;
        p.CoulombFrictionNm = 0.02;
        p.FrictionSmoothingRadPerS = 0.1;
        p.TorqueLimitNm = 4.1;

        p.LowerLimitRad = new double[] { -Math.PI, -1.5, -1.5, -1.7 };
        p.UpperLimitRad = new double[] { Math.PI, 1.5, 1.4, 1.97 };
        return p;
    }

    /// <summary>
    /// Derin kopya: diziler de kopyalanır. Böylece kopya değiştirilince asıl parametreler bozulmaz
    /// (ör. yük eklenmiş kol, gerçek kolu değiştirmez).
    /// </summary>
    public ArmParameters Clone()
    {
        ArmParameters c = (ArmParameters)MemberwiseClone();
        c.MassKg = (double[])MassKg.Clone();
        c.ComDistanceM = (double[])ComDistanceM.Clone();
        c.ComAngleRad = (double[])ComAngleRad.Clone();
        c.IxxKgM2 = (double[])IxxKgM2.Clone();
        c.IyyKgM2 = (double[])IyyKgM2.Clone();
        c.IzzKgM2 = (double[])IzzKgM2.Clone();
        c.IxzKgM2 = (double[])IxzKgM2.Clone();
        c.LowerLimitRad = (double[])LowerLimitRad.Clone();
        c.UpperLimitRad = (double[])UpperLimitRad.Clone();
        return c;
    }

    /// <summary>Motor rotorunun eklem tarafına yansıyan eylemsizliği N²·Jm, kg·m². M'nin köşegenine eklenir.</summary>
    [Category("3 Motor ve sürtünme"), DisplayName("N²·Jm (kg·m², hesaplanır)"), Description("Motor rotorunun eklemde görünen eylemsizliği; M'nin köşegenine eklenir.")]
    public double ReflectedRotorInertiaKgM2 { get { return GearRatio * GearRatio * RotorInertiaKgM2; } }

    private static double Deg(double degrees)
    {
        return degrees * Math.PI / 180.0;
    }
}
