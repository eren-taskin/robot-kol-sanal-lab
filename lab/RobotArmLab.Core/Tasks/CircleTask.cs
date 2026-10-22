using System;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;

namespace RobotArmLab.Core.Tasks;

/// <summary>Çemberin çizildiği düzlem.</summary>
public enum CirclePlane
{
    /// <summary>Yatay düzlem (z sabit): x = cx + r cos φ, y = cy + r sin φ.</summary>
    Horizontal,

    /// <summary>Düşey x–z düzlemi (y sabit): x = cx + r cos φ, z = cz + r sin φ.</summary>
    Vertical,
}

/// <summary>
/// Çember izleme: önce başlangıç açılarından çemberin ilk noktasına (φ = 0) beşinci derece yaklaşma yörüngesi
/// (süre Ta), sonra takım noktası sabit eğim ψ ile çember çizer. Her an nokta ters kinematikle eklem açısına
/// çevrilir; q̇_ref ve q̈_ref bu açıların zamana göre merkezî farkıyla bulunur.
/// </summary>
/// <remarks>
/// Açısal hız yumuşak başlar: ilk Tr saniyede φ̇ = ω·(3x² − 2x³), x = τ/Tr, yani φ = ω·Tr·(x³ − x⁴/2); sonra
/// φ̇ = ω = 2π/P. Böylece yaklaşmadan çembere geçişte hız ve ivme sıçramaz. Sayısal türev adımı δ = 1e-4 s:
/// kesme hatası δ² mertebesinde, yuvarlama hatası ε/δ² ≈ 1e-8 rad/s² mertebesindedir.
/// </remarks>
public class CircleTask : IReferenceTask
{
    private const double DerivativeStepS = 1e-4;

    private readonly ArmParameters parameters;
    private readonly QuinticTask approach;
    private double[] lastQ;   // ters kinematik bir an başarısız olursa son iyi çözüm tutulur

    /// <summary>Çemberin merkezi, m.</summary>
    public Point3 Center { get; }

    /// <summary>Yarıçap r, m.</summary>
    public double RadiusM { get; }

    /// <summary>Çemberin düzlemi.</summary>
    public CirclePlane Plane { get; }

    /// <summary>Bir turun süresi P, s.</summary>
    public double PeriodS { get; }

    /// <summary>Takım eğimi ψ, rad (sabit).</summary>
    public double PitchRad { get; }

    /// <summary>Dirsek dalı (true: yukarı).</summary>
    public bool ElbowUp { get; }

    /// <summary>Yaklaşma süresi Ta, s.</summary>
    public double ApproachS { get; }

    /// <summary>Açısal hızın yumuşak başlama süresi Tr, s.</summary>
    public double RampS { get; }

    /// <summary>Arayüzde gösterilen ad.</summary>
    public string Name { get { return Plane == CirclePlane.Horizontal ? "Yatay çember izleme" : "Düşey çember izleme"; } }

    /// <summary>
    /// Görevi kurar. Çemberin ilk noktası erişilemezse yaklaşma kurulamaz; bunu <see cref="Validate"/> bildirir.
    /// </summary>
    public CircleTask(ArmParameters parameters, double[] start, Point3 center, double radiusM, CirclePlane plane,
        double periodS, double pitchRad, bool elbowUp, double approachS, double rampS)
    {
        this.parameters = parameters;
        Center = center;
        RadiusM = radiusM;
        Plane = plane;
        PeriodS = periodS;
        PitchRad = pitchRad;
        ElbowUp = elbowUp;
        ApproachS = approachS;
        RampS = rampS;
        lastQ = (double[])start.Clone();
        double[] first = Solve(0.0);
        approach = new QuinticTask(start, first ?? start, 0.0, approachS);
    }

    /// <summary>
    /// Çemberin 360 noktasını ters kinematikle (eklem sınırlarıyla) dener; sorun yoksa boş metin, varsa Türkçe neden döndürür.
    /// </summary>
    public string Validate()
    {
        for (int k = 0; k < 360; k++)
        {
            double phi = 2.0 * Math.PI * k / 360.0;
            if (Solve(phi) == null)
                return "Çember φ = " + k + "° noktasında erişim alanının veya eklem sınırlarının dışına çıkıyor.";
        }
        return "";
    }

    /// <summary>Çember üzerindeki φ açısına (rad) karşılık gelen takım noktası, m.</summary>
    public Point3 PointAt(double phi)
    {
        double a = RadiusM * Math.Cos(phi), b = RadiusM * Math.Sin(phi);
        if (Plane == CirclePlane.Horizontal)
            return new Point3(Center.X + a, Center.Y + b, Center.Z);
        return new Point3(Center.X + a, Center.Y, Center.Z + b);
    }

    /// <summary>t anındaki referans: t &lt; Ta ise yaklaşma, sonra çember (hız ve ivme merkezî farkla).</summary>
    public JointReference At(double t)
    {
        if (t < ApproachS)
            return approach.At(t);

        double d = DerivativeStepS;
        double[] qm = JointsAt(t - d), q0 = JointsAt(t), qp = JointsAt(t + d);
        JointReference r = new JointReference();
        for (int i = 0; i < 4; i++)
        {
            r.Q[i] = q0[i];
            r.Qd[i] = (qp[i] - qm[i]) / (2.0 * d);
            r.Qdd[i] = (qp[i] - 2.0 * q0[i] + qm[i]) / (d * d);
        }
        return r;
    }

    /// <summary>Çember hareketinin başlamasından τ saniye sonraki açı φ(τ), rad (τ ≤ 0 için 0).</summary>
    public double PhaseAt(double tau)
    {
        double omega = 2.0 * Math.PI / PeriodS;
        if (tau <= 0.0)
            return 0.0;
        if (RampS > 0.0 && tau < RampS)
        {
            double x = tau / RampS;
            return omega * RampS * (x * x * x - 0.5 * x * x * x * x);
        }
        return omega * (tau - 0.5 * RampS);
    }

    private double[] JointsAt(double t)
    {
        double[] q = Solve(PhaseAt(t - ApproachS));
        if (q != null)
            lastQ = q;
        return lastQ;
    }

    private double[] Solve(double phi)
    {
        Point3 p = PointAt(phi);
        IkResult ik = InverseKinematics.Solve(parameters, p.X, p.Y, p.Z, PitchRad, ElbowUp, true);
        return ik.Success ? ik.Q : null;
    }
}
