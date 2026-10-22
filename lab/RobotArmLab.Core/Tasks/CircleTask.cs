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
    private const int ValidationPointCount = 360;   // Validate'in denediği nokta sayısı: her derecede bir

    private readonly ArmParameters parameters;
    private readonly Point3 center;        // çemberin merkezi, m
    private readonly double radiusM;       // yarıçap r, m
    private readonly CirclePlane plane;
    private readonly double periodS;       // bir turun süresi P, s
    private readonly double pitchRad;      // takım eğimi ψ, rad (sabit)
    private readonly bool elbowUp;         // dirsek dalı (true: yukarı)
    private readonly double approachS;     // yaklaşma süresi Ta, s
    private readonly double rampS;         // açısal hızın yumuşak başlama süresi Tr, s
    private readonly QuinticTask approach;
    private double[] lastQ;   // ters kinematik bir an başarısız olursa son iyi çözüm tutulur

    /// <summary>
    /// Görevi kurar. Çemberin ilk noktası erişilemezse yaklaşma kurulamaz; bunu <see cref="Validate"/> bildirir.
    /// </summary>
    public CircleTask(ArmParameters parameters, double[] start, Point3 center, double radiusM, CirclePlane plane,
        double periodS, double pitchRad, bool elbowUp, double approachS, double rampS)
    {
        this.parameters = parameters;
        this.center = center;
        this.radiusM = radiusM;
        this.plane = plane;
        this.periodS = periodS;
        this.pitchRad = pitchRad;
        this.elbowUp = elbowUp;
        this.approachS = approachS;
        this.rampS = rampS;
        lastQ = (double[])start.Clone();
        double[] first = Solve(0.0);
        approach = new QuinticTask(start, first ?? start, 0.0, approachS);
    }

    /// <summary>
    /// Çemberin her derecesindeki noktayı (360 nokta) ters kinematikle (eklem sınırlarıyla) dener; sorun yoksa boş metin, varsa Türkçe neden döndürür.
    /// </summary>
    public string Validate()
    {
        for (int k = 0; k < ValidationPointCount; k++)
        {
            double phi = 2.0 * Math.PI * k / ValidationPointCount;
            if (Solve(phi) == null)
                return "Çember φ = " + k + "° noktasında erişim alanının veya eklem sınırlarının dışına çıkıyor.";
        }
        return "";
    }

    /// <summary>Çember üzerindeki φ açısına (rad) karşılık gelen takım noktası, m.</summary>
    public Point3 PointAt(double phi)
    {
        double a = radiusM * Math.Cos(phi), b = radiusM * Math.Sin(phi);
        if (plane == CirclePlane.Horizontal)
            return new Point3(center.X + a, center.Y + b, center.Z);
        return new Point3(center.X + a, center.Y, center.Z + b);
    }

    /// <summary>t anındaki referans: t &lt; Ta ise yaklaşma, sonra çember (hız ve ivme merkezî farkla).</summary>
    public JointReference At(double t)
    {
        if (t < approachS)
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

    // Çember hareketinin başlamasından τ saniye sonraki açı φ(τ), rad (τ ≤ 0 için 0).
    private double PhaseAt(double tau)
    {
        double omega = 2.0 * Math.PI / periodS;
        if (tau <= 0.0)
            return 0.0;
        if (rampS > 0.0 && tau < rampS)
        {
            // φ̇ = ω(3x² − 2x³) ifadesinin integrali: φ = ω·Tr·(x³ − x⁴/2), x = τ/Tr.
            double x = tau / rampS;
            return omega * rampS * (x * x * x - 0.5 * x * x * x * x);
        }
        // Yumuşak başlama bitince (x = 1) biriken açı ω·Tr/2'dir; sonra φ̇ = ω sabit:
        // φ = ω·Tr/2 + ω·(τ − Tr) = ω·(τ − Tr/2).
        return omega * (tau - 0.5 * rampS);
    }

    private double[] JointsAt(double t)
    {
        double[] q = Solve(PhaseAt(t - approachS));
        if (q != null)
            lastQ = q;
        return lastQ;
    }

    private double[] Solve(double phi)
    {
        Point3 p = PointAt(phi);
        IkResult ik = InverseKinematics.Solve(parameters, p.X, p.Y, p.Z, pitchRad, elbowUp, true);
        return ik.Success ? ik.Q : null;
    }
}
