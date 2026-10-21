using System;
using RobotArmLab.Core.Model;

namespace RobotArmLab.Core.Kinematics;

/// <summary>
/// İleri kinematik: eklem açılarından (q1..q4, rad) eklem noktalarının ve takım noktasının konumunu bulur.
/// Rapor eş. (1)–(4). Kol bir taban dönüşü (q1) ve üç paralel eğim ekseninden (q2, q3, q4) oluşur; bu yüzden
/// hesap önce düşey düzlemde (r, z) yapılır, sonra q1 ile döndürülür.
/// </summary>
public static class ForwardKinematics
{
    /// <summary>
    /// Her uzvun yatayın altındaki mutlak eğim açısı θ, rad. Eş. (1): θ1 = 0, θ2 = q2, θ3 = q2 + q3,
    /// θ4 = q2 + q3 + q4. Pozitif açı kolu öne ve aşağı yatırır.
    /// </summary>
    public static double[] Thetas(double[] q)
    {
        double[] theta = new double[4];
        theta[0] = 0.0;
        theta[1] = q[1];
        theta[2] = q[1] + q[2];
        theta[3] = q[1] + q[2] + q[3];
        return theta;
    }

    /// <summary>
    /// Kolun beş çizim noktası, taban çerçevesinde (m): [0] taban, [1] omuz (eklem 2), [2] dirsek (eklem 3),
    /// [3] bilek (eklem 4), [4] takım noktası. Eş. (2)–(4) her ara noktada ayrı ayrı uygulanır.
    /// </summary>
    public static Point3[] JointPositions(ArmParameters p, double[] q)
    {
        double[] t = Thetas(q);
        double beta = p.BetaRad;

        // Düşey düzlemde yatay uzaklık r ve yükseklik z, eklemden ekleme toplanarak (m).
        double r2 = 0.0, z2 = p.D1M;
        double r3 = r2 + p.L2M * Math.Cos(t[1] - beta), z3 = z2 - p.L2M * Math.Sin(t[1] - beta);
        double r4 = r3 + p.L3M * Math.Cos(t[2]), z4 = z3 - p.L3M * Math.Sin(t[2]);
        double r5 = r4 + p.L4M * Math.Cos(t[3]), z5 = z4 - p.L4M * Math.Sin(t[3]);

        Point3[] points = new Point3[5];
        points[0] = new Point3(0.0, 0.0, 0.0);
        points[1] = InBaseFrame(r2, z2, q[0]);
        points[2] = InBaseFrame(r3, z3, q[0]);
        points[3] = InBaseFrame(r4, z4, q[0]);
        points[4] = InBaseFrame(r5, z5, q[0]);
        return points;
    }

    /// <summary>
    /// Takım noktasının konumu p = [r cos q1, r sin q1, z], m. Eş. (2)–(4):
    /// r = L2 cos(θ2 − β) + L3 cos θ3 + L4 cos θ4,  z = d1 − L2 sin(θ2 − β) − L3 sin θ3 − L4 sin θ4.
    /// </summary>
    public static Point3 ToolPosition(ArmParameters p, double[] q)
    {
        Point3[] points = JointPositions(p, q);
        return points[4];
    }

    /// <summary>Takımın yatayın altındaki eğim açısı ψ = θ4 = q2 + q3 + q4, rad.</summary>
    public static double ToolPitch(double[] q)
    {
        return q[1] + q[2] + q[3];
    }

    /// <summary>
    /// Her uzvun kütle merkezinin dönme eksenine (z eksenine) yatay uzaklığı ρ_i, m. Rapor eş. (14)'teki
    /// ρ1..ρ4 tanımları; m11 (taban eylemsizliği) bunlarla hesaplanır.
    /// </summary>
    public static double[] ComRadii(ArmParameters p, double[] q)
    {
        double[] t = Thetas(q);
        double a2 = p.L2M * Math.Cos(t[1] - p.BetaRad);   // eklem 3'ün yatay uzaklığı, m
        double a3 = p.L3M * Math.Cos(t[2]);               // eklem 3 → eklem 4 yatay payı, m
        double[] rho = new double[4];
        rho[0] = p.ComDistanceM[0] * Math.Cos(t[0] + p.ComAngleRad[0]);
        rho[1] = p.ComDistanceM[1] * Math.Cos(t[1] + p.ComAngleRad[1]);
        rho[2] = a2 + p.ComDistanceM[2] * Math.Cos(t[2] + p.ComAngleRad[2]);
        rho[3] = a2 + a3 + p.ComDistanceM[3] * Math.Cos(t[3] + p.ComAngleRad[3]);
        return rho;
    }

    /// <summary>
    /// Her uzvun kütle merkezinin yüksekliği z_i, m. Potansiyel enerji V = g Σ m_i z_i bununla hesaplanır.
    /// Kütle merkezi uzuv çerçevesinde (ℓ cos ε, 0, −ℓ sin ε) noktasındadır, yatayın θ + ε altında.
    /// </summary>
    public static double[] ComHeights(ArmParameters p, double[] q)
    {
        double[] t = Thetas(q);
        double z3 = p.D1M - p.L2M * Math.Sin(t[1] - p.BetaRad);   // eklem 3 yüksekliği, m
        double z4 = z3 - p.L3M * Math.Sin(t[2]);                   // eklem 4 yüksekliği, m
        double[] z = new double[4];
        z[0] = -p.ComDistanceM[0] * Math.Sin(t[0] + p.ComAngleRad[0]);
        z[1] = p.D1M - p.ComDistanceM[1] * Math.Sin(t[1] + p.ComAngleRad[1]);
        z[2] = z3 - p.ComDistanceM[2] * Math.Sin(t[2] + p.ComAngleRad[2]);
        z[3] = z4 - p.ComDistanceM[3] * Math.Sin(t[3] + p.ComAngleRad[3]);
        return z;
    }

    // Düşey düzlemdeki (r, z) noktayı taban açısı q1 ile döndürüp taban çerçevesine taşır.
    private static Point3 InBaseFrame(double r, double z, double q1)
    {
        return new Point3(r * Math.Cos(q1), r * Math.Sin(q1), z);
    }
}
