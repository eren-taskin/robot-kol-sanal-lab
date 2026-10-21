using System;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;

namespace RobotArmLab.Core.Dynamics;

/// <summary>
/// Kütle (eylemsizlik) matrisi M(q) ve onun eklem açılarına göre türevi ∂M/∂q, raporun kapalı formlarıyla
/// (eş. (14)–(17), SPEC §5.3). M, kinetik enerjiyi verir: T = ½ q̇ᵀ M(q) q̇. Taban eklemi (q1) eğim
/// eklemleriyle kütle matrisinde bağlaşmaz: M1k = 0 (k = 2, 3, 4).
/// </summary>
public static class MassMatrixFormulas
{
    /// <summary>
    /// M(q), kg·m². Köşegene motorun yansıyan eylemsizliği N²·Jm eklenir (SPEC §5.2).
    /// </summary>
    public static double[,] Compute(ArmParameters p, double[] q)
    {
        double[,] m = new double[4, 4];
        m[0, 0] = YawInertia(p, q);                            // eş. (14)–(17): m11
        double[] k = CouplingTerms(p, q);
        FillPitchBlock(m, p, k[0], k[1], k[2], k[3]);          // eş. (14)–(17): eğim bloğu
        // M12 = M13 = M14 = 0: taban dönüşü eğim eklemleriyle bağlaşmaz. Dizi sıfırla başladığı için yazılmaz.

        for (int i = 0; i < 4; i++)
            m[i, i] += p.ReflectedRotorInertiaKgM2;
        return m;
    }

    /// <summary>
    /// ∂M/∂q analitik olarak: sonuç dM[k, i, j] = ∂M_ij / ∂q_k, kg·m²/rad. Christoffel sembolleri (eş. (13))
    /// bundan kurulur. M, q1'e bağlı olmadığından dM[0, *, *] = 0 kalır.
    /// </summary>
    public static double[,,] Derivative(ArmParameters p, double[] q)
    {
        double[,,] dM = new double[4, 4, 4];
        double[] dk = new double[4];
        double[,] block = new double[4, 4];
        for (int k = 1; k < 4; k++)
        {
            dM[k, 0, 0] = YawInertiaDerivative(p, q, k);

            // Eğim bloğu yalnız h, k23, k24, k34 üzerinden q'ya bağlıdır.
            CouplingTermDerivatives(p, q, k, dk);
            FillPitchBlockDerivative(block, p, dk[0], dk[1], dk[2], dk[3]);
            for (int i = 1; i < 4; i++)
                for (int j = 1; j < 4; j++)
                    dM[k, i, j] = block[i, j];
        }
        return dM;
    }

    // m11 = Σ [ m ρ² + Ixx sin²θ + Izz cos²θ − 2 Ixz sin θ cos θ ], kg·m².
    private static double YawInertia(ArmParameters p, double[] q)
    {
        double[] t = ForwardKinematics.Thetas(q);
        double[] rho = ForwardKinematics.ComRadii(p, q);
        double m11 = 0.0;
        for (int i = 0; i < 4; i++)
        {
            double s = Math.Sin(t[i]), c = Math.Cos(t[i]);
            m11 += p.MassKg[i] * rho[i] * rho[i]
                 + p.IxxKgM2[i] * s * s + p.IzzKgM2[i] * c * c - 2.0 * p.IxzKgM2[i] * s * c;
        }
        return m11;
    }

    // ∂m11/∂q_k = Σ_i [ 2 m_i ρ_i ∂ρ_i/∂q_k + ((Ixx − Izz) sin 2θ_i − 2 Ixz cos 2θ_i) ∂θ_i/∂q_k ].
    private static double YawInertiaDerivative(ArmParameters p, double[] q, int k)
    {
        double[] t = ForwardKinematics.Thetas(q);
        double[] rho = ForwardKinematics.ComRadii(p, q);
        double sum = 0.0;
        for (int i = 0; i < 4; i++)
        {
            sum += 2.0 * p.MassKg[i] * rho[i] * RadiusDerivative(p, t, i, k);
            if (ThetaDependsOn(i, k))
                sum += (p.IxxKgM2[i] - p.IzzKgM2[i]) * Math.Sin(2.0 * t[i]) - 2.0 * p.IxzKgM2[i] * Math.Cos(2.0 * t[i]);
        }
        return sum;
    }

    // ∂ρ_i/∂q_k, m/rad. ρ_i, kendinden önceki uzuvların payları ile kendi kütle merkezinin payının toplamıdır.
    private static double RadiusDerivative(ArmParameters p, double[] t, int i, int k)
    {
        double d = 0.0;
        if (i >= 2 && k == 1)                         // L2 cos(θ2 − β) payı: θ2 yalnız q2'ye bağlı
            d -= p.L2M * Math.Sin(t[1] - p.BetaRad);
        if (i == 3 && (k == 1 || k == 2))             // L3 cos θ3 payı: θ3, q2 ve q3'e bağlı
            d -= p.L3M * Math.Sin(t[2]);
        if (ThetaDependsOn(i, k))                     // ℓ_i cos(θ_i + ε_i) payı
            d -= p.ComDistanceM[i] * Math.Sin(t[i] + p.ComAngleRad[i]);
        return d;
    }

    // θ_i, q_k'yı içeriyor mu? θ_i = q2 + … + q_i olduğundan 1 ≤ k ≤ i ise evet (sıfır tabanlı indis).
    private static bool ThetaDependsOn(int i, int k)
    {
        return k >= 1 && k <= i;
    }

    // Bağlaşım terimleri [h, k23, k24, k34], m² (SPEC §5.3). Kütleyle çarpılınca kg·m² olur.
    private static double[] CouplingTerms(ArmParameters p, double[] q)
    {
        double b = p.BetaRad, e3 = p.ComAngleRad[2], e4 = p.ComAngleRad[3];
        double l3 = p.ComDistanceM[2], l4 = p.ComDistanceM[3];
        double h = p.L2M * l3 * Math.Cos(q[2] + b + e3);
        double k23 = p.L2M * p.L3M * Math.Cos(q[2] + b);
        double k24 = p.L2M * l4 * Math.Cos(q[2] + q[3] + b + e4);
        double k34 = p.L3M * l4 * Math.Cos(q[3] + e4);
        return new double[] { h, k23, k24, k34 };
    }

    // Bağlaşım terimlerinin q_k'ya göre türevi, m²/rad. h ve k23 yalnız q3'e, k34 yalnız q4'e, k24 ikisine bağlı.
    private static void CouplingTermDerivatives(ArmParameters p, double[] q, int k, double[] dk)
    {
        double b = p.BetaRad, e3 = p.ComAngleRad[2], e4 = p.ComAngleRad[3];
        double l3 = p.ComDistanceM[2], l4 = p.ComDistanceM[3];
        dk[0] = (k == 2) ? -p.L2M * l3 * Math.Sin(q[2] + b + e3) : 0.0;
        dk[1] = (k == 2) ? -p.L2M * p.L3M * Math.Sin(q[2] + b) : 0.0;
        dk[2] = (k == 2 || k == 3) ? -p.L2M * l4 * Math.Sin(q[2] + q[3] + b + e4) : 0.0;
        dk[3] = (k == 3) ? -p.L3M * l4 * Math.Sin(q[3] + e4) : 0.0;
    }

    // Eğim bloğu M22..M44, kg·m² (simetrik). Eş. (14)–(17), SPEC §5.3 satır satır. J_i = Iyy_i.
    private static void FillPitchBlock(double[,] m, ArmParameters p, double h, double k23, double k24, double k34)
    {
        double J2 = p.IyyKgM2[1], J3 = p.IyyKgM2[2], J4 = p.IyyKgM2[3];
        double m2 = p.MassKg[1], m3 = p.MassKg[2], m4 = p.MassKg[3];
        double l2 = p.ComDistanceM[1], l3 = p.ComDistanceM[2], l4 = p.ComDistanceM[3];
        double L2 = p.L2M, L3 = p.L3M;

        double M44 = J4 + m4 * l4 * l4;
        double M34 = J4 + m4 * (l4 * l4 + k34);
        double M33 = J3 + J4 + m3 * l3 * l3 + m4 * (L3 * L3 + l4 * l4 + 2.0 * k34);
        double M24 = J4 + m4 * (l4 * l4 + k34 + k24);
        double M23 = J3 + J4 + m3 * (l3 * l3 + h) + m4 * (L3 * L3 + l4 * l4 + 2.0 * k34 + k23 + k24);
        double M22 = J2 + J3 + J4 + m2 * l2 * l2 + m3 * (L2 * L2 + l3 * l3 + 2.0 * h)
                   + m4 * (L2 * L2 + L3 * L3 + l4 * l4 + 2.0 * k23 + 2.0 * k24 + 2.0 * k34);
        SetSymmetricBlock(m, M22, M23, M24, M33, M34, M44);
    }

    // Eğim bloğunun q_k'ya göre türevi: yukarıdaki satırların türevi. Sabit terimler (J, m ℓ², m L²) sıfır olur,
    // geriye yalnız bağlaşım terimlerinin türevleri (dh, dk23, dk24, dk34) kalır. kg·m²/rad.
    private static void FillPitchBlockDerivative(double[,] m, ArmParameters p, double dh, double dk23, double dk24, double dk34)
    {
        double m3 = p.MassKg[2], m4 = p.MassKg[3];

        double dM44 = 0.0;
        double dM34 = m4 * dk34;
        double dM33 = m4 * 2.0 * dk34;
        double dM24 = m4 * (dk34 + dk24);
        double dM23 = m3 * dh + m4 * (2.0 * dk34 + dk23 + dk24);
        double dM22 = m3 * 2.0 * dh + m4 * (2.0 * dk23 + 2.0 * dk24 + 2.0 * dk34);
        SetSymmetricBlock(m, dM22, dM23, dM24, dM33, dM34, dM44);
    }

    // 3×3 simetrik eğim bloğunu 4×4 matrisin sağ alt köşesine yazar.
    private static void SetSymmetricBlock(double[,] m, double a22, double a23, double a24, double a33, double a34, double a44)
    {
        m[1, 1] = a22; m[1, 2] = a23; m[1, 3] = a24;
        m[2, 1] = a23; m[2, 2] = a33; m[2, 3] = a34;
        m[3, 1] = a24; m[3, 2] = a34; m[3, 3] = a44;
    }
}
