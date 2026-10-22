using System;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Numerics;

namespace RobotArmLab.Core.Dynamics;

/// <summary>
/// Kolun hareket denklemi (rapor eş. (11)): M(q) q̈ + c(q, q̇) + g(q) = τ − τf(q̇).
/// Bu sınıf denklemin her terimini ayrı ayrı hesaplar ve ileri dinamiği (q̈'yı) bulur. Parametreler (yük
/// eklenmişse yüklü uzuv 4 ile) ve yerçekimi g kurucuda verilir.
/// </summary>
public class ArmDynamics
{
    /// <summary>Dinamiğin kullandığı parametreler (yük zaten uzuv 4'e katılmış olmalıdır).</summary>
    public ArmParameters Parameters { get; }

    /// <summary>Yerçekimi ivmesi g, m/s² (−z yönünde).</summary>
    public double GravityMps2 { get; }

    /// <summary>Parametreler ve yerçekimi ivmesi (m/s²) ile dinamik modeli kurar.</summary>
    public ArmDynamics(ArmParameters parameters, double gravityMps2)
    {
        Parameters = parameters;
        GravityMps2 = gravityMps2;
    }

    /// <summary>
    /// Kütle matrisi M(q), kg·m². Rapor eş. (14) m11, (15) ρ_i, (16) h, k23, k24, k34, (17) eğim bloğu.
    /// Ayrıntı <see cref="MassMatrixFormulas"/> sınıfında.
    /// </summary>
    public double[,] MassMatrix(double[] q)
    {
        return MassMatrixFormulas.Compute(Parameters, q);
    }

    /// <summary>
    /// Coriolis ve merkezkaç tork vektörü c(q, q̇) = C(q, q̇) q̇, N·m. Rapor eş. (13), Christoffel sembolleri:
    /// c_k = Σ_i Σ_j c_ijk q̇_i q̇_j,  c_ijk = ½ (∂M_kj/∂q_i + ∂M_ki/∂q_j − ∂M_ij/∂q_k).
    /// Hız sıfırsa bu terim sıfırdır; hızın karesiyle büyür.
    /// </summary>
    public double[] CoriolisVector(double[] q, double[] qd)
    {
        double[,,] dM = MassMatrixFormulas.Derivative(Parameters, q);   // dM[k, i, j] = ∂M_ij/∂q_k
        double[] c = new double[4];
        for (int k = 0; k < 4; k++)
        {
            double sum = 0.0;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    double christoffel = 0.5 * (dM[i, k, j] + dM[j, k, i] - dM[k, i, j]);
                    sum += christoffel * qd[i] * qd[j];
                }
            }
            c[k] = sum;
        }
        return c;
    }

    /// <summary>
    /// Yerçekimi tork vektörü g(q), N·m. Rapor eş. (18). Bilekten omuza doğru toplanır: her eklem, kendisinden
    /// sonraki bütün uzuvların ağırlığını taşır. Taban eklemi düşey eksenli olduğundan g1 = 0.
    /// </summary>
    public double[] GravityVector(double[] q)
    {
        ArmParameters p = Parameters;
        double[] t = ForwardKinematics.Thetas(q);
        double g = GravityMps2;
        double m2 = p.MassKg[1], m3 = p.MassKg[2], m4 = p.MassKg[3];
        double l2 = p.ComDistanceM[1], l3 = p.ComDistanceM[2], l4 = p.ComDistanceM[3];
        double e2 = p.ComAngleRad[1], e3 = p.ComAngleRad[2], e4 = p.ComAngleRad[3];

        double g4 = -g * m4 * l4 * Math.Cos(t[3] + e4);                                             // eş. (18)
        double g3 = g4 - g * (m3 * l3 * Math.Cos(t[2] + e3) + m4 * p.L3M * Math.Cos(t[2]));          // eş. (18)
        double g2 = g3 - g * (m2 * l2 * Math.Cos(t[1] + e2) + (m3 + m4) * p.L2M * Math.Cos(t[1] - p.BetaRad)); // eş. (18)
        double g1 = 0.0;                                                                            // eş. (18)
        return new double[] { g1, g2, g3, g4 };
    }

    /// <summary>
    /// Eklem sürtünme torku τf, N·m. Rapor eş. (20): τf = fv q̇ + fc tanh(q̇ / ωs). tanh, Coulomb
    /// sürtünmesinin sıfır hızdaki ani işaret değişimini yumuşatır (yoksa çözücü takılırdı).
    /// </summary>
    public double[] Friction(double[] qd)
    {
        ArmParameters p = Parameters;
        double[] f = new double[4];
        for (int i = 0; i < 4; i++)
            f[i] = p.ViscousFrictionNmsPerRad * qd[i] + p.CoulombFrictionNm * Math.Tanh(qd[i] / p.FrictionSmoothingRadPerS);
        return f;
    }

    /// <summary>
    /// İleri dinamik: eklem torku τ (N·m) verilince ivme q̈ = M⁻¹ (τ − c − g − τf), rad/s². Rapor eş. (21).
    /// M'nin tersi alınmaz; M q̈ = b denklemi Cholesky ile çözülür.
    /// </summary>
    public double[] Acceleration(double[] q, double[] qd, double[] tau)
    {
        double[,] m = MassMatrix(q);
        double[] c = CoriolisVector(q, qd);
        double[] g = GravityVector(q);
        double[] f = Friction(qd);

        double[] b = new double[4];
        for (int i = 0; i < 4; i++)
            b[i] = tau[i] - c[i] - g[i] - f[i];
        return Small4x4.SolveSymmetricPositiveDefinite(m, b);
    }

    /// <summary>
    /// Ters dinamik: istenen ivme q̈ (rad/s²) için gereken tork τ = M(q) q̈ + c(q, q̇) + g(q) + τf(q̇), N·m.
    /// Eş. (11)'in τ'ya göre çözülmüş biçimi; model tabanlı kontrolcüler kendi (tahmini) modelleriyle bunu çağırır.
    /// </summary>
    public double[] InverseDynamics(double[] q, double[] qd, double[] qdd)
    {
        double[] mqdd = Small4x4.Multiply(MassMatrix(q), qdd);
        double[] c = CoriolisVector(q, qd);
        double[] g = GravityVector(q);
        double[] f = Friction(qd);
        double[] tau = new double[4];
        for (int i = 0; i < 4; i++)
            tau[i] = mqdd[i] + c[i] + g[i] + f[i];
        return tau;
    }

    /// <summary>Kinetik enerji T = ½ q̇ᵀ M(q) q̇, J.</summary>
    public double KineticEnergy(double[] q, double[] qd)
    {
        double[] mqd = Small4x4.Multiply(MassMatrix(q), qd);
        double sum = 0.0;
        for (int i = 0; i < 4; i++)
            sum += qd[i] * mqd[i];
        return 0.5 * sum;
    }

    /// <summary>
    /// Potansiyel enerji V = g Σ m_i z_i, J. z_i, uzuv i'nin kütle merkezinin yüksekliğidir (taban z = 0).
    /// g(q) = ∂V/∂q olmalıdır; testler bunu sayısal türevle doğrular.
    /// </summary>
    public double PotentialEnergy(double[] q)
    {
        double[] z = ForwardKinematics.ComHeights(Parameters, q);
        double sum = 0.0;
        for (int i = 0; i < 4; i++)
            sum += Parameters.MassKg[i] * z[i];
        return GravityMps2 * sum;
    }
}
