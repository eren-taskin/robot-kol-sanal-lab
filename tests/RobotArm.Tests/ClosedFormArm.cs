using RobotArm.Core.Model;
using G = RobotArm.Core.Model.OpenManipulatorXGeometry;

namespace RobotArm.Tests;

/// <summary>
/// Independent check of the general dynamics: the closed forms of SPEC §5.3 (report eqs. 13–18) for the
/// OpenMANIPULATOR-X, typed from SPEC exactly, plus the Christoffel Coriolis matrix with ∂M/∂q by central
/// differences (step 1e-6, SPEC §5.3). Test code only; it allocates freely. Rotor inertia is not included
/// (Jm = 0), as in the reference vectors.
/// </summary>
public sealed class ClosedFormArm
{
    /// <summary>Central-difference step for ∂M/∂q. SPEC §5.3.</summary>
    public const double DerivativeStep = 1e-6;

    private readonly BodyParameters[] _b;

    /// <summary>Closed forms over the four bodies of <paramref name="model"/> (with or without payload).</summary>
    public ClosedFormArm(ArmModel model)
    {
        if (model.DegreesOfFreedom != 4)
            throw new ArgumentException("The closed forms of SPEC §5.3 are for the 4-DOF arm.", nameof(model));
        _b = model.Bodies.ToArray();
    }

    /// <summary>M(q) by SPEC §5.3: m11, the pitch block, and M1k = 0.</summary>
    public double[,] InertiaMatrix(ReadOnlySpan<double> q)
    {
        double[] theta = [0.0, q[1], q[1] + q[2], q[1] + q[2] + q[3]];
        var (L2, L3, beta) = (G.L2, G.L3, G.Beta);

        double[] rho =
        [
            _b[0].Ell * Math.Cos(_b[0].Eps),
            _b[1].Ell * Math.Cos(theta[1] + _b[1].Eps),
            L2 * Math.Cos(theta[1] - beta) + _b[2].Ell * Math.Cos(theta[2] + _b[2].Eps),
            L2 * Math.Cos(theta[1] - beta) + L3 * Math.Cos(theta[2]) + _b[3].Ell * Math.Cos(theta[3] + _b[3].Eps),
        ];
        var m11 = 0.0;
        for (var i = 0; i < 4; i++)
        {
            var s = Math.Sin(theta[i]);
            var c = Math.Cos(theta[i]);
            m11 += _b[i].Mass * rho[i] * rho[i] + _b[i].Ixx * s * s + _b[i].Izz * c * c - 2.0 * _b[i].Ixz * s * c;
        }

        var (q3, q4) = (q[2], q[3]);
        var (J2, J3, J4) = (_b[1].Iyy, _b[2].Iyy, _b[3].Iyy);
        var (m2, m3, m4) = (_b[1].Mass, _b[2].Mass, _b[3].Mass);
        var (l2, l3, l4) = (_b[1].Ell, _b[2].Ell, _b[3].Ell);
        var (e3, e4) = (_b[2].Eps, _b[3].Eps);

        var h = L2 * l3 * Math.Cos(q3 + beta + e3);
        var k23 = L2 * L3 * Math.Cos(q3 + beta);
        var k24 = L2 * l4 * Math.Cos(q3 + q4 + beta + e4);
        var k34 = L3 * l4 * Math.Cos(q4 + e4);

        var M44 = J4 + m4 * l4 * l4;
        var M34 = J4 + m4 * (l4 * l4 + k34);
        var M33 = J3 + J4 + m3 * l3 * l3 + m4 * (L3 * L3 + l4 * l4 + 2 * k34);
        var M24 = J4 + m4 * (l4 * l4 + k34 + k24);
        var M23 = J3 + J4 + m3 * (l3 * l3 + h) + m4 * (L3 * L3 + l4 * l4 + 2 * k34 + k23 + k24);
        var M22 = J2 + J3 + J4 + m2 * l2 * l2 + m3 * (L2 * L2 + l3 * l3 + 2 * h)
                  + m4 * (L2 * L2 + L3 * L3 + l4 * l4 + 2 * k23 + 2 * k24 + 2 * k34);

        return new double[,]
        {
            { m11, 0.0, 0.0, 0.0 },
            { 0.0, M22, M23, M24 },
            { 0.0, M23, M33, M34 },
            { 0.0, M24, M34, M44 },
        };
    }

    /// <summary>Gravity torques g(q) by SPEC §5.3 for gravity of magnitude <paramref name="g"/> along −z.</summary>
    public double[] Gravity(ReadOnlySpan<double> q, double g)
    {
        var (t2, t3, t4) = (q[1], q[1] + q[2], q[1] + q[2] + q[3]);
        var (m2, m3, m4) = (_b[1].Mass, _b[2].Mass, _b[3].Mass);
        var (l2, l3, l4) = (_b[1].Ell, _b[2].Ell, _b[3].Ell);
        var (e2, e3, e4) = (_b[1].Eps, _b[2].Eps, _b[3].Eps);

        var g4 = -g * m4 * l4 * Math.Cos(t4 + e4);
        var g3 = g4 - g * (m3 * l3 * Math.Cos(t3 + e3) + m4 * G.L3 * Math.Cos(t3));
        var g2 = g3 - g * (m2 * l2 * Math.Cos(t2 + e2) + (m3 + m4) * G.L2 * Math.Cos(t2 - G.Beta));
        return [0.0, g2, g3, g4];
    }

    /// <summary>∂M/∂q_i for i = 0..3 by central differences of the closed-form M, step 1e-6. SPEC §5.3.</summary>
    public double[][,] InertiaDerivatives(ReadOnlySpan<double> q) =>
        CentralDifferences(q, x => InertiaMatrix(x));

    /// <summary>
    /// ∂M/∂q_i of any M(q) by central differences with step 1e-6 (SPEC §5.3), for i = 0..3.
    /// </summary>
    public static double[][,] CentralDifferences(ReadOnlySpan<double> q, Func<double[], double[,]> inertia)
    {
        var result = new double[4][,];
        for (var i = 0; i < 4; i++)
        {
            var plus = q.ToArray();
            var minus = q.ToArray();
            plus[i] += DerivativeStep;
            minus[i] -= DerivativeStep;
            var mp = inertia(plus);
            var mm = inertia(minus);
            var d = new double[4, 4];
            for (var r = 0; r < 4; r++)
                for (var c = 0; c < 4; c++)
                    d[r, c] = (mp[r, c] - mm[r, c]) / (2.0 * DerivativeStep);
            result[i] = d;
        }
        return result;
    }

    /// <summary>
    /// Coriolis matrix from the Christoffel symbols, SPEC §5.3 [eq. 13]:
    /// C_kj = Σi c_ijk q̇i, c_ijk = ½ (∂M_kj/∂q_i + ∂M_ki/∂q_j − ∂M_ij/∂q_k).
    /// </summary>
    public double[,] Coriolis(ReadOnlySpan<double> q, ReadOnlySpan<double> qd)
    {
        var dM = InertiaDerivatives(q);
        var c = new double[4, 4];
        for (var k = 0; k < 4; k++)
            for (var j = 0; j < 4; j++)
            {
                var s = 0.0;
                for (var i = 0; i < 4; i++)
                    s += 0.5 * (dM[i][k, j] + dM[j][k, i] - dM[k][i, j]) * qd[i];
                c[k, j] = s;
            }
        return c;
    }
}
