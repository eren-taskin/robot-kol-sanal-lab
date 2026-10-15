using System.Numerics;
using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using RobotArm.Core.Model;
using RobotArm.Core.Tasks;

namespace RobotArm.Tests;

/// <summary>
/// 1-DOF mass–damper–spring m ẍ + c ẋ + k x = u under the sampled PD law u = −s (kp x + kd ẋ) with a common scale
/// s, with everything in closed form (SPEC §12.3–§12.4 tests). Same numbers as the T07 tests of SPEC §6.3.
/// </summary>
internal static class OneDof
{
    public const double Mass = 0.5;
    public const double Damping = 0.3;
    public const double Spring = 2.0;
    public const double Kp = 10.0;
    public const double Kd = 0.5;
    public const double Ts = 0.05;

    public static LinearSystem Plant() => new(2, 1, [0.0, 1.0, -Spring / Mass, -Damping / Mass], [0.0, 1.0 / Mass]);

    /// <summary>
    /// Φ = e^{AT} for the underdamped case, e^{AT} = e^{σT}[cos(ωT) I + sin(ωT)/ω (A − σI)], and Γ = A⁻¹(Φ − I)B.
    /// </summary>
    public static (double[] Phi, double[] Gamma) Discretisation()
    {
        var sigma = -Damping / (2.0 * Mass);
        var omega = Math.Sqrt(Spring / Mass - sigma * sigma);
        double[] a = [0.0, 1.0, -Spring / Mass, -Damping / Mass];
        var e = Math.Exp(sigma * Ts);
        var cos = Math.Cos(omega * Ts);
        var sin = Math.Sin(omega * Ts) / omega;
        var phi = new double[4];
        for (var r = 0; r < 2; r++)
            for (var c = 0; c < 2; c++)
                phi[r * 2 + c] = e * ((r == c ? cos : 0.0) + sin * (a[r * 2 + c] - (r == c ? sigma : 0.0)));
        var det = Spring / Mass;
        double[] v = [phi[1] / Mass, (phi[3] - 1.0) / Mass];
        double[] gamma = [(a[3] * v[0] - a[1] * v[1]) / det, (-a[2] * v[0] + a[0] * v[1]) / det];
        return (phi, gamma);
    }

    /// <summary>Trace and determinant of Φ − Γ s [kp kd]; both are affine in s.</summary>
    public static (double Trace, double Det) Characteristic(double s)
    {
        var (phi, gamma) = Discretisation();
        double[] cl =
        [
            phi[0] - gamma[0] * s * Kp, phi[1] - gamma[0] * s * Kd,
            phi[2] - gamma[1] * s * Kp, phi[3] - gamma[1] * s * Kd,
        ];
        return (cl[0] + cl[3], cl[0] * cl[3] - cl[1] * cl[2]);
    }

    /// <summary>Roots of z² − tr z + det = 0.</summary>
    public static Complex[] Poles(double s)
    {
        var (tr, det) = Characteristic(s);
        var disc = Complex.Sqrt(tr * tr - 4.0 * det);
        return [(tr + disc) / 2.0, (tr - disc) / 2.0];
    }

    /// <summary>
    /// The smallest s &gt; 0 at which the Jury conditions |det| &lt; 1, 1 ± tr + det &gt; 0 first fail. Each is affine in
    /// s, so each boundary is one linear equation; the stable set is an interval and its upper end is the answer.
    /// </summary>
    public static double CriticalScale()
    {
        var (tr0, det0) = Characteristic(0.0);
        var (tr1, det1) = Characteristic(1.0);
        var dTr = tr1 - tr0;
        var dDet = det1 - det0;
        var candidates = new List<double>
        {
            (1.0 - det0) / dDet,                          // det = 1 (complex pair on the circle)
            (-1.0 - det0) / dDet,                         // det = −1
            -(1.0 + tr0 + det0) / (dTr + dDet),           // z = −1
            -(1.0 - tr0 + det0) / (dDet - dTr),           // z = +1
        };
        static bool Stable(double s)
        {
            var (tr, det) = Characteristic(s);
            return Math.Abs(det) < 1.0 && 1.0 - tr + det > 0.0 && 1.0 + tr + det > 0.0;
        }
        return candidates.Where(s => s > 0.0 && double.IsFinite(s) && Stable(s * (1.0 - 1e-9))).Min();
    }

    /// <summary>The sampled loop with all gains scaled by s.</summary>
    public static SampledLoopModel Model(double s = 1.0) => SampledLoopModel.Create(Plant(), Ts, [s * Kp, s * Kd]);

    /// <summary>L(z) = [kp kd] adj(zI − Φ) Γ / det(zI − Φ), the analytic discrete loop.</summary>
    public static Complex Loop(double frequency)
    {
        var (phi, gamma) = Discretisation();
        var z = Complex.FromPolarCoordinates(1.0, frequency * Ts);
        Complex m00 = z - phi[0], m01 = -phi[1], m10 = -phi[2], m11 = z - phi[3];
        var det = m00 * m11 - m01 * m10;
        // (zI − Φ)⁻¹ Γ = adj · Γ / det, adj = [[m11, −m01], [−m10, m00]].
        var x0 = (m11 * gamma[0] - m01 * gamma[1]) / det;
        var x1 = (-m10 * gamma[0] + m00 * gamma[1]) / det;
        return Kp * x0 + Kd * x1;
    }
}

/// <summary>Shared set-up for the linear-analysis tests of SPEC §12.</summary>
internal static class AnalysisFixture
{
    /// <summary>The first library scenario: "Dünya – boş kol – noktaya git" (PDG, Earth, no payload).</summary>
    public static Scenario EarthReach => ScenarioLibrary.All[0];

    public static double[] StartPose => EarthReach.Task.Start.ToArray();

    public static PdGravity DefaultPdg(Planet? planet = null, double payload = 0.0) =>
        new(ControllerBelief.OpenManipulatorX(planet ?? Planet.Earth, payload), PdGravity.DefaultGains);

    public static ComputedTorque CtPlusI(Planet? planet = null, double payload = 0.0) =>
        new(ControllerBelief.OpenManipulatorX(planet ?? Planet.Earth, payload), ComputedTorque.DefaultIntegralPdGains,
            integralGains: ComputedTorque.DefaultIntegralGains);
}
