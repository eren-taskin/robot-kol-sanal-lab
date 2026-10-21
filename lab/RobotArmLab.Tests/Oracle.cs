using OldArmModel = RobotArm.Core.Model.ArmModel;
using OldPlanet = RobotArm.Core.Model.Planet;
using OldRnea = RobotArm.Core.Dynamics.RecursiveNewtonEuler;
using OldPlant = RobotArm.Core.Simulation.ArmPlant;

namespace RobotArmLab.Tests;

/// <summary>
/// The old RobotArm.Core (recursive Newton–Euler, general chain) used only as a test oracle.
/// The lab core never references it.
/// </summary>
internal sealed class Oracle
{
    private readonly OldRnea _rnea;
    private readonly OldPlant _plant;
    private readonly OldPlanet _planet;

    public Oracle(double payloadKg, OldPlanet planet)
    {
        OldArmModel model = OldArmModel.OpenManipulatorX().WithPayload(payloadKg);
        _rnea = new OldRnea(model);
        _planet = planet;
        _plant = new OldPlant(OldArmModel.OpenManipulatorX(), planet);
        _plant.SetPayload(payloadKg);
    }

    public double[,] M(double[] q)
    {
        double[] flat = new double[16];
        _rnea.InertiaMatrix(q, flat);
        double[,] m = new double[4, 4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m[i, j] = flat[4 * i + j];
        return m;
    }

    /// <summary>h(q, q̇) = C(q, q̇) q̇ + g(q).</summary>
    public double[] Bias(double[] q, double[] qd)
    {
        double[] h = new double[4];
        _rnea.BiasTorque(q, qd, _planet, h);
        return h;
    }

    /// <summary>q̈ with the default friction of SPEC §3.3 (no saturation for |τ| &lt; 4.1 N·m).</summary>
    public double[] Qdd(double[] q, double[] qd, double[] tau)
    {
        double[] x = new double[8];
        double[] dx = new double[8];
        for (int i = 0; i < 4; i++) { x[i] = q[i]; x[4 + i] = qd[i]; }
        _plant.Derivatives(0.0, x, tau, dx);
        return dx[4..];
    }
}
