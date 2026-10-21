using RobotArmLab.Core.Dynamics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Solvers;

namespace RobotArmLab.Tests;

public class EnergyTests
{
    // Free motion: zero torque and zero friction, Earth, from the SPEC test state (test_vectors free_motion.x0).
    private static double MaxEnergyDrift(IOdeSolver solver, double h, double T)
    {
        ArmParameters p = ArmParameters.Default();
        p.ViscousFrictionNmsPerRad = 0.0;
        p.CoulombFrictionNm = 0.0;
        ArmDynamics dyn = new ArmDynamics(p, 9.81);
        double[] zero = new double[4];
        OdeFunction f = delegate (double t, double[] x)
        {
            double[] qdd = dyn.Acceleration(x[..4], x[4..], zero);
            return new[] { x[4], x[5], x[6], x[7], qdd[0], qdd[1], qdd[2], qdd[3] };
        };

        double[] state = TestVectors.Vector(TestVectors.Root.GetProperty("free_motion").GetProperty("x0"));
        double e0 = Energy(dyn, state);
        double maxDrift = 0.0;
        int n = (int)Math.Round(T / h);
        for (int k = 0; k < n; k++)
        {
            state = solver.Step(f, k * h, state, h);
            maxDrift = Math.Max(maxDrift, Math.Abs(Energy(dyn, state) - e0));
        }
        return maxDrift;
    }

    private static double Energy(ArmDynamics dyn, double[] x)
    {
        return dyn.KineticEnergy(x[..4], x[4..]) + dyn.PotentialEnergy(x[..4]);
    }

    [Fact]
    public void Rk4ConservesEnergyAndEulerDoesNot()
    {
        double rk4 = MaxEnergyDrift(new Rk4Solver(), 0.0005, 2.0);
        double euler = MaxEnergyDrift(new EulerSolver(), 0.0005, 2.0);
        Assert.True(rk4 < 1e-5, $"RK4 drift {rk4:E3} J");
        Assert.True(euler > 1e-3, $"Euler drift {euler:E3} J");
        Assert.True(euler > 1000 * rk4, $"Euler {euler:E3} J vs RK4 {rk4:E3} J");
    }
}
