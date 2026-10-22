using RobotArmLab.Core.Dynamics;
using RobotArmLab.Core.Model;
using OldPlanet = RobotArm.Core.Model.Planet;

namespace RobotArmLab.Tests;

/// <summary>Lab dynamics (closed-form, report eq. (11)–(21)) against the old core's recursive Newton–Euler.</summary>
public class OracleDynamicsTests
{
    public static TheoryData<double, string> Cases()
    {
        TheoryData<double, string> data = new TheoryData<double, string>();
        data.Add(0.0, "Earth");
        data.Add(0.0, "Moon");
        data.Add(0.5, "Earth");
        data.Add(0.5, "Moon");
        data.Add(0.3, "Earth");
        return data;
    }

    [Theory]
    [MemberData(nameof(Cases))]
    public void MassBiasAndAccelerationMatchOldCoreOn200RandomStates(double payloadKg, string planet)
    {
        OldPlanet oldPlanet = planet == "Earth" ? OldPlanet.Earth : OldPlanet.Moon;
        GravityEnvironment env = planet == "Earth" ? GravityEnvironment.Earth() : GravityEnvironment.Moon();
        ArmParameters p = new Payload(payloadKg).ApplyTo(ArmParameters.Default());
        ArmDynamics dyn = new ArmDynamics(p, env.GravityMps2);
        Oracle oracle = new Oracle(payloadKg, oldPlanet);

        Random rng = new Random(445);
        for (int n = 0; n < 200; n++)
        {
            double[] q = Check.RandomQInLimits(rng, ArmParameters.Default());
            double[] qd = Check.Uniform(rng, 3.0);
            double[] tau = Check.Uniform(rng, 4.0);

            Check.RelativeClose(oracle.M(q), dyn.MassMatrix(q), 1e-9, "M");

            double[] c = dyn.CoriolisVector(q, qd);
            double[] g = dyn.GravityVector(q);
            double[] cg = new double[4];
            for (int i = 0; i < 4; i++) cg[i] = c[i] + g[i];
            Check.RelativeClose(oracle.Bias(q, qd), cg, 1e-9, "c + g");

            Check.RelativeClose(oracle.Qdd(q, qd, tau), dyn.Acceleration(q, qd, tau), 1e-9, "qdd");
        }
    }
}
