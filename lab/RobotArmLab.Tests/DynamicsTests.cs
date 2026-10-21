using System.Text.Json;
using RobotArmLab.Core.Dynamics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Numerics;
using OldPlanet = RobotArm.Core.Model.Planet;

namespace RobotArmLab.Tests;

public class DynamicsTests
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

    [Fact]
    public void MassMatrixAndGravityMatchTestVectors()
    {
        foreach (JsonElement pose in TestVectors.Root.GetProperty("poses").EnumerateArray())
        {
            double payload = pose.GetProperty("payload_kg").GetDouble();
            ArmDynamics dyn = new ArmDynamics(new Payload(payload).ApplyTo(ArmParameters.Default()), 9.81);
            double[] q = TestVectors.Vector(pose.GetProperty("q"));
            Check.RelativeClose(TestVectors.Matrix(pose.GetProperty("M")), dyn.MassMatrix(q), 1e-9, "M");
            Check.RelativeClose(TestVectors.Vector(pose.GetProperty("gravity_earth")), dyn.GravityVector(q), 1e-9, "g");
        }
    }

    [Fact]
    public void MassMatrixIsSymmetricPositiveDefinite()
    {
        ArmDynamics dyn = new ArmDynamics(ArmParameters.Default(), 9.81);
        Random rng = new Random(2);
        for (int n = 0; n < 200; n++)
        {
            double[,] m = dyn.MassMatrix(Check.Uniform(rng, Math.PI));
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    Assert.Equal(m[i, j], m[j, i]);
            double[,] l = Small4x4.Cholesky(m);   // throws if not positive definite
            for (int i = 0; i < 4; i++)
                Assert.True(l[i, i] > 0.0);
        }
    }

    [Fact]
    public void GravityIsGradientOfPotentialEnergy()
    {
        ArmDynamics dyn = new ArmDynamics(new Payload(0.2).ApplyTo(ArmParameters.Default()), 9.81);
        Random rng = new Random(3);
        for (int n = 0; n < 100; n++)
        {
            double[] q = Check.RandomQInLimits(rng, ArmParameters.Default());
            double[] g = dyn.GravityVector(q);
            double[] numeric = new double[4];
            for (int k = 0; k < 4; k++)
            {
                double[] qp = (double[])q.Clone(), qm = (double[])q.Clone();
                qp[k] += 1e-6; qm[k] -= 1e-6;
                numeric[k] = (dyn.PotentialEnergy(qp) - dyn.PotentialEnergy(qm)) / 2e-6;
            }
            for (int k = 0; k < 4; k++)
                Assert.Equal(numeric[k], g[k], 1e-8);
        }
    }

    [Fact]
    public void AnalyticMassMatrixDerivativeMatchesCentralDifferences()
    {
        ArmParameters p = ArmParameters.Default();
        Random rng = new Random(4);
        for (int n = 0; n < 50; n++)
        {
            double[] q = Check.RandomQInLimits(rng, p);
            double[,,] dM = MassMatrixFormulas.Derivative(p, q);
            for (int k = 0; k < 4; k++)
            {
                double[] qp = (double[])q.Clone(), qm = (double[])q.Clone();
                qp[k] += 1e-6; qm[k] -= 1e-6;
                double[,] mp = MassMatrixFormulas.Compute(p, qp), mm = MassMatrixFormulas.Compute(p, qm);
                for (int i = 0; i < 4; i++)
                    for (int j = 0; j < 4; j++)
                        Assert.Equal((mp[i, j] - mm[i, j]) / 2e-6, dM[k, i, j], 1e-10);
            }
        }
    }

    [Fact]
    public void PotentialEnergyAtFreeMotionStartMatchesTestVectors()
    {
        JsonElement fm = TestVectors.Root.GetProperty("free_motion");
        double[] x0 = TestVectors.Vector(fm.GetProperty("x0"));
        ArmDynamics dyn = new ArmDynamics(ArmParameters.Default(), 9.81);
        double[] q = x0[..4], qd = x0[4..];
        double e = dyn.KineticEnergy(q, qd) + dyn.PotentialEnergy(q);
        Assert.Equal(fm.GetProperty("energy0").GetDouble(), e, 1e-12);
    }
}
