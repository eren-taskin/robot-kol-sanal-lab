using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using static RobotArm.Tests.DynamicsTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §5.1–§5.2: inverse dynamics by RNEA is consistent with M and h, τ = M q̈ + h, and the API checks
/// its arguments.
/// </summary>
public sealed class RecursiveNewtonEulerTests
{
    private static readonly ArmModel s_arm = ArmModel.OpenManipulatorX();

    [Theory]
    [InlineData(0.0, 0.0)]
    [InlineData(0.5, 0.0)]
    [InlineData(0.0, 2.0e-7)]
    public void Inverse_EqualsInertiaTimesAccelerationPlusBias(double payloadKg, double jm)
    {
        var rnea = new RecursiveNewtonEuler(WithRotorInertia(s_arm.WithPayload(payloadKg), jm));
        var random = new Random(11);
        var tau = new double[4];
        var h = new double[4];
        var m = new double[16];

        for (var trial = 0; trial < 200; trial++)
        {
            var q = RandomVector(random, 4, Math.PI);
            var qd = RandomVector(random, 4, 3.0);
            var qdd = RandomVector(random, 4, 20.0);

            rnea.Inverse(q, qd, qdd, Planet.HighG.GravityVector, tau);
            rnea.BiasTorque(q, qd, Planet.HighG, h);
            rnea.InertiaMatrix(q, m);

            var expected = new double[4];
            for (var r = 0; r < 4; r++)
            {
                expected[r] = h[r];
                for (var c = 0; c < 4; c++)
                    expected[r] += m[r * 4 + c] * qdd[c];
            }
            var diff = MaxAbsDifference(expected, tau);
            Assert.True(diff <= 1e-12 * MaxAbs(expected), $"difference {diff:E3}");
        }
    }

    [Fact]
    public void ForwardDynamics_ByCholesky_RecoversAcceleration()
    {
        var rnea = new RecursiveNewtonEuler(s_arm);
        double[] q = [0.2, -0.4, 0.9, -0.3];
        double[] qd = [0.5, -1.0, 0.7, 2.0];
        double[] qdd = [1.5, -3.0, 4.0, -2.5];
        var tau = new double[4];
        var rhs = new double[4];
        var m = new double[16];

        rnea.Inverse(q, qd, qdd, Planet.Earth.GravityVector, tau);
        rnea.BiasTorque(q, qd, Planet.Earth, rhs);
        for (var i = 0; i < 4; i++) rhs[i] = tau[i] - rhs[i];
        rnea.InertiaMatrix(q, m);
        Cholesky.FactorAndSolve(m, 4, rhs);

        for (var i = 0; i < 4; i++)
            Assert.Equal(qdd[i], rhs[i], 1e-10);
    }

    [Fact]
    public void Constructor_NullModel_Throws()
    {
        Assert.Throws<ArgumentNullException>(() => new RecursiveNewtonEuler(null!));
    }

    [Fact]
    public void WrongSpanLengths_Throw()
    {
        var rnea = new RecursiveNewtonEuler(s_arm);
        Assert.Throws<ArgumentException>(() => rnea.Inverse(new double[3], new double[4], new double[4], Vector3d.Zero, new double[4]));
        Assert.Throws<ArgumentException>(() => rnea.Inverse(new double[4], new double[4], new double[4], Vector3d.Zero, new double[5]));
        Assert.Throws<ArgumentException>(() => rnea.BiasTorque(new double[4], new double[2], Vector3d.Zero, new double[4]));
        Assert.Throws<ArgumentException>(() => rnea.InertiaMatrix(new double[4], new double[15]));
    }

    [Fact]
    public void Model_AndDegreesOfFreedom_AreExposed()
    {
        var rnea = new RecursiveNewtonEuler(s_arm);
        Assert.Same(s_arm, rnea.Model);
        Assert.Equal(4, rnea.DegreesOfFreedom);
    }
}
