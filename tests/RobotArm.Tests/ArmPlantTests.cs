using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §5.4 and §6.1 (V15 of SPEC §11): the plant applies saturation and friction and solves
/// M q̈ = τ − τf − h; the saturation flags are reported and are the same for every RK4 stage.
/// </summary>
public sealed class ArmPlantTests(ITestOutputHelper output)
{
    private static readonly ArmModel s_arm = ArmModel.OpenManipulatorX();

    [Fact]
    public void Sizes_AreEightStatesAndFourInputs()
    {
        var plant = new ArmPlant(s_arm, Planet.Earth);
        Assert.Equal(8, plant.StateCount);
        Assert.Equal(4, plant.InputCount);
    }

    [Fact]
    public void V15_Commanded10Nm_IsAppliedAs41Nm_AndFlagged()
    {
        var plant = new ArmPlant(s_arm, Planet.Earth);
        double[] x = [0.0, 0.2, 0.3, -0.1, 0.0, 0.0, 0.0, 0.0];
        double[] u = [10.0, -10.0, 4.1, 2.0];
        var dx = new double[8];

        plant.Derivatives(0.0, x, u, dx);

        Assert.Equal([4.1, -4.1, 4.1, 2.0], plant.AppliedTorque.ToArray());
        Assert.Equal([true, true, false, false], plant.Saturated.ToArray());

        // The acceleration is the one produced by 4.1 N·m, not 10 N·m.
        var dxClamped = new double[8];
        new ArmPlant(s_arm, Planet.Earth).Derivatives(0.0, x, [4.1, -4.1, 4.1, 2.0], dxClamped);
        Assert.Equal(dxClamped, dx);
    }

    [Fact]
    public void V15_Simulation_ReportsSaturationAfterAdvance()
    {
        var plant = new ArmPlant(s_arm, Planet.Earth);
        var sim = new Simulation(plant, new Rk4Solver(8), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

        sim.Advance([10.0, 0.0, 0.0, -0.5], 1e-3);
        Assert.Equal([true, false, false, false], sim.Saturated.ToArray());
        Assert.Equal([4.1, 0.0, 0.0, -0.5], sim.AppliedTorque.ToArray());

        sim.Advance([0.0, 0.0, 0.0, 0.0], 1e-3);
        Assert.Equal([false, false, false, false], sim.Saturated.ToArray());
    }

    [Fact]
    public void SaturationFlags_AreTheSameInAllFourRk4Stages()
    {
        // The input is held over the step (SPEC §6.2), so every stage sees the same u and the same flags.
        var recorder = new RecordingPlant(new ArmPlant(s_arm, Planet.HighG));
        var solver = new Rk4Solver(8);
        double[] x = [0.4, 0.5, -0.3, 0.6, 1.0, -2.0, 3.0, 0.5];
        double[] u = [5.0, -1.0, -7.5, 4.0];

        solver.Step(recorder, 0.0, x, u, 2e-3);

        Assert.Equal(4, recorder.Flags.Count);
        foreach (var stage in recorder.Flags)
            Assert.Equal([true, false, true, false], stage);
        Assert.Equal([true, false, true, false], recorder.Inner.Saturated.ToArray());
    }

    [Fact]
    public void Derivatives_WithFriction_MatchClosedFormForwardDynamics()
    {
        // SPEC §6.1: q̈ = M⁻¹(τ − τf − C q̇ − g), here with the closed forms of SPEC §5.3 and a plain
        // Gaussian elimination, independent of RNEA and Cholesky.
        var arm = s_arm.WithPayload(0.3);
        var planet = Planet.Mars;
        var plant = new ArmPlant(s_arm, planet);
        plant.SetPayload(0.3);
        var closed = new ClosedFormArm(arm);
        var a = arm.Actuator;
        var random = new Random(11);
        var dx = new double[8];
        var worst = 0.0;

        for (var trial = 0; trial < 200; trial++)
        {
            var q = DynamicsTestHelpers.RandomVector(random, 4, 1.4);
            var qd = DynamicsTestHelpers.RandomVector(random, 4, 2.0);
            qd[trial % 4] = 0.003 * (2.0 * random.NextDouble() - 1.0); // inside the tanh smoothing band
            var u = DynamicsTestHelpers.RandomVector(random, 4, 6.0);
            double[] x = [.. q, .. qd];

            plant.Derivatives(0.0, x, u, dx);

            var m = closed.InertiaMatrix(q);
            var c = closed.Coriolis(q, qd);
            var g = closed.Gravity(q, planet.G);
            var rhs = new double[4];
            for (var i = 0; i < 4; i++)
            {
                var tau = Math.Clamp(u[i], -a.StallTorque, a.StallTorque);
                var tauF = a.ViscousFriction * qd[i] + a.CoulombFriction * Math.Tanh(qd[i] / a.SmoothingSpeed);
                var cqd = 0.0;
                for (var j = 0; j < 4; j++) cqd += c[i, j] * qd[j];
                rhs[i] = tau - tauF - cqd - g[i];
            }
            var qdd = GaussSolve(m, rhs);

            Assert.Equal(qd, dx[..4]);
            var scale = Math.Max(1.0, DynamicsTestHelpers.MaxAbs(qdd));
            worst = Math.Max(worst, DynamicsTestHelpers.MaxAbsDifference(dx.AsSpan(4), qdd) / scale);
        }

        output.WriteLine(Invariant($"Plant q̈ vs closed form with friction, Mars, 0.3 kg, 200 states: max relative difference {worst:E3}"));
        Assert.True(worst < 1e-6, Invariant($"{worst:E3}")); // C uses central differences of M (step 1e-6)
    }

    [Fact]
    public void Derivatives_RejectWrongLengths()
    {
        var plant = new ArmPlant(s_arm, Planet.Earth);
        Assert.Throws<ArgumentException>(() => plant.Derivatives(0, new double[7], new double[4], new double[8]));
        Assert.Throws<ArgumentException>(() => plant.Derivatives(0, new double[8], new double[3], new double[8]));
        Assert.Throws<ArgumentException>(() => plant.Derivatives(0, new double[8], new double[4], new double[9]));
    }

    /// <summary>Solves a x = b by Gaussian elimination with partial pivoting; independent of Core.</summary>
    internal static double[] GaussSolve(double[,] a, double[] b)
    {
        var n = b.Length;
        var m = (double[,])a.Clone();
        var x = (double[])b.Clone();
        for (var k = 0; k < n; k++)
        {
            var p = k;
            for (var i = k + 1; i < n; i++)
                if (Math.Abs(m[i, k]) > Math.Abs(m[p, k])) p = i;
            for (var j = 0; j < n; j++) (m[k, j], m[p, j]) = (m[p, j], m[k, j]);
            (x[k], x[p]) = (x[p], x[k]);
            for (var i = k + 1; i < n; i++)
            {
                var f = m[i, k] / m[k, k];
                for (var j = k; j < n; j++) m[i, j] -= f * m[k, j];
                x[i] -= f * x[k];
            }
        }
        for (var i = n - 1; i >= 0; i--)
        {
            var s = x[i];
            for (var j = i + 1; j < n; j++) s -= m[i, j] * x[j];
            x[i] = s / m[i, i];
        }
        return x;
    }

    /// <summary>Passes calls to an <see cref="ArmPlant"/> and records its saturation flags after each one.</summary>
    private sealed class RecordingPlant(ArmPlant inner) : IPlant
    {
        public ArmPlant Inner { get; } = inner;
        public List<bool[]> Flags { get; } = [];
        public int StateCount => Inner.StateCount;
        public int InputCount => Inner.InputCount;

        public void Derivatives(double t, ReadOnlySpan<double> x, ReadOnlySpan<double> u, Span<double> dx)
        {
            Inner.Derivatives(t, x, u, dx);
            Flags.Add(Inner.Saturated.ToArray());
        }
    }
}
