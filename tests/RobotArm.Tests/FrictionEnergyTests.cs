using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.SimulationTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §5.4: friction τf,i = fv q̇i + fc tanh(q̇i / ωs) in the plant. With τ = 0 the only non-conservative
/// torque is friction, so dE/dt = −q̇ᵀ τf(q̇) ≤ 0. V13 cannot see friction (the held arm never moves), this
/// test can: a plant with friction missing or of the wrong sign fails it.
/// </summary>
public sealed class FrictionEnergyTests(ITestOutputHelper output)
{
    [Fact]
    public void Friction_WithZeroTorque_EnergyDecreases_AtTheRateQdTransposeTauF()
    {
        const double h = 5e-4;
        var arm = ArmModel.OpenManipulatorX(); // default friction of SPEC §3.3, Jm = 0
        var x0 = TestVectors.Instance.FreeMotion.X0;
        var g = Planet.Earth.G;
        var sim = new Simulation(new ArmPlant(arm, Planet.Earth), new Rk4Solver(8), x0, FaultChecks.None);
        var rnea = new RecursiveNewtonEuler(arm);
        var u = new double[4];
        var steps = Steps(0.4, h);

        var energy = new double[steps + 1];
        var power = new double[steps + 1]; // dE/dt = −q̇ᵀ τf at each step boundary
        energy[0] = Energy(arm, rnea, g, sim.State);
        power[0] = FrictionPower(arm, sim.QDot);
        var worstRise = double.NegativeInfinity;
        for (var k = 1; k <= steps; k++)
        {
            sim.Advance(u, h);
            energy[k] = Energy(arm, rnea, g, sim.State);
            power[k] = FrictionPower(arm, sim.QDot);
            worstRise = Math.Max(worstRise, energy[k] - energy[k - 1]);
        }

        // E(t) − E(0) against ∫ −q̇ᵀ τf dt by Simpson's rule over pairs of steps (fourth order, like RK4).
        var integral = 0.0;
        var worstMismatch = 0.0;
        for (var k = 2; k <= steps; k += 2)
        {
            integral += h / 3.0 * (power[k - 2] + 4.0 * power[k - 1] + power[k]);
            worstMismatch = Math.Max(worstMismatch, Math.Abs(energy[k] - energy[0] - integral));
        }

        var dissipated = energy[0] - energy[steps];
        output.WriteLine(Invariant($"Friction on, τ = 0, 0.4 s at h = 0.5 ms: E0 = {energy[0]:F6} J, dissipated {dissipated:E4} J"));
        output.WriteLine(Invariant($"  largest one-step rise of E: {worstRise:E3} J; max |ΔE − ∫ −q̇ᵀτf dt| = {worstMismatch:E3} J ({worstMismatch / dissipated:E2} of the dissipated energy)"));

        Assert.True(dissipated > 1e-3, Invariant($"friction removed only {dissipated:E3} J"));
        Assert.True(worstRise < 1e-9, Invariant($"energy rose by {worstRise:E3} J in one step"));
        Assert.True(worstMismatch < 1e-6 * dissipated, Invariant($"energy balance off by {worstMismatch:E3} J"));
    }

    /// <summary>dE/dt from friction: −Σ q̇i τf,i(q̇i), with each joint's own actuator (SPEC §5.4).</summary>
    private static double FrictionPower(ArmModel arm, ReadOnlySpan<double> qd)
    {
        var p = 0.0;
        for (var i = 0; i < qd.Length; i++)
        {
            var a = arm.Actuators[i];
            p -= qd[i] * (a.ViscousFriction * qd[i] + a.CoulombFriction * Math.Tanh(qd[i] / a.SmoothingSpeed));
        }
        return p;
    }
}
