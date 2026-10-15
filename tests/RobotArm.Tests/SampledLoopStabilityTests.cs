using RobotArm.Core.Control;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.ControlTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §6.3 and §7, docs/DECISIONS.md D6: stability of the sampled closed loop, ρ(Φ − ΓK) &lt; 1, with Φ and Γ
/// from RK4 sub-steps and ρ by repeated squaring (<see cref="SampledLoopStability"/>).
/// </summary>
public sealed class SampledLoopStabilityTests(ITestOutputHelper output)
{
    // 1-DOF mass–damper–spring m ẍ + c ẋ + k x = u under u = −kp x − kd ẋ sampled with Ts.
    private const double Mass = 0.5;
    private const double Damping = 0.3;
    private const double Spring = 2.0;
    private const double Kp = 10.0;
    private const double Kd = 0.5;
    private const double Ts = 0.05;

    private static LinearSystem MassDamperSpring() =>
        new(2, 1, [0.0, 1.0, -Spring / Mass, -Damping / Mass], [0.0, 1.0 / Mass]);

    /// <summary>
    /// Φ = e^{AT} in closed form for the underdamped 2 × 2 case, A = [[0, 1], [−k/m, −c/m]] with eigenvalues
    /// σ ± iω: e^{AT} = e^{σT}[cos(ωT) I + sin(ωT)/ω (A − σI)]; Γ = A⁻¹(Φ − I)B.
    /// </summary>
    private static (double[,] Phi, double[] Gamma) ClosedFormDiscretisation()
    {
        var sigma = -Damping / (2.0 * Mass);
        var omega = Math.Sqrt(Spring / Mass - sigma * sigma);
        double[,] a = { { 0.0, 1.0 }, { -Spring / Mass, -Damping / Mass } };
        var e = Math.Exp(sigma * Ts);
        var cos = Math.Cos(omega * Ts);
        var sin = Math.Sin(omega * Ts) / omega;
        var phi = new double[2, 2];
        for (var r = 0; r < 2; r++)
            for (var c = 0; c < 2; c++)
                phi[r, c] = e * ((r == c ? cos : 0.0) + sin * (a[r, c] - (r == c ? sigma : 0.0)));

        // A⁻¹ = [[a11, −a01], [−a10, a00]] / det, det = k/m; B = [0, 1/m].
        var det = Spring / Mass;
        double[] v = [(phi[0, 1]) / Mass, (phi[1, 1] - 1.0) / Mass]; // (Φ − I) B
        double[] gamma = [(a[1, 1] * v[0] - a[0, 1] * v[1]) / det, (-a[1, 0] * v[0] + a[0, 0] * v[1]) / det];
        return (phi, gamma);
    }

    [Fact]
    public void Discretise_MatchesClosedFormZeroOrderHold_ForMassDamperSpring()
    {
        var (phi, gamma) = SampledLoopStability.Discretise(MassDamperSpring(), Ts);
        var (phiExact, gammaExact) = ClosedFormDiscretisation();
        for (var r = 0; r < 2; r++)
        {
            for (var c = 0; c < 2; c++)
                Assert.Equal(phiExact[r, c], phi[r * 2 + c], 1e-12);
            Assert.Equal(gammaExact[r], gamma[r], 1e-12);
        }
    }

    /// <summary>
    /// ρ of the sampled 1-DOF loop equals the closed form to 1e-6: Φcl = Φ − ΓK is 2 × 2, so its eigenvalues follow
    /// from trace and determinant; for a complex pair ρ = √det.
    /// </summary>
    [Fact]
    public void SpectralRadius_MatchesClosedForm_For1DofMassDamperSpring()
    {
        var (phi, gamma) = ClosedFormDiscretisation();
        double[] k = [Kp, Kd];
        var cl = new double[4];
        for (var r = 0; r < 2; r++)
            for (var c = 0; c < 2; c++)
                cl[r * 2 + c] = phi[r, c] - gamma[r] * k[c];
        var trace = cl[0] + cl[3];
        var det = cl[0] * cl[3] - cl[1] * cl[2];
        var disc = trace * trace - 4.0 * det;
        var exact = disc < 0.0
            ? Math.Sqrt(det)
            : Math.Max(Math.Abs(0.5 * (trace + Math.Sqrt(disc))), Math.Abs(0.5 * (trace - Math.Sqrt(disc))));

        var rho = SampledLoopStability.SpectralRadius(MassDamperSpring(), k, Ts);
        var rho12 = SampledLoopStability.SpectralRadius(SampledLoopStability.ClosedLoop(MassDamperSpring(), k, Ts), 2, maxSquarings: 12);
        output.WriteLine(Invariant($"closed form ρ = {exact:R} ({(disc < 0 ? "complex pair" : "real")})"));
        output.WriteLine(Invariant($"repeated squaring until converged: {rho:R}, error {rho - exact:E2}"));
        output.WriteLine(Invariant($"after only 12 squarings:            {rho12:R}, error {rho12 - exact:E2}"));

        Assert.True(disc < 0.0, "the example is meant to have a complex pair");
        Assert.Equal(exact, rho, 1e-6);
        Assert.Equal(exact, rho, 1e-10);
    }

    [Fact]
    public void SpectralRadius_HandlesJordanBlockRotationAndNilpotentMatrices()
    {
        // Defective (Jordan block): ‖P^m‖ grows like m ρ^m, so convergence is slow in m but fine with m = 2^k.
        Assert.Equal(0.9, SampledLoopStability.SpectralRadius([0.9, 1.0, 0.0, 0.9], 2), 1e-12);
        // Non-normal rotation with growth: eigenvalues 1.05 e^{±i·0.3}.
        var (c, s) = (1.05 * Math.Cos(0.3), 1.05 * Math.Sin(0.3));
        Assert.Equal(1.05, SampledLoopStability.SpectralRadius([c, 20.0 * s, -s / 20.0, c], 2), 1e-12);
        // Nilpotent.
        Assert.Equal(0.0, SampledLoopStability.SpectralRadius([0.0, 1.0, 0.0, 0.0], 2));
        // Diagonal with mixed signs.
        Assert.Equal(1.3, SampledLoopStability.SpectralRadius([0.2, 0.0, 0.0, 0.0, -1.3, 0.0, 0.0, 0.0, 0.7], 3), 1e-12);
    }

    /// <summary>
    /// The linearisation used for the arm (A, B) equals the Jacobian of the plant's ẋ = f(x, u) at rest with
    /// u0 = g(q0), by central differences (SPEC §6.1).
    /// </summary>
    [Fact]
    public void ArmAtRest_EqualsJacobianOfThePlant()
    {
        var arm = ArmModel.OpenManipulatorX();
        var system = LinearSystem.ArmAtRest(arm, Planet.HighG, 0.4, PoseC);
        var plant = new ArmPlant(arm, Planet.HighG);
        plant.SetPayload(0.4);
        var u0 = new ClosedFormArm(arm.WithPayload(0.4)).Gravity(PoseC, Planet.HighG.G);
        var x0 = new double[8];
        PoseC.CopyTo(x0, 0);
        var plus = new double[8];
        var minus = new double[8];
        var worst = 0.0;
        const double delta = 1e-7;

        for (var j = 0; j < 12; j++)
        {
            var x = (double[])x0.Clone();
            var u = (double[])u0.Clone();
            if (j < 8) x[j] += delta; else u[j - 8] += delta;
            plant.Derivatives(0.0, x, u, plus);
            if (j < 8) x[j] -= 2 * delta; else u[j - 8] -= 2 * delta;
            plant.Derivatives(0.0, x, u, minus);
            for (var i = 0; i < 8; i++)
            {
                var fd = (plus[i] - minus[i]) / (2 * delta);
                var analytic = j < 8 ? system.A[i * 8 + j] : system.B[i * 4 + (j - 8)];
                worst = Math.Max(worst, Math.Abs(fd - analytic) / Math.Max(1.0, Math.Abs(analytic)));
            }
        }
        output.WriteLine(Invariant($"largest relative difference {worst:E2}"));
        Assert.True(worst < 1e-5, Invariant($"{worst}"));
    }

    /// <summary>
    /// T07 item C: with the default gains, ρ &lt; 1 at Ts = 2 ms on every planet × payload (0, 0.5 kg) at three poses,
    /// for PDG and CT. For PID the linearised loop is its PD part without gravity compensation; it is reported, and
    /// asserted stable wherever its continuous loop is stable (where it is not, Kp cannot hold the arm against the
    /// gravity gradient, and no sample period helps: MaxStableTs = 0).
    /// </summary>
    [Fact]
    public void DefaultGains_AreStableAt2ms_OnEveryPlanetPayloadAndPose()
    {
        output.WriteLine("controller      planet  payload  pose  ρ(Ts = 2 ms)   MaxStableTs");
        foreach (var kind in Enum.GetValues<ControllerKind>())
            foreach (var planet in Planet.All)
                foreach (var payload in Payloads)
                    for (var p = 0; p < StabilityPoses.Length; p++)
                    {
                        var q0 = StabilityPoses[p];
                        var system = LinearSystem.ArmAtRest(ArmModel.OpenManipulatorX(), planet, payload, q0);
                        var ctl = (ILinearisableController)Create(kind, ControllerBelief.OpenManipulatorX(planet, payload));
                        var rho = SampledLoopStability.SpectralRadius(system, ctl, q0, ControlLoop.DefaultSamplePeriod);
                        var maxTs = SampledLoopStability.MaxStableSamplePeriod(system, ctl, q0);
                        output.WriteLine(Invariant($"{kind,-15} {planet.Key,-7} {payload,4:F1} kg  {"ABC"[p]}     {rho:F6}       {maxTs * 1e3,7:F2} ms"));

                        if (kind == ControllerKind.Pid && maxTs == 0.0)
                        {
                            Assert.True(rho > 1.0);
                            continue;
                        }
                        Assert.True(rho < 1.0, Invariant($"{kind} {planet.Key} {payload} kg pose {p}: ρ = {rho}"));
                        Assert.True(maxTs > ControlLoop.DefaultSamplePeriod);
                    }
    }

    [Theory]
    [InlineData(ControllerKind.PdGravity)]
    [InlineData(ControllerKind.ComputedTorque)]
    [InlineData(ControllerKind.Pid)]
    public void SpectralRadius_ExceedsOne_FarAboveMaxStableTs(ControllerKind kind)
    {
        foreach (var q0 in StabilityPoses)
        {
            var system = LinearSystem.ArmAtRest(ArmModel.OpenManipulatorX(), Planet.Earth, 0.0, q0);
            var ctl = (ILinearisableController)Create(kind, ControllerBelief.OpenManipulatorX(Planet.Earth));
            var maxTs = SampledLoopStability.MaxStableSamplePeriod(system, ctl, q0);
            Assert.True(double.IsFinite(maxTs) && maxTs > 0.0);
            Assert.True(SampledLoopStability.SpectralRadius(system, ctl, q0, 0.99 * maxTs) < 1.0);
            Assert.True(SampledLoopStability.SpectralRadius(system, ctl, q0, 1.01 * maxTs) > 1.0);
            foreach (var factor in new[] { 2.0, 4.0 })
            {
                var rho = SampledLoopStability.SpectralRadius(system, ctl, q0, factor * maxTs);
                output.WriteLine(Invariant($"{kind} pose [{TestFormat.Join(q0)}]: MaxStableTs {maxTs * 1e3:F3} ms, ρ({factor}×) = {rho:F3}"));
                Assert.True(rho > 1.0);
            }
        }
    }

    /// <summary>
    /// The prediction agrees with the nonlinear simulation: hold pose q0 from a small offset (1e-4 rad per joint)
    /// with the sample period 10 % above and 10 % below MaxStableTs. Above, the error grows; below, it decays.
    /// RK4 plant, perfect sensor, h = Ts/k ≤ 0.5 ms.
    /// </summary>
    [Theory]
    [InlineData(ControllerKind.PdGravity, 0.0)]
    [InlineData(ControllerKind.ComputedTorque, 0.0)]
    [InlineData(ControllerKind.PdGravity, 0.5)]
    public void Prediction_AgreesWithSimulation_AroundMaxStableTs(ControllerKind kind, double payload)
    {
        var q0 = PoseB;
        var planet = Planet.Earth;
        var system = LinearSystem.ArmAtRest(ArmModel.OpenManipulatorX(), planet, payload, q0);
        var ctl = (ILinearisableController)Create(kind, ControllerBelief.OpenManipulatorX(planet, payload));
        var maxTs = SampledLoopStability.MaxStableSamplePeriod(system, ctl, q0);

        foreach (var factor in new[] { 1.1, 0.9 })
        {
            var ts = factor * maxTs;
            var rho = SampledLoopStability.SpectralRadius(system, ctl, q0, ts);
            var stepsPerSample = (int)Math.Ceiling(ts / 5e-4);
            var h = ts / stepsPerSample;
            var samples = (int)Math.Min(4000, Math.Ceiling(Math.Log(100.0) / Math.Abs(Math.Log(rho))) + 40);

            var start = q0.Select(v => v + 1e-4).ToArray();
            var loop = Loop(Create(kind, ControllerBelief.OpenManipulatorX(planet, payload)), new PerfectSensor(4),
                new ConstantReference(q0), planet, payload, start, h, ts);
            var early = 0.0;
            var late = 0.0;
            for (var s = 0; s < samples; s++)
            {
                if (!loop.AdvanceSample()) break;
                var e = MaxError(q0, loop.Simulation.Q);
                if (s < 20) early = Math.Max(early, e);
                if (s >= samples - 20) late = Math.Max(late, e);
            }
            var faulted = loop.Simulation.IsStopped;
            output.WriteLine(Invariant(
                $"{kind} {payload} kg: MaxStableTs {maxTs * 1e3:F3} ms; Ts = {factor}× → ρ = {rho:F5}, {samples} samples, max error first 20 {early:E2}, last 20 {late:E2}{(faulted ? $" (fault {loop.Simulation.Fault!.Reason})" : "")}"));

            if (factor > 1.0)
            {
                Assert.True(rho > 1.0);
                Assert.True(faulted || late > 10.0 * early, "the error should grow above MaxStableTs");
            }
            else
            {
                Assert.True(rho < 1.0);
                Assert.False(faulted);
                Assert.True(late < 0.1 * early, "the error should decay below MaxStableTs");
            }
        }
    }
}
