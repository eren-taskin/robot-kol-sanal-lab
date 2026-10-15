using System.Numerics;
using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.ControlTestHelpers;

namespace RobotArm.Tests;

/// <summary>SPEC §12.2: the discrete closed-loop model of each controller and its poles (V37).</summary>
public sealed class ClosedLoopPolesTests(ITestOutputHelper output)
{
    private const double Ts = ControlLoop.DefaultSamplePeriod;

    [Fact]
    public void PdTypeLaws_Have8Poles_AndLawsWithIntegrators_Have12()
    {
        var arm = ArmModel.OpenManipulatorX();
        var q0 = AnalysisFixture.StartPose;
        var belief = ControllerBelief.OpenManipulatorX(Planet.Earth);
        (ILinearisableController Law, int States)[] cases =
        [
            (new PdGravity(belief, PdGravity.DefaultGains), 8),
            (new ComputedTorque(belief, ComputedTorque.DefaultGains), 8),
            (new Pid(belief, Pid.DefaultGains), 12),
            (AnalysisFixture.CtPlusI(), 12),
        ];
        foreach (var (law, states) in cases)
        {
            var poles = LinearAnalysis.ClosedLoopPoles(arm, Planet.Earth, 0.0, q0, law, Ts);
            Assert.Equal(states, poles.Poles.Count);
            Assert.True(poles.SpectralRadius < 1.0, Invariant($"{law.GetType().Name}: ρ = {poles.SpectralRadius}"));
            foreach (var p in poles.Poles)
            {
                // s = ln(z)/Ts round trip, and ωn, ζ from s.
                var back = Complex.Exp(p.S * Ts);
                Assert.True(Complex.Abs(back - p.Z) < 1e-12);
                Assert.Equal(Complex.Abs(p.S), p.NaturalFrequency, 1e-9);
                Assert.Equal(-p.S.Real / p.NaturalFrequency, p.DampingRatio, 1e-12);
            }
        }
    }

    /// <summary>
    /// For the PD-type laws, ρ of the model equals <see cref="SampledLoopStability.SpectralRadius(LinearSystem, ILinearisableController, ReadOnlySpan{double}, double, int)"/>
    /// (same Φ − ΓK), on every planet and payload at poses A, B, C.
    /// </summary>
    [Fact]
    public void PdTypeModel_EqualsSampledLoopStability()
    {
        foreach (var planet in Planet.All)
            foreach (var payload in Payloads)
                foreach (var q0 in StabilityPoses)
                    foreach (var kind in new[] { ControllerKind.PdGravity, ControllerKind.ComputedTorque })
                    {
                        var ctl = (ILinearisableController)Create(kind, ControllerBelief.OpenManipulatorX(planet, payload));
                        var system = LinearSystem.ArmAtRest(ArmModel.OpenManipulatorX(), planet, payload, q0);
                        var expected = SampledLoopStability.SpectralRadius(system, ctl, q0, Ts);
                        var poles = LinearAnalysis.ClosedLoopPoles(ArmModel.OpenManipulatorX(), planet, payload, q0, ctl, Ts);
                        Assert.Equal(expected, poles.SpectralRadius, 1e-9);
                    }
    }

    /// <summary>
    /// The augmented model (K, L, Kz, integrator states) reproduces the real controllers' Compute driving the
    /// linear plant x_{k+1} = Φ x_k + Γ (u_k − u0), for a tiny reference step and initial offset (so the laws' own
    /// nonlinearity is negligible). Covers PID and CT+I (12 states, integral) and PDG and CT (8 states).
    /// </summary>
    [Theory]
    [InlineData(ControllerKind.Pid, false)]
    [InlineData(ControllerKind.PdGravity, false)]
    [InlineData(ControllerKind.ComputedTorque, false)]
    [InlineData(ControllerKind.ComputedTorque, true)]
    public void AugmentedModel_ReproducesTheControllerLaw(ControllerKind kind, bool integral)
    {
        var planet = Planet.Earth;
        var q0 = AnalysisFixture.StartPose;
        var belief = ControllerBelief.OpenManipulatorX(planet);
        IController Make() => integral ? AnalysisFixture.CtPlusI() : Create(kind, belief);
        var model = SampledLoopModel.ForArm(ArmModel.OpenManipulatorX(), planet, 0.0, q0, (ILinearisableController)Make(), Ts);
        var n = model.StateCount;
        const double scale = 1e-6;
        double[] step = [scale, -2 * scale, scale, 3 * scale];
        double[] offset = [-scale, scale, 2 * scale, -scale];

        // u0: what the law outputs at rest at q0 with q_d = q0 (ĝ for PDG/CT, 0 for PID with an empty integral).
        var u0 = new double[4];
        if (kind != ControllerKind.Pid)
            new RecursiveNewtonEuler(belief.Model).BiasTorque(q0, new double[4], planet, u0);

        var controller = Make();
        var zero = new double[4];
        var target = q0.Zip(step, (a, b) => a + b).ToArray();
        var u = new double[4];
        // One sample at rest at t = −Ts so that every later sample has Δt = Ts (integral and prediction).
        controller.Compute(-Ts, q0, zero, new JointReference(q0, zero, zero), u);

        var x = new double[8];
        for (var i = 0; i < 4; i++) x[i] = offset[i];
        var xi = new double[n];
        Array.Copy(x, xi, 8);
        var acl = model.ClosedLoopMatrix();
        var bcl = model.ClosedLoopReferenceMatrix();
        var worst = 0.0;
        for (var s = 0; s < 200; s++)
        {
            var q = new double[4];
            var qd = new double[4];
            for (var i = 0; i < 4; i++) { q[i] = q0[i] + x[i]; qd[i] = x[4 + i]; }
            controller.Compute(s * Ts, q, qd, new JointReference(target, zero, zero), u);
            var next = new double[8];
            for (var r = 0; r < 8; r++)
            {
                var sum = 0.0;
                for (var c = 0; c < 8; c++) sum += model.Phi[r * 8 + c] * x[c];
                for (var j = 0; j < 4; j++) sum += model.Gamma[r * 4 + j] * (u[j] - u0[j]);
                next[r] = sum;
            }
            x = next;

            var nextXi = new double[n];
            for (var r = 0; r < n; r++)
            {
                var sum = 0.0;
                for (var c = 0; c < n; c++) sum += acl[r * n + c] * xi[c];
                for (var j = 0; j < 4; j++) sum += bcl[r * 4 + j] * step[j];
                nextXi[r] = sum;
            }
            xi = nextXi;
            for (var r = 0; r < 8; r++) worst = Math.Max(worst, Math.Abs(xi[r] - x[r]));
        }
        output.WriteLine(Invariant($"{kind}{(integral ? "+I" : "")}: {n} states, largest state difference over 200 samples {worst:E2} (signal scale {scale:E0})"));
        Assert.True(worst < 1e-3 * scale, Invariant($"{worst}"));
    }

    [Fact]
    public void DiscretePole_HandlesSpecialPoints()
    {
        var zero = DiscretePole.FromZ(Complex.Zero, Ts);
        Assert.Equal(double.PositiveInfinity, zero.NaturalFrequency);
        Assert.Equal(1.0, zero.DampingRatio);
        var one = DiscretePole.FromZ(Complex.One, Ts);
        Assert.Equal(0.0, one.NaturalFrequency);
        Assert.True(double.IsNaN(one.DampingRatio));
        var negative = DiscretePole.FromZ(new Complex(-0.5, 0.0), Ts);
        Assert.Equal(Math.PI / Ts, negative.S.Imaginary, 1e-9);
        var unstable = DiscretePole.FromZ(Complex.FromPolarCoordinates(1.1, 0.2), Ts);
        Assert.True(unstable.DampingRatio < 0.0);
    }

    /// <summary>
    /// Report (A1): default PDG gains on Earth at the start pose of the first library scenario: closed-loop poles,
    /// loop-at-a-time margins per joint and the critical scale factor on all gains.
    /// </summary>
    [Fact]
    public void Report_DefaultPdg_EarthReachStartPose()
    {
        var arm = ArmModel.OpenManipulatorX();
        var q0 = AnalysisFixture.StartPose;
        var pdg = AnalysisFixture.DefaultPdg();
        var model = SampledLoopModel.ForArm(arm, Planet.Earth, 0.0, q0, pdg, Ts);
        var poles = LinearAnalysis.ClosedLoopPoles(model);
        output.WriteLine(Invariant($"PDG default gains, Earth, 0 kg, q0 = [{TestFormat.Join(q0)}], Ts = {Ts * 1e3} ms; ρ = {poles.SpectralRadius:F6}"));
        output.WriteLine("      z                      |z|        s (1/s)                    ωn (rad/s)   ζ");
        foreach (var p in poles.Poles)
            output.WriteLine(Invariant($"  {p.Z.Real,9:F6} {p.Z.Imaginary,+10:F6}i  {p.Modulus:F6}  {p.S.Real,10:F3} {p.S.Imaginary,+10:F3}i  {p.NaturalFrequency,10:F3}  {p.DampingRatio,7:F4}"));

        output.WriteLine("joint  GM (dB)   ω_pc (rad/s)   PM (deg)   ω_gc (rad/s)   open-loop unstable poles");
        foreach (var r in LoopFrequencyResponse.ComputeAll(model))
        {
            output.WriteLine(Invariant($"  {r.Joint + 1}   {r.GainMarginDb,8:F3}   {r.PhaseCrossoverFrequency,10:F2}   {r.PhaseMarginDeg,8:F3}   {r.GainCrossoverFrequency,10:F3}   {r.OpenLoopUnstablePoles}"));
            Assert.True(r.GainMarginDb > 0.0);
            Assert.True(r.PhaseMarginDeg > 0.0);
        }

        var locus = RootLocus.Sweep(arm, Planet.Earth, 0.0, q0, pdg, Ts, LocusParameter.ScaleAll, 0.05, 50.0);
        output.WriteLine(Invariant($"critical scale factor on all gains: {locus.CriticalValue:F6} ({20 * Math.Log10(locus.CriticalValue!.Value):F3} dB), {locus.Values.Count} locus points"));
        foreach (var c in locus.Crossings)
            output.WriteLine(Invariant($"  crossing at k = {c.Value:F6} ({(c.BecomesUnstable ? "becomes unstable" : "becomes stable")})"));
        Assert.True(poles.SpectralRadius < 1.0);
        Assert.True(locus.CriticalValue > 1.0);
    }
}
