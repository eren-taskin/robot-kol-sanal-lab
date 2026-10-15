using System.Numerics;
using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using RobotArm.Core.Model;
using Xunit.Abstractions;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>SPEC §12.3: root locus over one gain or a common scale factor (V38).</summary>
public sealed class RootLocusTests(ITestOutputHelper output)
{
    private const double Ts = ControlLoop.DefaultSamplePeriod;

    /// <summary>
    /// 1-DOF mass–spring–damper with sampled PD: the discrete characteristic polynomial z² − tr(s) z + det(s) is
    /// affine in s in closed form, so the poles and the critical scale (Jury) are known exactly.
    /// </summary>
    [Fact]
    public void OneDof_PolesAndCriticalScale_MatchTheClosedForm()
    {
        var (phi, gamma) = OneDof.Discretisation();
        double[] ClosedLoop(double s) =>
        [
            phi[0] - gamma[0] * s * OneDof.Kp, phi[1] - gamma[0] * s * OneDof.Kd,
            phi[2] - gamma[1] * s * OneDof.Kp, phi[3] - gamma[1] * s * OneDof.Kd,
        ];
        var locus = RootLocus.Sweep(ClosedLoop, 2, 0.01, 100.0);
        var exact = OneDof.CriticalScale();

        var worst = 0.0;
        for (var i = 0; i < locus.Values.Count; i++)
        {
            var expected = OneDof.Poles(locus.Values[i]);
            var got = new[] { locus.Tracks[0][i], locus.Tracks[1][i] };
            var direct = Math.Max(Complex.Abs(got[0] - expected[0]), Complex.Abs(got[1] - expected[1]));
            var swapped = Math.Max(Complex.Abs(got[0] - expected[1]), Complex.Abs(got[1] - expected[0]));
            worst = Math.Max(worst, Math.Min(direct, swapped) / Math.Max(1.0, Complex.Abs(expected[0])));
        }
        output.WriteLine(Invariant($"{locus.Values.Count} points; largest pole error {worst:E2}"));
        output.WriteLine(Invariant($"critical scale: closed form {exact:R}, bisection {locus.CriticalValue:R}"));
        foreach (var c in locus.Crossings) output.WriteLine(Invariant($"  crossing {c.Value:R} {(c.BecomesUnstable ? "→ unstable" : "→ stable")}"));

        Assert.True(worst < 1e-12);
        Assert.NotNull(locus.CriticalValue);
        Assert.Equal(exact, locus.CriticalValue!.Value, exact * 1e-8);
        Assert.True(locus.CriticalValue > 1.0, "the nominal loop is stable");
    }

    /// <summary>The same through <see cref="SampledLoopModel"/> (Φ, Γ by RK4 sub-steps): agrees with the closed form.</summary>
    [Fact]
    public void OneDof_ThroughSampledLoopModel_FindsTheSameCriticalScale()
    {
        var locus = RootLocus.Sweep(s => OneDof.Model(s).ClosedLoopMatrix(), 2, 0.01, 100.0);
        Assert.Equal(OneDof.CriticalScale(), locus.CriticalValue!.Value, OneDof.CriticalScale() * 1e-8);
    }

    /// <summary>Consecutive points on a track are close (relative step ≤ MaxPoleStep), so the tracks are continuous.</summary>
    [Fact]
    public void Tracks_AreContinuous_AndRefinedWherePolesMoveFast()
    {
        var arm = ArmModel.OpenManipulatorX();
        var q0 = AnalysisFixture.StartPose;
        var options = new RootLocusOptions();
        var locus = RootLocus.Sweep(arm, Planet.Earth, 0.0, q0, AnalysisFixture.DefaultPdg(), Ts, LocusParameter.ScaleAll, 0.05, 50.0, options);
        var worst = 0.0;
        foreach (var track in locus.Tracks)
            for (var i = 1; i < track.Count; i++)
                worst = Math.Max(worst, Complex.Abs(track[i] - track[i - 1]) / Math.Max(1.0, Math.Max(Complex.Abs(track[i]), Complex.Abs(track[i - 1]))));
        output.WriteLine(Invariant($"{locus.Values.Count} points (initial {options.InitialPoints}); largest relative step on a track {worst:F4}"));
        Assert.True(locus.Values.Count > options.InitialPoints);
        Assert.True(locus.Values.Count < options.MaxPoints);
        Assert.True(worst <= options.MaxPoleStep);
        Assert.Equal(8, locus.Tracks.Count);
        for (var i = 1; i < locus.Values.Count; i++) Assert.True(locus.Values[i] > locus.Values[i - 1]);
    }

    /// <summary>
    /// On the arm the crossings found by bisection separate stable from unstable: ρ just below and just above,
    /// recomputed through <see cref="LinearAnalysis.ClosedLoopPoles(ArmModel, Planet, double, ReadOnlySpan{double}, ILinearisableController, double)"/>
    /// with a controller whose gains are changed by hand. Scale on all gains (PDG, CT) and one gain of one joint.
    /// </summary>
    [Theory]
    [InlineData("PDG", LocusGain.Scale, -1, 0.05, 50.0)]
    [InlineData("CT", LocusGain.Scale, -1, 0.05, 50.0)]
    [InlineData("PDG", LocusGain.Kd, 3, 0.005, 5.0)]
    [InlineData("PDG", LocusGain.Kp, 3, 0.5, 5000.0)]
    [InlineData("PDG", LocusGain.Kd, 0, 0.01, 100.0)]
    public void Crossing_SeparatesStableFromUnstable_OnTheArm(string law, LocusGain gain, int joint, double min, double max)
    {
        var arm = ArmModel.OpenManipulatorX();
        var q0 = AnalysisFixture.StartPose;
        var belief = ControllerBelief.OpenManipulatorX(Planet.Earth);
        ILinearisableController controller = law == "CT" ? new ComputedTorque(belief, ComputedTorque.DefaultGains) : AnalysisFixture.DefaultPdg();
        var parameter = new LocusParameter(gain, joint);
        var locus = RootLocus.Sweep(arm, Planet.Earth, 0.0, q0, controller, Ts, parameter, min, max);
        output.WriteLine(Invariant($"{law} {gain} joint {joint + 1}: {locus.Values.Count} points, crossings: {string.Join(", ", locus.Crossings.Select(c => Invariant($"{c.Value:G8}{(c.BecomesUnstable ? "↑" : "↓")}")))}"));
        Assert.NotEmpty(locus.Crossings);
        foreach (var c in locus.Crossings)
        {
            double Rho(double v)
            {
                var (kp, kd, ki) = ControllerGains.Get(controller);
                if (gain == LocusGain.Scale) { for (var i = 0; i < 4; i++) { kp[i] *= v; kd[i] *= v; } }
                else if (gain == LocusGain.Kp) kp[joint] = v;
                else kd[joint] = v;
                var changed = ControllerGains.Build(controller, kp, kd, ki);
                return LinearAnalysis.ClosedLoopPoles(arm, Planet.Earth, 0.0, q0, changed, Ts).SpectralRadius;
            }
            var below = Rho(c.Value * (1 - 1e-6));
            var above = Rho(c.Value * (1 + 1e-6));
            Assert.Equal(c.BecomesUnstable, below < 1.0);
            Assert.Equal(c.BecomesUnstable, above > 1.0);
        }
    }

    [Fact]
    public void Sweep_RejectsBadArguments()
    {
        var arm = ArmModel.OpenManipulatorX();
        var q0 = AnalysisFixture.StartPose;
        var pdg = AnalysisFixture.DefaultPdg();
        Assert.Throws<ArgumentOutOfRangeException>(() => RootLocus.Sweep(s => OneDof.Model(s).ClosedLoopMatrix(), 2, 0.0, 1.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => RootLocus.Sweep(s => OneDof.Model(s).ClosedLoopMatrix(), 2, 2.0, 1.0));
        Assert.Throws<ArgumentException>(() => RootLocus.Sweep(arm, Planet.Earth, 0, q0, pdg, Ts, new LocusParameter(LocusGain.Ki, 0), 0.1, 1));
        Assert.Throws<ArgumentException>(() => RootLocus.Sweep(arm, Planet.Earth, 0, q0, pdg, Ts, new LocusParameter(LocusGain.Kp, 4), 0.1, 1));
    }
}
