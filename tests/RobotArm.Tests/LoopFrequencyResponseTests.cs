using System.Numerics;
using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using RobotArm.Core.Model;
using RobotArm.Core.Tasks;
using Xunit.Abstractions;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>SPEC §12.4: loop-at-a-time frequency response and margins (V39).</summary>
public sealed class LoopFrequencyResponseTests(ITestOutputHelper output)
{
    private const double Ts = ControlLoop.DefaultSamplePeriod;

    /// <summary>
    /// 1-DOF: L(e^{jωTs}) equals the analytic discrete loop [kp kd] adj(zI − Φ) Γ / det(zI − Φ) with closed-form Φ, Γ,
    /// in magnitude and phase (mod 360°) at every grid frequency up to Nyquist.
    /// </summary>
    [Fact]
    public void OneDof_MatchesTheAnalyticDiscreteLoop()
    {
        var response = LoopFrequencyResponse.Compute(OneDof.Model(), 0);
        var worstDb = 0.0;
        var worstDeg = 0.0;
        for (var p = 0; p < response.Frequencies.Count; p++)
        {
            var l = OneDof.Loop(response.Frequencies[p]);
            worstDb = Math.Max(worstDb, Math.Abs(20 * Math.Log10(Complex.Abs(l)) - response.MagnitudeDb[p]));
            var phase = Math.Atan2(l.Imaginary, l.Real) * 180 / Math.PI;
            var diff = Math.IEEERemainder(response.PhaseDeg[p] - phase, 360.0);
            worstDeg = Math.Max(worstDeg, Math.Abs(diff));
        }
        output.WriteLine(Invariant($"{response.Frequencies.Count} points to {response.Frequencies[^1]:F3} rad/s (π/Ts = {Math.PI / OneDof.Ts:F3}); largest |ΔdB| {worstDb:E2}, |Δphase| {worstDeg:E2}°"));
        Assert.Equal(Math.PI / OneDof.Ts, response.Frequencies[^1], 1e-12);
        Assert.True(worstDb < 1e-9);
        Assert.True(worstDeg < 1e-7);
        for (var p = 1; p < response.PhaseDeg.Count; p++)
            Assert.True(Math.Abs(response.PhaseDeg[p] - response.PhaseDeg[p - 1]) < 180.0, "phase is unwrapped");
    }

    /// <summary>
    /// Single loop: the gain margin in dB equals 20·log10 of the critical scale factor of the root locus
    /// (closed form, Jury), and |L| = 1 at the gain crossover.
    /// </summary>
    [Fact]
    public void OneDof_GainMargin_EqualsCriticalScaleFactorOfRootLocus()
    {
        var response = LoopFrequencyResponse.Compute(OneDof.Model(), 0);
        var critical = OneDof.CriticalScale();
        var locus = RootLocus.Sweep(s => OneDof.Model(s).ClosedLoopMatrix(), 2, 0.01, 100.0);
        output.WriteLine(Invariant($"GM = {response.GainMarginDb:R} dB at {response.PhaseCrossoverFrequency:R} rad/s; 20 log10(k_crit) = {20 * Math.Log10(critical):R} (closed form), {20 * Math.Log10(locus.CriticalValue!.Value):R} (root locus)"));
        output.WriteLine(Invariant($"PM = {response.PhaseMarginDeg:R}° at {response.GainCrossoverFrequency:R} rad/s"));
        Assert.Equal(20 * Math.Log10(critical), response.GainMarginDb, 1e-6);
        Assert.Equal(20 * Math.Log10(locus.CriticalValue!.Value), response.GainMarginDb, 1e-6);
        Assert.Equal(1.0, Complex.Abs(OneDof.Loop(response.GainCrossoverFrequency)), 1e-9);
        Assert.Equal(0, response.OpenLoopUnstablePoles);
    }

    /// <summary>
    /// On the arm, the margins are where they say: at ω_pc the loop is real and negative with |L| = 10^(−GM/20), at
    /// ω_gc |L| = 1 with phase −180° + PM; scaling joint i's row of K_ξ by 10^(GM/20) puts a pole on the unit circle.
    /// Default PDG and CT+I at the library start pose.
    /// </summary>
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ArmMargins_AreConsistentWithTheClosedLoop(bool ctPlusI)
    {
        var arm = ArmModel.OpenManipulatorX();
        var q0 = AnalysisFixture.StartPose;
        ILinearisableController law = ctPlusI ? AnalysisFixture.CtPlusI() : AnalysisFixture.DefaultPdg();
        var model = SampledLoopModel.ForArm(arm, Planet.Earth, 0.0, q0, law, Ts);
        foreach (var r in LoopFrequencyResponse.ComputeAll(model))
        {
            output.WriteLine(Invariant($"{(ctPlusI ? "CT+I" : "PDG")} joint {r.Joint + 1}: GM {r.GainMarginDb:F4} dB at {r.PhaseCrossoverFrequency:F3}, PM {r.PhaseMarginDeg:F4}° at {r.GainCrossoverFrequency:F4} rad/s, open-loop unstable {r.OpenLoopUnstablePoles}"));
            var lpc = LoopFrequencyResponse.Evaluate(model, r.Joint, r.PhaseCrossoverFrequency);
            Assert.True(lpc.Real < 0.0);
            Assert.True(Math.Abs(lpc.Imaginary) < 1e-6 * Complex.Abs(lpc));
            Assert.Equal(-r.GainMarginDb, 20 * Math.Log10(Complex.Abs(lpc)), 1e-9);
            var lgc = LoopFrequencyResponse.Evaluate(model, r.Joint, r.GainCrossoverFrequency);
            Assert.Equal(1.0, Complex.Abs(lgc), 1e-9);

            // Scale row i of K_ξ by the gain margin: the closed loop is then on the stability boundary.
            var factor = Math.Pow(10, r.GainMarginDb / 20);
            var a = model.OpenLoopMatrix();
            var b = model.InputMatrix();
            var k = model.AugmentedFeedbackGain();
            var n = model.StateCount;
            for (var c = 0; c < n; c++) k[r.Joint * n + c] *= factor;
            var acl = DenseMatrix.Add(a, DenseMatrix.Multiply(b, k, n, 4, n), -1.0);
            Assert.Equal(1.0, EigenSolver.SpectralRadius(acl, n), 1e-6);
        }
    }

    /// <summary>
    /// Regression check across all five planets, payloads 0 and 0.5 kg, and the distinct start poses of
    /// <see cref="ScenarioLibrary.All"/> for the default gains of all four controllers (PID, PDG, CT, CT+I).
    /// CT+I must keep ρ &lt; 1, gain margin ≥ 6 dB and phase margin ≥ 40° on every joint in every case.
    /// </summary>
    [Fact]
    public void DefaultGains_MarginsAtLibraryStartPoses_DoNotRegress()
    {
        var arm = ArmModel.OpenManipulatorX();
        var poses = new List<(string Name, double[] Q)>();
        foreach (var s in ScenarioLibrary.All)
        {
            var q = s.Task.Start.ToArray();
            if (!poses.Any(p => p.Q.Zip(q, (a, b) => Math.Abs(a - b)).Max() < 1e-12))
                poses.Add((s.Name, q));
        }

        foreach (var law in new[] { "PID", "PDG", "CT", "CT+I" })
        {
            var maxRho = 0.0;
            var minGm = new[] { double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity };
            var minAbsGm = new[] { double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity };
            var minPm = new[] { double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity };

            foreach (var planet in Planet.All)
            {
                foreach (var payload in new[] { 0.0, 0.5 })
                {
                    var belief = new ControllerBelief(arm, planet, payload);
                    ILinearisableController controller = law switch
                    {
                        "PID" => new Pid(belief, Pid.DefaultGains),
                        "PDG" => new PdGravity(belief, PdGravity.DefaultGains),
                        "CT" => new ComputedTorque(belief, ComputedTorque.DefaultGains),
                        _ => new ComputedTorque(belief, ComputedTorque.DefaultIntegralPdGains, integralGains: ComputedTorque.DefaultIntegralGains),
                    };
                    for (var pIndex = 0; pIndex < poses.Count; pIndex++)
                    {
                        var (poseName, q0) = poses[pIndex];
                        var model = SampledLoopModel.ForArm(arm, planet, payload, q0, controller, Ts);
                        var rho = LinearAnalysis.ClosedLoopPoles(model).SpectralRadius;
                        maxRho = Math.Max(maxRho, rho);
                        Assert.True(rho < 1.0, Invariant($"{law} {planet.Key} {payload} kg ({poseName}): ρ = {rho:F6}"));

                        for (var j = 0; j < 4; j++)
                        {
                            var b = LoopFrequencyResponse.Compute(model, j, 200);
                            minGm[j] = Math.Min(minGm[j], b.GainMarginDb);
                            minAbsGm[j] = Math.Min(minAbsGm[j], Math.Abs(b.GainMarginDb));
                            minPm[j] = Math.Min(minPm[j], b.PhaseMarginDeg);

                            if (law == "CT+I")
                            {
                                Assert.True(b.GainMarginDb >= 6.0, Invariant($"CT+I J{j + 1} {planet.Key} {payload} kg ({poseName}): GM = {b.GainMarginDb:F2} dB"));
                                Assert.True(b.PhaseMarginDeg >= 40.0, Invariant($"CT+I J{j + 1} {planet.Key} {payload} kg ({poseName}): PM = {b.PhaseMarginDeg:F2}°"));
                            }
                            else if (law == "PDG" && pIndex != 1)
                            {
                                Assert.True(b.GainMarginDb >= 6.0, Invariant($"PDG J{j + 1} {planet.Key} {payload} kg ({poseName}): GM = {b.GainMarginDb:F2} dB"));
                                Assert.True(b.PhaseMarginDeg >= 40.0, Invariant($"PDG J{j + 1} {planet.Key} {payload} kg ({poseName}): PM = {b.PhaseMarginDeg:F2}°"));
                            }
                        }
                    }
                }
            }

            output.WriteLine(Invariant($"{law,-4}: max ρ = {maxRho:F6}; min GM = [{string.Join(", ", minGm.Select(x => x.ToString("F2", System.Globalization.CultureInfo.InvariantCulture)))}] dB (min |GM| = [{string.Join(", ", minAbsGm.Select(x => x.ToString("F2", System.Globalization.CultureInfo.InvariantCulture)))}] dB); min PM = [{string.Join(", ", minPm.Select(x => x.ToString("F2", System.Globalization.CultureInfo.InvariantCulture)))}]°"));

            for (var j = 0; j < 4; j++)
            {
                if (law == "PDG")
                {
                    Assert.True(minGm[j] >= 2.6);
                    Assert.True(minPm[j] >= 37.0);
                }
                else if (law == "PID")
                {
                    Assert.True(minAbsGm[j] >= 1.0);
                    Assert.True(minPm[j] >= 10.9);
                }
                else if (law == "CT")
                {
                    Assert.True(minGm[j] >= 0.34);
                    Assert.True(minPm[j] >= 10.3);
                }
            }
        }
    }

    [Fact]
    public void Compute_RejectsBadArguments()
    {
        var model = OneDof.Model();
        Assert.Throws<ArgumentOutOfRangeException>(() => LoopFrequencyResponse.Compute(model, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => LoopFrequencyResponse.Compute(model, 0, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => LoopFrequencyResponse.Compute(model, 0, minFrequency: 1e6));
    }
}
