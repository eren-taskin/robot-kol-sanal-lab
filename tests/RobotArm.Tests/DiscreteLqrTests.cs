using RobotArm.Core.Analysis;
using RobotArm.Core.Control;
using RobotArm.Core.Model;
using Xunit.Abstractions;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>SPEC §12.6: discrete LQR by Riccati doubling, and its diagonal projection (V41).</summary>
public sealed class DiscreteLqrTests(ITestOutputHelper output)
{
    private const double Ts = ControlLoop.DefaultSamplePeriod;

    /// <summary>Scalar x+ = a x + b u: the DARE is b²p² + p(r − a²r − qb²) − qr = 0, so p is known in closed form.</summary>
    [Fact]
    public void Scalar_MatchesClosedFormRiccati()
    {
        const double a = 1.1, b = 0.5, q = 2.0, r = 0.3;
        var c1 = r - a * a * r - q * b * b;
        var exact = (-c1 + Math.Sqrt(c1 * c1 + 4 * b * b * q * r)) / (2 * b * b);
        var result = DiscreteLqr.Solve([a], [b], 1, 1, [q], [r], 1.0);
        Assert.Equal(exact, result.Riccati[0], 1e-12 * exact);
        Assert.Equal(b * exact * a / (r + b * b * exact), result.Gain[0], 1e-12);
        Assert.True(result.Poles.SpectralRadius < 1.0);
        Assert.Null(result.Projection);
    }

    /// <summary>
    /// The arm at the library start pose, Earth, Ts = 2 ms: DARE residual &lt; 1e-9, the LQR loop is stable, and the
    /// diagonal projection is reported with its poles and ρ.
    /// </summary>
    [Fact]
    public void Arm_ResidualIsSmall_AndLoopIsStable()
    {
        var q0 = AnalysisFixture.StartPose;
        double[] qPos = [100.0, 100.0, 100.0, 100.0];
        double[] qVel = [0.1, 0.1, 0.1, 0.1];
        double[] rIn = [1.0, 1.0, 1.0, 1.0];
        var lqr = DiscreteLqr.ForArm(ArmModel.OpenManipulatorX(), Planet.Earth, 0.0, q0, Ts, qPos, qVel, rIn);

        output.WriteLine(Invariant($"Q = diag({TestFormat.Join(qPos)}, {TestFormat.Join(qVel)}), R = diag({TestFormat.Join(rIn)}); {lqr.Iterations} doubling steps, residual {lqr.Residual:E2}"));
        output.WriteLine("K (4 × 8):");
        for (var i = 0; i < 4; i++)
            output.WriteLine("  " + string.Join(" ", Enumerable.Range(0, 8).Select(j => Invariant($"{lqr.Gain[i * 8 + j],9:F4}"))));
        output.WriteLine(Invariant($"LQR ρ = {lqr.Poles.SpectralRadius:F6}; slowest ωn = {lqr.Poles.Poles[0].NaturalFrequency:F3} rad/s, ζ = {lqr.Poles.Poles[0].DampingRatio:F3}"));
        var p = lqr.Projection!;
        output.WriteLine(Invariant($"projection Kp = [{TestFormat.Join(p.Kp.ToArray())}], Kd = [{TestFormat.Join(p.Kd.ToArray())}], ρ = {p.SpectralRadius:F6}"));
        foreach (var (full, proj) in lqr.Poles.Poles.Zip(p.Poles.Poles))
            output.WriteLine(Invariant($"  LQR z = {full.Z.Real,9:F6}{full.Z.Imaginary,+10:F6}i (ωn {full.NaturalFrequency,8:F3}, ζ {full.DampingRatio,6:F3})   projection z = {proj.Z.Real,9:F6}{proj.Z.Imaginary,+10:F6}i (ωn {proj.NaturalFrequency,8:F3}, ζ {proj.DampingRatio,6:F3})"));

        Assert.Equal(32, lqr.Gain.Count);
        Assert.True(lqr.Residual < 1e-9, Invariant($"residual {lqr.Residual}"));
        Assert.True(lqr.Poles.SpectralRadius < 1.0);
        Assert.Equal(8, p.Poles.Poles.Count);
        for (var i = 0; i < 4; i++)
        {
            Assert.Equal(lqr.Gain[i * 8 + i], p.Kp[i]);
            Assert.Equal(lqr.Gain[i * 8 + 4 + i], p.Kd[i]);
        }
        // P is symmetric.
        for (var i = 0; i < 8; i++)
            for (var j = 0; j < 8; j++) Assert.Equal(lqr.Riccati[i * 8 + j], lqr.Riccati[j * 8 + i]);
    }

    /// <summary>On every planet, 0 and 0.5 kg, at the poses A, B, C: residual &lt; 1e-9 and a stable loop.</summary>
    [Fact]
    public void Arm_SolvesEverywhere()
    {
        var worst = 0.0;
        foreach (var planet in Planet.All)
            foreach (var payload in ControlTestHelpers.Payloads)
                foreach (var q0 in ControlTestHelpers.StabilityPoses)
                {
                    var lqr = DiscreteLqr.ForArm(ArmModel.OpenManipulatorX(), planet, payload, q0, Ts, [100, 100, 100, 100], [0.1, 0.1, 0.1, 0.1], [1, 1, 1, 1]);
                    worst = Math.Max(worst, lqr.Residual);
                    Assert.True(lqr.Residual < 1e-9);
                    Assert.True(lqr.Poles.SpectralRadius < 1.0);
                }
        output.WriteLine(Invariant($"largest residual {worst:E2}"));
    }

    [Fact]
    public void Solve_RejectsBadWeights()
    {
        Assert.Throws<ArgumentException>(() => DiscreteLqr.Solve([1.0], [1.0], 1, 1, [-1.0], [1.0], 1.0));
        Assert.Throws<ArgumentException>(() => DiscreteLqr.Solve([1.0], [1.0], 1, 1, [1.0], [0.0], 1.0));
        Assert.Throws<ArgumentException>(() => DiscreteLqr.Solve([1.0], [1.0, 2.0], 1, 1, [1.0], [1.0], 1.0));
    }
}
