using System.Numerics;
using RobotArm.Core.Control;
using RobotArm.Core.Model;

namespace RobotArm.Core.Analysis;

/// <summary>
/// One pole of a sampled closed loop: z, its s-plane equivalent s = ln(z)/Ts, ωn = |s| and ζ = −Re(s)/|s|.
/// SPEC §12.2.
/// </summary>
/// <remarks>
/// s uses the principal logarithm, so |Im s| ≤ π/Ts: a discrete pole only fixes s up to multiples of 2π/Ts
/// (aliasing), and a pole on the negative real axis has no continuous counterpart of the same order. z = 0 gives
/// s = −∞, ωn = +∞ and ζ = 1; z = 1 gives s = 0, ωn = 0 and ζ = NaN.
/// </remarks>
/// <param name="Z">The discrete pole.</param>
/// <param name="S">ln(z)/Ts, 1/s.</param>
/// <param name="NaturalFrequency">ωn = |s|, rad/s.</param>
/// <param name="DampingRatio">ζ = −Re(s)/|s|; negative for an unstable pole.</param>
public readonly record struct DiscretePole(Complex Z, Complex S, double NaturalFrequency, double DampingRatio)
{
    /// <summary>|z|.</summary>
    public double Modulus => Complex.Abs(Z);

    /// <summary>The pole z with its s-plane equivalent for the sample period Ts. SPEC §12.2.</summary>
    public static DiscretePole FromZ(Complex z, double samplePeriod)
    {
        if (z == Complex.Zero)
            return new DiscretePole(z, new Complex(double.NegativeInfinity, 0.0), double.PositiveInfinity, 1.0);
        var s = Complex.Log(z) / samplePeriod;
        var wn = Complex.Abs(s);
        return new DiscretePole(z, s, wn, wn > 0.0 ? -s.Real / wn : double.NaN);
    }
}

/// <summary>The poles of a sampled closed loop. SPEC §12.2.</summary>
/// <param name="SamplePeriod">Ts, s.</param>
/// <param name="Poles">All N poles, by decreasing |z|.</param>
/// <param name="SpectralRadius">ρ = max |z|; below 1 means stable.</param>
public sealed record PoleSet(double SamplePeriod, IReadOnlyList<DiscretePole> Poles, double SpectralRadius)
{
    /// <summary>The poles of the N × N row-major closed-loop matrix. SPEC §12.2.</summary>
    public static PoleSet FromMatrix(ReadOnlySpan<double> closedLoop, int n, double samplePeriod)
    {
        var values = EigenSolver.Eigenvalues(closedLoop, n);
        var poles = new DiscretePole[n];
        var rho = 0.0;
        for (var i = 0; i < n; i++)
        {
            poles[i] = DiscretePole.FromZ(values[i], samplePeriod);
            rho = Math.Max(rho, Complex.Abs(values[i]));
        }
        return new PoleSet(samplePeriod, Array.AsReadOnly(poles), rho);
    }
}

/// <summary>Entry points of the linear analysis of the sampled arm loop. SPEC §12; docs/DECISIONS.md D9.</summary>
public static class LinearAnalysis
{
    /// <summary>
    /// The z-poles of the arm's sampled closed loop under <paramref name="controller"/>, linearised at rest at
    /// <paramref name="q0"/>, with s = ln(z)/Ts, ωn and ζ of each: 8 poles for PD-type laws (PDG, CT), 12 with an
    /// integral (PID, CT+I). SPEC §12.2. The approximations are listed in <see cref="SampledLoopModel"/>.
    /// </summary>
    public static PoleSet ClosedLoopPoles(ArmModel arm, Planet planet, double payloadKg, ReadOnlySpan<double> q0,
        ILinearisableController controller, double samplePeriod) =>
        ClosedLoopPoles(SampledLoopModel.ForArm(arm, planet, payloadKg, q0, controller, samplePeriod));

    /// <summary>The poles of any sampled loop model. SPEC §12.2.</summary>
    public static PoleSet ClosedLoopPoles(SampledLoopModel model)
    {
        ArgumentNullException.ThrowIfNull(model);
        return PoleSet.FromMatrix(model.ClosedLoopMatrix(), model.StateCount, model.SamplePeriod);
    }
}
