using System.Globalization;
using RobotArm.Core.Control;
using RobotArm.Core.Model;

namespace RobotArm.Core.Analysis;

/// <summary>
/// The linear discrete closed loop of a sampled controller on a plant held by zero-order hold, with an optional
/// integrator state per input and a reference input. SPEC §12.2; docs/DECISIONS.md D9.
/// </summary>
/// <remarks>
/// <para>
/// Plant: x_{k+1} = Φ x_k + Γ u_k with Φ, Γ from <see cref="SampledLoopStability.Discretise"/> (RK4 sub-steps, no
/// matrix exponential). Law (see <see cref="IAugmentedLinearisableController"/>):
/// u_k = −K x_k + L δr + Kz w_k with w_k = w_{k−1} + Ts (δr − S x_k), S = [I 0] picking the positions.
/// </para>
/// <para>
/// Without an integral the state is ξ = x (8 states for the arm) and A_ξ = Φ, B_ξ = Γ, K_ξ = K, L_ξ = L. With an
/// integral the state is ξ = [x; w_{k−1}] (12 states) and
/// <code>
/// A_ξ = [[Φ, 0], [−Ts S, I]],  B_ξ = [[Γ], [0]],  E_ξ = [[0], [Ts I]],
/// K_ξ = [K + Ts Kz S, −Kz],    L_ξ = L + Ts Kz,
/// ξ_{k+1} = (A_ξ − B_ξ K_ξ) ξ_k + (B_ξ L_ξ + E_ξ) δr,   u_k = −K_ξ ξ_k + L_ξ δr.
/// </code>
/// </para>
/// <para>
/// <b>What is approximated.</b> For the arm (<see cref="ForArm"/>) the plant is <see cref="LinearSystem.ArmAtRest"/>:
/// linearised at rest at one pose q0, friction replaced by its slope at q̇ = 0 (d = fv + fc/ωs, valid only for
/// |q̇| well below ωs = 0.1 rad/s), gravity by its gradient G (central differences), no Coriolis terms (they are
/// quadratic in q̇), no saturation or controller clamping, no encoder quantisation, noise or speed filter (a
/// perfect sensor), and every sample exactly Ts after the previous one. The integral is linearised around the value
/// that holds the equilibrium (for PID: ∫e·Ki = g(q0)), without conditional integration.
/// </para>
/// </remarks>
public sealed class SampledLoopModel
{
    private readonly double[] _phi;
    private readonly double[] _gamma;
    private readonly double[] _k;
    private readonly double[] _l;
    private readonly double[] _kz;

    private SampledLoopModel(double[] phi, double[] gamma, int plantStates, int inputs, double samplePeriod,
        double[] k, double[] l, double[]? kz)
    {
        PlantStateCount = plantStates;
        InputCount = inputs;
        SamplePeriod = samplePeriod;
        _phi = phi;
        _gamma = gamma;
        _k = k;
        _l = l;
        HasIntegral = kz is not null;
        _kz = kz ?? new double[inputs * inputs];
        if (HasIntegral && plantStates != 2 * inputs)
            throw new ArgumentException("An integral needs a plant with states [positions; speeds], nx = 2·nu.", nameof(kz));
        StateCount = plantStates + (HasIntegral ? inputs : 0);
    }

    /// <summary>States of the plant nx (8 for the arm).</summary>
    public int PlantStateCount { get; }

    /// <summary>Inputs nu (joints).</summary>
    public int InputCount { get; }

    /// <summary>Whether the model has nu integrator states.</summary>
    public bool HasIntegral { get; }

    /// <summary>States N of the closed loop: nx, or nx + nu with the integral (8 or 12 for the arm).</summary>
    public int StateCount { get; }

    /// <summary>Ts, s.</summary>
    public double SamplePeriod { get; }

    /// <summary>Φ, nx × nx row-major.</summary>
    public ReadOnlySpan<double> Phi => _phi;

    /// <summary>Γ, nx × nu row-major.</summary>
    public ReadOnlySpan<double> Gamma => _gamma;

    /// <summary>K, nu × nx row-major (the PD part of the law).</summary>
    public ReadOnlySpan<double> FeedbackGain => _k;

    /// <summary>L, nu × nu row-major.</summary>
    public ReadOnlySpan<double> ReferenceGain => _l;

    /// <summary>Kz, nu × nu row-major (zero without an integral).</summary>
    public ReadOnlySpan<double> IntegralGain => _kz;

    /// <summary>
    /// Builds the model from a continuous plant: Φ, Γ by <see cref="SampledLoopStability.Discretise"/>. SPEC §12.2.
    /// </summary>
    /// <param name="plant">The continuous plant.</param>
    /// <param name="samplePeriod">Ts, s.</param>
    /// <param name="feedbackGain">K, nu × nx.</param>
    /// <param name="referenceGain">L, nu × nu; null takes the position block of K (right when the law has no direct position term besides the error).</param>
    /// <param name="integralGain">Kz, nu × nu, or null for no integrator states.</param>
    /// <param name="substeps">RK4 sub-steps per Ts.</param>
    /// <exception cref="ArgumentException">A gain matrix has the wrong size or a value that is not finite.</exception>
    public static SampledLoopModel Create(LinearSystem plant, double samplePeriod, ReadOnlySpan<double> feedbackGain,
        ReadOnlySpan<double> referenceGain = default, double[]? integralGain = null, int substeps = SampledLoopStability.DefaultSubsteps)
    {
        ArgumentNullException.ThrowIfNull(plant);
        var (phi, gamma) = SampledLoopStability.Discretise(plant, samplePeriod, substeps);
        return Create(phi, gamma, plant.StateCount, plant.InputCount, samplePeriod, feedbackGain, referenceGain, integralGain);
    }

    /// <summary>Builds the model from Φ and Γ already computed (they do not depend on the gains).</summary>
    internal static SampledLoopModel Create(double[] phi, double[] gamma, int nx, int nu, double samplePeriod,
        ReadOnlySpan<double> feedbackGain, ReadOnlySpan<double> referenceGain, double[]? integralGain)
    {
        Check(feedbackGain, nu * nx, "K", nameof(feedbackGain));
        double[] l;
        if (referenceGain.IsEmpty)
        {
            l = new double[nu * nu];
            for (var i = 0; i < nu; i++)
                for (var j = 0; j < nu; j++) l[i * nu + j] = feedbackGain[i * nx + j];
        }
        else
        {
            Check(referenceGain, nu * nu, "L", nameof(referenceGain));
            l = referenceGain.ToArray();
        }
        if (integralGain is not null) Check(integralGain, nu * nu, "Kz", nameof(integralGain));
        return new SampledLoopModel(phi, gamma, nx, nu, samplePeriod, feedbackGain.ToArray(), l, (double[]?)integralGain?.Clone());
    }

    /// <summary>
    /// The arm linearised at rest at <paramref name="q0"/> (<see cref="LinearSystem.ArmAtRest"/>) under
    /// <paramref name="controller"/> sampled every Ts. K, L and Kz come from the controller at this Ts; a
    /// controller that is only <see cref="ILinearisableController"/> gets L = position block of K and no integral.
    /// SPEC §12.2; docs/DECISIONS.md D9. See the remarks of the class for what is approximated.
    /// </summary>
    /// <param name="arm">The true arm without payload.</param>
    /// <param name="planet">The true planet.</param>
    /// <param name="payloadKg">The true payload, kg.</param>
    /// <param name="q0">The pose, rad.</param>
    /// <param name="controller">The controller, with its own belief (SPEC §5.6).</param>
    /// <param name="samplePeriod">Ts, s.</param>
    /// <param name="substeps">RK4 sub-steps per Ts.</param>
    public static SampledLoopModel ForArm(ArmModel arm, Planet planet, double payloadKg, ReadOnlySpan<double> q0,
        ILinearisableController controller, double samplePeriod, int substeps = SampledLoopStability.DefaultSubsteps)
    {
        ArgumentNullException.ThrowIfNull(controller);
        var plant = LinearSystem.ArmAtRest(arm, planet, payloadKg, q0);
        var (phi, gamma) = SampledLoopStability.Discretise(plant, samplePeriod, substeps);
        return ForController(phi, gamma, plant.StateCount, plant.InputCount, q0, controller, samplePeriod);
    }

    /// <summary>The model of <paramref name="controller"/> on a plant already discretised.</summary>
    internal static SampledLoopModel ForController(double[] phi, double[] gamma, int nx, int nu, ReadOnlySpan<double> q0,
        ILinearisableController controller, double samplePeriod)
    {
        var k = new double[nu * nx];
        controller.LinearFeedbackGain(q0, samplePeriod, k);
        if (controller is not IAugmentedLinearisableController augmented)
            return Create(phi, gamma, nx, nu, samplePeriod, k, default, null);
        var l = new double[nu * nu];
        augmented.LinearReferenceGain(q0, samplePeriod, l);
        double[]? kz = null;
        if (augmented.HasIntegral)
        {
            kz = new double[nu * nu];
            augmented.LinearIntegralGain(q0, samplePeriod, kz);
        }
        return Create(phi, gamma, nx, nu, samplePeriod, k, l, kz);
    }

    /// <summary>A_ξ, N × N row-major (see the class remarks).</summary>
    public double[] OpenLoopMatrix()
    {
        var n = StateCount;
        var nx = PlantStateCount;
        var a = new double[n * n];
        for (var r = 0; r < nx; r++)
            for (var c = 0; c < nx; c++) a[r * n + c] = _phi[r * nx + c];
        if (HasIntegral)
            for (var i = 0; i < InputCount; i++)
            {
                a[(nx + i) * n + i] = -SamplePeriod;
                a[(nx + i) * n + nx + i] = 1.0;
            }
        return a;
    }

    /// <summary>B_ξ, N × nu row-major.</summary>
    public double[] InputMatrix()
    {
        var b = new double[StateCount * InputCount];
        Array.Copy(_gamma, b, _gamma.Length);
        return b;
    }

    /// <summary>E_ξ, N × nu row-major: how δr enters the integrators directly (zero without an integral).</summary>
    public double[] ReferenceInputMatrix()
    {
        var e = new double[StateCount * InputCount];
        if (HasIntegral)
            for (var i = 0; i < InputCount; i++) e[(PlantStateCount + i) * InputCount + i] = SamplePeriod;
        return e;
    }

    /// <summary>K_ξ, nu × N row-major: u = −K_ξ ξ + L_ξ δr.</summary>
    public double[] AugmentedFeedbackGain()
    {
        var n = StateCount;
        var nx = PlantStateCount;
        var nu = InputCount;
        var k = new double[nu * n];
        for (var i = 0; i < nu; i++)
        {
            for (var j = 0; j < nx; j++) k[i * n + j] = _k[i * nx + j];
            if (!HasIntegral) continue;
            for (var j = 0; j < nu; j++)
            {
                k[i * n + j] += SamplePeriod * _kz[i * nu + j];
                k[i * n + nx + j] = -_kz[i * nu + j];
            }
        }
        return k;
    }

    /// <summary>L_ξ, nu × nu row-major.</summary>
    public double[] AugmentedReferenceGain()
    {
        var l = (double[])_l.Clone();
        if (HasIntegral)
            for (var i = 0; i < l.Length; i++) l[i] += SamplePeriod * _kz[i];
        return l;
    }

    /// <summary>A_cl = A_ξ − B_ξ K_ξ, N × N row-major. SPEC §12.2.</summary>
    public double[] ClosedLoopMatrix() =>
        DenseMatrix.Add(OpenLoopMatrix(), DenseMatrix.Multiply(InputMatrix(), AugmentedFeedbackGain(), StateCount, InputCount, StateCount), -1.0);

    /// <summary>B_cl = B_ξ L_ξ + E_ξ, N × nu row-major: ξ_{k+1} = A_cl ξ_k + B_cl δr. SPEC §12.2.</summary>
    public double[] ClosedLoopReferenceMatrix() =>
        DenseMatrix.Add(ReferenceInputMatrix(), DenseMatrix.Multiply(InputMatrix(), AugmentedReferenceGain(), StateCount, InputCount, InputCount));

    private static void Check(ReadOnlySpan<double> m, int count, string what, string name)
    {
        if (m.Length != count)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"{what} must have {count} entries, got {m.Length}."), name);
        foreach (var v in m)
            if (!double.IsFinite(v)) throw new ArgumentException($"{what} must be finite.", name);
    }
}
