using System.Globalization;
using RobotArm.Core.Control;
using RobotArm.Core.Model;

namespace RobotArm.Core.Analysis;

/// <summary>
/// Response of the linear sampled closed loop to a step of the constant joint reference, at the controller samples,
/// to overlay on a nonlinear run. SPEC §12.5; docs/DECISIONS.md D9.
/// </summary>
/// <remarks>
/// <para>
/// From rest at the equilibrium q0 (ξ0 = 0), the reference jumps to q0 + δr at t = 0 (q̇_d = q̈_d = 0):
/// ξ_{k+1} = A_cl ξ_k + B_cl δr and u_k = u0 − K_ξ ξ_k + L_ξ δr (<see cref="SampledLoopModel"/>). Sample k is at
/// t = k·Ts and describes the state before the command of that sample, like a log row (SPEC §10).
/// </para>
/// <para>
/// <b>What is approximated</b>: everything listed in <see cref="SampledLoopModel"/>. The prediction is good while the
/// step is small: M, G and the friction slope are those of q0, while the nonlinear arm moves to q0 + δr, its friction
/// leaves the linear band once |q̇| approaches ωs = 0.1 rad/s, and a large step makes the controller clamp.
/// </para>
/// </remarks>
public sealed class LinearStepResponse
{
    private readonly double[] _positions;
    private readonly double[] _velocities;
    private readonly double[] _commands;

    private LinearStepResponse(int samples, int joints, double samplePeriod, double[] positions, double[] velocities, double[] commands)
    {
        SampleCount = samples;
        JointCount = joints;
        SamplePeriod = samplePeriod;
        _positions = positions;
        _velocities = velocities;
        _commands = commands;
    }

    /// <summary>Number of samples (t = 0 included).</summary>
    public int SampleCount { get; }

    /// <summary>Number of joints n.</summary>
    public int JointCount { get; }

    /// <summary>Ts, s.</summary>
    public double SamplePeriod { get; }

    /// <summary>Time of sample k, s.</summary>
    public double Time(int sample) => sample * SamplePeriod;

    /// <summary>Predicted q = q0 + δq, sample × joint row-major, rad.</summary>
    public ReadOnlySpan<double> Positions => _positions;

    /// <summary>Predicted q̇, sample × joint row-major, rad/s.</summary>
    public ReadOnlySpan<double> Velocities => _velocities;

    /// <summary>Predicted change of the command δu = u − u0 at each sample, sample × joint row-major, N·m.</summary>
    public ReadOnlySpan<double> CommandChanges => _commands;

    /// <summary>Predicted q of one joint at one sample, rad.</summary>
    public double Position(int sample, int joint) => _positions[sample * JointCount + joint];

    /// <summary>
    /// The step response of <paramref name="model"/> (plant states [δq; δq̇]) over <paramref name="duration"/>.
    /// SPEC §12.5.
    /// </summary>
    /// <param name="model">The sampled loop.</param>
    /// <param name="q0">The equilibrium pose, rad (added to δq).</param>
    /// <param name="step">δr per joint, rad.</param>
    /// <param name="duration">Length, s; samples k = 0 … floor(duration/Ts).</param>
    /// <exception cref="ArgumentException">A vector has the wrong length or the plant states are not [positions; speeds].</exception>
    /// <exception cref="ArgumentOutOfRangeException">The duration is not positive and finite.</exception>
    public static LinearStepResponse Compute(SampledLoopModel model, ReadOnlySpan<double> q0, ReadOnlySpan<double> step, double duration)
    {
        ArgumentNullException.ThrowIfNull(model);
        var nu = model.InputCount;
        if (model.PlantStateCount != 2 * nu)
            throw new ArgumentException("The plant states must be [positions; speeds].", nameof(model));
        if (q0.Length != nu || step.Length != nu)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Expected {nu} values in q0 and the step."));
        if (!double.IsFinite(duration) || duration <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(duration), duration, "Must be a finite number greater than 0 s.");

        var n = model.StateCount;
        var acl = model.ClosedLoopMatrix();
        var bcl = model.ClosedLoopReferenceMatrix();
        var k = model.AugmentedFeedbackGain();
        var l = model.AugmentedReferenceGain();
        var drive = DenseMatrix.Multiply(bcl, step, n, nu, 1);
        var feedforward = DenseMatrix.Multiply(l, step, nu, nu, 1);

        var samples = (int)Math.Floor(duration / model.SamplePeriod + 1e-9) + 1;
        var positions = new double[samples * nu];
        var velocities = new double[samples * nu];
        var commands = new double[samples * nu];
        var xi = new double[n];
        var next = new double[n];
        for (var s = 0; s < samples; s++)
        {
            for (var i = 0; i < nu; i++)
            {
                positions[s * nu + i] = q0[i] + xi[i];
                velocities[s * nu + i] = xi[nu + i];
                var u = feedforward[i];
                for (var j = 0; j < n; j++) u -= k[i * n + j] * xi[j];
                commands[s * nu + i] = u;
            }
            for (var r = 0; r < n; r++)
            {
                var sum = drive[r];
                for (var c = 0; c < n; c++) sum += acl[r * n + c] * xi[c];
                next[r] = sum;
            }
            (xi, next) = (next, xi);
        }
        return new LinearStepResponse(samples, nu, model.SamplePeriod, positions, velocities, commands);
    }

    /// <summary>
    /// The step response of the arm under <paramref name="controller"/> linearised at rest at <paramref name="q0"/>.
    /// SPEC §12.5.
    /// </summary>
    public static LinearStepResponse Compute(ArmModel arm, Planet planet, double payloadKg, ReadOnlySpan<double> q0,
        ILinearisableController controller, double samplePeriod, ReadOnlySpan<double> step, double duration) =>
        Compute(SampledLoopModel.ForArm(arm, planet, payloadKg, q0, controller, samplePeriod), q0, step, duration);
}
