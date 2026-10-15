namespace RobotArm.Core.Control;

/// <summary>
/// The reference of one controller sample: desired joint angles q_d, speeds q̇_d and accelerations q̈_d. SPEC §7.
/// A <c>ref struct</c> over spans, so a caller passes slices of its own buffers without allocating.
/// </summary>
public readonly ref struct JointReference
{
    /// <summary>Wraps the three spans, which must have the same length. SPEC §7.</summary>
    /// <exception cref="ArgumentException">The spans have different lengths.</exception>
    public JointReference(ReadOnlySpan<double> position, ReadOnlySpan<double> velocity, ReadOnlySpan<double> acceleration)
    {
        if (velocity.Length != position.Length || acceleration.Length != position.Length)
            throw new ArgumentException(
                $"q_d, q̇_d and q̈_d must have the same length, got {position.Length}, {velocity.Length} and {acceleration.Length} (SPEC §7).");
        Position = position;
        Velocity = velocity;
        Acceleration = acceleration;
    }

    /// <summary>Desired joint angles q_d, rad.</summary>
    public ReadOnlySpan<double> Position { get; }

    /// <summary>Desired joint speeds q̇_d, rad/s.</summary>
    public ReadOnlySpan<double> Velocity { get; }

    /// <summary>Desired joint accelerations q̈_d, rad/s².</summary>
    public ReadOnlySpan<double> Acceleration { get; }
}

/// <summary>
/// A joint controller: given time, measured q and q̇ and the reference, returns u (one torque per joint). SPEC §7.
/// </summary>
/// <remarks>
/// <para>
/// A controller never reads the true plant (SPEC §5.6). It sees only the measurements of a sensor (SPEC §6.3) and
/// its own model of the arm (its belief, <see cref="ControllerBelief"/>). It knows τmax from that belief, clamps
/// its own output to ±τmax and reports which joints it clamped in <see cref="Clamped"/>. It is never told whether
/// the plant saturated (docs/DECISIONS.md D6).
/// </para>
/// <para><see cref="Compute"/> must not allocate (PROJECT.md §3).</para>
/// </remarks>
public interface IController
{
    /// <summary>Number of joints n: the length of q, q̇, the reference and u.</summary>
    int DegreesOfFreedom { get; }

    /// <summary>
    /// For each joint, whether the last <see cref="Compute"/> clamped the output to ±τmax of the controller's own
    /// model. SPEC §7. An output of the controller, for logging; all false after <see cref="Reset"/>.
    /// </summary>
    ReadOnlySpan<bool> Clamped { get; }

    /// <summary>Clears the internal state (integrators, the time of the last sample, <see cref="Clamped"/>). SPEC §7.</summary>
    void Reset();

    /// <summary>Computes the torque command u. SPEC §7. Does not allocate.</summary>
    /// <param name="t">Time of the sample, s.</param>
    /// <param name="q">Measured joint angles, rad (SPEC §6.3).</param>
    /// <param name="qd">Measured or estimated joint speeds, rad/s (SPEC §6.3).</param>
    /// <param name="reference">q_d, q̇_d and q̈_d at <paramref name="t"/>.</param>
    /// <param name="u">Output: commanded joint torques, N·m, within ±τmax of the controller's model.</param>
    void Compute(double t, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, JointReference reference, Span<double> u);
}

/// <summary>
/// A controller whose law, linearised at rest at a pose q0 with a constant reference q_d = q0 and sampled every
/// Ts, is a PD-type state feedback u_k = u0 − Kq δq_k − Kq̇ δq̇_k. SPEC §7; used by
/// <see cref="SampledLoopStability"/> (docs/DECISIONS.md D6).
/// </summary>
public interface ILinearisableController
{
    /// <summary>
    /// Writes the effective gain K = [Kq | Kq̇] (n rows, 2n columns, row-major) of the law linearised at rest at
    /// <paramref name="q0"/>, so that δu = −K [δq; δq̇]. SPEC §7. Allocates; not for once per step.
    /// </summary>
    /// <param name="q0">The pose, rad; also the constant reference.</param>
    /// <param name="samplePeriod">Ts, s, for laws that depend on it (the half-sample prediction of computed torque).</param>
    /// <param name="k">Output: n × 2n gain matrix, row-major.</param>
    void LinearFeedbackGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> k);
}

/// <summary>
/// A linearisable controller that also states how the reference and an optional integral enter its law, so the
/// sampled closed loop can be built with a reference input and integrator states. SPEC §12.2;
/// docs/DECISIONS.md D9.
/// </summary>
/// <remarks>
/// Linearised at rest at q0 with the constant reference q_d = q0 + δr, the law is
/// <code>
/// δu_k = −K x_k + L δr + Kz w_k,   w_k = w_{k−1} + Ts (δr − δq_k)
/// </code>
/// with x = [δq; δq̇], K from <see cref="ILinearisableController.LinearFeedbackGain"/> (the PD part), and w the
/// deviation of the rectangle-sum integral ∫e dt from the value that holds the equilibrium. Clamping and the
/// conditional integration are left out (they are not linear), and every sample is taken Ts after the previous one
/// (the first sample after a reset, with Δt = 0, is not modelled).
/// </remarks>
public interface IAugmentedLinearisableController : ILinearisableController
{
    /// <summary>Whether the law has an integral, so the linear model has n integrator states. SPEC §12.2.</summary>
    bool HasIntegral { get; }

    /// <summary>Writes L (n × n, row-major): δu = L δr for a step δr of the constant reference. SPEC §12.2.</summary>
    /// <param name="q0">The pose, rad.</param>
    /// <param name="samplePeriod">Ts, s.</param>
    /// <param name="l">Output: n × n, row-major.</param>
    void LinearReferenceGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> l);

    /// <summary>Writes Kz (n × n, row-major), the gain on the integral state w; zero without an integral. SPEC §12.2.</summary>
    /// <param name="q0">The pose, rad.</param>
    /// <param name="samplePeriod">Ts, s.</param>
    /// <param name="kz">Output: n × n, row-major.</param>
    void LinearIntegralGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> kz);
}
