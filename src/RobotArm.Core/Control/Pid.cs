namespace RobotArm.Core.Control;

/// <summary>
/// Independent-joint PID: ui = Kp,i ei + Ki,i ∫ei dt + Kd,i ėi with e = q_d − q, ė = q̇_d − q̇, clamped to ±τmax
/// of the controller's own model, with conditional integration as anti-windup. SPEC §7; docs/DECISIONS.md D6.
/// </summary>
/// <remarks>
/// <para>
/// The integral is a rectangle sum over the time between samples: at each sample the candidate integral
/// Ii + ei·Δt is formed and the output computed with it. If that output exceeds ±τmax the controller clamps it and
/// keeps the old integral for that joint (it stops integrating while it clamps); otherwise the candidate is kept.
/// The first sample after <see cref="Reset"/> has Δt = 0. The controller never reads the plant's saturation flags.
/// </para>
/// <para>No gravity model: the integral term has to take up the gravity torque. <see cref="Compute"/> does not allocate.</para>
/// </remarks>
public sealed class Pid : IController, IAugmentedLinearisableController
{
    private readonly int _n;
    private readonly double[] _kp;
    private readonly double[] _ki;
    private readonly double[] _kd;
    private readonly double[] _torqueLimit;
    private readonly double[] _integral;
    private readonly bool[] _clamped;
    private double _lastTime;
    private bool _started;

    /// <summary>Creates the controller. SPEC §7.</summary>
    /// <param name="belief">The controller's model; only τmax of each joint is used (SPEC §5.6).</param>
    /// <param name="gains">Per-joint gains.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentException">The gains are for another number of joints.</exception>
    public Pid(ControllerBelief belief, PidGains gains)
    {
        ArgumentNullException.ThrowIfNull(belief);
        ArgumentNullException.ThrowIfNull(gains);
        GainCheck.MatchesBelief(gains.DegreesOfFreedom, belief, nameof(gains));
        Belief = belief;
        Gains = gains;
        _n = belief.DegreesOfFreedom;
        _kp = [.. gains.Kp];
        _ki = [.. gains.Ki];
        _kd = [.. gains.Kd];
        _torqueLimit = belief.TorqueLimits();
        _integral = new double[_n];
        _clamped = new bool[_n];
    }

    /// <summary>
    /// Default gains for the OpenMANIPULATOR-X (joints 1–4), found in T07 so that SPEC §7's criterion holds.
    /// Kp in N·m/rad, Ki in N·m/(rad·s), Kd in N·m·s/rad. SPEC §7.
    /// </summary>
    public static PidGains DefaultGains { get; } = new(
        [6.0, 6.0, 6.0, 2.0],
        [5.0, 20.0, 20.0, 8.0],
        [0.3, 0.6, 0.3, 0.05]);

    /// <summary>The controller's model (SPEC §5.6).</summary>
    public ControllerBelief Belief { get; private set; }

    /// <summary>Records an informed payload update without resetting the integrator; PID uses only torque limits. SPEC §8.3.</summary>
    public void SetPayloadBelief(double payloadKg) => Belief = new ControllerBelief(Belief.BaseModel, Belief.Planet, payloadKg);

    /// <summary>The gains.</summary>
    public PidGains Gains { get; }

    /// <inheritdoc/>
    public int DegreesOfFreedom => _n;

    /// <inheritdoc/>
    public ReadOnlySpan<bool> Clamped => _clamped;

    /// <summary>The integral ∫e dt of each joint, rad·s.</summary>
    public ReadOnlySpan<double> Integral => _integral;

    /// <inheritdoc/>
    public void Reset()
    {
        Array.Clear(_integral);
        Array.Clear(_clamped);
        _started = false;
        _lastTime = 0.0;
    }

    /// <inheritdoc/>
    public void Compute(double t, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, JointReference reference, Span<double> u)
    {
        GainCheck.CheckLengths(_n, q, qd, reference, u);
        var dt = _started ? t - _lastTime : 0.0;
        _started = true;
        _lastTime = t;

        for (var i = 0; i < _n; i++)
        {
            var e = reference.Position[i] - q[i];
            var eDot = reference.Velocity[i] - qd[i];
            var candidate = _integral[i] + e * dt;
            var v = _kp[i] * e + _ki[i] * candidate + _kd[i] * eDot;
            var limit = _torqueLimit[i];
            if (Math.Abs(v) > limit)
            {
                u[i] = Math.Clamp(v, -limit, limit);
                _clamped[i] = true; // conditional integration: keep the old integral
            }
            else
            {
                u[i] = v;
                _clamped[i] = false;
                _integral[i] = candidate;
            }
        }
    }

    /// <summary>
    /// K = [diag(Kp) | diag(Kd)]: the PD part only. The integral is left out (Ki is small against Kp at the sample
    /// rates of interest), and the plant's gravity gradient is not compensated, so it stays in the plant model of
    /// <see cref="LinearSystem.ArmAtRest"/>. Does not depend on Ts. SPEC §7; docs/DECISIONS.md D6.
    /// </summary>
    public void LinearFeedbackGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> k)
    {
        GainCheck.CheckGainMatrix(_n, q0, samplePeriod, k);
        k.Clear();
        for (var i = 0; i < _n; i++)
        {
            k[i * 2 * _n + i] = _kp[i];
            k[i * 2 * _n + _n + i] = _kd[i];
        }
    }

    /// <summary>Always true: the linear model keeps the n integrator states, even where Ki = 0. SPEC §12.2.</summary>
    public bool HasIntegral => true;

    /// <summary>L = diag(Kp): the reference enters only through e. SPEC §12.2; docs/DECISIONS.md D9.</summary>
    public void LinearReferenceGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> l) =>
        Diagonal(q0, samplePeriod, l, _kp);

    /// <summary>Kz = diag(Ki). SPEC §12.2; docs/DECISIONS.md D9.</summary>
    public void LinearIntegralGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> kz) =>
        Diagonal(q0, samplePeriod, kz, _ki);

    private void Diagonal(ReadOnlySpan<double> q0, double samplePeriod, Span<double> m, double[] diagonal)
    {
        GainCheck.CheckSquareMatrix(_n, q0, samplePeriod, m);
        m.Clear();
        for (var i = 0; i < _n; i++) m[i * _n + i] = diagonal[i];
    }
}
