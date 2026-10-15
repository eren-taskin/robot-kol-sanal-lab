using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;

namespace RobotArm.Core.Control;

/// <summary>
/// PD with gravity compensation: u = Kp e + Kd ė + ĝ(q), clamped to ±τmax of the controller's own model.
/// e = q_d − q, ė = q̇_d − q̇, ĝ from the controller's belief (SPEC §5.6). SPEC §7.
/// </summary>
/// <remarks><see cref="Compute"/> does not allocate.</remarks>
public sealed class PdGravity : IController, IAugmentedLinearisableController
{
    private readonly int _n;
    private readonly double[] _kp;
    private readonly double[] _kd;
    private readonly double[] _torqueLimit;
    private RecursiveNewtonEuler _dynamics;
    private readonly Vector3d _gravity;
    private readonly double[] _zero;
    private readonly double[] _g;
    private readonly bool[] _clamped;

    /// <summary>Creates the controller. SPEC §7.</summary>
    /// <param name="belief">The controller's model: ĝ and τmax come from it (SPEC §5.6).</param>
    /// <param name="gains">Kp in N·m/rad and Kd in N·m·s/rad per joint.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentException">The gains are for another number of joints.</exception>
    public PdGravity(ControllerBelief belief, PdGains gains)
    {
        ArgumentNullException.ThrowIfNull(belief);
        ArgumentNullException.ThrowIfNull(gains);
        GainCheck.MatchesBelief(gains.DegreesOfFreedom, belief, nameof(gains));
        Belief = belief;
        Gains = gains;
        _n = belief.DegreesOfFreedom;
        _kp = [.. gains.Kp];
        _kd = [.. gains.Kd];
        _torqueLimit = belief.TorqueLimits();
        _dynamics = new RecursiveNewtonEuler(belief.Model);
        _gravity = belief.Planet.GravityVector;
        _zero = new double[_n];
        _g = new double[_n];
        _clamped = new bool[_n];
    }

    /// <summary>
    /// Default gains for the OpenMANIPULATOR-X (joints 1–4), found in T07 so that SPEC §7's criterion holds.
    /// Kp in N·m/rad, Kd in N·m·s/rad. SPEC §7.
    /// </summary>
    public static PdGains DefaultGains { get; } = new(
        [6.0, 6.0, 6.0, 2.0],
        [0.3, 0.6, 0.3, 0.05]);

    /// <summary>The controller's model (SPEC §5.6).</summary>
    public ControllerBelief Belief { get; private set; }

    /// <summary>Updates only the payload belief at an attachment boundary. SPEC §8.3.</summary>
    public void SetPayloadBelief(double payloadKg)
    {
        Belief = new ControllerBelief(Belief.BaseModel, Belief.Planet, payloadKg);
        _dynamics = new RecursiveNewtonEuler(Belief.Model);
    }

    /// <summary>The gains.</summary>
    public PdGains Gains { get; }

    /// <inheritdoc/>
    public int DegreesOfFreedom => _n;

    /// <inheritdoc/>
    public ReadOnlySpan<bool> Clamped => _clamped;

    /// <inheritdoc/>
    public void Reset() => Array.Clear(_clamped);

    /// <inheritdoc/>
    public void Compute(double t, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, JointReference reference, Span<double> u)
    {
        GainCheck.CheckLengths(_n, q, qd, reference, u);
        _dynamics.BiasTorque(q, _zero, _gravity, _g);
        for (var i = 0; i < _n; i++)
        {
            var v = _kp[i] * (reference.Position[i] - q[i]) + _kd[i] * (reference.Velocity[i] - qd[i]) + _g[i];
            var limit = _torqueLimit[i];
            _clamped[i] = Math.Abs(v) > limit;
            u[i] = Math.Clamp(v, -limit, limit);
        }
    }

    /// <summary>
    /// K = [diag(Kp) − Ĝ(q0) | diag(Kd)], Ĝ = ∂ĝ/∂q of the belief: ĝ(q) is computed from the sampled q and held
    /// with the rest of u, so its gradient belongs to the sampled feedback. With an exact belief, Ĝ cancels the
    /// plant's G in the closed loop. Does not depend on Ts. SPEC §7; docs/DECISIONS.md D6.
    /// </summary>
    public void LinearFeedbackGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> k)
    {
        GainCheck.CheckGainMatrix(_n, q0, samplePeriod, k);
        var gradient = Belief.GravityGradient(q0);
        k.Clear();
        for (var i = 0; i < _n; i++)
        {
            for (var j = 0; j < _n; j++)
                k[i * 2 * _n + j] = -gradient[i * _n + j];
            k[i * 2 * _n + i] += _kp[i];
            k[i * 2 * _n + _n + i] = _kd[i];
        }
    }

    /// <summary>False: PDG has no integral. SPEC §12.2.</summary>
    public bool HasIntegral => false;

    /// <summary>
    /// L = diag(Kp): ĝ depends on the measured q, not on q_d, so the reference enters only through e.
    /// SPEC §12.2; docs/DECISIONS.md D9.
    /// </summary>
    public void LinearReferenceGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> l)
    {
        GainCheck.CheckSquareMatrix(_n, q0, samplePeriod, l);
        l.Clear();
        for (var i = 0; i < _n; i++) l[i * _n + i] = _kp[i];
    }

    /// <summary>Kz = 0. SPEC §12.2.</summary>
    public void LinearIntegralGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> kz)
    {
        GainCheck.CheckSquareMatrix(_n, q0, samplePeriod, kz);
        kz.Clear();
    }
}
