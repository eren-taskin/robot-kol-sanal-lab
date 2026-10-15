using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;

namespace RobotArm.Core.Control;

/// <summary>
/// Computed torque: u = M̂(q̄)(q̈_d + Kd ė + Kp e) + ĥ(q̄, q̄̇) + τ̂f(q̄̇), clamped to ±τmax of the controller's own
/// model. e = q_d − q, ė = q̇_d − q̇ from the measured q and q̇; M̂, ĥ and τ̂f from the controller's belief
/// (SPEC §5.6); (q̄, q̄̇) is the measured state, optionally predicted to the middle of the hold. SPEC §7;
/// docs/DECISIONS.md D7.
/// </summary>
/// <remarks>
/// <para>
/// a = q̈_d + Kd ė + Kp e is the commanded acceleration. M̂(q̄) a + ĥ(q̄, q̄̇) is one inverse-dynamics call
/// (RNEA, SPEC §5.2). The model's friction τ̂f,i = fv q̄̇i + fc tanh(q̄̇i/ωs) (SPEC §5.4, the belief's actuators)
/// is part of the nonlinear terms that computed torque cancels. Without it the friction stays in the loop: near
/// rest it is damping d = fv + fc/ωs = 0.21 N·m·s/rad, which on the light wrist joint (d/M44 ≈ 570 1/s) swamps Kd
/// and turns the designed loop ë + Kd ė + Kp e = 0 into a mode that creeps with a time constant of seconds.
/// </para>
/// <para>
/// <b>Half-sample prediction</b> (<see cref="PredictHalfSample"/>, on by default). u is held for Ts, but the
/// torque the model asks for changes during the hold. On the wrist this matters: M44 ≈ 3.7e-4 kg·m², so a gravity
/// torque that goes stale while θ4 = q2 + q3 + q4 turns at 2–3 rad/s, or friction that changes within its
/// 1.8 ms time constant M44/d, gives tracking errors of several 1e-3 rad at Ts = 2 ms. With the prediction the
/// model is evaluated at the state expected half-way through the hold, if the joints accelerate at a:
/// q̄ = q + q̇ Δt/2 + a Δt²/8, q̄̇ = q̇ + a Δt/2, with Δt the time since the previous sample (0 on the first sample
/// after <see cref="Reset"/>, so then q̄ = q). Without it (q̄, q̄̇) = (q, q̇), the textbook law.
/// </para>
/// <para>Kp is in 1/s² and Kd in 1/s. <see cref="Compute"/> does not allocate.</para>
/// </remarks>
public sealed class ComputedTorque : IController, ILinearisableController
{
    private readonly int _n;
    private readonly double[] _kp;
    private readonly double[] _kd;
    private readonly double[] _torqueLimit;
    private readonly double[] _viscousFriction;
    private readonly double[] _coulombFriction;
    private readonly double[] _smoothingSpeed;
    private readonly RecursiveNewtonEuler _dynamics;
    private readonly Vector3d _gravity;
    private readonly double[] _acceleration;
    private readonly double[] _qPredicted;
    private readonly double[] _qdPredicted;
    private readonly bool[] _clamped;
    private double _lastTime;
    private bool _started;

    /// <summary>Creates the controller. SPEC §7.</summary>
    /// <param name="belief">The controller's model: M̂, ĥ, τ̂f and τmax come from it (SPEC §5.6).</param>
    /// <param name="gains">Kp in 1/s² and Kd in 1/s per joint.</param>
    /// <param name="predictHalfSample">Evaluate the model at the state predicted for the middle of the hold.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentException">The gains are for another number of joints.</exception>
    public ComputedTorque(ControllerBelief belief, PdGains gains, bool predictHalfSample = true)
    {
        ArgumentNullException.ThrowIfNull(belief);
        ArgumentNullException.ThrowIfNull(gains);
        GainCheck.MatchesBelief(gains.DegreesOfFreedom, belief, nameof(gains));
        Belief = belief;
        Gains = gains;
        PredictHalfSample = predictHalfSample;
        _n = belief.DegreesOfFreedom;
        _kp = [.. gains.Kp];
        _kd = [.. gains.Kd];
        _torqueLimit = belief.TorqueLimits();
        _viscousFriction = new double[_n];
        _coulombFriction = new double[_n];
        _smoothingSpeed = new double[_n];
        for (var i = 0; i < _n; i++)
        {
            var actuator = belief.Model.Actuators[i];
            _viscousFriction[i] = actuator.ViscousFriction;
            _coulombFriction[i] = actuator.CoulombFriction;
            _smoothingSpeed[i] = actuator.SmoothingSpeed;
        }
        _dynamics = new RecursiveNewtonEuler(belief.Model);
        _gravity = belief.Planet.GravityVector;
        _acceleration = new double[_n];
        _qPredicted = new double[_n];
        _qdPredicted = new double[_n];
        _clamped = new bool[_n];
    }

    /// <summary>
    /// Default gains for the OpenMANIPULATOR-X (joints 1–4), found in T07 so that SPEC §7's criterion holds:
    /// ωn = 12 rad/s and ζ = 1 on every joint. Kp in 1/s², Kd in 1/s. SPEC §7.
    /// </summary>
    public static PdGains DefaultGains { get; } = new(
        [144.0, 144.0, 144.0, 144.0],
        [24.0, 24.0, 24.0, 24.0]);

    /// <summary>The controller's model (SPEC §5.6).</summary>
    public ControllerBelief Belief { get; }

    /// <summary>The gains.</summary>
    public PdGains Gains { get; }

    /// <summary>Whether the model is evaluated at the state predicted for the middle of the hold (see remarks).</summary>
    public bool PredictHalfSample { get; }

    /// <inheritdoc/>
    public int DegreesOfFreedom => _n;

    /// <inheritdoc/>
    public ReadOnlySpan<bool> Clamped => _clamped;

    /// <inheritdoc/>
    public void Reset()
    {
        Array.Clear(_clamped);
        _started = false;
        _lastTime = 0.0;
    }

    /// <inheritdoc/>
    public void Compute(double t, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, JointReference reference, Span<double> u)
    {
        GainCheck.CheckLengths(_n, q, qd, reference, u);
        var dt = _started && PredictHalfSample ? t - _lastTime : 0.0;
        _started = true;
        _lastTime = t;

        for (var i = 0; i < _n; i++)
        {
            var a = reference.Acceleration[i]
                + _kd[i] * (reference.Velocity[i] - qd[i])
                + _kp[i] * (reference.Position[i] - q[i]);
            _acceleration[i] = a;
            _qPredicted[i] = q[i] + 0.5 * dt * qd[i] + 0.125 * dt * dt * a;
            _qdPredicted[i] = qd[i] + 0.5 * dt * a;
        }

        _dynamics.Inverse(_qPredicted, _qdPredicted, _acceleration, _gravity, u); // M̂(q̄) a + ĥ(q̄, q̄̇)

        for (var i = 0; i < _n; i++)
        {
            var speed = _qdPredicted[i];
            var v = u[i] + _viscousFriction[i] * speed + _coulombFriction[i] * Math.Tanh(speed / _smoothingSpeed[i]);
            var limit = _torqueLimit[i];
            _clamped[i] = Math.Abs(v) > limit;
            u[i] = Math.Clamp(v, -limit, limit);
        }
    }

    /// <summary>
    /// K = [P diag(Kp) − Ĝ | P diag(Kd) − Ĝ Ts/2 − D̂] with P = M̂(q0) + Ĝ Ts²/8 + D̂ Ts/2, where Ĝ = ∂ĝ/∂q at q0
    /// and D̂ = diag(fv + fc/ωs) of the belief, and Ts = 0 when <see cref="PredictHalfSample"/> is off.
    /// </summary>
    /// <remarks>
    /// At rest e = 0 and a = 0, so the change of M̂(q̄) does not enter, and ĥ has no term linear in q̇ (Coriolis
    /// terms are quadratic). The rest follows from δa = −Kp δq − Kd δq̇, δq̄ = δq + δq̇ Ts/2 + δa Ts²/8 and
    /// δq̄̇ = δq̇ + δa Ts/2. Ĝ and D̂ are sampled and held like the rest of u, so with an exact belief they cancel
    /// the plant's continuous G and D only approximately. Without prediction, K = [M̂ Kp − Ĝ | M̂ Kd − D̂].
    /// SPEC §7; docs/DECISIONS.md D6.
    /// </remarks>
    public void LinearFeedbackGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> k)
    {
        GainCheck.CheckGainMatrix(_n, q0, samplePeriod, k);
        var ts = PredictHalfSample ? samplePeriod : 0.0;
        var m = new double[_n * _n];
        _dynamics.InertiaMatrix(q0, m);
        var gradient = Belief.GravityGradient(q0);
        var width = 2 * _n;
        for (var i = 0; i < _n; i++)
        {
            var d = _viscousFriction[i] + _coulombFriction[i] / _smoothingSpeed[i];
            for (var j = 0; j < _n; j++)
            {
                var p = m[i * _n + j] + gradient[i * _n + j] * ts * ts / 8.0 + (i == j ? d * ts / 2.0 : 0.0);
                k[i * width + j] = p * _kp[j] - gradient[i * _n + j];
                k[i * width + _n + j] = p * _kd[j] - gradient[i * _n + j] * ts / 2.0 - (i == j ? d : 0.0);
            }
        }
    }
}
