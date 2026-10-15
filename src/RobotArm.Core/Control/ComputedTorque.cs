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
/// a = q̈_d + Kd ė + Kp e + Ki ∫e is the commanded acceleration; Ki is zero unless explicitly enabled.
/// CT+I uses conditional integration per joint: a clamped output keeps that joint's previous integral (D8). M̂(q̄) a + ĥ(q̄, q̄̇) is one inverse-dynamics call
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
public sealed class ComputedTorque : IController, IAugmentedLinearisableController
{
    private readonly int _n;
    private readonly double[] _kp;
    private readonly double[] _kd;
    private readonly double[] _torqueLimit;
    private readonly double[] _viscousFriction;
    private readonly double[] _coulombFriction;
    private readonly double[] _smoothingSpeed;
    private RecursiveNewtonEuler _dynamics;
    private readonly double[] _ki;
    private readonly double[] _integral;
    private readonly double[] _candidate;
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
    /// <param name="integralGains">Optional Ki in 1/s³; null disables integration (SPEC §7, D8).</param>
    /// <param name="predictHalfSample">Evaluate the model at the state predicted for the middle of the hold.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentException">The gains are for another number of joints.</exception>
    public ComputedTorque(ControllerBelief belief, PdGains gains, bool predictHalfSample = true, IEnumerable<double>? integralGains = null)
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
        _ki = integralGains is null ? new double[_n] : [.. GainCheck.Copy(integralGains, nameof(integralGains))];
        GainCheck.SameLength(_n, _ki.Length, nameof(integralGains));
        IntegralEnabled = integralGains is not null;
        _integral = new double[_n];
        _candidate = new double[_n];
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
    /// Default gains for the OpenMANIPULATOR-X (joints 1–4), retuned in A1c so that every joint has at least 6 dB gain
    /// margin and 40° phase margin across all library start poses, planets and payloads (0 and 0.5 kg) while keeping
    /// the SPEC §7 step settling time under 1 s and V18 quintic tracking error below 1e-3 rad:
    /// Kp = [1600, 1600, 1800, 1800] 1/s², Kd = [350, 350, 350, 350] 1/s. SPEC §7; docs/DECISIONS.md A1c.
    /// </summary>
    public static PdGains DefaultGains { get; } = new(
        [1600.0, 1600.0, 1800.0, 1800.0],
        [350.0, 350.0, 350.0, 350.0]);

    /// <summary>The controller's model (SPEC §5.6).</summary>
    public ControllerBelief Belief { get; private set; }

    /// <summary>Whether CT+I is enabled. Off by default. SPEC §7.</summary>
    public bool IntegralEnabled { get; }

    /// <summary>Ki per joint, 1/s³; all zero when <see cref="IntegralEnabled"/> is false. SPEC §7.</summary>
    public IReadOnlyList<double> IntegralGains => Array.AsReadOnly((double[])_ki.Clone());

    /// <summary>Integral error per joint, rad·s. SPEC §7.</summary>
    public ReadOnlySpan<double> Integral => _integral;

    /// <summary>Default optional Ki, 1/s³. SPEC §7.</summary>
    public static IReadOnlyList<double> DefaultIntegralGains { get; } = Array.AsReadOnly(new[] { 10000.0, 10000.0, 10000.0, 10000.0 });

    /// <summary>PD gains tuned with DefaultIntegralGains for the CT+I robustness option: Kp=3600 1/s², Kd=360 1/s. SPEC §7.</summary>
    public static PdGains DefaultIntegralPdGains { get; } = new(
        [3600.0, 3600.0, 3600.0, 3600.0], [360.0, 360.0, 360.0, 360.0]);

    /// <summary>Updates the payload belief at an attachment boundary without resetting the integral or sample history. SPEC §8.3.</summary>
    public void SetPayloadBelief(double payloadKg)
    {
        Belief = new ControllerBelief(Belief.BaseModel, Belief.Planet, payloadKg);
        _dynamics = new RecursiveNewtonEuler(Belief.Model);
    }

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
        Array.Clear(_integral);
        _started = false;
        _lastTime = 0.0;
    }

    /// <inheritdoc/>
    public void Compute(double t, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, JointReference reference, Span<double> u)
    {
        GainCheck.CheckLengths(_n, q, qd, reference, u);
        var elapsed = _started ? t - _lastTime : 0.0;
        var dt = PredictHalfSample ? elapsed : 0.0;
        _started = true;
        _lastTime = t;

        for (var i = 0; i < _n; i++)
        {
            _candidate[i] = _integral[i] + (IntegralEnabled ? (reference.Position[i] - q[i]) * elapsed : 0.0);
            var a = _ki[i] * _candidate[i] + reference.Acceleration[i]
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
            if (!_clamped[i]) _integral[i] = _candidate[i];
        }
    }

    /// <summary>
    /// PD part only; the integral state is excluded, as for PID. SPEC §7.
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
        var p = PredictionInertia(q0, ts, out var gradient);
        var width = 2 * _n;
        for (var i = 0; i < _n; i++)
        {
            var d = _viscousFriction[i] + _coulombFriction[i] / _smoothingSpeed[i];
            for (var j = 0; j < _n; j++)
            {
                k[i * width + j] = p[i * _n + j] * _kp[j] - gradient[i * _n + j];
                k[i * width + _n + j] = p[i * _n + j] * _kd[j] - gradient[i * _n + j] * ts / 2.0 - (i == j ? d : 0.0);
            }
        }
    }

    /// <summary>True when CT+I is enabled. SPEC §12.2.</summary>
    public bool HasIntegral => IntegralEnabled;

    /// <summary>
    /// L = P diag(Kp), with P of <see cref="LinearFeedbackGain"/>: a step of q_d changes a by Kp δr, and a enters
    /// u through M̂ and through the predicted state. SPEC §12.2; docs/DECISIONS.md D9.
    /// </summary>
    public void LinearReferenceGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> l) =>
        PredictionTimesDiagonal(q0, samplePeriod, l, _kp);

    /// <summary>
    /// Kz = P diag(Ki) (zero without CT+I): the integral enters u through a, like the reference. The integral
    /// itself always uses the elapsed sample time, even when the prediction is off. SPEC §12.2; docs/DECISIONS.md D9.
    /// </summary>
    public void LinearIntegralGain(ReadOnlySpan<double> q0, double samplePeriod, Span<double> kz) =>
        PredictionTimesDiagonal(q0, samplePeriod, kz, _ki);

    private void PredictionTimesDiagonal(ReadOnlySpan<double> q0, double samplePeriod, Span<double> m, double[] diagonal)
    {
        GainCheck.CheckSquareMatrix(_n, q0, samplePeriod, m);
        var p = PredictionInertia(q0, PredictHalfSample ? samplePeriod : 0.0, out _);
        for (var i = 0; i < _n; i++)
            for (var j = 0; j < _n; j++)
                m[i * _n + j] = p[i * _n + j] * diagonal[j];
    }

    /// <summary>P = M̂(q0) + Ĝ ts²/8 + D̂ ts/2, n × n row-major; also returns Ĝ. Allocates.</summary>
    private double[] PredictionInertia(ReadOnlySpan<double> q0, double ts, out double[] gradient)
    {
        var p = new double[_n * _n];
        _dynamics.InertiaMatrix(q0, p);
        gradient = Belief.GravityGradient(q0);
        for (var i = 0; i < _n; i++)
        {
            var d = _viscousFriction[i] + _coulombFriction[i] / _smoothingSpeed[i];
            for (var j = 0; j < _n; j++)
                p[i * _n + j] += gradient[i * _n + j] * ts * ts / 8.0 + (i == j ? d * ts / 2.0 : 0.0);
        }
        return p;
    }
}
