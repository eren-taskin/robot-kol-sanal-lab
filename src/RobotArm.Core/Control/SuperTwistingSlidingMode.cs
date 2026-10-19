using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;

namespace RobotArm.Core.Control;

/// <summary>
/// Per-joint gains of the super-twisting sliding mode controller: the sliding-surface slope Λ (1/s), the
/// square-root gain k1 (rad^½/s^{3/2}) and the switching gain k2 (rad/s³). SPEC §7; docs/DECISIONS.md D13.
/// </summary>
public sealed record SuperTwistingGains
{
    /// <summary>Creates the gains; the arrays are copied. SPEC §7.</summary>
    /// <exception cref="ArgumentException">The arrays are empty or have different lengths.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A gain is negative or not finite.</exception>
    public SuperTwistingGains(IEnumerable<double> lambda, IEnumerable<double> k1, IEnumerable<double> k2)
    {
        Lambda = GainCheck.Copy(lambda, nameof(lambda));
        K1 = GainCheck.Copy(k1, nameof(k1));
        K2 = GainCheck.Copy(k2, nameof(k2));
        GainCheck.SameLength(Lambda.Count, K1.Count, nameof(k1));
        GainCheck.SameLength(Lambda.Count, K2.Count, nameof(k2));
    }

    /// <summary>Sliding-surface slope Λ of each joint, s = ė + Λe, 1/s.</summary>
    public IReadOnlyList<double> Lambda { get; }

    /// <summary>Gain k1 of the k1·|s|^½·sign(s) term of each joint.</summary>
    public IReadOnlyList<double> K1 { get; }

    /// <summary>Gain k2 of ẇ = k2·sign(s) of each joint, rad/s³.</summary>
    public IReadOnlyList<double> K2 { get; }

    /// <summary>Number of joints.</summary>
    public int DegreesOfFreedom => Lambda.Count;
}

/// <summary>
/// Super-twisting sliding mode control (STSMC) on top of the controller's own inverse dynamics:
/// u = M̂(q̄)(q̈_d + Λė + v) + ĥ(q̄, q̄̇) + τ̂f(q̄̇), clamped to ±τmax of the controller's own model, with the
/// sliding variable s = ė + Λe and the super-twisting correction v = k1·|s|^½·sign(s) + w, ẇ = k2·sign(s).
/// SPEC §7; docs/DECISIONS.md D13.
/// </summary>
/// <remarks>
/// <para>
/// e = q_d − q and ė = q̇_d − q̇ from the measured q and q̇. With an exact model the inner loop gives q̈ = q̈_d + Λė + v,
/// so ṡ = −v: the classic super-twisting loop, which drives s and ṡ to zero in finite time against a perturbation
/// whose time derivative is bounded by L if k2 &gt; L (k1 = 1.5√L, k2 = 1.1L is the standard choice). A wrong
/// model enters as that perturbation (D13 measures its L). Once s = 0, e decays as ė = −Λe.
/// </para>
/// <para>
/// <b>Sampling.</b> w is integrated once per sample with the elapsed time Δt since the previous sample (rectangle
/// rule, nothing on the first sample after <see cref="Reset"/>): w ← w + Δt·k2·φ(s), using the s of this sample.
/// The model is evaluated exactly as by <see cref="ComputedTorque"/>, at the state predicted for the middle of the
/// hold when <see cref="PredictHalfSample"/> is on (D7): with a = q̈_d + Λė + v, q̄ = q + q̇·Δt/2 + a·Δt²/8 and
/// q̄̇ = q̇ + a·Δt/2. Sampled, the super-twisting loop does not reach s = 0 but a band |s| = O(Ts²), |ṡ| = O(Ts).
/// </para>
/// <para>
/// <b>Anti-windup.</b> As in CT+I: a joint whose output is clamped keeps its previous w for that sample.
/// </para>
/// <para>
/// <b>Boundary layer.</b> With <see cref="BoundaryLayer"/> ε &gt; 0, sign(s) is replaced by s/(|s| + ε) in both
/// terms. ε = 0 (the default) is the pure super-twisting law.
/// </para>
/// <para>
/// <b>Not linearisable.</b> |s|^½·sign(s) has an infinite slope at s = 0, so the law has no linearisation at rest and
/// the controller deliberately does not implement <see cref="ILinearisableController"/>: poles, root locus, Bode
/// margins and the gain tuner of SPEC §12 do not apply to it.
/// </para>
/// <para><see cref="Compute"/> does not allocate.</para>
/// </remarks>
public sealed class SuperTwistingSlidingMode : IController
{
    private readonly int _n;
    private readonly double[] _lambda;
    private readonly double[] _k1;
    private readonly double[] _k2;
    private readonly double[] _torqueLimit;
    private readonly double[] _viscousFriction;
    private readonly double[] _coulombFriction;
    private readonly double[] _smoothingSpeed;
    private RecursiveNewtonEuler _dynamics;
    private readonly Vector3d _gravity;
    private readonly double[] _w;
    private readonly double[] _candidate;
    private readonly double[] _s;
    private readonly double[] _acceleration;
    private readonly double[] _qPredicted;
    private readonly double[] _qdPredicted;
    private readonly bool[] _clamped;
    private double _lastTime;
    private bool _started;

    /// <summary>Creates the controller. SPEC §7.</summary>
    /// <param name="belief">The controller's model: M̂, ĥ, τ̂f and τmax come from it (SPEC §5.6).</param>
    /// <param name="gains">Λ, k1 and k2 per joint.</param>
    /// <param name="boundaryLayer">ε ≥ 0 of s/(|s| + ε) in place of sign(s), rad/s; 0 is the pure law.</param>
    /// <param name="predictHalfSample">Evaluate the model at the state predicted for the middle of the hold (D7).</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentException">The gains are for another number of joints.</exception>
    /// <exception cref="ArgumentOutOfRangeException">ε is negative or not finite.</exception>
    public SuperTwistingSlidingMode(ControllerBelief belief, SuperTwistingGains gains, double boundaryLayer = 0.0, bool predictHalfSample = true)
    {
        ArgumentNullException.ThrowIfNull(belief);
        ArgumentNullException.ThrowIfNull(gains);
        GainCheck.MatchesBelief(gains.DegreesOfFreedom, belief, nameof(gains));
        if (!double.IsFinite(boundaryLayer) || boundaryLayer < 0.0)
            throw new ArgumentOutOfRangeException(nameof(boundaryLayer), boundaryLayer, "The boundary layer ε must be a finite number of at least 0 (SPEC §7).");
        Belief = belief;
        Gains = gains;
        BoundaryLayer = boundaryLayer;
        PredictHalfSample = predictHalfSample;
        _n = belief.DegreesOfFreedom;
        _lambda = [.. gains.Lambda];
        _k1 = [.. gains.K1];
        _k2 = [.. gains.K2];
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
        _w = new double[_n];
        _candidate = new double[_n];
        _s = new double[_n];
        _acceleration = new double[_n];
        _qPredicted = new double[_n];
        _qdPredicted = new double[_n];
        _clamped = new bool[_n];
    }

    /// <summary>
    /// Default gains for the OpenMANIPULATOR-X (joints 1–4): Λ = 20 1/s, and the standard choice k1 = 1.5√L,
    /// k2 = 1.1L with the bound L = [93.8, 93.8, 199.0, 1013.0] rad/s³ on the perturbation's derivative, measured for
    /// the "0,3 kg habersiz yük" case along the library path (joint 1 does not move there, so it takes joint 2's L):
    /// k1 = [14.53, 14.53, 21.16, 47.74], k2 = [103.2, 103.2, 218.9, 1114.3]. SPEC §7; docs/DECISIONS.md D13.
    /// </summary>
    public static SuperTwistingGains DefaultGains { get; } = new(
        [20.0, 20.0, 20.0, 20.0],
        [14.53, 14.53, 21.16, 47.74],
        [103.2, 103.2, 218.9, 1114.3]);

    /// <summary>The controller's model (SPEC §5.6).</summary>
    public ControllerBelief Belief { get; private set; }

    /// <summary>The gains.</summary>
    public SuperTwistingGains Gains { get; }

    /// <summary>ε of the boundary layer, rad/s; 0 is the pure super-twisting law.</summary>
    public double BoundaryLayer { get; }

    /// <summary>Whether the model is evaluated at the state predicted for the middle of the hold (D7).</summary>
    public bool PredictHalfSample { get; }

    /// <summary>The integral term w of each joint after the last sample, rad/s².</summary>
    public ReadOnlySpan<double> W => _w;

    /// <summary>The sliding variable s = ė + Λe of each joint at the last sample, rad/s.</summary>
    public ReadOnlySpan<double> SlidingVariable => _s;

    /// <inheritdoc/>
    public int DegreesOfFreedom => _n;

    /// <inheritdoc/>
    public ReadOnlySpan<bool> Clamped => _clamped;

    /// <summary>Updates the payload belief at an attachment boundary without resetting w or the sample history. SPEC §8.3.</summary>
    public void SetPayloadBelief(double payloadKg)
    {
        Belief = new ControllerBelief(Belief.BaseModel, Belief.Planet, payloadKg);
        _dynamics = new RecursiveNewtonEuler(Belief.Model);
    }

    /// <inheritdoc/>
    public void Reset()
    {
        Array.Clear(_clamped);
        Array.Clear(_w);
        Array.Clear(_s);
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
            var e = reference.Position[i] - q[i];
            var eDot = reference.Velocity[i] - qd[i];
            var s = eDot + _lambda[i] * e;
            var phi = Switch(s, BoundaryLayer);
            _s[i] = s;
            _candidate[i] = _w[i] + elapsed * _k2[i] * phi;
            var v = _k1[i] * Math.Sqrt(Math.Abs(s)) * phi + _candidate[i];
            var a = reference.Acceleration[i] + _lambda[i] * eDot + v;
            _acceleration[i] = a;
            _qPredicted[i] = q[i] + 0.5 * dt * qd[i] + 0.125 * dt * dt * a;
            _qdPredicted[i] = qd[i] + 0.5 * dt * a;
        }

        _dynamics.Inverse(_qPredicted, _qdPredicted, _acceleration, _gravity, u); // M̂(q̄) a + ĥ(q̄, q̄̇)

        for (var i = 0; i < _n; i++)
        {
            var speed = _qdPredicted[i];
            var value = u[i] + _viscousFriction[i] * speed + _coulombFriction[i] * Math.Tanh(speed / _smoothingSpeed[i]);
            var limit = _torqueLimit[i];
            _clamped[i] = Math.Abs(value) > limit;
            u[i] = Math.Clamp(value, -limit, limit);
            if (!_clamped[i]) _w[i] = _candidate[i];
        }
    }

    /// <summary>sign(s) for ε = 0 (0 at s = 0), otherwise s/(|s| + ε); NaN stays NaN (SPEC §6.4). SPEC §7.</summary>
    internal static double Switch(double s, double epsilon) =>
        epsilon > 0.0 ? s / (Math.Abs(s) + epsilon) : s > 0.0 ? 1.0 : s < 0.0 ? -1.0 : s;
}
