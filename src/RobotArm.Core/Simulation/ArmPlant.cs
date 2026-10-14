using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;

namespace RobotArm.Core.Simulation;

/// <summary>
/// The true arm as a plant: state x = [q; q̇] (2n values), input u = commanded joint torques (n values).
/// ẋ = [q̇; M(q)⁻¹(τ(u) − τf(q̇) − h(q, q̇))] with actuator saturation and friction. SPEC §5.4, §6.1.
/// </summary>
/// <remarks>
/// <para>
/// The parameters are the true ones (SPEC §5.6): <see cref="BaseModel"/> without payload, the
/// <see cref="Planet"/>, and the friction parameters fv, fc, ωs of <see cref="ArmModel.Actuator"/>
/// (SPEC §3.3). Friction is switched off by giving a model with fv = fc = 0.
/// </para>
/// <para>
/// <see cref="Derivatives"/> does not allocate (PROJECT.md §3). <see cref="SetPayload"/> builds a new
/// dynamics object and may allocate; it is called only between steps (SPEC §5.5).
/// An instance holds work buffers, so it is not safe to use from two threads at once.
/// </para>
/// </remarks>
public sealed class ArmPlant : IPlant
{
    private readonly int _n;
    private readonly double _stallTorque;
    private readonly double _viscousFriction;
    private readonly double _coulombFriction;
    private readonly double _smoothingSpeed;
    private readonly Vector3d _gravity;
    private RecursiveNewtonEuler _dynamics;

    private readonly double[] _massMatrix;
    private readonly double[] _rhs;
    private readonly double[] _appliedTorque;
    private readonly bool[] _saturated;

    /// <summary>Creates the plant for <paramref name="model"/> on <paramref name="planet"/>, without payload. SPEC §6.1.</summary>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    public ArmPlant(ArmModel model, Planet planet)
    {
        ArgumentNullException.ThrowIfNull(model);
        ArgumentNullException.ThrowIfNull(planet);

        BaseModel = model;
        Planet = planet;
        _n = model.DegreesOfFreedom;
        var actuator = model.Actuator;
        _stallTorque = actuator.StallTorque;
        _viscousFriction = actuator.ViscousFriction;
        _coulombFriction = actuator.CoulombFriction;
        _smoothingSpeed = actuator.SmoothingSpeed;
        _gravity = planet.GravityVector;
        _dynamics = new RecursiveNewtonEuler(model);

        _massMatrix = new double[_n * _n];
        _rhs = new double[_n];
        _appliedTorque = new double[_n];
        _saturated = new bool[_n];
    }

    /// <summary>The arm without payload. SPEC §3, §4.1.</summary>
    public ArmModel BaseModel { get; }

    /// <summary>The model the dynamics use now: <see cref="BaseModel"/> with the attached payload. SPEC §5.5.</summary>
    public ArmModel Model => _dynamics.Model;

    /// <summary>The environment; gravity (0, 0, −g). SPEC §3.5.</summary>
    public Planet Planet { get; }

    /// <summary>Attached payload mass in kg, 0 when none. SPEC §5.5.</summary>
    public double PayloadKg { get; private set; }

    /// <summary>Number of joints n.</summary>
    public int DegreesOfFreedom => _n;

    /// <summary>2n: joint angles then joint speeds. SPEC §2, §6.1.</summary>
    public int StateCount => 2 * _n;

    /// <summary>n: one commanded torque per joint. SPEC §5.4.</summary>
    public int InputCount => _n;

    /// <summary>
    /// Torques τ = clamp(u, −τmax, +τmax) applied in the last <see cref="Derivatives"/> call, N·m. SPEC §5.4.
    /// </summary>
    public ReadOnlySpan<double> AppliedTorque => _appliedTorque;

    /// <summary>
    /// For each joint, whether |u| exceeded τmax in the last <see cref="Derivatives"/> call. SPEC §5.4.
    /// They depend on u only, and a solver holds u over a step (SPEC §6.2), so after a step they hold for
    /// every stage of that step.
    /// </summary>
    public ReadOnlySpan<bool> Saturated => _saturated;

    /// <summary>
    /// Replaces the payload: the dynamics switch to <see cref="BaseModel"/> with a point mass of
    /// <paramref name="massKg"/> at the tool point (0 detaches it). SPEC §5.5. Call only between steps.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The mass is negative or not finite.</exception>
    public void SetPayload(double massKg)
    {
        var model = BaseModel.WithPayload(massKg);
        _dynamics = new RecursiveNewtonEuler(model);
        PayloadKg = massKg;
    }

    /// <summary>
    /// ẋ = [q̇; q̈] with M(q) q̈ = τ − τf − h(q, q̇) solved by Cholesky, τi = clamp(ui, −τmax, +τmax) and
    /// τf,i = fv q̇i + fc tanh(q̇i / ωs). SPEC §5.2, §5.4, §6.1. Records the applied torques and saturation
    /// flags. Does not allocate.
    /// </summary>
    /// <remarks>
    /// A NaN or infinite input gives NaN derivatives instead of an exception, so that the state becomes
    /// non-finite and the simulation stops with the fault of SPEC §6.4. A NaN or infinite angle q makes M(q)
    /// undefined; then every derivative is NaN and the Cholesky solve is skipped.
    /// </remarks>
    /// <exception cref="ArgumentException">A span has the wrong length, or M is not positive definite.</exception>
    public void Derivatives(double t, ReadOnlySpan<double> x, ReadOnlySpan<double> u, Span<double> dx)
    {
        if (x.Length != 2 * _n)
            throw new ArgumentException($"Expected {2 * _n} state values, got {x.Length} (SPEC §6.1).", nameof(x));
        if (u.Length != _n)
            throw new ArgumentException($"Expected {_n} inputs, got {u.Length} (SPEC §6.1).", nameof(u));
        if (dx.Length != 2 * _n)
            throw new ArgumentException($"Expected room for {2 * _n} derivatives, got {dx.Length} (SPEC §6.1).", nameof(dx));

        var q = x[.._n];
        var qd = x[_n..];

        foreach (var qi in q)
        {
            if (!double.IsFinite(qi))
            {
                dx.Fill(double.NaN);
                return;
            }
        }

        // h(q, q̇) first, into the right-hand side; the inertia matrix next.
        _dynamics.BiasTorque(q, qd, _gravity, _rhs);
        _dynamics.InertiaMatrix(q, _massMatrix);

        for (var i = 0; i < _n; i++)
        {
            var ui = u[i];
            var tau = Math.Clamp(ui, -_stallTorque, _stallTorque);
            _saturated[i] = Math.Abs(ui) > _stallTorque;
            _appliedTorque[i] = tau;

            var friction = _viscousFriction * qd[i] + _coulombFriction * Math.Tanh(qd[i] / _smoothingSpeed);
            _rhs[i] = tau - friction - _rhs[i];
        }

        Cholesky.FactorAndSolve(_massMatrix, _n, _rhs);

        qd.CopyTo(dx);
        _rhs.CopyTo(dx[_n..]);
    }
}
