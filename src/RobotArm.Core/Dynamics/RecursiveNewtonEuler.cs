using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;

namespace RobotArm.Core.Dynamics;

/// <summary>
/// Rigid-body dynamics of a serial arm by the recursive Newton–Euler algorithm (Luh, Walker and Paul)
/// over the chain of SPEC §4.1, with the inertia matrix by the Walker–Orin column method. SPEC §5.1–§5.2.
/// </summary>
/// <remarks>
/// <para>
/// Works for any <see cref="ArmModel"/>: the chain is data (SPEC §4.1), so a 6-DOF arm needs no new code.
/// Velocities, accelerations, forces and moments of body i are expressed in its own frame Fi.
/// </para>
/// <para>
/// Gravity enters as a base acceleration a0 = −gravity, so the output contains g(q) (SPEC §5.1).
/// The reflected rotor inertia N²·Jm (SPEC §3.3, §5.2) acts on each joint's own acceleration:
/// it is on the diagonal of <see cref="InertiaMatrix"/> and adds N²·Jm·q̈i to <see cref="Inverse"/>,
/// so that Inverse(q, q̇, q̈) = M(q) q̈ + h(q, q̇) holds with the same M.
/// </para>
/// <para>
/// Buffers are sized from the model in the constructor; no public method allocates (PROJECT.md §3).
/// An instance holds work buffers, so it is not safe to use from two threads at once.
/// Joint limits are never checked here (SPEC §3.4).
/// </para>
/// </remarks>
public sealed class RecursiveNewtonEuler
{
    private readonly int _n;
    private readonly JointAxis[] _axis;
    private readonly Vector3d[] _axisVector;
    private readonly Vector3d[] _origin;
    private readonly double[] _mass;
    private readonly Vector3d[] _com;
    private readonly Matrix3d[] _inertia;
    private readonly double _reflectedRotorInertia;

    // Work buffers. _rotation[i] = R(axis i, qi): maps vectors from Fi to the parent frame.
    private readonly Matrix3d[] _rotation;
    private readonly Vector3d[] _force;
    private readonly Vector3d[] _moment;
    private readonly double[] _zeros;
    private readonly double[] _unit;
    private readonly double[] _column;

    /// <summary>Creates the algorithm for <paramref name="model"/> and sizes its buffers. SPEC §5.2.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="model"/> is null.</exception>
    public RecursiveNewtonEuler(ArmModel model)
    {
        ArgumentNullException.ThrowIfNull(model);
        Model = model;
        _n = model.DegreesOfFreedom;

        _axis = new JointAxis[_n];
        _axisVector = new Vector3d[_n];
        _origin = new Vector3d[_n];
        _mass = new double[_n];
        _com = new Vector3d[_n];
        _inertia = new Matrix3d[_n];
        for (var i = 0; i < _n; i++)
        {
            var joint = model.Joints[i];
            var body = model.Bodies[i];
            _axis[i] = joint.Axis;
            _axisVector[i] = joint.AxisVector;
            _origin[i] = joint.Origin;
            _mass[i] = body.Mass;
            _com[i] = body.CenterOfMass;
            _inertia[i] = body.Inertia;
        }
        _reflectedRotorInertia = model.Actuator.ReflectedRotorInertia;

        _rotation = new Matrix3d[_n];
        _force = new Vector3d[_n];
        _moment = new Vector3d[_n];
        _zeros = new double[_n];
        _unit = new double[_n];
        _column = new double[_n];
    }

    /// <summary>The arm whose dynamics this instance computes.</summary>
    public ArmModel Model { get; }

    /// <summary>Number of joints n; vectors have length n and matrices n×n.</summary>
    public int DegreesOfFreedom => _n;

    /// <summary>
    /// Inverse dynamics τ = M(q) q̈ + C(q, q̇) q̇ + g(q) by the recursive Newton–Euler algorithm,
    /// including the reflected rotor inertia N²·Jm·q̈i. SPEC §5.1, §5.2. Does not allocate.
    /// </summary>
    /// <param name="q">Joint angles, rad.</param>
    /// <param name="qd">Joint speeds, rad/s.</param>
    /// <param name="qdd">Joint accelerations, rad/s².</param>
    /// <param name="gravity">Gravity vector in the base frame, m/s², e.g. (0, 0, −g) (SPEC §3.5).</param>
    /// <param name="tau">Output: joint torques, N·m.</param>
    /// <exception cref="ArgumentException">A span length differs from the number of joints.</exception>
    public void Inverse(
        ReadOnlySpan<double> q, ReadOnlySpan<double> qd, ReadOnlySpan<double> qdd,
        Vector3d gravity, Span<double> tau)
    {
        RequireLength(q, nameof(q));
        RequireLength(qd, nameof(qd));
        RequireLength(qdd, nameof(qdd));
        RequireLength(tau, nameof(tau));

        SetRotations(q);
        RigidBodyPass(qd, qdd, -gravity, tau);
        for (var i = 0; i < _n; i++)
            tau[i] += _reflectedRotorInertia * qdd[i];
    }

    /// <summary>
    /// Bias torques h(q, q̇) = C(q, q̇) q̇ + g(q) = RNEA(q, q̇, 0). SPEC §5.2. Does not allocate.
    /// </summary>
    /// <param name="q">Joint angles, rad.</param>
    /// <param name="qd">Joint speeds, rad/s.</param>
    /// <param name="gravity">Gravity vector in the base frame, m/s², e.g. (0, 0, −g) (SPEC §3.5).</param>
    /// <param name="h">Output: bias torques, N·m.</param>
    /// <exception cref="ArgumentException">A span length differs from the number of joints.</exception>
    public void BiasTorque(ReadOnlySpan<double> q, ReadOnlySpan<double> qd, Vector3d gravity, Span<double> h)
    {
        RequireLength(q, nameof(q));
        RequireLength(qd, nameof(qd));
        RequireLength(h, nameof(h));

        SetRotations(q);
        RigidBodyPass(qd, _zeros, -gravity, h);
    }

    /// <summary>
    /// Bias torques h(q, q̇) on <paramref name="planet"/>, gravity (0, 0, −g). SPEC §3.5, §5.2.
    /// Does not allocate.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="planet"/> is null.</exception>
    /// <exception cref="ArgumentException">A span length differs from the number of joints.</exception>
    public void BiasTorque(ReadOnlySpan<double> q, ReadOnlySpan<double> qd, Planet planet, Span<double> h)
    {
        ArgumentNullException.ThrowIfNull(planet);
        BiasTorque(q, qd, planet.GravityVector, h);
    }

    /// <summary>
    /// Inertia matrix M(q), row-major n×n: column k is RNEA(q, 0, e_k) with gravity zero (Walker and Orin),
    /// plus N²·Jm on each diagonal entry. SPEC §5.2. Does not allocate.
    /// </summary>
    /// <remarks>
    /// The entries on and below the diagonal are taken from the columns and mirrored above it, so the
    /// result is exactly symmetric, as the Cholesky solver of SPEC §5.2 expects.
    /// </remarks>
    /// <param name="q">Joint angles, rad.</param>
    /// <param name="m">Output: M(q) in row-major order, length n², kg·m².</param>
    /// <exception cref="ArgumentException">A span length is wrong.</exception>
    public void InertiaMatrix(ReadOnlySpan<double> q, Span<double> m)
    {
        RequireLength(q, nameof(q));
        if (m.Length != _n * _n)
            throw new ArgumentException($"Expected room for {_n}×{_n} = {_n * _n} entries, got {m.Length} (SPEC §5.2).", nameof(m));

        SetRotations(q);
        for (var k = 0; k < _n; k++)
        {
            _unit[k] = 1.0;
            RigidBodyPass(_zeros, _unit, Vector3d.Zero, _column);
            _unit[k] = 0.0;

            for (var i = k; i < _n; i++)
            {
                m[i * _n + k] = _column[i];
                m[k * _n + i] = _column[i];
            }
            m[k * _n + k] += _reflectedRotorInertia;
        }
    }

    private void SetRotations(ReadOnlySpan<double> q)
    {
        for (var i = 0; i < _n; i++)
            _rotation[i] = Rotations.About(_axis[i], q[i]);
    }

    /// <summary>
    /// Newton–Euler recursion without rotor inertia. Needs <see cref="SetRotations"/> first.
    /// <paramref name="baseAcceleration"/> is a0 = −gravity in the base frame.
    /// </summary>
    private void RigidBodyPass(
        ReadOnlySpan<double> qd, ReadOnlySpan<double> qdd, Vector3d baseAcceleration, Span<double> tau)
    {
        // Forward recursion: velocities and accelerations, base to tip, in each body's own frame.
        var omegaParent = Vector3d.Zero;
        var omegaDotParent = Vector3d.Zero;
        var accelParent = baseAcceleration;
        for (var i = 0; i < _n; i++)
        {
            var r = _rotation[i];
            var z = _axisVector[i];
            var p = _origin[i];

            // Acceleration of the joint origin, in the parent frame, then turned into Fi.
            var accelAtJoint = accelParent
                + Vector3d.Cross(omegaDotParent, p)
                + Vector3d.Cross(omegaParent, Vector3d.Cross(omegaParent, p));
            var omegaIn = TransposeTimes(r, omegaParent);
            var omega = omegaIn + qd[i] * z;
            var omegaDot = TransposeTimes(r, omegaDotParent) + Vector3d.Cross(omegaIn, qd[i] * z) + qdd[i] * z;
            var accel = TransposeTimes(r, accelAtJoint);

            // Newton and Euler equations of body i about its centre of mass.
            var c = _com[i];
            var accelCom = accel + Vector3d.Cross(omegaDot, c) + Vector3d.Cross(omega, Vector3d.Cross(omega, c));
            var inertia = _inertia[i];
            _force[i] = _mass[i] * accelCom;
            _moment[i] = inertia * omegaDot + Vector3d.Cross(omega, inertia * omega);

            omegaParent = omega;
            omegaDotParent = omegaDot;
            accelParent = accel;
        }

        // Backward recursion: forces and moments at each joint, tip to base.
        var childForce = Vector3d.Zero;   // force of child body on this body's joint, in this frame
        var childMoment = Vector3d.Zero;
        var childOrigin = Vector3d.Zero;  // origin of the child joint in this frame
        for (var i = _n - 1; i >= 0; i--)
        {
            var f = _force[i] + childForce;
            var nMoment = _moment[i] + childMoment
                + Vector3d.Cross(_com[i], _force[i])
                + Vector3d.Cross(childOrigin, childForce);
            tau[i] = Vector3d.Dot(nMoment, _axisVector[i]);

            var r = _rotation[i];
            childForce = r * f;
            childMoment = r * nMoment;
            childOrigin = _origin[i];
        }
    }

    private static Vector3d TransposeTimes(Matrix3d r, Vector3d v) => new(
        r.M00 * v.X + r.M10 * v.Y + r.M20 * v.Z,
        r.M01 * v.X + r.M11 * v.Y + r.M21 * v.Z,
        r.M02 * v.X + r.M12 * v.Y + r.M22 * v.Z);

    private void RequireLength(ReadOnlySpan<double> span, string name)
    {
        if (span.Length != _n)
            throw new ArgumentException($"Expected {_n} values, got {span.Length} (SPEC §5.2).", name);
    }
}
