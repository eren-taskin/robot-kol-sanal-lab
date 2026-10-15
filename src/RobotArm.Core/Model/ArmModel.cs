using G = RobotArm.Core.Model.OpenManipulatorXGeometry;

namespace RobotArm.Core.Model;

/// <summary>
/// A serial arm as data: joints, bodies, tool point and actuators. SPEC §4.1 ("the chain is data, not code"),
/// §3.2, §3.3. Joint i moves body i and is driven by actuator i; the lists are zero-based. Immutable.
/// </summary>
/// <remarks>
/// The model holds the joint limits but never rejects a configuration for being outside them (SPEC §3.4).
/// </remarks>
public sealed class ArmModel : IEquatable<ArmModel>
{
    private readonly JointDefinition[] _joints;
    private readonly BodyParameters[] _bodies;
    private readonly ActuatorParameters[] _actuators;

    /// <summary>Joints in order from the base, zero-based. SPEC §4.1.</summary>
    public IReadOnlyList<JointDefinition> Joints { get; }

    /// <summary>Bodies in order from the base; body i moves with joint i. SPEC §3.2.</summary>
    public IReadOnlyList<BodyParameters> Bodies { get; }

    /// <summary>Tool point in the frame of the last joint, m. SPEC §4.1.</summary>
    public Vector3d ToolPoint { get; }

    /// <summary>
    /// Actuator and friction parameters, one per joint; actuator i drives joint i. SPEC §3.3. For the
    /// OpenMANIPULATOR-X all four are the XM430-W350 values (docs/DECISIONS.md D5).
    /// </summary>
    public IReadOnlyList<ActuatorParameters> Actuators { get; }

    /// <summary>Number of joints (4 for the OpenMANIPULATOR-X).</summary>
    public int DegreesOfFreedom => _joints.Length;

    /// <summary>Creates a model with one actuator per joint. The sequences are copied. SPEC §4.1, §3.3.</summary>
    /// <exception cref="ArgumentNullException">A sequence or an element is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// No joints, a different number of joints and bodies or actuators, or a tool point that is not finite.
    /// </exception>
    public ArmModel(
        IEnumerable<JointDefinition> joints,
        IEnumerable<BodyParameters> bodies,
        Vector3d toolPoint,
        IEnumerable<ActuatorParameters> actuators)
    {
        ArgumentNullException.ThrowIfNull(joints);
        ArgumentNullException.ThrowIfNull(bodies);
        ArgumentNullException.ThrowIfNull(actuators);

        _joints = joints.ToArray();
        _bodies = bodies.ToArray();
        _actuators = actuators.ToArray();

        if (_joints.Length == 0)
            throw new ArgumentOutOfRangeException(nameof(joints), "An arm needs at least one joint (SPEC §4.1).");
        if (_bodies.Length != _joints.Length)
            throw new ArgumentOutOfRangeException(nameof(bodies), _bodies.Length,
                $"An arm needs one body per joint: {_joints.Length} joints but {_bodies.Length} bodies (SPEC §4.1).");
        if (Array.IndexOf(_joints, null) >= 0)
            throw new ArgumentNullException(nameof(joints), "A joint is null.");
        if (_actuators.Length != _joints.Length)
            throw new ArgumentOutOfRangeException(nameof(actuators), _actuators.Length,
                $"An arm needs one actuator per joint: {_joints.Length} joints but {_actuators.Length} actuators (SPEC §3.3).");
        if (Array.IndexOf(_bodies, null) >= 0)
            throw new ArgumentNullException(nameof(bodies), "A body is null.");
        if (Array.IndexOf(_actuators, null) >= 0)
            throw new ArgumentNullException(nameof(actuators), "An actuator is null.");
        if (!toolPoint.IsFinite)
            throw new ArgumentOutOfRangeException(nameof(toolPoint), toolPoint, "Tool point must be finite (SPEC §4.1).");

        Joints = Array.AsReadOnly(_joints);
        Bodies = Array.AsReadOnly(_bodies);
        ToolPoint = toolPoint;
        Actuators = Array.AsReadOnly(_actuators);
    }

    /// <summary>
    /// Creates a model whose joints all have the same <paramref name="actuator"/>. The sequences are copied.
    /// SPEC §4.1, §3.3.
    /// </summary>
    /// <exception cref="ArgumentNullException">A sequence, an element or the actuator is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// No joints, a different number of joints and bodies, or a tool point that is not finite.
    /// </exception>
    public ArmModel(
        IEnumerable<JointDefinition> joints,
        IEnumerable<BodyParameters> bodies,
        Vector3d toolPoint,
        ActuatorParameters actuator)
        : this(Copy(joints), bodies, toolPoint, actuator)
    {
    }

    private ArmModel(JointDefinition[] joints, IEnumerable<BodyParameters> bodies, Vector3d toolPoint, ActuatorParameters actuator)
        : this(joints, bodies, toolPoint, Enumerable.Repeat(actuator ?? throw new ArgumentNullException(nameof(actuator)), joints.Length))
    {
    }

    /// <summary>
    /// The OpenMANIPULATOR-X of SPEC §3.1–§3.4 and the chain table of SPEC §4.1, with the actuator
    /// defaults of SPEC §3.3.
    /// </summary>
    public static ArmModel OpenManipulatorX()
    {
        JointDefinition[] joints =
        [
            new(Vector3d.Zero, JointAxis.Z, G.Joint1Lower, G.Joint1Upper),
            new(new Vector3d(0.0, 0.0, G.D1), JointAxis.Y, G.Joint2Lower, G.Joint2Upper),
            new(new Vector3d(G.A2X, 0.0, G.A2Z), JointAxis.Y, G.Joint3Lower, G.Joint3Upper),
            new(new Vector3d(G.L3, 0.0, 0.0), JointAxis.Y, G.Joint4Lower, G.Joint4Upper),
        ];

        // SPEC §3.2, Table 5: m (kg), ℓ (m), ε (deg), Ixx, Iyy, Izz, Ixz (kg·m²).
        BodyParameters[] bodies =
        [
            new(0.0984, 0.0474, Deg(-90.4), 34.5e-6, 32.7e-6, 18.9e-6, -0.4e-6),
            new(0.1385, 0.1022, Deg(-84.2), 330.6e-6, 342.9e-6, 60.3e-6, -38.5e-6),
            new(0.1327, 0.0909, Deg(-0.1), 30.7e-6, 242.3e-6, 251.6e-6, -0.3e-6),
            new(0.1433, 0.0451, Deg(-11.4), 80.9e-6, 76.0e-6, 93.1e-6, -1.0e-6),
        ];

        // SPEC §3.3: every joint has an XM430-W350 with the default parameters.
        return new ArmModel(joints, bodies, new Vector3d(G.L4, 0.0, 0.0), new ActuatorParameters());
    }

    /// <summary>
    /// A new model whose last body is combined with a point mass rigidly attached at the tool point.
    /// SPEC §5.5. A massless payload returns this model, which is equal to itself.
    /// </summary>
    /// <remarks>
    /// Only planar data is supported (SPEC §3.2): the last body must have its centre of mass in its x–z plane
    /// and no Ixy or Iyz terms (<see cref="BodyParameters.IsPlanar"/>), and the tool point must have y = 0.
    /// That is the case for the OpenMANIPULATOR-X. Anything else throws <see cref="NotSupportedException"/>
    /// instead of returning a wrong combined body (docs/DECISIONS.md D5).
    /// </remarks>
    /// <param name="massKg">Payload mass mp in kg, finite and at least 0.</param>
    /// <exception cref="ArgumentOutOfRangeException">The mass is negative or not finite.</exception>
    /// <exception cref="NotSupportedException">
    /// The mass is positive and the last body or the tool point is not planar.
    /// </exception>
    public ArmModel WithPayload(double massKg)
    {
        if (!double.IsFinite(massKg) || massKg < 0.0)
            throw new ArgumentOutOfRangeException(nameof(massKg), massKg, "Payload mass must be a finite number of at least 0 kg (SPEC §5.5).");
        if (massKg == 0.0)
            return this;

        var last = _bodies[^1];
        if (!last.IsPlanar || ToolPoint.Y != 0.0)
            throw new NotSupportedException(
                "WithPayload supports only planar data: the last body's centre of mass and the tool point in its x–z plane " +
                "(y = 0) and no Ixy or Iyz inertia terms (SPEC §3.2, §5.5; docs/DECISIONS.md D5).");

        var c4 = last.CenterOfMass;
        var t = ToolPoint;
        var m = last.Mass + massKg;
        var c = (last.Mass * c4 + massKg * t) / m;
        var inertia = last.Inertia
            + Matrix3d.PointMassInertia(last.Mass, c4 - c)
            + Matrix3d.PointMassInertia(massKg, t - c);

        var bodies = (BodyParameters[])_bodies.Clone();
        bodies[^1] = BodyParameters.FromCenterOfMassAndInertia(m, c, inertia);
        return new ArmModel(_joints, bodies, ToolPoint, _actuators);
    }

    /// <summary>Value equality: same joints, bodies, tool point and actuator parameters of every joint.</summary>
    public bool Equals(ArmModel? other) =>
        other is not null &&
        (ReferenceEquals(this, other) ||
         (_joints.AsSpan().SequenceEqual(other._joints) &&
          _bodies.AsSpan().SequenceEqual(other._bodies) &&
          ToolPoint.Equals(other.ToolPoint) &&
          _actuators.AsSpan().SequenceEqual(other._actuators)));

    /// <inheritdoc/>
    public override bool Equals(object? obj) => Equals(obj as ArmModel);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        var hash = new HashCode();
        foreach (var j in _joints) hash.Add(j);
        foreach (var b in _bodies) hash.Add(b);
        hash.Add(ToolPoint);
        foreach (var a in _actuators) hash.Add(a);
        return hash.ToHashCode();
    }

    private static JointDefinition[] Copy(IEnumerable<JointDefinition> joints)
    {
        ArgumentNullException.ThrowIfNull(joints);
        return joints.ToArray();
    }

    private static double Deg(double degrees) => degrees * Math.PI / 180.0;
}
