using RobotArm.Core.Control;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using G = RobotArm.Core.Model.OpenManipulatorXGeometry;

namespace RobotArm.Core.Tasks;

/// <summary>Joint quintic with zero velocity and acceleration at both ends, held outside its duration. SPEC §8.1.</summary>
public sealed class QuinticJointTrajectory : IReferenceSource
{
    private readonly double[] _start, _end;

    /// <summary>Copies endpoints and validates a positive duration in seconds. SPEC §8.1.</summary>
    public QuinticJointTrajectory(ReadOnlySpan<double> start, ReadOnlySpan<double> end, double duration)
    {
        if (start.IsEmpty || start.Length != end.Length) throw new ArgumentException("Endpoints must have the same nonzero length.");
        foreach (var v in start) if (!double.IsFinite(v)) throw new ArgumentOutOfRangeException(nameof(start));
        foreach (var v in end) if (!double.IsFinite(v)) throw new ArgumentOutOfRangeException(nameof(end));
        TimeLaw.ValidateDuration(duration);
        _start = start.ToArray(); _end = end.ToArray(); Duration = duration;
    }

    /// <summary>Motion duration, s. SPEC §8.1.</summary>
    public double Duration { get; }
    /// <inheritdoc/>
    public int DegreesOfFreedom => _start.Length;
    /// <inheritdoc/>
    public void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration)
    {
        var (s, ds, dds) = TimeLaw.Evaluate(t, Duration);
        for (var i = 0; i < _start.Length; i++)
        {
            var delta = _end[i] - _start[i];
            position[i] = _start[i] + s * delta; velocity[i] = ds * delta; acceleration[i] = dds * delta;
        }
    }
}

/// <summary>Clean Cartesian trajectory failure, propagated into RunResult. SPEC §8.1.</summary>
public sealed class TrajectoryException : Exception
{
    /// <summary>Creates a failure with a stable reason. SPEC §8.1.</summary>
    public TrajectoryException(string reason) : base(reason) { }
}

/// <summary>Cartesian line with quintic timing and fixed pitch; IK at every sample. SPEC §8.1.</summary>
public sealed class CartesianLineTrajectory : IReferenceSource
{
    private readonly Vector3d _start, _delta;
    private readonly double _pitch, _duration;
    private readonly ElbowBranch _branch;
    /// <summary>Defines a line; unreachable or singular samples fail with TrajectoryException. SPEC §8.1.</summary>
    public CartesianLineTrajectory(Vector3d start, Vector3d end, double pitch, double duration, ElbowBranch branch = ElbowBranch.Up)
    {
        CartesianReference.Validate(start, pitch, duration, branch);
        if (!end.IsFinite) throw new ArgumentOutOfRangeException(nameof(end));
        // atan2 jumps by 2π across the negative x axis; reject that path instead of emitting a joint step.
        if ((start.Y < 0 && end.Y >= 0) || (end.Y < 0 && start.Y >= 0))
        {
            var crossing = start.X + (end.X - start.X) * (-start.Y / (end.Y - start.Y));
            if (crossing <= 0) throw new TrajectoryException("Trajectory: yaw branch cut");
        }
        _start = start; _delta = end - start; _pitch = pitch; _duration = duration; _branch = branch;
    }
    /// <inheritdoc/>
    public int DegreesOfFreedom => 4;
    /// <inheritdoc/>
    public void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration)
    {
        var (s, ds, dds) = TimeLaw.Evaluate(t, _duration);
        CartesianReference.Solve(_start + s * _delta, ds * _delta, dds * _delta, _pitch, _branch, position, velocity, acceleration);
    }
}

/// <summary>One circle in the horizontal XY plane, quintic phase from rest to rest. SPEC §8.1.</summary>
public sealed class CartesianCircleTrajectory : IReferenceSource
{
    private readonly Vector3d _center;
    private readonly double _radius, _pitch, _duration;
    private readonly ElbowBranch _branch;
    /// <summary>Starts at center + (radius,0,0), with fixed tool pitch. SPEC §8.1.</summary>
    public CartesianCircleTrajectory(Vector3d center, double radius, double pitch, double duration, ElbowBranch branch = ElbowBranch.Up)
    {
        CartesianReference.Validate(center, pitch, duration, branch);
        if (!(radius > 0) || !double.IsFinite(radius)) throw new ArgumentOutOfRangeException(nameof(radius));
        if (Math.Abs(center.Y) <= radius && center.X - Math.Sqrt(radius * radius - center.Y * center.Y) <= 0)
            throw new TrajectoryException("Trajectory: yaw branch cut");
        _center = center; _radius = radius; _pitch = pitch; _duration = duration; _branch = branch;
    }
    /// <inheritdoc/>
    public int DegreesOfFreedom => 4;
    /// <inheritdoc/>
    public void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration)
    {
        var (s, ds, dds) = TimeLaw.Evaluate(t, _duration);
        var a = 2 * Math.PI * s; var v = 2 * Math.PI * ds; var acc = 2 * Math.PI * dds;
        var radial = new Vector3d(Math.Cos(a), Math.Sin(a), 0);
        var tangent = new Vector3d(-Math.Sin(a), Math.Cos(a), 0);
        CartesianReference.Solve(_center + _radius * radial, _radius * v * tangent,
            _radius * (acc * tangent - v * v * radial), _pitch, _branch, position, velocity, acceleration);
    }
}

internal static class TimeLaw
{
    internal static void ValidateDuration(double duration)
    {
        if (!double.IsFinite(duration) || duration <= 0) throw new ArgumentOutOfRangeException(nameof(duration));
    }
    internal static (double S, double V, double A) Evaluate(double t, double duration)
    {
        if (!double.IsFinite(t)) throw new ArgumentOutOfRangeException(nameof(t));
        var u = Math.Clamp(t / duration, 0, 1);
        return (u * u * u * (10 - 15 * u + 6 * u * u),
            30 * u * u * (1 - u) * (1 - u) / duration,
            60 * u * (1 - u) * (1 - 2 * u) / (duration * duration));
    }
}

internal static class CartesianReference
{
    internal static void Validate(Vector3d p, double pitch, double duration, ElbowBranch branch)
    {
        if (!p.IsFinite || !double.IsFinite(pitch) || !Enum.IsDefined(branch)) throw new ArgumentOutOfRangeException(nameof(p));
        TimeLaw.ValidateDuration(duration);
    }
    internal static void Solve(Vector3d p, Vector3d v, Vector3d a, double pitch, ElbowBranch branch,
        Span<double> q, Span<double> qd, Span<double> qdd)
    {
        var ik = InverseKinematics.Solve(p, pitch, branch);
        if (!ik.Success) throw new TrajectoryException($"Trajectory: {ik.Failure}");
        ik.Q.CopyTo(q);
        var r2 = p.X * p.X + p.Y * p.Y;
        if (r2 < 1e-12 || !PlanarJacobian.TryInvert(q, out var inverse)) throw new TrajectoryException("Trajectory: Singular");
        var r = Math.Sqrt(r2);
        var rv = (p.X * v.X + p.Y * v.Y) / r;
        var ra = (v.X * v.X + v.Y * v.Y + p.X * a.X + p.Y * a.Y - rv * rv) / r;
        qd[0] = (p.X * v.Y - p.Y * v.X) / r2;
        qdd[0] = (p.X * a.Y - p.Y * a.X) / r2 - 2 * rv / r * qd[0];
        var planarV = inverse * new Vector3d(rv, v.Z, 0);
        qd[1] = planarV.X; qd[2] = planarV.Y; qd[3] = planarV.Z;
        // Analytic Jdot*qdot: each link contributes its centripetal acceleration in (r,z).
        var t2 = q[1] - G.Beta; var t3 = q[1] + q[2]; var t4 = t3 + q[3];
        var w2 = qd[1]; var w3 = w2 + qd[2]; var w4 = w3 + qd[3];
        var radial = -G.L2 * Math.Cos(t2) * w2 * w2 - G.L3 * Math.Cos(t3) * w3 * w3 - G.L4 * Math.Cos(t4) * w4 * w4;
        var vertical = G.L2 * Math.Sin(t2) * w2 * w2 + G.L3 * Math.Sin(t3) * w3 * w3 + G.L4 * Math.Sin(t4) * w4 * w4;
        var planarA = inverse * new Vector3d(ra - radial, a.Z - vertical, 0);
        qdd[1] = planarA.X; qdd[2] = planarA.Y; qdd[3] = planarA.Z;
    }
}
