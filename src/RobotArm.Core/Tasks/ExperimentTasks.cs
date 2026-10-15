using RobotArm.Core.Control;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;

namespace RobotArm.Core.Tasks;

/// <summary>Task phases recorded for the web page and log. SPEC §8.2, §10.</summary>
public enum TaskPhase
{
    /// <summary>Moving to the target. SPEC §8.2.</summary>
    Moving,
    /// <summary>Waiting for pickup conditions. SPEC §8.2.</summary>
    Picking,
    /// <summary>Transporting the attached object. SPEC §8.2.</summary>
    Carrying,
    /// <summary>Waiting to release at the destination. SPEC §8.2.</summary>
    Placing,
    /// <summary>Holding the final reference. SPEC §8.2.</summary>
    Holding,
    /// <summary>Object has been released. SPEC §8.2.</summary>
    Released
}

internal enum PayloadEvent { None, Attach, Release }

/// <summary>Reference and state machine for one task. Subclasses implement the four tasks of SPEC §8.2.</summary>
public abstract class ExperimentTask : IReferenceSource
{
    /// <summary>Creates task state from immutable settings. SPEC §8.2.</summary>
    protected ExperimentTask(TaskSettings settings) { Settings = settings; FinalTarget = settings.Target; }
    /// <summary>Task input. SPEC §8.2.</summary>
    protected TaskSettings Settings { get; }
    /// <summary>Current task phase. SPEC §8.2.</summary>
    public TaskPhase Phase { get; protected set; }
    /// <summary>Final tool target used for settling metrics. SPEC §9.</summary>
    public Vector3d FinalTarget { get; protected set; }
    /// <summary>Time from which final-target settling is measured, s. SPEC §9.</summary>
    public double SettlingFromTime { get; protected set; }
    /// <summary>Task failure discovered at a boundary, if any. SPEC §8.2.</summary>
    public string? FailureReason { get; protected set; }
    /// <inheritdoc/>
    public int DegreesOfFreedom => 4;
    /// <inheritdoc/>
    public abstract void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration);
    internal virtual PayloadEvent Observe(double t, Vector3d tool, double speed) => PayloadEvent.None;
    internal abstract string? EvaluateOutcome(RunMetrics metrics, double endTime);
}

/// <summary>Quintic joint move to the IK target, followed by a hold. SPEC §8.2.</summary>
public sealed class Reach : ExperimentTask
{
    private readonly QuinticJointTrajectory _path;
    /// <summary>Creates a smooth move from Start to Target and Pitch. SPEC §8.2.</summary>
    public Reach(TaskSettings settings) : base(settings)
    {
        var end = TaskGeometry.Solve(settings.Target, settings.Pitch, settings.Branch);
        _path = new(settings.Start.ToArray(), end.ToArray(), settings.MotionDuration);
        SettlingFromTime = settings.MotionDuration;
    }
    /// <inheritdoc/>
    public override void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration)
    { Phase = t < Settings.MotionDuration ? TaskPhase.Moving : TaskPhase.Holding; _path.Evaluate(t, position, velocity, acceleration); }
    internal override string? EvaluateOutcome(RunMetrics m, double endTime) =>
        m.SettlingTime is double settled && endTime - settled >= 0.5 - 1e-9 ? null : "Reach: tool did not stay within 2 mm for 0.5 s";
}

/// <summary>Quintic approach and transport with measured pickup/release conditions. SPEC §5.5, §8.2.</summary>
public sealed class PickPlace : ExperimentTask
{
    private readonly QuinticJointTrajectory _approach, _transport;
    private double? _transportStart;
    /// <summary>Creates both smooth legs; starts with the object detached. SPEC §8.2.</summary>
    public PickPlace(TaskSettings settings) : base(settings)
    {
        var pick = TaskGeometry.Solve(settings.ObjectPosition, settings.PickPitch, settings.Branch);
        var place = TaskGeometry.Solve(settings.Target, settings.Pitch, settings.Branch);
        _approach = new(settings.Start.ToArray(), pick.ToArray(), settings.MotionDuration);
        _transport = new(pick.ToArray(), place.ToArray(), settings.MotionDuration);
        ObjectPosition = settings.ObjectPosition;
        SettlingFromTime = double.PositiveInfinity;
    }
    /// <summary>Position of the object, including its actual release position. SPEC §5.5.</summary>
    public Vector3d ObjectPosition { get; private set; }
    /// <summary>Whether the object is attached, even for a zero-mass object. SPEC §5.5.</summary>
    public bool Attached { get; private set; }
    /// <summary>Whether the release event occurred. SPEC §8.2.</summary>
    public bool Released { get; private set; }
    /// <inheritdoc/>
    public override void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration)
    {
        if (_transportStart is double start) _transport.Evaluate(t - start, position, velocity, acceleration);
        else _approach.Evaluate(t, position, velocity, acceleration);
    }
    internal override PayloadEvent Observe(double t, Vector3d tool, double speed)
    {
        if (Attached) ObjectPosition = tool;
        if (Released) return PayloadEvent.None;
        if (_transportStart is not double start)
        {
            Phase = t < Settings.MotionDuration ? TaskPhase.Moving : TaskPhase.Picking;
            if (t >= Settings.MotionDuration)
            {
                if ((tool - ObjectPosition).Length <= 0.01 && speed < 0.05)
                {
                    Attached = true; ObjectPosition = tool; _transportStart = t;
                    SettlingFromTime = t + Settings.MotionDuration; Phase = TaskPhase.Carrying;
                    return PayloadEvent.Attach;
                }
                if (t > Settings.MotionDuration + Settings.EventTimeout) FailureReason = "PickPlace: pickup conditions not met";
            }
        }
        else
        {
            Phase = t < start + Settings.MotionDuration ? TaskPhase.Carrying : TaskPhase.Placing;
            if (t >= start + Settings.MotionDuration)
            {
                if ((tool - FinalTarget).Length <= 0.005 && speed < 0.05)
                {
                    Released = true; Attached = false; ObjectPosition = tool; Phase = TaskPhase.Released;
                    return PayloadEvent.Release;
                }
                if (t > start + Settings.MotionDuration + Settings.EventTimeout) FailureReason = "PickPlace: release target not reached";
            }
        }
        return PayloadEvent.None;
    }
    internal override string? EvaluateOutcome(RunMetrics metrics, double endTime) =>
        Released && (ObjectPosition - FinalTarget).Length <= 0.005 ? null : FailureReason ?? "PickPlace: object was not released at the target";
}

/// <summary>Cartesian line or circle with fixed pitch and smooth timing. SPEC §8.2.</summary>
public sealed class Track : ExperimentTask
{
    private readonly IReferenceSource _path;
    /// <summary>Starts the Cartesian path at the initial tool pose without a reference jump. SPEC §8.2.</summary>
    public Track(TaskSettings settings) : base(settings)
    {
        var start = TaskGeometry.Tool(settings.Start);
        var pitch = settings.Start.J2 + settings.Start.J3 + settings.Start.J4;
        _path = settings.Shape == TrackShape.Line
            ? new CartesianLineTrajectory(start, settings.Target, pitch, settings.MotionDuration, settings.Branch)
            : new CartesianCircleTrajectory(start - new Vector3d(settings.CircleRadius, 0, 0), settings.CircleRadius, pitch, settings.MotionDuration, settings.Branch);
        // Refuse an IK branch/yaw wrap that would introduce a jump at the start.
        Span<double> q = stackalloc double[4]; Span<double> v = stackalloc double[4]; Span<double> a = stackalloc double[4];
        _path.Evaluate(0, q, v, a);
        for (var i = 0; i < 4; i++) if (Math.Abs(q[i] - settings.Start[i]) > 1e-8) throw new TrajectoryException("Trajectory: initial IK branch does not match Start");
        FinalTarget = settings.Shape == TrackShape.Line ? settings.Target : start;
        SettlingFromTime = settings.MotionDuration;
    }
    /// <inheritdoc/>
    public override void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration)
    { Phase = t < Settings.MotionDuration ? TaskPhase.Moving : TaskPhase.Holding; _path.Evaluate(t, position, velocity, acceleration); }
    internal override string? EvaluateOutcome(RunMetrics m, double endTime) =>
        endTime >= Settings.MotionDuration && m.TrackingRmsToolError < Settings.TrackingTolerance ? null : "Track: RMS tool error exceeds the threshold or path is incomplete";
}

/// <summary>Holds the initial pose through an optional external torque pulse. SPEC §8.2.</summary>
public sealed class Hold : ExperimentTask
{
    /// <summary>Uses Start as the constant reference; no step input is introduced. SPEC §8.2.</summary>
    public Hold(TaskSettings settings) : base(settings)
    {
        FinalTarget = TaskGeometry.Tool(settings.Start); Phase = TaskPhase.Holding;
        SettlingFromTime = settings.PulseDuration > 0 ? settings.PulseStart + settings.PulseDuration : 0;
    }
    /// <inheritdoc/>
    public override void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration)
    { Settings.Start.CopyTo(position); velocity.Clear(); acceleration.Clear(); }
    internal override string? EvaluateOutcome(RunMetrics m, double endTime)
    {
        if (Settings.PulseDuration == 0)
            return m.MaxToolError <= 0.002 ? null : "Hold: pose was not held within 2 mm";
        return endTime >= SettlingFromTime + 1 && m.SettlingTime is double t && t <= SettlingFromTime + 1
            ? null : "Hold: did not recover within 2 mm in 1 s after the pulse";
    }
}
