using System.Collections.ObjectModel;
using RobotArm.Core.Control;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Logging;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using RobotArm.Core.Tasks;
using Sim = RobotArm.Core.Simulation.Simulation;

namespace RobotArm.Web.Models;

/// <summary>
/// Deterministic experiment runner for <see cref="JsController"/> ("Kendi kontrolcün", T10b).
/// Executes the exact same step/sample loop and task state machine as <see cref="ExperimentRunner"/>
/// while keeping <c>RobotArm.Core</c> unchanged.
/// </summary>
public sealed class CustomExperimentRunner
{
    private enum PickPayloadEvent { None, Attach, Release }

    private sealed class PickPlaceTracker
    {
        private readonly TaskSettings _settings;
        private readonly QuinticJointTrajectory _approach;
        private readonly QuinticJointTrajectory _transport;
        private double? _transportStart;

        public PickPlaceTracker(TaskSettings settings)
        {
            _settings = settings;
            var pick = SolveIk(settings.ObjectPosition, settings.PickPitch, settings.Branch);
            var place = SolveIk(settings.Target, settings.Pitch, settings.Branch);
            _approach = new QuinticJointTrajectory(settings.Start.ToArray(), pick.ToArray(), settings.MotionDuration);
            _transport = new QuinticJointTrajectory(pick.ToArray(), place.ToArray(), settings.MotionDuration);
            ObjectPosition = settings.ObjectPosition;
            FinalTarget = settings.Target;
            SettlingFromTime = double.PositiveInfinity;
            Phase = TaskPhase.Moving;
        }

        public Vector3d ObjectPosition { get; private set; }
        public Vector3d FinalTarget { get; }
        public double SettlingFromTime { get; private set; }
        public TaskPhase Phase { get; private set; }
        public bool Attached { get; private set; }
        public bool Released { get; private set; }
        public string? FailureReason { get; private set; }

        public void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration)
        {
            if (_transportStart is double start)
            {
                _transport.Evaluate(t - start, position, velocity, acceleration);
            }
            else
            {
                _approach.Evaluate(t, position, velocity, acceleration);
            }
        }

        public PickPayloadEvent Observe(double t, Vector3d tool, double speed)
        {
            if (Attached) ObjectPosition = tool;
            if (Released) return PickPayloadEvent.None;
            if (_transportStart is not double start)
            {
                Phase = t < _settings.MotionDuration ? TaskPhase.Moving : TaskPhase.Picking;
                if (t >= _settings.MotionDuration)
                {
                    if ((tool - ObjectPosition).Length <= 0.01 && speed < 0.05)
                    {
                        Attached = true;
                        ObjectPosition = tool;
                        _transportStart = t;
                        SettlingFromTime = t + _settings.MotionDuration;
                        Phase = TaskPhase.Carrying;
                        return PickPayloadEvent.Attach;
                    }
                    if (t > _settings.MotionDuration + _settings.EventTimeout)
                    {
                        FailureReason = "PickPlace: pickup conditions not met";
                    }
                }
            }
            else
            {
                Phase = t < start + _settings.MotionDuration ? TaskPhase.Carrying : TaskPhase.Placing;
                if (t >= start + _settings.MotionDuration)
                {
                    if ((tool - FinalTarget).Length <= 0.005 && speed < 0.05)
                    {
                        Released = true;
                        Attached = false;
                        ObjectPosition = tool;
                        Phase = TaskPhase.Released;
                        return PickPayloadEvent.Release;
                    }
                    if (t > start + _settings.MotionDuration + _settings.EventTimeout)
                    {
                        FailureReason = "PickPlace: release target not reached";
                    }
                }
            }
            return PickPayloadEvent.None;
        }

        public string? EvaluateOutcome() =>
            Released && (ObjectPosition - FinalTarget).Length <= 0.005
                ? null
                : FailureReason ?? "PickPlace: object was not released at the target";

        private static JointVector SolveIk(Vector3d p, double pitch, ElbowBranch branch)
        {
            var solution = InverseKinematics.Solve(p, pitch, branch);
            if (!solution.Success) throw new TrajectoryException($"Trajectory: {solution.Failure}");
            return new JointVector(solution.Q[0], solution.Q[1], solution.Q[2], solution.Q[3]);
        }
    }

    private Scenario _scenario = null!;
    private string _json = "";
    private Sim _simulation = null!;
    private JsController _controller = null!;
    private ISensor _sensor = null!;
    private ExperimentTask? _task;
    private PickPlaceTracker? _pickPlace;
    private Fault? _customFault;
    private List<LogRow> _rows = [];
    private ReadOnlyCollection<LogRow> _log = new List<LogRow>().AsReadOnly();
    private readonly double[] _qMeasured = new double[4], _vMeasured = new double[4], _qRef = new double[4], _vRef = new double[4], _aRef = new double[4], _u = new double[4], _disturbance = new double[4];
    private int _stepsPerSample;
    private long _stepCount, _totalSteps;
    private double _requestedTime, _beliefPayload;
    private bool _started;

    public bool IsFinished { get; private set; }
    public RunResult? Result { get; private set; }
    public IReadOnlyList<LogRow> Log => _log;

    public ExperimentSnapshot Current
    {
        get
        {
            if (!_started) throw new InvalidOperationException("Call Start first.");
            var tool = ForwardKinematics.ClosedForm(_simulation.Q).Position;
            var phase = _pickPlace?.Phase ?? _task?.Phase ?? TaskPhase.Moving;
            return new ExperimentSnapshot(
                Math.Min(_scenario.Duration, _stepCount * _scenario.Step),
                JointVector.From(_simulation.Q),
                JointVector.From(_simulation.QDot),
                tool,
                _pickPlace?.Attached ?? (_simulation.PayloadKg > 0),
                _simulation.PayloadKg,
                _pickPlace is null ? tool : _pickPlace.Attached ? tool : _pickPlace.ObjectPosition,
                phase,
                _rows.Count);
        }
    }

    public void Start(Scenario scenario, JsController controller, string scenarioJson)
    {
        ArgumentNullException.ThrowIfNull(scenario);
        ArgumentNullException.ThrowIfNull(controller);
        scenario.Validate();

        _scenario = scenario;
        _json = scenarioJson;
        _totalSteps = (long)Math.Round(scenario.Duration / scenario.Step);
        _stepsPerSample = ControlLoop.StepsPerSamplePeriod(scenario.Step, scenario.SamplePeriod);
        _stepCount = 0;
        _requestedTime = 0;
        IsFinished = false;
        Result = null;
        _task = null;
        _pickPlace = null;
        _customFault = null;
        _rows = new List<LogRow>((int)Math.Ceiling((double)_totalSteps / _stepsPerSample));
        _log = _rows.AsReadOnly();

        var model = ArmModel.OpenManipulatorX();
        var planet = scenario.GetPlanet();
        var payload = scenario.Task.Kind == TaskKind.PickPlace ? 0 : scenario.PayloadKg;
        var plant = new ArmPlant(model, planet);
        plant.SetPayload(payload);

        var state = new double[8];
        scenario.Task.Start.CopyTo(state);
        _simulation = new Sim(plant, new Rk4Solver(8), state);
        _beliefPayload = scenario.Belief.PayloadKg ?? payload;
        _controller = controller;
        _controller.Reset();
        _sensor = scenario.UseEncoder
            ? new EncoderSensor(model, new EncoderSensorOptions
            {
                Seed = scenario.Seed,
                NoiseStdDev = scenario.NoiseStdDev,
                FilterTimeConstant = scenario.FilterTimeConstant
            })
            : new PerfectSensor(4);
        _started = true;

        try
        {
            if (scenario.Task.Kind == TaskKind.PickPlace)
            {
                _pickPlace = new PickPlaceTracker(scenario.Task);
            }
            else
            {
                _task = scenario.Task.Kind switch
                {
                    TaskKind.Reach => new Reach(scenario.Task),
                    TaskKind.Track => new Track(scenario.Task),
                    TaskKind.Hold => new Hold(scenario.Task),
                    _ => throw new ArgumentOutOfRangeException(nameof(scenario))
                };
            }
            PrepareSample();
        }
        catch (TrajectoryException e)
        {
            Finish(e.Message);
        }
        catch (CustomControllerFaultException e)
        {
            _customFault = new Fault(FaultReason.NonFinite, _stepCount * _scenario.Step, -1, double.NaN);
            Finish(e.Message);
        }
    }

    public void AdvanceBy(double simSeconds)
    {
        if (!double.IsFinite(simSeconds) || simSeconds < 0) throw new ArgumentOutOfRangeException(nameof(simSeconds));
        if (!_started) throw new InvalidOperationException("Call Start first.");
        if (IsFinished) return;

        _requestedTime = Math.Min(_scenario.Duration, _requestedTime + simSeconds);
        var target = Math.Min(_totalSteps, (long)Math.Floor(_requestedTime / _scenario.Step + 1e-8));
        while (_stepCount < target && !IsFinished)
        {
            if (_stepCount > 0 && _stepCount % _stepsPerSample == 0)
            {
                try
                {
                    PrepareSample();
                }
                catch (TrajectoryException e)
                {
                    Finish(e.Message);
                }
                catch (CustomControllerFaultException e)
                {
                    _customFault = new Fault(FaultReason.NonFinite, _stepCount * _scenario.Step, -1, double.NaN);
                    Finish(e.Message);
                }
                if (IsFinished) break;
            }

            SetDisturbance(_stepCount * _scenario.Step);
            var running = _simulation.Advance(_u, _scenario.Step);
            _stepCount++;
            var row = _rows[^1];
            _rows[^1] = row with { Interval = Math.Min(_scenario.Duration, _stepCount * _scenario.Step) - row.Time };
            if (!running)
            {
                var fault = _simulation.Fault!;
                Finish(FormattableString.Invariant($"{fault.Reason}: t={fault.Time:R} s, joint={fault.Joint + 1}, value={fault.Value:R}"));
            }
            else if (_stepCount == _totalSteps)
            {
                Finish(null);
            }
        }
    }

    public RunResult Run(Scenario scenario, JsController controller, string scenarioJson)
    {
        Start(scenario, controller, scenarioJson);
        AdvanceBy(scenario.Duration);
        return Result!;
    }

    private void PrepareSample()
    {
        var t = _stepCount * _scenario.Step;
        var pose = ForwardKinematics.ClosedForm(_simulation.Q);
        var j = PlanarJacobian.Compute(_simulation.Q);
        var planarSpeed = j * new Vector3d(_simulation.QDot[1], _simulation.QDot[2], _simulation.QDot[3]);
        var r = ForwardKinematics.Planar(_simulation.Q).R;
        var speed = Math.Sqrt(planarSpeed.X * planarSpeed.X + planarSpeed.Y * planarSpeed.Y + r * r * _simulation.QDot[0] * _simulation.QDot[0]);

        if (_pickPlace is not null)
        {
            var change = _pickPlace.Observe(t, pose.Position, speed);
            if (_pickPlace.FailureReason is string failure)
            {
                Finish(failure);
                return;
            }
            if (change != PickPayloadEvent.None)
            {
                var payload = change == PickPayloadEvent.Attach ? _scenario.PayloadKg : 0;
                _simulation.AttachPayload(payload);
                if (_scenario.Belief.Knowledge == PayloadKnowledge.Informed)
                {
                    _beliefPayload = payload;
                    _controller.SetPayloadBelief(payload);
                }
            }
            _pickPlace.Evaluate(t, _qRef, _vRef, _aRef);
        }
        else
        {
            _task!.Evaluate(t, _qRef, _vRef, _aRef);
        }

        _sensor.Measure(t, _simulation.Q, _simulation.QDot, _qMeasured, _vMeasured);
        _controller.Compute(t, _qMeasured, _vMeasured, new JointReference(_qRef, _vRef, _aRef), _u);

        Span<double> applied = stackalloc double[4];
        var clamped = 0;
        var saturated = 0;
        for (var i = 0; i < 4; i++)
        {
            var limit = _simulation.Plant.Model.Actuators[i].StallTorque;
            applied[i] = Math.Clamp(_u[i], -limit, limit);
            if (_controller.Clamped[i]) clamped |= 1 << i;
            if (Math.Abs(_u[i]) > limit) saturated |= 1 << i;
        }

        SetDisturbance(t);
        var current = Current;
        var phase = _pickPlace?.Phase ?? _task!.Phase;
        _rows.Add(new LogRow(
            t,
            0,
            current.Q,
            current.QDot,
            JointVector.From(_qRef),
            JointVector.From(_vRef),
            JointVector.From(_aRef),
            JointVector.From(_u),
            JointVector.From(applied),
            clamped,
            saturated,
            pose.Position,
            ForwardKinematics.ClosedForm(_qRef).Position,
            current.PayloadAttached,
            current.PayloadKg,
            _beliefPayload,
            phase,
            current.ObjectPosition,
            JointVector.From(_disturbance)));
    }

    private void SetDisturbance(double t)
    {
        var task = _scenario.Task;
        if (task.Kind == TaskKind.Hold && t >= task.PulseStart && t < task.PulseStart + task.PulseDuration)
        {
            task.Disturbance.CopyTo(_disturbance);
        }
        else
        {
            Array.Clear(_disturbance);
        }
        _simulation.Plant.SetDisturbance(_disturbance);
    }

    private void Finish(string? failure)
    {
        var finalTarget = _pickPlace?.FinalTarget ?? _task?.FinalTarget ?? _scenario.Task.Target;
        var settlingFrom = _pickPlace?.SettlingFromTime ?? _task?.SettlingFromTime ?? double.PositiveInfinity;
        var metrics = RunMetrics.Compute(_log, finalTarget, settlingFrom, _scenario.Task.MotionDuration);
        var endTime = _stepCount * _scenario.Step;
        failure ??= EvaluateTaskOutcome(metrics, endTime);
        IsFinished = true;
        Result = new RunResult(_json, _log, metrics, failure is null, failure, _customFault ?? _simulation.Fault, Current);
    }

    private string? EvaluateTaskOutcome(RunMetrics m, double endTime)
    {
        if (_pickPlace is not null)
        {
            return _pickPlace.EvaluateOutcome();
        }
        if (_task is null)
        {
            return null;
        }
        return _scenario.Task.Kind switch
        {
            TaskKind.Reach => m.SettlingTime is double settled && endTime - settled >= 0.5 - 1e-9
                ? null
                : "Reach: tool did not stay within 2 mm for 0.5 s",
            TaskKind.Track => endTime >= _scenario.Task.MotionDuration && m.TrackingRmsToolError < _scenario.Task.TrackingTolerance
                ? null
                : "Track: RMS tool error exceeds the threshold or path is incomplete",
            TaskKind.Hold => _scenario.Task.PulseDuration == 0
                ? (m.MaxToolError <= 0.002 ? null : "Hold: pose was not held within 2 mm")
                : (endTime >= _task.SettlingFromTime + 1 && m.SettlingTime is double t && t <= _task.SettlingFromTime + 1
                    ? null
                    : "Hold: did not recover within 2 mm in 1 s after the pulse"),
            _ => null
        };
    }
}
