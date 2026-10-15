using System.Collections.ObjectModel;
using RobotArm.Core.Control;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Logging;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Sim = RobotArm.Core.Simulation.Simulation;

namespace RobotArm.Core.Tasks;

/// <summary>Immutable view of the current state for animation frames. SPEC §8.3.</summary>
/// <param name="Time">Simulated time, s.</param>
/// <param name="Q">True joint angles, rad.</param>
/// <param name="QDot">True joint speeds, rad/s.</param>
/// <param name="Tool">Current tool position, m.</param>
/// <param name="PayloadAttached">Object attachment state.</param>
/// <param name="PayloadKg">Attached mass, kg.</param>
/// <param name="ObjectPosition">PickPlace object position, m.</param>
/// <param name="Phase">Current task phase.</param>
/// <param name="SampleCount">Number of controller samples logged.</param>
public readonly record struct ExperimentSnapshot(double Time, JointVector Q, JointVector QDot, Vector3d Tool,
    bool PayloadAttached, double PayloadKg, Vector3d ObjectPosition, TaskPhase Phase, int SampleCount);

/// <summary>Completed run, including its replayable input and sample log. SPEC §8.3, §9, §10.</summary>
/// <param name="ScenarioJson">Full serialized scenario stored with the log.</param>
/// <param name="Log">Read-only sample rows, never changed after completion.</param>
/// <param name="Metrics">Metrics derived from Log.</param>
/// <param name="Success">Whether the task met its criterion without a fault.</param>
/// <param name="FailureReason">Fault or task failure explanation, null on success.</param>
/// <param name="Fault">Physical fault, if any.</param>
/// <param name="Final">Actual end state, which may fall between controller samples.</param>
public sealed record RunResult(string ScenarioJson, IReadOnlyList<LogRow> Log, RunMetrics Metrics,
    bool Success, string? FailureReason, Fault? Fault, ExperimentSnapshot Final);

/// <summary>Deterministic experiment execution, either synchronous or in simulated-time batches. SPEC §8.3.</summary>
public sealed class ExperimentRunner
{
    private Scenario _scenario = null!;
    private string _json = "";
    private Sim _simulation = null!;
    private IController _controller = null!;
    private ISensor _sensor = null!;
    private ExperimentTask? _task;
    private List<LogRow> _rows = [];
    private ReadOnlyCollection<LogRow> _log = new List<LogRow>().AsReadOnly();
    private readonly double[] _qMeasured = new double[4], _vMeasured = new double[4], _qRef = new double[4], _vRef = new double[4], _aRef = new double[4], _u = new double[4], _disturbance = new double[4];
    private int _stepsPerSample;
    private long _stepCount, _totalSteps;
    private double _requestedTime, _beliefPayload;
    private bool _started;

    /// <summary>True after completion or a fault/task failure, false before Start. SPEC §8.3.</summary>
    public bool IsFinished { get; private set; }
    /// <summary>Completed result, or null while running. SPEC §8.3.</summary>
    public RunResult? Result { get; private set; }
    /// <summary>Read-only live log; rows are appended only at controller samples. SPEC §10.</summary>
    public IReadOnlyList<LogRow> Log => _log;
    /// <summary>Current true state; reading it does not advance the simulation or allocate. SPEC §8.3.</summary>
    public ExperimentSnapshot Current
    {
        get
        {
            if (!_started) throw new InvalidOperationException("Call Start first.");
            var tool = ForwardKinematics.ClosedForm(_simulation.Q).Position;
            var pick = _task as PickPlace;
            return new(Math.Min(_scenario.Duration, _stepCount * _scenario.Step), JointVector.From(_simulation.Q), JointVector.From(_simulation.QDot), tool,
                pick?.Attached ?? (_simulation.PayloadKg > 0), _simulation.PayloadKg,
                pick is null ? tool : pick.Attached ? tool : pick.ObjectPosition, _task?.Phase ?? TaskPhase.Moving, _rows.Count);
        }
    }

    /// <summary>Starts or replaces a run; validates input and computes/logs the first controller sample at t=0. SPEC §8.3.</summary>
    public void Start(Scenario scenario)
    {
        ArgumentNullException.ThrowIfNull(scenario);
        scenario.Validate();
        _scenario = scenario; _json = ScenarioJson.Serialize(scenario);
        _totalSteps = (long)Math.Round(scenario.Duration / scenario.Step);
        _stepsPerSample = ControlLoop.StepsPerSamplePeriod(scenario.Step, scenario.SamplePeriod);
        _stepCount = 0; _requestedTime = 0; IsFinished = false; Result = null; _task = null;
        _rows = new List<LogRow>((int)Math.Ceiling((double)_totalSteps / _stepsPerSample)); _log = _rows.AsReadOnly();
        var model = ArmModel.OpenManipulatorX(); var planet = scenario.GetPlanet();
        var payload = scenario.Task.Kind == TaskKind.PickPlace ? 0 : scenario.PayloadKg;
        var plant = new ArmPlant(model, planet); plant.SetPayload(payload);
        var state = new double[8]; scenario.Task.Start.CopyTo(state);
        _simulation = new Sim(plant, new Rk4Solver(8), state);
        var scale = scenario.Belief.MassScale;
        var believedModel = scale == 1 ? model : new ArmModel(model.Joints,
            model.Bodies.Select(b => new BodyParameters(b.Mass * scale, b.CenterOfMass, scale * b.Inertia)), model.ToolPoint, model.Actuators);
        _beliefPayload = scenario.Belief.PayloadKg ?? payload;
        var belief = new ControllerBelief(believedModel, scenario.Belief.Gravity is double g ? Planet.Custom(g) : planet, _beliefPayload);
        _controller = CreateController(scenario.Controller, belief);
        _sensor = scenario.UseEncoder ? new EncoderSensor(model, new EncoderSensorOptions
        { Seed = scenario.Seed, NoiseStdDev = scenario.NoiseStdDev, FilterTimeConstant = scenario.FilterTimeConstant }) : new PerfectSensor(4);
        _started = true;
        try
        {
            _task = scenario.Task.Kind switch
            {
                TaskKind.Reach => new Reach(scenario.Task), TaskKind.PickPlace => new PickPlace(scenario.Task),
                TaskKind.Track => new Track(scenario.Task), TaskKind.Hold => new Hold(scenario.Task),
                _ => throw new ArgumentOutOfRangeException(nameof(scenario))
            };
            PrepareSample();
        }
        catch (TrajectoryException e) { Finish(e.Message); }
    }

    /// <summary>Advances by accumulated simulated seconds, using only whole h steps; fractions carry across calls. SPEC §8.3.</summary>
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
                try { PrepareSample(); }
                catch (TrajectoryException e) { Finish(e.Message); }
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
            else if (_stepCount == _totalSteps) Finish(null);
        }
    }

    /// <summary>Runs the same state machine to completion in one batch. SPEC §8.3.</summary>
    public RunResult Run(Scenario scenario) { Start(scenario); AdvanceBy(scenario.Duration); return Result!; }

    private static IController CreateController(ControllerSettings c, ControllerBelief belief)
    {
        var defaults = c.Method == ControlMethod.CT
            ? c.IntegralEnabled ? ComputedTorque.DefaultIntegralPdGains : ComputedTorque.DefaultGains
            : PdGravity.DefaultGains;
        var kp = c.Kp?.ToArray() ?? defaults.Kp.ToArray(); var kd = c.Kd?.ToArray() ?? defaults.Kd.ToArray();
        return c.Method switch
        {
            ControlMethod.PID => new Pid(belief, new PidGains(kp, c.Ki?.ToArray() ?? Pid.DefaultGains.Ki.ToArray(), kd)),
            ControlMethod.PDG => new PdGravity(belief, new PdGains(kp, kd)),
            ControlMethod.CT => new ComputedTorque(belief, new PdGains(kp, kd), c.PredictHalfSample,
                c.IntegralEnabled ? c.Ki?.ToArray() ?? ComputedTorque.DefaultIntegralGains.ToArray() : null),
            _ => throw new ArgumentOutOfRangeException(nameof(c))
        };
    }

    private void PrepareSample()
    {
        var t = _stepCount * _scenario.Step;
        var pose = ForwardKinematics.ClosedForm(_simulation.Q);
        var j = PlanarJacobian.Compute(_simulation.Q);
        var planarSpeed = j * new Vector3d(_simulation.QDot[1], _simulation.QDot[2], _simulation.QDot[3]);
        var r = ForwardKinematics.Planar(_simulation.Q).R;
        var speed = Math.Sqrt(planarSpeed.X * planarSpeed.X + planarSpeed.Y * planarSpeed.Y + r * r * _simulation.QDot[0] * _simulation.QDot[0]);
        var change = _task!.Observe(t, pose.Position, speed);
        if (_task.FailureReason is string failure) { Finish(failure); return; }
        if (change != PayloadEvent.None)
        {
            var payload = change == PayloadEvent.Attach ? _scenario.PayloadKg : 0;
            _simulation.AttachPayload(payload);
            if (_scenario.Belief.Knowledge == PayloadKnowledge.Informed)
            {
                _beliefPayload = payload;
                if (_controller is ComputedTorque ct) ct.SetPayloadBelief(payload);
                else if (_controller is PdGravity pdg) pdg.SetPayloadBelief(payload);
                else if (_controller is Pid pid) pid.SetPayloadBelief(payload);
            }
        }
        _task.Evaluate(t, _qRef, _vRef, _aRef);
        _sensor.Measure(t, _simulation.Q, _simulation.QDot, _qMeasured, _vMeasured);
        _controller.Compute(t, _qMeasured, _vMeasured, new JointReference(_qRef, _vRef, _aRef), _u);
        Span<double> applied = stackalloc double[4]; var clamped = 0; var saturated = 0;
        for (var i = 0; i < 4; i++)
        {
            var limit = _simulation.Plant.Model.Actuators[i].StallTorque;
            applied[i] = Math.Clamp(_u[i], -limit, limit);
            if (_controller.Clamped[i]) clamped |= 1 << i;
            if (Math.Abs(_u[i]) > limit) saturated |= 1 << i;
        }
        SetDisturbance(t);
        var current = Current;
        _rows.Add(new(t, 0, current.Q, current.QDot, JointVector.From(_qRef), JointVector.From(_vRef), JointVector.From(_aRef),
            JointVector.From(_u), JointVector.From(applied), clamped, saturated, pose.Position, ForwardKinematics.ClosedForm(_qRef).Position,
            current.PayloadAttached, current.PayloadKg, _beliefPayload, _task.Phase, current.ObjectPosition, JointVector.From(_disturbance)));
    }

    private void SetDisturbance(double t)
    {
        var task = _scenario.Task;
        if (task.Kind == TaskKind.Hold && t >= task.PulseStart && t < task.PulseStart + task.PulseDuration) task.Disturbance.CopyTo(_disturbance);
        else Array.Clear(_disturbance);
        _simulation.Plant.SetDisturbance(_disturbance);
    }

    private void Finish(string? failure)
    {
        var metrics = RunMetrics.Compute(_log, _task?.FinalTarget ?? _scenario.Task.Target,
            _task?.SettlingFromTime ?? double.PositiveInfinity, _scenario.Task.MotionDuration);
        failure ??= _task?.EvaluateOutcome(metrics, _stepCount * _scenario.Step);
        IsFinished = true;
        Result = new(_json, _log, metrics, failure is null, failure, _simulation.Fault, Current);
    }
}
