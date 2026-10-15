using System.Text.Json;
using System.Text.Json.Serialization;
using RobotArm.Core.Control;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using Sim = RobotArm.Core.Simulation.Simulation;

namespace RobotArm.Core.Tasks;

/// <summary>Experiment task keys. SPEC §8.2.</summary>
public enum TaskKind
{
    /// <summary>Quintic joint move followed by a hold. SPEC §8.2.</summary>
    Reach,
    /// <summary>Quintic approach, attach, transport and release. SPEC §8.2.</summary>
    PickPlace,
    /// <summary>Cartesian tracking. SPEC §8.2.</summary>
    Track,
    /// <summary>Constant pose with an optional external torque pulse. SPEC §8.2.</summary>
    Hold
}
/// <summary>Controller selection; CT's integral option is separate. SPEC §7.</summary>
public enum ControlMethod
{
    /// <summary>Independent joint PID. SPEC §7.</summary>
    PID,
    /// <summary>PD plus gravity. SPEC §7.</summary>
    PDG,
    /// <summary>Computed torque, optionally CT+I. SPEC §7.</summary>
    CT
}
/// <summary>Whether PickPlace events update the controller's payload model. SPEC §8.3.</summary>
public enum PayloadKnowledge
{
    /// <summary>Attachment and release update the believed payload. SPEC §8.3.</summary>
    Informed,
    /// <summary>The starting payload belief stays fixed. SPEC §8.3.</summary>
    Unaware
}
/// <summary>Cartesian tracking shape. SPEC §8.1.</summary>
public enum TrackShape
{
    /// <summary>Straight line from the initial tool point. SPEC §8.1.</summary>
    Line,
    /// <summary>Horizontal circle through the initial tool point. SPEC §8.1.</summary>
    Circle
}

/// <summary>Immutable task parameters in SI units. SPEC §8.2.</summary>
public sealed record TaskSettings
{
    /// <summary>Task selection. SPEC §8.2.</summary>
    public TaskKind Kind { get; init; } = TaskKind.Reach;
    /// <summary>Initial joint pose; initial speed is zero. SPEC §8.2.</summary>
    public JointVector Start { get; init; } = new(0, -0.4, -0.3, 1.1);
    /// <summary>Reach/Place target tool point. SPEC §8.2.</summary>
    public Vector3d Target { get; init; } = TaskGeometry.Tool(new(0, 0.1, -0.8, 0.6));
    /// <summary>Target tool pitch, rad. SPEC §8.2.</summary>
    public double Pitch { get; init; } = -0.1;
    /// <summary>IK branch for targets and Cartesian paths. SPEC §8.1.</summary>
    public ElbowBranch Branch { get; init; } = ElbowBranch.Up;
    /// <summary>Duration of Reach/Track, or each PickPlace leg, s. SPEC §8.1.</summary>
    public double MotionDuration { get; init; } = 2;
    /// <summary>Object position before pickup. SPEC §5.5, §8.2.</summary>
    public Vector3d ObjectPosition { get; init; } = TaskGeometry.Tool(new(0, 0.1, -0.8, 0.6));
    /// <summary>Tool pitch during pickup, rad. SPEC §8.2.</summary>
    public double PickPitch { get; init; } = -0.1;
    /// <summary>Maximum pickup or release wait after a leg, s. SPEC §8.2.</summary>
    public double EventTimeout { get; init; } = 2;
    /// <summary>Cartesian tracking shape. SPEC §8.2.</summary>
    public TrackShape Shape { get; init; } = TrackShape.Circle;
    /// <summary>Circle radius, m; center is start tool point minus (radius,0,0). SPEC §8.1.</summary>
    public double CircleRadius { get; init; } = 0.02;
    /// <summary>RMS tracking success threshold, m. SPEC §8.2.</summary>
    public double TrackingTolerance { get; init; } = 0.002;
    /// <summary>Hold external torque pulse, N·m. SPEC §8.2.</summary>
    public JointVector Disturbance { get; init; } = new(0, 0.2, 0, 0);
    /// <summary>Pulse start, s. SPEC §8.2.</summary>
    public double PulseStart { get; init; } = 1;
    /// <summary>Pulse duration, s; zero disables the pulse. SPEC §8.2.</summary>
    public double PulseDuration { get; init; } = 0.1;
}

/// <summary>Believed environment, mass scale and payload policy. SPEC §5.6, §8.3.</summary>
public sealed record BeliefSettings
{
    /// <summary>Null uses the true planet's g; otherwise custom g in 0..30. SPEC §8.3.</summary>
    public double? Gravity { get; init; }
    /// <summary>Null uses initial plant payload; PickPlace initially has no attached payload. SPEC §8.3.</summary>
    public double? PayloadKg { get; init; }
    /// <summary>Scale of believed body masses and inertia tensors, excluding the payload. SPEC §5.6.</summary>
    public double MassScale { get; init; } = 1;
    /// <summary>Payload information policy at PickPlace events. SPEC §8.3.</summary>
    public PayloadKnowledge Knowledge { get; init; } = PayloadKnowledge.Informed;
}

/// <summary>Controller law and optional gain overrides. SPEC §7, §8.3.</summary>
public sealed record ControllerSettings
{
    /// <summary>Control law. SPEC §7.</summary>
    public ControlMethod Method { get; init; } = ControlMethod.CT;
    /// <summary>Optional integral acceleration feedback for CT, disabled by default. SPEC §7.</summary>
    public bool IntegralEnabled { get; init; }
    /// <summary>Optional Kp override; null selects the documented default. SPEC §7.</summary>
    public JointVector? Kp { get; init; }
    /// <summary>Optional Kd override. SPEC §7.</summary>
    public JointVector? Kd { get; init; }
    /// <summary>Optional PID or CT+I Ki override. SPEC §7.</summary>
    public JointVector? Ki { get; init; }
    /// <summary>CT midpoint prediction. SPEC §7.</summary>
    public bool PredictHalfSample { get; init; } = true;
}

/// <summary>Complete, JSON-serializable experiment input. All values are immutable. SPEC §8.3.</summary>
public sealed record Scenario
{
    /// <summary>Turkish display name. SPEC §8.3.</summary>
    public string Name { get; init; } = "Deney";
    /// <summary>One-line teaching explanation. SPEC §8.3.</summary>
    public string Description { get; init; } = "";
    /// <summary>Library assertion about the demonstrated outcome. SPEC §8.3.</summary>
    public bool ExpectedSuccess { get; init; } = true;
    /// <summary>Planet key of SPEC §3.5, or Custom. SPEC §8.3.</summary>
    public string PlanetKey { get; init; } = "Earth";
    /// <summary>g when PlanetKey is Custom. SPEC §8.3.</summary>
    public double CustomGravity { get; init; } = Planet.Earth.G;
    /// <summary>Attached mass for Reach/Track/Hold, or object mass for PickPlace, 0..0.5 kg. SPEC §8.3.</summary>
    public double PayloadKg { get; init; }
    /// <summary>Task and reference settings. SPEC §8.3.</summary>
    public TaskSettings Task { get; init; } = new();
    /// <summary>Controller and gains. SPEC §8.3.</summary>
    public ControllerSettings Controller { get; init; } = new();
    /// <summary>Independent controller model. SPEC §8.3.</summary>
    public BeliefSettings Belief { get; init; } = new();
    /// <summary>RK4 step, s. SPEC §6.2, §8.3.</summary>
    public double Step { get; init; } = Sim.DefaultStep;
    /// <summary>Controller period, an integer multiple of Step. SPEC §6.3, §8.3.</summary>
    public double SamplePeriod { get; init; } = ControlLoop.DefaultSamplePeriod;
    /// <summary>Use encoders instead of the perfect sensor. SPEC §8.3.</summary>
    public bool UseEncoder { get; init; }
    /// <summary>Encoder speed filter time constant, s. SPEC §8.3.</summary>
    public double FilterTimeConstant { get; init; } = 0.005;
    /// <summary>Angle noise standard deviation, rad. SPEC §8.3.</summary>
    public double NoiseStdDev { get; init; }
    /// <summary>Noise seed. SPEC §8.3.</summary>
    public int Seed { get; init; } = 445;
    /// <summary>Run duration, (0,60] s, an integer multiple of Step. SPEC §8.3.</summary>
    public double Duration { get; init; } = 10;

    /// <summary>Resolves the environment, validating custom gravity. SPEC §8.3.</summary>
    public Planet GetPlanet() => PlanetKey == Planet.CustomKey ? Planet.Custom(CustomGravity)
        : Planet.All.FirstOrDefault(p => p.Key == PlanetKey) ?? throw new ArgumentOutOfRangeException(nameof(PlanetKey));

    /// <summary>Rejects invalid scenarios before allocating a run or taking any simulation steps. SPEC §8.3.</summary>
    public void Validate()
    {
        ArgumentNullException.ThrowIfNull(Task); ArgumentNullException.ThrowIfNull(Controller); ArgumentNullException.ThrowIfNull(Belief);
        _ = GetPlanet();
        if (!(Step >= Sim.MinStep && Step <= Sim.MaxStep)) throw new ArgumentOutOfRangeException(nameof(Step));
        _ = ControlLoop.StepsPerSamplePeriod(Step, SamplePeriod);
        if (!double.IsFinite(Duration) || Duration <= 0 || Duration > 60) throw new ArgumentOutOfRangeException(nameof(Duration), "Duration must be in (0,60] simulated seconds (SPEC §8.3).");
        if (Math.Abs(Math.Round(Duration / Step) * Step - Duration) > 1e-9 * Duration) throw new ArgumentOutOfRangeException(nameof(Duration), "Duration must be an integer multiple of h.");
        Range(PayloadKg, 0, 0.5, nameof(PayloadKg));
        if (Belief.PayloadKg is double payload) Range(payload, 0, 0.5, nameof(Belief.PayloadKg));
        if (Belief.Gravity is double gravity) _ = Planet.Custom(gravity);
        TimeLaw.ValidateDuration(Belief.MassScale);
        Range(FilterTimeConstant, 0, double.MaxValue, nameof(FilterTimeConstant)); Range(NoiseStdDev, 0, double.MaxValue, nameof(NoiseStdDev));
        if (!Enum.IsDefined(Task.Kind) || !Enum.IsDefined(Task.Shape) || !Enum.IsDefined(Task.Branch) || !Enum.IsDefined(Controller.Method) || !Enum.IsDefined(Belief.Knowledge)) throw new ArgumentOutOfRangeException(nameof(Task));
        Controller.Kp?.Validate(nameof(Controller.Kp), true); Controller.Kd?.Validate(nameof(Controller.Kd), true); Controller.Ki?.Validate(nameof(Controller.Ki), true);
        if (Controller.IntegralEnabled && Controller.Method != ControlMethod.CT) throw new ArgumentException("IntegralEnabled is the CT+I option.");
        Task.Start.Validate(nameof(Task.Start)); Task.Disturbance.Validate(nameof(Task.Disturbance));
        var model = ArmModel.OpenManipulatorX();
        for (var i = 0; i < 4; i++) if (!model.Joints[i].IsWithinLimits(Task.Start[i])) throw new ArgumentOutOfRangeException(nameof(Task.Start), "Initial pose is outside joint limits.");
        if (TaskGeometry.Tool(Task.Start).Z < 0) throw new ArgumentOutOfRangeException(nameof(Task.Start), "Initial tool is below the floor.");
        if (!Task.Target.IsFinite || !Task.ObjectPosition.IsFinite || !double.IsFinite(Task.Pitch) || !double.IsFinite(Task.PickPitch)) throw new ArgumentOutOfRangeException(nameof(Task));
        TimeLaw.ValidateDuration(Task.MotionDuration); TimeLaw.ValidateDuration(Task.EventTimeout); TimeLaw.ValidateDuration(Task.CircleRadius); TimeLaw.ValidateDuration(Task.TrackingTolerance);
        Range(Task.PulseStart, 0, 60, nameof(Task.PulseStart)); Range(Task.PulseDuration, 0, 60, nameof(Task.PulseDuration));
        if (Task.Kind != TaskKind.Hold && Task.MotionDuration > Duration) throw new ArgumentOutOfRangeException(nameof(Task.MotionDuration));
    }
    private static void Range(double value, double min, double max, string name)
    { if (!double.IsFinite(value) || value < min || value > max) throw new ArgumentOutOfRangeException(name); }
}

/// <summary>System.Text.Json scenario interchange with strict enum names and vector support. SPEC §8.3, §10.</summary>
public static class ScenarioJson
{
    private static readonly JsonSerializerOptions Options = CreateOptions();
    /// <summary>Validates and serializes the complete input. SPEC §8.3.</summary>
    public static string Serialize(Scenario scenario) { scenario.Validate(); return JsonSerializer.Serialize(scenario, Options); }
    /// <summary>Reads and validates the complete input. SPEC §8.3.</summary>
    public static Scenario Deserialize(string json)
    {
        var scenario = JsonSerializer.Deserialize<Scenario>(json, Options) ?? throw new JsonException("Expected a scenario.");
        scenario.Validate(); return scenario;
    }
    private static JsonSerializerOptions CreateOptions()
    {
        var options = new JsonSerializerOptions { WriteIndented = true, UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow };
        options.Converters.Add(new JsonStringEnumConverter(allowIntegerValues: false));
        options.Converters.Add(new VectorConverter()); return options;
    }
    private sealed class VectorConverter : JsonConverter<Vector3d>
    {
        public override Vector3d Read(ref Utf8JsonReader reader, Type type, JsonSerializerOptions options)
        {
            using var document = JsonDocument.ParseValue(ref reader);
            var root = document.RootElement;
            return new(root.GetProperty("X").GetDouble(), root.GetProperty("Y").GetDouble(), root.GetProperty("Z").GetDouble());
        }
        public override void Write(Utf8JsonWriter writer, Vector3d value, JsonSerializerOptions options)
        { writer.WriteStartObject(); writer.WriteNumber("X", value.X); writer.WriteNumber("Y", value.Y); writer.WriteNumber("Z", value.Z); writer.WriteEndObject(); }
    }
}

internal static class TaskGeometry
{
    internal static Vector3d Tool(JointVector q) { Span<double> a = stackalloc double[4]; q.CopyTo(a); return ForwardKinematics.ClosedForm(a).Position; }
    internal static JointVector Solve(Vector3d p, double pitch, ElbowBranch branch)
    {
        var solution = InverseKinematics.Solve(p, pitch, branch);
        if (!solution.Success) throw new TrajectoryException($"Trajectory: {solution.Failure}");
        return new(solution.Q[0], solution.Q[1], solution.Q[2], solution.Q[3]);
    }
}
