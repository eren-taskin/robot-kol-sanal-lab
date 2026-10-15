using System.Globalization;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;

namespace RobotArm.Core.Simulation;

/// <summary>
/// One running experiment of the plant: owns the time and a copy of the state, advances them with a
/// solver, checks the faults of SPEC §6.4 after each step, and switches the payload at step boundaries
/// (SPEC §5.5). SPEC §6.1, §6.2, §6.4.
/// </summary>
/// <remarks>
/// <see cref="Advance"/> does not allocate (PROJECT.md §3). Once a fault is set the simulation is stopped:
/// further calls to <see cref="Advance"/> throw. The initial state is not checked; the first check is after
/// the first step.
/// </remarks>
public sealed class Simulation
{
    /// <summary>Smallest allowed step, s. SPEC §6.2: 0.1 ms.</summary>
    public const double MinStep = 1e-4;

    /// <summary>Largest allowed step, s. SPEC §6.2: 5 ms.</summary>
    public const double MaxStep = 5e-3;

    /// <summary>Default step, s. SPEC §6.2: 0.5 ms (docs/DECISIONS.md D3).</summary>
    public const double DefaultStep = 5e-4;

    private readonly ArmPlant _plant;
    private readonly ISolver _solver;
    private readonly int _n;
    private readonly double[] _state;
    private readonly double[] _appliedTorque;
    private readonly bool[] _saturated;
    private readonly Frame[] _frames;

    /// <summary>Starts a simulation of <paramref name="plant"/> from <paramref name="initialState"/>, which is copied.</summary>
    /// <param name="plant">The true arm (SPEC §5.6).</param>
    /// <param name="solver">The integrator, sized for the plant's state (SPEC §6.2).</param>
    /// <param name="initialState">x0 = [q; q̇], length <see cref="ArmPlant.StateCount"/>.</param>
    /// <param name="checks">
    /// The fault checks to run after each step (SPEC §6.4); all by default. The non-finite check always runs.
    /// </param>
    /// <param name="startTime">Initial time, s.</param>
    /// <exception cref="ArgumentNullException">The plant or solver is null.</exception>
    /// <exception cref="ArgumentException">The initial state has the wrong length.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The start time is not finite.</exception>
    public Simulation(
        ArmPlant plant, ISolver solver, ReadOnlySpan<double> initialState,
        FaultChecks checks = FaultChecks.All, double startTime = 0.0)
    {
        ArgumentNullException.ThrowIfNull(plant);
        ArgumentNullException.ThrowIfNull(solver);
        if (initialState.Length != plant.StateCount)
            throw new ArgumentException(
                $"Expected {plant.StateCount} initial state values, got {initialState.Length} (SPEC §6.1).", nameof(initialState));
        if (!double.IsFinite(startTime))
            throw new ArgumentOutOfRangeException(nameof(startTime), startTime, "Start time must be finite.");

        _plant = plant;
        _solver = solver;
        _n = plant.DegreesOfFreedom;
        _state = initialState.ToArray();
        _appliedTorque = new double[_n];
        _saturated = new bool[_n];
        _frames = new Frame[_n];
        Checks = checks;
        Time = startTime;
    }

    /// <summary>The plant being simulated.</summary>
    public ArmPlant Plant => _plant;

    /// <summary>Simulated time, s.</summary>
    public double Time { get; private set; }

    /// <summary>Number of completed steps.</summary>
    public long StepCount { get; private set; }

    /// <summary>The state x = [q; q̇]. SPEC §2.</summary>
    public ReadOnlySpan<double> State => _state;

    /// <summary>Joint angles q, rad.</summary>
    public ReadOnlySpan<double> Q => _state.AsSpan(0, _n);

    /// <summary>Joint speeds q̇, rad/s.</summary>
    public ReadOnlySpan<double> QDot => _state.AsSpan(_n, _n);

    /// <summary>Torques applied in the last step, after saturation, N·m. SPEC §5.4.</summary>
    public ReadOnlySpan<double> AppliedTorque => _appliedTorque;

    /// <summary>
    /// For each joint, whether the command was saturated in the last step. SPEC §5.4. The input is held over
    /// the step (SPEC §6.2), so the flags are the same for every stage of the solver.
    /// </summary>
    public ReadOnlySpan<bool> Saturated => _saturated;

    /// <summary>
    /// The optional fault checks run after each step. SPEC §6.4. May be changed between steps. A non-finite
    /// state is always a <see cref="FaultReason.NonFinite"/> fault, whether or not <see cref="FaultChecks.NonFinite"/>
    /// is set (docs/DECISIONS.md D5).
    /// </summary>
    public FaultChecks Checks { get; set; }

    /// <summary>The fault that stopped the simulation, or null while it runs. SPEC §6.4.</summary>
    public Fault? Fault { get; private set; }

    /// <summary>True once a fault has stopped the simulation.</summary>
    public bool IsStopped => Fault is not null;

    /// <summary>Attached payload mass in kg, 0 when none. SPEC §5.5.</summary>
    public double PayloadKg => _plant.PayloadKg;

    /// <summary>
    /// Advances the state by one step of length <paramref name="h"/> with <paramref name="u"/> held over the
    /// step, then runs the fault checks. SPEC §6.2, §6.4. Does not allocate.
    /// </summary>
    /// <returns>True if the simulation is still running, false if this step produced a fault.</returns>
    /// <exception cref="InvalidOperationException">A fault has already stopped the simulation.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="h"/> is outside 0.1 ms to 5 ms.</exception>
    /// <exception cref="ArgumentException"><paramref name="u"/> has the wrong length.</exception>
    public bool Advance(ReadOnlySpan<double> u, double h)
    {
        if (Fault is not null)
            throw new InvalidOperationException(string.Create(CultureInfo.InvariantCulture,
                $"The simulation stopped at t = {Fault.Time} s with a {Fault.Reason} fault (SPEC §6.4)."));
        if (!(h >= MinStep && h <= MaxStep))
            throw new ArgumentOutOfRangeException(nameof(h), h, "Step must be between 0.1 ms and 5 ms (SPEC §6.2).");

        _solver.Step(_plant, Time, _state, u, h);
        StepCount++;
        Time += h;
        _plant.AppliedTorque.CopyTo(_appliedTorque);
        _plant.Saturated.CopyTo(_saturated);

        Fault = DetectFault();
        return Fault is null;
    }

    /// <summary>
    /// Attaches a payload of <paramref name="massKg"/> at the tool point, replacing any attached one.
    /// The state is unchanged; the parameters jump. SPEC §5.5.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The mass is negative or not finite.</exception>
    public void AttachPayload(double massKg) => _plant.SetPayload(massKg);

    /// <summary>Detaches the payload. The state is unchanged; the parameters jump. SPEC §5.5.</summary>
    public void DetachPayload() => _plant.SetPayload(0.0);

    private Fault? DetectFault()
    {
        // A NaN or infinite state is never valid, so it is always reported, whatever Checks says, and before
        // the joint-limit check, which would otherwise call a NaN angle a JointLimit fault (docs/DECISIONS.md D5).
        foreach (var v in _state)
            if (!double.IsFinite(v))
                return new Fault(FaultReason.NonFinite, Time, -1, double.NaN);

        var model = _plant.Model;
        if ((Checks & FaultChecks.JointLimit) != 0)
        {
            for (var i = 0; i < _n; i++)
                if (!model.Joints[i].IsWithinLimits(_state[i]))
                    return new Fault(FaultReason.JointLimit, Time, i, _state[i]);
        }

        if ((Checks & FaultChecks.Floor) != 0)
        {
            var tool = ForwardKinematics.ComputeFrames(model, Q, _frames);
            if (tool.Z < 0.0)
                return new Fault(FaultReason.Floor, Time, -1, tool.Z);
        }

        return null;
    }
}
