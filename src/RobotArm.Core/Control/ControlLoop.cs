using System.Globalization;
using RobotArm.Core.Simulation;
using Sim = RobotArm.Core.Simulation.Simulation;

namespace RobotArm.Core.Control;

/// <summary>
/// Software in the loop: the plant runs with step h, the sensor and controller every Ts = k·h, and the command u
/// is held between controller samples (zero-order hold). SPEC §6.3.
/// </summary>
/// <remarks>
/// <para>
/// One sample (<see cref="AdvanceSample"/>) at t_k = t0 + k·Ts: the sensor measures the true state, the reference
/// is evaluated, the controller computes u, and the simulation advances k steps of h with u held. The sample time
/// is computed from the sample count, so it does not drift.
/// </para>
/// <para>
/// The controller sees only the sensor's output and the reference (SPEC §5.6). The loop records, for logging,
/// whether the plant saturated in any step of the last sample (SPEC §5.4) and which joints the controller clamped
/// (<see cref="IController.Clamped"/>). <see cref="AdvanceSample"/> does not allocate (PROJECT.md §3).
/// </para>
/// </remarks>
public sealed class ControlLoop
{
    /// <summary>Default controller sample period, s. SPEC §6.3: 2 ms.</summary>
    public const double DefaultSamplePeriod = 2e-3;

    private readonly int _n;
    private readonly double _startTime;
    private readonly double[] _qMeasured;
    private readonly double[] _qdMeasured;
    private readonly double[] _qRef;
    private readonly double[] _qdRef;
    private readonly double[] _qddRef;
    private readonly double[] _command;
    private readonly bool[] _plantSaturated;

    /// <summary>Creates the loop. SPEC §6.3.</summary>
    /// <param name="simulation">The true plant and its solver (SPEC §6).</param>
    /// <param name="controller">The controller (SPEC §7).</param>
    /// <param name="sensor">The sensor between plant and controller (SPEC §6.3).</param>
    /// <param name="reference">The reference (SPEC §8.1).</param>
    /// <param name="step">Plant step h, s, within the range of SPEC §6.2.</param>
    /// <param name="samplePeriod">Controller period Ts, s: an integer multiple of h (to 1e-9 relative).</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentException">A part is sized for another number of joints.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// h is outside 0.1 ms to 5 ms, or Ts is not a finite positive integer multiple of h.
    /// </exception>
    public ControlLoop(
        Sim simulation, IController controller, ISensor sensor, IReferenceSource reference,
        double step = Sim.DefaultStep, double samplePeriod = DefaultSamplePeriod)
    {
        ArgumentNullException.ThrowIfNull(simulation);
        ArgumentNullException.ThrowIfNull(controller);
        ArgumentNullException.ThrowIfNull(sensor);
        ArgumentNullException.ThrowIfNull(reference);

        _n = simulation.Plant.DegreesOfFreedom;
        RequireSize(controller.DegreesOfFreedom, nameof(controller));
        RequireSize(sensor.DegreesOfFreedom, nameof(sensor));
        RequireSize(reference.DegreesOfFreedom, nameof(reference));

        if (!(step >= Sim.MinStep && step <= Sim.MaxStep))
            throw new ArgumentOutOfRangeException(nameof(step), step, "Step must be between 0.1 ms and 5 ms (SPEC §6.2).");
        StepsPerSample = StepsPerSamplePeriod(step, samplePeriod);

        Simulation = simulation;
        Controller = controller;
        Sensor = sensor;
        Reference = reference;
        Step = step;
        SamplePeriod = samplePeriod;
        _startTime = simulation.Time;
        _qMeasured = new double[_n];
        _qdMeasured = new double[_n];
        _qRef = new double[_n];
        _qdRef = new double[_n];
        _qddRef = new double[_n];
        _command = new double[_n];
        _plantSaturated = new bool[_n];
    }

    /// <summary>The simulation of the true plant.</summary>
    public Sim Simulation { get; }

    /// <summary>The controller.</summary>
    public IController Controller { get; }

    /// <summary>The sensor.</summary>
    public ISensor Sensor { get; }

    /// <summary>The reference.</summary>
    public IReferenceSource Reference { get; }

    /// <summary>Plant step h, s.</summary>
    public double Step { get; }

    /// <summary>Controller sample period Ts, s.</summary>
    public double SamplePeriod { get; }

    /// <summary>Ts / h, the number of plant steps per controller sample.</summary>
    public int StepsPerSample { get; }

    /// <summary>Number of completed controller samples.</summary>
    public long SampleCount { get; private set; }

    /// <summary>Time of the next controller sample, s: t0 + <see cref="SampleCount"/>·Ts.</summary>
    public double Time => _startTime + SampleCount * SamplePeriod;

    /// <summary>Time of the last controller sample, s; the time at which the values below were taken.</summary>
    public double LastSampleTime { get; private set; }

    /// <summary>Angles the controller saw at the last sample, rad.</summary>
    public ReadOnlySpan<double> MeasuredQ => _qMeasured;

    /// <summary>Speeds the controller saw at the last sample, rad/s.</summary>
    public ReadOnlySpan<double> MeasuredQDot => _qdMeasured;

    /// <summary>q_d at the last sample, rad.</summary>
    public ReadOnlySpan<double> ReferenceQ => _qRef;

    /// <summary>q̇_d at the last sample, rad/s.</summary>
    public ReadOnlySpan<double> ReferenceQDot => _qdRef;

    /// <summary>The command u computed at the last sample and held since, N·m.</summary>
    public ReadOnlySpan<double> Command => _command;

    /// <summary>For each joint, whether the plant saturated in any step of the last sample (SPEC §5.4).</summary>
    public ReadOnlySpan<bool> PlantSaturated => _plantSaturated;

    /// <summary>
    /// The number of plant steps per controller sample, Ts/h, after checking that Ts is a positive integer multiple
    /// of h (to 1e-9 relative). SPEC §6.3.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">h or Ts is not positive and finite, or Ts is not a multiple of h.</exception>
    public static int StepsPerSamplePeriod(double step, double samplePeriod)
    {
        if (!double.IsFinite(step) || step <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(step), step, "Step must be a finite number greater than 0 s (SPEC §6.2).");
        if (!double.IsFinite(samplePeriod) || samplePeriod <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(samplePeriod), samplePeriod,
                "Sample period must be a finite number greater than 0 s (SPEC §6.3).");
        var ratio = Math.Round(samplePeriod / step);
        if (ratio < 1.0 || ratio > int.MaxValue || Math.Abs(ratio * step - samplePeriod) > 1e-9 * samplePeriod)
            throw new ArgumentOutOfRangeException(nameof(samplePeriod), samplePeriod, string.Create(CultureInfo.InvariantCulture,
                $"The sample period Ts = {samplePeriod} s must be an integer multiple of the step h = {step} s (SPEC §6.3)."));
        return (int)ratio;
    }

    /// <summary>
    /// One controller sample: measure, evaluate the reference, compute u, then advance the plant
    /// <see cref="StepsPerSample"/> steps with u held. SPEC §6.3. Does not allocate.
    /// </summary>
    /// <returns>True while the simulation runs; false if a fault stopped it during this sample (SPEC §6.4).</returns>
    /// <exception cref="InvalidOperationException">A fault has already stopped the simulation.</exception>
    public bool AdvanceSample()
    {
        if (Simulation.IsStopped)
            throw new InvalidOperationException("The simulation has stopped with a fault (SPEC §6.4).");

        var t = Time;
        Sensor.Measure(t, Simulation.Q, Simulation.QDot, _qMeasured, _qdMeasured);
        Reference.Evaluate(t, _qRef, _qdRef, _qddRef);
        Controller.Compute(t, _qMeasured, _qdMeasured, new JointReference(_qRef, _qdRef, _qddRef), _command);
        LastSampleTime = t;
        SampleCount++;

        Array.Clear(_plantSaturated);
        for (var s = 0; s < StepsPerSample; s++)
        {
            var running = Simulation.Advance(_command, Step);
            var saturated = Simulation.Saturated;
            for (var i = 0; i < _n; i++)
                _plantSaturated[i] |= saturated[i];
            if (!running)
                return false;
        }
        return true;
    }

    /// <summary>
    /// Runs samples until the next sample time reaches <paramref name="endTime"/> (to half a sample) or a fault
    /// stops the simulation. SPEC §6.3. Does not allocate.
    /// </summary>
    /// <returns>True if the end time was reached without a fault.</returns>
    public bool RunUntil(double endTime)
    {
        while (Time < endTime - 0.5 * SamplePeriod)
            if (!AdvanceSample())
                return false;
        return true;
    }

    private void RequireSize(int size, string name)
    {
        if (size != _n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"The plant has {_n} joints but this part is for {size} (SPEC §6.3)."), name);
    }
}
