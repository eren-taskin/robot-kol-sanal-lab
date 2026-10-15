using RobotArm.Core.Control;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;

namespace RobotArm.Tests;

/// <summary>The three controllers of SPEC §7.</summary>
public enum ControllerKind
{
    Pid,
    PdGravity,
    ComputedTorque,
}

/// <summary>What one closed-loop run produced (SPEC §7 criterion, §9 metrics in their simplest form).</summary>
/// <param name="SettlingTime">
/// Time after which every joint stays within <see cref="ControlTestHelpers.SettlingBand"/> of the set point, s,
/// measured from the start; NaN if the run faulted.
/// </param>
/// <param name="Settled">True if the error was inside the band at the end of the run.</param>
/// <param name="Clamped">True if the controller clamped its output in any sample.</param>
/// <param name="PlantSaturated">True if the plant saturated in any step.</param>
/// <param name="FinalError">Largest |q_d − q| at the end, rad.</param>
/// <param name="MaxCommand">Largest |u| commanded, N·m.</param>
/// <param name="Fault">The fault that stopped the run, or null.</param>
public sealed record StepResult(
    double SettlingTime, bool Settled, bool Clamped, bool PlantSaturated, double FinalError, double MaxCommand, Fault? Fault);

/// <summary>Shared set-up for the controller tests of SPEC §6.3, §7 and V16–V18 of SPEC §11.</summary>
internal static class ControlTestHelpers
{
    /// <summary>Settling band of SPEC §7: 2 % of the 0.5 rad step, rad.</summary>
    public const double SettlingBand = 0.01;

    /// <summary>
    /// Pose A of SPEC §7's step test: arm raised and leaning back. B = A + 0.5 rad on every pitch joint.
    /// The tool stays at least 0.14 m above the floor at both ends.
    /// </summary>
    public static readonly double[] PoseA = [0.0, -0.6, -0.3, -0.3];

    /// <summary>Pose B = A + 0.5 rad on joints 2, 3 and 4 (SPEC §7).</summary>
    public static readonly double[] PoseB = [0.0, -0.1, 0.2, 0.2];

    /// <summary>A third pose for the stability checks, leaning back with the base turned.</summary>
    public static readonly double[] PoseC = [0.5, -1.0, 0.3, 0.7];

    public static readonly double[][] StabilityPoses = [PoseA, PoseB, PoseC];

    public static readonly double[] Payloads = [0.0, 0.5];

    public static IController Create(ControllerKind kind, ControllerBelief belief) => kind switch
    {
        ControllerKind.Pid => new Pid(belief, Pid.DefaultGains),
        ControllerKind.PdGravity => new PdGravity(belief, PdGravity.DefaultGains),
        ControllerKind.ComputedTorque => new ComputedTorque(belief, ComputedTorque.DefaultGains),
        _ => throw new ArgumentOutOfRangeException(nameof(kind)),
    };

    /// <summary>
    /// A control loop on the true OpenMANIPULATOR-X with <paramref name="payloadKg"/> on <paramref name="planet"/>,
    /// RK4, default h and Ts, all fault checks, starting at rest at <paramref name="start"/>.
    /// </summary>
    public static ControlLoop Loop(
        IController controller, ISensor sensor, IReferenceSource reference, Planet planet, double payloadKg,
        ReadOnlySpan<double> start, double step = Simulation.DefaultStep, double samplePeriod = ControlLoop.DefaultSamplePeriod)
    {
        var plant = new ArmPlant(ArmModel.OpenManipulatorX(), planet);
        plant.SetPayload(payloadKg);
        var x0 = new double[plant.StateCount];
        start.CopyTo(x0);
        var sim = new Simulation(plant, new Rk4Solver(plant.StateCount), x0, FaultChecks.All);
        return new ControlLoop(sim, controller, sensor, reference, step, samplePeriod);
    }

    /// <summary>
    /// The step test of SPEC §7 with the default gains and an exact belief, perfect sensor: from rest at
    /// <paramref name="start"/> to the set point <paramref name="target"/>, run for <paramref name="duration"/>.
    /// </summary>
    public static StepResult StepResponse(
        ControllerKind kind, Planet planet, double payloadKg, double[] start, double[] target,
        double duration = 2.0, ISensor? sensor = null)
    {
        var controller = Create(kind, ControllerBelief.OpenManipulatorX(planet, payloadKg));
        var loop = Loop(controller, sensor ?? new PerfectSensor(4), new ConstantReference(target), planet, payloadKg, start);
        return Run(loop, target, duration);
    }

    /// <summary>Runs <paramref name="loop"/> to <paramref name="duration"/> against the set point and summarises it.</summary>
    public static StepResult Run(ControlLoop loop, double[] target, double duration)
    {
        var sim = loop.Simulation;
        var lastOutside = 0.0;
        var clamped = false;
        var saturated = false;
        var maxCommand = 0.0;
        while (loop.Time < duration - 1e-9)
        {
            var running = loop.AdvanceSample();
            for (var i = 0; i < 4; i++)
            {
                clamped |= loop.Controller.Clamped[i];
                saturated |= loop.PlantSaturated[i];
                maxCommand = Math.Max(maxCommand, Math.Abs(loop.Command[i]));
            }
            if (!running)
                return new StepResult(double.NaN, false, clamped, saturated, double.NaN, maxCommand, sim.Fault);
            if (MaxError(target, sim.Q) > SettlingBand)
                lastOutside = sim.Time;
        }
        var final = MaxError(target, sim.Q);
        return new StepResult(lastOutside, final <= SettlingBand, clamped, saturated, final, maxCommand, null);
    }

    public static double MaxError(ReadOnlySpan<double> target, ReadOnlySpan<double> q)
    {
        var e = 0.0;
        for (var i = 0; i < target.Length; i++)
            e = Math.Max(e, Math.Abs(target[i] - q[i]));
        return e;
    }

    /// <summary>The effective gain K of <paramref name="controller"/> at rest at <paramref name="q0"/> for Ts (n × 2n).</summary>
    public static double[] Gain(IController controller, double[] q0, double samplePeriod = ControlLoop.DefaultSamplePeriod)
    {
        var k = new double[2 * 4 * 4];
        ((ILinearisableController)controller).LinearFeedbackGain(q0, samplePeriod, k);
        return k;
    }
}

/// <summary>
/// Joint-space quintic from <c>start</c> to <c>end</c> over <c>duration</c>, zero speed and acceleration at both
/// ends, then constant (SPEC §8.1). Test-only stand-in until T08 adds the trajectories to Core.
/// </summary>
internal sealed class QuinticReference(double[] start, double[] end, double duration) : IReferenceSource
{
    public int DegreesOfFreedom => start.Length;

    public void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration)
    {
        var tau = Math.Clamp(t / duration, 0.0, 1.0);
        var s = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);
        var ds = 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau) / duration;
        var dds = 60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau) / (duration * duration);
        for (var i = 0; i < start.Length; i++)
        {
            var delta = end[i] - start[i];
            position[i] = start[i] + s * delta;
            velocity[i] = ds * delta;
            acceleration[i] = dds * delta;
        }
    }
}
