using RobotArmLab.Core.Controllers;
using RobotArmLab.Core.Dynamics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Simulation;
using RobotArmLab.Core.Solvers;
using RobotArmLab.Core.Tasks;

namespace RobotArmLab.Tests;

public class ClosedLoopTests
{
    private static readonly double[] Start = { 0.0, -0.6, 0.3, 0.6 };
    private static readonly double[] Goal = { 0.5, -0.2, 0.2, 0.8 };

    internal static ArmDynamics Model(double payloadKg)
    {
        return new ArmDynamics(new Payload(payloadKg).ApplyTo(ArmParameters.Default()), 9.81);
    }

    internal static IController Make(string kind, double estimatedPayloadKg)
    {
        switch (kind)
        {
            case "PID": return new PidController(PidGains.Default(), 4.1);
            case "CT": return new ComputedTorqueController(Model(estimatedPayloadKg), ComputedTorqueGains.Default());
            case "STSMC": return new SuperTwistingController(Model(estimatedPayloadKg), SuperTwistingGains.Default());
            default: return new FeedForwardStsmcController(Model(estimatedPayloadKg), SuperTwistingGains.Default());
        }
    }

    internal static ClosedLoopSimulator Build(IController c, IReferenceTask task, double truePayloadKg, JointSensor sensor,
        ActuatorKind actuator, double h = 0.0005)
    {
        OpenLoopSimulator plant = new OpenLoopSimulator(ArmParameters.Default(), GravityEnvironment.Earth(), new Payload(truePayloadKg), new Rk4Solver());
        plant.StepSizeS = h;
        ClosedLoopSimulator loop = new ClosedLoopSimulator(plant, c, task, sensor, actuator, 0.002);
        loop.Reset(ArmState.AtRest(Start[0], Start[1], Start[2], Start[3]));
        return loop;
    }

    /// <summary>Runs to tEnd or a fault; returns the largest |q_ref − q| and the largest sample-to-sample change of the torque command.</summary>
    internal static double MaxError(ClosedLoopSimulator loop, double tEnd, out double maxCommandJump)
    {
        double worst = 0.0;
        maxCommandJump = 0.0;
        double[] previous = null;
        while (!loop.Plant.IsStopped && loop.Plant.TimeS < tEnd - 1e-9)
        {
            List<SimulationSample> samples = loop.Step();
            foreach (SimulationSample s in samples)
                for (int i = 0; i < 4; i++)
                    worst = Math.Max(worst, Math.Abs(s.QRef[i] - s.Q[i]));
            double[] command = samples[samples.Count - 1].TauCommand;
            for (int i = 0; previous != null && i < 4; i++)
                maxCommandJump = Math.Max(maxCommandJump, Math.Abs(command[i] - previous[i]));
            previous = command;
        }
        return worst;
    }

    internal static double MaxError(ClosedLoopSimulator loop, double tEnd)
    {
        double jump;
        double worst = MaxError(loop, tEnd, out jump);
        Assert.False(loop.Plant.IsStopped, loop.Plant.StopMessage);
        return worst;
    }

    [Theory]
    [InlineData("PID", 0.1)]
    [InlineData("CT", 1e-4)]
    [InlineData("STSMC", 5e-4)]
    [InlineData("FF", 5e-4)]
    public void QuinticIsTrackedWithExactModel(string kind, double tolerance)
    {
        QuinticTask task = new QuinticTask(Start, Goal, 0.0, 2.0);
        double err = MaxError(Build(Make(kind, 0.0), task, 0.0, new JointSensor(), ActuatorKind.IdealTorque), 3.0);
        Assert.True(err < tolerance, kind + " max error " + err);
    }

    [Fact]
    public void UnannouncedPayloadReproducesOldCoreD13()
    {
        // Plant 0.3 kg, model 0 kg: CT stays bounded with a large error, STSMC with the standard gains drifts
        // into the wrist limit (old core D13: CT peak 0.56 rad, STSMC JointLimit j4).
        QuinticTask task = new QuinticTask(Start, Goal, 0.0, 2.0);
        double ct = MaxError(Build(Make("CT", 0.0), task, 0.3, new JointSensor(), ActuatorKind.IdealTorque), 3.0);
        Assert.InRange(ct, 0.1, 0.6);
        ClosedLoopSimulator st = Build(Make("STSMC", 0.0), task, 0.3, new JointSensor(), ActuatorKind.IdealTorque);
        double jump;
        MaxError(st, 3.0, out jump);
        Assert.Equal(StopReason.JointLimit, st.Plant.StopReason);
    }

    [Fact]
    public void OverestimatedPayloadIsHandledBySlidingModeButNotByComputedTorque()
    {
        // Plant 0 kg, model 0.3 kg: CT's input gain is ~13× too high on the wrist and it chatters into a limit;
        // the super-twisting laws stay within a few hundredths of a radian (old core D13: STSMC peak 0.025 rad).
        QuinticTask task = new QuinticTask(Start, Goal, 0.0, 2.0);
        ClosedLoopSimulator ct = Build(Make("CT", 0.3), task, 0.0, new JointSensor(), ActuatorKind.IdealTorque);
        double jump;
        MaxError(ct, 3.0, out jump);
        Assert.True(ct.Plant.IsStopped);
        Assert.True(MaxError(Build(Make("STSMC", 0.3), task, 0.0, new JointSensor(), ActuatorKind.IdealTorque), 3.0) < 0.05);
        Assert.True(MaxError(Build(Make("FF", 0.3), task, 0.0, new JointSensor(), ActuatorKind.IdealTorque), 3.0) < 0.1);
    }

    [Fact]
    public void FeedForwardStsmcCommandIsSmootherThanComputedTorqueUnderEncoderNoise()
    {
        QuinticTask task = new QuinticTask(Start, Goal, 0.0, 2.0);
        double ctJump, ffJump;
        JointSensor a = new JointSensor(), b = new JointSensor();
        a.Kind = b.Kind = SensorKind.Encoder;
        a.NoiseStdDevRad = b.NoiseStdDevRad = 1e-3;
        MaxError(Build(Make("CT", 0.0), task, 0.0, a, ActuatorKind.IdealTorque), 3.0, out ctJump);
        double ff = MaxError(Build(Make("FF", 0.0), task, 0.0, b, ActuatorKind.IdealTorque), 3.0, out ffJump);
        Assert.True(ffJump < 0.5 * ctJump, "FF jump " + ffJump + ", CT jump " + ctJump);
        Assert.True(ff < 0.05, "FF error " + ff);
    }

    [Theory]
    [InlineData("PID", 0.1)]
    [InlineData("CT", 2e-3)]
    [InlineData("STSMC", 2e-3)]
    [InlineData("FF", 2e-3)]
    public void MotorActuatorTracksWithSmallSolverStep(string kind, double tolerance)
    {
        QuinticTask task = new QuinticTask(Start, Goal, 0.0, 2.0);
        double err = MaxError(Build(Make(kind, 0.0), task, 0.0, new JointSensor(), ActuatorKind.Motor, 0.0001), 3.0);
        Assert.True(err < tolerance, kind + " max error " + err);
    }

    [Fact]
    public void TorqueCommandIsHeldForOneSamplePeriod()
    {
        ClosedLoopSimulator loop = Build(Make("CT", 0.0), new QuinticTask(Start, Goal, 0.0, 2.0), 0.0, new JointSensor(), ActuatorKind.IdealTorque);
        List<SimulationSample> samples = loop.Step();
        Assert.Equal(4, samples.Count);   // Ts / h = 2 ms / 0.5 ms
        Assert.Equal(0.002, samples[3].TimeS, 12);
        for (int n = 0; n < 4; n++)
            Assert.Equal(samples[0].Tau, samples[n].Tau);   // ideal torque: the held command, nothing else
    }
}
