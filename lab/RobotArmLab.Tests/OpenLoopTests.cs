using RobotArmLab.Core.Model;
using RobotArmLab.Core.Simulation;
using RobotArmLab.Core.Solvers;

namespace RobotArmLab.Tests;

public class OpenLoopTests
{
    private static void RunUntil(OpenLoopSimulator sim, double tEnd)
    {
        while (!sim.IsStopped && sim.TimeS < tEnd - 1e-12)
            sim.Step();
    }

    [Fact]
    public void ZeroInputUnderEarthGravityFallsAndStopsWithReason()
    {
        OpenLoopSimulator sim = new OpenLoopSimulator(ArmParameters.Default(), GravityEnvironment.Earth(), new Payload(0.0), new Rk4Solver());
        sim.Reset(ArmState.AtRest(0.0, 0.0, 0.0, 0.0));
        RunUntil(sim, 10.0);
        Assert.True(sim.IsStopped);
        Assert.True(sim.StopReason == StopReason.JointLimit || sim.StopReason == StopReason.Floor, sim.StopReason.ToString());
        Assert.False(string.IsNullOrEmpty(sim.StopMessage));
    }

    [Fact]
    public void ZeroInputInOrbitDoesNotMove()
    {
        OpenLoopSimulator sim = new OpenLoopSimulator(ArmParameters.Default(), GravityEnvironment.Orbit(), new Payload(0.3), new Rk4Solver());
        sim.Reset(ArmState.AtRest(0.2, -0.4, 0.3, 0.5));
        RunUntil(sim, 1.0);
        Assert.False(sim.IsStopped);
        Assert.Equal(new[] { 0.2, -0.4, 0.3, 0.5 }, sim.State.Q);
        Assert.Equal(new double[4], sim.State.Qd);
    }

    [Fact]
    public void StepReturnsOneSamplePerSolverStepWithEnergyAndTool()
    {
        OpenLoopSimulator sim = new OpenLoopSimulator(ArmParameters.Default(), GravityEnvironment.Moon(), new Payload(0.0), new HeunSolver());
        sim.StepSizeS = 0.001;
        sim.FrameTimeS = 0.02;
        sim.Reset(ArmState.AtRest(0.0, -0.6, -0.3, -0.3));
        List<SimulationSample> samples = sim.Step();
        Assert.Equal(20, samples.Count);
        Assert.Equal(0.02, samples[19].TimeS, 12);
        Assert.True(samples[19].ToolZ > 0.0);
        Assert.True(samples[19].KineticEnergyJ > 0.0);   // the arm has started to fall
    }

    [Fact]
    public void TorqueInputIsSaturatedAndStepStartsAtT0()
    {
        OpenLoopSimulator sim = new OpenLoopSimulator(ArmParameters.Default(), GravityEnvironment.Orbit(), new Payload(0.0), new Rk4Solver());
        sim.Inputs[0].Shape = InputShape.Step;
        sim.Inputs[0].Amplitude = 10.0;
        sim.Inputs[0].StepTimeS = 0.1;
        Assert.Equal(0.0, sim.AppliedTorque(0.05, new double[4])[0]);
        Assert.Equal(4.1, sim.AppliedTorque(0.15, new double[4])[0], 12);
    }

    [Fact]
    public void VoltageModeWithDormandPrinceSpinsTheBaseTowardsNoLoadSpeed()
    {
        ArmParameters p = ArmParameters.Default();
        p.ViscousFrictionNmsPerRad = 0.0;
        p.CoulombFrictionNm = 0.0;
        OpenLoopSimulator sim = new OpenLoopSimulator(p, GravityEnvironment.Orbit(), new Payload(0.0), new DormandPrinceSolver());
        sim.Mode = InputMode.Voltage;
        sim.Inputs[0].Amplitude = 3.0;   // 3 V on the base joint, the others at 0 V
        sim.StepSizeS = 0.02;
        sim.Reset(ArmState.AtRest(-3.0, 0.0, 0.0, 0.0));
        RunUntil(sim, 0.2);
        Assert.False(sim.IsStopped, sim.StopMessage);
        double expected = 3.0 / 12.0 * 46.0 * 2.0 * Math.PI / 60.0;
        Assert.InRange(sim.State.Qd[0], 0.95 * expected, 1.05 * expected);
    }
}
