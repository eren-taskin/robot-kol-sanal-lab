using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Tests;

public class ClosedLoopSessionTests
{
    private static LabSession ClosedSession(ControllerKind controller, TaskKind task)
    {
        LabSession s = new LabSession();
        s.ClosedLoop = true;
        s.Closed.Controller = controller;
        s.Closed.Task = task;
        s.DurationS = 4.0;
        return s;
    }

    private static double RunAndMaxError(LabSession s)
    {
        Assert.Empty(s.Reset());
        double worst = 0.0;
        while (s.CanContinue)
        {
            foreach (SimulationSample x in s.Advance(0.25))
                for (int i = 0; i < 4; i++)
                    worst = Math.Max(worst, Math.Abs(x.QRef[i] - x.Q[i]));
        }
        Assert.False(s.IsStopped, s.StatusMessage);
        return worst;
    }

    [Theory]
    [InlineData(ControllerKind.ComputedTorque, TaskKind.JointQuintic, 1e-4)]
    [InlineData(ControllerKind.ComputedTorque, TaskKind.Cartesian, 1e-3)]
    [InlineData(ControllerKind.ComputedTorque, TaskKind.Circle, 1e-3)]
    [InlineData(ControllerKind.SuperTwisting, TaskKind.Circle, 1e-3)]
    [InlineData(ControllerKind.FeedForwardSuperTwisting, TaskKind.Circle, 1e-3)]
    [InlineData(ControllerKind.Pid, TaskKind.JointQuintic, 0.1)]
    public void DefaultTasksRunWithoutFaults(ControllerKind controller, TaskKind task, double tolerance)
    {
        double err = RunAndMaxError(ClosedSession(controller, task));
        Assert.True(err < tolerance, controller + " / " + task + ": " + err);
    }

    [Fact]
    public void JointStepSettlesAndHistoryStartsWithReference()
    {
        LabSession s = ClosedSession(ControllerKind.ComputedTorque, TaskKind.JointStep);
        s.Closed.StartTimeS = 0.5;
        Assert.Empty(s.Reset());
        Assert.NotNull(s.History[0].QRef);
        while (s.CanContinue)
            s.Advance(0.25);
        Assert.False(s.IsStopped, s.StatusMessage);
        SimulationSample last = s.History[s.History.Count - 1];
        for (int i = 0; i < 4; i++)
            Assert.True(Math.Abs(last.QRef[i] - last.Q[i]) < 1e-3, "joint " + i);
    }

    [Fact]
    public void DormandPrinceClosedLoopSamplesEveryTs()
    {
        LabSession s = ClosedSession(ControllerKind.ComputedTorque, TaskKind.JointQuintic);
        s.Solver = SolverKind.DormandPrince;
        Assert.Empty(s.Reset());
        List<SimulationSample> fresh = s.Advance(0.1);
        Assert.Equal(0.002, fresh[1].TimeS - fresh[0].TimeS, 12);
        Assert.True(RunAndMaxError(s) < 1e-3);
    }

    [Fact]
    public void SamplePeriodMustBeAMultipleOfTheSolverStep()
    {
        LabSession s = ClosedSession(ControllerKind.ComputedTorque, TaskKind.JointQuintic);
        s.Closed.SamplePeriodS = 0.0015;
        s.StepSizeS = 0.001;
        Assert.Contains(s.Validate(), e => e.Contains("tam katı"));
        s.Closed.SamplePeriodS = 0.003;
        Assert.Empty(s.Validate());
    }

    [Fact]
    public void MotorActuatorNeedsASmallSolverStep()
    {
        LabSession s = ClosedSession(ControllerKind.ComputedTorque, TaskKind.JointQuintic);
        s.Closed.Actuator = ActuatorKind.Motor;
        Assert.Contains(s.Validate(), e => e.Contains("0,2 ms"));
        s.StepSizeS = 0.0001;
        double err = RunAndMaxError(s);
        Assert.True(err < 1e-2, "error " + err);   // stretched start pose: motor torque and back-EMF matter
    }

    [Fact]
    public void UnreachableTargetsAndBadSettingsAreReportedNotThrown()
    {
        LabSession s = ClosedSession(ControllerKind.ComputedTorque, TaskKind.Cartesian);
        s.Closed.CartesianTarget = new Point3(1.0, 0.0, 0.2);
        Assert.Contains(s.Validate(), e => e.Contains("Kartezyen"));
        s.Closed.Task = TaskKind.JointQuintic;
        s.Closed.TargetQRad = new double[] { 0.0, 2.0, 0.0, 0.0 };
        Assert.Contains(s.Validate(), e => e.Contains("Eklem 2 hedef"));
        s.Closed.TargetQRad = new double[] { 0.5, -0.2, 0.2, 0.8 };
        s.Closed.EstimatedPayloadKg = -1.0;
        s.Closed.ComputedTorque.Kp = new double[] { 1.0, double.NaN, 1.0, 1.0 };
        List<string> errors = s.Validate();
        Assert.Contains(errors, e => e.Contains("Tahmini yük"));
        Assert.Contains(errors, e => e.Contains("kazanç"));
    }

    [Fact]
    public void ModelMismatchComesOnlyFromTheEstimatedPayload()
    {
        LabSession exact = ClosedSession(ControllerKind.ComputedTorque, TaskKind.JointQuintic);
        exact.Payload.MassKg = 0.2;
        exact.Closed.EstimatedPayloadKg = 0.2;
        LabSession wrong = ClosedSession(ControllerKind.ComputedTorque, TaskKind.JointQuintic);
        wrong.Payload.MassKg = 0.2;
        Assert.True(RunAndMaxError(exact) < 1e-4);
        Assert.True(RunAndMaxError(wrong) > 1e-2);
    }
}
