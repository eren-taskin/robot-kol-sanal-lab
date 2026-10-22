using RobotArmLab.Core.Dynamics;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Motor;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;
using RobotArmLab.Core.Tasks;

namespace RobotArmLab.Tests;

public class ClosedLoopPartsTests
{
    [Fact]
    public void InverseDynamicsUndoesForwardDynamics()
    {
        ArmDynamics d = new ArmDynamics(new Payload(0.2).ApplyTo(ArmParameters.Default()), 9.81);
        double[] q = { 0.3, -0.5, 0.4, 0.7 }, qd = { 0.5, -1.0, 0.8, 2.0 }, tau = { 0.1, -0.4, 0.3, 0.05 };
        Check.RelativeClose(tau, d.InverseDynamics(q, qd, d.Acceleration(q, qd, tau)), 1e-10, "τ");
    }

    [Fact]
    public void MotorVoltageForInvertsMotorTorque()
    {
        DcMotor m = DcMotor.Xm430W350(353.5);
        Assert.Equal(1.3, m.Torque(m.VoltageFor(1.3, -0.7), -0.7, 4.1), 12);
        Assert.Equal(12.0, m.VoltageFor(4.1, 0.0), 12);   // stall torque needs the full 12 V
    }

    [Fact]
    public void EncoderQuantisesAndDifferencesAndIsRepeatable()
    {
        JointSensor s = new JointSensor();
        s.Kind = SensorKind.Encoder;
        double r = 2.0 * Math.PI / 4096.0;
        double[] qm = new double[4], qdm = new double[4];
        s.Measure(0.0, new[] { 0.1, 0.2, 0.3, 0.4 }, new double[4], qm, qdm);
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(0.0, Math.IEEERemainder(qm[i], r), 12);
            Assert.True(Math.Abs(qm[i] - 0.1 * (i + 1)) <= 0.5 * r + 1e-15);
            Assert.Equal(0.0, qdm[i]);   // no previous angle on the first sample
        }
        double first = qm[0];
        s.Measure(0.002, new[] { 0.1 + 10 * r, 0.2, 0.3, 0.4 }, new double[4], qm, qdm);
        Assert.Equal((qm[0] - first) / 0.002, qdm[0], 9);

        JointSensor a = new JointSensor(), b = new JointSensor();
        a.Kind = b.Kind = SensorKind.Encoder;
        a.NoiseStdDevRad = b.NoiseStdDevRad = 0.01;
        double[] qa = new double[4], qb = new double[4], v = new double[4];
        a.Measure(0.0, new double[4], new double[4], qa, v);
        b.Measure(0.0, new double[4], new double[4], qb, v);
        Assert.Equal(qa, qb);
        Assert.NotEqual(new double[4], qa);
    }

    [Fact]
    public void QuinticStartsAndEndsAtRestAndMatchesNumericalDerivatives()
    {
        double[] q0 = { 0.0, -0.6, 0.3, 0.6 }, qf = { 0.5, -0.2, 0.2, 0.8 };
        QuinticTask task = new QuinticTask(q0, qf, 0.5, 2.0);
        Assert.Equal(q0, task.At(0.0).Q);
        Assert.Equal(q0, task.At(0.5).Q);
        Check.RelativeClose(qf, task.At(2.5).Q, 1e-15, "qf");
        Assert.Equal(new double[4], task.At(2.5).Qd);
        double t = 1.3, d = 1e-5;
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal((task.At(t + d).Q[i] - task.At(t - d).Q[i]) / (2 * d), task.At(t).Qd[i], 7);
            Assert.Equal((task.At(t + d).Qd[i] - task.At(t - d).Qd[i]) / (2 * d), task.At(t).Qdd[i], 5);
        }
    }

    [Fact]
    public void CartesianTaskEndsAtTheTargetAndRejectsUnreachableOnes()
    {
        ArmParameters p = ArmParameters.Default();
        string error;
        Point3 target = new Point3(0.2, 0.05, 0.15);
        QuinticTask task = CartesianTask.Create(p, new double[4], target, 0.0, true, 0.0, 1.0, out error);
        Assert.NotNull(task);
        Point3 reached = ForwardKinematics.ToolPosition(p, task.At(1.0).Q);
        Assert.Equal(target.X, reached.X, 10);
        Assert.Equal(target.Y, reached.Y, 10);
        Assert.Equal(target.Z, reached.Z, 10);
        Assert.Null(CartesianTask.Create(p, new double[4], new Point3(1.0, 0.0, 0.2), 0.0, true, 0.0, 1.0, out error));
        Assert.Contains("erişim", error);
    }

    [Theory]
    [InlineData(CirclePlane.Vertical)]
    [InlineData(CirclePlane.Horizontal)]
    public void CircleTaskPutsTheToolOnTheCircle(CirclePlane plane)
    {
        ArmParameters p = ArmParameters.Default();
        Point3 c = new Point3(0.2, 0.0, 0.15);
        CircleTask task = new CircleTask(p, new double[4], c, 0.03, plane, 4.0, 0.0, true, 2.0, 2.0);
        Assert.Equal("", task.Validate());
        Point3 start = ForwardKinematics.ToolPosition(p, task.At(2.0).Q);
        Assert.Equal(0.23, start.X, 10);   // the approach ends at φ = 0
        for (double t = 2.0; t < 10.0; t += 0.37)
        {
            Point3 tool = ForwardKinematics.ToolPosition(p, task.At(t).Q);
            double u = tool.X - c.X, w = plane == CirclePlane.Vertical ? tool.Z - c.Z : tool.Y - c.Y;
            Assert.Equal(0.03, Math.Sqrt(u * u + w * w), 10);
        }
        double[] qd = task.At(2.0).Qd;   // the ramp starts the circle at rest (φ ∝ τ³, so only δ² is left)
        for (int i = 0; i < 4; i++)
            Assert.True(Math.Abs(qd[i]) < 1e-8, "qd" + i + " = " + qd[i]);
    }

    [Fact]
    public void CircleOutsideTheWorkspaceIsReported()
    {
        CircleTask task = new CircleTask(ArmParameters.Default(), new double[4], new Point3(0.35, 0.0, 0.1), 0.1,
            CirclePlane.Horizontal, 4.0, 0.0, true, 2.0, 2.0);
        Assert.Contains("φ", task.Validate());
    }
}
