using RobotArmLab.Core.Motor;
using RobotArmLab.Core.Solvers;

namespace RobotArmLab.Tests;

public class MotorTests
{
    private readonly DcMotor _m = DcMotor.Xm430W350(353.5);

    [Fact]
    public void StallTorqueAt12VMatchesDatasheet()
    {
        Assert.InRange(_m.Torque(12.0, 0.0, 100.0), 0.98 * 4.1, 1.02 * 4.1);
        Assert.InRange(_m.StallTorqueNm(12.0), 0.98 * 4.1, 1.02 * 4.1);
    }

    [Fact]
    public void NoLoadSpeedAt12VMatchesDatasheet()
    {
        double datasheet = 46.0 * 2.0 * Math.PI / 60.0;   // 46 rpm in rad/s
        Assert.InRange(_m.NoLoadSpeedRadPerS(12.0), 0.95 * datasheet, 1.05 * datasheet);

        // Simulated: a small inertia with no load, driven at 12 V from rest, settles at the no-load speed.
        double J = 1e-3;   // kg·m²
        OdeFunction f = delegate (double t, double[] x) { return new[] { _m.Torque(12.0, x[0], 4.1) / J }; };
        double[] w = { 0.0 };
        Rk4Solver rk4 = new Rk4Solver();
        for (int k = 0; k < 2000; k++)
            w = rk4.Step(f, k * 1e-4, w, 1e-4);
        Assert.InRange(w[0], 0.95 * datasheet, 1.05 * datasheet);
    }

    [Fact]
    public void VoltageAndTorqueAreClamped()
    {
        Assert.Equal(_m.Torque(12.0, 1.0, 4.1), _m.Torque(30.0, 1.0, 4.1), 12);
        Assert.Equal(4.1, _m.Torque(12.0, -3.0, 4.1), 12);       // back-driven: would exceed 4.1 without saturation
        Assert.Equal(-4.1, _m.Torque(-12.0, 3.0, 4.1), 12);
    }
}
