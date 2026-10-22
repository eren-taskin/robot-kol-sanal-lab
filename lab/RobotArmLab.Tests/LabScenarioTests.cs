using RobotArmLab.Core.Model;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Tests;

// The manual checks in docs/WINDOWS_KURULUM.md, run through LabSession exactly as the form drives it.
// The numbers quoted in that document come from these tests.
public class LabScenarioTests
{
    private static double Deg(double d) { return d * Math.PI / 180.0; }

    private static void RunToEnd(LabSession s)
    {
        int guard = 0;
        while (s.CanContinue && guard++ < 100000)
            s.Advance(0.03);
    }

    private static LabSession Fall(GravityEnvironment env, double payloadKg)
    {
        LabSession s = new LabSession();
        s.Environment = env;
        s.Payload.MassKg = payloadKg;
        Assert.Empty(s.Reset());
        RunToEnd(s);
        return s;
    }

    [Fact]
    public void OrbitWithZeroInputDoesNotMove()
    {
        LabSession s = new LabSession();
        s.Environment = GravityEnvironment.Orbit();
        s.DurationS = 2.0;
        s.Reset();
        RunToEnd(s);
        Assert.True(s.IsFinished);
        Assert.Equal(new double[4], s.History[s.History.Count - 1].Q);
    }

    [Fact]
    public void EarthMoonAndPayloadFallTimesToTheFloor()
    {
        LabSession earth = Fall(GravityEnvironment.Earth(), 0.0);
        LabSession moon = Fall(GravityEnvironment.Moon(), 0.0);
        LabSession loaded = Fall(GravityEnvironment.Earth(), 0.5);

        Assert.Equal(StopReason.Floor, earth.StopReason);
        Assert.Equal(StopReason.Floor, moon.StopReason);
        Assert.Equal(StopReason.Floor, loaded.StopReason);
        Assert.InRange(earth.TimeS, 0.16, 0.18);    // 0.169 s
        Assert.InRange(moon.TimeS, 0.50, 0.53);     // 0.515 s; without friction the ratio is exactly √(9.81/1.62) ≈ 2.46
        Assert.InRange(loaded.TimeS, 0.18, 0.20);   // 0.192 s: the tip mass makes the arm a longer pendulum
        Assert.True(loaded.TimeS > earth.TimeS);
    }

    [Fact]
    public void TwelveVoltsOnTheBaseWithoutGravityReachesAbout46Rpm()
    {
        LabSession s = new LabSession();
        s.Environment = GravityEnvironment.Orbit();
        s.Mode = InputMode.Voltage;
        s.Inputs[0].Amplitude = 12.0;
        s.InitialQRad[0] = Deg(-170.0);   // room to turn before the +180° limit
        s.DurationS = 1.0;
        Assert.Empty(s.Reset());
        RunToEnd(s);
        Assert.True(s.IsFinished, s.StatusMessage);
        double rpm = s.History[s.History.Count - 1].Qd[0] * 60.0 / (2.0 * Math.PI);
        Assert.InRange(rpm, 44.5, 46.0);   // 45.2 rpm: slightly below 46 because of joint friction
    }

    [Fact]
    public void TorqueInputAboveTheLimitIsShownSaturated()
    {
        LabSession s = new LabSession();
        s.Environment = GravityEnvironment.Orbit();
        s.Inputs[0].Amplitude = 10.0;
        s.InitialQRad[0] = Deg(-170.0);
        s.Reset();
        s.Advance(0.05);
        Assert.Equal(4.1, s.History[s.History.Count - 1].Tau[0], 12);
    }

    [Fact]
    public void EulerAtFiveMillisecondsDriftsInEnergyAndRk4DoesNot()
    {
        double euler = EnergyDrift(SolverKind.Euler);
        double rk4 = EnergyDrift(SolverKind.Rk4);
        Assert.InRange(euler, 5e-3, 2e-2);   // 1.0e-2 J of 0.758 J before the joint limit at 0.14 s
        Assert.True(rk4 < 1e-6, $"RK4 drift {rk4:E3} J");
    }

    // Friction off (fv = fc = 0 in the PropertyGrid), Earth, pose (0°, −60°, 30°, 30°), h = 5 ms.
    private static double EnergyDrift(SolverKind kind)
    {
        LabSession s = new LabSession();
        s.Solver = kind;
        s.StepSizeS = 0.005;
        s.Parameters.ViscousFrictionNmsPerRad = 0.0;
        s.Parameters.CoulombFrictionNm = 0.0;
        s.InitialQRad = new[] { 0.0, Deg(-60.0), Deg(30.0), Deg(30.0) };
        Assert.Empty(s.Reset());
        RunToEnd(s);
        double e0 = s.History[0].TotalEnergyJ;
        double maxDrift = 0.0;
        foreach (SimulationSample sample in s.History)
            maxDrift = Math.Max(maxDrift, Math.Abs(sample.TotalEnergyJ - e0));
        return maxDrift;
    }
}
