using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Tests;

/// <summary>
/// The six closed-loop manual checks of docs/WINDOWS_KURULUM.md, run headless with the window's default settings
/// (q0 = 0, 10 s, RK4, h = 0.5 ms, Ts = 2 ms, quintic to [0.5, −0.2, 0.2, 0.8] rad in 2 s). The numbers there come from here.
/// </summary>
public class ClosedLoopScenarioTests
{
    private static readonly List<ControllerKind> All = new List<ControllerKind>
    {
        ControllerKind.Pid, ControllerKind.ComputedTorque, ControllerKind.SuperTwisting, ControllerKind.FeedForwardSuperTwisting,
    };

    private static LabSession Default()
    {
        LabSession s = new LabSession();
        s.ClosedLoop = true;
        return s;
    }

    [Fact]
    public void C1_ExactModelAllControllersTrack()
    {
        List<ComparisonResult> r = Default().CompareControllers(All);
        foreach (ComparisonResult x in r)
            Assert.False(x.Faulted, x.Name);
        Assert.InRange(r[0].Metrics.MaxErrorRad, 0.055, 0.065);   // PID ≈ 0.060 rad
        Assert.True(r[1].Metrics.MaxErrorRad < 2e-5);              // CT ≈ 1.7e-5 rad
        Assert.True(r[2].Metrics.MaxErrorRad < 2e-4);              // STSMC ≈ 1.4e-4 rad
        Assert.True(r[3].Metrics.MaxErrorRad < 2e-4);              // FF-STSMC ≈ 1.4e-4 rad
    }

    [Fact]
    public void C2_UnderestimatedPayload()
    {
        LabSession s = Default();
        s.Payload.MassKg = 0.3;
        List<ComparisonResult> r = s.CompareControllers(All);
        Assert.InRange(r[1].Metrics.MaxErrorRad, 0.55, 0.65);      // CT ≈ 0.61 rad, no fault
        Assert.False(r[1].Faulted);
        Assert.False(r[2].Faulted);                                // STSMC from q0 = 0: peak ≈ 1.32 rad, no fault
        Assert.True(r[2].Metrics.MaxErrorRad > 1.0);
        Assert.True(r[3].Faulted);                                 // FF-STSMC: joint 3 limit at 1.264 s
        Assert.Contains("Eklem 3", r[3].Message);
    }

    [Fact]
    public void C3_OverestimatedPayload()
    {
        LabSession s = Default();
        s.Closed.EstimatedPayloadKg = 0.3;
        List<ComparisonResult> r = s.CompareControllers(All);
        Assert.True(r[1].Faulted);                                 // CT chatters at ±4.1 N·m and hits the floor at 0.423 s
        Assert.Contains("zemin", r[1].Message);
        Assert.False(r[2].Faulted);
        Assert.True(r[2].Metrics.MaxErrorRad < 0.05);              // STSMC ≈ 0.041 rad
        Assert.True(r[2].Metrics.MaxToolErrorMm < 5.0);            // ≈ 4.6 mm
    }

    [Fact]
    public void C4_EncoderNoise()
    {
        LabSession s = Default();
        s.Closed.Sensor = SensorKind.Encoder;
        s.Closed.NoiseStdDevRad = 1e-3;
        List<ComparisonResult> r = s.CompareControllers(All);
        Assert.True(r[1].Metrics.ChatteringNmPerS > 4000.0);       // CT ≈ 4800 N·m/s
        Assert.True(r[1].Metrics.SaturationTimeS > 3.0);           // ≈ 3.2 s at ±τmax
        Assert.True(r[3].Metrics.ChatteringNmPerS < 600.0);        // FF-STSMC ≈ 510 N·m/s
        Assert.Equal(0.0, r[3].Metrics.SaturationTimeS);
    }

    [Fact]
    public void C5_MotorActuator()
    {
        LabSession s = Default();
        s.Closed.Actuator = ActuatorKind.Motor;
        Assert.Contains(s.Validate(), e => e.Contains("0,2 ms"));
        s.StepSizeS = 0.0001;
        List<ComparisonResult> r = s.CompareControllers(All);
        Assert.False(r[1].Faulted);
        Assert.InRange(r[1].Metrics.MaxErrorRad, 0.005, 0.007);    // CT ≈ 6.1e-3 rad
    }

    [Fact]
    public void C6_CircleAndPickAndPlace()
    {
        LabSession s = Default();
        s.Closed.Task = TaskKind.Circle;
        List<ComparisonResult> r = s.CompareControllers(All);
        Assert.True(r[3].Metrics.MaxToolErrorMm < 0.02);           // FF-STSMC ≈ 0.014 mm
        s.Payload.MassKg = 0.3;
        s.Closed.PickAndPlace = true;
        s.Closed.PickTimeS = 4.0;
        r = s.CompareControllers(All);
        Assert.False(r[1].Faulted);                                // CT: wrist sags, error ≈ 0.70 rad
        Assert.InRange(r[1].Metrics.MaxErrorRad, 0.6, 0.8);
        Assert.True(r[2].Faulted);                                 // STSMC: joint 4 limit at 5.241 s
        Assert.Contains("Eklem 4", r[2].Message);
    }
}
