using System.Globalization;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Tests;

public class MetricsAndComparisonTests
{
    // Hand-made run, t = 0..1 s every 10 ms: joint 1 sits at 0 until 0.5 s, overshoots to 1.1 until 0.7 s,
    // then sits on the reference 1.0. Torque 1 N·m on joint 1; command saturated (4.1) before 0.2 s, then 0.
    private static List<SimulationSample> HandMadeRun(ArmParameters p)
    {
        List<SimulationSample> samples = new List<SimulationSample>();
        for (int n = 0; n <= 100; n++)
        {
            double t = n * 0.01;
            SimulationSample s = new SimulationSample();
            s.TimeS = t;
            s.QRef = new double[] { 1.0, 0.0, 0.0, 0.0 };
            s.Q = new double[] { t < 0.5 - 1e-9 ? 0.0 : (t < 0.7 - 1e-9 ? 1.1 : 1.0), 0.0, 0.0, 0.0 };
            s.Qd = new double[4];
            s.Tau = new double[] { 1.0, 0.0, 0.0, 0.0 };
            s.TauCommand = new double[] { t < 0.2 - 1e-9 ? 4.1 : 0.0, 0.0, 0.0, 0.0 };
            Point3 tool = ForwardKinematics.ToolPosition(p, s.Q);
            s.ToolX = tool.X;
            s.ToolY = tool.Y;
            s.ToolZ = tool.Z;
            samples.Add(s);
        }
        return samples;
    }

    [Fact]
    public void MetricsOfAHandMadeSignalAreTheKnownValues()
    {
        ArmParameters p = ArmParameters.Default();
        ControlMetrics m = ControlMetrics.Compute(HandMadeRun(p), p, 4.1, 0.005);
        Assert.Equal(Math.Sqrt((50 * 1.0 + 20 * 0.01) / (4.0 * 101)), m.RmsErrorRad, 12);   // 50 samples e = 1, 20 samples e = −0.1
        Assert.Equal(1.0, m.MaxErrorRad, 12);
        Assert.Equal(0.0, m.FinalErrorRad, 12);
        Assert.Equal(0.69, m.SettlingTimeS, 9);       // last sample outside the band
        Assert.Equal(10.0, m.OvershootPercent, 9);    // (1.1 − 1.0) / (1.0 − 0)
        Assert.Equal(1.0, m.MaxTorqueNm, 12);
        Assert.Equal(0.19, m.SaturationTimeS, 9);     // samples 0.01 … 0.19, 10 ms each
        Assert.Equal(1.0, m.TorqueSquaredIntegral, 9);
        Assert.Equal(4.1, m.ChatteringNmPerS, 9);     // one jump of 4.1 N·m in 1 s
        Assert.Equal(0.0, m.FinalToolErrorMm, 9);
        Assert.True(m.MaxToolErrorMm > 100.0);        // 1 rad of base rotation moves the tool by decimetres
    }

    [Fact]
    public void SettlingIsNaNWhenTheRunEndsOutsideTheBand()
    {
        ArmParameters p = ArmParameters.Default();
        List<SimulationSample> run = HandMadeRun(p);
        run[run.Count - 1].Q[0] = 0.9;
        Assert.True(double.IsNaN(ControlMetrics.Compute(run, p, 4.1, 0.005).SettlingTimeS));
    }

    private static LabSession CompareSession()
    {
        LabSession s = new LabSession();
        s.InitialQRad = new double[] { 0.0, -0.6, 0.3, 0.6 };
        s.DurationS = 1.0;
        s.ClosedLoop = true;
        s.Closed.MotionDurationS = 0.8;
        s.Closed.Sensor = SensorKind.Encoder;
        s.Closed.NoiseStdDevRad = 1e-3;
        return s;
    }

    private static readonly List<ControllerKind> AllKinds = new List<ControllerKind>
    {
        ControllerKind.Pid, ControllerKind.ComputedTorque, ControllerKind.SuperTwisting, ControllerKind.FeedForwardSuperTwisting,
    };

    [Fact]
    public void CompareControllersIsDeterministicAndLeavesTheSessionAlone()
    {
        LabSession s = CompareSession();
        List<ComparisonResult> a = s.CompareControllers(AllKinds);
        List<ComparisonResult> b = s.CompareControllers(AllKinds);
        Assert.Equal(4, a.Count);
        for (int k = 0; k < 4; k++)
        {
            Assert.Equal(AllKinds[k], a[k].Kind);
            Assert.False(a[k].Faulted, a[k].Name + ": " + a[k].Message);
            Assert.Equal(ComparisonCsv.Values(a[k].Metrics), ComparisonCsv.Values(b[k].Metrics));
            Assert.Equal(a[k].ToolErrorMm, b[k].ToolErrorMm);
            Assert.Equal(1.0, a[k].TimeS[a[k].TimeS.Count - 1], 9);
        }
        Assert.Equal(ControllerKind.ComputedTorque, s.Closed.Controller);   // template untouched
        Assert.Single(s.History);
        Assert.True(a[1].Metrics.ChatteringNmPerS > 2.0 * a[3].Metrics.ChatteringNmPerS);   // CT vs FF-STSMC under noise
    }

    [Fact]
    public void PickAndPlaceSwitchesThePayloadAtTheSetTime()
    {
        LabSession s = new LabSession();
        s.InitialQRad = new double[] { 0.0, -0.6, 0.3, 0.6 };
        s.DurationS = 2.0;
        s.Payload.MassKg = 0.3;
        s.ClosedLoop = true;
        s.Closed.PickAndPlace = true;
        s.Closed.PickTimeS = 1.0;
        Assert.Empty(s.Reset());
        while (s.CanContinue)
            s.Advance(0.25);
        Assert.False(s.IsStopped, s.StatusMessage);
        double before = 0.0, after = 0.0;
        foreach (SimulationSample x in s.History)
        {
            Assert.Equal(x.TimeS <= 1.0 + 1e-9 ? 0.0 : 0.3, x.PayloadKg);
            double e = Math.Abs(x.QRef[3] - x.Q[3]);
            if (x.TimeS <= 1.0) before = Math.Max(before, e); else after = Math.Max(after, e);
        }
        Assert.True(before < 1e-4, "before " + before);   // exact model until the pick
        Assert.True(after > 0.05, "after " + after);      // the estimate stays at 0 kg
        Assert.Empty(s.Reset());
        Assert.Equal(0.0, s.History[0].PayloadKg);      // Reset puts the payload back to 0
    }

    [Fact]
    public void ClosedLoopCsvCarriesReferenceAndCommand()
    {
        LabSession s = CompareSession();
        Assert.Empty(s.Reset());
        s.Advance(0.05);
        StringWriter w = new StringWriter();
        s.WriteCsv(w);
        string[] lines = w.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries);
        string[] header = lines[0].TrimEnd('\r').Split(',');
        Assert.Equal("qref1_rad", header[19]);
        Assert.Equal("taucmd4_Nm", header[30]);
        Assert.Equal("payload_kg", header[31]);
        SimulationSample last = s.History[s.History.Count - 1];
        string[] cells = lines[lines.Length - 1].TrimEnd('\r').Split(',');
        Assert.Equal(32, cells.Length);
        Assert.Equal(last.QRef[0], double.Parse(cells[19], CultureInfo.InvariantCulture), 9);
        Assert.Equal(last.TauCommand[3], double.Parse(cells[30], CultureInfo.InvariantCulture), 9);

        StringWriter table = new StringWriter();
        ComparisonCsv.Write(s.CompareControllers(AllKinds), table);
        string[] rows = table.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries);
        Assert.Equal(ComparisonCsv.Header, rows[0].TrimEnd('\r'));
        Assert.Equal(5, rows.Length);
        Assert.StartsWith("\"PID", rows[1]);
    }
}
