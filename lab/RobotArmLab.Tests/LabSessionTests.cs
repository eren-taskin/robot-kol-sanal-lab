using System.Globalization;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Tests;

public class LabSessionTests
{
    // Runs the session to the end with 30 ms "timer ticks", as the form does.
    private static void RunToEnd(LabSession s)
    {
        int guard = 0;
        while (s.CanContinue && guard++ < 100000)
            s.Advance(0.03);
    }

    private static LabSession ShortEarthRun()
    {
        LabSession s = new LabSession();
        s.DurationS = 0.1;
        s.InitialQRad = new[] { 0.0, -0.6, 0.3, 0.2 };
        Assert.Empty(s.Reset());
        return s;
    }

    [Fact]
    public void ResetReproducesIdenticalRuns()
    {
        LabSession s = ShortEarthRun();
        s.Solver = SolverKind.DormandPrince;   // the adaptive solver keeps state between steps: must be rebuilt
        s.Reset();
        RunToEnd(s);
        List<SimulationSample> first = s.History;

        s.Reset();
        RunToEnd(s);
        List<SimulationSample> second = s.History;

        Assert.Equal(first.Count, second.Count);
        for (int n = 0; n < first.Count; n++)
        {
            Assert.Equal(first[n].TimeS, second[n].TimeS);
            Assert.Equal(first[n].Q, second[n].Q);
            Assert.Equal(first[n].Qd, second[n].Qd);
        }
    }

    [Fact]
    public void ChangingAParameterThenResetChangesTheResult()
    {
        LabSession s = ShortEarthRun();
        RunToEnd(s);
        double[] before = s.History[s.History.Count - 1].Q;

        s.Parameters.MassKg[3] = 2.0 * s.Parameters.MassKg[3];
        Assert.Empty(s.Reset());
        RunToEnd(s);
        double[] after = s.History[s.History.Count - 1].Q;

        Assert.NotEqual(before[1], after[1]);
    }

    [Fact]
    public void ParameterChangeWithoutResetDoesNotAffectTheRunningSimulation()
    {
        LabSession a = ShortEarthRun();
        LabSession b = ShortEarthRun();
        b.Parameters.MassKg[3] = 5.0;   // edited in the PropertyGrid, but Sıfırla not pressed
        RunToEnd(a);
        RunToEnd(b);
        Assert.Equal(a.History[a.History.Count - 1].Q, b.History[b.History.Count - 1].Q);
    }

    [Fact]
    public void AdvanceWithSpeedTwoCoversTwiceTheSimTime()
    {
        LabSession s = new LabSession();
        s.Environment = GravityEnvironment.Orbit();
        s.Reset();
        s.Advance(0.1);
        Assert.Equal(0.1, s.TimeS, 9);

        s.SpeedFactor = 2.0;
        s.Reset();
        List<SimulationSample> fresh = s.Advance(0.1);
        Assert.Equal(0.2, s.TimeS, 9);
        Assert.Equal(400, fresh.Count);                 // h = 0.5 ms
        Assert.Equal(401, s.History.Count);             // plus the t = 0 sample
    }

    [Fact]
    public void AdvanceIsCappedSoTheUiCannotFreeze()
    {
        LabSession s = new LabSession();
        s.Environment = GravityEnvironment.Orbit();
        s.StepSizeS = 1e-6;
        s.Reset();
        s.Advance(10.0);   // a 10 s stall of the timer: only 0.25 s of wall clock is honoured, and at most 20000 steps
        Assert.Equal(LabSession.MaxStepsPerCall * 1e-6, s.TimeS, 9);
        s.Advance(0.01);   // the backlog was dropped: the next tick (10000 steps) continues normally
        Assert.Equal(LabSession.MaxStepsPerCall * 1e-6 + 0.01, s.TimeS, 9);
    }

    [Fact]
    public void RunEndsAtTheDurationOrAtTheFloor()
    {
        LabSession orbit = new LabSession();
        orbit.Environment = GravityEnvironment.Orbit();
        orbit.DurationS = 0.25;
        orbit.Reset();
        RunToEnd(orbit);
        Assert.True(orbit.IsFinished);
        Assert.Equal(0.25, orbit.TimeS, 9);
        Assert.Equal("Süre doldu.", orbit.StatusMessage);

        LabSession earth = new LabSession();
        RunToEnd(earth);
        Assert.True(earth.IsStopped);
        Assert.Contains("zemin", earth.StatusMessage);
    }

    [Fact]
    public void CsvHasHeaderAndDotDecimalsUnderTurkishCulture()
    {
        CultureInfo saved = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("tr-TR");
            LabSession s = ShortEarthRun();
            s.Advance(0.01);
            StringWriter w = new StringWriter();
            s.WriteCsv(w);
            string[] lines = w.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries);

            Assert.Equal(CsvExporter.Header, lines[0].TrimEnd('\r'));
            Assert.Equal(s.History.Count + 1, lines.Length);
            string[] cells = lines[lines.Length - 1].TrimEnd('\r').Split(',');
            Assert.Equal(19, cells.Length);
            Assert.Contains(".", cells[0]);
            Assert.Equal(s.TimeS, double.Parse(cells[0], CultureInfo.InvariantCulture), 12);
        }
        finally
        {
            CultureInfo.CurrentCulture = saved;
        }
    }

    [Fact]
    public void InvalidSettingsGiveTurkishErrorsAndDoNotThrow()
    {
        LabSession s = new LabSession();
        s.Advance(0.05);
        double timeBefore = s.TimeS;

        s.Parameters.MassKg[1] = -0.1;
        s.StepSizeS = 0.0;
        s.InitialQRad[1] = 2.0;   // joint 2 limit is ±1.5 rad
        s.Payload.MassKg = -1.0;
        List<string> errors = s.Reset();

        Assert.Contains(errors, e => e.Contains("Uzuv 2 kütlesi"));
        Assert.Contains(errors, e => e.Contains("Adım boyu h"));
        Assert.Contains(errors, e => e.Contains("Yük kütlesi"));
        Assert.Equal(timeBefore, s.TimeS);   // the previous run is kept

        s.Parameters = ArmParameters.Default();
        s.StepSizeS = 0.0005;
        s.Payload.MassKg = 0.0;
        errors = s.Reset();
        Assert.Single(errors);
        Assert.Contains("Eklem 2 başlangıç açısı", errors[0]);
    }

    [Fact]
    public void NonFiniteSettingsAreReportedNotThrown()
    {
        LabSession s = new LabSession();
        s.Environment.GravityMps2 = double.NaN;
        s.Inputs[0].Amplitude = double.PositiveInfinity;
        s.Parameters.LowerLimitRad = new double[3];
        List<string> errors = s.Validate();
        Assert.True(errors.Count >= 3);
    }
}
