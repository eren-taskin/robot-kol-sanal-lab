using System.Diagnostics;
using System.Globalization;
using RobotArm.Core.Model;
using RobotArm.Core.Tasks;
using Xunit.Abstractions;

namespace RobotArm.Tests;

public sealed class RobustnessReportTests(ITestOutputHelper output)
{
    [Fact]
    [Trait("Category", "Report")]
    public void GenerateRobustnessCsv_AndMeasureReleasePickPlace()
    {
        var root = FindRoot(); var directory = Path.Combine(root, "docs", "results"); Directory.CreateDirectory(directory);
        var path = Path.Combine(directory, "robustness.csv");
        var rows = new List<(TaskKind Task, string Planet, string Controller, bool Success)>();
        using (var writer = new StreamWriter(path, false, new System.Text.UTF8Encoding(false)))
        {
            writer.WriteLine("task,planet,payload_kg,controller,belief,success,settling_time_s,rms_tool_error_m,peak_abs_tau_Nm,clamped_percent,fault_reason");
            foreach (var task in new[] { TaskKind.Reach, TaskKind.PickPlace })
            foreach (var planet in Planet.All)
            foreach (var payload in new[] { 0.0, 0.1, 0.3, 0.5 })
            foreach (var control in new[] { "PID", "PDG", "CT", "CT+I" })
            {
                var baseline = task == TaskKind.PickPlace ? ScenarioLibrary.EarthPickPlace : new Scenario();
                var s = baseline with
                {
                    PlanetKey = planet.Key, PayloadKg = payload,
                    Controller = new ControllerSettings { Method = control.StartsWith("CT", StringComparison.Ordinal) ? ControlMethod.CT : Enum.Parse<ControlMethod>(control), IntegralEnabled = control == "CT+I" },
                    Belief = new BeliefSettings { Knowledge = PayloadKnowledge.Informed }
                };
                var r = new ExperimentRunner().Run(s);
                Assert.NotEmpty(r.Log); Assert.True(double.IsFinite(r.Metrics.RmsToolError));
                Assert.Equal(r.Success, r.FailureReason is null);
                var m = r.Metrics;
                writer.WriteLine(string.Join(',', task, planet.Key, Number(payload), control, "informed", r.Success ? "true" : "false",
                    m.SettlingTime is double settling ? Number(settling) : "", Number(m.RmsToolError),
                    Number(ExperimentTests.Max(m.PeakTorque)), Number(m.AnyClampedPercent), Quote(r.FailureReason ?? "")));
                rows.Add((task, planet.Key, control, r.Success));
            }
        }
        Assert.Equal(160, rows.Count);
        foreach (var group in rows.GroupBy(r => (r.Task, r.Controller)))
        {
            var counts = Planet.All.Select(p => group.Count(r => r.Planet == p.Key && r.Success));
            output.WriteLine($"{group.Key.Task} | {group.Key.Controller} | {string.Join(" | ", counts)} | {group.Count(r => r.Success)}/20");
        }
        var scenario = ScenarioLibrary.EarthPickPlace;
        // Warm the JIT and tiered compilation before measuring one complete run, including logs and metrics.
        for (var i = 0; i < 3; i++) Assert.True(new ExperimentRunner().Run(scenario).Success);
        var watch = Stopwatch.StartNew(); var benchmark = new ExperimentRunner().Run(scenario); watch.Stop();
        Assert.True(benchmark.Success); Assert.Equal(5000, benchmark.Log.Count); Assert.Equal(10, benchmark.Final.Time);
        output.WriteLine(FormattableString.Invariant($"10 s Earth PickPlace, CT, 0.3 kg informed: {watch.Elapsed.TotalMilliseconds:F3} ms; {10 / watch.Elapsed.TotalSeconds:F2}x simulated/real; {benchmark.Log.Count} rows. Runtime: {System.Runtime.InteropServices.RuntimeInformation.FrameworkDescription}; OS: {System.Runtime.InteropServices.RuntimeInformation.OSDescription}; architecture: {System.Runtime.InteropServices.RuntimeInformation.ProcessArchitecture}."));
        output.WriteLine(path);
    }
    private static string Number(double value) => value.ToString("R", CultureInfo.InvariantCulture);
    private static string Quote(string value) => "\"" + value.Replace("\"", "\"\"", StringComparison.Ordinal) + "\"";
    private static string FindRoot()
    {
        for (var directory = new DirectoryInfo(AppContext.BaseDirectory); directory is not null; directory = directory.Parent)
            if (File.Exists(Path.Combine(directory.FullName, "PROJECT.md"))) return directory.FullName;
        throw new DirectoryNotFoundException("Cannot find the repository root for the report output.");
    }
}
