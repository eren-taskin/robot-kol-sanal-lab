using System.Globalization;
using RobotArm.Core.Tasks;
using Xunit.Abstractions;

namespace RobotArm.Tests;

/// <summary>Report category (excluded from CI): writes the S1 comparison and gain-scale tables (docs/DECISIONS.md D13).</summary>
public sealed class StsmcReportTests(ITestOutputHelper output)
{
    [Fact]
    [Trait("Category", "Report")]
    public void GenerateStsmcComparisonCsv()
    {
        var directory = Path.Combine(FindRoot(), "docs", "results"); Directory.CreateDirectory(directory);
        var path = Path.Combine(directory, "stsmc_comparison.csv");
        using (var writer = new StreamWriter(path, false, new System.Text.UTF8Encoding(false)))
        {
            writer.WriteLine("case,controller," + Columns("rms_moving_rad") + "," + Columns("peak_error_rad") + "," + Columns("final_error_rad") + ","
                + Columns("chatter_Nm") + "," + Columns("clamped_percent") + ",any_clamped_percent,fault,fault_time_s,fault_joint");
            foreach (var under in new[] { true, false })
            foreach (var controller in StsmcExperiment.Controllers)
            {
                var r = StsmcExperiment.Compare(under, controller);
                writer.WriteLine(string.Join(',', r.Case, Quote(controller), Numbers(r.Rms), Numbers(r.Peak), Numbers(r.Final), Numbers(r.Chatter),
                    Numbers(r.ClampedPercent), Number(r.AnyClampedPercent), r.Fault?.Reason.ToString() ?? "none",
                    r.Fault is null ? "" : Number(r.Fault.Time), r.Fault is null ? "" : (r.Fault.Joint + 1).ToString(CultureInfo.InvariantCulture)));
            }
        }
        output.WriteLine(File.ReadAllText(path));

        // Sensitivity: k1 = 1.5√(cL), k2 = 1.1cL with the measured L scaled by c.
        var scalePath = Path.Combine(directory, "stsmc_gain_scale.csv");
        var gains = RobotArm.Core.Control.SuperTwistingSlidingMode.DefaultGains;
        using (var writer = new StreamWriter(scalePath, false, new System.Text.UTF8Encoding(false)))
        {
            writer.WriteLine("case,L_scale,max_rms_moving_rad,max_peak_error_rad,max_chatter_Nm,any_clamped_percent,fault");
            foreach (var under in new[] { true, false })
            foreach (var c in new[] { 1.0, 3.0, 10.0, 30.0, 100.0 })
            {
                var s = StsmcExperiment.Case(under, "STSMC (ε = 0)");
                s = s with { Controller = s.Controller with
                {
                    K1 = JointVector.From(gains.K1.Select(k => k * Math.Sqrt(c)).ToArray()),
                    K2 = JointVector.From(gains.K2.Select(k => k * c).ToArray())
                } };
                var r = StsmcExperiment.Compare(s, under ? "underestimate" : "overestimate", "STSMC (ε = 0)");
                writer.WriteLine(string.Join(',', r.Case, Number(c), Number(r.Rms.Max()), Number(r.Peak.Max()), Number(r.Chatter.Max()),
                    Number(r.AnyClampedPercent), r.Fault?.Reason.ToString() ?? "none"));
            }
        }
        output.WriteLine(File.ReadAllText(scalePath));
    }

    private static string Columns(string name) => string.Join(',', Enumerable.Range(1, 4).Select(i => $"{name}_j{i}"));
    private static string Numbers(double[] values) => string.Join(',', values.Select(Number));
    private static string Number(double value) => value.ToString("R", CultureInfo.InvariantCulture);
    private static string Quote(string value) => "\"" + value.Replace("\"", "\"\"", StringComparison.Ordinal) + "\"";

    private static string FindRoot()
    {
        var directory = new DirectoryInfo(AppContext.BaseDirectory);
        while (directory is not null && !File.Exists(Path.Combine(directory.FullName, "RobotArm.sln"))) directory = directory.Parent;
        return directory?.FullName ?? throw new DirectoryNotFoundException("RobotArm.sln not found.");
    }
}
