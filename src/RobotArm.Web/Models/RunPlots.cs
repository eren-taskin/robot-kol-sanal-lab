using RobotArm.Core.Logging;
using RobotArm.Core.Model;
using RobotArm.Core.Tasks;

namespace RobotArm.Web.Models;

/// <summary>The four time plots of one run (T10 item 1).</summary>
public sealed record RunPlotSet(PlotSpec Angles, PlotSpec Errors, PlotSpec Torques, PlotSpec ToolError, int RowCount);

/// <summary>One finished run kept for comparison (T10 item 3).</summary>
public sealed record RunRecord(int Id, string Label, string Color, Scenario Scenario, RunResult Result);

/// <summary>A signal that can be overlaid across runs.</summary>
public sealed record CompareSignal(string Key, string Label, string AxisLabel);

/// <summary>
/// Builds plot specs from a run log. No physics: every value is read from the log rows that Core wrote
/// (SPEC §10). The plot component decimates each series to at most about 2000 points.
/// </summary>
public static class RunPlots
{
    public static readonly string[] JointColors = ["#2563eb", "#ea580c", "#16a34a", "#9333ea"];
    public static readonly string[] RunColors = ["#2563eb", "#dc2626", "#16a34a", "#9333ea"];
    private const string PayloadColor = "#0f766e";
    private const string BeliefColor = "#b45309";
    private const string LimitColor = "#b91c1c";

    public static readonly double TauMax = ArmModel.OpenManipulatorX().Actuators[0].StallTorque;

    public static readonly IReadOnlyList<CompareSignal> CompareSignals = BuildCompareSignals();

    private static CompareSignal[] BuildCompareSignals()
    {
        var list = new List<CompareSignal> { new("tool", "Uç konum hatası |p − p_ref|", "hata (mm)") };
        for (var i = 1; i <= 4; i++) list.Add(new($"e{i}", $"Eklem {i} hatası e{i}", "e (rad)"));
        for (var i = 1; i <= 4; i++) list.Add(new($"q{i}", $"Eklem {i} açısı q{i}", "q (rad)"));
        for (var i = 1; i <= 4; i++) list.Add(new($"tau{i}", $"Eklem {i} torku τ{i}", "τ (N·m)"));
        return list.ToArray();
    }

    /// <summary>Builds the four plots. <paramref name="logScaleErrors"/> shows |e| and the tool error on log axes.</summary>
    public static RunPlotSet Build(IReadOnlyList<LogRow> log, bool logScaleErrors)
    {
        var n = log.Count;
        var t = new double[n];
        var q = Jagged(n);
        var qd = Jagged(n);
        var e = Jagged(n);
        var tau = Jagged(n);
        var toolErr = new double[n];
        var payload = new double[n];
        var belief = new double[n];
        var anyPayload = false;
        for (var k = 0; k < n; k++)
        {
            var r = log[k];
            t[k] = r.Time;
            for (var i = 0; i < 4; i++)
            {
                q[i][k] = r.Q[i];
                qd[i][k] = r.Reference[i];
                var ei = r.Reference[i] - r.Q[i];
                e[i][k] = logScaleErrors ? Math.Abs(ei) : ei;
                tau[i][k] = r.AppliedTorque[i];
            }
            toolErr[k] = (r.Tool - r.ReferenceTool).Length * 1000.0;
            payload[k] = r.PayloadAttached ? r.PayloadKg : 0.0;
            belief[k] = r.BeliefPayloadKg;
            anyPayload |= r.PayloadKg > 0 || r.BeliefPayloadKg > 0 || r.PayloadAttached;
        }

        var markers = PayloadMarkers(log);
        var timeAxis = new PlotAxis("t (s)");

        var angleSeries = new List<PlotSeries>();
        for (var i = 0; i < 4; i++)
        {
            angleSeries.Add(new PlotSeries($"q{i + 1}", JointColors[i], t, q[i]));
            angleSeries.Add(new PlotSeries($"q{i + 1} ref", JointColors[i], t, qd[i]) { Dashed = true, Width = 1.2, HideInLegend = true });
        }
        var angles = new PlotSpec
        {
            Title = "Eklem açıları ve referanslar",
            X = timeAxis,
            Y = new PlotAxis("q (rad)"),
            Series = angleSeries,
            Markers = markers,
            Note = "Düz çizgi: ölçülen gerçek açı q · kesikli çizgi: referans q_d."
        };

        var errors = new PlotSpec
        {
            Title = logScaleErrors ? "Eklem hataları |e| = |q_d − q| (log)" : "Eklem hataları e = q_d − q",
            X = timeAxis,
            Y = new PlotAxis(logScaleErrors ? "|e| (rad)" : "e (rad)", logScaleErrors ? AxisScale.Log : AxisScale.Linear,
                logScaleErrors ? 1e-7 : null),
            Series = Enumerable.Range(0, 4).Select(i => new PlotSeries($"e{i + 1}", JointColors[i], t, e[i])).ToArray(),
            Markers = markers
        };

        var (bands, clampNote) = ClampBands(log);
        var torques = new PlotSpec
        {
            Title = "Uygulanan motor torkları τ",
            X = timeAxis,
            Y = new PlotAxis("τ (N·m)"),
            Series = Enumerable.Range(0, 4).Select(i => new PlotSeries($"τ{i + 1}", JointColors[i], t, tau[i])).ToArray(),
            HLines =
            [
                new PlotHLine(TauMax, $"+τmax = {LabFormat.Fixed(TauMax, 1)} N·m", LimitColor),
                new PlotHLine(-TauMax, $"−τmax = −{LabFormat.Fixed(TauMax, 1)} N·m", LimitColor)
            ],
            Bands = bands,
            Markers = markers,
            Note = clampNote
        };

        var toolSeries = new List<PlotSeries> { new("|p − p_ref|", "#0f172a", t, toolErr) };
        if (anyPayload)
        {
            toolSeries.Add(new PlotSeries("gerçek yük", PayloadColor, t, payload) { Axis = YAxisSide.Right, Width = 1.4 });
            toolSeries.Add(new PlotSeries("inanılan yük", BeliefColor, t, belief) { Axis = YAxisSide.Right, Dashed = true, Width = 1.4 });
        }
        var tool = new PlotSpec
        {
            Title = "Uç konum hatası" + (logScaleErrors ? " (log)" : ""),
            X = timeAxis,
            Y = new PlotAxis("hata (mm)", logScaleErrors ? AxisScale.Log : AxisScale.Linear, logScaleErrors ? 1e-4 : 0.0),
            Y2 = anyPayload ? new PlotAxis("yük (kg)", AxisScale.Linear, 0.0) : null,
            Series = toolSeries,
            Markers = markers
        };

        return new RunPlotSet(angles, errors, torques, tool, n);
    }

    /// <summary>Overlay of one signal from each kept run.</summary>
    public static PlotSpec Compare(IReadOnlyList<RunRecord> runs, string signalKey)
    {
        var signal = CompareSignals.FirstOrDefault(s => s.Key == signalKey) ?? CompareSignals[0];
        var series = new List<PlotSeries>();
        var markers = new List<PlotMarker>();
        foreach (var run in runs)
        {
            var log = run.Result.Log;
            var t = new double[log.Count];
            var y = new double[log.Count];
            for (var k = 0; k < log.Count; k++)
            {
                var r = log[k];
                t[k] = r.Time;
                y[k] = signal.Key switch
                {
                    "tool" => (r.Tool - r.ReferenceTool).Length * 1000.0,
                    ['e', var c] => r.Reference[c - '1'] - r.Q[c - '1'],
                    ['q', var c] => r.Q[c - '1'],
                    ['t', 'a', 'u', var c] => r.AppliedTorque[c - '1'],
                    _ => double.NaN
                };
            }
            series.Add(new PlotSeries($"#{run.Id} {run.Label}", run.Color, t, y));
            foreach (var m in PayloadMarkers(log))
            {
                markers.Add(m with { Color = run.Color, Label = $"#{run.Id} {m.Label}" });
            }
        }
        var hlines = signal.Key.StartsWith("tau", StringComparison.Ordinal)
            ? new[] { new PlotHLine(TauMax, "+τmax", LimitColor), new PlotHLine(-TauMax, "−τmax", LimitColor) }
            : [];
        return new PlotSpec
        {
            Title = $"Karşılaştırma: {signal.Label}",
            X = new PlotAxis("t (s)"),
            Y = new PlotAxis(signal.AxisLabel, AxisScale.Linear, signal.Key == "tool" ? 0.0 : null),
            Series = series,
            Markers = markers,
            HLines = hlines
        };
    }

    /// <summary>Vertical lines where the attachment flag changes between logged samples.</summary>
    private static PlotMarker[] PayloadMarkers(IReadOnlyList<LogRow> log)
    {
        var list = new List<PlotMarker>();
        for (var k = 1; k < log.Count; k++)
        {
            if (log[k].PayloadAttached == log[k - 1].PayloadAttached) continue;
            list.Add(log[k].PayloadAttached
                ? new PlotMarker(log[k].Time, "yük alındı", PayloadColor)
                : new PlotMarker(log[k].Time, "yük bırakıldı", PayloadColor));
        }
        return list.ToArray();
    }

    /// <summary>
    /// Shaded intervals where a joint's torque was limited: controller clamping or plant saturation
    /// (SPEC §9 keeps them apart; both mean τ sits at ±τmax). Each row covers [t, t + interval).
    /// </summary>
    private static (PlotBand[] Bands, string? Note) ClampBands(IReadOnlyList<LogRow> log)
    {
        var bands = new List<PlotBand>();
        var clampTime = new double[4];
        var satTime = new double[4];
        for (var i = 0; i < 4; i++)
        {
            var bit = 1 << i;
            double? start = null;
            double end = 0;
            foreach (var r in log)
            {
                var limited = ((r.ClampedMask | r.SaturatedMask) & bit) != 0;
                if ((r.ClampedMask & bit) != 0) clampTime[i] += r.Interval;
                if ((r.SaturatedMask & bit) != 0) satTime[i] += r.Interval;
                if (limited)
                {
                    start ??= r.Time;
                    end = r.Time + r.Interval;
                }
                else if (start is double s)
                {
                    bands.Add(new PlotBand(s, end, JointColors[i], 0.16));
                    start = null;
                }
            }
            if (start is double s2) bands.Add(new PlotBand(s2, end, JointColors[i], 0.16));
        }

        var parts = new List<string>();
        for (var i = 0; i < 4; i++)
        {
            if (clampTime[i] > 0) parts.Add($"E{i + 1} kontrolcü kırptı {LabFormat.Fixed(clampTime[i], 3)} s");
            if (satTime[i] > 0) parts.Add($"E{i + 1} motor doydu {LabFormat.Fixed(satTime[i], 3)} s");
        }
        var note = parts.Count == 0
            ? "Hiçbir eklemde kırpma veya doyum yok; gölgeli bölge oluşmadı."
            : "Gölgeli bölgeler (eklem rengiyle): " + string.Join(" · ", parts) + ".";
        return (bands.ToArray(), note);
    }

    private static double[][] Jagged(int n) => [new double[n], new double[n], new double[n], new double[n]];
}
