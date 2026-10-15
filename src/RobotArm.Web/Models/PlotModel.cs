using System.Globalization;

namespace RobotArm.Web.Models;

/// <summary>Scale of one plot axis.</summary>
public enum AxisScale
{
    Linear,
    Log
}

/// <summary>Which y axis a series, line or band belongs to.</summary>
public enum YAxisSide
{
    Left,
    Right
}

/// <summary>One plot axis. Min/Max null means "fit the data".</summary>
public sealed record PlotAxis(string Label, AxisScale Scale = AxisScale.Linear, double? Min = null, double? Max = null);

/// <summary>
/// One series of (x, y) points. The arrays are owned by the series and must not be changed after it is
/// built. X need not be monotonic (root locus, pole maps). Non-finite points break the line.
/// </summary>
public sealed record PlotSeries(string Name, string Color, double[] X, double[] Y)
{
    public YAxisSide Axis { get; init; } = YAxisSide.Left;
    public bool Dashed { get; init; }
    public double Width { get; init; } = 1.6;
    /// <summary>Draw points as small circles instead of a line (pole maps).</summary>
    public bool PointsOnly { get; init; }
    /// <summary>Not listed in the legend (for example the second half of a paired series).</summary>
    public bool HideInLegend { get; init; }
}

/// <summary>Vertical line at X, for example a payload attach/release event.</summary>
public sealed record PlotMarker(double X, string Label, string Color);

/// <summary>Shaded x interval [X0, X1], for example controller clamping.</summary>
public sealed record PlotBand(double X0, double X1, string Color, double Opacity = 0.14);

/// <summary>Horizontal line at Y, for example the ±τmax limits.</summary>
public sealed record PlotHLine(double Y, string Label, string Color, YAxisSide Axis = YAxisSide.Left);

/// <summary>Everything one plot shows. Build a new instance to change the plot; the component compares references.</summary>
public sealed record PlotSpec
{
    public string Title { get; init; } = "";
    public PlotAxis X { get; init; } = new("");
    public PlotAxis Y { get; init; } = new("");
    /// <summary>Optional second y axis on the right; series with Axis = Right use it.</summary>
    public PlotAxis? Y2 { get; init; }
    public IReadOnlyList<PlotSeries> Series { get; init; } = [];
    public IReadOnlyList<PlotMarker> Markers { get; init; } = [];
    public IReadOnlyList<PlotBand> Bands { get; init; } = [];
    public IReadOnlyList<PlotHLine> HLines { get; init; } = [];
    /// <summary>Same data units per pixel on x and y (linear axes only), for z-plane and s-plane plots.</summary>
    public bool EqualAspect { get; init; }
    /// <summary>Draw the unit circle (with EqualAspect, for z-plane pole maps).</summary>
    public bool UnitCircle { get; init; }
    /// <summary>Optional note shown under the legend, e.g. which joints were clamped.</summary>
    public string? Note { get; init; }
}

/// <summary>Plot helpers: decimation, nice ticks and on-screen number formatting.</summary>
public static class PlotMath
{
    /// <summary>Maximum points kept per series (T10: about 2000).</summary>
    public const int MaxPointsPerSeries = 2000;

    /// <summary>
    /// Min/max decimation for series with increasing x: the data are cut into buckets and each bucket keeps
    /// its first, lowest and highest point in x order, so peaks and steps survive. At most about
    /// <paramref name="maxPoints"/> points are returned. Shorter series are returned unchanged.
    /// </summary>
    public static (double[] X, double[] Y) Decimate(double[] x, double[] y, int maxPoints = MaxPointsPerSeries)
    {
        var n = Math.Min(x.Length, y.Length);
        if (n <= maxPoints || maxPoints < 6)
        {
            return (x, y);
        }

        var buckets = maxPoints / 3;
        var outX = new List<double>(maxPoints + 3);
        var outY = new List<double>(maxPoints + 3);
        for (var b = 0; b < buckets; b++)
        {
            var start = (int)((long)b * n / buckets);
            var end = (int)((long)(b + 1) * n / buckets);
            if (end <= start) continue;
            int iMin = start, iMax = start;
            for (var i = start + 1; i < end; i++)
            {
                if (y[i] < y[iMin] || double.IsNaN(y[iMin])) iMin = i;
                if (y[i] > y[iMax] || double.IsNaN(y[iMax])) iMax = i;
            }
            Add(start);
            var (a, c) = iMin < iMax ? (iMin, iMax) : (iMax, iMin);
            if (a != start) Add(a);
            if (c != a && c != start) Add(c);
        }
        if (outX[^1] != x[n - 1]) Add(n - 1);
        return (outX.ToArray(), outY.ToArray());

        void Add(int i)
        {
            outX.Add(x[i]);
            outY.Add(y[i]);
        }
    }

    /// <summary>Linear ticks at 1, 2 or 5 × 10^k, about <paramref name="target"/> of them.</summary>
    public static (double[] Ticks, double Step) LinearTicks(double min, double max, int target)
    {
        var span = max - min;
        if (!(span > 0) || !double.IsFinite(span))
        {
            return ([min], 1);
        }
        var raw = span / Math.Max(1, target);
        var mag = Math.Pow(10, Math.Floor(Math.Log10(raw)));
        var norm = raw / mag;
        var step = (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
        var first = Math.Ceiling(min / step - 1e-9) * step;
        var ticks = new List<double>();
        for (var v = first; v <= max + step * 1e-9 && ticks.Count < 50; v += step)
        {
            ticks.Add(Math.Abs(v) < step * 1e-9 ? 0.0 : v);
        }
        return (ticks.ToArray(), step);
    }

    /// <summary>Log ticks: every decade, or every n-th decade when there are many; adds 2 and 5 within one or two decades.</summary>
    public static double[] LogTicks(double min, double max, int target)
    {
        var lo = Math.Floor(Math.Log10(min) + 1e-9);
        var hi = Math.Ceiling(Math.Log10(max) - 1e-9);
        var decades = Math.Max(1, (int)(hi - lo));
        var ticks = new List<double>();
        if (decades <= 2)
        {
            for (var d = lo; d <= hi; d++)
            {
                foreach (var m in new[] { 1.0, 2.0, 5.0 })
                {
                    var v = m * Math.Pow(10, d);
                    if (v >= min * (1 - 1e-9) && v <= max * (1 + 1e-9)) ticks.Add(v);
                }
            }
        }
        else
        {
            var every = Math.Max(1, (int)Math.Ceiling(decades / (double)Math.Max(2, target)));
            for (var d = lo; d <= hi; d += every)
            {
                var v = Math.Pow(10, d);
                if (v >= min * (1 - 1e-9) && v <= max * (1 + 1e-9)) ticks.Add(v);
            }
        }
        if (ticks.Count == 0) ticks.Add(min);
        return ticks.ToArray();
    }

    /// <summary>Tick label with a decimal comma; the number of decimals follows the tick step.</summary>
    public static string FormatTick(double v, double step)
    {
        if (v == 0) return "0";
        var a = Math.Abs(v);
        if (a >= 1e5 || a < 1e-4)
        {
            return v.ToString("0.#E+0", CultureInfo.InvariantCulture).Replace('.', ',');
        }
        var decimals = step > 0 ? Math.Clamp((int)Math.Ceiling(-Math.Log10(step) - 1e-9), 0, 6) : 3;
        return v.ToString("F" + decimals, CultureInfo.InvariantCulture).Replace('.', ',');
    }

    /// <summary>Log tick label: plain numbers between 1e-3 and 1e4, otherwise 1E-5 style.</summary>
    public static string FormatLogTick(double v)
    {
        var a = Math.Abs(v);
        if (a >= 1e-3 && a < 1e4)
        {
            return v.ToString("0.###", CultureInfo.InvariantCulture).Replace('.', ',');
        }
        return v.ToString("0E+0", CultureInfo.InvariantCulture);
    }

    /// <summary>Read-out value with four significant digits and a decimal comma.</summary>
    public static string FormatValue(double v) =>
        double.IsFinite(v) ? v.ToString("G4", CultureInfo.InvariantCulture).Replace('.', ',') : "—";

    /// <summary>SVG coordinate with invariant culture.</summary>
    public static string Svg(double v) => v.ToString("0.##", CultureInfo.InvariantCulture);
}
