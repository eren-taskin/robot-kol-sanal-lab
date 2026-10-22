using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Windows.Forms;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;
using ScottPlot.WinForms;

namespace RobotArmLab.Forms;

/// <summary>
/// Alttaki grafik sekmeleri (ScottPlot): açılar q (kesikli: referans q_ref), hızlar q̇, torklar τ (kesikli: kontrolcünün
/// komutu), takım konumu, enerji ve hata (eklem hatası e = q_ref − q ve takım hatası). ScottPlot yalnız çizer;
/// çizilen bütün sayılar simülasyon örneklerinden gelir. Kesikli çizgiler yalnız kapalı çevrimde görünür.
/// </summary>
public class PlotPanel : UserControl
{
    private const double PlotIntervalS = 0.005;   // grafiğe 5 ms'de bir nokta eklenir (çizim hızlı kalsın diye)
    private const double TimeToleranceS = 1e-9;   // örnek zamanları k·h toplamıdır; tam 5 ms yerine 4,999999 ms çıkabilir

    // Bütün çizgilerin değerleri tek bir "series" dizisinde tutulur. Her grup, dizide şu numaradan başlar:
    private const int AngleSeries = 0;        // q1..q4 (4 seri)
    private const int AngleRefSeries = 4;     // q_ref1..4 (4)
    private const int SpeedSeries = 8;        // q̇1..4 (4)
    private const int TorqueSeries = 12;      // τ1..4 (4)
    private const int TorqueCmdSeries = 16;   // τ_cmd1..4 (4)
    private const int ToolSeries = 20;        // x, y, z (3)
    private const int EnergySeries = 23;      // T, V, T + V (3)
    private const int ErrorSeries = 26;       // e1..4 (4)
    private const int ToolErrorSeries = 30;   // takım hatası (1)
    private const int SeriesCount = 31;       // toplam

    private readonly TabControl tabs = new TabControl();
    private readonly List<FormsPlot> plots = new List<FormsPlot>();
    private readonly List<double> time = new List<double>();
    private readonly List<double>[] series = new List<double>[SeriesCount];
    private readonly List<ScottPlot.Plottables.Scatter> closedLoopLines = new List<ScottPlot.Plottables.Scatter>();
    private double lastPlottedTimeS = double.NegativeInfinity;

    /// <summary>Kol parametreleri (takım hatası için ileri kinematik); MainForm Sıfırla'da günceller.</summary>
    [Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public ArmParameters Parameters { get; set; } = ArmParameters.Default();

    /// <summary>Altı grafik sekmesini kurar. Her çizgi, zaman listesini ve kendi değer listesini doğrudan okur.</summary>
    public PlotPanel()
    {
        tabs.Dock = DockStyle.Fill;
        FormsPlot angles = AddPlot("Açılar q", "q (°)", new[] { "q1", "q2", "q3", "q4" }, AngleSeries);
        AddDashed(angles, new[] { "q1 ref", "q2 ref", "q3 ref", "q4 ref" }, AngleRefSeries);
        AddPlot("Hızlar q̇", "q̇ (°/s)", new[] { "q̇1", "q̇2", "q̇3", "q̇4" }, SpeedSeries);
        FormsPlot torques = AddPlot("Torklar τ", "τ (N·m)", new[] { "τ1", "τ2", "τ3", "τ4" }, TorqueSeries);
        AddDashed(torques, new[] { "τ1 komut", "τ2 komut", "τ3 komut", "τ4 komut" }, TorqueCmdSeries);
        AddPlot("Takım konumu", "konum (m)", new[] { "x", "y", "z" }, ToolSeries);
        AddPlot("Enerji", "enerji (J)", new[] { "T (kinetik)", "V (potansiyel)", "T + V" }, EnergySeries);
        // Hata grafiği e1..e4 ve takım hatasını birlikte çizer (ErrorSeries'ten başlayan 5 seri: 26..30).
        AddPlot("Hata", "e (°), takım hatası (mm)", new[] { "e1", "e2", "e3", "e4", "takım (mm)" }, ErrorSeries);
        tabs.SelectedIndexChanged += delegate (object sender, EventArgs e) { RefreshVisible(); };
        Controls.Add(tabs);
        SetClosedLoop(false);
    }

    /// <summary>Grafik sekmelerinin sonuna başka bir sekme ekler (ör. Karşılaştır paneli).</summary>
    public void AddPage(string title, Control content)
    {
        content.Dock = DockStyle.Fill;
        TabPage page = new TabPage(title);
        page.Controls.Add(content);
        tabs.TabPages.Add(page);
    }

    /// <summary>Referans ve komut çizgilerini kapalı çevrimde gösterir, açık çevrimde gizler.</summary>
    public void SetClosedLoop(bool closedLoop)
    {
        for (int k = 0; k < closedLoopLines.Count; k++)
            closedLoopLines[k].IsVisible = closedLoop;
    }

    /// <summary>Bütün grafikleri boşaltır (Sıfırla'da çağrılır).</summary>
    public void Clear()
    {
        time.Clear();
        for (int k = 0; k < SeriesCount; k++)
            series[k].Clear();
        lastPlottedTimeS = double.NegativeInfinity;
        RefreshVisible();
    }

    /// <summary>Yeni örnekleri ekler; birbirine PlotIntervalS'den yakın örnekler atlanır.</summary>
    public void Append(List<SimulationSample> samples)
    {
        for (int n = 0; n < samples.Count; n++)
        {
            SimulationSample s = samples[n];
            if (s.TimeS < lastPlottedTimeS + PlotIntervalS - TimeToleranceS)
                continue;
            lastPlottedTimeS = s.TimeS;
            time.Add(s.TimeS);
            double[] v = Values(s);
            for (int k = 0; k < SeriesCount; k++)
                series[k].Add(v[k]);
        }
    }

    /// <summary>Yalnız görünen sekmedeki grafiği eksenleri verilere uydurarak yeniden çizer.</summary>
    public void RefreshVisible()
    {
        if (tabs.SelectedIndex < 0 || tabs.SelectedIndex >= plots.Count)
            return;
        FormsPlot plot = plots[tabs.SelectedIndex];
        plot.Plot.Axes.AutoScale();
        plot.Refresh();
    }

    // Bir örneğin 31 grafik değeri, yukarıdaki seri numaralarına yerleştirilir. Açılar dereceye çevrilir. Açık çevrimde referans yoktur:
    // referans olarak q, komut olarak τ yazılır (çizgiler gizlidir), hata 0 olur.
    private double[] Values(SimulationSample s)
    {
        double toDeg = 180.0 / Math.PI;
        bool closed = s.QRef != null;
        double[] v = new double[SeriesCount];
        for (int i = 0; i < 4; i++)
        {
            double qRef = closed ? s.QRef[i] : s.Q[i];
            v[AngleSeries + i] = s.Q[i] * toDeg;
            v[AngleRefSeries + i] = qRef * toDeg;
            v[SpeedSeries + i] = s.Qd[i] * toDeg;
            v[TorqueSeries + i] = s.Tau[i];
            v[TorqueCmdSeries + i] = closed ? s.TauCommand[i] : s.Tau[i];
            v[ErrorSeries + i] = (qRef - s.Q[i]) * toDeg;
        }
        v[ToolSeries] = s.ToolX;
        v[ToolSeries + 1] = s.ToolY;
        v[ToolSeries + 2] = s.ToolZ;
        v[EnergySeries] = s.KineticEnergyJ;
        v[EnergySeries + 1] = s.PotentialEnergyJ;
        v[EnergySeries + 2] = s.TotalEnergyJ;
        v[ToolErrorSeries] = closed ? ControlMetrics.ToolErrorMm(Parameters, s) : 0.0;
        return v;
    }

    // Bir sekme ve içinde bir grafik oluşturur; names.Length kadar çizgi ekler (ilk çizgi firstSeries numaralı seri).
    private FormsPlot AddPlot(string title, string yLabel, string[] names, int firstSeries)
    {
        FormsPlot plot = new FormsPlot();
        plot.Dock = DockStyle.Fill;
        for (int k = 0; k < names.Length; k++)
        {
            series[firstSeries + k] = new List<double>();
            ScottPlot.Plottables.Scatter line = plot.Plot.Add.ScatterLine(time, series[firstSeries + k]);
            line.LegendText = names[k];
            line.LineWidth = 1.5f;
        }
        plot.Plot.XLabel("t (s)");
        plot.Plot.YLabel(yLabel);
        plot.Plot.ShowLegend(ScottPlot.Alignment.UpperRight);
        plots.Add(plot);

        TabPage page = new TabPage(title);
        page.Controls.Add(plot);
        tabs.TabPages.Add(page);
        return plot;
    }

    // Var olan dört çizginin üstüne aynı renkte kesikli dört çizgi ekler (referans veya komut); yalnız kapalı çevrimde görünür.
    private void AddDashed(FormsPlot plot, string[] names, int firstSeries)
    {
        List<ScottPlot.IPlottable> existing = new List<ScottPlot.IPlottable>(plot.Plot.GetPlottables());
        for (int k = 0; k < names.Length; k++)
        {
            series[firstSeries + k] = new List<double>();
            ScottPlot.Plottables.Scatter line = plot.Plot.Add.ScatterLine(time, series[firstSeries + k]);
            line.LegendText = names[k];
            line.LineWidth = 1.5f;
            line.LinePattern = ScottPlot.LinePattern.Dashed;
            line.Color = ((ScottPlot.Plottables.Scatter)existing[k]).Color;
            closedLoopLines.Add(line);
        }
    }
}
