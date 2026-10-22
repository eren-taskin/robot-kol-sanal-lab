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
    private const int SeriesCount = 31;          // q, q_ref, q̇, τ, τ_cmd (4'er), konum 3, enerji 3, e 4, takım hatası 1

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
        FormsPlot angles = AddPlot("Açılar q", "q (°)", new[] { "q1", "q2", "q3", "q4" }, 0);
        AddDashed(angles, new[] { "q1 ref", "q2 ref", "q3 ref", "q4 ref" }, 4);
        AddPlot("Hızlar q̇", "q̇ (°/s)", new[] { "q̇1", "q̇2", "q̇3", "q̇4" }, 8);
        FormsPlot torques = AddPlot("Torklar τ", "τ (N·m)", new[] { "τ1", "τ2", "τ3", "τ4" }, 12);
        AddDashed(torques, new[] { "τ1 komut", "τ2 komut", "τ3 komut", "τ4 komut" }, 16);
        AddPlot("Takım konumu", "konum (m)", new[] { "x", "y", "z" }, 20);
        AddPlot("Enerji", "enerji (J)", new[] { "T (kinetik)", "V (potansiyel)", "T + V" }, 23);
        AddPlot("Hata", "e (°), takım hatası (mm)", new[] { "e1", "e2", "e3", "e4", "takım (mm)" }, 26);
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
            if (s.TimeS < lastPlottedTimeS + PlotIntervalS - 1e-9)
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

    // Bir örneğin 31 grafik değeri, SeriesCount sırasıyla. Açılar dereceye çevrilir. Açık çevrimde referans yoktur:
    // referans olarak q, komut olarak τ yazılır (çizgiler gizlidir), hata 0 olur.
    private double[] Values(SimulationSample s)
    {
        double toDeg = 180.0 / Math.PI;
        bool closed = s.QRef != null;
        double[] v = new double[SeriesCount];
        for (int i = 0; i < 4; i++)
        {
            double qRef = closed ? s.QRef[i] : s.Q[i];
            v[i] = s.Q[i] * toDeg;
            v[4 + i] = qRef * toDeg;
            v[8 + i] = s.Qd[i] * toDeg;
            v[12 + i] = s.Tau[i];
            v[16 + i] = closed ? s.TauCommand[i] : s.Tau[i];
            v[26 + i] = (qRef - s.Q[i]) * toDeg;
        }
        v[20] = s.ToolX;
        v[21] = s.ToolY;
        v[22] = s.ToolZ;
        v[23] = s.KineticEnergyJ;
        v[24] = s.PotentialEnergyJ;
        v[25] = s.TotalEnergyJ;
        v[30] = closed ? ControlMetrics.ToolErrorMm(Parameters, s) : 0.0;
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
