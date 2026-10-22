using System;
using System.Collections.Generic;
using System.Windows.Forms;
using RobotArmLab.Core.Simulation;
using ScottPlot.WinForms;

namespace RobotArmLab.Forms;

/// <summary>
/// Alttaki grafik sekmeleri (ScottPlot): açılar q, hızlar q̇, torklar τ, takım konumu ve enerji. ScottPlot yalnız
/// çizer; çizilen bütün sayılar simülasyon örneklerinden (SimulationSample) gelir. Tork grafiği, motor modeli ve
/// doyumdan sonra kola gerçekten uygulanan torku gösterir.
/// </summary>
public class PlotPanel : UserControl
{
    private const double PlotIntervalS = 0.005;   // grafiğe 5 ms'de bir nokta eklenir (çizim hızlı kalsın diye)
    private const int SeriesCount = 18;          // 4 q + 4 q̇ + 4 τ + 3 konum + 3 enerji

    private readonly TabControl tabs = new TabControl();
    private readonly List<FormsPlot> plots = new List<FormsPlot>();
    private readonly List<double> time = new List<double>();
    private readonly List<double>[] series = new List<double>[SeriesCount];
    private double lastPlottedTimeS = double.NegativeInfinity;

    /// <summary>Beş grafik sekmesini kurar. Her çizgi, zaman listesini ve kendi değer listesini doğrudan okur.</summary>
    public PlotPanel()
    {
        tabs.Dock = DockStyle.Fill;
        AddPlot("Açılar q", "q (°)", new[] { "q1", "q2", "q3", "q4" }, 0);
        AddPlot("Hızlar q̇", "q̇ (°/s)", new[] { "q̇1", "q̇2", "q̇3", "q̇4" }, 4);
        AddPlot("Torklar τ", "τ (N·m)", new[] { "τ1", "τ2", "τ3", "τ4" }, 8);
        AddPlot("Takım konumu", "konum (m)", new[] { "x", "y", "z" }, 12);
        AddPlot("Enerji", "enerji (J)", new[] { "T (kinetik)", "V (potansiyel)", "T + V" }, 15);
        tabs.SelectedIndexChanged += delegate (object sender, EventArgs e) { RefreshVisible(); };
        Controls.Add(tabs);
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
        if (tabs.SelectedIndex < 0)
            return;
        FormsPlot plot = plots[tabs.SelectedIndex];
        plot.Plot.Axes.AutoScale();
        plot.Refresh();
    }

    // Bir örneğin 18 grafik değeri, SeriesCount sırasıyla. Açı ve hız dereceye çevrilir.
    private static double[] Values(SimulationSample s)
    {
        double toDeg = 180.0 / Math.PI;
        double[] v = new double[SeriesCount];
        for (int i = 0; i < 4; i++)
        {
            v[i] = s.Q[i] * toDeg;
            v[4 + i] = s.Qd[i] * toDeg;
            v[8 + i] = s.Tau[i];
        }
        v[12] = s.ToolX;
        v[13] = s.ToolY;
        v[14] = s.ToolZ;
        v[15] = s.KineticEnergyJ;
        v[16] = s.PotentialEnergyJ;
        v[17] = s.TotalEnergyJ;
        return v;
    }

    // Bir sekme ve içinde bir grafik oluşturur; names.Length kadar çizgi ekler (ilk çizgi firstSeries numaralı seri).
    private void AddPlot(string title, string yLabel, string[] names, int firstSeries)
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
    }
}
