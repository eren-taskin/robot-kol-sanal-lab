using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Windows.Forms;
using RobotArmLab.Core.Session;
using ScottPlot.WinForms;

namespace RobotArmLab.Forms;

/// <summary>
/// "Karşılaştır" sekmesi: işaretlenen kontrolcüler aynı deneyle (o anki bütün ayarlar) baştan sona koşturulur;
/// takım hatası eğrileri üst üste çizilir ve ölçüt tablosu gösterilir. Hesap LabSession.CompareControllers'dadır;
/// bu panel yalnız listeyi okur, sonucu çizer ve tabloyu CSV'ye kaydettirir.
/// </summary>
public class ComparePanel : UserControl
{
    private static readonly string[] Columns =
    {
        "RMS (rad)", "En büyük (rad)", "Son (rad)", "Takım en büyük (mm)", "Takım son (mm)", "Oturma (s)",
        "Aşma (%)", "En büyük |τ| (N·m)", "Doyum (s)", "∫τ² dt", "Titreşim (N·m/s)",
    };

    private readonly CheckedListBox listControllers = new CheckedListBox();
    private readonly Button btnRun = new Button();
    private readonly Button btnSaveTable = new Button();
    private readonly FormsPlot plot = new FormsPlot();
    private readonly DataGridView grid = new DataGridView();
    private readonly SaveFileDialog saveDialog = new SaveFileDialog();
    private List<ComparisonResult> results = new List<ComparisonResult>();

    /// <summary>
    /// Karşılaştırmadan önce çağrılır: form kontrollerdeki değerleri oturuma yazar ve oturumu döndürür.
    /// MainForm bunu kurar; panel formun diğer sekmelerini bilmez.
    /// </summary>
    [Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public Func<LabSession> PrepareSession { get; set; }

    /// <summary>Denetimleri kurar: solda kontrolcü listesi ve düğmeler, sağda grafik ve tablo.</summary>
    public ComparePanel()
    {
        Panel left = new Panel();
        left.Dock = DockStyle.Left;
        left.Width = 230;
        listControllers.Dock = DockStyle.Top;
        listControllers.Height = 90;
        listControllers.CheckOnClick = true;
        string[] names = ClosedLoopBuilder.DisplayNames();
        for (int k = 0; k < names.Length; k++)
            listControllers.Items.Add(names[k], true);
        SetUpButton(btnRun, "Karşılaştır (aynı deney)", btnRun_Click);
        SetUpButton(btnSaveTable, "Tabloyu CSV kaydet", btnSaveTable_Click);
        left.Controls.Add(btnSaveTable);
        left.Controls.Add(btnRun);
        left.Controls.Add(listControllers);

        SplitContainer split = new SplitContainer();
        split.Dock = DockStyle.Fill;
        split.Orientation = Orientation.Horizontal;
        plot.Dock = DockStyle.Fill;
        plot.Plot.XLabel("t (s)");
        plot.Plot.YLabel("takım hatası (mm)");
        split.Panel1.Controls.Add(plot);
        SetUpGrid();
        split.Panel2.Controls.Add(grid);
        Controls.Add(split);
        Controls.Add(left);
        saveDialog.DefaultExt = "csv";
        saveDialog.FileName = "karsilastirma.csv";
        saveDialog.Filter = "CSV dosyası (*.csv)|*.csv";
    }

    private static void SetUpButton(Button b, string text, EventHandler click)
    {
        b.Dock = DockStyle.Top;
        b.Height = 32;
        b.Text = text;
        b.Click += click;
    }

    // Salt okunur tablo: ilk sütun kontrolcü adı, sonra 11 ölçüt, sonda durma nedeni.
    private void SetUpGrid()
    {
        grid.Dock = DockStyle.Fill;
        grid.ReadOnly = true;
        grid.AllowUserToAddRows = false;
        grid.RowHeadersVisible = false;
        grid.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.AllCells;
        grid.Columns.Add("name", "Kontrolcü");
        for (int c = 0; c < Columns.Length; c++)
            grid.Columns.Add("m" + c, Columns[c]);
        grid.Columns.Add("message", "Not");
    }

    // İşaretli kontrolcüleri koşturur (birkaç saniye sürebilir; imleç bekleme imlecine döner).
    private void btnRun_Click(object sender, EventArgs e)
    {
        LabSession session = PrepareSession == null ? null : PrepareSession();
        List<ControllerKind> kinds = new List<ControllerKind>();
        for (int k = 0; k < listControllers.Items.Count; k++)
        {
            if (listControllers.GetItemChecked(k))
                kinds.Add((ControllerKind)k);
        }
        if (session == null || kinds.Count == 0)
            return;
        Cursor = Cursors.WaitCursor;
        try
        {
            results = session.CompareControllers(kinds);
        }
        finally
        {
            Cursor = Cursors.Default;
        }
        ShowResults();
    }

    // Eğrileri üst üste çizer ve tabloyu doldurur.
    private void ShowResults()
    {
        plot.Plot.Clear();
        grid.Rows.Clear();
        for (int k = 0; k < results.Count; k++)
        {
            ComparisonResult r = results[k];
            ScottPlot.Plottables.Scatter line = plot.Plot.Add.ScatterLine(r.TimeS, r.ToolErrorMm);
            line.LegendText = r.Name;
            line.LineWidth = 1.5f;
            object[] row = new object[Columns.Length + 2];
            row[0] = r.Name;
            double[] v = ComparisonCsv.Values(r.Metrics);
            for (int c = 0; c < Columns.Length; c++)
                row[c + 1] = v == null ? "" : Format(v[c]);
            row[Columns.Length + 1] = r.Message;
            grid.Rows.Add(row);
        }
        plot.Plot.ShowLegend(ScottPlot.Alignment.UpperRight);
        plot.Plot.Axes.AutoScale();
        plot.Refresh();
    }

    // NaN yalnız oturma süresinde çıkar: koşu sonunda hata bandın dışındadır.
    private static string Format(double v)
    {
        return double.IsNaN(v) ? "oturmadı" : v.ToString("G4", CultureInfo.CurrentCulture);
    }

    private void btnSaveTable_Click(object sender, EventArgs e)
    {
        if (results.Count == 0 || saveDialog.ShowDialog(this) != DialogResult.OK)
            return;
        try
        {
            using (StreamWriter w = new StreamWriter(saveDialog.FileName, false, new System.Text.UTF8Encoding(false)))
            {
                ComparisonCsv.Write(results, w);
            }
        }
        catch (IOException ex)
        {
            MessageBox.Show(this, "Dosya yazılamadı: " + ex.Message, "CSV kaydet", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
        catch (UnauthorizedAccessException ex)
        {
            MessageBox.Show(this, "Bu klasöre yazma izni yok: " + ex.Message, "CSV kaydet", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
    }
}
