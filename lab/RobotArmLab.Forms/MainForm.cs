using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows.Forms;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Forms;

/// <summary>
/// Ana pencere. Yalnız bağlantı (wiring) yapar: kontrollerdeki değerleri LabSession'a yazar, düğmelere basılınca
/// oturumun metotlarını çağırır ve sonuçları çizer. Fizik, çözücü, zamanlama ve denetim LabSession'dadır.
/// </summary>
public partial class MainForm : Form
{
    private static readonly double[] Speeds = { 0.25, 0.5, 1.0, 2.0, 4.0 };   // cmbSpeed sırasıyla

    private readonly LabSession session = new LabSession();
    private readonly Stopwatch clock = new Stopwatch();   // iki zamanlayıcı tıkı arasında geçen gerçek süre
    private PlotPanel plots;
    private ComparePanel comparePanel;
    private ArmView3D view3D;   // 3B görünüm açılamadıysa null
    private string viewNote = "";   // 3B görünüm açılamadıysa nedeni (durum etiketinde gösterilir)

    /// <summary>Pencereyi kurar, çalışma anında eklenen görünümleri oluşturur ve kontrolleri oturumdan doldurur.</summary>
    public MainForm()
    {
        InitializeComponent();
        CreateRuntimeViews();
        cmbSpeed.SelectedIndex = 2;   // 1×
        LoadControlsFromSession();
        plots.Append(session.History);
        ShowState();
    }

    // Grafikler ve 3B görünüm tasarımcıda değil burada eklenir: tasarımcı bu kütüphaneleri yüklemek zorunda kalmaz
    // ve 3B görünüm açılamazsa (ör. eksik WPF bileşeni) yalnız o panel gizlenir, program çalışmaya devam eder.
    private void CreateRuntimeViews()
    {
        plots = new PlotPanel();
        plots.Dock = DockStyle.Fill;
        plotHost.Controls.Add(plots);
        comparePanel = new ComparePanel();
        comparePanel.PrepareSession = ApplyControlsToSession;
        plots.AddPage("Karşılaştır", comparePanel);
        try
        {
            view3D = new ArmView3D();
            view3D.Dock = DockStyle.Fill;
            view3DHost.Controls.Add(view3D);
        }
        catch (Exception ex)
        {
            view3D = null;
            splitViews.Panel1Collapsed = true;
            viewNote = "3B görünüm açılamadı, yalnız 2B görünüm kullanılıyor: " + ex.Message;
        }
    }

    // Oturumdaki ayarları sekmelerdeki kontrollere yazar. PropertyGrid parametre nesnesini doğrudan düzenler.
    private void LoadControlsFromSession()
    {
        propertyGrid.SelectedObject = session.Parameters;
        environmentPanel.LoadFrom(session);
        solverPanel.LoadFrom(session);
        inputPanel.LoadFrom(session);
        controlPanel.LoadFrom(session);
        taskPanel.LoadFrom(session);
    }

    // Bütün sekmelerdeki değerleri oturuma yazar ve oturumu döndürür (Karşılaştır paneli de bunu çağırır).
    private LabSession ApplyControlsToSession()
    {
        environmentPanel.ApplyTo(session);
        solverPanel.ApplyTo(session);
        inputPanel.ApplyTo(session);
        controlPanel.ApplyTo(session);
        taskPanel.ApplyTo(session);
        return session;
    }

    // Kontrollerdeki değerleri oturuma yazar ve simülasyonu baştan kurar. Ayar hatalıysa hataları gösterir.
    private bool ResetFromControls()
    {
        ApplyControlsToSession();
        List<string> errors = session.Reset();
        if (errors.Count > 0)
        {
            MessageBox.Show(this, string.Join(Environment.NewLine, errors), "Ayarlar geçersiz",
                MessageBoxButtons.OK, MessageBoxIcon.Warning);
            lblStatus.Text = errors[0];
            return false;
        }
        plots.Parameters = session.Parameters;
        plots.SetClosedLoop(session.ClosedLoop);
        plots.Clear();
        plots.Append(session.History);
        if (view3D != null)
            view3D.SetTarget(session.ClosedLoop ? TaskGeometry.Path(session.Closed, session.Parameters) : new List<Point3>());
        ShowState();
        return true;
    }

    // Başlat: koşu yeni başlıyorsa veya bitmişse önce ayarlar uygulanır; duraklatılmış koşu kaldığı yerden sürer.
    private void btnStart_Click(object sender, EventArgs e)
    {
        if (session.IsAtStart || !session.CanContinue)
        {
            if (!ResetFromControls())
                return;
        }
        clock.Restart();
        timer.Start();
    }

    private void btnPause_Click(object sender, EventArgs e)
    {
        timer.Stop();
    }

    private void btnReset_Click(object sender, EventArgs e)
    {
        timer.Stop();
        ResetFromControls();
    }

    // Zamanlayıcı tıkı: geçen gerçek süre kadar simülasyon ilerletilir, yeni örnekler grafiğe eklenir, ekran yenilenir.
    // Bir while döngüsü kullanılmaz: döngü pencerenin mesaj kuyruğunu kilitler, düğmeler ve çizim donardı.
    private void timer_Tick(object sender, EventArgs e)
    {
        double wallSeconds = clock.Elapsed.TotalSeconds;
        clock.Restart();
        List<SimulationSample> fresh = session.Advance(wallSeconds);
        plots.Append(fresh);
        ShowState();
        if (!session.CanContinue)
            timer.Stop();
    }

    // Kolun pozunu iki görünüme, zamanı ve durma nedenini araç çubuğuna yazar, görünen grafiği yeniler.
    private void ShowState()
    {
        Point3[] points = session.CurrentJointPositions();
        double payloadKg = session.History[session.History.Count - 1].PayloadKg;   // al-bırakta koşu sırasında değişir
        armView2D.SetPose(points, session.CurrentQRad()[0], payloadKg);
        if (view3D != null)
            view3D.SetPose(points, payloadKg);
        plots.RefreshVisible();
        lblSimTime.Text = "t = " + session.TimeS.ToString("0.000", CultureInfo.CurrentCulture) + " s";
        lblStatus.Text = session.StatusMessage.Length > 0 ? session.StatusMessage : viewNote;
    }

    // Hız değişikliği hemen uygulanır; Sıfırla gerekmez.
    private void cmbSpeed_SelectedIndexChanged(object sender, EventArgs e)
    {
        if (cmbSpeed.SelectedIndex >= 0)
            session.SpeedFactor = Speeds[cmbSpeed.SelectedIndex];
    }

    private void btnDefaults_Click(object sender, EventArgs e)
    {
        session.Parameters = ArmParameters.Default();
        propertyGrid.SelectedObject = session.Parameters;
    }

    private void btnSaveCsv_Click(object sender, EventArgs e)
    {
        if (saveCsvDialog.ShowDialog(this) != DialogResult.OK)
            return;
        try
        {
            session.SaveCsv(saveCsvDialog.FileName);
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

    private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
    {
        timer.Stop();
    }
}
