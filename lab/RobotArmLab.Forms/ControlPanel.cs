using System;
using System.Windows.Forms;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Forms;

/// <summary>
/// "Kontrol" sekmesi: açık/kapalı çevrim seçimi, kontrolcü, kazançlar (PropertyGrid), kontrolcünün tahmini yükü,
/// örnekleme süresi Ts, algılayıcı (kusursuz / kodlayıcı + gürültü) ve eyleyici (ideal tork / motor).
/// Kazanç tablosu oturumdaki kazanç nesnesini doğrudan düzenler (Kol parametreleri sekmesindeki gibi).
/// </summary>
public partial class ControlPanel : UserControl
{
    private ClosedLoopSettings settings;   // LoadFrom'da verilen oturumun kapalı çevrim ayarları

    /// <summary>Kontrolleri kurar ve kontrolcü listesini doldurur.</summary>
    public ControlPanel()
    {
        InitializeComponent();
        cmbController.Items.AddRange(ClosedLoopBuilder.DisplayNames());
    }

    /// <summary>Oturumdaki kapalı çevrim ayarlarını kontrollere yazar.</summary>
    public void LoadFrom(LabSession session)
    {
        settings = session.Closed;
        rbClosed.Checked = session.ClosedLoop;
        rbOpen.Checked = !session.ClosedLoop;
        cmbController.SelectedIndex = (int)settings.Controller;
        NumericBox.Set(numEstimatedPayload, settings.EstimatedPayloadKg);
        NumericBox.Set(numSamplePeriodMs, settings.SamplePeriodS * 1000.0);
        cmbSensor.SelectedIndex = (int)settings.Sensor;
        NumericBox.Set(numCounts, settings.EncoderCounts);
        NumericBox.Set(numNoiseMrad, settings.NoiseStdDevRad * 1000.0);
        NumericBox.Set(numSeed, settings.NoiseSeed);
        cmbActuator.SelectedIndex = (int)settings.Actuator;
        ShowGains();
    }

    /// <summary>Kontrollerdeki değerleri oturuma yazar (kazançlar zaten PropertyGrid'de düzenlenmiştir).</summary>
    public void ApplyTo(LabSession session)
    {
        session.ClosedLoop = rbClosed.Checked;
        ClosedLoopSettings c = session.Closed;
        c.Controller = (ControllerKind)Math.Max(0, cmbController.SelectedIndex);
        c.EstimatedPayloadKg = (double)numEstimatedPayload.Value;
        c.SamplePeriodS = (double)numSamplePeriodMs.Value / 1000.0;
        c.Sensor = (SensorKind)Math.Max(0, cmbSensor.SelectedIndex);
        c.EncoderCounts = (int)numCounts.Value;
        c.NoiseStdDevRad = (double)numNoiseMrad.Value / 1000.0;
        c.NoiseSeed = (int)numSeed.Value;
        c.Actuator = (ActuatorKind)Math.Max(0, cmbActuator.SelectedIndex);
    }

    private void cmbController_SelectedIndexChanged(object sender, EventArgs e)
    {
        ShowGains();
    }

    // Motor seçilince h ≤ 0,2 ms notu görünür.
    private void cmbActuator_SelectedIndexChanged(object sender, EventArgs e)
    {
        lblMotorNote.Visible = cmbActuator.SelectedIndex == (int)ActuatorKind.Motor;
    }

    // Seçilen kontrolcünün kazanç nesnesi PropertyGrid'e verilir.
    private void ShowGains()
    {
        if (settings == null || cmbController.SelectedIndex < 0)
            return;
        switch ((ControllerKind)cmbController.SelectedIndex)
        {
            case ControllerKind.Pid:
                gridGains.SelectedObject = settings.Pid;
                break;
            case ControllerKind.SuperTwisting:
                gridGains.SelectedObject = settings.SuperTwisting;
                break;
            case ControllerKind.FeedForwardSuperTwisting:
                gridGains.SelectedObject = settings.FeedForward;
                break;
            default:
                gridGains.SelectedObject = settings.ComputedTorque;
                break;
        }
    }
}
