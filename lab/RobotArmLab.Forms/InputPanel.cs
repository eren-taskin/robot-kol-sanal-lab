using System;
using System.Windows.Forms;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Forms;

/// <summary>"Giriş" sekmesi: tork/gerilim seçimi ve eklem başına giriş sinyali ile başlangıç durumu tablosu.</summary>
public partial class InputPanel : UserControl
{
    /// <summary>Kontrolleri kurar; tablonun sütunlarını InputGridBinder oluşturur.</summary>
    public InputPanel()
    {
        InitializeComponent();
        InputGridBinder.Setup(gridInputs);
    }

    /// <summary>Oturumdaki giriş modunu, sinyalleri ve başlangıç durumunu kontrollere yazar.</summary>
    public void LoadFrom(LabSession session)
    {
        rbVoltage.Checked = session.Mode == InputMode.Voltage;
        rbTorque.Checked = session.Mode == InputMode.Torque;
        InputGridBinder.Load(gridInputs, session);
    }

    /// <summary>Kontrollerdeki değerleri oturuma yazar.</summary>
    public void ApplyTo(LabSession session)
    {
        session.Mode = rbVoltage.Checked ? InputMode.Voltage : InputMode.Torque;
        InputGridBinder.Read(gridInputs, session);
    }

    // Mod değişince genlik sütununun birimi (N·m veya V) güncellenir.
    private void rbMode_CheckedChanged(object sender, EventArgs e)
    {
        InputGridBinder.SetMode(gridInputs, rbVoltage.Checked ? InputMode.Voltage : InputMode.Torque);
    }
}
