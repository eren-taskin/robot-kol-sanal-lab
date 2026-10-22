using System;
using System.Collections.Generic;
using System.Windows.Forms;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Session;

namespace RobotArmLab.Forms;

/// <summary>"Ortam ve yük" sekmesi: hazır ortam seçimi, düzenlenebilir g ve yük kütlesi.</summary>
public partial class EnvironmentPanel : UserControl
{
    private readonly List<GravityEnvironment> presets = GravityEnvironment.Presets();

    /// <summary>Kontrolleri kurar ve açılır listeyi beş hazır ortamla doldurur.</summary>
    public EnvironmentPanel()
    {
        InitializeComponent();
        for (int i = 0; i < presets.Count; i++)
            cmbEnvironment.Items.Add(presets[i]);
    }

    /// <summary>Oturumdaki ortamı ve yükü kontrollere yazar.</summary>
    public void LoadFrom(LabSession session)
    {
        for (int i = 0; i < presets.Count; i++)
        {
            if (presets[i].Name == session.Environment.Name)
                cmbEnvironment.SelectedIndex = i;
        }
        NumericBox.Set(numGravity, session.Environment.GravityMps2);
        NumericBox.Set(numPayload, session.Payload.MassKg);
    }

    /// <summary>Kontrollerdeki değerleri oturuma yazar. g kutusu elle değiştirildiyse o değer kullanılır.</summary>
    public void ApplyTo(LabSession session)
    {
        string name = cmbEnvironment.SelectedIndex >= 0 ? presets[cmbEnvironment.SelectedIndex].Name : "Özel";
        session.Environment = new GravityEnvironment(name, (double)numGravity.Value);
        session.Payload = new Payload((double)numPayload.Value);
    }

    // Hazır ortam seçilince g kutusuna o ortamın değeri yazılır; öğrenci sonra bunu değiştirebilir.
    private void cmbEnvironment_SelectedIndexChanged(object sender, EventArgs e)
    {
        if (cmbEnvironment.SelectedIndex >= 0)
            NumericBox.Set(numGravity, presets[cmbEnvironment.SelectedIndex].GravityMps2);
    }
}
