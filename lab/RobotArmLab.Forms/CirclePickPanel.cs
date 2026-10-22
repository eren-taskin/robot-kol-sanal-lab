using System;
using System.Windows.Forms;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Tasks;

namespace RobotArmLab.Forms;

/// <summary>
/// Görev sekmesinin alt bölümü: çember ayarları (yarıçap mm, tur süresi, düzlem, yaklaşma ve yumuşak başlama süreleri)
/// ve al-bırak (yükün kavranacağı an). Tasarımcı dosyası 250 satırı aşmasın diye TaskPanel'den ayrılmıştır.
/// </summary>
public partial class CirclePickPanel : UserControl
{
    /// <summary>Kontrolleri kurar.</summary>
    public CirclePickPanel()
    {
        InitializeComponent();
    }

    /// <summary>Ayarlardaki çember ve al-bırak değerlerini kontrollere yazar.</summary>
    public void LoadFrom(ClosedLoopSettings c)
    {
        NumericBox.Set(numRadiusMm, c.CircleRadiusM * 1000.0);
        NumericBox.Set(numPeriodS, c.CirclePeriodS);
        cmbPlane.SelectedIndex = (int)c.CirclePlane;
        NumericBox.Set(numApproachS, c.CircleApproachS);
        NumericBox.Set(numRampS, c.CircleRampS);
        chkPick.Checked = c.PickAndPlace;
        NumericBox.Set(numPickS, c.PickTimeS);
    }

    /// <summary>Kontrollerdeki değerleri ayarlara yazar.</summary>
    public void ApplyTo(ClosedLoopSettings c)
    {
        c.CircleRadiusM = (double)numRadiusMm.Value / 1000.0;
        c.CirclePeriodS = (double)numPeriodS.Value;
        c.CirclePlane = (CirclePlane)Math.Max(0, cmbPlane.SelectedIndex);
        c.CircleApproachS = (double)numApproachS.Value;
        c.CircleRampS = (double)numRampS.Value;
        c.PickAndPlace = chkPick.Checked;
        c.PickTimeS = (double)numPickS.Value;
    }

    /// <summary>Çember kutularını gösterir veya gizler; al-bırak her görevde görünür.</summary>
    public void ShowCircle(bool visible)
    {
        lblRadius.Visible = numRadiusMm.Visible = lblPeriod.Visible = numPeriodS.Visible = visible;
        lblPlane.Visible = cmbPlane.Visible = lblApproach.Visible = numApproachS.Visible = visible;
        lblRamp.Visible = numRampS.Visible = visible;
    }
}
