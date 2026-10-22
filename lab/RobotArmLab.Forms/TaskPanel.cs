using System;
using System.Windows.Forms;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Session;

namespace RobotArmLab.Forms;

/// <summary>
/// "Görev" sekmesi: görev seçimi ve parametreleri. Açılar derece, konumlar milimetre olarak girilir; oturuma
/// radyan ve metre yazılır. Seçilen göreve göre yalnız ilgili kutular görünür. Çember ve al-bırak ayarları
/// <see cref="CirclePickPanel"/> içindedir.
/// </summary>
public partial class TaskPanel : UserControl
{
    private const double Deg = Math.PI / 180.0;

    // x, y, z kutuları Kartezyende hedefi, çemberde merkezi gösterir. İkisi ayrı tutulur ve görev değişince
    // kutular ilgili noktayla doldurulur; yoksa Kartezyen hedef farkında olmadan çember merkezi olurdu.
    private Point3 cartesianTarget = new Point3(0.0, 0.0, 0.0);
    private Point3 circleCenter = new Point3(0.0, 0.0, 0.0);
    private TaskKind shownTask = TaskKind.JointQuintic;

    /// <summary>Kontrolleri kurar ve görev listesini doldurur.</summary>
    public TaskPanel()
    {
        InitializeComponent();
        cmbTask.Items.AddRange(ClosedLoopBuilder.TaskDisplayNames());
    }

    /// <summary>Oturumdaki görev ayarlarını kontrollere yazar.</summary>
    public void LoadFrom(LabSession session)
    {
        ClosedLoopSettings c = session.Closed;
        cmbTask.SelectedIndex = (int)c.Task;   // değişim olayı eski kutu değerlerini saklar; hemen aşağıda üzerine yazılır
        cartesianTarget = c.CartesianTarget;
        circleCenter = c.CircleCenter;
        shownTask = c.Task;
        NumericBox.Set(numQ1, c.TargetQRad[0] / Deg);
        NumericBox.Set(numQ2, c.TargetQRad[1] / Deg);
        NumericBox.Set(numQ3, c.TargetQRad[2] / Deg);
        NumericBox.Set(numQ4, c.TargetQRad[3] / Deg);
        ShowPoint(c.Task == TaskKind.Circle ? circleCenter : cartesianTarget);
        NumericBox.Set(numPitchDeg, c.PitchRad / Deg);
        chkElbowUp.Checked = c.ElbowUp;
        NumericBox.Set(numStartS, c.StartTimeS);
        NumericBox.Set(numDurationS, c.MotionDurationS);
        circlePickPanel.LoadFrom(c);
        UpdateVisible();
    }

    /// <summary>Kontrollerdeki değerleri oturuma yazar. x, y, z kutuları çemberde merkezi, Kartezyende hedefi verir.</summary>
    public void ApplyTo(LabSession session)
    {
        ClosedLoopSettings c = session.Closed;
        c.Task = (TaskKind)Math.Max(0, cmbTask.SelectedIndex);
        c.TargetQRad = new double[] { (double)numQ1.Value * Deg, (double)numQ2.Value * Deg, (double)numQ3.Value * Deg, (double)numQ4.Value * Deg };
        KeepPoint();
        c.CartesianTarget = cartesianTarget;
        c.CircleCenter = circleCenter;
        c.PitchRad = (double)numPitchDeg.Value * Deg;
        c.ElbowUp = chkElbowUp.Checked;
        c.StartTimeS = (double)numStartS.Value;
        c.MotionDurationS = (double)numDurationS.Value;
        circlePickPanel.ApplyTo(c);
    }

    private void cmbTask_SelectedIndexChanged(object sender, EventArgs e)
    {
        KeepPoint();
        shownTask = (TaskKind)Math.Max(0, cmbTask.SelectedIndex);
        ShowPoint(shownTask == TaskKind.Circle ? circleCenter : cartesianTarget);
        UpdateVisible();
    }

    // Kutulardaki x, y, z (mm) değerini, kutuların o an gösterdiği noktaya (hedef veya merkez) yazar.
    private void KeepPoint()
    {
        Point3 p = new Point3((double)numX.Value / 1000.0, (double)numY.Value / 1000.0, (double)numZ.Value / 1000.0);
        if (shownTask == TaskKind.Circle)
            circleCenter = p;
        else if (shownTask == TaskKind.Cartesian)
            cartesianTarget = p;
    }

    private void ShowPoint(Point3 p)
    {
        NumericBox.Set(numX, p.X * 1000.0);
        NumericBox.Set(numY, p.Y * 1000.0);
        NumericBox.Set(numZ, p.Z * 1000.0);
    }

    // Eklem görevlerinde açılar, Kartezyen ve çemberde x, y, z ve eğim; süre T yalnız noktadan noktaya görevlerde.
    private void UpdateVisible()
    {
        TaskKind task = (TaskKind)Math.Max(0, cmbTask.SelectedIndex);
        bool joint = task == TaskKind.JointStep || task == TaskKind.JointQuintic;
        bool circle = task == TaskKind.Circle;
        lblAngles.Visible = numQ1.Visible = numQ2.Visible = numQ3.Visible = numQ4.Visible = joint;
        lblXyz.Visible = numX.Visible = numY.Visible = numZ.Visible = !joint;
        lblPitch.Visible = numPitchDeg.Visible = chkElbowUp.Visible = !joint;
        lblXyz.Text = circle ? "Çember merkezi x, y, z (mm):" : "Hedef x, y, z (mm):";
        lblDuration.Visible = numDurationS.Visible = task == TaskKind.JointQuintic || task == TaskKind.Cartesian;
        circlePickPanel.ShowCircle(circle);
    }
}
