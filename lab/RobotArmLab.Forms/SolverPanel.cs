using System;
using System.Globalization;
using System.Windows.Forms;
using RobotArmLab.Core.Session;

namespace RobotArmLab.Forms;

/// <summary>
/// "Çözücü" sekmesi: çözücü seçimi, sabit adımlılar için h (ms), Dormand–Prince için rtol/atol ve simülasyon süresi.
/// Seçilen çözücüye göre ilgisiz kutular kapatılır.
/// </summary>
public partial class SolverPanel : UserControl
{
    /// <summary>Kontrolleri kurar ve çözücü listesini doldurur.</summary>
    public SolverPanel()
    {
        InitializeComponent();
        cmbSolver.Items.AddRange(SolverFactory.DisplayNames());
    }

    /// <summary>Oturumdaki çözücü ayarlarını kontrollere yazar.</summary>
    public void LoadFrom(LabSession session)
    {
        cmbSolver.SelectedIndex = (int)session.Solver;
        NumericBox.Set(numStepMs, session.StepSizeS * 1000.0);
        txtRtol.Text = session.RelativeTolerance.ToString("0.###E+0", CultureInfo.InvariantCulture);
        txtAtol.Text = session.AbsoluteTolerance.ToString("0.###E+0", CultureInfo.InvariantCulture);
        NumericBox.Set(numDuration, session.DurationS);
        UpdateEnabled();
    }

    /// <summary>
    /// Kontrollerdeki değerleri oturuma yazar. Tolerans sayı değilse NaN yazılır; oturumun denetimi bunu
    /// Türkçe bir hatayla bildirir.
    /// </summary>
    public void ApplyTo(LabSession session)
    {
        session.Solver = (SolverKind)cmbSolver.SelectedIndex;
        session.StepSizeS = (double)numStepMs.Value / 1000.0;
        session.RelativeTolerance = ParseNumber(txtRtol.Text);
        session.AbsoluteTolerance = ParseNumber(txtAtol.Text);
        session.DurationS = (double)numDuration.Value;
    }

    private void cmbSolver_SelectedIndexChanged(object sender, EventArgs e)
    {
        UpdateEnabled();
    }

    // Sabit adımlı çözücüde h, Dormand–Prince'te rtol ve atol düzenlenebilir.
    private void UpdateEnabled()
    {
        bool fixedStep = SolverFactory.IsFixedStep((SolverKind)Math.Max(0, cmbSolver.SelectedIndex));
        numStepMs.Enabled = fixedStep;
        txtRtol.Enabled = !fixedStep;
        txtAtol.Enabled = !fixedStep;
    }

    // "1e-6" ve "0,001" (Türkçe) ve "0.001" yazımlarının hepsi kabul edilir.
    private static double ParseNumber(string text)
    {
        double value;
        if (double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value))
            return value;
        if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
            return value;
        return double.NaN;
    }
}
