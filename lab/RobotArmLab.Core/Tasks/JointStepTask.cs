namespace RobotArmLab.Core.Tasks;

/// <summary>
/// Eklem basamağı: t0 anına kadar referans başlangıç açısıdır, sonra birden hedef açıya atlar; q̇_ref = q̈_ref = 0.
/// Kontrolcünün en sert sınavıdır: hata ilk anda tam basamak yüksekliğidir, tork genellikle doyuma girer.
/// </summary>
public class JointStepTask : IReferenceTask
{
    private readonly double[] start;
    private readonly double[] target;
    private readonly double stepTimeS;

    /// <summary>Arayüzde gösterilen ad.</summary>
    public string Name { get { return "Eklem basamağı"; } }

    /// <summary>Başlangıç q0 (rad), hedef (rad) ve basamak anı t0 (s) ile görevi kurar.</summary>
    public JointStepTask(double[] start, double[] target, double stepTimeS)
    {
        this.start = (double[])start.Clone();
        this.target = (double[])target.Clone();
        this.stepTimeS = stepTimeS;
    }

    /// <summary>t &lt; t0 ise q0, değilse hedef; hız ve ivme sıfır.</summary>
    public JointReference At(double t)
    {
        JointReference r = new JointReference();
        double[] q = t >= stepTimeS ? target : start;
        for (int i = 0; i < 4; i++)
            r.Q[i] = q[i];
        return r;
    }
}
