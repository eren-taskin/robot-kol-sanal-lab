namespace RobotArmLab.Core.Tasks;

/// <summary>
/// Eklem uzayında noktadan noktaya beşinci derece (quintic) yörünge. s = (t − t0)/T için
/// q(t) = q0 + (qf − q0)(10s³ − 15s⁴ + 6s⁵). Başta ve sonda hız ve ivme sıfırdır, bu yüzden tork sıçramaz.
/// t0'dan önce q0'da, t0 + T'den sonra qf'de durur.
/// </summary>
public class QuinticTask : IReferenceTask
{
    private readonly double[] start;
    private readonly double[] goal;
    private readonly double startTimeS;
    private readonly double durationS;

    /// <summary>Arayüzde gösterilen ad.</summary>
    public string Name { get; set; }

    /// <summary>Başlangıç q0 (rad), hedef qf (rad), başlama anı t0 (s) ve süre T (s, &gt; 0) ile görevi kurar.</summary>
    public QuinticTask(double[] start, double[] goal, double startTimeS, double durationS)
    {
        this.start = (double[])start.Clone();
        this.goal = (double[])goal.Clone();
        this.startTimeS = startTimeS;
        this.durationS = durationS;
        Name = "Eklem uzayında beşinci derece yörünge";
    }

    /// <summary>Hedef açılar qf, rad (kopya).</summary>
    public double[] Goal { get { return (double[])goal.Clone(); } }

    /// <summary>t anındaki referans: p(s) = 10s³ − 15s⁴ + 6s⁵, ṗ = p'(s)/T, p̈ = p''(s)/T².</summary>
    public JointReference At(double t)
    {
        double T = durationS;
        double s = (t - startTimeS) / T;
        if (s < 0.0) s = 0.0;
        if (s > 1.0) s = 1.0;
        double p = s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
        double pd = 30.0 * s * s * (1.0 - s) * (1.0 - s) / T;
        double pdd = 60.0 * s * (1.0 - s) * (1.0 - 2.0 * s) / (T * T);

        JointReference r = new JointReference();
        for (int i = 0; i < 4; i++)
        {
            double delta = goal[i] - start[i];
            r.Q[i] = start[i] + delta * p;
            r.Qd[i] = delta * pd;
            r.Qdd[i] = delta * pdd;
        }
        return r;
    }
}
