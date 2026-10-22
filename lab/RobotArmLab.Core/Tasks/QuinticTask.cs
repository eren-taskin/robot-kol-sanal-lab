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

    /// <summary>Başlangıç q0 (rad), hedef qf (rad), başlama anı t0 (s) ve süre T (s, &gt; 0) ile görevi kurar.</summary>
    public QuinticTask(double[] start, double[] goal, double startTimeS, double durationS)
    {
        this.start = (double[])start.Clone();
        this.goal = (double[])goal.Clone();
        this.startTimeS = startTimeS;
        this.durationS = durationS;
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
        double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
        double p = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
        double pd = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T;           // dp/ds = 30s² − 60s³ + 30s⁴, ds/dt = 1/T
        double pdd = (60.0 * s - 180.0 * s2 + 120.0 * s3) / (T * T);   // d²p/ds² = 60s − 180s² + 120s³

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
