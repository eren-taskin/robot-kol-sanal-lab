namespace RobotArmLab.Core.Simulation;

/// <summary>
/// Kolun anlık durumu: eklem açıları q (rad) ve eklem hızları q̇ (rad/s). Çözücüler 8 elemanlı
/// x = [q1..q4, q̇1..q̇4] vektörüyle çalışır (rapor eş. (21)); bu sınıf ikisi arasında çeviri yapar.
/// </summary>
public class ArmState
{
    /// <summary>Eklem açıları q1..q4, rad.</summary>
    public double[] Q = new double[4];

    /// <summary>Eklem hızları q̇1..q̇4, rad/s.</summary>
    public double[] Qd = new double[4];

    /// <summary>Durumu 8 elemanlı x = [q; q̇] vektörüne yazar.</summary>
    public double[] ToVector()
    {
        double[] x = new double[8];
        for (int i = 0; i < 4; i++)
        {
            x[i] = Q[i];
            x[4 + i] = Qd[i];
        }
        return x;
    }

    /// <summary>8 elemanlı x = [q; q̇] vektöründen bir durum oluşturur.</summary>
    public static ArmState FromVector(double[] x)
    {
        ArmState s = new ArmState();
        for (int i = 0; i < 4; i++)
        {
            s.Q[i] = x[i];
            s.Qd[i] = x[4 + i];
        }
        return s;
    }

    /// <summary>Verilen açılarda (rad) duran (q̇ = 0) bir durum oluşturur.</summary>
    public static ArmState AtRest(double q1, double q2, double q3, double q4)
    {
        ArmState s = new ArmState();
        s.Q[0] = q1;
        s.Q[1] = q2;
        s.Q[2] = q3;
        s.Q[3] = q4;
        return s;
    }
}
