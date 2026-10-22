namespace RobotArmLab.Core.Tasks;

/// <summary>Bir anın eklem referansı: konum q_ref (rad), hız q̇_ref (rad/s) ve ivme q̈_ref (rad/s²), 4'er eleman.</summary>
public class JointReference
{
    /// <summary>Referans eklem açıları, rad.</summary>
    public double[] Q = new double[4];

    /// <summary>Referans eklem hızları, rad/s.</summary>
    public double[] Qd = new double[4];

    /// <summary>Referans eklem ivmeleri, rad/s².</summary>
    public double[] Qdd = new double[4];
}

/// <summary>
/// Kapalı çevrim görevi: her t anında kontrolcünün izleyeceği eklem referansını verir. Görevler: eklem basamağı,
/// eklem uzayında beşinci derece yörünge, Kartezyen hedef (ters kinematik + beşinci derece) ve çember izleme.
/// </summary>
public interface IReferenceTask
{
    /// <summary>Arayüzde gösterilen ad.</summary>
    string Name { get; }

    /// <summary>t (s) anındaki eklem referansı.</summary>
    JointReference At(double t);
}
