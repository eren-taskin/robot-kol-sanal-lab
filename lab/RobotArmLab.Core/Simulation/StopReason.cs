namespace RobotArmLab.Core.Simulation;

/// <summary>Simülasyonun neden durduğu (SPEC §6.4 hata durumları).</summary>
public enum StopReason
{
    /// <summary>Durmadı, çalışıyor.</summary>
    None,

    /// <summary>Bir eklem açısı sınırını aştı.</summary>
    JointLimit,

    /// <summary>Takım noktası zeminin (z = 0) altına indi.</summary>
    Floor,

    /// <summary>Durumda NaN veya sonsuz değer oluştu (sayısal patlama).</summary>
    NotFinite,
}
