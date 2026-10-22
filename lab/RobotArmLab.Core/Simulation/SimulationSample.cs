namespace RobotArmLab.Core.Simulation;

/// <summary>Bir çözücü adımının sonunda kaydedilen örnek: grafikler ve animasyon bunu kullanır.</summary>
public class SimulationSample
{
    /// <summary>Zaman t, s.</summary>
    public double TimeS;

    /// <summary>Eklem açıları q, rad.</summary>
    public double[] Q;

    /// <summary>Eklem hızları q̇, rad/s.</summary>
    public double[] Qd;

    /// <summary>Eklemlere uygulanan (doyumdan sonraki) motor torku τ, N·m.</summary>
    public double[] Tau;

    /// <summary>Referans eklem açıları q_ref, rad. Açık çevrimde null.</summary>
    public double[] QRef;

    /// <summary>Referans eklem hızları q̇_ref, rad/s. Açık çevrimde null.</summary>
    public double[] QdRef;

    /// <summary>Kontrolcünün bu örnekleme aralığındaki tork komutu, N·m (doyumdan önce). Açık çevrimde null.</summary>
    public double[] TauCommand;

    /// <summary>Takım noktasının x konumu, m.</summary>
    public double ToolX;

    /// <summary>Takım noktasının y konumu, m.</summary>
    public double ToolY;

    /// <summary>Takım noktasının z konumu (yükseklik), m.</summary>
    public double ToolZ;

    /// <summary>Takım noktasındaki (gerçek) yük kütlesi, kg. Al-bırak görevinde koşu sırasında değişir.</summary>
    public double PayloadKg;

    /// <summary>Kinetik enerji T = ½ q̇ᵀ M q̇, J.</summary>
    public double KineticEnergyJ;

    /// <summary>Potansiyel enerji V = g Σ m z, J.</summary>
    public double PotentialEnergyJ;

    /// <summary>Toplam mekanik enerji T + V, J. Sürtünmesiz ve torksuz kolda sabit kalmalıdır.</summary>
    public double TotalEnergyJ { get { return KineticEnergyJ + PotentialEnergyJ; } }
}
