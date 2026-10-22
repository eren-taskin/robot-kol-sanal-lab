using RobotArmLab.Core.Controllers;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Simulation;
using RobotArmLab.Core.Tasks;

namespace RobotArmLab.Core.Session;

/// <summary>Formda seçilebilen kontrolcüler.</summary>
public enum ControllerKind
{
    /// <summary>Bağımsız eklem PID'i (taban çizgisi).</summary>
    Pid,

    /// <summary>Hesaplanmış tork (CT).</summary>
    ComputedTorque,

    /// <summary>Süper-büküm kayan kip (STSMC).</summary>
    SuperTwisting,

    /// <summary>İleri beslemeli süper-büküm kayan kip (FF-STSMC).</summary>
    FeedForwardSuperTwisting,
}

/// <summary>Formda seçilebilen kapalı çevrim görevleri.</summary>
public enum TaskKind
{
    /// <summary>Eklem basamağı.</summary>
    JointStep,

    /// <summary>Eklem uzayında beşinci derece yörünge.</summary>
    JointQuintic,

    /// <summary>Kartezyen hedef (ters kinematik + beşinci derece).</summary>
    Cartesian,

    /// <summary>Çember izleme.</summary>
    Circle,
}

/// <summary>
/// Kapalı çevrim deneyinin bütün ayarları: kontrolcü ve kazançları, kontrolcünün tahmini yükü, örnekleme süresi,
/// algılayıcı, eyleyici ve görev. Gerçek kol (tesis) LabSession'daki parametre ve yükle kurulur; buradaki tahmini
/// yük yalnız kontrolcünün kendi modeline girer, böylece model uyumsuzluğu (gerçek yük ≠ tahmini yük) denenebilir.
/// </summary>
public class ClosedLoopSettings
{
    /// <summary>Seçilen kontrolcü.</summary>
    public ControllerKind Controller { get; set; } = ControllerKind.ComputedTorque;

    /// <summary>PID kazançları.</summary>
    public PidGains Pid { get; set; } = PidGains.Default();

    /// <summary>Hesaplanmış tork kazançları.</summary>
    public ComputedTorqueGains ComputedTorque { get; set; } = ComputedTorqueGains.Default();

    /// <summary>STSMC kazançları.</summary>
    public SuperTwistingGains SuperTwisting { get; set; } = SuperTwistingGains.Default();

    /// <summary>FF-STSMC kazançları (varsayılanı STSMC ile aynı).</summary>
    public SuperTwistingGains FeedForward { get; set; } = SuperTwistingGains.Default();

    /// <summary>Kontrolcünün modelindeki (tahmini) yük kütlesi, kg. Gerçek yükten farklıysa model yanlıştır.</summary>
    public double EstimatedPayloadKg { get; set; }

    /// <summary>Örnekleme süresi Ts, s (varsayılan 2 ms).</summary>
    public double SamplePeriodS { get; set; } = 0.002;

    /// <summary>Algılayıcı türü.</summary>
    public SensorKind Sensor { get; set; } = SensorKind.Perfect;

    /// <summary>Kodlayıcının tur başına sayımı.</summary>
    public int EncoderCounts { get; set; } = 4096;

    /// <summary>Açı gürültüsünün standart sapması σ, rad.</summary>
    public double NoiseStdDevRad { get; set; }

    /// <summary>Gürültü tohumu.</summary>
    public int NoiseSeed { get; set; } = 1;

    /// <summary>Eyleyici türü.</summary>
    public ActuatorKind Actuator { get; set; } = ActuatorKind.IdealTorque;

    /// <summary>Seçilen görev.</summary>
    public TaskKind Task { get; set; } = TaskKind.JointQuintic;

    /// <summary>Basamak ve beşinci derece görevlerinin hedef eklem açıları, rad.</summary>
    public double[] TargetQRad { get; set; } = new double[] { 0.5, -0.2, 0.2, 0.8 };

    /// <summary>Basamak anı veya hareketin başlama anı t0, s.</summary>
    public double StartTimeS { get; set; } = 0.0;

    /// <summary>Noktadan noktaya hareketin süresi T, s.</summary>
    public double MotionDurationS { get; set; } = 2.0;

    /// <summary>Kartezyen hedef konumu, m.</summary>
    public Point3 CartesianTarget { get; set; } = new Point3(0.2, 0.05, 0.15);

    /// <summary>Kartezyen hedefte ve çemberde takım eğimi ψ, rad.</summary>
    public double PitchRad { get; set; }

    /// <summary>Ters kinematik dirsek dalı (true: yukarı).</summary>
    public bool ElbowUp { get; set; } = true;

    /// <summary>Çemberin merkezi, m.</summary>
    public Point3 CircleCenter { get; set; } = new Point3(0.2, 0.0, 0.15);

    /// <summary>Çemberin yarıçapı, m.</summary>
    public double CircleRadiusM { get; set; } = 0.03;

    /// <summary>Çemberin düzlemi.</summary>
    public CirclePlane CirclePlane { get; set; } = CirclePlane.Vertical;

    /// <summary>Bir turun süresi, s.</summary>
    public double CirclePeriodS { get; set; } = 4.0;

    /// <summary>Çembere yaklaşma süresi, s.</summary>
    public double CircleApproachS { get; set; } = 2.0;

    /// <summary>Çemberde açısal hızın yumuşak başlama süresi, s.</summary>
    public double CircleRampS { get; set; } = 2.0;

    /// <summary>
    /// Al-bırak: açıksa kol yüksüz başlar ve <see cref="PickTimeS"/> anında oturumdaki yük kütlesini kavrar.
    /// Kontrolcünün tahmini yükü (<see cref="EstimatedPayloadKg"/>) değişmez.
    /// </summary>
    public bool PickAndPlace { get; set; }

    /// <summary>Al-bırakta yükün kavrandığı an, s.</summary>
    public double PickTimeS { get; set; } = 1.0;

    /// <summary>Derin kopya: Reset'te kopya kullanılır, form ayarları değiştirse de koşu etkilenmez.</summary>
    public ClosedLoopSettings Clone()
    {
        ClosedLoopSettings c = (ClosedLoopSettings)MemberwiseClone();
        c.Pid = Pid.Clone();
        c.ComputedTorque = ComputedTorque.Clone();
        c.SuperTwisting = SuperTwisting.Clone();
        c.FeedForward = FeedForward.Clone();
        c.TargetQRad = (double[])TargetQRad.Clone();
        c.CartesianTarget = new Point3(CartesianTarget.X, CartesianTarget.Y, CartesianTarget.Z);
        c.CircleCenter = new Point3(CircleCenter.X, CircleCenter.Y, CircleCenter.Z);
        return c;
    }
}
