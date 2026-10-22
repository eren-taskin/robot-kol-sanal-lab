using System;
using System.Collections.Generic;
using System.IO;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Core.Session;

/// <summary>
/// Formun arkasındaki laboratuvar oturumu: bütün deney ayarlarını tutar, simülatörü bu ayarlardan kurar,
/// zamanlayıcının her tıkında simülasyonu duvar saatine göre ilerletir ve grafikler için geçmişi saklar.
/// Form yalnız kontrollerdeki değerleri buraya yazar ve sonuçları çizer; bütün mantık buradadır, bu yüzden
/// pencere açmadan test edilebilir.
/// </summary>
public class LabSession
{
    /// <summary>Bir Advance çağrısında işlenecek en uzun duvar saati süresi, s. Zamanlayıcı takılırsa birikmiş süre atılır.</summary>
    public const double MaxWallClockPerCallS = 0.25;

    /// <summary>Bir Advance çağrısında atılacak en çok çözücü adımı. Arayüz donmasın diye iş burada kesilir.</summary>
    public const int MaxStepsPerCall = 20000;

    // Dormand–Prince'te örneklerin (çıktının) alındığı aralık, s. Alt adımları çözücü kendisi seçer.
    private const double DormandPrinceOutputIntervalS = 0.005;

    private OpenLoopSimulator simulator;   // kol (tesis); kapalı çevrimde de bu nesne ilerler
    private ClosedLoopSimulator loop;       // kapalı çevrimde dolu, açık çevrimde null
    private double targetTimeS;   // duvar saatine göre ulaşılması gereken simülasyon zamanı, s

    // ---------------- Ayarlar (form bunları yazar, Reset bunları okur) ----------------

    /// <summary>Kolun fiziksel parametreleri (PropertyGrid bu nesneyi düzenler).</summary>
    public ArmParameters Parameters { get; set; }

    /// <summary>Ortam ve yerçekimi ivmesi.</summary>
    public GravityEnvironment Environment { get; set; }

    /// <summary>Takım noktasındaki yük.</summary>
    public Payload Payload { get; set; }

    /// <summary>Seçilen çözücü.</summary>
    public SolverKind Solver { get; set; }

    /// <summary>Sabit adımlı çözücülerin adım boyu h, s.</summary>
    public double StepSizeS { get; set; }

    /// <summary>Dormand–Prince bağıl toleransı rtol.</summary>
    public double RelativeTolerance { get; set; }

    /// <summary>Dormand–Prince mutlak toleransı atol.</summary>
    public double AbsoluteTolerance { get; set; }

    /// <summary>Giriş türü: tork (N·m) veya gerilim (V).</summary>
    public InputMode Mode { get; set; }

    /// <summary>Her eklemin giriş sinyali (4 eleman).</summary>
    public JointInput[] Inputs { get; set; }

    /// <summary>Başlangıç eklem açıları q0, rad (4 eleman).</summary>
    public double[] InitialQRad { get; set; }

    /// <summary>Başlangıç eklem hızları q̇0, rad/s (4 eleman).</summary>
    public double[] InitialQdRadPerS { get; set; }

    /// <summary>Kapalı çevrim mi (true: kontrolcü görevi izler) yoksa açık çevrim mi (girişler doğrudan uygulanır).</summary>
    public bool ClosedLoop { get; set; }

    /// <summary>Kapalı çevrim ayarları: kontrolcü, tahmini yük, Ts, algılayıcı, eyleyici, görev.</summary>
    public ClosedLoopSettings Closed { get; set; }

    /// <summary>Simülasyonun süresi, s. Bu zamana gelince koşu biter.</summary>
    public double DurationS { get; set; }

    /// <summary>Oynatma hızı çarpanı: 1 gerçek zaman, 2 iki kat hızlı. Reset gerektirmez.</summary>
    public double SpeedFactor { get; set; }

    // ---------------- Sonuçlar ----------------

    /// <summary>Son Reset'ten beri üretilen bütün örnekler (ilki t = 0 anı). Grafikler ve CSV bunu kullanır.</summary>
    public List<SimulationSample> History { get; private set; }

    /// <summary>Simülasyon zamanı t, s.</summary>
    public double TimeS { get { return simulator.TimeS; } }

    /// <summary>Simülasyon süre dolduğu için bitti mi.</summary>
    // Yarım adım payı: zaman stepCount·h ile hesaplandığından son adım DurationS'a tam değil çok yakın düşer.
    public bool IsFinished { get { return TimeS >= DurationS - 0.5 * simulator.StepSizeS; } }

    /// <summary>Simülasyon bir hata yüzünden (sınır, zemin, NaN) durdu mu.</summary>
    public bool IsStopped { get { return simulator.IsStopped; } }

    /// <summary>Son Reset'ten beri hiç ilerlenmedi mi (t = 0). Form, Başlat'ta ayarları bu durumda yeniden uygular.</summary>
    public bool IsAtStart { get { return History.Count <= 1; } }

    /// <summary>Koşu sürdürülebilir mi (ne durdu ne bitti).</summary>
    public bool CanContinue { get { return !IsStopped && !IsFinished; } }

    /// <summary>Durma nedeni (çalışıyorsa None).</summary>
    public StopReason StopReason { get { return simulator.StopReason; } }

    /// <summary>Kullanıcıya gösterilecek durum metni: hata açıklaması, "Süre doldu." veya boş.</summary>
    public string StatusMessage
    {
        get
        {
            if (IsStopped)
                return simulator.StopMessage;
            if (IsFinished)
                return "Süre doldu.";
            return "";
        }
    }

    /// <summary>
    /// Varsayılan ayarlarla oturumu kurar: Dünya, yük yok, RK4, h = 0,5 ms, tork modu, sıfır giriş,
    /// q0 = 0, süre 10 s, hız 1×. Kurucu Reset'i de çağırır, böylece çizilecek bir durum hep vardır.
    /// </summary>
    public LabSession()
    {
        Parameters = ArmParameters.Default();
        Environment = GravityEnvironment.Earth();
        Payload = new Payload(0.0);
        Solver = SolverKind.Rk4;
        StepSizeS = 0.0005;
        RelativeTolerance = 1e-6;
        AbsoluteTolerance = 1e-9;
        Mode = InputMode.Torque;
        Inputs = new JointInput[4];
        for (int i = 0; i < 4; i++)
            Inputs[i] = new JointInput();
        InitialQRad = new double[4];
        InitialQdRadPerS = new double[4];
        Closed = new ClosedLoopSettings();
        DurationS = 10.0;
        SpeedFactor = 1.0;
        Reset();
    }

    /// <summary>Ayarları denetler; hataları Türkçe cümleler olarak döndürür (boş liste = ayarlar geçerli).</summary>
    public List<string> Validate()
    {
        return SettingsValidator.Validate(this);
    }

    /// <summary>
    /// Ayarlar geçerliyse simülatörü sıfırdan kurar, zamanı 0'a alır ve geçmişi temizler; hata listesini
    /// döndürür. Ayarlar geçersizse hiçbir şey değişmez ve istisna atılmaz, yalnız hatalar döner.
    /// Aynı ayarlarla iki Reset aynı koşuyu verir: her seferinde yeni çözücü ve yeni girişler kurulur.
    /// </summary>
    public List<string> Reset()
    {
        List<string> errors = Validate();
        if (errors.Count > 0)
            return errors;

        GravityEnvironment environment = new GravityEnvironment(Environment.Name, Environment.GravityMps2);
        OpenLoopSimulator sim = new OpenLoopSimulator(Parameters.Clone(), environment, new Payload(Payload.MassKg),
            SolverFactory.Create(Solver, RelativeTolerance, AbsoluteTolerance));
        sim.Mode = Mode;
        for (int i = 0; i < 4; i++)
            sim.Inputs[i] = Inputs[i].Clone();   // kopya: form tabloyu değiştirse de koşu etkilenmez
        sim.StepSizeS = EffectiveStepS();
        sim.FrameTimeS = sim.StepSizeS;   // her Step() tek çözücü adımı atar; ilerleme hızını Advance yönetir

        ArmState initial = new ArmState();
        for (int i = 0; i < 4; i++)
        {
            initial.Q[i] = InitialQRad[i];
            initial.Qd[i] = InitialQdRadPerS[i];
        }
        string taskError;   // görev Validate'te denetlendiği için burada boş kalır
        loop = ClosedLoop ? ClosedLoopBuilder.Build(Closed.Clone(), Parameters.Clone(), environment.GravityMps2,
            InitialQRad, sim, out taskError) : null;
        if (loop != null)
            loop.Reset(initial);   // tesisi de sıfırlar; kare süresini Ts yapar
        else
            sim.Reset(initial);

        simulator = sim;
        targetTimeS = 0.0;
        History = new List<SimulationSample>();
        History.Add(loop != null ? loop.CurrentSample() : sim.CurrentSample());
        return errors;
    }

    /// <summary>
    /// Simülasyonu wallClockSeconds × SpeedFactor kadar ilerletir ve yeni örnekleri döndürür (geçmişe de eklenir).
    /// Duvar saati en çok MaxWallClockPerCallS, adım sayısı en çok MaxStepsPerCall ile sınırlanır; iş sınıra
    /// takılırsa geride kalan süre atılır, simülasyon gerçek zamandan yavaş akar ama arayüz donmaz.
    /// </summary>
    public List<SimulationSample> Advance(double wallClockSeconds)
    {
        List<SimulationSample> fresh = new List<SimulationSample>();
        if (!CanContinue || double.IsNaN(wallClockSeconds) || wallClockSeconds <= 0.0)
            return fresh;

        double wall = Math.Min(wallClockSeconds, MaxWallClockPerCallS);
        targetTimeS = Math.Min(targetTimeS + wall * SpeedFactor, DurationS);
        double halfStep = 0.5 * (loop != null ? loop.SamplePeriodS : simulator.StepSizeS);
        int steps = 0;
        while (!simulator.IsStopped && simulator.TimeS < targetTimeS - halfStep && steps < MaxStepsPerCall)
        {
            fresh.AddRange(loop != null ? loop.Step() : simulator.Step());
            steps++;
        }
        if (steps >= MaxStepsPerCall)
            targetTimeS = simulator.TimeS;   // yetişilemeyen süreyi biriktirme

        History.AddRange(fresh);
        return fresh;
    }

    /// <summary>Kolun şu anki eklem noktaları (taban, omuz, dirsek, bilek, takım), m. Çizimler yalnız bunu kullanır.</summary>
    public Point3[] CurrentJointPositions()
    {
        return ForwardKinematics.JointPositions(simulator.Parameters, simulator.State.Q);
    }

    /// <summary>Kolun şu anki eklem açıları q, rad (kopya).</summary>
    public double[] CurrentQRad() { return (double[])simulator.State.Q.Clone(); }

    // Simülatörün kullandığı adım veya çıktı aralığı, s. Kapalı çevrimde Dormand–Prince'in çıktı aralığı Ts'dir.
    private double EffectiveStepS()
    {
        if (SolverFactory.IsFixedStep(Solver))
            return StepSizeS;
        return ClosedLoop && Closed != null ? Closed.SamplePeriodS : DormandPrinceOutputIntervalS;
    }

    /// <summary>Aynı deneyi seçilen her kontrolcüyle baştan sona koşturur; oturum değişmez (bkz. ControllerComparison).</summary>
    public List<ComparisonResult> CompareControllers(List<ControllerKind> kinds) { return ControllerComparison.Run(this, kinds); }

    /// <summary>Geçmişi CSV olarak yazar (nokta ondalık, virgül ayırıcı; bilgisayarın dil ayarından bağımsız).</summary>
    public void WriteCsv(TextWriter writer)
    {
        CsvExporter.Write(History, writer);
    }

    /// <summary>Geçmişi verilen yoldaki dosyaya CSV olarak kaydeder (UTF-8).</summary>
    public void SaveCsv(string path)
    {
        using (StreamWriter writer = new StreamWriter(path, false, new System.Text.UTF8Encoding(false)))
        {
            WriteCsv(writer);
        }
    }
}
