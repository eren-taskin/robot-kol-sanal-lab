using System.Collections.Generic;
using RobotArmLab.Core.Controllers;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Motor;
using RobotArmLab.Core.Tasks;

namespace RobotArmLab.Core.Simulation;

/// <summary>
/// Örneklemeli kapalı çevrim: kontrolcü her Ts'de bir kez çalışır, komutu sıfırıncı dereceden tutucuyla (ZOH) Ts
/// boyunca sabit kalır, tesis (kol) bu arada seçilen ADD çözücüsüyle h adımlarıyla ilerler.
/// </summary>
/// <remarks>
/// Tesis olarak açık çevrim simülatörü kullanılır: her eklemin girişi sabit (Constant) bir sinyaldir ve kontrolcü
/// her örnekte yalnız bu sabitin değerini değiştirir. ZOH tam olarak budur. İdeal tork eyleyicisinde giriş N·m
/// (tork modu), motor eyleyicisinde V'tur (gerilim modu): tork Ts boyunca gerçek eklem hızıyla DC motor modelinden
/// hesaplanır, zıt EMK ve 12 V doyumu tesiste kalır. Ts, sabit adımlı çözücülerde h'nin tam katı olmalıdır
/// (ayar denetimi bunu bildirir); Dormand–Prince'te her Ts bir çıktı aralığıdır.
///
/// Motor eyleyicisi zıt EMK sönümüyle (b ≈ 0,85 N·m·s/rad) sistemi çok sertleştirir: hafif bilekte özdeğer
/// b/M ≈ 5600 1/s'dir. h = 0,5 ms'de RK4 tam kararlılık sınırında kalır (hλ ≈ −2,8) ve sahte bir denge noktasına
/// takılır; bu yüzden ayar denetimi motor eyleyicisinde h ≤ 0,2 ms ister.
///
/// Bir Step(): (1) algılayıcı ölçer, (2) görev referansı verir, (3) kontrolcü torku hesaplar, (4) motor seçiliyse
/// tork gerilime çevrilir, (5) tesis Ts kadar ilerler. Örneklere referans ve tork komutu eklenir.
/// </remarks>
public class ClosedLoopSimulator
{
    private readonly DcMotor motor;
    private readonly double[] qMeasured = new double[4];
    private readonly double[] qdMeasured = new double[4];
    private readonly double initialPayloadKg;   // Reset'te tesise geri yüklenen (al-bırak öncesi) yük
    private double[] lastCommand = new double[4];

    /// <summary>Tesis: gerçek parametreler ve gerçek yükle kurulmuş açık çevrim simülatörü.</summary>
    public OpenLoopSimulator Plant { get; }

    /// <summary>Kontrolcü (kendi tahmini modeliyle).</summary>
    public IController Controller { get; }

    /// <summary>İzlenen görev.</summary>
    public IReferenceTask Task { get; }

    /// <summary>Eklem algılayıcısı.</summary>
    public JointSensor Sensor { get; }

    /// <summary>Eyleyici türü (ideal tork veya motor).</summary>
    public ActuatorKind Actuator { get; }

    /// <summary>Örnekleme süresi Ts, s.</summary>
    public double SamplePeriodS { get; }

    /// <summary>
    /// Al-bırak: gerçek yükün değiştiği an, s (varsayılan +∞ = değişmez). Bu andan sonraki ilk örnekte tesisin yükü
    /// <see cref="SwitchedPayloadKg"/> olur; kontrolcünün tahmini yükü aynı kalır.
    /// </summary>
    public double PayloadSwitchTimeS { get; set; } = double.PositiveInfinity;

    /// <summary>Al-bırak anından sonraki gerçek yük kütlesi, kg.</summary>
    public double SwitchedPayloadKg { get; set; }

    /// <summary>
    /// Kapalı çevrimi kurar. plant'in StepSizeS'i (h) önceden ayarlanmış olmalıdır; FrameTimeS burada Ts yapılır.
    /// Kurucu Reset çağırmaz; koşudan önce <see cref="Reset"/> çağrılmalıdır.
    /// </summary>
    public ClosedLoopSimulator(OpenLoopSimulator plant, IController controller, IReferenceTask task, JointSensor sensor,
        ActuatorKind actuator, double samplePeriodS)
    {
        Plant = plant;
        Controller = controller;
        Task = task;
        Sensor = sensor;
        Actuator = actuator;
        SamplePeriodS = samplePeriodS;
        motor = DcMotor.Xm430W350(plant.Parameters.GearRatio);
        initialPayloadKg = plant.Payload.MassKg;
    }

    /// <summary>Tesisi verilen durumdan başlatır; kontrolcüyü, algılayıcıyı ve tutulan komutu sıfırlar.</summary>
    public void Reset(ArmState initial)
    {
        Plant.Mode = Actuator == ActuatorKind.Motor ? InputMode.Voltage : InputMode.Torque;
        for (int i = 0; i < 4; i++)
            Plant.Inputs[i] = new JointInput();   // sabit 0; Step her örnekte genliği yazar
        Plant.FrameTimeS = SamplePeriodS;
        Plant.Payload = new Payload(initialPayloadKg);
        Plant.Reset(initial);
        Controller.Reset();
        Sensor.Reset();
        lastCommand = new double[4];
    }

    /// <summary>Bir örnekleme süresi Ts kadar ilerler ve bu aralıktaki tesis örneklerini döndürür.</summary>
    public List<SimulationSample> Step()
    {
        double t = Plant.TimeS;
        if (t >= PayloadSwitchTimeS - 1e-9 && Plant.Payload.MassKg != SwitchedPayloadKg)
            Plant.ChangePayload(SwitchedPayloadKg);   // yük kavrandı
        Sensor.Measure(t, Plant.State.Q, Plant.State.Qd, qMeasured, qdMeasured);
        JointReference r = Task.At(t);
        double[] tau = Controller.Compute(t, qMeasured, qdMeasured, r.Q, r.Qd, r.Qdd);
        for (int i = 0; i < 4; i++)
        {
            double u = Actuator == ActuatorKind.Motor ? motor.VoltageFor(tau[i], qdMeasured[i]) : tau[i];
            Plant.Inputs[i].Amplitude = u;
        }
        lastCommand = (double[])tau.Clone();

        List<SimulationSample> samples = Plant.Step();
        for (int n = 0; n < samples.Count; n++)
            Annotate(samples[n]);
        return samples;
    }

    /// <summary>Şu anki durumun örneği (referans ve son tork komutuyla).</summary>
    public SimulationSample CurrentSample()
    {
        SimulationSample s = Plant.CurrentSample();
        Annotate(s);
        return s;
    }

    private void Annotate(SimulationSample s)
    {
        JointReference r = Task.At(s.TimeS);
        s.QRef = r.Q;
        s.QdRef = r.Qd;
        s.TauCommand = (double[])lastCommand.Clone();
    }
}
