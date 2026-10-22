using System;
using System.Collections.Generic;
using RobotArmLab.Core.Dynamics;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Motor;
using RobotArmLab.Core.Solvers;

namespace RobotArmLab.Core.Simulation;

/// <summary>
/// Açık çevrim simülatörü: kola tork veya motor gerilimi uygulanır, kolun nereye gittiği izlenir; geri besleme
/// yoktur. Durum denklemi ẋ = [q̇; M⁻¹(τ − c − g − τf)] (rapor eş. (21)) seçilen çözücüyle ilerletilir.
/// Her Step() çağrısı bir ekran karesi (FrameTimeS) kadar ilerler ve o karedeki örnekleri döndürür.
/// </summary>
public class OpenLoopSimulator
{
    private const double DefaultStepS = 0.0005;   // 0,5 ms: RK4 ile kolun en hızlı (bilek) hareketini rahatça çözer
    private const double DefaultFrameS = 0.02;    // 20 ms = saniyede 50 ekran karesi

    private ArmDynamics dynamics;   // yük katılmış parametrelerle kurulan model
    private DcMotor motor;
    private long stepCount;         // atılan çözücü adımı sayısı; zaman = stepCount · h

    /// <summary>Kolun gerçek parametreleri (yüksüz). Değiştirilirse Reset çağrılmalıdır.</summary>
    public ArmParameters Parameters { get; set; }

    /// <summary>Ortam (yerçekimi). Değiştirilirse Reset çağrılmalıdır.</summary>
    public GravityEnvironment Environment { get; set; }

    /// <summary>Takım noktasındaki yük. Değiştirilirse Reset çağrılmalıdır.</summary>
    public Payload Payload { get; set; }

    /// <summary>Kullanılan ADD çözücüsü (Euler, Heun, RK4 veya Dormand–Prince).</summary>
    public IOdeSolver Solver { get; set; }

    /// <summary>Giriş türü: tork (N·m) veya gerilim (V).</summary>
    public InputMode Mode { get; set; }

    /// <summary>Her eklemin giriş sinyali (4 eleman).</summary>
    public JointInput[] Inputs { get; set; }

    /// <summary>Çözücü adım boyu h, s (varsayılan 0,5 ms). Dormand–Prince için çıktı aralığıdır.</summary>
    public double StepSizeS { get; set; }

    /// <summary>Bir ekran karesinin süresi, s (varsayılan 20 ms = 50 kare/s).</summary>
    public double FrameTimeS { get; set; }

    /// <summary>Simülasyon zamanı t, s.</summary>
    public double TimeS { get { return stepCount * StepSizeS; } }

    /// <summary>Kolun şu anki durumu.</summary>
    public ArmState State { get; private set; }

    /// <summary>Durma nedeni (çalışıyorsa None).</summary>
    public StopReason StopReason { get; private set; }

    /// <summary>Durma nedeninin kullanıcıya gösterilecek Türkçe açıklaması.</summary>
    public string StopMessage { get; private set; }

    /// <summary>Simülasyon durdu mu (hata nedeniyle).</summary>
    public bool IsStopped { get { return StopReason != StopReason.None; } }

    /// <summary>
    /// Simülatörü kurar ve kolu q = 0'da durur hâlde başlatır. Varsayılan: tork modu, bütün girişler 0,
    /// h = 0,5 ms, kare = 20 ms.
    /// </summary>
    public OpenLoopSimulator(ArmParameters parameters, GravityEnvironment environment, Payload payload, IOdeSolver solver)
    {
        Parameters = parameters;
        Environment = environment;
        Payload = payload;
        Solver = solver;
        Mode = InputMode.Torque;
        Inputs = new JointInput[4];
        for (int i = 0; i < 4; i++)
            Inputs[i] = new JointInput();
        StepSizeS = DefaultStepS;
        FrameTimeS = DefaultFrameS;
        Reset(ArmState.AtRest(0.0, 0.0, 0.0, 0.0));
    }

    /// <summary>
    /// Zamanı sıfırlar, kolu verilen durumdan başlatır ve modeli parametre, ortam ve yükten yeniden kurar.
    /// Yük burada uzuv 4'e katılır.
    /// </summary>
    public void Reset(ArmState initial)
    {
        ArmParameters withPayload = Payload.ApplyTo(Parameters);
        dynamics = new ArmDynamics(withPayload, Environment.GravityMps2);
        motor = DcMotor.Xm430W350(Parameters.GearRatio);
        State = ArmState.FromVector(initial.ToVector());
        stepCount = 0;
        StopReason = StopReason.None;
        StopMessage = "";
    }

    /// <summary>
    /// Yükü koşu sırasında değiştirir (al-bırak görevi): model yeni yükle yeniden kurulur, durum ve zaman korunur.
    /// Kol yükü anında kavramış gibi davranır; kontrolcünün tahmini yükü bundan etkilenmez.
    /// </summary>
    public void ChangePayload(double massKg)
    {
        Payload = new Payload(massKg);
        dynamics = new ArmDynamics(Payload.ApplyTo(Parameters), Environment.GravityMps2);
    }

    /// <summary>
    /// Bir ekran karesi kadar ilerler: FrameTimeS / StepSizeS çözücü adımı atar, her adımdan sonra hata
    /// durumlarını denetler. Her adımın sonundaki örneği listeye ekler. Hata olursa durur ve nedeni kaydeder.
    /// </summary>
    public List<SimulationSample> Step()
    {
        List<SimulationSample> samples = new List<SimulationSample>();
        int stepsPerFrame = Math.Max(1, (int)Math.Round(FrameTimeS / StepSizeS));
        for (int n = 0; n < stepsPerFrame && !IsStopped; n++)
        {
            double[] x = State.ToVector();
            try
            {
                x = Solver.Step(Derivatives, TimeS, x, StepSizeS);
            }
            catch (InvalidOperationException ex)
            {
                Stop(StopReason.NotFinite, "Sayısal hata: " + ex.Message);
                break;
            }
            stepCount++;
            State = ArmState.FromVector(x);
            CheckFaults();
            samples.Add(CurrentSample());
        }
        return samples;
    }

    /// <summary>Şu anki durumdan bir örnek oluşturur (zaman, açı, hız, tork, takım konumu, enerji).</summary>
    public SimulationSample CurrentSample()
    {
        SimulationSample s = new SimulationSample();
        s.TimeS = TimeS;
        s.Q = (double[])State.Q.Clone();
        s.Qd = (double[])State.Qd.Clone();
        s.Tau = AppliedTorque(TimeS, State.Qd);
        Point3 tool = ForwardKinematics.ToolPosition(dynamics.Parameters, State.Q);
        s.ToolX = tool.X;
        s.ToolY = tool.Y;
        s.ToolZ = tool.Z;
        s.KineticEnergyJ = dynamics.KineticEnergy(State.Q, State.Qd);
        s.PotentialEnergyJ = dynamics.PotentialEnergy(State.Q);
        s.PayloadKg = Payload.MassKg;
        return s;
    }

    /// <summary>
    /// Durum denkleminin sağ tarafı f(t, x) = [q̇; q̈], rapor eş. (21). Çözücü bunu her aşamada çağırır.
    /// Gerilim modunda tork q̇'ya bağlı olduğundan tork her çağrıda yeniden hesaplanır.
    /// </summary>
    public double[] Derivatives(double t, double[] x)
    {
        double[] dx = new double[8];
        for (int i = 0; i < 8; i++)
        {
            if (double.IsNaN(x[i]) || double.IsInfinity(x[i]))
            {
                for (int j = 0; j < 8; j++)
                    dx[j] = double.NaN;
                return dx;
            }
        }

        ArmState s = ArmState.FromVector(x);
        double[] tau = AppliedTorque(t, s.Qd);
        double[] qdd = dynamics.Acceleration(s.Q, s.Qd, tau);
        for (int i = 0; i < 4; i++)
        {
            dx[i] = s.Qd[i];
            dx[4 + i] = qdd[i];
        }
        return dx;
    }

    /// <summary>
    /// t anında eklemlere uygulanan tork, N·m. Tork modunda giriş doğrudan ±τmax'a kırpılır; gerilim modunda
    /// DC motor modeli gerilim ve eklem hızından torku hesaplar, sonra kırpar (doyum fiziktir, tesiste uygulanır).
    /// </summary>
    public double[] AppliedTorque(double t, double[] qd)
    {
        double limit = Parameters.TorqueLimitNm;
        double[] tau = new double[4];
        for (int i = 0; i < 4; i++)
        {
            double u = Inputs[i].Value(t);
            if (Mode == InputMode.Voltage)
                tau[i] = motor.Torque(u, qd[i], limit);
            else
                tau[i] = Math.Max(-limit, Math.Min(limit, u));
        }
        return tau;
    }

    // SPEC §6.4: sayısal patlama, eklem sınırı aşımı, takım noktasının zemine inmesi.
    private void CheckFaults()
    {
        double[] x = State.ToVector();
        for (int i = 0; i < 8; i++)
        {
            if (double.IsNaN(x[i]) || double.IsInfinity(x[i]))
            {
                Stop(StopReason.NotFinite, "Durumda NaN veya sonsuz değer oluştu; adım boyunu küçültün.");
                return;
            }
        }
        for (int i = 0; i < 4; i++)
        {
            if (State.Q[i] < Parameters.LowerLimitRad[i] || State.Q[i] > Parameters.UpperLimitRad[i])
            {
                Stop(StopReason.JointLimit, "Eklem " + (i + 1) + " sınırını aştı (t = " + TimeS.ToString("0.000") + " s).");
                return;
            }
        }
        Point3 tool = ForwardKinematics.ToolPosition(dynamics.Parameters, State.Q);
        if (tool.Z < 0.0)
            Stop(StopReason.Floor, "Takım noktası zemine çarptı (t = " + TimeS.ToString("0.000") + " s).");
    }

    private void Stop(StopReason reason, string message)
    {
        StopReason = reason;
        StopMessage = message;
    }
}
