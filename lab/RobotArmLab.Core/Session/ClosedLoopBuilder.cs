using RobotArmLab.Core.Controllers;
using RobotArmLab.Core.Dynamics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Simulation;
using RobotArmLab.Core.Tasks;

namespace RobotArmLab.Core.Session;

/// <summary>Kapalı çevrim ayarlarından kontrolcüyü, görevi, algılayıcıyı ve kapalı çevrim simülatörünü kurar.</summary>
public static class ClosedLoopBuilder
{
    /// <summary>
    /// Kapalı çevrimi kurar. plant gerçek parametre ve gerçek yükle kurulmuş olmalıdır. Görev kurulamazsa
    /// (erişilemeyen hedef) null döner ve error nedeni verir. Kontrolcü ve kazançlar ayarların kopyasıyla kurulur.
    /// </summary>
    public static ClosedLoopSimulator Build(ClosedLoopSettings settings, ArmParameters parameters, double gravityMps2,
        double[] initialQ, OpenLoopSimulator plant, out string error)
    {
        IReferenceTask task = CreateTask(settings, parameters, initialQ, out error);
        if (task == null)
            return null;
        JointSensor sensor = new JointSensor();
        sensor.Kind = settings.Sensor;
        sensor.CountsPerRevolution = settings.EncoderCounts;
        sensor.NoiseStdDevRad = settings.NoiseStdDevRad;
        sensor.Seed = settings.NoiseSeed;
        IController controller = CreateController(settings, parameters, gravityMps2);
        double payloadKg = plant.Payload.MassKg;
        if (settings.PickAndPlace)
            plant.Payload = new Payload(0.0);   // yüksüz başlar; yük PickTimeS'de kavranır
        ClosedLoopSimulator loop = new ClosedLoopSimulator(plant, controller, task, sensor, settings.Actuator, settings.SamplePeriodS);
        if (settings.PickAndPlace)
        {
            loop.PayloadSwitchTimeS = settings.PickTimeS;
            loop.SwitchedPayloadKg = payloadKg;
        }
        return loop;
    }

    /// <summary>Kontrolcülerin açılır listede görünen adları, <see cref="ControllerKind"/> sırasıyla.</summary>
    public static string[] DisplayNames()
    {
        return new string[] { "PID (taban çizgisi)", "Hesaplanmış tork (CT)", "Süper-büküm kayan kip (STSMC)",
            "İleri beslemeli süper-büküm (FF-STSMC)" };
    }

    /// <summary>Görevlerin açılır listede görünen adları, <see cref="TaskKind"/> sırasıyla.</summary>
    public static string[] TaskDisplayNames()
    {
        return new string[] { "Eklem basamağı", "Eklem uzayında beşinci derece", "Kartezyen hedef (ters kinematik)", "Çember izleme" };
    }

    /// <summary>
    /// Seçilen kontrolcüyü kurar. Model tabanlı kontrolcüler kendi ArmDynamics nesnelerini tahmini yükle kurar
    /// (gerçek yük tesiste kalır).
    /// </summary>
    public static IController CreateController(ClosedLoopSettings settings, ArmParameters parameters, double gravityMps2)
    {
        ArmDynamics model = new ArmDynamics(new Payload(settings.EstimatedPayloadKg).ApplyTo(parameters), gravityMps2);
        switch (settings.Controller)
        {
            case ControllerKind.Pid:
                return new PidController(settings.Pid.Clone(), parameters.TorqueLimitNm);
            case ControllerKind.SuperTwisting:
                return new SuperTwistingController(model, settings.SuperTwisting.Clone());
            case ControllerKind.FeedForwardSuperTwisting:
                return new FeedForwardStsmcController(model, settings.FeedForward.Clone());
            default:
                return new ComputedTorqueController(model, settings.ComputedTorque.Clone());
        }
    }

    /// <summary>Seçilen görevi kurar; kurulamazsa null döner ve error Türkçe nedeni verir.</summary>
    public static IReferenceTask CreateTask(ClosedLoopSettings s, ArmParameters parameters, double[] initialQ, out string error)
    {
        error = "";
        switch (s.Task)
        {
            case TaskKind.JointStep:
                return new JointStepTask(initialQ, s.TargetQRad, s.StartTimeS);
            case TaskKind.Cartesian:
                return CartesianTask.Create(parameters, initialQ, s.CartesianTarget, s.PitchRad, s.ElbowUp,
                    s.StartTimeS, s.MotionDurationS, out error);
            case TaskKind.Circle:
                CircleTask circle = new CircleTask(parameters, initialQ, s.CircleCenter, s.CircleRadiusM, s.CirclePlane,
                    s.CirclePeriodS, s.PitchRad, s.ElbowUp, s.CircleApproachS, s.CircleRampS);
                error = circle.Validate();
                return error.Length == 0 ? circle : null;
            default:
                return new QuinticTask(initialQ, s.TargetQRad, s.StartTimeS, s.MotionDurationS);
        }
    }
}
