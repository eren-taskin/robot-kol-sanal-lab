using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;

namespace RobotArmLab.Core.Tasks;

/// <summary>
/// Kartezyen hedef: takım noktasının hedef konumu (x, y, z) ve eğimi ψ ters kinematikle eklem açılarına çevrilir,
/// sonra başlangıç açılarından oraya eklem uzayında beşinci derece yörünge kurulur. Takım düz çizgide gitmez;
/// eklemler düzgün hareket eder.
/// </summary>
public static class CartesianTask
{
    /// <summary>
    /// Görevi kurar. Hedef erişilemezse veya eklem sınırları dışındaysa null döner ve error Türkçe nedeni verir.
    /// x, y, z m; pitchRad rad; elbowUp dirsek dalı; t0 ve T s.
    /// </summary>
    public static QuinticTask Create(ArmParameters p, double[] start, Point3 target, double pitchRad, bool elbowUp,
        double startTimeS, double durationS, out string error)
    {
        IkResult ik = InverseKinematics.Solve(p, target.X, target.Y, target.Z, pitchRad, elbowUp, true);
        if (!ik.Success)
        {
            error = "Kartezyen hedef: " + ik.Reason;
            return null;
        }
        error = "";
        QuinticTask task = new QuinticTask(start, ik.Q, startTimeS, durationS);
        task.Name = "Kartezyen hedef (ters kinematik + beşinci derece)";
        return task;
    }
}
