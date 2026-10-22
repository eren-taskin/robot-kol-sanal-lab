using System;
using System.Collections.Generic;
using RobotArmLab.Core.Kinematics;
using RobotArmLab.Core.Model;
using RobotArmLab.Core.Tasks;

namespace RobotArmLab.Core.Session;

/// <summary>
/// Görevin takım uzayındaki hedefi, 3B görünümde çizilmek için: basamak ve beşinci derece görevlerinde hedef
/// açıların ileri kinematiği, Kartezyen görevde hedef nokta, çemberde 73 noktalık kapalı yol (ilk nokta sonda tekrar).
/// </summary>
public static class TaskGeometry
{
    private const int CircleSegmentCount = 72;   // çember 72 parçalı çokgenle çizilir (5°'de bir nokta)

    /// <summary>Hedef noktaları (m). Tek nokta: hedef; birden çok nokta: izlenecek yol. Ayar geçersizse boş liste.</summary>
    public static List<Point3> Path(ClosedLoopSettings s, ArmParameters parameters)
    {
        List<Point3> points = new List<Point3>();
        if (s.Task == TaskKind.Cartesian)
            points.Add(new Point3(s.CartesianTarget.X, s.CartesianTarget.Y, s.CartesianTarget.Z));
        else if (s.Task == TaskKind.Circle)
        {
            CircleTask circle = new CircleTask(parameters, new double[4], s.CircleCenter, s.CircleRadiusM, s.CirclePlane,
                s.CirclePeriodS, s.PitchRad, s.ElbowUp, s.CircleApproachS, s.CircleRampS);
            for (int k = 0; k <= CircleSegmentCount; k++)   // k = 72 noktası k = 0 ile aynıdır: yol kapanır
                points.Add(circle.PointAt(2.0 * Math.PI * k / CircleSegmentCount));
        }
        else if (s.TargetQRad != null && s.TargetQRad.Length == 4)
            points.Add(ForwardKinematics.ToolPosition(parameters, s.TargetQRad));
        return points;
    }
}
