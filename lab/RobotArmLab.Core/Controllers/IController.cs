namespace RobotArmLab.Core.Controllers;

/// <summary>
/// Kapalı çevrim kontrolcü arayüzü (uygulamaları W3'te: hesaplanmış tork ve süper-büküm kayan kip).
/// Ölçülen durum ve referans verilince eklem torkunu hesaplar.
/// </summary>
public interface IController
{
    /// <summary>Arayüzde gösterilen ad.</summary>
    string Name { get; }

    /// <summary>
    /// t (s) anında, ölçülen q (rad), q̇ (rad/s) ve referans q_ref (rad), q̇_ref (rad/s), q̈_ref (rad/s²)
    /// için eklem torku τ'yu (N·m, 4 eleman) döndürür.
    /// </summary>
    double[] Compute(double t, double[] q, double[] qd, double[] qRef, double[] qdRef, double[] qddRef);
}
