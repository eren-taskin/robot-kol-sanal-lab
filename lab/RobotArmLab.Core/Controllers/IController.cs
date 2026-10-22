namespace RobotArmLab.Core.Controllers;

/// <summary>
/// Kapalı çevrim kontrolcü arayüzü. Uygulamaları: PID (taban çizgisi), hesaplanmış tork, süper-büküm kayan kip
/// ve ileri beslemeli süper-büküm kayan kip. Ölçülen durum ve referans verilince eklem torkunu hesaplar.
/// Kontrolcü her örnekleme anında (Ts) bir kez çağrılır; aradaki süre t'lerin farkından bulunur.
/// </summary>
public interface IController
{
    /// <summary>İç durumu (integral, süper-büküm terimi, son örnek zamanı) sıfırlar. Her koşunun başında çağrılır.</summary>
    void Reset();

    /// <summary>
    /// t (s) anında, ölçülen q (rad), q̇ (rad/s) ve referans q_ref (rad), q̇_ref (rad/s), q̈_ref (rad/s²)
    /// için eklem torku τ'yu (N·m, 4 eleman) döndürür.
    /// </summary>
    double[] Compute(double t, double[] q, double[] qd, double[] qRef, double[] qdRef, double[] qddRef);
}
