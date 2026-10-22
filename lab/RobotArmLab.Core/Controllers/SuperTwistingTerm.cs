using System;

namespace RobotArmLab.Core.Controllers;

/// <summary>
/// Süper-büküm algoritmasının geri besleme terimi (Levant 1998, Automatica 34(3)), eklem başına:
/// u_st = k1·|s|^½·sign(s) + v,  v̇ = k2·sign(s). STSMC ve FF-STSMC bu sınıfı ortak kullanır.
/// </summary>
/// <remarks>
/// v örnek başına bir kez dikdörtgen kuralıyla toplanır: v ← v + Δt·k2·sign(s). Önce aday değer hesaplanır;
/// kontrolcü çıkışı kırpılmadıysa <see cref="Commit"/> ile kabul edilir (sargı önleme, eski çekirdekle aynı).
/// Sürekli zamanda s ve ṡ sonlu sürede sıfıra iner; örneklemeli uygulamada |s| = O(Ts²) bandında kalır.
/// </remarks>
public class SuperTwistingTerm
{
    private readonly double[] v = new double[4];           // her eklemin integral terimi v, rad/s²
    private readonly double[] candidate = new double[4];   // bu örnekte hesaplanan, henüz kabul edilmemiş v

    /// <summary>v'yi sıfırlar.</summary>
    public void Reset()
    {
        Array.Clear(v);
        Array.Clear(candidate);
    }

    /// <summary>
    /// Eklem i için kayma değişkeni s'den süper-büküm terimini hesaplar (v'nin adayıyla). dt: önceki örnekten
    /// beri geçen süre, s. v henüz değişmez; değişmesi için <see cref="Commit"/> çağrılmalıdır.
    /// </summary>
    public double Evaluate(int i, double s, double dt, double k1, double k2)
    {
        double sign = Sign(s);
        candidate[i] = v[i] + dt * k2 * sign;
        return k1 * Math.Sqrt(Math.Abs(s)) * sign + candidate[i];
    }

    /// <summary>Eklem i'nin aday v değerini kabul eder (çıkış kırpılmadıysa çağrılır).</summary>
    public void Commit(int i)
    {
        v[i] = candidate[i];
    }

    // İşaret fonksiyonu: pozitifte +1, negatifte −1, sıfırda 0. Math.Sign kullanılmaz, çünkü s NaN olursa
    // (sayısal patlama) istisna atar; burada NaN olduğu gibi geçer ve simülatör "sayısal hata" diye durur.
    private static double Sign(double s)
    {
        if (s > 0.0)
            return 1.0;
        if (s < 0.0)
            return -1.0;
        return s;
    }
}
