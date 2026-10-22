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
    private readonly double[] candidate = new double[4];

    /// <summary>Her eklemin integral terimi v, rad/s².</summary>
    public double[] V { get; } = new double[4];

    /// <summary>Son örnekteki kayma değişkeni s = ė + Λe, rad/s.</summary>
    public double[] S { get; } = new double[4];

    /// <summary>v ve s'yi sıfırlar.</summary>
    public void Reset()
    {
        Array.Clear(V);
        Array.Clear(S);
        Array.Clear(candidate);
    }

    /// <summary>
    /// Eklem i için kayma değişkeni s'den süper-büküm terimini hesaplar (v'nin adayıyla). dt: önceki örnekten
    /// beri geçen süre, s. v henüz değişmez; değişmesi için <see cref="Commit"/> çağrılmalıdır.
    /// </summary>
    public double Evaluate(int i, double s, double dt, double k1, double k2)
    {
        double sign = s > 0.0 ? 1.0 : (s < 0.0 ? -1.0 : s);   // sign(0) = 0; NaN kalır (Math.Sign NaN'da istisna atar)
        S[i] = s;
        candidate[i] = V[i] + dt * k2 * sign;
        return k1 * Math.Sqrt(Math.Abs(s)) * sign + candidate[i];
    }

    /// <summary>Eklem i'nin aday v değerini kabul eder (çıkış kırpılmadıysa çağrılır).</summary>
    public void Commit(int i)
    {
        V[i] = candidate[i];
    }
}
