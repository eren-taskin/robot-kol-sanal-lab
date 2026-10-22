using System;

namespace RobotArmLab.Core.Solvers;

/// <summary>
/// Dormand–Prince RK45: uyarlamalı adımlı Runge–Kutta (MATLAB'ın ode45'i ile aynı yöntem). Her denemede
/// 7 eğimle hem 5. hem 4. mertebe bir çözüm bulur; ikisinin farkı yerel hatanın tahminidir. Hata
/// toleransın altındaysa adım kabul edilir, değilse reddedilip daha küçük adımla tekrarlanır.
/// </summary>
/// <remarks>
/// Step(f, t, x, h) çağrısı x(t + h)'yi verir; aralık içinde kaç alt adım atılacağına çözücü karar verir.
/// Hata ölçütü (ode45 gibi): her bileşen için e_i / (atol + rtol·max(|x_i|, |x_yeni,i|)), sonra bunların
/// karekök-ortalama-karesi ≤ 1 olmalı. MATLAB varsayılanları rtol = 1e-3, atol = 1e-6'dır; burada fizik
/// için daha sıkı değerler (1e-6, 1e-9) varsayılandır.
/// </remarks>
public class DormandPrinceSolver : IOdeSolver
{
    // Butcher tablosu (Dormand ve Prince, 1980). c: aşama zamanları, A: aşama ağırlıkları.
    private static readonly double[] C = { 0.0, 1.0 / 5, 3.0 / 10, 4.0 / 5, 8.0 / 9, 1.0, 1.0 };
    private static readonly double[,] A =
    {
        { 0, 0, 0, 0, 0, 0, 0 },
        { 1.0 / 5, 0, 0, 0, 0, 0, 0 },
        { 3.0 / 40, 9.0 / 40, 0, 0, 0, 0, 0 },
        { 44.0 / 45, -56.0 / 15, 32.0 / 9, 0, 0, 0, 0 },
        { 19372.0 / 6561, -25360.0 / 2187, 64448.0 / 6561, -212.0 / 729, 0, 0, 0 },
        { 9017.0 / 3168, -355.0 / 33, 46732.0 / 5247, 49.0 / 176, -5103.0 / 18656, 0, 0 },
        { 35.0 / 384, 0, 500.0 / 1113, 125.0 / 192, -2187.0 / 6784, 11.0 / 84, 0 },
    };
    // 5. mertebe çözümün ağırlıkları (A'nın son satırıyla aynı) ve 4. mertebe çözümünkiler.
    private static readonly double[] B5 = { 35.0 / 384, 0, 500.0 / 1113, 125.0 / 192, -2187.0 / 6784, 11.0 / 84, 0 };
    private static readonly double[] B4 = { 5179.0 / 57600, 0, 7571.0 / 16695, 393.0 / 640, -92097.0 / 339200, 187.0 / 2100, 1.0 / 40 };

    // Adım boyu denetimi (ode45 ile aynı kurallar). Yerel hata h⁵ ile orantılıdır, bu yüzden hatayı 1'e indiren
    // adım h·(1/hata)^(1/5)'tir. Güvenlik çarpanı 0,9 tahmini biraz küçük tutar, adım bir seferde en çok 5 kat
    // büyür ve 5 kat (0,2) küçülür.
    private const double ErrorExponent = 1.0 / 5.0;
    private const double SafetyFactor = 0.9;
    private const double MinStepScale = 0.2;
    private const double MaxStepScale = 5.0;
    private const double FirstStepGuessS = 0.01;   // ilk çağrıda kaba başlangıç tahmini, s
    private const double MinStepS = 1e-12;          // bunun altına inen adım sistemin patladığı anlamına gelir, s

    /// <summary>Bağıl tolerans rtol (boyutsuz).</summary>
    public double RelativeTolerance { get; set; }

    /// <summary>Mutlak tolerans atol (durumun birimiyle: rad veya rad/s).</summary>
    public double AbsoluteTolerance { get; set; }

    private double nextStepS;   // bir sonraki alt adım için tahmin, s (0 = henüz yok)

    /// <summary>En son kabul edilen alt adımın boyu, s.</summary>
    public double LastAcceptedStepS { get; private set; }

    /// <summary>Şimdiye kadar kabul edilen alt adım sayısı.</summary>
    public int AcceptedStepCount { get; private set; }

    /// <summary>Şimdiye kadar reddedilen (hatası toleransı aşan) deneme sayısı.</summary>
    public int RejectedStepCount { get; private set; }

    /// <summary>Varsayılan toleranslarla (rtol = 1e-6, atol = 1e-9) bir çözücü oluşturur.</summary>
    public DormandPrinceSolver()
    {
        RelativeTolerance = 1e-6;
        AbsoluteTolerance = 1e-9;
    }

    /// <summary>
    /// x(t)'den x(t + h)'ye uyarlamalı alt adımlarla ilerler. Bir sonraki alt adım boyu hatadan hesaplanır:
    /// h_yeni = h · 0,9 · (1/hata)^(1/5), 0,2 ile 5 kat arasında sınırlanır.
    /// </summary>
    public double[] Step(OdeFunction f, double t, double[] x, double h)
    {
        double tEnd = t + h;
        if (nextStepS <= 0.0)
            nextStepS = Math.Min(h, FirstStepGuessS);
        double[] current = x;
        double time = t;

        while (time < tEnd)
        {
            bool clipped = nextStepS >= tEnd - time;   // aralığın sonuna denk getirmek için kısaltıldı mı
            double hStep = clipped ? tEnd - time : nextStepS;
            double error;
            double[] next = TryStep(f, time, current, hStep, out error);
            double factor = error > 0.0 ? SafetyFactor * Math.Pow(1.0 / error, ErrorExponent) : MaxStepScale;
            double hNew = hStep * Math.Max(MinStepScale, Math.Min(MaxStepScale, factor));

            if (error <= 1.0)
            {
                time = clipped ? tEnd : time + hStep;
                current = next;
                LastAcceptedStepS = hStep;
                AcceptedStepCount++;
                if (!clipped)
                    nextStepS = hNew;   // kısaltılmış son adım, bir sonraki tahmini küçültmesin
            }
            else
            {
                RejectedStepCount++;
                nextStepS = hNew;
                if (nextStepS < MinStepS)
                    throw new InvalidOperationException("Dormand–Prince: adım boyu 1e-12 s'nin altına düştü (sistem çok sert veya NaN).");
            }
        }
        return current;
    }

    // Tek bir deneme adımı: 7 eğim, 5. mertebe çözüm ve hatanın normu (≤ 1 ise kabul).
    private double[] TryStep(OdeFunction f, double t, double[] x, double h, out double errorNorm)
    {
        int n = x.Length;
        double[][] k = new double[7][];
        for (int s = 0; s < 7; s++)
        {
            double[] stage = (double[])x.Clone();
            for (int j = 0; j < s; j++)
                for (int i = 0; i < n; i++)
                    stage[i] += h * A[s, j] * k[j][i];
            k[s] = f(t + C[s] * h, stage);
        }

        double[] x5 = new double[n];
        double sumSquares = 0.0;
        for (int i = 0; i < n; i++)
        {
            double d5 = 0.0, d4 = 0.0;
            for (int s = 0; s < 7; s++)
            {
                d5 += B5[s] * k[s][i];
                d4 += B4[s] * k[s][i];
            }
            x5[i] = x[i] + h * d5;
            double scale = AbsoluteTolerance + RelativeTolerance * Math.Max(Math.Abs(x[i]), Math.Abs(x5[i]));
            double e = h * (d5 - d4) / scale;
            sumSquares += e * e;
        }
        errorNorm = Math.Sqrt(sumSquares / n);
        if (double.IsNaN(errorNorm))
            errorNorm = double.PositiveInfinity;
        return x5;
    }
}
