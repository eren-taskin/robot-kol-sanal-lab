using System;

namespace RobotArmLab.Core.Simulation;

/// <summary>Kontrolcünün kolu nasıl ölçtüğü.</summary>
public enum SensorKind
{
    /// <summary>Kusursuz ölçüm: kontrolcü gerçek q ve q̇'yu görür.</summary>
    Perfect,

    /// <summary>Kodlayıcı (encoder): açı nicemlenir (isteğe bağlı gürültüyle), hız sonlu farkla bulunur.</summary>
    Encoder,
}

/// <summary>
/// Eklem algılayıcısı. Kodlayıcı modunda ölçülen açı q_m = r·round((q + n)/r), r = 2π/sayım (4096 sayım/tur için
/// r = 1,53e-3 rad), n ~ N(0, σ²) tohumlu Gauss gürültüsü. Hız geri farkla bulunur: q̇_m = (q_m(t) − q_m(t − Ts))/Ts;
/// Reset'ten sonraki ilk örnekte önceki açı olmadığından hız 0'dır. Aynı tohum aynı ölçümleri verir.
/// </summary>
public class JointSensor
{
    private readonly double[] previous = new double[4];
    private Random random;
    private bool started;
    private double lastTimeS;

    /// <summary>Algılayıcı türü.</summary>
    public SensorKind Kind { get; set; }

    /// <summary>Kodlayıcının tur başına sayımı (varsayılan 4096, XM430-W350).</summary>
    public int CountsPerRevolution { get; set; }

    /// <summary>Açı gürültüsünün standart sapması σ, rad (0: gürültü yok).</summary>
    public double NoiseStdDevRad { get; set; }

    /// <summary>Gürültü üretecinin tohumu.</summary>
    public int Seed { get; set; }

    /// <summary>Kusursuz algılayıcı kurar; kodlayıcı ayarları 4096 sayım, gürültü yok, tohum 1.</summary>
    public JointSensor()
    {
        Kind = SensorKind.Perfect;
        CountsPerRevolution = 4096;
        Seed = 1;
        Reset();
    }

    /// <summary>Gürültü üretecini tohumdan yeniden kurar ve önceki ölçümü unutur.</summary>
    public void Reset()
    {
        random = new Random(Seed);
        started = false;
        lastTimeS = 0.0;
        Array.Clear(previous);
    }

    /// <summary>t anında gerçek q, q̇'dan ölçülen qMeasured (rad) ve qdMeasured'ı (rad/s) doldurur.</summary>
    public void Measure(double t, double[] q, double[] qd, double[] qMeasured, double[] qdMeasured)
    {
        if (Kind == SensorKind.Perfect)
        {
            Array.Copy(q, qMeasured, 4);
            Array.Copy(qd, qdMeasured, 4);
            return;
        }
        double dt = t - lastTimeS;
        double r = 2.0 * Math.PI / CountsPerRevolution;   // kodlayıcı çözünürlüğü, rad
        for (int i = 0; i < 4; i++)
        {
            double angle = q[i] + (NoiseStdDevRad > 0.0 ? NoiseStdDevRad * NextGaussian() : 0.0);
            double measured = r * Math.Round(angle / r);
            qdMeasured[i] = started && dt > 0.0 ? (measured - previous[i]) / dt : 0.0;
            qMeasured[i] = measured;
            previous[i] = measured;
        }
        started = true;
        lastTimeS = t;
    }

    // Box–Muller: iki düzgün sayıdan bir standart normal sayı.
    private double NextGaussian()
    {
        double u1 = 1.0 - random.NextDouble();   // (0, 1] aralığında, logaritma sonlu kalır
        double u2 = random.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
    }
}
