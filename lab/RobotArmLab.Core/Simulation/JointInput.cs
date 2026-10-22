using System;

namespace RobotArmLab.Core.Simulation;

/// <summary>
/// Bir eklemin açık çevrim giriş sinyali u(t): sabit, t0'da basamak veya sinüs. Birim, giriş moduna göre
/// N·m (tork) veya V (gerilim) olur.
/// </summary>
public class JointInput
{
    /// <summary>Sinyalin biçimi.</summary>
    public InputShape Shape { get; set; }

    /// <summary>Genlik (sabit değer, basamak yüksekliği veya sinüs genliği), N·m veya V.</summary>
    public double Amplitude { get; set; }

    /// <summary>Basamağın başladığı an t0, s.</summary>
    public double StepTimeS { get; set; }

    /// <summary>Sinüsün frekansı f, Hz.</summary>
    public double FrequencyHz { get; set; }

    /// <summary>Sıfır giriş (sabit 0) oluşturur.</summary>
    public JointInput()
    {
        Shape = InputShape.Constant;
    }

    /// <summary>Aynı değerlerle yeni bir giriş nesnesi döndürür.</summary>
    public JointInput Clone()
    {
        return (JointInput)MemberwiseClone();   // bütün alanlar sayı veya enum olduğundan sığ kopya yeterlidir
    }

    /// <summary>t anındaki giriş değeri, N·m veya V.</summary>
    public double Value(double t)
    {
        switch (Shape)
        {
            case InputShape.Step:
                return t >= StepTimeS ? Amplitude : 0.0;
            case InputShape.Sine:
                return Amplitude * Math.Sin(2.0 * Math.PI * FrequencyHz * t);
            default:
                return Amplitude;
        }
    }
}
