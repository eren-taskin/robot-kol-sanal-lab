namespace RobotArmLab.Core.Simulation;

/// <summary>Açık çevrimde kola ne uygulandığı.</summary>
public enum InputMode
{
    /// <summary>Eklem torku doğrudan verilir, N·m (yalnız ±τmax doyumu uygulanır).</summary>
    Torque,

    /// <summary>Motor gerilimi verilir, V; tork DC motor modeliyle hesaplanır (zıt EMK frenler).</summary>
    Voltage,
}

/// <summary>Bir eklemin giriş sinyalinin biçimi.</summary>
public enum InputShape
{
    /// <summary>Baştan beri sabit değer.</summary>
    Constant,

    /// <summary>t0 anına kadar 0, sonra sabit değer (basamak).</summary>
    Step,

    /// <summary>Genlik · sin(2π f t) (sinüs).</summary>
    Sine,
}
