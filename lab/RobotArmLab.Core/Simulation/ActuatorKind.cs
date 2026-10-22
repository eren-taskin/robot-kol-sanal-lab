namespace RobotArmLab.Core.Simulation;

/// <summary>Kontrolcünün tork komutunun kola nasıl ulaştığı.</summary>
public enum ActuatorKind
{
    /// <summary>İdeal tork: komut doğrudan uygulanır (yalnız ±τmax doyumu).</summary>
    IdealTorque,

    /// <summary>
    /// Motor: komut ters DC motor modeliyle gerilime çevrilir (V = R·τ/(N·Kt) + N·Ke·q̇_ölçülen), gerilim Ts boyunca
    /// tutulur ve tork DC motor modelinden gelir. Böylece zıt EMK ve 12 V sınırı devreye girer.
    /// </summary>
    Motor,
}
