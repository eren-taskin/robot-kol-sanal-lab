namespace RobotArm.Core.Model;

/// <summary>
/// Actuator and friction parameters shared by every joint (DYNAMIXEL XM430-W350). SPEC §3.3, §5.4.
/// The defaults are the values of SPEC §3.3; experiments change them with <c>with</c> or an initializer.
/// </summary>
public sealed record ActuatorParameters
{
    private readonly double _stallTorque = 4.1;
    private readonly double _gearRatio = 353.5;
    private readonly int _encoderCountsPerRevolution = 4096;
    private readonly double _noLoadSpeed = 4.8;
    private readonly double _rotorInertia;
    private readonly double _viscousFriction = 0.01;
    private readonly double _coulombFriction = 0.02;
    private readonly double _smoothingSpeed = 0.1;

    /// <summary>Stall torque τmax in N·m, the saturation limit of SPEC §5.4. SPEC §3.3: 4.1 N·m at 12 V.</summary>
    public double StallTorque
    {
        get => _stallTorque;
        init => _stallTorque = Positive(value, nameof(StallTorque), " N·m");
    }

    /// <summary>Gear ratio N. SPEC §3.3: 353.5.</summary>
    public double GearRatio
    {
        get => _gearRatio;
        init => _gearRatio = Positive(value, nameof(GearRatio), "");
    }

    /// <summary>
    /// Encoder counts per output revolution. SPEC §3.3: 4096, as in the ROBOTIS e-manual for the XM430-W350
    /// (resolution 4096 pulse/rev, https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/).
    /// </summary>
    public int EncoderCountsPerRevolution
    {
        get => _encoderCountsPerRevolution;
        init => _encoderCountsPerRevolution = value > 0
            ? value
            : throw new ArgumentOutOfRangeException(nameof(EncoderCountsPerRevolution), value,
                "Encoder counts per revolution must be greater than 0 (SPEC §3.3).");
    }

    /// <summary>
    /// No-load speed in rad/s. SPEC §3.3: 46 rpm at 12 V (4.82 rad/s, rounded to 4.8), as in the ROBOTIS e-manual
    /// for the XM430-W350 (https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/).
    /// </summary>
    public double NoLoadSpeed
    {
        get => _noLoadSpeed;
        init => _noLoadSpeed = Positive(value, nameof(NoLoadSpeed), " rad/s");
    }

    /// <summary>Rotor inertia Jm in kg·m², before the gear. SPEC §3.3: unknown, default 0.</summary>
    public double RotorInertia
    {
        get => _rotorInertia;
        init => _rotorInertia = NonNegative(value, nameof(RotorInertia), " kg·m²");
    }

    /// <summary>Viscous friction fv in N·m·s/rad. SPEC §3.3: default 0.01.</summary>
    public double ViscousFriction
    {
        get => _viscousFriction;
        init => _viscousFriction = NonNegative(value, nameof(ViscousFriction), " N·m·s/rad");
    }

    /// <summary>Coulomb friction fc in N·m. SPEC §3.3: default 0.02.</summary>
    public double CoulombFriction
    {
        get => _coulombFriction;
        init => _coulombFriction = NonNegative(value, nameof(CoulombFriction), " N·m");
    }

    /// <summary>
    /// Smoothing speed ωs of the tanh friction model in rad/s. SPEC §3.3: 0.1. Together with fc it sets the
    /// largest stable step (SPEC §6.2, docs/DECISIONS.md D3).
    /// </summary>
    public double SmoothingSpeed
    {
        get => _smoothingSpeed;
        init => _smoothingSpeed = Positive(value, nameof(SmoothingSpeed), " rad/s");
    }

    /// <summary>Reflected rotor inertia N²·Jm added to each diagonal entry of M. SPEC §5.2.</summary>
    public double ReflectedRotorInertia => _gearRatio * _gearRatio * _rotorInertia;

    /// <summary>Encoder resolution 2π / counts, in rad. SPEC §6.3.</summary>
    public double EncoderResolution => 2.0 * Math.PI / _encoderCountsPerRevolution;

    private static double Positive(double value, string name, string unit) =>
        double.IsFinite(value) && value > 0.0
            ? value
            : throw new ArgumentOutOfRangeException(name, value, $"{name} must be a finite number greater than 0{unit} (SPEC §3.3).");

    private static double NonNegative(double value, string name, string unit) =>
        double.IsFinite(value) && value >= 0.0
            ? value
            : throw new ArgumentOutOfRangeException(name, value, $"{name} must be a finite number of at least 0{unit} (SPEC §3.3).");
}
