using System.Globalization;
using RobotArm.Core.Model;

namespace RobotArm.Core.Control;

/// <summary>
/// Turns the true joint state into what the controller sees: measured angles and estimated speeds. SPEC §6.3.
/// Called once per controller sample; <see cref="Measure"/> must not allocate.
/// </summary>
public interface ISensor
{
    /// <summary>Number of joints n.</summary>
    int DegreesOfFreedom { get; }

    /// <summary>Clears the internal state (previous sample, filter, random generator). SPEC §6.3.</summary>
    void Reset();

    /// <summary>Measures the joints at time <paramref name="t"/>. SPEC §6.3. Does not allocate.</summary>
    /// <param name="t">Time of the sample, s.</param>
    /// <param name="q">True joint angles, rad.</param>
    /// <param name="qd">True joint speeds, rad/s.</param>
    /// <param name="qMeasured">Output: measured angles, rad.</param>
    /// <param name="qdMeasured">Output: measured or estimated speeds, rad/s.</param>
    void Measure(double t, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, Span<double> qMeasured, Span<double> qdMeasured);
}

/// <summary>The "perfect sensor" of SPEC §6.3: returns the true angles and speeds. For comparison.</summary>
public sealed class PerfectSensor : ISensor
{
    /// <summary>Creates the sensor for <paramref name="degreesOfFreedom"/> joints. SPEC §6.3.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The number of joints is not greater than 0.</exception>
    public PerfectSensor(int degreesOfFreedom)
    {
        if (degreesOfFreedom <= 0)
            throw new ArgumentOutOfRangeException(nameof(degreesOfFreedom), degreesOfFreedom, "Number of joints must be greater than 0.");
        DegreesOfFreedom = degreesOfFreedom;
    }

    /// <inheritdoc/>
    public int DegreesOfFreedom { get; }

    /// <inheritdoc/>
    public void Reset()
    {
    }

    /// <inheritdoc/>
    public void Measure(double t, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, Span<double> qMeasured, Span<double> qdMeasured)
    {
        SensorCheck.Lengths(DegreesOfFreedom, q, qd, qMeasured, qdMeasured);
        q.CopyTo(qMeasured);
        qd.CopyTo(qdMeasured);
    }
}

/// <summary>Options of the <see cref="EncoderSensor"/>. SPEC §6.3.</summary>
public sealed record EncoderSensorOptions
{
    private readonly double _noiseStdDev;
    private readonly double _filterTimeConstant;

    /// <summary>
    /// Standard deviation of Gaussian noise added to each angle before quantisation, rad. 0 (the default) for none.
    /// </summary>
    public double NoiseStdDev
    {
        get => _noiseStdDev;
        init => _noiseStdDev = SensorCheck.NonNegative(value, nameof(NoiseStdDev));
    }

    /// <summary>
    /// Time constant τ of the first-order low-pass filter on the speed estimate, s. 0 (the default) for no filter.
    /// </summary>
    public double FilterTimeConstant
    {
        get => _filterTimeConstant;
        init => _filterTimeConstant = SensorCheck.NonNegative(value, nameof(FilterTimeConstant));
    }

    /// <summary>Seed of the random generator, so that runs are repeatable (SPEC §6.3, §8.3).</summary>
    public int Seed { get; init; }
}

/// <summary>
/// Encoder model of SPEC §6.3: angles quantised to the encoder resolution 2π/counts of each joint (SPEC §3.3),
/// optionally with seeded Gaussian noise; speeds by backward difference of the quantised angles between samples,
/// optionally low-pass filtered.
/// </summary>
/// <remarks>
/// <para>
/// Measured angle: q_m = r·round((q + n)/r), r = 2π/4096 = 1.53e-3 rad, n ~ N(0, σ²) when σ &gt; 0.
/// Speed estimate: v = (q_m(t) − q_m(t − Δt))/Δt, then y ← y + α(v − y) with α = Δt/(τ + Δt) when τ &gt; 0.
/// The first sample after <see cref="Reset"/> has no previous angle, so its speed estimate is 0.
/// </para>
/// <para>
/// Noise comes from <see cref="Random"/> with the given seed (Box–Muller), so the same seed gives the same
/// measurements. <see cref="Measure"/> does not allocate.
/// </para>
/// </remarks>
public sealed class EncoderSensor : ISensor
{
    private readonly int _n;
    private readonly double[] _resolution;
    private readonly double[] _previous;
    private readonly double[] _speed;
    private Random _random;
    private double _spareGaussian;
    private bool _hasSpare;
    private bool _started;
    private double _lastTime;

    /// <summary>Creates the sensor for the encoders of <paramref name="model"/>. SPEC §6.3.</summary>
    /// <param name="model">The arm; each joint's resolution is <see cref="ActuatorParameters.EncoderResolution"/>.</param>
    /// <param name="options">Noise, filter and seed; null for the defaults (no noise, no filter).</param>
    /// <exception cref="ArgumentNullException"><paramref name="model"/> is null.</exception>
    public EncoderSensor(ArmModel model, EncoderSensorOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(model);
        Options = options ?? new EncoderSensorOptions();
        _n = model.DegreesOfFreedom;
        _resolution = new double[_n];
        for (var i = 0; i < _n; i++)
            _resolution[i] = model.Actuators[i].EncoderResolution;
        _previous = new double[_n];
        _speed = new double[_n];
        _random = new Random(Options.Seed);
    }

    /// <summary>The options.</summary>
    public EncoderSensorOptions Options { get; }

    /// <inheritdoc/>
    public int DegreesOfFreedom => _n;

    /// <summary>Resolution of each joint's encoder, rad.</summary>
    public ReadOnlySpan<double> Resolution => _resolution;

    /// <inheritdoc/>
    public void Reset()
    {
        _random = new Random(Options.Seed);
        _hasSpare = false;
        _started = false;
        _lastTime = 0.0;
        Array.Clear(_previous);
        Array.Clear(_speed);
    }

    /// <inheritdoc/>
    /// <exception cref="ArgumentException">A span has the wrong length, or the time did not increase.</exception>
    public void Measure(double t, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, Span<double> qMeasured, Span<double> qdMeasured)
    {
        SensorCheck.Lengths(_n, q, qd, qMeasured, qdMeasured);
        var dt = t - _lastTime;
        if (_started && !(dt > 0.0))
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"Sample time must increase: {t} s after {_lastTime} s (SPEC §6.3)."), nameof(t));

        var tau = Options.FilterTimeConstant;
        var alpha = tau > 0.0 ? dt / (tau + dt) : 1.0;
        for (var i = 0; i < _n; i++)
        {
            var angle = q[i];
            if (Options.NoiseStdDev > 0.0)
                angle += Options.NoiseStdDev * NextGaussian();
            var r = _resolution[i];
            var measured = r * Math.Round(angle / r);
            if (_started)
            {
                var raw = (measured - _previous[i]) / dt;
                _speed[i] += alpha * (raw - _speed[i]);
            }
            _previous[i] = measured;
            qMeasured[i] = measured;
            qdMeasured[i] = _speed[i];
        }
        _started = true;
        _lastTime = t;
    }

    /// <summary>Standard normal number by the Box–Muller method; two per pair of uniforms.</summary>
    private double NextGaussian()
    {
        if (_hasSpare)
        {
            _hasSpare = false;
            return _spareGaussian;
        }
        var u1 = 1.0 - _random.NextDouble(); // in (0, 1], so the logarithm is finite
        var u2 = _random.NextDouble();
        var radius = Math.Sqrt(-2.0 * Math.Log(u1));
        var angle = 2.0 * Math.PI * u2;
        _spareGaussian = radius * Math.Sin(angle);
        _hasSpare = true;
        return radius * Math.Cos(angle);
    }
}

internal static class SensorCheck
{
    public static void Lengths(int n, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, Span<double> qMeasured, Span<double> qdMeasured)
    {
        if (q.Length != n || qd.Length != n || qMeasured.Length != n || qdMeasured.Length != n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"Expected {n} values in q, q̇ and both outputs (SPEC §6.3)."));
    }

    public static double NonNegative(double value, string name) =>
        double.IsFinite(value) && value >= 0.0
            ? value
            : throw new ArgumentOutOfRangeException(name, value, "Must be a finite number of at least 0 (SPEC §6.3).");
}
