using System.Globalization;

namespace RobotArm.Core.Control;

/// <summary>
/// Per-joint proportional and derivative gains of a PD-type law. SPEC §7. Units depend on the controller:
/// N·m/rad and N·m·s/rad for <see cref="PdGravity"/>; 1/s² and 1/s for <see cref="ComputedTorque"/>.
/// </summary>
public sealed record PdGains
{
    /// <summary>Creates the gains; the arrays are copied. SPEC §7.</summary>
    /// <exception cref="ArgumentException">The arrays are empty or have different lengths.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A gain is negative or not finite.</exception>
    public PdGains(IEnumerable<double> kp, IEnumerable<double> kd)
    {
        Kp = GainCheck.Copy(kp, nameof(kp));
        Kd = GainCheck.Copy(kd, nameof(kd));
        GainCheck.SameLength(Kp.Count, Kd.Count, nameof(kd));
    }

    /// <summary>Proportional gain of each joint.</summary>
    public IReadOnlyList<double> Kp { get; }

    /// <summary>Derivative gain of each joint.</summary>
    public IReadOnlyList<double> Kd { get; }

    /// <summary>Number of joints.</summary>
    public int DegreesOfFreedom => Kp.Count;
}

/// <summary>Per-joint gains of the independent-joint PID: N·m/rad, N·m/(rad·s), N·m·s/rad. SPEC §7.</summary>
public sealed record PidGains
{
    /// <summary>Creates the gains; the arrays are copied. SPEC §7.</summary>
    /// <exception cref="ArgumentException">The arrays are empty or have different lengths.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A gain is negative or not finite.</exception>
    public PidGains(IEnumerable<double> kp, IEnumerable<double> ki, IEnumerable<double> kd)
    {
        Kp = GainCheck.Copy(kp, nameof(kp));
        Ki = GainCheck.Copy(ki, nameof(ki));
        Kd = GainCheck.Copy(kd, nameof(kd));
        GainCheck.SameLength(Kp.Count, Ki.Count, nameof(ki));
        GainCheck.SameLength(Kp.Count, Kd.Count, nameof(kd));
    }

    /// <summary>Proportional gain of each joint, N·m/rad.</summary>
    public IReadOnlyList<double> Kp { get; }

    /// <summary>Integral gain of each joint, N·m/(rad·s).</summary>
    public IReadOnlyList<double> Ki { get; }

    /// <summary>Derivative gain of each joint, N·m·s/rad.</summary>
    public IReadOnlyList<double> Kd { get; }

    /// <summary>Number of joints.</summary>
    public int DegreesOfFreedom => Kp.Count;
}

internal static class GainCheck
{
    public static IReadOnlyList<double> Copy(IEnumerable<double> values, string name)
    {
        ArgumentNullException.ThrowIfNull(values, name);
        var copy = values.ToArray();
        if (copy.Length == 0)
            throw new ArgumentException("At least one gain is needed (SPEC §7).", name);
        foreach (var v in copy)
            if (!double.IsFinite(v) || v < 0.0)
                throw new ArgumentOutOfRangeException(name, v, "A gain must be a finite number of at least 0 (SPEC §7).");
        return Array.AsReadOnly(copy);
    }

    public static void SameLength(int expected, int actual, string name)
    {
        if (actual != expected)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"Expected {expected} gains, got {actual} (SPEC §7)."), name);
    }

    public static void MatchesBelief(int gains, ControllerBelief belief, string name)
    {
        if (gains != belief.DegreesOfFreedom)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"The gains are for {gains} joints but the model has {belief.DegreesOfFreedom} (SPEC §7)."), name);
    }

    public static void CheckLengths(int n, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, JointReference reference, Span<double> u)
    {
        if (q.Length != n || qd.Length != n || reference.Position.Length != n || u.Length != n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"Expected {n} values in q, q̇, the reference and u (SPEC §7)."));
    }

    public static void CheckSquareMatrix(int n, ReadOnlySpan<double> q0, double samplePeriod, Span<double> m)
    {
        if (!double.IsFinite(samplePeriod) || samplePeriod < 0.0)
            throw new ArgumentOutOfRangeException(nameof(samplePeriod), samplePeriod, "Sample period must be a finite number of at least 0 s.");
        if (q0.Length != n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Expected {n} joint angles, got {q0.Length}."), nameof(q0));
        if (m.Length != n * n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Expected room for {n * n} gains, got {m.Length}."), nameof(m));
    }

    public static void CheckGainMatrix(int n, ReadOnlySpan<double> q0, double samplePeriod, Span<double> k)
    {
        if (!double.IsFinite(samplePeriod) || samplePeriod < 0.0)
            throw new ArgumentOutOfRangeException(nameof(samplePeriod), samplePeriod, "Sample period must be a finite number of at least 0 s.");
        if (q0.Length != n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Expected {n} joint angles, got {q0.Length}."), nameof(q0));
        if (k.Length != 2 * n * n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Expected room for {2 * n * n} gains, got {k.Length}."), nameof(k));
    }
}
