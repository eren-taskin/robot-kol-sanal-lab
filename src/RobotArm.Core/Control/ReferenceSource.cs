namespace RobotArm.Core.Control;

/// <summary>
/// Gives the joint reference q_d(t), q̇_d(t), q̈_d(t) at each controller sample. SPEC §6.3, §7, §8.1.
/// <see cref="Evaluate"/> must not allocate. The trajectories of SPEC §8.1 implement it (task T08).
/// </summary>
public interface IReferenceSource
{
    /// <summary>Number of joints n.</summary>
    int DegreesOfFreedom { get; }

    /// <summary>Writes the reference at time <paramref name="t"/>. Does not allocate.</summary>
    /// <param name="t">Time, s.</param>
    /// <param name="position">Output: q_d, rad.</param>
    /// <param name="velocity">Output: q̇_d, rad/s.</param>
    /// <param name="acceleration">Output: q̈_d, rad/s².</param>
    void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration);
}

/// <summary>A constant set point: q_d fixed, q̇_d = q̈_d = 0. A step when the arm starts elsewhere. SPEC §7.</summary>
public sealed class ConstantReference : IReferenceSource
{
    private readonly double[] _position;

    /// <summary>Creates the set point; the angles are copied.</summary>
    /// <exception cref="ArgumentException">No angles.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An angle is not finite.</exception>
    public ConstantReference(ReadOnlySpan<double> position)
    {
        if (position.IsEmpty)
            throw new ArgumentException("At least one joint angle is needed.", nameof(position));
        foreach (var p in position)
            if (!double.IsFinite(p))
                throw new ArgumentOutOfRangeException(nameof(position), p, "The set point must be finite.");
        _position = position.ToArray();
    }

    /// <summary>The set point q_d, rad.</summary>
    public ReadOnlySpan<double> Position => _position;

    /// <inheritdoc/>
    public int DegreesOfFreedom => _position.Length;

    /// <inheritdoc/>
    public void Evaluate(double t, Span<double> position, Span<double> velocity, Span<double> acceleration)
    {
        _position.CopyTo(position);
        velocity[.._position.Length].Clear();
        acceleration[.._position.Length].Clear();
    }
}
