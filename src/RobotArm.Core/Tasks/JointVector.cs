using System.Text.Json.Serialization;

namespace RobotArm.Core.Tasks;

/// <summary>Immutable four-joint data for scenarios and log rows. SPEC §8.3, §10.</summary>
/// <param name="J1">Joint 1 value.</param>
/// <param name="J2">Joint 2 value.</param>
/// <param name="J3">Joint 3 value.</param>
/// <param name="J4">Joint 4 value.</param>
public readonly record struct JointVector(double J1, double J2, double J3, double J4)
{
    /// <summary>Value by zero-based joint index. SPEC §2.</summary>
    [JsonIgnore]
    public double this[int i] => i switch { 0 => J1, 1 => J2, 2 => J3, 3 => J4, _ => throw new ArgumentOutOfRangeException(nameof(i)) };

    /// <summary>Copies four values without retaining the input. SPEC §10.</summary>
    public static JointVector From(ReadOnlySpan<double> values) => new(values[0], values[1], values[2], values[3]);

    /// <summary>Writes four values without allocation. SPEC §8.</summary>
    public void CopyTo(Span<double> values) { for (var i = 0; i < 4; i++) values[i] = this[i]; }

    /// <summary>Returns an independent array. SPEC §8.3.</summary>
    public double[] ToArray() => [J1, J2, J3, J4];

    internal void Validate(string name, bool nonNegative = false)
    {
        for (var i = 0; i < 4; i++)
            if (!double.IsFinite(this[i]) || (nonNegative && this[i] < 0))
                throw new ArgumentOutOfRangeException(name, "Expected four finite joint values (SPEC §8.3).");
    }
}
