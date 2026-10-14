using System.Globalization;

namespace RobotArm.Tests;

/// <summary>Formatting of numbers in test output and failure messages with InvariantCulture (PROJECT.md §3).</summary>
internal static class TestFormat
{
    /// <summary>The values joined by ", ", each in round-trip format with InvariantCulture.</summary>
    public static string Join(IEnumerable<double> values) =>
        string.Join(", ", values.Select(v => v.ToString("R", CultureInfo.InvariantCulture)));
}
