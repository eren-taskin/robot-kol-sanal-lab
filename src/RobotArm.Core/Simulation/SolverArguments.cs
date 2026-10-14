namespace RobotArm.Core.Simulation;

/// <summary>Argument checks shared by the solvers of SPEC §6.2. Does not allocate unless it throws.</summary>
internal static class SolverArguments
{
    public static void Check(IPlant plant, int stateCount, Span<double> x, ReadOnlySpan<double> u, double h)
    {
        ArgumentNullException.ThrowIfNull(plant);
        if (plant.StateCount != stateCount)
            throw new ArgumentException(
                $"The solver was built for {stateCount} states but the plant has {plant.StateCount} (SPEC §6.2).", nameof(plant));
        if (x.Length != stateCount)
            throw new ArgumentException($"Expected {stateCount} state values, got {x.Length} (SPEC §6.1).", nameof(x));
        if (u.Length != plant.InputCount)
            throw new ArgumentException($"Expected {plant.InputCount} inputs, got {u.Length} (SPEC §6.1).", nameof(u));
        if (!double.IsFinite(h) || h <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(h), h, "Step length must be a finite number greater than 0 s (SPEC §6.2).");
    }

    public static int CheckStateCount(int stateCount) =>
        stateCount > 0
            ? stateCount
            : throw new ArgumentOutOfRangeException(nameof(stateCount), stateCount, "State count must be greater than 0 (SPEC §6.1).");
}
