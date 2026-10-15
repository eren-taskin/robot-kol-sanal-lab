namespace RobotArm.Core.Simulation;

/// <summary>Why a simulation stopped. SPEC §6.4.</summary>
public enum FaultReason
{
    /// <summary>No fault.</summary>
    None,

    /// <summary>The state contains NaN or infinity.</summary>
    NonFinite,

    /// <summary>A joint angle left its limits (SPEC §3.4).</summary>
    JointLimit,

    /// <summary>The tool point went below z = 0.</summary>
    Floor,
}

/// <summary>Which fault checks of SPEC §6.4 a <see cref="Simulation"/> runs after each step.</summary>
[Flags]
public enum FaultChecks
{
    /// <summary>
    /// No optional checks: joint limits and floor are not checked. A non-finite state still stops the
    /// simulation (docs/DECISIONS.md D5).
    /// </summary>
    None = 0,

    /// <summary>
    /// NaN or infinity in the state. Kept for completeness; this check always runs, set or not, because a
    /// non-finite state is never valid (docs/DECISIONS.md D5).
    /// </summary>
    NonFinite = 1,

    /// <summary>Joint angles outside the limits of SPEC §3.4.</summary>
    JointLimit = 2,

    /// <summary>Tool point below z = 0; leave it out when the task allows it (SPEC §6.4).</summary>
    Floor = 4,

    /// <summary>Every check of SPEC §6.4 (the default).</summary>
    All = NonFinite | JointLimit | Floor,
}

/// <summary>A fault that stopped the simulation. SPEC §6.4.</summary>
/// <param name="Reason">What happened.</param>
/// <param name="Time">Simulated time at the end of the step that produced the fault, s.</param>
/// <param name="Joint">Zero-based joint index for <see cref="FaultReason.JointLimit"/>, otherwise −1.</param>
/// <param name="Value">
/// The offending number: the joint angle in rad (JointLimit) or the tool height z in m (Floor); NaN for NonFinite.
/// </param>
public sealed record Fault(FaultReason Reason, double Time, int Joint, double Value);
