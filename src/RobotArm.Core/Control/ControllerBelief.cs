using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;

namespace RobotArm.Core.Control;

/// <summary>
/// A controller's own model of the arm (its "belief"): the arm, the planet and the payload it assumes. SPEC §5.6.
/// It may differ from the true plant (wrong payload, wrong g, other parameters); that is how the model-mismatch
/// experiments are made.
/// </summary>
public sealed class ControllerBelief
{
    /// <summary>Creates the belief. SPEC §5.6.</summary>
    /// <param name="model">The arm the controller assumes, without payload.</param>
    /// <param name="planet">The planet the controller assumes.</param>
    /// <param name="payloadKg">The payload the controller assumes at the tool point, kg (SPEC §5.5).</param>
    /// <exception cref="ArgumentNullException">The model or planet is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The payload is negative or not finite.</exception>
    public ControllerBelief(ArmModel model, Planet planet, double payloadKg = 0.0)
    {
        ArgumentNullException.ThrowIfNull(model);
        ArgumentNullException.ThrowIfNull(planet);
        BaseModel = model;
        Planet = planet;
        Model = model.WithPayload(payloadKg);
        PayloadKg = payloadKg;
    }

    /// <summary>The assumed arm without payload.</summary>
    public ArmModel BaseModel { get; }

    /// <summary>The assumed arm with the assumed payload: the model the controller computes with. SPEC §5.5.</summary>
    public ArmModel Model { get; }

    /// <summary>The assumed planet. SPEC §3.5.</summary>
    public Planet Planet { get; }

    /// <summary>The assumed payload, kg.</summary>
    public double PayloadKg { get; }

    /// <summary>Number of joints n.</summary>
    public int DegreesOfFreedom => Model.DegreesOfFreedom;

    /// <summary>The exact belief for the default arm: OpenMANIPULATOR-X on <paramref name="planet"/> with <paramref name="payloadKg"/>.</summary>
    public static ControllerBelief OpenManipulatorX(Planet planet, double payloadKg = 0.0) =>
        new(ArmModel.OpenManipulatorX(), planet, payloadKg);

    /// <summary>τmax of each joint in the assumed model, N·m (SPEC §3.3). The controller clamps its output to ±τmax.</summary>
    internal double[] TorqueLimits()
    {
        var limits = new double[DegreesOfFreedom];
        for (var i = 0; i < limits.Length; i++)
            limits[i] = Model.Actuators[i].StallTorque;
        return limits;
    }

    /// <summary>
    /// G = ∂g/∂q of the assumed model at <paramref name="q0"/> by central differences of the bias torque with
    /// q̇ = 0 (step 1e-6 rad), n × n row-major, column j = ∂g/∂qj. SPEC §7; docs/DECISIONS.md D6. Allocates.
    /// </summary>
    internal double[] GravityGradient(ReadOnlySpan<double> q0) =>
        LinearSystem.GravityGradient(new RecursiveNewtonEuler(Model), Planet.GravityVector, q0);
}
