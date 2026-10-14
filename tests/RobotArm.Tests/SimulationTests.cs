using RobotArm.Core.Dynamics;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.SimulationTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §6.4 (faults), §5.5 (payload switch at a step boundary), §6.2 (step range) and V13 of SPEC §11
/// (statics), through <see cref="Simulation"/>.
/// </summary>
public sealed class SimulationTests(ITestOutputHelper output)
{
    private static readonly double[] s_zeroTorque = new double[4];

    /// <summary>Poses inside the limits of SPEC §3.4 with the tool above the floor.</summary>
    private static readonly double[][] s_staticPoses =
    [
        [0.0, -0.4, 0.2, 1.0],
        [0.3, -0.5, 0.9, -0.3],
        [-1.2, -0.8, 0.6, 0.8],
        [1.0, -0.6, -0.4, 1.2],
        [-2.8, -1.2, 0.4, 1.5],
    ];

    // Payload in grams so that the test names contain no decimal separator.
    public static TheoryData<string, int> PlanetsAndPayloads()
    {
        var data = new TheoryData<string, int>();
        foreach (var p in Planet.All)
            foreach (var grams in new[] { 0, 500 })
                data.Add(p.Key, grams);
        return data;
    }

    /// <remarks>
    /// Run at the default step h = 0.5 ms of SPEC §6.2. With the default ωs = 0.1 rad/s this is below the
    /// stability limit of every pose (checked here with <see cref="StabilityEstimate"/>); with the old
    /// ωs = 0.01 it was not, see <see cref="StepStabilityTests"/> and docs/DECISIONS.md D3.
    /// </remarks>
    [Theory]
    [MemberData(nameof(PlanetsAndPayloads))]
    public void V13_GravityTorque_HoldsArmStillFor1s_WithFriction(string planetKey, int payloadGrams)
    {
        // τ = g(q) from the closed form of SPEC §5.3, independent of the plant's RNEA; friction on (defaults).
        const double h = Simulation.DefaultStep;
        var payloadKg = payloadGrams / 1000.0;
        var planet = Planet.All.Single(p => p.Key == planetKey);
        var arm = ArmModel.OpenManipulatorX();
        var closed = new ClosedFormArm(arm.WithPayload(payloadKg));
        var worst = 0.0;

        foreach (var q in s_staticPoses)
        {
            var g = closed.Gravity(q, planet.G);
            Assert.True(DynamicsTestHelpers.MaxAbs(g) < arm.Actuator.StallTorque,
                Invariant($"pose ({q[0]}, {q[1]}, {q[2]}, {q[3]}) needs {DynamicsTestHelpers.MaxAbs(g):F3} N·m > τmax"));
            for (var i = 0; i < 4; i++)
                Assert.True(arm.Joints[i].IsWithinLimits(q[i]));
            Assert.True(ForwardKinematics.ClosedForm(q).Position.Z > 0.0);
            Assert.True(arm.Actuator.ViscousFriction > 0.0 && arm.Actuator.CoulombFriction > 0.0);
            Assert.True(h < StabilityEstimate.MaxStableStep(arm.WithPayload(payloadKg), q));

            var plant = new ArmPlant(arm, planet);
            plant.SetPayload(payloadKg);
            var sim = new Simulation(plant, new Rk4Solver(8), [.. q, 0.0, 0.0, 0.0, 0.0]);
            var steps = Steps(1.0, h);
            for (var step = 0; step < steps; step++)
            {
                Assert.True(sim.Advance(g, h));
                worst = Math.Max(worst, DynamicsTestHelpers.MaxAbs(sim.QDot));
            }
            Assert.Equal([false, false, false, false], sim.Saturated.ToArray());
        }

        output.WriteLine(Invariant($"V13 {planetKey}, {payloadKg:F1} kg, {s_staticPoses.Length} poses, 1 s at h = 0.5 ms: max |q̇| = {worst:E3} rad/s"));
        Assert.True(worst < 1e-9, Invariant($"max |q̇| = {worst:E3}"));
    }

    [Fact]
    public void Fault_JointLimit_StopsWithJointAndAngle()
    {
        // Orbit (no gravity), joint 2 swings back towards its lower limit −1.5 rad; the tool stays high.
        var plant = new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Orbit);
        var sim = new Simulation(plant, new Rk4Solver(8), [0.0, -1.4, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0]);

        var steps = RunUntilFault(sim, 1e-3, 1000);

        var fault = Assert.IsType<Fault>(sim.Fault);
        Assert.Equal(FaultReason.JointLimit, fault.Reason);
        Assert.Equal(1, fault.Joint);
        Assert.True(fault.Value < -1.5);
        Assert.Equal(sim.Time, fault.Time);
        output.WriteLine(Invariant($"Joint-limit fault after {steps} steps, t = {fault.Time:F3} s, q2 = {fault.Value:F5} rad"));
    }

    [Fact]
    public void Fault_Floor_StopsWithToolHeight()
    {
        // Earth, no torque, from q = (0.3, 0.30, 0.25, 0.45): tool 3.9 mm above the floor, inside the limits.
        var plant = new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Earth);
        double[] q0 = [0.3, 0.30, 0.25, 0.45];
        Assert.InRange(ForwardKinematics.ClosedForm(q0).Position.Z, 0.0, 0.01);
        var sim = new Simulation(plant, new Rk4Solver(8), [.. q0, 0.0, 0.0, 0.0, 0.0]);

        var steps = RunUntilFault(sim, 1e-3, 1000);

        var fault = Assert.IsType<Fault>(sim.Fault);
        Assert.Equal(FaultReason.Floor, fault.Reason);
        Assert.Equal(-1, fault.Joint);
        Assert.True(fault.Value < 0.0);
        Assert.Equal(fault.Value, ForwardKinematics.ClosedForm(sim.Q).Position.Z, 1e-12);
        for (var i = 0; i < 4; i++)
            Assert.True(plant.Model.Joints[i].IsWithinLimits(sim.Q[i]));
        output.WriteLine(Invariant($"Floor fault after {steps} steps, t = {fault.Time:F3} s, tool z = {fault.Value * 1000:F3} mm"));
    }

    [Fact]
    public void Fault_Floor_IsNotReported_WhenTheTaskAllowsIt()
    {
        var plant = new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Earth);
        var sim = new Simulation(plant, new Rk4Solver(8), [0.3, 0.30, 0.25, 0.45, 0.0, 0.0, 0.0, 0.0],
            FaultChecks.All & ~FaultChecks.Floor);

        Run(sim, s_zeroTorque, 1e-3, 100);

        Assert.True(ForwardKinematics.ClosedForm(sim.Q).Position.Z < 0.0);
        Assert.Null(sim.Fault);
    }

    [Fact]
    public void Fault_NonFinite_StopsWhenStateBecomesNaN()
    {
        var plant = new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Earth);
        var sim = new Simulation(plant, new Rk4Solver(8), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

        Assert.False(sim.Advance([double.NaN, 0.0, 0.0, 0.0], 1e-3));

        var fault = Assert.IsType<Fault>(sim.Fault);
        Assert.Equal(FaultReason.NonFinite, fault.Reason);
        Assert.Equal(1e-3, fault.Time);
        Assert.Contains(sim.State.ToArray(), double.IsNaN);
    }

    [Fact]
    public void Fault_StopsTheSimulation_FurtherAdvanceThrows()
    {
        var plant = new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Earth);
        var sim = new Simulation(plant, new Rk4Solver(8), new double[8]);
        sim.Advance([0.0, double.NaN, 0.0, 0.0], 1e-3);
        var time = sim.Time;
        var state = sim.State.ToArray();

        Assert.True(sim.IsStopped);
        var ex = Assert.Throws<InvalidOperationException>(() => sim.Advance(s_zeroTorque, 1e-3));
        Assert.Contains("t = 0.001 s", ex.Message);
        Assert.Equal(time, sim.Time);
        Assert.Equal(state, sim.State.ToArray());
    }

    [Fact]
    public void Fault_InfiniteCommand_IsSaturated_NotAFault()
    {
        var sim = new Simulation(new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Earth), new Rk4Solver(8), new double[8]);
        Assert.True(sim.Advance([double.PositiveInfinity, 0.0, 0.0, 0.0], 1e-3));
        Assert.Equal(4.1, sim.AppliedTorque[0]);
        Assert.True(sim.Saturated[0]);
    }

    [Fact]
    public void FaultChecksNone_NeverStops()
    {
        var plant = new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Earth);
        var sim = new Simulation(plant, new Rk4Solver(8), new double[8], FaultChecks.None);
        Assert.True(sim.Advance([double.NaN, 0.0, 0.0, 0.0], 1e-3));
        Assert.Null(sim.Fault);
        Assert.Contains(sim.State.ToArray(), double.IsNaN);
        Assert.True(sim.Advance([0.0, 0.0, 0.0, 0.0], 1e-3));
    }

    [Fact]
    public void Payload_AttachAndDetach_SwitchParameters_StateUnchanged()
    {
        var arm = ArmModel.OpenManipulatorX();
        var plant = new ArmPlant(arm, Planet.Earth);
        var sim = new Simulation(plant, new Rk4Solver(8), [0.2, 0.3, 0.4, 0.1, 0.5, -0.2, 0.3, 0.1]);
        double[] u = [0.0, -0.8, -0.4, -0.06];
        Run(sim, u, 1e-3, 50);
        var before = sim.State.ToArray();
        var timeBefore = sim.Time;

        sim.AttachPayload(0.3);

        Assert.Equal(before, sim.State.ToArray());
        Assert.Equal(timeBefore, sim.Time);
        Assert.Equal(0.3, sim.PayloadKg);
        Assert.Equal(arm.WithPayload(0.3), plant.Model);
        Assert.Same(arm, plant.BaseModel);

        // The parameters jumped: the next step differs from a run without the payload.
        var withPayload = NextState(sim, u);
        sim.DetachPayload();
        Assert.Equal(0.0, sim.PayloadKg);
        Assert.Equal(arm, plant.Model);
        var reference = new Simulation(new ArmPlant(arm, Planet.Earth), new Rk4Solver(8), before);
        reference.Advance(u, 1e-3);
        Assert.True(MaxAbsDiff(withPayload, reference.State) > 1e-6);

        // And it equals a run that had the payload attached from the start of this step.
        var loaded = new ArmPlant(arm, Planet.Earth);
        loaded.SetPayload(0.3);
        var check = new Simulation(loaded, new Rk4Solver(8), before);
        check.Advance(u, 1e-3);
        Assert.Equal(check.State.ToArray(), withPayload);
    }

    [Theory]
    [InlineData(0.99e-4)]
    [InlineData(5.01e-3)]
    [InlineData(0.0)]
    [InlineData(double.NaN)]
    public void Advance_RejectsStepOutsideSpecRange(double h)
    {
        var sim = new Simulation(new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Earth), new Rk4Solver(8), new double[8]);
        Assert.Throws<ArgumentOutOfRangeException>(() => sim.Advance(s_zeroTorque, h));
    }

    [Fact]
    public void Constructor_CopiesInitialState()
    {
        var x0 = new double[8];
        var sim = new Simulation(new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Earth), new Rk4Solver(8), x0);
        x0[1] = 1.0;
        Assert.Equal(0.0, sim.State[1]);
    }

    [Fact]
    public void Solver_RejectsPlantOfDifferentSize()
    {
        var plant = new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Earth);
        Assert.Throws<ArgumentException>(() => new Rk4Solver(6).Step(plant, 0.0, new double[8], new double[4], 1e-3));
        Assert.Throws<ArgumentException>(() => new EulerSolver(6).Step(plant, 0.0, new double[8], new double[4], 1e-3));
    }

    private static double[] NextState(Simulation sim, double[] u)
    {
        sim.Advance(u, 1e-3);
        return sim.State.ToArray();
    }

    private static int RunUntilFault(Simulation sim, double h, int maxSteps)
    {
        for (var i = 1; i <= maxSteps; i++)
            if (!sim.Advance(s_zeroTorque, h))
                return i;
        Assert.Fail(Invariant($"no fault within {maxSteps} steps"));
        return -1;
    }
}
