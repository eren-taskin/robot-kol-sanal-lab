using System.Diagnostics;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;

namespace RobotArm.Tests;

/// <summary>
/// PROJECT.md §3 and SPEC §6.2 (no allocation inside Step): <see cref="ArmPlant.Derivatives"/>,
/// <see cref="Rk4Solver.Step"/>, <see cref="EulerSolver.Step"/> and <see cref="Simulation.Advance"/> allocate
/// 0 bytes after warm-up. Also reports the time per RK4 step; printed, not asserted, because machines differ.
/// </summary>
public sealed class SimulationAllocationTests(ITestOutputHelper output)
{
    private const int Calls = 10_000;
    private static readonly double[] s_x0 = [0.3, -0.5, 0.8, 0.2, 0.4, -1.2, 0.9, 1.5];
    private static readonly double[] s_u = [0.5, -5.0, 1.0, 0.2];

    private static ArmPlant NewPlant()
    {
        var plant = new ArmPlant(ArmModel.OpenManipulatorX(), Planet.Earth);
        plant.SetPayload(0.3);
        return plant;
    }

    [Fact]
    public void Derivatives_AllocateNothing_AfterWarmUp()
    {
        var plant = NewPlant();
        var x = (double[])s_x0.Clone();
        var dx = new double[8];
        for (var i = 0; i < Calls; i++) plant.Derivatives(0.0, x, s_u, dx);

        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < Calls; i++)
        {
            x[1] = -0.5 + 1e-5 * i;
            plant.Derivatives(0.0, x, s_u, dx);
        }
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        output.WriteLine(Invariant($"ArmPlant.Derivatives: {allocated} bytes in {Calls} calls"));
        Assert.Equal(0, allocated);
    }

    [Fact]
    public void Rk4Step_AllocatesNothing_AfterWarmUp_AndReportsTimePerStep()
    {
        var plant = NewPlant();
        var solver = new Rk4Solver(plant.StateCount);
        var x = (double[])s_x0.Clone();
        for (var i = 0; i < 2 * Calls; i++)
        {
            solver.Step(plant, 0.0, x, s_u, 1e-3);
            if (i % 100 == 0) s_x0.CopyTo(x, 0);
        }

        s_x0.CopyTo(x, 0);
        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < Calls; i++)
        {
            solver.Step(plant, i * 1e-3, x, s_u, 1e-3);
            if (i % 100 == 99) s_x0.CopyTo(x, 0);
        }
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(0, allocated);

        s_x0.CopyTo(x, 0);
        var sw = Stopwatch.StartNew();
        for (var i = 0; i < Calls; i++)
        {
            solver.Step(plant, i * 1e-3, x, s_u, 1e-3);
            if (i % 100 == 99) s_x0.CopyTo(x, 0);
        }
        sw.Stop();

#if DEBUG
        const string configuration = "Debug";
#else
        const string configuration = "Release";
#endif
        output.WriteLine(
            Invariant($"{configuration}: Rk4Solver.Step {sw.Elapsed.TotalMicroseconds / Calls:F3} µs/step (4 Derivatives calls), allocated {allocated} bytes in {Calls} steps"));
    }

    [Fact]
    public void EulerStepAndSimulationAdvance_AllocateNothing_AfterWarmUp()
    {
        var plant = NewPlant();
        var euler = new EulerSolver(plant.StateCount);
        var x = (double[])s_x0.Clone();
        var sim = new Simulation(NewPlant(), new Rk4Solver(8), s_x0, FaultChecks.All);
        double[] hold = [0.0, 0.0, 0.0, 0.0];
        for (var i = 0; i < Calls; i++)
        {
            euler.Step(plant, 0.0, x, s_u, 1e-4);
            s_x0.CopyTo(x, 0);
        }
        // Warm up Advance on a copy that is allowed to fault, then measure on a fresh one.
        var warm = new Simulation(NewPlant(), new Rk4Solver(8), s_x0, FaultChecks.All);
        for (var i = 0; i < 50 && !warm.IsStopped; i++) warm.Advance(hold, 1e-4);

        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < Calls; i++)
        {
            euler.Step(plant, 0.0, x, s_u, 1e-4);
            s_x0.CopyTo(x, 0);
        }
        for (var i = 0; i < 50 && !sim.IsStopped; i++) sim.Advance(hold, 1e-4);
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        output.WriteLine(Invariant($"EulerSolver.Step and Simulation.Advance: {allocated} bytes"));
        Assert.False(sim.IsStopped);
        Assert.Equal(0, allocated);
    }
}
