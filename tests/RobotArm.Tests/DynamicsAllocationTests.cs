using System.Diagnostics;
using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using Xunit.Abstractions;

namespace RobotArm.Tests;

/// <summary>
/// PROJECT.md §3 (no allocation in anything called once per simulation step) for the dynamics of SPEC §5.2:
/// InertiaMatrix, BiasTorque and the Cholesky solve allocate no managed memory. Also reports the time per
/// call; the timing is printed, not asserted, because CI machines differ.
/// </summary>
public sealed class DynamicsAllocationTests(ITestOutputHelper output)
{
    private const int Calls = 10_000;

    [Fact]
    public void InertiaMatrixAndBiasTorque_AllocateNothing_In10000Calls()
    {
        var rnea = new RecursiveNewtonEuler(ArmModel.OpenManipulatorX().WithPayload(0.3));
        var planet = Planet.Earth;
        double[] q = [0.3, -0.5, 0.8, 0.2];
        double[] qd = [0.4, -1.2, 0.9, 1.5];
        var m = new double[16];
        var h = new double[4];
        var rhs = new double[4];

        // Warm-up: JIT and tiering, static initialisers.
        for (var i = 0; i < 2 * Calls; i++)
        {
            rnea.InertiaMatrix(q, m);
            rnea.BiasTorque(q, qd, planet, h);
            Cholesky.FactorAndSolve(m, 4, h);
        }

        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < Calls; i++)
        {
            q[1] = -0.5 + 1e-5 * i;
            rnea.InertiaMatrix(q, m);
            rnea.BiasTorque(q, qd, planet, h);
            h.CopyTo(rhs, 0);
            Cholesky.FactorAndSolve(m, 4, rhs);
        }
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0, allocated);

        var swM = Stopwatch.StartNew();
        for (var i = 0; i < Calls; i++) rnea.InertiaMatrix(q, m);
        swM.Stop();
        var swH = Stopwatch.StartNew();
        for (var i = 0; i < Calls; i++) rnea.BiasTorque(q, qd, planet, h);
        swH.Stop();

#if DEBUG
        const string configuration = "Debug";
#else
        const string configuration = "Release";
#endif
        output.WriteLine(
            $"{configuration}: InertiaMatrix {swM.Elapsed.TotalMicroseconds / Calls:F3} µs/call, " +
            $"BiasTorque {swH.Elapsed.TotalMicroseconds / Calls:F3} µs/call, allocated {allocated} bytes in {Calls} iterations");
    }
}
