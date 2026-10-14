using RobotArm.Core.Dynamics;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;

namespace RobotArm.Tests;

/// <summary>Shared helpers for the simulation tests of SPEC §6 and V9–V15 of SPEC §11.</summary>
internal static class SimulationTestHelpers
{
    /// <summary>The OpenMANIPULATOR-X with fv = fc = 0 and Jm = 0, as V9–V12 require.</summary>
    public static ArmModel FrictionlessArm()
    {
        var arm = ArmModel.OpenManipulatorX();
        return new ArmModel(arm.Joints, arm.Bodies, arm.ToolPoint,
            arm.Actuator with { ViscousFriction = 0.0, CoulombFriction = 0.0, RotorInertia = 0.0 });
    }

    /// <summary>A simulation of the free-motion run of the vectors: Earth, RK4, faults off.</summary>
    public static Simulation FreeMotion(FaultChecks checks = FaultChecks.None)
    {
        var fm = TestVectors.Instance.FreeMotion;
        var plant = new ArmPlant(FrictionlessArm(), Planet.Earth);
        return new Simulation(plant, new Rk4Solver(plant.StateCount), fm.X0, checks);
    }

    /// <summary>Advances <paramref name="sim"/> by <paramref name="steps"/> steps of <paramref name="h"/> with u held.</summary>
    public static void Run(Simulation sim, ReadOnlySpan<double> u, double h, int steps)
    {
        for (var i = 0; i < steps; i++)
            sim.Advance(u, h);
    }

    /// <summary>Number of steps of length h in T, checked to be an integer.</summary>
    public static int Steps(double duration, double h)
    {
        var n = (int)Math.Round(duration / h);
        Assert.True(Math.Abs(n * h - duration) < 1e-12, FormattableString.Invariant($"{duration} is not a multiple of {h}"));
        return n;
    }

    /// <summary>
    /// Mechanical energy ½ q̇ᵀ M(q) q̇ + Σ mi g zi of <paramref name="model"/> on <paramref name="g"/>, with M from
    /// the general algorithm and the centre-of-mass heights from the frame chain (SPEC §4.1, §5.2).
    /// </summary>
    public static double Energy(ArmModel model, RecursiveNewtonEuler rnea, double g, ReadOnlySpan<double> x)
    {
        var n = model.DegreesOfFreedom;
        var q = x[..n];
        var qd = x[n..];
        var m = new double[n * n];
        rnea.InertiaMatrix(q, m);
        var kinetic = 0.0;
        for (var r = 0; r < n; r++)
            for (var c = 0; c < n; c++)
                kinetic += 0.5 * qd[r] * m[r * n + c] * qd[c];

        var frames = new Frame[n];
        ForwardKinematics.ComputeFrames(model, q, frames);
        var potential = 0.0;
        for (var i = 0; i < n; i++)
            potential += model.Bodies[i].Mass * g * frames[i].TransformPoint(model.Bodies[i].CenterOfMass).Z;
        return kinetic + potential;
    }

    /// <summary>First row of M(q) times q̇: the vertical angular momentum p1 = Σ M1j q̇j. Report §3.7.</summary>
    public static double YawMomentum(RecursiveNewtonEuler rnea, ReadOnlySpan<double> x)
    {
        var n = rnea.DegreesOfFreedom;
        var m = new double[n * n];
        rnea.InertiaMatrix(x[..n], m);
        var p = 0.0;
        for (var j = 0; j < n; j++)
            p += m[j] * x[n + j];
        return p;
    }

    /// <summary>Largest |a_i − b_i|.</summary>
    public static double MaxAbsDiff(ReadOnlySpan<double> a, ReadOnlySpan<double> b) =>
        DynamicsTestHelpers.MaxAbsDifference(a, b);
}
