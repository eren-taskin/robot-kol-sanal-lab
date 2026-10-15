using RobotArm.Core.Dynamics;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.DynamicsTestHelpers;
using static RobotArm.Tests.SimulationTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §4.1 and §5.2 for a chain that is not the OpenMANIPULATOR-X: the general frame chain, RNEA, the
/// inertia matrix, the plant and the solver on the 6-DOF test chain of <see cref="SixJointChain"/>
/// (roll axes, off-axis centres of mass, full inertia tensors, per-joint actuators). None of these checks
/// uses a closed form of the 4-DOF arm.
/// </summary>
public sealed class SixJointChainTests(ITestOutputHelper output)
{
    private const int N = 6;
    private static readonly ArmModel s_chain = SixJointChain.Create();

    [Fact]
    public void Chain_HasRollAxes_NonPlanarBodies_AndPerJointActuators()
    {
        Assert.Equal(N, s_chain.DegreesOfFreedom);
        Assert.Equal(
            new[] { JointAxis.Z, JointAxis.Y, JointAxis.Y, JointAxis.X, JointAxis.Y, JointAxis.X },
            s_chain.Joints.Select(j => j.Axis));
        foreach (var b in s_chain.Bodies)
        {
            Assert.False(b.IsPlanar);
            Assert.NotEqual(0.0, b.CenterOfMass.Y);
            Assert.NotEqual(0.0, b.Ixy);
            Assert.NotEqual(0.0, b.Ixz);
            Assert.NotEqual(0.0, b.Iyz);
            Assert.Equal(b.Inertia.Transpose(), b.Inertia);
        }
        Assert.Equal(N, s_chain.Actuators.Count);
        Assert.NotEqual(s_chain.Actuators[0], s_chain.Actuators[5]);
        Assert.NotEqual(s_chain.Actuators[2], s_chain.Actuators[3]);
    }

    [Fact]
    public void WithPayload_NonPlanarBody_ThrowsNotSupported()
    {
        var ex = Assert.Throws<NotSupportedException>(() => s_chain.WithPayload(0.1));
        Assert.Contains("planar", ex.Message);
        Assert.Same(s_chain, s_chain.WithPayload(0.0));
    }

    [Fact]
    public void KineticEnergy_EqualsSumOverBodies_FromFrameVelocities_In100RandomStates()
    {
        // ½ q̇ᵀ M q̇ = Σ (½ m v² + ½ ωᵀ I ω) + Σ ½ N²Jm q̇², with v (centre of mass, base frame) and ω (body frame)
        // from ComputeFrames by central differences along q̇. This checks M against the frame chain, independently
        // of RNEA's own kinematics.
        const double eps = 1e-6;
        var rnea = new RecursiveNewtonEuler(s_chain);
        var random = new Random(6061);
        var m = new double[N * N];
        var frames = new Frame[N];
        var plus = new Frame[N];
        var minus = new Frame[N];
        var qPlus = new double[N];
        var qMinus = new double[N];
        var worst = 0.0;
        var worstAbsolute = 0.0;

        for (var trial = 0; trial < 100; trial++)
        {
            var q = RandomVector(random, N, Math.PI);
            var qd = RandomVector(random, N, 2.0);
            rnea.InertiaMatrix(q, m);
            var fromM = 0.0;
            for (var r = 0; r < N; r++)
                for (var c = 0; c < N; c++)
                    fromM += 0.5 * qd[r] * m[r * N + c] * qd[c];

            for (var i = 0; i < N; i++)
            {
                qPlus[i] = q[i] + eps * qd[i];
                qMinus[i] = q[i] - eps * qd[i];
            }
            ForwardKinematics.ComputeFrames(s_chain, q, frames);
            ForwardKinematics.ComputeFrames(s_chain, qPlus, plus);
            ForwardKinematics.ComputeFrames(s_chain, qMinus, minus);

            var fromBodies = 0.0;
            for (var i = 0; i < N; i++)
            {
                var body = s_chain.Bodies[i];
                var v = (plus[i].TransformPoint(body.CenterOfMass) - minus[i].TransformPoint(body.CenterOfMass)) / (2.0 * eps);
                var rDot = (1.0 / (2.0 * eps)) * (plus[i].Rotation - minus[i].Rotation);
                var omega = Vee(frames[i].Rotation.Transpose() * rDot);
                fromBodies += 0.5 * body.Mass * v.LengthSquared + 0.5 * Vector3d.Dot(omega, body.Inertia * omega);
                fromBodies += 0.5 * s_chain.Actuators[i].ReflectedRotorInertia * qd[i] * qd[i];
            }

            worst = Math.Max(worst, Math.Abs(fromM - fromBodies) / fromM);
            worstAbsolute = Math.Max(worstAbsolute, Math.Abs(fromM - fromBodies));
        }

        output.WriteLine(Invariant($"6-DOF kinetic energy, 100 random states: largest mismatch {worstAbsolute:E3} J, relative {worst:E3}"));
        Assert.True(worst < 1e-8, Invariant($"{worst:E3}"));
    }

    [Fact]
    public void Inverse_EqualsInertiaTimesQddPlusBias_InRandomStates()
    {
        var rnea = new RecursiveNewtonEuler(s_chain);
        var random = new Random(6062);
        var m = new double[N * N];
        var h = new double[N];
        var tau = new double[N];
        var worst = 0.0;
        Vector3d[] gravities = [Planet.Earth.GravityVector, new(1.2, -0.7, -9.7)];

        for (var trial = 0; trial < 200; trial++)
        {
            var gravity = gravities[trial % 2];
            var q = RandomVector(random, N, Math.PI);
            var qd = RandomVector(random, N, 2.0);
            var qdd = RandomVector(random, N, 5.0);
            rnea.Inverse(q, qd, qdd, gravity, tau);
            rnea.InertiaMatrix(q, m);
            rnea.BiasTorque(q, qd, gravity, h);

            var expected = new double[N];
            for (var r = 0; r < N; r++)
            {
                expected[r] = h[r];
                for (var c = 0; c < N; c++) expected[r] += m[r * N + c] * qdd[c];
            }
            worst = Math.Max(worst, MaxAbsDifference(expected, tau) / MaxAbs(tau));
        }

        output.WriteLine(Invariant($"6-DOF τ vs M q̈ + h, 200 random states: largest relative difference {worst:E3}"));
        Assert.True(worst < 1e-12, Invariant($"{worst:E3}"));
    }

    [Fact]
    public void BiasTorque_MatchesLagrangeEquations_ByCentralDifferences()
    {
        // Independent of RNEA's recursion: g(q) = ∂V/∂q with V = Σ m g z from the frame chain, and the velocity
        // terms C q̇ = Ṁ q̇ − ½ ∂(q̇ᵀ M q̇)/∂q with M differentiated numerically (SPEC §5.1, report eq. 13).
        const double eps = 1e-6;
        var rnea = new RecursiveNewtonEuler(s_chain);
        var random = new Random(6063);
        var g = Planet.Earth.G;
        var worstGravity = 0.0;
        var worstVelocity = 0.0;

        for (var trial = 0; trial < 50; trial++)
        {
            var q = RandomVector(random, N, Math.PI);
            var qd = RandomVector(random, N, 2.0);

            var gravity = new double[N];
            rnea.BiasTorque(q, new double[N], Planet.Earth, gravity);
            var velocityTerms = new double[N];
            rnea.BiasTorque(q, qd, Vector3d.Zero, velocityTerms);

            var gradV = new double[N];
            var gradT = new double[N];
            for (var k = 0; k < N; k++)
            {
                var qp = (double[])q.Clone();
                var qm = (double[])q.Clone();
                qp[k] += eps;
                qm[k] -= eps;
                gradV[k] = (Potential(qp, g) - Potential(qm, g)) / (2.0 * eps);
                gradT[k] = (Kinetic(rnea, qp, qd) - Kinetic(rnea, qm, qd)) / (2.0 * eps);
            }

            var qpDir = new double[N];
            var qmDir = new double[N];
            for (var i = 0; i < N; i++)
            {
                qpDir[i] = q[i] + eps * qd[i];
                qmDir[i] = q[i] - eps * qd[i];
            }
            var mp = new double[N * N];
            var mm = new double[N * N];
            rnea.InertiaMatrix(qpDir, mp);
            rnea.InertiaMatrix(qmDir, mm);
            var cqd = new double[N];
            for (var r = 0; r < N; r++)
            {
                for (var c = 0; c < N; c++) cqd[r] += (mp[r * N + c] - mm[r * N + c]) / (2.0 * eps) * qd[c];
                cqd[r] -= gradT[r];
            }

            worstGravity = Math.Max(worstGravity, MaxAbsDifference(gradV, gravity) / MaxAbs(gravity));
            worstVelocity = Math.Max(worstVelocity, MaxAbsDifference(cqd, velocityTerms) / MaxAbs(velocityTerms));
        }

        output.WriteLine(Invariant($"6-DOF g(q) vs ∂V/∂q: {worstGravity:E3}; C q̇ vs Lagrange: {worstVelocity:E3} (relative, 50 states)"));
        Assert.True(worstGravity < 1e-7, Invariant($"{worstGravity:E3}"));
        Assert.True(worstVelocity < 1e-6, Invariant($"{worstVelocity:E3}"));
    }

    [Fact]
    public void FreeMotion_WithoutFriction_ConservesEnergy_DriftBelow1e5JIn04sAtHalfMs()
    {
        const double h = 5e-4;
        var chain = SixJointChain.Create(friction: false);
        var plant = new ArmPlant(chain, Planet.Earth);
        Assert.Equal(12, plant.StateCount);
        double[] x0 = [0.3, -0.5, 0.8, 0.4, -0.6, 0.2, 1.0, -0.8, 1.2, 2.0, -1.5, 2.5];
        var sim = new Simulation(plant, new Rk4Solver(plant.StateCount), x0, FaultChecks.None);
        var rnea = new RecursiveNewtonEuler(chain);
        var e0 = Energy(chain, rnea, Planet.Earth.G, sim.State);
        var u = new double[N];
        var worst = 0.0;
        var moved = 0.0;

        for (var i = 0; i < Steps(0.4, h); i++)
        {
            Assert.True(sim.Advance(u, h));
            worst = Math.Max(worst, Math.Abs(Energy(chain, rnea, Planet.Earth.G, sim.State) - e0));
            moved = Math.Max(moved, MaxAbsDiff(sim.Q, x0.AsSpan(0, N)));
        }

        output.WriteLine(Invariant($"6-DOF free motion, E0 = {e0:F6} J: max |E − E0| over 0.4 s at h = 0.5 ms = {worst:E3} J; max |q − q0| = {moved:F3} rad"));
        Assert.True(moved > 0.1, Invariant($"the state hardly moved: {moved:E3} rad"));
        Assert.True(worst < 1e-5, Invariant($"energy drift {worst:E3} J"));
    }

    [Fact]
    public void InertiaMatrix_IsSymmetricPositiveDefinite_AndRawColumnsSymmetric_In1000RandomPoses()
    {
        var rnea = new RecursiveNewtonEuler(s_chain);
        var random = new Random(6064);
        var m = new double[N * N];
        var raw = new double[N * N];
        var worstAsymmetry = 0.0;

        for (var trial = 0; trial < 1000; trial++)
        {
            var q = RandomVector(random, N, Math.PI);
            rnea.InertiaColumns(q, raw);
            for (var r = 0; r < N; r++)
                for (var c = 0; c < r; c++)
                    worstAsymmetry = Math.Max(worstAsymmetry, Math.Abs(raw[r * N + c] - raw[c * N + r]));

            rnea.InertiaMatrix(q, m);
            var x = RandomVector(random, N, 1.0);
            var quadratic = 0.0;
            for (var r = 0; r < N; r++)
                for (var c = 0; c < N; c++)
                    quadratic += x[r] * m[r * N + c] * x[c];
            Assert.True(quadratic > 0.0, Invariant($"xᵀMx = {quadratic:E3}"));
            Cholesky.Factor(m, N);
        }

        output.WriteLine(Invariant($"6-DOF raw RNEA columns, 1000 poses: largest |M_rc − M_cr| = {worstAsymmetry:E3}"));
        Assert.True(worstAsymmetry < 1e-12, Invariant($"{worstAsymmetry:E3}"));
    }

    [Fact]
    public void StabilityEstimate_WorksOnTheSixJointChain()
    {
        var h = StabilityEstimate.MaxStableStep(s_chain, [0.3, -0.5, 0.8, 0.4, -0.6, 0.2]);
        output.WriteLine(Invariant($"6-DOF MaxStableStep at the free-motion start pose: {h * 1e3:F4} ms"));
        Assert.True(double.IsFinite(h) && h > 0.0);
    }

    /// <summary>ω from the skew-symmetric S = RᵀṘ: (S21, S02, S10), averaged with the opposite entries.</summary>
    private static Vector3d Vee(Matrix3d s) =>
        new(0.5 * (s.M21 - s.M12), 0.5 * (s.M02 - s.M20), 0.5 * (s.M10 - s.M01));

    private static double Potential(ReadOnlySpan<double> q, double g)
    {
        var frames = new Frame[N];
        ForwardKinematics.ComputeFrames(s_chain, q, frames);
        var v = 0.0;
        for (var i = 0; i < N; i++)
            v += s_chain.Bodies[i].Mass * g * frames[i].TransformPoint(s_chain.Bodies[i].CenterOfMass).Z;
        return v;
    }

    private static double Kinetic(RecursiveNewtonEuler rnea, ReadOnlySpan<double> q, ReadOnlySpan<double> qd)
    {
        var m = new double[N * N];
        rnea.InertiaMatrix(q, m);
        var t = 0.0;
        for (var r = 0; r < N; r++)
            for (var c = 0; c < N; c++)
                t += 0.5 * qd[r] * m[r * N + c] * qd[c];
        return t;
    }
}
