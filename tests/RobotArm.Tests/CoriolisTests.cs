using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using Xunit.Abstractions;
using static RobotArm.Tests.DynamicsTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §5.3 Coriolis matrix from the Christoffel symbols (V8 of SPEC §11), and the velocity part of the
/// general bias torque against it.
/// </summary>
public sealed class CoriolisTests(ITestOutputHelper output)
{
    private static readonly ArmModel s_arm = ArmModel.OpenManipulatorX();

    /// <summary>
    /// V8: zᵀ(Ṁ − 2C)z = 0 for 200 seeded random (q, q̇, z). C is the closed-form Christoffel matrix;
    /// Ṁ = Σi ∂M/∂q_i q̇i by central differences (step 1e-6) of the GENERAL M, so the test also ties the
    /// general algorithm to the closed-form C. Tolerance 1e-6 · |z|² · |q̇| · max|∂M/∂q|.
    /// </summary>
    [Fact]
    public void MdotMinusTwoC_IsSkewSymmetric_In200RandomStates()
    {
        var rnea = new RecursiveNewtonEuler(s_arm);
        var closed = new ClosedFormArm(s_arm);
        var random = new Random(8);
        var worst = 0.0;

        for (var trial = 0; trial < 200; trial++)
        {
            var q = RandomVector(random, 4, Math.PI);
            var qd = RandomVector(random, 4, 3.0);
            var z = RandomVector(random, 4, 1.0);

            var dM = ClosedFormArm.CentralDifferences(q, x => GeneralInertia(rnea, x));
            var c = closed.Coriolis(q, qd);

            var maxDm = dM.Max(MaxAbs);
            var value = 0.0;
            for (var r = 0; r < 4; r++)
                for (var k = 0; k < 4; k++)
                {
                    var mdot = 0.0;
                    for (var i = 0; i < 4; i++)
                        mdot += dM[i][r, k] * qd[i];
                    value += z[r] * (mdot - 2.0 * c[r, k]) * z[k];
                }

            var scale = Norm2(z) * Norm2(z) * Norm2(qd) * maxDm;
            worst = Math.Max(worst, Math.Abs(value) / scale);
            Assert.True(Math.Abs(value) <= 1e-6 * scale, $"zᵀ(Ṁ − 2C)z = {value:E3}, scale {scale:E3}");
        }
        output.WriteLine($"V8 max |zᵀ(Ṁ − 2C)z| / (|z|²|q̇| max|∂M/∂q|) over 200 states = {worst:E3}");
    }

    /// <summary>
    /// With gravity zero the general bias torque is C(q, q̇) q̇ alone; it must equal the closed-form
    /// Christoffel C times q̇. The only error is the central-difference error of ∂M/∂q.
    /// </summary>
    [Fact]
    public void BiasTorque_WithoutGravity_EqualsClosedFormCoriolisTimesSpeed()
    {
        var rnea = new RecursiveNewtonEuler(s_arm);
        var closed = new ClosedFormArm(s_arm);
        var random = new Random(9);
        var h = new double[4];
        var worst = 0.0;

        for (var trial = 0; trial < 200; trial++)
        {
            var q = RandomVector(random, 4, Math.PI);
            var qd = RandomVector(random, 4, 3.0);
            rnea.BiasTorque(q, qd, Vector3d.Zero, h);

            var c = closed.Coriolis(q, qd);
            var expected = new double[4];
            for (var k = 0; k < 4; k++)
                for (var j = 0; j < 4; j++)
                    expected[k] += c[k, j] * qd[j];

            var rel = MaxAbsDifference(expected, h) / MaxAbs(expected);
            worst = Math.Max(worst, rel);
            Assert.True(rel <= 1e-6, $"q = [{string.Join(", ", q)}], q̇ = [{string.Join(", ", qd)}]: relative difference {rel:E3}");
        }
        output.WriteLine($"C(q, q̇)q̇ closed form vs general bias torque (g = 0), 200 states: max relative difference = {worst:E3}");
    }

    private static double Norm2(double[] v) => Math.Sqrt(v.Sum(x => x * x));
}
