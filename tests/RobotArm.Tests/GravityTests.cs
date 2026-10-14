using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;
using Xunit.Abstractions;
using static RobotArm.Tests.DynamicsTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §5.2–§5.3 (V7 of SPEC §11): the bias torque with q̇ = 0 equals the closed-form gravity torque
/// g(q) for gravity (0, 0, −g) on every planet of SPEC §3.5, and equals <c>gravity_earth</c> in the vectors.
/// </summary>
public sealed class GravityTests(ITestOutputHelper output)
{
    private static readonly ArmModel s_arm = ArmModel.OpenManipulatorX();
    private static readonly double[] s_zero = new double[4];

    public static TheoryData<string> PlanetKeys()
    {
        var data = new TheoryData<string>();
        foreach (var p in Planet.All) data.Add(p.Key);
        return data;
    }

    public static TheoryData<int> UnloadedPoseIndices() => InertiaMatrixTests.UnloadedPoseIndices();

    [Theory]
    [MemberData(nameof(PlanetKeys))]
    public void BiasTorque_ZeroSpeed_MatchesClosedFormGravity_InRandomPoses(string planetKey)
    {
        var planet = Planet.All.Single(p => p.Key == planetKey);
        var rnea = new RecursiveNewtonEuler(s_arm);
        var closed = new ClosedFormArm(s_arm);
        var random = new Random(7);
        var h = new double[4];
        var worst = 0.0;

        for (var trial = 0; trial < 1000; trial++)
        {
            var q = RandomVector(random, 4, Math.PI);
            rnea.BiasTorque(q, s_zero, planet, h);
            var expected = closed.Gravity(q, planet.G);

            var diff = MaxAbsDifference(expected, h);
            var scale = MaxAbs(expected);
            Assert.True(diff <= RelTol * scale,
                $"{planet.Key}, q = [{string.Join(", ", q)}]: difference {diff:E3}, scale {scale:E3}");
            if (scale > 0.0) worst = Math.Max(worst, diff / scale);
        }
        output.WriteLine($"V7 {planet.Key} (g = {planet.G}): max |Δg|/max|g| = {worst:E3}");
    }

    [Fact]
    public void BiasTorque_ZeroSpeed_InOrbit_IsExactlyZero()
    {
        var rnea = new RecursiveNewtonEuler(s_arm);
        var h = new double[4];
        rnea.BiasTorque([0.3, -0.7, 1.1, 0.4], s_zero, Planet.Orbit, h);
        Assert.Equal(new double[4], h);
    }

    [Fact]
    public void BiasTorque_GravityVectorIsMinusGAlongZ()
    {
        var rnea = new RecursiveNewtonEuler(s_arm);
        double[] q = [0.4, 0.2, -0.3, 0.5];
        var a = new double[4];
        var b = new double[4];
        rnea.BiasTorque(q, s_zero, Planet.Mars, a);
        rnea.BiasTorque(q, s_zero, new Vector3d(0.0, 0.0, -Planet.Mars.G), b);
        Assert.Equal(a, b);
    }

    [Theory]
    [MemberData(nameof(UnloadedPoseIndices))]
    public void BiasTorque_ZeroSpeed_MatchesVectorGravityEarth(int index)
    {
        var pose = TestVectors.Instance.Poses[index];
        var h = new double[4];
        new RecursiveNewtonEuler(s_arm).BiasTorque(pose.Q, s_zero, Planet.Earth, h);
        var closed = new ClosedFormArm(s_arm).Gravity(pose.Q, Planet.Earth.G);

        var scale = MaxAbs(pose.GravityEarth);
        var relGeneral = MaxAbsDifference(pose.GravityEarth, h) / scale;
        var relClosed = MaxAbsDifference(pose.GravityEarth, closed) / scale;
        output.WriteLine($"V7 pose {index}: general vs vectors {relGeneral:E3}, closed form vs vectors {relClosed:E3}");
        Assert.True(relGeneral <= RelTol, $"general g vs vectors: relative difference {relGeneral:E3}");
        Assert.True(relClosed <= RelTol, $"closed-form g vs vectors: relative difference {relClosed:E3}");
    }

    [Fact]
    public void BiasTorque_ZeroSpeed_AtHorizontalReach_MatchesTorqueBudgetVectors()
    {
        var vectors = TestVectors.Instance;
        var q = vectors.HorizontalReachPose;
        var h = new double[4];
        var checkedEntries = 0;
        foreach (var (key, expected) in vectors.GravityTorqueHorizontalReach)
        {
            // Keys are "<Planet>_payload<mp>", e.g. "HighG_payload0.5".
            var parts = key.Split("_payload");
            var planet = Planet.All.Single(p => p.Key == parts[0]);
            var mp = double.Parse(parts[1], System.Globalization.CultureInfo.InvariantCulture);
            new RecursiveNewtonEuler(s_arm.WithPayload(mp)).BiasTorque(q, s_zero, planet, h);

            var scale = MaxAbs(expected);
            Assert.True(MaxAbsDifference(expected, h) <= RelTol * scale, $"{key}: expected [{string.Join(", ", expected)}], got [{string.Join(", ", h)}]");
            checkedEntries++;
        }
        Assert.True(checkedEntries > 0);
    }
}
