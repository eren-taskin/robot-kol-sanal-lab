using RobotArm.Core.Model;

namespace RobotArm.Tests;

/// <summary>SPEC §3.5: environments.</summary>
public sealed class EnvironmentTests
{
    [Fact]
    public void Planets_EqualSpecTable()
    {
        Assert.Equal(["Earth", "Moon", "Mars", "HighG", "Orbit"], Planet.All.Select(p => p.Key));
        Assert.Equal(["Dünya", "Ay", "Mars", "Yüksek g (Jüpiter düzeyi)", "Yörünge (mikro yerçekimi)"],
            Planet.All.Select(p => p.DisplayName));
        Assert.Equal([9.81, 1.62, 3.71, 24.79, 0.0], Planet.All.Select(p => p.G));
    }

    [Fact]
    public void Planets_EqualReferenceVectors()
    {
        var expected = TestVectors.Instance.Planets;
        Assert.Equal(expected.Count, Planet.All.Count);
        foreach (var p in Planet.All)
            Assert.Equal(expected[p.Key], p.G);
    }

    [Fact]
    public void GravityVector_PointsAlongMinusZ()
    {
        Assert.Equal(new Vector3d(0, 0, -9.81), Planet.Earth.GravityVector);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(12.5)]
    [InlineData(30.0)]
    public void Custom_GInsideZeroToThirty_IsAccepted(double g)
    {
        var p = Planet.Custom(g);
        Assert.Equal(g, p.G);
        Assert.Equal(Planet.CustomKey, p.Key);
    }

    [Theory]
    [InlineData(-0.01)]
    [InlineData(30.01)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void Custom_GOutsideZeroToThirty_Throws(double g)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => Planet.Custom(g));
    }
}
