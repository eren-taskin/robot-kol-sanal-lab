namespace RobotArm.Tests;

public sealed class TestVectorsTests
{
    [Fact]
    public void TestVectors_Load_ContainsSixteenPoses()
    {
        var vectors = TestVectors.Load(Path.Combine(AppContext.BaseDirectory, TestVectors.FileName));

        Assert.Equal(16, vectors.Poses.Count);
    }
}
