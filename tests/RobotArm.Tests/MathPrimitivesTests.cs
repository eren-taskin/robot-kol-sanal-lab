using RobotArm.Core.Model;

namespace RobotArm.Tests;

/// <summary>Vector3d and Matrix3d used by SPEC §4.1 and §5.5.</summary>
public sealed class MathPrimitivesTests
{
    [Fact]
    public void Vector_CrossAndDot_FollowRightHandRule()
    {
        Assert.Equal(Vector3d.UnitZ, Vector3d.Cross(Vector3d.UnitX, Vector3d.UnitY));
        Assert.Equal(32.0, Vector3d.Dot(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6)));
        Assert.Equal(5.0, new Vector3d(3, 0, 4).Length);
    }

    [Fact]
    public void Matrix_ProductsAndTranspose_AreCorrect()
    {
        var a = new Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 10);
        Assert.Equal(new Vector3d(14, 32, 53), a * new Vector3d(1, 2, 3));
        Assert.Equal(a, a * Matrix3d.Identity);
        Assert.Equal(new Matrix3d(1, 4, 7, 2, 5, 8, 3, 6, 10), a.Transpose());
        Assert.Equal(-3.0, a.Determinant, 1e-12);
        Assert.Equal(new Matrix3d(30, 36, 45, 66, 81, 102, 109, 134, 169), a * a);
        Assert.Equal(6.0, a[1, 2]);
    }

    [Fact]
    public void PointMassInertia_EqualsParallelAxisTerm()
    {
        var d = new Vector3d(0.1, 0.0, 0.2);
        var i = Matrix3d.PointMassInertia(2.0, d);
        Assert.Equal(2.0 * 0.04, i.M00, 1e-15);
        Assert.Equal(2.0 * 0.05, i.M11, 1e-15);
        Assert.Equal(2.0 * 0.01, i.M22, 1e-15);
        Assert.Equal(-2.0 * 0.02, i.M02, 1e-15);
        Assert.True(i.IsSymmetric);
    }

    [Fact]
    public void IsSymmetricPositiveDefinite_DetectsIndefiniteAndAsymmetric()
    {
        Assert.True(Matrix3d.Identity.IsSymmetricPositiveDefinite);
        Assert.False(Matrix3d.Diagonal(1, -1, 1).IsSymmetricPositiveDefinite);
        Assert.False(new Matrix3d(1, 0.1, 0, 0, 1, 0, 0, 0, 1).IsSymmetricPositiveDefinite);
        Assert.False(Matrix3d.Zero.IsSymmetricPositiveDefinite);
    }
}
