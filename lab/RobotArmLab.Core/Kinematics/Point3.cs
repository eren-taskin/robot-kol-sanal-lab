namespace RobotArmLab.Core.Kinematics;

/// <summary>Taban çerçevesinde bir nokta (x ileri, y sola, z yukarı), m. Çizim ve takım konumu için.</summary>
public class Point3
{
    /// <summary>x koordinatı, m.</summary>
    public double X;

    /// <summary>y koordinatı, m.</summary>
    public double Y;

    /// <summary>z koordinatı (yükseklik), m. z = 0 zemindir.</summary>
    public double Z;

    /// <summary>Verilen koordinatlarla (m) bir nokta oluşturur.</summary>
    public Point3(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }
}
