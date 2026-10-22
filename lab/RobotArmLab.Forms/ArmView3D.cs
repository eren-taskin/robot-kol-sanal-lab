using System.Windows.Forms;
using System.Windows.Forms.Integration;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using RobotArmLab.Core.Kinematics;

namespace RobotArmLab.Forms;

/// <summary>
/// Kolun 3B görünümü. HelixToolkit (bir WPF kütüphanesi) çizimi yapar; WPF denetimi Windows Forms penceresine
/// ElementHost ile gömülür. Bu sınıf kinematik hesaplamaz: yalnız ForwardKinematics'in verdiği eklem noktalarına
/// borular (uzuvlar) ve küreler (eklemler) yerleştirir. Fare: sol tuşla sürükle = döndür, tekerlek = yakınlaştır.
/// </summary>
public class ArmView3D : UserControl
{
    private readonly HelixViewport3D viewport = new HelixViewport3D();
    private readonly PipeVisual3D[] links = new PipeVisual3D[4];       // taban→omuz, omuz→dirsek, dirsek→bilek, bilek→takım
    private readonly SphereVisual3D[] joints = new SphereVisual3D[3];  // omuz, dirsek, bilek
    private readonly SphereVisual3D tool = new SphereVisual3D();
    private readonly SphereVisual3D payload = new SphereVisual3D();

    /// <summary>
    /// Sahneyi kurar: ışıklar, zemin ızgarası, taban, uzuvlar, eklemler, takım işareti, yük küresi ve kamera.
    /// HelixToolkit yüklenemezse bu kurucu istisna atar; MainForm bunu yakalayıp yalnız 2B görünümle devam eder.
    /// </summary>
    public ArmView3D()
    {
        viewport.Children.Add(new DefaultLights());
        viewport.Children.Add(CreateGround());
        viewport.Children.Add(CreatePedestal());

        Brush[] linkColors = { Brushes.DimGray, Brushes.SteelBlue, Brushes.SteelBlue, Brushes.SlateGray };
        for (int i = 0; i < 4; i++)
        {
            links[i] = new PipeVisual3D();
            links[i].Diameter = 0.022;
            links[i].Fill = linkColors[i];
            viewport.Children.Add(links[i]);
        }
        for (int i = 0; i < 3; i++)
        {
            joints[i] = Sphere(0.016, Brushes.Orange);
            viewport.Children.Add(joints[i]);
        }
        tool.Radius = 0.008;
        tool.Fill = Brushes.Red;
        viewport.Children.Add(tool);
        payload.Radius = 0.022;
        payload.Fill = Brushes.Goldenrod;
        payload.Visible = false;
        viewport.Children.Add(payload);

        SetUpCamera();
        ElementHost host = new ElementHost();
        host.Dock = DockStyle.Fill;
        host.Child = viewport;
        Controls.Add(host);
    }

    /// <summary>
    /// Kolu verilen eklem noktalarına (taban, omuz, dirsek, bilek, takım; m) yerleştirir. Yük kütlesi sıfırdan
    /// büyükse takım noktasında yük küresi gösterilir.
    /// </summary>
    public void SetPose(Point3[] points, double payloadKg)
    {
        for (int i = 0; i < 4; i++)
        {
            links[i].Point1 = ToWpf(points[i]);
            links[i].Point2 = ToWpf(points[i + 1]);
        }
        for (int i = 0; i < 3; i++)
            joints[i].Center = ToWpf(points[i + 1]);
        tool.Center = ToWpf(points[4]);
        payload.Center = ToWpf(points[4]);
        payload.Visible = payloadKg > 0.0;
    }

    // Kamera kolun önüne ve biraz yukarısına konur, tabanın 15 cm üstüne bakar. z ekseni yukarıdır (taban çerçevesi).
    private void SetUpCamera()
    {
        viewport.ModelUpDirection = new Vector3D(0, 0, 1);
        viewport.ShowViewCube = true;
        viewport.ZoomExtentsWhenLoaded = false;
        viewport.RotateGesture = new MouseGesture(MouseAction.LeftClick);
        PerspectiveCamera camera = new PerspectiveCamera();
        camera.Position = new Point3D(0.75, -0.75, 0.55);
        camera.LookDirection = new Vector3D(-0.75, 0.75, -0.40);
        camera.UpDirection = new Vector3D(0, 0, 1);
        camera.FieldOfView = 45;
        viewport.Camera = camera;
    }

    // Zemin ızgarası: 1 m × 1 m, ince çizgiler 5 cm, kalın çizgiler 10 cm aralıkla, z = 0 düzleminde.
    private static GridLinesVisual3D CreateGround()
    {
        GridLinesVisual3D grid = new GridLinesVisual3D();
        grid.Width = 1.0;
        grid.Length = 1.0;
        grid.MinorDistance = 0.05;
        grid.MajorDistance = 0.1;
        grid.Thickness = 0.001;
        grid.Normal = new Vector3D(0, 0, 1);
        grid.Fill = Brushes.LightGray;
        return grid;
    }

    // Kolun oturduğu silindir biçimli taban (görsel; dinamiğe girmez).
    private static PipeVisual3D CreatePedestal()
    {
        PipeVisual3D pedestal = new PipeVisual3D();
        pedestal.Point1 = new Point3D(0, 0, 0);
        pedestal.Point2 = new Point3D(0, 0, 0.03);
        pedestal.Diameter = 0.08;
        pedestal.Fill = Brushes.Black;
        return pedestal;
    }

    private static SphereVisual3D Sphere(double radius, Brush fill)
    {
        SphereVisual3D s = new SphereVisual3D();
        s.Radius = radius;
        s.Fill = fill;
        return s;
    }

    private static Point3D ToWpf(Point3 p)
    {
        return new Point3D(p.X, p.Y, p.Z);
    }
}
