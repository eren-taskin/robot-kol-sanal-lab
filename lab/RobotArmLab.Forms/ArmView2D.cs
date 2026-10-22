using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using RobotArmLab.Core.Kinematics;

namespace RobotArmLab.Forms;

/// <summary>
/// Kolun yan görünümü (r–z düzlemi), GDI+ ile çizilir. Hiçbir dış kütüphaneye bağlı olmadığı için 3B görünüm
/// açılamazsa da çalışır. Kinematik hesaplamaz: ForwardKinematics'in verdiği eklem noktalarını, kolun döndüğü
/// düşey düzleme izdüşürüp çizer. r yatay uzaklık, z yüksekliktir; zemin z = 0'dır.
/// </summary>
public class ArmView2D : UserControl
{
    private const double ViewHalfWidthM = 0.45;   // çizilen bölge: r ∈ [−0,45; 0,45] m
    private const double ViewTopM = 0.50;         // z ∈ [−0,05; 0,50] m
    private const double ViewBottomM = -0.05;

    private double[] r = new double[5];   // eklem noktalarının yatay konumu, m
    private double[] z = new double[5];   // eklem noktalarının yüksekliği, m
    private bool hasPose;
    private bool showPayload;

    /// <summary>Titremesiz çizim için çift tamponlamayı açar.</summary>
    public ArmView2D()
    {
        DoubleBuffered = true;
        BackColor = Color.White;
        ResizeRedraw = true;
    }

    /// <summary>
    /// Yeni pozu alır ve yeniden çizer. points: taban, omuz, dirsek, bilek, takım (m); baseAngleRad: q1.
    /// Noktalar q1 doğrultusundaki düşey düzleme izdüşürülür: r = x cos q1 + y sin q1. Bu bir çizim
    /// izdüşümüdür; eklem noktalarının kendisi çekirdekte hesaplanır.
    /// </summary>
    public void SetPose(Point3[] points, double baseAngleRad, double payloadKg)
    {
        double c = Math.Cos(baseAngleRad), s = Math.Sin(baseAngleRad);
        for (int i = 0; i < 5; i++)
        {
            r[i] = points[i].X * c + points[i].Y * s;
            z[i] = points[i].Z;
        }
        showPayload = payloadKg > 0.0;
        hasPose = true;
        Invalidate();
    }

    /// <summary>Izgarayı, zemini, tabanı ve kolu çizer.</summary>
    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        DrawGrid(g);
        if (!hasPose)
            return;

        using (Pen linkPen = new Pen(Color.SteelBlue, 6f))
        {
            linkPen.StartCap = LineCap.Round;
            linkPen.EndCap = LineCap.Round;
            for (int i = 0; i < 4; i++)
                g.DrawLine(linkPen, ToScreen(r[i], z[i]), ToScreen(r[i + 1], z[i + 1]));
        }
        for (int i = 1; i < 4; i++)
            FillCircle(g, Brushes.Orange, r[i], z[i], 6f);
        FillCircle(g, Brushes.Red, r[4], z[4], 4f);
        if (showPayload)
            FillCircle(g, Brushes.Goldenrod, r[4], z[4], 9f);
        g.DrawString("Yan görünüm (r–z düzlemi)", Font, Brushes.Gray, 4f, 4f);
    }

    // 10 cm aralıklı ızgara, kalın zemin çizgisi (z = 0) ve taban bloğu.
    private void DrawGrid(Graphics g)
    {
        using (Pen gridPen = new Pen(Color.Gainsboro))
        {
            for (int k = -4; k <= 4; k++)
                g.DrawLine(gridPen, ToScreen(0.1 * k, ViewBottomM), ToScreen(0.1 * k, ViewTopM));
            for (int k = 0; k <= 5; k++)
                g.DrawLine(gridPen, ToScreen(-ViewHalfWidthM, 0.1 * k), ToScreen(ViewHalfWidthM, 0.1 * k));
        }
        using (Pen groundPen = new Pen(Color.SaddleBrown, 3f))
            g.DrawLine(groundPen, ToScreen(-ViewHalfWidthM, 0.0), ToScreen(ViewHalfWidthM, 0.0));
        PointF topLeft = ToScreen(-0.04, 0.03);
        PointF bottomRight = ToScreen(0.04, 0.0);
        g.FillRectangle(Brushes.Black, topLeft.X, topLeft.Y, bottomRight.X - topLeft.X, bottomRight.Y - topLeft.Y);
    }

    private void FillCircle(Graphics g, Brush brush, double rm, double zm, float radiusPx)
    {
        PointF c = ToScreen(rm, zm);
        g.FillEllipse(brush, c.X - radiusPx, c.Y - radiusPx, 2f * radiusPx, 2f * radiusPx);
    }

    // Metre → piksel. Ölçek, bölge pencereye sığacak ve en-boy oranı korunacak biçimde seçilir; ekranda y aşağı doğrudur.
    private PointF ToScreen(double rm, double zm)
    {
        double scale = Math.Min(ClientSize.Width / (2.0 * ViewHalfWidthM), ClientSize.Height / (ViewTopM - ViewBottomM));
        double x = ClientSize.Width / 2.0 + rm * scale;
        double y = ClientSize.Height - (zm - ViewBottomM) * scale;
        return new PointF((float)x, (float)y);
    }
}
