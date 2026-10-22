using System;
using System.Windows.Forms;

namespace RobotArmLab.Forms;

/// <summary>Uygulamanın giriş noktası: ana pencereyi açar.</summary>
internal static class Program
{
    /// <summary>
    /// [STAThread]: Windows Forms ve WPF (3B görünüm) tek iş parçacıklı "apartman" ister; yoksa ElementHost çalışmaz.
    /// </summary>
    [STAThread]
    private static void Main()
    {
        ApplicationConfiguration.Initialize();   // yüksek DPI ve görsel stiller (proje dosyasındaki ayarlardan üretilir)
        Application.Run(new MainForm());
    }
}
