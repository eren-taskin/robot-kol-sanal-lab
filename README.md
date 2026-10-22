# Robot Kol Sanal Laboratuvarı — KON 445E

4 serbestlik dereceli OpenMANIPULATOR-X robot kolu için Windows Forms sanal laboratuvarı (KON 445E, İTÜ, Güz 2026).

Öğrenci kolun parametrelerini, ortamı (Dünya, Ay, Mars, yüksek g, yörünge), yükü, sayısal çözücüyü (Euler, Heun, RK4, Dormand–Prince) seçer. Kolu açık çevrimde tork veya motor gerilimiyle sürer ya da kapalı çevrimde bir kontrolcüyle (PID, hesaplanmış tork, süper-büküm kayan kip, ileri beslemeli süper-büküm) bir görevi izletir. Kol 2B ve 3B olarak canlandırılır; açılar, hızlar, torklar, takım konumu, enerji ve hata grafiklerde gösterilir; kontrolcüler aynı deneyle karşılaştırılır ve sonuçlar CSV olarak kaydedilir. Fizik `SPEC.md`'de tanımlıdır.

## Visual Studio 2026'da açma ve çalıştırma

Gereken: Windows 10 (2004 veya sonrası) ya da Windows 11, **Visual Studio 2026** ve onun **.NET masaüstü geliştirme** iş yükü. Bu iş yükü .NET 10 SDK'yı da getirir; depodaki `global.json` .NET 10 ister.

1. Visual Studio'da **Dosya → Aç → Proje/Çözüm** ile `lab/RobotArmLab.sln` dosyasını açın.
2. Çözümde üç proje vardır:
   - `RobotArmLab.Forms`: pencere ve grafikler (başlangıç projesi),
   - `RobotArmLab.Core`: model, çözücüler, kontrolcüler, oturum,
   - `RobotArmLab.Tests`: xunit testleri.
3. **F5** ile çalıştırın. İlk derlemede NuGet paketleri (ScottPlot.WinForms, HelixToolkit.Wpf) iner.
4. Testler için **Test → Test Gezgini → Tümünü Çalıştır**.

Ayrıntılı kurulum ve Windows'ta elle denetim listesi: [`docs/WINDOWS_KURULUM.md`](docs/WINDOWS_KURULUM.md).

Komut satırından (Windows, macOS veya Linux; Forms uygulaması yalnız Windows'ta çalışır):

```
dotnet build lab/RobotArmLab.sln
dotnet test lab/RobotArmLab.sln
```

## Kodun anlatımı

`docs/anlatim/` klasöründeki belgeler Türkçedir:

- [`KOD_TURU.md`](docs/anlatim/KOD_TURU.md): Başlat düğmesinden çözücüye ve grafiklere bir zamanlayıcı tıkı; algılayıcı → kontrolcü → motor → tesis → ölçütler zinciri. Dosya, metot ve satır numaralarıyla, çağrı sırasıyla.
- [`SAHIPLIK.md`](docs/anlatim/SAHIPLIK.md): dosyaların dört role bölünmesi; her rol için okuma sırası, olası sorular ve kısa cevapları, "göster" görevleri.
- [`W1.md`](docs/anlatim/W1.md), [`W2.md`](docs/anlatim/W2.md), [`W3.md`](docs/anlatim/W3.md): çekirdeğin, Forms arayüzünün ve kapalı çevrimin dosya dosya anlatımı.

## Depo düzeni

- `lab/RobotArmLab.sln`: laboratuvar (yukarıdaki üç proje).
- `lab/verification/RobotArmLab.Verification.sln`: lab çekirdeğinin dinamiğini eski çekirdekle karşılaştıran doğrulama testleri.
- `src/RobotArm.Core`, `tests/RobotArm.Tests` (`RobotArm.sln`): eski çekirdek ve testleri. Yalnız doğrulamada kâhin (oracle) olarak kullanılır.
- `reference/`: bağımsız Python referans modeli ve test vektörleri (`test_vectors.json`).
- `docs/`: anlatım belgeleri, rapor, kararlar.
- `archive/`: artık derlenmeyen eski web arayüzü ([`archive/README.md`](archive/README.md)).
