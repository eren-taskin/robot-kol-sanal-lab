# Arşiv

Bu klasördeki dosyalar artık derlenmez ve CI'da çalışmaz; yalnız saklanır.

## İçerik

- `RobotArm.Web/` — Blazor WebAssembly ile yazılmış eski çevrim içi laboratuvar arayüzü (T09–S1).
- `workflows/pages.yml` — bu arayüzü GitHub Pages'e yükleyen eski iş akışı. `.github/workflows/` dışında olduğu için GitHub artık çalıştırmaz.

## Neden arşivlendi

Hocanın Ekim 2026'daki geri bildirimi üzerine laboratuvar Visual Studio'da açılan bir Windows Forms uygulaması olarak yeniden yazıldı (`lab/RobotArmLab.sln`, W1–W3). Hoca çözümdeki her satırı sorabileceği için çözümde yalnız Forms laboratuvarı için yazdığımız ve anlatabildiğimiz kod kaldı. Web arayüzü 22 Ekim 2026'da (W4) buraya taşındı.

Eski çekirdek (`src/RobotArm.Core`) ve testleri (`tests/RobotArm.Tests`) arşivlenmedi: lab çekirdeğinin doğrulanmasında test kâhini (oracle) olarak kullanılırlar (`lab/verification/`).

## Web sürümünü çalıştırmak

Proje burada derlenmez (çekirdeğe giden göreli yol değişti). Son çalışan web sürümü `web-final` etiketindedir:

```
git checkout web-final
dotnet run --project src/RobotArm.Web
```
