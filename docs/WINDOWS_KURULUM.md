# Windows'ta kurulum ve elle denetim listesi

Bu belge, laboratuvar uygulamasını (`lab/RobotArmLab.Forms`) Windows'ta ilk kez çalıştıracak ekip arkadaşı içindir. Uygulama Mac'te geliştirilir ve yalnız derlenir; ekranda nasıl göründüğü ve davrandığı Windows'ta bu listeyle denetlenmelidir.

Gereken: Windows 10 (sürüm 2004 veya sonrası) ya da Windows 11, internet bağlantısı (ilk derlemede NuGet paketleri iner).

---

## 1. Visual Studio 2026 kurulumu

1. https://visualstudio.microsoft.com adresinden **Visual Studio 2026 Community** yükleyicisini indirip çalıştırın.
2. "İş yükleri" (Workloads) ekranında **.NET masaüstü geliştirme** (".NET desktop development") kutusunu işaretleyin. Başka iş yükü gerekmez.
3. "Yükle"ye basın ve kurulumun bitmesini bekleyin. İstenirse bilgisayarı yeniden başlatın.

## 2. Depoyu GitHub Desktop ile klonlama

1. https://desktop.github.com adresinden **GitHub Desktop**'ı kurun ve GitHub hesabınızla oturum açın.
2. **File → Clone repository…** menüsünü açın.
3. "GitHub.com" sekmesinde depoyu (`robot-kol-sanal-lab`) seçin; "Local path" olarak kısa bir yol seçin (ör. `C:\Projeler\robot-kol-sanal-lab`). Türkçe karakter ve boşluk içermeyen bir yol sorun çıkarmaz.
4. **Clone**'a basın.
5. Güncel kodu almak için ileride üstteki **Fetch origin → Pull** düğmesini kullanın.

## 3. Çözümü açma ve çalıştırma

1. Visual Studio'yu açın → **Bir proje veya çözüm aç** → klonladığınız klasörde `lab\RobotArmLab.sln` dosyasını seçin.
2. Çözüm Gezgini'nde (Solution Explorer) üç proje görünür: `RobotArmLab.Forms`, `RobotArmLab.Core`, `RobotArmLab.Tests`. Başka proje yoktur; eski çekirdekle karşılaştırma testleri ayrı bir çözümdedir (`lab\verification\RobotArmLab.Verification.sln`).
3. `RobotArmLab.Forms` çözümdeki ilk proje olduğu için başlangıç projesidir (kalın yazılır). Değilse sağ tıklayın → **Başlangıç Projesi Olarak Ayarla** (Set as Startup Project).
4. **F5**'e basın. İlk derlemede NuGet paketleri (ScottPlot.WinForms 5.1.59, HelixToolkit.Wpf 3.1.2) iner; bu bir iki dakika sürebilir.
5. "Robot Kol Sanal Laboratuvarı — açık çevrim" penceresi açılmalıdır.

Testleri Visual Studio'da çalıştırmak için: **Test → Test Gezgini** → "Tümünü Çalıştır". 82 test geçmelidir.

Formu tasarımcıda açmak için: Çözüm Gezgini'nde `MainForm.cs`'ye çift tıklayın (veya sağ tık → **Tasarımcıyı Görüntüle**). 3B görünüm ve grafikler tasarımcıda boş panel olarak görünür; çalışma anında eklenirler.

### Sorun çıkarsa

- **Paketler inmiyor:** Araçlar → NuGet Paket Yöneticisi → Paket Yöneticisi Ayarları → "nuget.org" kaynağının işaretli olduğunu denetleyin; çözüme sağ tık → **NuGet Paketlerini Geri Yükle**.
- **3B görünüm yok, araç çubuğunda "3B görünüm açılamadı…" yazıyor:** Uygulama 2B görünümle çalışmaya devam eder. Yazının devamındaki hata metnini ekibe iletin.
- **"SDK bulunamadı" hatası:** Depodaki `global.json` .NET 10 SDK ister; Visual Studio 2026 bunu getirir. Visual Studio Installer'da "Güncelle"ye basın.

---

## 4. Elle denetim listesi (16 madde)

Her denemeye **Sıfırla** ile başlayın: ayarlar Sıfırla'ya basınca (veya yeni/bitmiş bir koşuda Başlat'a basınca) uygulanır. Varsayılanlar: Dünya, yük 0, RK4, h = 0,5 ms, tork modu, bütün girişler 0, bütün açılar 0°, süre 10 s, hız 1×. Parantez içindeki sayılar `lab/RobotArmLab.Tests/LabScenarioTests.cs` testlerinden gelir; ekranda son basamaklar biraz farklı olabilir.

| # | Ne yapılır | Beklenen sonuç |
|---|---|---|
| 1 | Programı açın. | Solda altı sekme (Kol parametreleri, Ortam ve yük, Çözücü, Giriş, Kontrol, Görev), ortada 3B görünüm ve 2B yan görünüm, altta yedi sekme (Açılar, Hızlar, Torklar, Takım konumu, Enerji, Hata, Karşılaştır). Kol düz uzanmış (q = 0) görünür. "Kontrol" sekmesinde "Açık çevrim" seçilidir. 3B görünümde sol tuşla sürükleyince sahne döner, tekerlek yakınlaştırır. |
| 2 | Ortam ve yük → **Yörünge**, Hız **4×**, Sıfırla, Başlat. | Kol hiç kıpırdamaz; açı grafiği düz sıfır çizgisi. Yaklaşık 2,5 s sonra zaman t = 10,000 s'de durur, durum etiketi "Süre doldu." |
| 3 | Ortam **Dünya**, girişler 0, Sıfırla, Başlat. | Kol kendi ağırlığıyla düşer ve hemen durur. Durum etiketi "Takım noktası zemine çarptı (t = 0,169 s)." (test: 0,169 s). 2B görünümde takım (kırmızı nokta) zemin çizgisine değmiştir. |
| 4 | Aynısını **Ay** ile yapın (isterseniz **Mars** ile de). | Ay'da düşme yavaştır: "zemine çarptı (t = 0,515 s)" (test: 0,515 s, Dünya'nın ≈ 3 katı). Mars: t ≈ 0,295 s. Sürtünme (fv, fc) 0 yapılırsa oran tam √(9,81/1,62) ≈ 2,46 olur (0,181 s ve 0,445 s); sürtünme zayıf yerçekiminde ağırlığa göre daha büyük pay aldığı için Ay'daki düşüşü daha çok yavaşlatır. |
| 5 | Dünya, Yük kütlesi **0,5 kg**, Sıfırla, Başlat. | Takım noktasında sarı yük küresi görünür. Kol yine zemine çarpar, ama biraz **daha geç**: t ≈ 0,192 s (test: 0,192 s, yüksüz 0,169 s). Neden: uca eklenen kütle kolu "daha uzun bir sarkaç" yapar; düşme açısal ivmesi azalır. (Görev metnindeki "yüklü kol daha hızlı düşer" beklentisi simülasyonla doğrulanmadı.) |
| 6 | Yörünge, Giriş → **Gerilim (V)**, Eklem 1 satırı: Tür Sabit, Genlik **12**, q0 **−170**; Çözücü → Süre **1** s; Sıfırla, Başlat. Hızlar q̇ sekmesine bakın. | q̇1 ≈ 0,03 s içinde yükselir ve ≈ **271 °/s** (4,73 rad/s ≈ **45,2 rpm**) değerinde düzleşir (test: 45,2 rpm). Veri sayfasındaki yüksüz hız 46 rpm'dir; fark eklem sürtünmesindendir (fv ve fc 0 yapılırsa 45,9 rpm). Taban 3B görünümde döner. |
| 7 | Kol parametreleri → "3 Motor ve sürtünme" altında **Viskoz sürtünme fv = 0** ve **Coulomb sürtünmesi fc = 0**. Ortam Dünya, giriş tork modu ve 0, Eklem 2/3/4 q0 = **−60 / 30 / 30**, Çözücü **Euler**, h = **5** ms. Sıfırla, Başlat, Enerji sekmesi. Sonra yalnız çözücüyü **RK4** yapıp tekrarlayın. | Euler: T + V çizgisi düz değil, 0,758 J'den ≈ 0,768 J'ye çıkar (test: en büyük sapma 1,0·10⁻² J). RK4: T + V düz kalır (test: < 10⁻⁶ J). İki koşu da t ≈ 0,14 s'de "Eklem … sınırını aştı" ile durur. Bitince **Varsayılana dön**'e basın. |
| 8 | Yörünge, tork modu, Eklem 1: Genlik **10** (N·m), q0 **−170**; Sıfırla, Başlat; Torklar τ sekmesi. | τ1 çizgisi 10'da değil **4,1 N·m**'de düzdür: grafik doyumdan sonra kola gerçekten uygulanan torku gösterir (test: 4,1 N·m). |
| 9 | Varsayılan ayarlar, Yörünge, Hız **0,25×**, Başlat; birkaç saniye sonra Hız **2×** yapın. | 0,25×'te zaman etiketi gerçek saatten 4 kat yavaş, 2×'te 2 kat hızlı ilerler. Hız değişikliği Sıfırla gerektirmez, koşu kesilmeden uygulanır. **Duraklat** zamanı dondurur, **Başlat** kaldığı yerden sürdürür. |
| 10 | Giriş tablosunda Eklem 2 q0 = **100** yazın, Sıfırla. Sonra 0'a geri alın, Sıfırla, birkaç saniye çalıştırıp **CSV kaydet** ile dosyayı kaydedin ve Not Defteri ile açın. | İlk adımda "Ayarlar geçersiz" penceresi: "Eklem 2 başlangıç açısı 100° eklem sınırlarının dışında [-85,944°, 85,944°]." Program çökmez. CSV'nin ilk satırı `t_s,q1_rad,…,E_J` başlığıdır; sayılarda ondalık ayırıcı **noktadır** (ör. `0.0005`), sütunlar virgülle ayrılır. |

### Kapalı çevrim (W3) maddeleri

Her maddeye program yeni açılmış gibi başlayın (veya Kol parametreleri → **Varsayılana dön**, Ortam **Dünya**, yük **0**). Ortak adım: **Kontrol → Kapalı çevrim**. Varsayılan görev: eklem uzayında beşinci derece yol, hedef [28,6°; −11,5°; 11,5°; 45,8°], T = 2 s; başlangıç açıları 0°, süre 10 s, Ts = 2 ms, algılayıcı kusursuz, eyleyici ideal tork. "Karşılaştır" sekmesinde **Karşılaştır (aynı deney)** düğmesi dört kontrolcüyü birkaç saniyede koşturur (pencere bu sırada bekler). Sayılar `lab/RobotArmLab.Tests/ClosedLoopScenarioTests.cs` testlerinden gelir.

| # | Ne yapılır | Beklenen sonuç |
|---|---|---|
| 11 | Kapalı çevrim, kontrolcü **Hesaplanmış tork (CT)**, Sıfırla, Başlat. Açılar ve Hata sekmelerine bakın. Sonra Karşılaştır sekmesi → Karşılaştır. | Kesikli referans çizgileri düz çizgilerin tam üstündedir; Hata sekmesinde eğriler sıfıra yapışık. 3B görünümde hedefte yeşil küre. Tabloda hiçbir satırda not yok; en büyük hata PID ≈ **0,060 rad**, CT ≈ **1,7·10⁻⁵ rad**, STSMC ve FF-STSMC ≈ **1,4·10⁻⁴ rad**. |
| 12 | Ortam ve yük → Yük **0,3 kg** (Kontrol → Tahmini yük **0** kalır). Karşılaştır. | Az tahmin: CT düşmez ama en büyük hata ≈ **0,61 rad**; STSMC bu pozdan düşmez, en büyük hata ≈ **1,3 rad**; FF-STSMC satırında not: "Eklem 3 sınırını aştı (t = 1,264 s)." |
| 13 | Yük **0**, Kontrol → Tahmini yük **0,3**. Karşılaştır; sonra CT ile Sıfırla, Başlat, Torklar sekmesi. | Fazla tahmin: CT satırında "Takım noktası zemine çarptı (t = 0,423 s)."; tork komutu ±4,1 N·m arasında çırpınır. STSMC düşmez: en büyük hata < **0,05 rad** (≈ 0,041), takım hatası < **5 mm**. |
| 14 | Tahmini yük **0**, Algılayıcı **Kodlayıcı (nicemleme)**, gürültü σ **1** mrad. Karşılaştır. | Titreşim sütunu: CT ≈ **4 800 N·m/s** ve doyum ≈ **3,2 s**; FF-STSMC ≈ **510 N·m/s**, doyum 0. Torklar sekmesinde CT'nin kesikli komutu kalın bir bant gibi görünür. **Tabloyu CSV kaydet** ile kaydedilen dosyanın ilk satırı `controller,rms_rad,…` başlığıdır. |
| 15 | Algılayıcı **Kusursuz**, Eyleyici **Motor**. Kırmızı notu okuyun; Sıfırla. Sonra Çözücü → h **0,1** ms, Sıfırla, Başlat (CT). | Not: "Motor eyleyicisinde çözücü adımı h ≤ 0,2 ms olmalıdır …". h = 0,5 ms'de Sıfırla "Ayarlar geçersiz" penceresinde aynı nedeni gösterir. h = 0,1 ms'de koşu hatasız biter; CT en büyük hata ≈ **6,1·10⁻³ rad**. **CSV kaydet** dosyasında `qref1_rad`, `taucmd1_Nm` ve `payload_kg` sütunları doludur. |
| 16 | Eyleyici **İdeal tork**, h **0,5** ms. Görev → **Çember izleme** (varsayılan: merkez 200/0/150 mm, r = 30 mm, düşey, tur 4 s); Karşılaştır. Sonra Yük **0,3 kg**, **Al-bırak** işaretli, kavrama anı **4** s; Karşılaştır; CT ile Başlat. | 3B görünümde yeşil çember yolu. Yüksüz: FF-STSMC takım hatası < **0,02 mm**. Al-bırakta: sarı yük küresi t = 4 s'de belirir; CT düşmez ama hatası ≈ **0,70 rad**'a çıkar; STSMC satırında "Eklem 4 sınırını aştı (t = 5,241 s)." |

Bir madde beklendiği gibi değilse: maddenin numarasını, ekran görüntüsünü ve durum etiketindeki metni ekibe iletin.
