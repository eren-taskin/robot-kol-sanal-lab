# Yol haritası — 4-DOF Robot Kol Sanal Laboratuvarı

KON 445E · Güz 2026 · 10 hafta

Bu belge ekip içindir. Projenin bütününü, karşılaşacağımız zorlukları ve haftalık planı anlatır. Teknik ayrıntılar `SPEC.md`, çalışma kuralları `PROJECT.md`, araçlara verilecek istemler `TASKS.md` içindedir.

---

## 1. Ne yapıyoruz

Bir kontrol öğrencisinin tarayıcıdan açıp kullanacağı bir laboratuvar. Öğrenci şunları yapar:

1. Bir **ortam** seçer: Dünya, Ay, Mars, yüksek g ya da yörünge.
2. Bir **yük** seçer: 0 ile 0,5 kg arası.
3. Bir **görev** seçer: noktaya gitme, al-götür-bırak, yörünge izleme ya da bozucuya karşı tutma.
4. Bir **kontrolcü** seçer ve kazançlarını ayarlar: PID, PD + yerçekimi kompanzasyonu ya da computed torque. Model tabanlı kontrolcüye "yanlış model" de verebilir.
5. Deneyi çalıştırır. Kolu, açı, hata ve tork grafiklerini ve performans metriklerini görür. Koşuları üst üste karşılaştırır.

Hocanın istediği "değişen ortam ve nesnelere göre uygun çalışan kontrolcü" bu yapıda doğrudan ölçülebilir hâle geliyor. Aynı kontrolcü beş ortamda ve farklı yüklerde çalıştırılır, metrikleri bir tabloda karşılaştırılır.

## 2. Önce bütün projeyi çıkarıyoruz (Faz 0)

Planımız şu: önce yapay zekâ araçlarıyla projenin tamamını uçtan uca çıkarıyoruz (T01–T11). Böylece hangi sorunlarla karşılaşacağımızı ilk iki üç haftada görüyoruz. Kalan haftalarda her üye kendi modülünü derinlemesine anlıyor, sağlamlaştırıyor ve deneyleri yapıyor.

| Görev | İçerik | Araç | Tahmini süre |
|---|---|---|---|
| T01 | Çözüm iskeleti, otomatik test (CI) | Claude Code | 1 oturum |
| T02 | Parametreler, gezegenler, yük | Claude Code | 1 oturum |
| T03 | İleri ve ters kinematik | Claude Code | 1 oturum |
| T04 | Dinamik (genel algoritma + kapalı form testleri) | Claude Code | 1–2 oturum |
| T05 | Plant, RK4, simülasyon, doğruluk testleri | Codex | 1–2 oturum |
| T06 | Bağımsız inceleme | Antigravity | 1 oturum |
| T07 | Kontrolcüler, sensörler, örneklemeli döngü | Codex | 1–2 oturum |
| T08 | Görevler, senaryolar, metrikler, kayıt | Codex | 1–2 oturum |
| T09 | Web arayüzü 1: düzen, çalıştırma, kol çizimi | Antigravity | 1–2 oturum |
| T10 | Web arayüzü 2: grafikler, karşılaştırma, dışa aktarma | Antigravity | 1–2 oturum |
| T11 | GitHub Pages'te yayın | Claude Code | 1 oturum |

Her görevden sonra özet bana gelir. Ben kontrol edip bir sonraki istemi gerekiyorsa düzeltirim.

## 3. Ön hesaplar: modeli doğruladık

`reference/arm_reference.py` raporumuzdaki denklemleri bağımsız olarak hesaplıyor. Sonuçlar:

| Kontrol | Sonuç | Anlamı |
|---|---|---|
| Kapalı form M ile genel formül (12) arasındaki fark | 3,5 × 10⁻¹⁰ | Rapordaki (14)–(17) doğru |
| Yerçekimi (18) ile ∂V/∂q arasındaki fark | 3,3 × 10⁻¹⁰ N·m | (18) doğru |
| Ters kinematik → ileri kinematik | 5,6 × 10⁻¹⁷ m | (5)–(7) doğru |
| Serbest hareket, 0,4 s, h = 2 ms, enerji kayması | 2,3 × 10⁻⁶ J | Raporla aynı |
| Düşey açısal momentum kayması | 8,3 × 10⁻⁹ | Raporla aynı |
| h yarılanınca hata oranı | 17,9 ve 17,5 | Dördüncü mertebe (≈16) |

Kol yatayda 0,38 m uzanmışken eklemlere düşen en büyük yerçekimi torku (motor sınırı 4,1 N·m):

| Ortam | Yüksüz | 0,5 kg yükle |
|---|---|---|
| Yörünge (g = 0) | 0 | 0 |
| Ay (1,62) | 0,14 N·m | 0,45 N·m |
| Mars (3,71) | 0,32 N·m | 1,02 N·m |
| Dünya (9,81) | 0,85 N·m | 2,71 N·m |
| Yüksek g (24,79) | 2,13 N·m | **6,84 N·m, motor doyuma girer** |

Bu tablo laboratuvarın hikâyesini özetliyor. Ay'da kontrolcü neredeyse sadece ataletle uğraşır. Dünya'da yerçekimi kompanzasyonu önem kazanır. Yüksek g'de ise kontrolcü ne kadar iyi olursa olsun motor yetmez. Öğrenci doyumu ve integral birikmesini (windup) burada görür.

## 4. Nelerle karşılaşacağız

| # | Zorluk | Nasıl fark edilir | Önlem |
|---|---|---|---|
| 1 | Dinamikte işaret veya indeks hatası | Kol "makul" hareket eder ama yanlıştır. Gözle fark edilmez | Kapalı formlarla ve referans değerlerle karşılaştıran testler (V5, V7), enerji testi (V9) |
| 2 | Serbest kol kaotiktir | İki doğru simülasyon birkaç saniye sonra farklı sonuç verir | Uzun koşuları yörüngeyle değil, korunan büyüklüklerle (enerji, momentum) karşılaştırmak |
| 3 | Sert denklemler | Yüksek kazanç, çok küçük sürtünme yumuşatması ya da büyük rotor ataleti yüzünden titreşim veya NaN | Adımı yarılayıp sonucun beklenen kadar değiştiğini görmek; h ve Ts için güvenli aralıklar |
| 4 | Motor doyumu ve integral birikmesi | Yüksek g'de aşım, çok uzun toparlanma | Anti-windup; doyum süresini metrik olarak göstermek |
| 5 | Tekillik ve erişilemeyen hedef | Ters kinematikte NaN, eklemlerde ani sıçrama | Hata türü döndüren IK, determinant eşiği, arayüzde açık uyarı |
| 6 | Yük alındığında parametrelerin sıçraması | Tork grafiğinde ani değişim | Bu bir hata değil, deneyin kendisi. Grafikte işaretlenir |
| 7 | Tarayıcıda arayüz donması | Sayfa tepki vermez | Her ekran karesinde sabit miktarda simülasyon zamanı ilerletmek (Not 4'ün web karşılığı); kaydı her adımda değil her kontrol örneğinde almak |
| 8 | Yapay zekâ araçlarının birbirini bozması | Bir aracın değişikliği diğerinin testlerini kırar | Bir anda tek araç, her görevden sonra kayıt, testleri gevşetmek yasak |
| 9 | Ders kuralı: her satırı savunabilmek | Sunumda bir soruya cevap verilemez | Her modülün bir sahibi ve ikinci okuyucusu; haftalık kod okuma oturumu |
| 10 | Türkçe sistemde ondalık virgül | CSV dosyası bozuk okunur | Her yerde `InvariantCulture` |
| 11 | Mac ve Windows farkı | Bir bilgisayarda derlenir, diğerinde derlenmez | `global.json` ile tek SDK sürümü; Blazor iki sistemde de çalışır |
| 12 | Bilinmeyen parametreler (rotor ataleti, sürtünme) | Gerçek kola göre farklı davranış | Bunları deney parametresi olarak sunmak, raporda açıkça yazmak |

## 5. Yapabileceklerimiz (fırsatlar)

- **Dayanıklılık haritası:** Bir kontrolcü 5 ortam × 4 yük ızgarasında çalıştırılır, başarı ve hata bir ısı haritasında gösterilir. Hocanın isteğine en doğrudan cevap bu olur.
- **Yanlış model deneyi:** Computed torque yük kütlesini bilmediğinde ne olur? Hata ne kadar büyür?
- **Adaptif kontrol:** Bilinmeyen yükü çalışırken tahmin eden kontrolcü. İsteğe bağlı, ileri seviye.
- **Ortama göre kazanç ayarı (gain scheduling):** Her gezegen için ayrı kazanç seti.
- **MuJoCo ile çapraz doğrulama:** Aynı URDF ve aynı torklarla bağımsız bir fizik motoruyla karşılaştırma. Raporda güçlü bir doğruluk kanıtı olur.
- **3D görünüm**, **6-DOF kol verisi**, gerçek bir OpenMANIPULATOR-X bulunursa **donanımda deneme (HIL)**.
- **Öğrenci föyü modu:** Her senaryonun yanında soruları olan kısa bir laboratuvar föyü.

## 6. Haftalık plan

| Hafta | Hedef | Çıktı |
|---|---|---|
| 1 | Kurulumlar, T01–T04 | Kinematik ve dinamik testleri geçiyor |
| 2 | T05–T08 | Çekirdeğin tamamı test edilmiş hâlde |
| 3 | T09–T11 | **Laboratuvar internette çalışıyor.** Hocaya ilk demo |
| 4 | Sahiplik haftası: herkes kendi modülünü okur ve ekibe anlatır; T06 bulguları kapatılır | Kod okuma notları, düzeltmeler |
| 5 | Kontrolcü karşılaştırması: PID, PDG, CT; beş ortam | Karşılaştırma tabloları, raporun kontrol bölümü |
| 6 | Örnekleme ve zamanlama: Ts, enkoder, gürültü, gecikme | Örnekleme deneyleri |
| 7 | Değişen yük: al-bırak, yanlış model, (isteğe bağlı) adaptif kontrol | Dayanıklılık haritası |
| 8 | Veri: kayıt, karşılaştırma, içe/dışa aktarma; arayüz iyileştirme | Kullanılabilirlik düzeltmeleri |
| 9 | Seçmeli genişletmeler: MuJoCo, 3D, 6-DOF, föy modu | Seçilen 1–2 genişletme |
| 10 | Rapor, sunum, son test geçişi | Teslim |

Her haftanın derste işlenen konusu (zamanlayıcılar, nesne yönelimi, testler, entegratörler, SIL/HIL) o haftanın işine bağlanır. Ders notu geldikçe planı birlikte güncelleriz.
