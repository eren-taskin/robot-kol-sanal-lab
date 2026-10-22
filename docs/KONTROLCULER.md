# Kontrolcüler: yasalar, kaynaklar ve ölçülen karşılaştırma

Bu belge Forms laboratuvarındaki (`lab/RobotArmLab.Core/Controllers/`) dört kapalı çevrim kontrolcüsünü anlatır: yasanın kendisi, her terimin ne yaptığı, neden seçildiği, kaynağı ve aynı deneyde ölçülen sonuçlar. Kod satırlarının anlatımı `docs/anlatim/W3.md`'dedir.

Gösterim: q eklem açısı (rad), q_ref referans, e = q_ref − q, ė = q̇_ref − q̇. Şapkalı terimler (M̂, ĉ, ĝ, f̂) kontrolcünün **kendi** modelinden gelir; bu model *tahmini* yükle kurulur, kol ise *gerçek* yükle. Bütün kontrolcülerde çıkış ±τmax = ±4,1 N·m'ye kırpılır ve kırpılan eklemde integral terimi o örnekte güncellenmez (sargı önleme). Kontrolcü her Ts = 2 ms'de bir çalışır, tork Ts boyunca sabit tutulur (ZOH).

Hocanın istediği "2023 sonrası makalelerden en az iki teknik, taban çizgisi olarak": **hesaplanmış tork** (Şen ve Öke Günel 2023 bunu temel yasa olarak kullanır) ve **süper-büküm kayan kip** (ileri beslemeli biçimiyle Saied ve diğ. 2023). PID ikisinin de altındaki taban çizgisidir.

---

## 1. PID (taban çizgisi)

τi = Kp,i·ei + Ki,i·∫ei dt + Kd,i·ėi

- **Kp e**: hataya orantılı "yay".
- **Ki ∫e dt**: kalıcı hatayı giderir. Model olmadığından yerçekimi torkunu da bu terim taşır.
- **Kd ė**: sönüm.
- Kazançlar (eski çekirdek T07): Kp = [6; 6; 6; 2] N·m/rad, Ki = [5; 20; 20; 8] N·m/(rad·s), Kd = [0,3; 0,6; 0,3; 0,05] N·m·s/rad.

Neden: her kontrol dersinin başlangıç noktası; modelsiz olduğundan "model yanlış" deneylerinden etkilenmez. Bedeli: yavaştır ve yük değişince kazançları ayarlanmamış kalır.

## 2. Hesaplanmış tork (CT)

τ = M̂(q)·(q̈_ref + Kd·ė + Kp·e) + ĉ(q, q̇) + ĝ(q) + f̂(q̇)

- **ĉ + ĝ + f̂**: Coriolis/merkezkaç, yerçekimi ve sürtünme torklarını *önceden* verir (söker).
- **M̂(q)·(…)**: istenen ivmeyi torka çevirir. Model doğruysa her eklem ayrı ayrı ë + Kd ė + Kp e = 0 doğrusal sistemi olur.
- Kazançlar (eski çekirdek A1c): Kp = [1600; 1600; 1800; 1800] 1/s², Kd = 350 1/s (ωn = 40–42 rad/s). Bütün başlangıç pozları, gezegenler ve 0/0,5 kg için en az 6 dB kazanç ve 40° faz payı.
- Yarım örnek öngörüsü (D17): model, tutma aralığının ortasında beklenen durumda hesaplanır: q̄ = q + q̇Δt/2 + aΔt²/8, q̄̇ = q̇ + aΔt/2.

Kaynak: klasik ters dinamik (hesaplanmış tork) kontrolü; Şen ve Öke Günel (2023), *Transactions of the Institute of Measurement and Control*, DOI [10.1177/01423312231153255](https://doi.org/10.1177/01423312231153255), bu yasayı temel kontrol yasası olarak kullanır.

Neden: model doğruysa en iyi izlemeyi verir (tabloda 10⁻⁶ rad mertebesi). Zayıf yanı modelin doğruluğuna bağımlılığıdır.

## 3. Süper-büküm kayan kip (STSMC)

s = ė + Λ·e
τ = M̂(q)·(q̈_ref + Λ·ė + u_st) + ĉ(q, q̇) + ĝ(q) + f̂(q̇)
u_st = k1·|s|^½·sign(s) + v,  v̇ = k2·sign(s)

- **s (kayma yüzeyi)**: s = 0 olunca ė = −Λe, yani hata 1/Λ zaman sabitiyle söner. Λ = 20 1/s → 50 ms.
- **k1·|s|^½·sign(s)**: s'yi sıfıra çeken sürekli terim; |s|^½ küçük s'de daha dik olduğundan sıfıra sonlu sürede varılır.
- **v (∫k2·sign(s))**: süreksiz işaret fonksiyonu *integralin içindedir*, bu yüzden u_st zamanın sürekli fonksiyonudur. v, model hatasından gelen bozucuyu karşılar.
- Model doğruysa iç döngü q̈ = q̈_ref + Λė + u_st verir, yani ṡ = −u_st: klasik süper-büküm döngüsü. Bozucunun türevi L ile sınırlı ve k2 > L ise s ve ṡ sonlu sürede sıfıra iner.
- Kazançlar (eski çekirdek D13): L = [93,8; 93,8; 199,0; 1013,0] rad/s³, "0,3 kg habersiz yük" yolunda ölçüldü; Levant kuralı k1 = 1,5√L, k2 = 1,1L → k1 = [14,53; 14,53; 21,16; 47,74], k2 = [103,2; 103,2; 218,9; 1114,3].

Kaynak (kazanç kuralı): A. Levant, "Robust exact differentiation via sliding mode technique", *Automatica* 34(3), 1998, s. 379–384, DOI [10.1016/S0005-1098(97)00209-4](https://doi.org/10.1016/S0005-1098(97)00209-4).

Neden: model hatasına karşı dayanıklı olması beklenir ve birinci mertebe kayan kipe göre torku sarsmaz (bkz. §6).

## 4. İleri beslemeli süper-büküm (FF-STSMC)

τ = M̂(q_ref)·(q̈_ref + Λ·ė + u_st) + ĉ(q_ref, q̇_ref) + ĝ(q_ref) + f̂(q̇_ref)

Geri besleme (s, u_st, v) STSMC ile aynıdır; tek fark model teriminin **istenen yörüngede** hesaplanmasıdır.

- Ölçüm (kodlayıcı nicemlemesi, gürültü, sonlu farkla bulunan hız) model terimlerinin içinden geçmez; yalnız e ve ė üzerinden geri beslemeye girer.
- Bedeli: kol referanstan uzaksa model terimi yanlış yerin dinamiğidir; fark bozucu olarak geri beslemeye kalır.

Kaynak: Saied ve diğ. (2023), "Feedforward super-twisting sliding mode control for robotic manipulators: application to PKMs" (başlık ve yazar listesi DOI sayfasından teyit edilecek), *IEEE Transactions on Robotics*, DOI [10.1109/TRO.2023.3255586](https://doi.org/10.1109/TRO.2023.3255586). **Denklem numarası makaleden teyit edilecek.** Makaleye erişilemedi; yapı görev tanımına göre kuruldu. Geri besleme teriminin de M̂(q_ref) ile çarpılması (M̂(q) yerine) makaleden teyit edilecektir.

---

## 5. Ölçülen karşılaştırma

**Deney:** başlangıç q0 = [0; −0,6; 0,3; 0,6] rad (durgun), eklem uzayında beşinci derece yol q_ref → [0,5; −0,2; 0,2; 0,8] rad, T = 2 s, koşu 3 s. RK4, h = 0,5 ms, Ts = 2 ms, ideal tork eyleyicisi, varsayılan kazançlar. "Gerçek/tahmini" yük kg cinsindendir. Sayılar `LabSession.CompareControllers` ile üretildi (aynı ayarlarla her seferinde aynı çıkar). Sütunlar: RMS ve en büyük eklem hatası (rad), en büyük takım hatası (mm), en büyük |τ| (N·m), doyum süresi (s), titreşim göstergesi Σ|Δτ_cmd|/süre (N·m/s). "—" hatasız bitti demektir.

### Dünya (g = 9,81 m/s²)

| Durum | Kontrolcü | RMS | En büyük | Takım (mm) | en büyük abs τ | Doyum | Titreşim | Hata |
|---|---|---|---|---|---|---|---|---|
| tam model (0,3/0,3) | PID | 0,038 | 0,198 | 85,1 | 1,55 | 0 | 2,2 | — |
| | CT | 8,4·10⁻⁷ | 2,2·10⁻⁶ | 0,0003 | 1,05 | 0 | 0,91 | — |
| | STSMC | 6,9·10⁻⁵ | 1,4·10⁻⁴ | 0,014 | 1,07 | 0 | 65,6 | — |
| | FF-STSMC | 6,6·10⁻⁵ | 1,4·10⁻⁴ | 0,014 | 1,07 | 0 | 65,8 | — |
| az tahmin (0,3/0) | PID | 0,038 | 0,198 | 85,1 | 1,55 | 0 | 2,2 | — |
| | CT | 0,154 | 0,447 | 49,8 | 1,11 | 0 | 1,0 | — |
| | STSMC | 0,497 | 2,48 | 342 | 1,28 | 0 | 5,7 | eklem 4 sınırı, 1,441 s |
| | FF-STSMC | 0,459 | 1,70 | 145 | 1,44 | 0 | 3,5 | eklem 3 sınırı, 1,738 s |
| fazla tahmin (0/0,3) | PID | 0,0085 | 0,048 | 17,5 | 0,38 | 0 | 0,38 | — |
| | CT | 0,402 | 1,11 | 137 | 4,10 | 0,479 | 15 602 | eklem 3 sınırı, 0,481 s |
| | STSMC | 0,0073 | 0,024 | 2,77 | 1,05 | 0 | 932 | — |
| | FF-STSMC | 0,011 | 0,054 | 4,51 | 1,10 | 0 | 1 303 | — |
| kodlayıcı + σ = 1 mrad (0/0) | PID | 0,0084 | 0,050 | 18,0 | 2,19 | 0 | 690 | — |
| | CT | 0,016 | 0,067 | 4,36 | 4,10 | 0,792 | 4 490 | — |
| | STSMC | 0,0067 | 0,019 | 5,45 | 1,16 | 0 | 440 | — |
| | FF-STSMC | 0,0071 | 0,019 | 4,60 | 1,24 | 0 | 479 | — |

### Ay (g = 1,62 m/s²)

| Durum | Kontrolcü | RMS | En büyük | Takım (mm) | en büyük abs τ | Doyum | Titreşim | Hata |
|---|---|---|---|---|---|---|---|---|
| tam model (0,3/0,3) | PID | 0,0063 | 0,030 | 13,2 | 0,24 | 0 | 0,36 | — |
| | CT | 6,7·10⁻⁷ | 1,7·10⁻⁶ | 0,0003 | 0,18 | 0 | 0,21 | — |
| | STSMC | 6,8·10⁻⁵ | 1,4·10⁻⁴ | 0,014 | 0,21 | 0 | 65,0 | — |
| | FF-STSMC | 6,7·10⁻⁵ | 1,4·10⁻⁴ | 0,014 | 0,21 | 0 | 65,2 | — |
| az tahmin (0,3/0) | PID | 0,0063 | 0,030 | 13,2 | 0,24 | 0 | 0,36 | — |
| | CT | 0,026 | 0,080 | 8,89 | 0,21 | 0 | 0,29 | — |
| | STSMC | 0,283 | 1,32 | 154 | 0,46 | 0 | 6,1 | — |
| | FF-STSMC | 0,054 | 0,249 | 45,1 | 0,39 | 0 | 4,1 | — |
| fazla tahmin (0/0,3) | PID | 0,0020 | 0,0073 | 2,49 | 0,07 | 0 | 0,10 | — |
| | CT | 0,446 | 0,869 | 173 | 4,10 | 2,99 | 12 310 | — (3 s boyunca çırpınır) |
| | STSMC | 0,0086 | 0,029 | 2,52 | 0,59 | 0 | 913 | — |
| | FF-STSMC | 0,016 | 0,054 | 4,74 | 0,88 | 0 | 1 365 | — |
| kodlayıcı + σ = 1 mrad (0/0) | PID | 0,0019 | 0,0092 | 3,13 | 1,92 | 0 | 690 | — |
| | CT | 0,013 | 0,073 | 4,22 | 4,10 | 0,77 | 4 502 | — |
| | STSMC | 0,0067 | 0,019 | 5,49 | 0,91 | 0 | 434 | — |
| | FF-STSMC | 0,0064 | 0,019 | 4,65 | 0,92 | 0 | 482 | — |

### Yüksek g (Jüpiter düzeyi, g = 24,79 m/s²)

| Durum | Kontrolcü | RMS | En büyük | Takım (mm) | en büyük abs τ | Doyum | Titreşim | Hata |
|---|---|---|---|---|---|---|---|---|
| tam model (0,3/0,3) | PID | 0,078 | 0,423 | 171 | 3,47 | 0 | 4,6 | — |
| | CT | 1,6·10⁻⁶ | 3,7·10⁻⁶ | 0,0006 | 2,64 | 0 | 2,2 | — |
| | STSMC | 6,9·10⁻⁵ | 1,4·10⁻⁴ | 0,014 | 2,67 | 0 | 66,7 | — |
| | FF-STSMC | 6,5·10⁻⁵ | 1,4·10⁻⁴ | 0,013 | 2,67 | 0 | 45,9 | — |
| az tahmin (0,3/0) | PID | 0,078 | 0,423 | 171 | 3,47 | 0 | 4,6 | — |
| | CT | 0,238 | 0,766 | 90,1 | 2,72 | 0 | 2,6 | — |
| | STSMC | 0,308 | 0,786 | 215 | 1,98 | 0 | 23,0 | zemine çarptı, 0,202 s |
| | FF-STSMC | 0,306 | 0,798 | 205 | 2,47 | 0 | 22,8 | zemine çarptı, 0,283 s |
| fazla tahmin (0/0,3) | PID | 0,022 | 0,128 | 46,6 | 1,02 | 0 | 0,95 | — |
| | CT | 0,428 | 1,23 | 76,2 | 4,10 | 0,159 | 14 381 | eklem 3 sınırı, 0,161 s |
| | STSMC | 0,025 | 0,234 | 28,3 | 2,64 | 0 | 807 | — |
| | FF-STSMC | 0,016 | 0,086 | 13,8 | 2,64 | 0 | 1 203 | — |
| kodlayıcı + σ = 1 mrad (0/0) | PID | 0,021 | 0,128 | 46,6 | 2,76 | 0 | 696 | — |
| | CT | 0,027 | 0,098 | 6,27 | 4,10 | 0,884 | 4 496 | — |
| | STSMC | 0,0060 | 0,019 | 4,75 | 1,59 | 0 | 444 | — |
| | FF-STSMC | 0,0062 | 0,020 | 4,82 | 1,63 | 0 | 481 | — |

### Dürüst okuma

- **Tam model:** CT açık ara en iyisidir (10⁻⁶ rad); süper-büküm 10⁻⁴ rad'da, örnekleme bandında kalır ve torku 65 N·m/s düzeyinde titretir. PID 0,04–0,08 rad geride kalır.
- **Az tahmin (gerçek 0,3 kg, model 0 kg): standart kazançlı STSMC bilek sınırına sürüklenir.** Başta bilek |δ| ≈ 1500 rad/s² ister (yükün ağırlığı bölü küçük tahmini bilek eylemsizliği). v sıfırdan k2 = 1114 rad/s³ hızla büyür; aynı anda M̂44/M44 ≈ 0,07 olduğundan v'nin her rad/s²'si gereken torkun çok küçük bir kısmını verir. Bilek düzenli biçimde çöker; bu bir çırpınma değil, tek yönlü bir kaymadır (eski çekirdek D13 ile aynı: orada da eklem 4 sınırı). FF-STSMC de aynı nedenle eklem 3 sınırına gider. CT sınırlı kalır ama büyük kalıcı hata (0,45 rad) bırakır. Bu durumda en iyisi modelsiz PID'dir. Ay'da bozucu küçük olduğundan kimse düşmez; yüksek g'de iki süper-büküm de zemine çarpar. Sonuç başlangıç pozuna da bağlıdır: pencerenin varsayılan pozundan (q0 = 0) STSMC düşmez (en büyük hata 1,32 rad), FF-STSMC 1,264 s'de eklem 3 sınırına gider (`ClosedLoopScenarioTests.C2`).
- **Fazla tahmin (gerçek 0 kg, model 0,3 kg): CT sınıra çarpar.** M̂ bilekte gerçeğin ≈ 13 katıdır; CT'nin döngü kazancı da 13 kat büyür, örneklemeli döngü kararsızlaşır, tork ±4,1 N·m arasında çırpınır (titreşim 15 000 N·m/s) ve kol 0,48 s'de eklem 3 sınırını aşar. Süper-büküm burada üstündür: hata 0,02–0,05 rad, doyum yok.
- **Kodlayıcı gürültüsü:** CT'nin Kd = 350'si nicemlenmiş hız ölçümünü (0,77 rad/s'lik basamaklar) büyütür: tork 0,8 s doyumda, titreşim ≈ 4 500 N·m/s. Süper-büküm yasaları ≈ 10 kat daha pürüzsüz komut verir (≈ 450–480 N·m/s) ve hatası daha küçüktür. FF-STSMC'nin bu ayarda STSMC'den belirgin bir üstünlüğü ölçülmedi: model terimi ölçümden kurtulur, ama komut gürültüsünün büyük kısmı geri besleme teriminden (Λė ve |s|^½) gelir.

### Kazanç ölçeği (Dünya, q0 = [0; −0,6; 0,3; 0,6], aynı yol)

k1 = 1,5√(cL), k2 = 1,1cL (yani k1 × √c, k2 × c). Az tahmin (0,3/0) ve fazla tahmin (0/0,3) durumları:

| c | STSMC az tahmin | FF-STSMC az tahmin | STSMC fazla tahmin |
|---|---|---|---|
| 1 | eklem 4 sınırı (1,44 s); RMS 0,497; titreşim 5,7 | eklem 3 sınırı (1,74 s) | düşmez; RMS 0,0073; titreşim 932 |
| 2 | düşmez; RMS 0,182, en büyük 0,96 | düşmez; RMS 0,135 | düşmez; RMS 0,028; titreşim 2 048 |
| 3 | düşmez; RMS 0,200, en büyük 0,81 | düşmez; RMS 0,104 | düşmez; RMS 0,031; titreşim 3 392 |
| 5 | eklem 4 sınırı (2,81 s) | düşmez; RMS 0,077 | düşmez; RMS 0,038; titreşim 5 819 |
| 10 | eklem 4 sınırı (1,16 s) | düşmez; RMS 0,067 | eklem 3 sınırı (0,17 s) |
| 20 | eklem 4 sınırı (1,53 s) | düşmez; RMS 0,0089 | eklem 3 sınırı |
| **30** | **düşmez; RMS 0,0021, en büyük 0,032; titreşim 135** | düşmez; RMS 0,0029; titreşim 136 | eklem 3 sınırı |
| 50 | düşmez; RMS 0,00089; titreşim 230 | düşmez; RMS 0,0011 | zemine çarptı |
| 100 | düşmez; RMS 0,0012; titreşim 429 | düşmez; RMS 0,0018 | zemine çarptı |

Okuma: az tahmin durumunda STSMC'yi güvenle kararlı yapan ölçek **c = 30**'dur (c = 2–3 de düşmez ama 0,8–1 rad hata bırakır; c = 5–20 arası yeniden düşer, yani ilişki tek yönlü değildir). Bedeli titreşimdir: c = 30'da az tahmin koşusunun titreşimi 5,7'den 135 N·m/s'ye çıkar. Daha önemlisi, **aynı ölçekte fazla tahmin durumu bozulur**: c ≥ 10'da STSMC fazla tahminde eklem 3 sınırını aşar. Tek bir c iki durumu birden kurtarmaz (eski çekirdek D13 ile aynı sonuç). Varsayılan kazançlar bu yüzden standart kural (c = 1) olarak bırakıldı.

---

## 6. Hocanın sorabileceği sorular

1. **CT model uyumsuzluğunda neden başarısız oluyor?** — CT, M̂'nin gerçek M'ye eşit olduğunu varsayarak doğrusal olmayan terimleri söker. Az tahminde eksik ağırlık torku Δg yalnız M̂·Kp·e ile karşılanabilir, e = M̂⁻¹Δg/Kp kalıcı hata kalır; bilekte M̂44 ≈ 3,7·10⁻⁴ kg·m² çok küçük olduğundan bu ≈ 0,45 rad'dır. Fazla tahminde etkin kazanç M⁻¹M̂ ≈ 13 kat büyür; Ts = 2 ms'lik örneklemeli döngü bu kazancı kaldıramaz, tork ±τmax arasında çırpınır ve kol sınırı aşar.
2. **Süper-büküm neden birinci mertebe kayan kipten daha az titretir?** — Birinci mertebe kayan kipte komutun kendisi u = −K·sign(s)'dir: her örnekte ±K atlar. Süper-bükümde süreksiz sign(s) yalnız v̇'nin içindedir; v sürekli değişir, k1|s|^½ sign(s) terimi de s → 0'da sıfıra gider. Böylece kontrol sinyali süreklidir ve süreksizlik bir türev yukarı itilmiştir. Örnekleme ile s yalnız O(Ts²) bandına iner. Bu laboratuvarda birinci mertebe kayan kip uygulanmadı; karşılaştırma kuramsaldır, ölçülmedi.
3. **FF neden istenen yörüngeyi kullanıyor?** — Ölçüm gürültüsü ve nicemleme M̂(q), ĉ(q, q̇), f̂(q̇) içinden geçince torka doğrudan karışır (özellikle sonlu farkla bulunan hız). Referans pürüzsüz ve önceden bilinir; model terimi oradan hesaplanınca gürültüsüzdür. Kol referansa yakın olduğu sürece (iyi izleme) iki model terimi neredeyse aynıdır. Bedeli, kol referanstan uzaklaşınca model teriminin yanlış yerin dinamiği olmasıdır.
4. **Λ, k1, k2 ne yapar?** — Λ kayma yüzeyinin eğimidir: s = 0'da hata 1/Λ zaman sabitiyle söner (Λ = 20 → 50 ms). k1, s'yi sıfıra çeken karekök teriminin kazancıdır: büyük k1 daha hızlı yaklaşma, daha çok titreşim. k2, v'nin büyüme hızıdır ve bozucunun türevinin sınırı L'den büyük olmalıdır (k2 > L); Levant'ın kuralı k1 = 1,5√L, k2 = 1,1L'dir.
5. **Motor modunda neden h ≤ 0,2 ms gerekiyor?** — Zıt EMK motor eklemine b = (N·Kt)(N·Ke)/R ≈ 0,85 N·m·s/rad'lık bir sönüm ekler. Hafif bilekte bu q̈ = −(b/M)q̇ + … biçiminde hızlı bir kipe, özdeğer λ ≈ b/M ≈ 5600 1/s'ye karşılık gelir (sistem "sert" olur). RK4'ün gerçek eksendeki kararlılık bölgesi −2,785 ≤ hλ ≤ 0'dır. h = 0,5 ms'de hλ ≈ −2,8, tam sınırdadır; sınırda RK4'ün büyütme çarpanı R(hλ) = 1 olduğundan kol sahte bir dengede takılı kalır (hız −1,46 rad/s gösterir, açı değişmez). h = 0,2 ms'de hλ ≈ −1,1 güvenlidir; Euler ve Heun için sınır −2 olduğundan 0,2 ms onlara da yeter.
6. **Neden yarım örnek öngörüsü?** — Tork Ts boyunca tutulur, ama ĝ(q) ve f̂(q̇) bu arada bayatlar. Bilekte sürtünmenin zaman sabiti M44/d ≈ 1,8 ms Ts'den kısadır; öngörü modeli tutmanın ortasındaki tahmini durumda hesaplayarak bu hatayı azaltır (D17).
7. **Yük habersiz değişirse (al-bırak) ne olur?** — Görev sekmesinde "Al-bırak" açılınca kol yüksüz başlar ve belirlenen anda yükü kavrar; tahmini yük değişmez. Kavramadan önce model doğrudur, sonra az tahmin durumuna geçilir (bkz. WINDOWS_KURULUM.md, madde 16).
