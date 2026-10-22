# Sahiplik: dört rol

`lab/RobotArmLab.sln` içindeki bütün dosyalar dört role bölünmüştür. Her rolün sahibi kendi dosyalarını satır satır anlatabilmeli, diğer rollerin dosyalarını da `KOD_TURU.md` düzeyinde bilmelidir. Satır sayıları W4 commit'indeki kodun yaklaşık satır sayılarıdır (boş satır ve yorum dahil).

| Rol | Konu | Kod (satır) | Testler (satır) |
|---|---|---|---|
| 1 | Model, kinematik, dinamik | ~1090 | ~330 |
| 2 | Çözücüler, motor, açık çevrim simülasyonu, oturum | ~1360 | ~510 |
| 3 | Kontrolcüler, görevler, algılayıcı, ölçütler, karşılaştırma | ~1460 | ~640 |
| 4 | Forms arayüzü ve grafikler | ~1370 elle yazılmış + ~1260 tasarımcı | — |

Rol 1'in satır sayısı az görünür ama kütle matrisi ve türevi (eş. 13–17) en yoğun matematiği taşır. Rol 4'ün `*.Designer.cs` dosyaları Visual Studio tasarımcısının ürettiği yerleşim kodudur: her satırı ezberlemek gerekmez, ama olayların (`Click`, `Tick`) nerede bağlandığı bilinmelidir.

`Ctrl+G` satıra gider. `F12` bir metodun tanımına, `Shift+F12` onu çağıran yerlere götürür. "Göster" görevlerinde bunlar kullanılır.

---

## Rol 1 — Model, kinematik, dinamik

### Dosyalar

```
lab/RobotArmLab.Core/Model/ArmParameters.cs        bütün fiziksel parametreler (SPEC §3), Default(), Clone()
lab/RobotArmLab.Core/Model/GravityEnvironment.cs   ortam adı ve g; beş hazır ortam
lab/RobotArmLab.Core/Model/Payload.cs              uçtaki yük; ApplyTo() yükü uzuv 4'e katar (Steiner)
lab/RobotArmLab.Core/Kinematics/Point3.cs          (x, y, z) noktası
lab/RobotArmLab.Core/Kinematics/ForwardKinematics.cs  eklem açıları → eklem noktaları, takım konumu (eş. 1–4)
lab/RobotArmLab.Core/Kinematics/InverseKinematics.cs  takım hedefi → eklem açıları (eş. 5–7)
lab/RobotArmLab.Core/Dynamics/MassMatrixFormulas.cs  M(q) ve ∂M/∂q kapalı formda (eş. 14–17)
lab/RobotArmLab.Core/Dynamics/ArmDynamics.cs       c, g, τf, ileri ve ters dinamik, enerji (eş. 11–21)
lab/RobotArmLab.Core/Numerics/Small4x4.cs          Cholesky, M q̈ = b çözümü, matris–vektör çarpımı
lab/RobotArmLab.Core/Session/SettingsValidator.cs  parametre, ortam, yük ve başlangıç pozu denetimi
lab/RobotArmLab.Tests/KinematicsTests.cs, DynamicsTests.cs, EnergyTests.cs
lab/RobotArmLab.Tests/Assertions.cs, TestVectors.cs   test yardımcıları, reference/test_vectors.json okuyucusu
```

### Okuma sırası

1. `ArmParameters.cs`: `Default()` (satır 117) içindeki sayıları rapordaki Tablo 3 ve Tablo 5 ile karşılaştırın. `L2M` ve `BetaRad` hesaplanır (satır 41, 45).
2. `ForwardKinematics.cs`: `Thetas` (17), `JointPositions` (31). Önce düşey düzlemde r, z; sonra `InBaseFrame` (102) ile q1 döndürmesi.
3. `InverseKinematics.cs`: `Solve` (48). Sırası: q1, bilek noktası (u, v), kosinüs teoremiyle φ, sonra q2, q3, q4.
4. `Payload.cs`: `ApplyTo` (28). Dört numaralı adım adım yorumlar.
5. `MassMatrixFormulas.cs`: `Compute` (18), sonra `YawInertia`, `CouplingTerms`, `FillPitchBlock`. `Derivative` (35) aynı satırların türevidir.
6. `ArmDynamics.cs`: `GravityVector` (66), `CoriolisVector` (42), `Friction` (86), `Acceleration` (99), `InverseDynamics` (116), enerji (129, 142).
7. `Small4x4.cs`: `Cholesky` (16), `SolveSymmetricPositiveDefinite` (46).
8. `SettingsValidator.cs` ve testler.

### Hocanın sorabileceği 10 soru

1. **Kinematik neden önce düşey düzlemde hesaplanıyor?** — q1 düşey eksen etrafında döner; q2, q3, q4 birbirine paralel yatay eksenlerdir. Bu yüzden kolun bütün noktaları q1'in belirlediği düşey düzlemde kalır. Önce o düzlemde yatay uzaklık r ve yükseklik z bulunur, sonra x = r cos q1, y = r sin q1 (`InBaseFrame`).
2. **β nedir, neden var?** — Uzuv 2 düz değil, kırıktır: eklem 3, eklem 2'ye göre a2x = 24 mm ileride ve a2z = 128 mm yukarıdadır. β = atan2(a2z, a2x) ≈ 79,38°. Bu yüzden uzuv 2'nin payı L2 cos(θ2 − β) olarak yazılır.
3. **Ters kinematik ne zaman "erişilemez" der?** — Kosinüs teoreminden çıkan cos φ'nin mutlak değeri 1'den büyükse L2, L3 ve bilek uzaklığıyla üçgen kurulamaz (satır 58–60). Çözüm var ama bir açı sınır dışındaysa `JointLimit` döner. Fonksiyon hiçbir zaman NaN veya istisna döndürmez.
4. **"Dirsek yukarı" ve "dirsek aşağı" ne demek?** — Aynı takım noktasına iki farklı dirsek açısıyla gidilebilir: φ = ±acos(cos φ). `elbowUp` true ise φ negatif dal seçilir (satır 62).
5. **M(q) neden simetrik ve pozitif tanımlı?** — Kinetik enerji T = ½ q̇ᵀ M q̇ her q̇ ≠ 0 için pozitiftir; bu, M'nin pozitif tanımlı olması demektir. Simetri tanımdan gelir. Test `MassMatrixIsSymmetricPositiveDefinite` 200 rastgele pozda bunu denetler.
6. **q̈ bulmak için neden M⁻¹ alınmıyor?** — M simetrik pozitif tanımlı olduğundan M = L Lᵀ (Cholesky) ayrışır. Önce L y = b, sonra Lᵀ q̈ = y çözülür. Ters matristen daha az işlem yapar ve sayısal olarak daha kararlıdır. Cholesky başarısız olursa (köşegende ≤ 0) M fiziksel değildir ve istisna atılır.
7. **M'nin ilk satırında neden M12 = M13 = M14 = 0?** — Taban dönüşü eğim eklemleriyle bağlaşmaz: Ixy = Iyz = 0 kabulüyle (SPEC §3.2) uzuvlar eğim düzlemine göre simetriktir. m11 ise kolun ne kadar uzandığına bağlıdır (ρ_i, eş. 15).
8. **Coriolis terimi nasıl hesaplanıyor?** — Christoffel sembolleriyle: c_k = Σ Σ ½ (∂M_kj/∂q_i + ∂M_ki/∂q_j − ∂M_ij/∂q_k) q̇_i q̇_j (`CoriolisVector`). ∂M/∂q sayısal türevle değil analitik olarak hesaplanır (`MassMatrixFormulas.Derivative`); test bunu merkezî farkla karşılaştırır. Hız sıfırsa c = 0.
9. **g1 neden sıfır? g(q) doğru mu, nasıl biliyoruz?** — Taban ekseni düşeydir; yerçekimi düşey eksen etrafında moment üretmez. Doğruluk iki yoldan denetlenir: g(q) = ∂V/∂q olmalıdır (`GravityIsGradientOfPotentialEnergy`, sayısal türev) ve bağımsız Python modelinin `test_vectors.json` değerleriyle 1e-9 bağıl hatayla aynıdır.
10. **Yük nasıl ekleniyor?** — Ayrı bir cisim olarak değil: uzuv 4 ile uçtaki nokta kütle tek bir bileşik cisim yapılır (`Payload.ApplyTo`). Yeni kütle m4 + mp, yeni kütle merkezi ağırlıklı ortalama, eylemsizlik paralel eksen (Steiner) teoremiyle büyür. Asıl parametreler `Clone()` sayesinde değişmez.

Yedek soru: **Sürtünmede neden tanh var?** — Coulomb sürtünmesi fc·sign(q̇) sıfır hızda sıçrar; çözücü bu süreksizlikte takılır. tanh(q̇/ωs) aynı şekli yumuşak verir. ωs küçüldükçe sıçramaya yaklaşır ve sistem sertleşir.

### "Göster" görevleri

1. **"g'yi 20 yap ve yerçekimi torkunun kodda nasıl değiştiğini göster."** — Ortam ve yük sekmesinde g kutusuna 20 yazın (kutu en çok 100 kabul eder; denetim `SettingsValidator.cs` satır 95, `MaxGravityMps2`). Yol: `EnvironmentPanel.ApplyTo` (`EnvironmentPanel.cs:38`) → `LabSession.Reset` → `OpenLoopSimulator.Reset` (`OpenLoopSimulator.cs:90`, `new ArmDynamics(..., Environment.GravityMps2)`) → `ArmDynamics.GravityVector` satır 70 `double g = GravityMps2;` ve satır 75–77. g2, g3, g4'ün her terimi g ile çarpılır, yani bütün yerçekimi torkları 20 / 9,81 ≈ 2,04 kat büyür. Torklar grafiğinde kolu tutmak için gereken tork aynı oranda artar; tork modunda 0 girişle kol daha hızlı düşer.
2. **"q = 0 pozunda takım noktası nerede? Elle hesapla, kodla karşılaştır."** — `ForwardKinematics.cs` satır 37–40: θ = 0 iken r = L2 cos(−β) + L3 + L4 = a2x + L3 + L4 = 0,024 + 0,124 + 0,126 = 0,274 m; z = d1 − L2 sin(−β) = d1 + a2z = 0,0595 + 0,128 = 0,1875 m. Takım (0,274; 0; 0,1875) m'dedir. Formda bütün açılar 0 iken 3B görünümdeki kırmızı küre buradadır; CSV'deki x_m, z_m sütunları da aynı değeri verir.
3. **"0,5 kg yük ekle; M44'ün neden büyüdüğünü göster."** — `Payload.ApplyTo` (satır 28): yük takım noktasında (tx = L4) olduğundan bileşik kütle merkezi uca kayar, ℓ4 ve m4 büyür, Iyy4 Steiner terimiyle artar (satır 51–54). `MassMatrixFormulas.FillPitchBlock` içinde M44 = J4 + m4·ℓ4² olduğundan bilek ekleminin eylemsizliği belirgin biçimde artar. Test: `DynamicsTests.MassMatrixAndGravityMatchTestVectors` 0,5 kg pozlarını da içerir.

---

## Rol 2 — Çözücüler, motor, açık çevrim simülasyonu, oturum

### Dosyalar

```
lab/RobotArmLab.Core/Solvers/IOdeSolver.cs          ẋ = f(t, x) temsilcisi ve çözücü arayüzü
lab/RobotArmLab.Core/Solvers/VectorOps.cs           x + a·k
lab/RobotArmLab.Core/Solvers/EulerSolver.cs, HeunSolver.cs, Rk4Solver.cs, DormandPrinceSolver.cs
lab/RobotArmLab.Core/Motor/DcMotor.cs               XM430-W350 gerilim → tork modeli ve tersi
lab/RobotArmLab.Core/Simulation/ArmState.cs         q, q̇ ↔ x = [q; q̇]
lab/RobotArmLab.Core/Simulation/InputMode.cs        tork / gerilim; sabit / basamak / sinüs
lab/RobotArmLab.Core/Simulation/JointInput.cs       bir eklemin giriş sinyali u(t)
lab/RobotArmLab.Core/Simulation/OpenLoopSimulator.cs  durum denklemi, adım, hata denetimi, örnek
lab/RobotArmLab.Core/Simulation/SimulationSample.cs, StopReason.cs
lab/RobotArmLab.Core/Session/LabSession.cs          formun arkasındaki oturum: Reset, Advance, CSV
lab/RobotArmLab.Core/Session/SolverKind.cs          çözücü seçimi ve fabrikası
lab/RobotArmLab.Core/Session/CsvExporter.cs         örnekleri CSV'ye yazar
lab/RobotArmLab.Core/Session/ClosedLoopSettings.cs  kapalı çevrim ayarları (kontrolcü, Ts, görev, …)
lab/RobotArmLab.Core/Session/ClosedLoopBuilder.cs   ayarlardan kapalı çevrimi kurar
lab/RobotArmLab.Tests/SolverTests.cs, MotorTests.cs, OpenLoopTests.cs, LabSessionTests.cs, LabScenarioTests.cs
```

### Okuma sırası

1. `IOdeSolver.cs`, `EulerSolver.cs`, `HeunSolver.cs`, `Rk4Solver.cs`: aynı arayüz, artan mertebe.
2. `DormandPrinceSolver.cs`: `Step` (adım kabul/ret döngüsü), sonra `TryStep` (7 eğim, iki çözüm, hata normu).
3. `DcMotor.cs`: sınıf açıklamasındaki türetme, `Xm430W350` (43), `Torque` (62), `VoltageFor` (75).
4. `ArmState.cs`, `JointInput.cs`, `OpenLoopSimulator.cs`: `Reset` (87), `Step` (112), `Derivatives` (158), `AppliedTorque` (186), `CheckFaults` (202).
5. `LabSession.cs`: ayar özellikleri, `Reset` (151), `Advance` (192).
6. `SolverKind.cs`, `CsvExporter.cs`, `ClosedLoopSettings.cs`, `ClosedLoopBuilder.cs`.

### Hocanın sorabileceği 10 soru

1. **Durum vektörü nedir, çözücü neyi çözer?** — x = [q1..q4, q̇1..q̇4], 8 eleman. ẋ = f(t, x) = [q̇; q̈], q̈ ileri dinamikten gelir (`OpenLoopSimulator.Derivatives`). Çözücü bu fonksiyonun ne olduğunu bilmez; yalnız `OdeFunction` temsilcisi olarak çağırır.
2. **RK4 bir adımı nasıl atar?** — Dört eğim: başta k1, ortada k2 ve k3, sonda k4; x + h/6·(k1 + 2k2 + 2k3 + k4). Global hata h⁴ ile orantılıdır: h yarıya inince hata yaklaşık 16 kat küçülür (`SolverTests` bunu ölçer).
3. **Euler ile sürtünmesiz kolda ne olur?** — Açık Euler eğimi adımın başında ölçer; salınımda her adım yörüngenin dışına doğru taşar, toplam enerji T + V koşu boyunca artar. RK4'te aynı adımda sapma çok küçüktür. Enerji sekmesinde görülür (sürtünme 0, giriş 0).
4. **Dormand–Prince adımı nasıl seçer?** — Her denemede 5. ve 4. mertebe iki çözüm bulur; farkları hata tahminidir. Hata, atol + rtol·|x| ölçeğine bölünüp karekök-ortalama-kare alınır. ≤ 1 ise adım kabul edilir. Yeni adım h·0,9·(1/hata)^(1/5), 0,2 ile 5 kat arasında sınırlı. Kullanıcı h girmez; örnekler 5 ms aralıkla alınır.
5. **Gerilim modunda tork nasıl hesaplanır?** — `DcMotor.Torque`: gerilim ±12 V'a kırpılır, akım I = (V − N·Ke·q̇)/R, tork τ = N·Kt·I, sonra ±τmax. R = 12/2,3 = 5,217 Ω; N·Kt = 4,1/2,3 = 1,783 N·m/A; N·Ke = 12/(46 rpm) = 2,491 V·s/rad. Bu değerler veri sayfasındaki tutulma ve yüksüz hız noktalarından türetilir.
6. **Motor modelinde neden endüktans yok?** — Veri sayfası vermez; bu boyda motorlarda L/R ≈ 0,1 ms'dir, kolun mekanik zaman sabitleri milisaniyeler–saniyelerdir. Akım gerilime anında uyar kabul edilir, durum 8 değişkenli kalır.
7. **Simülasyon gerçek zamanla nasıl eşleşiyor?** — Her tıkta Stopwatch geçen gerçek süreyi verir. `Advance` hedef zamanı süre × hız çarpanı kadar ileri alır ve simülasyon o zamana gelene kadar adım atar. Bir çağrıda en çok 0,25 s ve 20 000 adım işlenir; yetişilemezse geride kalan süre atılır, arayüz donmaz.
8. **Simülasyon ne zaman ve neden durur?** — `CheckFaults`: durumda NaN veya ∞ (sayısal patlama; adım çok büyük), eklem sınırı aşımı, takım noktasının zeminin (z = 0) altına inmesi. Durma nedeni `StopReason`'a, Türkçe açıklama `StopMessage`'a yazılır ve araç çubuğunda görünür.
9. **Hatalı bir ayar programı çökertir mi?** — Hayır. `LabSession.Reset` önce `Validate` çağırır; sorunlar Türkçe cümle listesi olarak döner, simülatör değişmez, istisna atılmaz. Form listeyi `MessageBox` ile gösterir.
10. **CSV neden InvariantCulture ile yazılıyor?** — Türkçe Windows'ta ondalık ayırıcı virgüldür; "0,5" yazılsaydı CSV'nin virgül ayırıcısıyla karışırdı. Sayılar her zaman noktayla yazılır, Excel, MATLAB ve Python dosyayı doğru okur.

Yedek soru: **Kapalı çevrimde Ts neden h'nin tam katı olmalı?** — Tesis bir örnekleme aralığında Ts/h adım atar (`OpenLoopSimulator.Step`, `stepsPerFrame`). Tam kat değilse kontrolcü örneği adım sınırına denk gelmez ve gerçek örnekleme süresi istenenden farklı olurdu (`ClosedLoopValidator` bunu bildirir).

### "Göster" görevleri

1. **"Euler'e geç, h = 5 ms yap; enerjinin neden korunmadığını göster."** — Kol parametrelerinde fv ve fc'yi 0 yapın, Giriş sekmesinde başlangıç açılarını (0°, −60°, 30°, 30°) ve bütün girişleri 0 yapın, Çözücü sekmesinde Euler ve 5 ms seçin, Enerji sekmesini açıp Başlat'a basın. Kol düşer ve yaklaşık 0,14 s'de eklem sınırına çarpar; bu sürede T + V, 0,758 J'lük toplamdan yaklaşık 0,01 J sapar (`LabScenarioTests.EulerAtFiveMillisecondsDriftsInEnergyAndRk4DoesNot`). Kod yolu: `SolverPanel.ApplyTo` → `LabSession.Reset` → `SolverFactory.Create` (`SolverKind.cs:40`, `new EulerSolver()`) → her adımda `EulerSolver.Step` (satır 10): yalnız k1 = f(t, x) ile x + h·k1. Aynı deney RK4 ile tekrarlanınca sapma 1e-6 J'ün altındadır, T + V çizgisi düz kalır.
2. **"Eklem 2'ye 12 V basamak ver; kol dururken üretilen torku hesapla ve kodda göster."** — Giriş sekmesinde Gerilim modu, eklem 2 Basamak, genlik 12. `OpenLoopSimulator.AppliedTorque` (186) → `DcMotor.Torque` (62): q̇ = 0 iken I = 12/5,217 = 2,3 A, τ = 1,783 × 2,3 = 4,1 N·m (tutulma torku), doyum sınırına eşit. Kol hızlandıkça zıt EMK (N·Ke·q̇) akımı düşürür; Torklar grafiğinde τ2 hız arttıkça azalır.
3. **"Oynatma hızını 4× yap; kodda nerede etkilediğini göster."** — `MainForm.cmbSpeed_SelectedIndexChanged` → `session.SpeedFactor = 4` (Sıfırla gerekmez). `LabSession.Advance` satır 199: `targetTimeS += wall × SpeedFactor`. Aynı tıkta dört kat fazla adım atılır; fizik değişmez, yalnız ekranda daha hızlı akar.

---

## Rol 3 — Kontrolcüler, görevler, algılayıcı, ölçütler, karşılaştırma

### Dosyalar

```
lab/RobotArmLab.Core/Controllers/IController.cs
lab/RobotArmLab.Core/Controllers/PidController.cs, PidGains.cs
lab/RobotArmLab.Core/Controllers/ComputedTorqueController.cs (+ HalfSample), ComputedTorqueGains.cs
lab/RobotArmLab.Core/Controllers/SuperTwistingTerm.cs, SuperTwistingGains.cs
lab/RobotArmLab.Core/Controllers/SuperTwistingController.cs, FeedForwardStsmcController.cs
lab/RobotArmLab.Core/Tasks/IReferenceTask.cs (+ JointReference), JointStepTask.cs, QuinticTask.cs,
                           CartesianTask.cs, CircleTask.cs
lab/RobotArmLab.Core/Simulation/JointSensor.cs      kusursuz ölçüm / kodlayıcı + gürültü
lab/RobotArmLab.Core/Simulation/ActuatorKind.cs     ideal tork / motor
lab/RobotArmLab.Core/Simulation/ClosedLoopSimulator.cs  örneklemeli kapalı çevrim (ZOH)
lab/RobotArmLab.Core/Session/ClosedLoopValidator.cs kapalı çevrim ayar denetimi
lab/RobotArmLab.Core/Session/ControlMetrics.cs      başarım ölçütleri
lab/RobotArmLab.Core/Session/ControllerComparison.cs, ComparisonCsv.cs
lab/RobotArmLab.Core/Session/TaskGeometry.cs        görevin 3B'de çizilecek hedefi
lab/RobotArmLab.Tests/ClosedLoopPartsTests.cs, ClosedLoopTests.cs, ClosedLoopScenarioTests.cs,
                      ClosedLoopSessionTests.cs, MetricsAndComparisonTests.cs
```

### Okuma sırası

1. `IController.cs`, `PidController.cs` (en basit kontrolcü, sargı önleme).
2. `ComputedTorqueController.cs`: `Compute` (44), `HalfSample` (62).
3. `SuperTwistingTerm.cs`, `SuperTwistingController.cs`, sonra `FeedForwardStsmcController.cs` (farkı tek satırdır: model referansta hesaplanır).
4. `IReferenceTask.cs`, `JointStepTask.cs`, `QuinticTask.cs`, `CartesianTask.cs`, `CircleTask.cs`.
5. `JointSensor.cs`, `ActuatorKind.cs`, `ClosedLoopSimulator.cs`: `Step` (89).
6. `ClosedLoopValidator.cs`, `ControlMetrics.cs`, `ControllerComparison.cs`, `ComparisonCsv.cs`, `TaskGeometry.cs`.

### Hocanın sorabileceği 10 soru

1. **PID yerçekimini nasıl karşılar? Sargı önleme nedir?** — Yerçekimi modeli yoktur; kalıcı hatayı integral terimi giderir. Tork ±τmax'ta kırpılıyorsa o örnekte integral güncellenmez (`PidController.cs` satır 49–53). Yoksa motor doyumdayken integral büyümeye devam eder, doyumdan çıkınca büyük aşma olurdu.
2. **Hesaplanmış tork nasıl çalışır?** — τ = M̂(q)(q̈_ref + Kd ė + Kp e) + ĉ + ĝ + f̂. Model doğruysa doğrusal olmayan terimler sökülür, her eklem ë + Kd ė + Kp e = 0 gibi davranır. Varsayılan Kp = 1600 1/s² → ωn = √Kp = 40 rad/s; Kd = 350 1/s → ζ = Kd/(2√Kp) ≈ 4,4 (aşırı sönümlü, aşma yok).
3. **Yarım örnek öngörüsü neden var?** — Tork Ts boyunca sabit tutulur (ZOH), ama kol bu sürede hareket eder. Model, kolun tutma aralığının ortasında olacağı tahmini durumda hesaplanır: q̄ = q + q̇·Δt/2 + a·Δt²/8, q̄̇ = q̇ + a·Δt/2 (`HalfSample.Predict`). Hafif bilekte bayat sürtünme ve yerçekimi telafisinin yarattığı hatayı azaltır.
4. **Süper-büküm kontrolcüsü nedir?** — Kayma yüzeyi s = ė + Λe. Geri besleme u_st = k1·|s|^½·sign(s) + v, v̇ = k2·sign(s). Model hatası (habersiz yük) ṡ'ye bozucu olarak girer; türevi L ile sınırlıysa ve k2 > L ise s sonlu sürede sıfıra iner (Levant 1998). s = 0 olunca hata ė = −Λe ile sönümlenir. sign doğrudan torka değil integrale (v) girdiği için tork süreklidir, titreşim azdır.
5. **FF-STSMC'nin STSMC'den farkı ne?** — Model terimi ölçülen durumda değil, istenen yörüngede hesaplanır: `model.InverseDynamics(qRef, qdRef, a)`. Kodlayıcı nicemlemesi ve gürültü modelin içinden geçmez, tork komutu daha az gürültülüdür. Bedeli: kol referanstan uzaktayken aradaki fark geri beslemeye bozucu olarak yüklenir.
6. **"Tahmini yük" ile "yük" farkı?** — Yük (Ortam sekmesi) gerçek koldadır, tesisin dinamiğine girer. Tahmini yük (Kontrol sekmesi) yalnız kontrolcünün kendi modeline girer (`ClosedLoopBuilder.cs:57`). İkisi farklıysa model yanlıştır; kontrolcülerin model hatasına dayanıklılığı böyle denenir.
7. **Kodlayıcı nasıl modelleniyor?** — Ölçülen açı q_m = r·round((q + n)/r), r = 2π/4096 ≈ 1,53 mrad, n tohumlu Gauss gürültüsü. Hız geri farkla bulunur: (q_m − q_m,önceki)/Ts. İlk örnekte önceki açı olmadığından hız 0'dır. Aynı tohum aynı ölçümü verir.
8. **Sıfırıncı dereceden tutucu (ZOH) kodda nerede?** — `ClosedLoopSimulator.Step` satır 100: kontrolcü her Ts'de bir kez çalışır ve tesis girişinin (`JointInput`, şekli Constant) genliğini yazar. Tesis sonraki Ts boyunca bu sabit girişle Ts/h adım atar.
9. **Beşinci derece yörünge neden?** — p(s) = 10s³ − 15s⁴ + 6s⁵, s = (t − t0)/T. Başta ve sonda hız ve ivme sıfırdır; referansta sıçrama olmadığından tork da sıçramaz. Eklem basamağında ise hata ilk anda tam basamak yüksekliğidir, tork doyuma girer.
10. **Ölçütler nasıl hesaplanıyor?** — `ControlMetrics.Compute`, kaydedilmiş bütün örneklerden: RMS, en büyük ve son eklem hatası; takım hatası |p − FK(q_ref)| (mm); oturma süresi (en büyük hatanın 0,005 rad bandından son çıktığı an; sonda dışarıdaysa "oturmadı"); yüzde aşma; en büyük tork; doyum süresi; ∫Σ τ² dt; titreşim göstergesi Σ|Δτ_cmd|/süre.

Yedek soru: **Motor eyleyicisinde neden h ≤ 0,2 ms?** — Zıt EMK eşdeğer sönümü b = (N·Kt)(N·Ke)/R ≈ 0,85 N·m·s/rad. Hafif bilekte özdeğer λ ≈ b/M ≈ 5600 1/s. h = 0,5 ms'de hλ ≈ −2,8, RK4'ün gerçek eksendeki kararlılık sınırıdır (−2,785). Kol sahte bir dengede takılır. h = 0,2 ms'de hλ ≈ −1,1 güvenlidir.

### "Göster" görevleri

1. **"CT'de Kp'yi 400'e düşür; ne değişir, kodda nerede?"** — Kontrol sekmesinde kazanç tablosunda Kp = 400. `ComputedTorqueController.Compute` satır 52: a = q̈_ref + Kd·ė + Kp·e. ωn = 20 rad/s'ye iner; ζ = 350/40 ≈ 8,8 olur, daha da aşırı sönümlü ve yavaş. Hata sekmesinde e daha büyük ve yavaş söner.
2. **"Gerçek yük 0, tahmini yük 0,3 kg iken CT ile STSMC'yi karşılaştır."** — Ortam sekmesinde yük 0, Kontrol sekmesinde tahmini yük 0,3. Karşılaştır sekmesinde "Karşılaştır" → `ControllerComparison.RunOne` her kontrolcüyü aynı ayarlarla koşturur. Kontrolcünün modeli `ClosedLoopBuilder.cs:57`'de tahmini yükle kurulur, tesis gerçek yükle. CT'nin bilekteki giriş kazancı yaklaşık 13 kat fazla olur; tork titreşir ve kol eklem sınırına çarpar (tabloda Not sütunu). STSMC ve FF-STSMC hatayı birkaç yüzde radyanda tutar (`ClosedLoopTests.OverestimatedPayloadIsHandledBySlidingModeButNotByComputedTorque`). Ters durum (gerçek 0,3 kg, tahmini 0) STSMC'yi standart kazançlarla zorlar: CT sınırlı ama büyük hatayla izler, STSMC'nin hatası 1 rad'ı geçer, FF-STSMC eklem 3 sınırına çarpar (`ClosedLoopScenarioTests.C2_UnderestimatedPayload`). Bu yüzden "hangi kontrolcü daha dayanıklı" sorusunun cevabı hatanın yönüne bağlıdır.
3. **"Kodlayıcıyı aç, gürültüyü 1 mrad yap; ölçümün nerede bozulduğunu göster."** — Kontrol sekmesinde algılayıcı Kodlayıcı, gürültü 1 mrad. `JointSensor.Measure` satır 70: açıya gürültü eklenir, satır 71: çözünürlüğe yuvarlanır, satır 72: hız farkla bulunur (gürültü Ts'ye bölündüğü için hız gürültüsü büyür). Torklar grafiğinde kesikli komut çizgisi titrer. FF-STSMC'nin komutu CT'ninkinden daha az titrer, çünkü model gürültülü ölçümden değil referanstan hesaplanır (soru 5; `ClosedLoopTests.FeedForwardStsmcCommandIsSmootherThanComputedTorqueUnderEncoderNoise`).

---

## Rol 4 — Forms arayüzü ve grafikler

### Dosyalar

```
lab/RobotArmLab.Forms/Program.cs                    giriş noktası, [STAThread]
lab/RobotArmLab.Forms/MainForm.cs (+ .Designer.cs)  ana pencere: bağlantılar, zamanlayıcı, ShowState
lab/RobotArmLab.Forms/EnvironmentPanel.cs (+ .Designer.cs)  ortam ve yük sekmesi
lab/RobotArmLab.Forms/SolverPanel.cs (+ .Designer.cs)       çözücü sekmesi
lab/RobotArmLab.Forms/InputPanel.cs (+ .Designer.cs), InputGridBinder.cs   giriş tablosu
lab/RobotArmLab.Forms/ControlPanel.cs (+ .Designer.cs)      kontrol sekmesi
lab/RobotArmLab.Forms/TaskPanel.cs (+ .Designer.cs), CirclePickPanel.cs (+ .Designer.cs)  görev sekmesi
lab/RobotArmLab.Forms/NumericBox.cs                 NumericUpDown'a güvenli değer yazma
lab/RobotArmLab.Forms/PlotPanel.cs                  ScottPlot grafik sekmeleri
lab/RobotArmLab.Forms/ComparePanel.cs               Karşılaştır sekmesi
lab/RobotArmLab.Forms/ArmView2D.cs                  GDI+ yan görünüm
lab/RobotArmLab.Forms/ArmView3D.cs                  HelixToolkit 3B görünüm (WPF, ElementHost)
lab/RobotArmLab.Forms/RobotArmLab.Forms.csproj      hedef çerçeve, WinForms + WPF, NuGet paketleri
```

### Okuma sırası

1. `Program.cs`, `MainForm.cs`: kurucu, `CreateRuntimeViews`, `ApplyControlsToSession`, `ResetFromControls`, `btnStart_Click`, `timer_Tick`, `ShowState`.
2. `MainForm.Designer.cs`: yerleşim (`splitMain`, `tabSettings`, `splitViews`) ve olay bağlantıları (`btnStart.Click`, `timer.Tick`).
3. Sekme panelleri: her birinde aynı desen vardır, `LoadFrom(session)` ve `ApplyTo(session)`.
4. `PlotPanel.cs`, `ComparePanel.cs`.
5. `ArmView2D.cs`, `ArmView3D.cs`.

### Hocanın sorabileceği 10 soru

1. **Fizik formda mı?** — Hayır. `MainForm` yalnız bağlantı yapar: kutulardaki değerleri `LabSession`'a yazar, düğmelerde oturumun metotlarını çağırır, sonuçları çizer. Bütün mantık çekirdektedir; bu yüzden testler pencere açmadan çalışır ve çekirdek Mac'te de derlenir.
2. **Zamanlayıcı nasıl çalışıyor, neden `while` yok?** — `System.Windows.Forms.Timer`, 30 ms aralık, olayı pencere iş parçacığında çalıştırır. Her tıkta geçen gerçek süre kadar ilerlenir (`KOD_TURU.md` §1b). `while` döngüsü mesaj kuyruğunu kilitler: düğmeler ve çizim donar.
3. **Her sekme oturumla nasıl konuşur?** — Hepsinde iki metot vardır: `LoadFrom(session)` oturumdaki değeri kutulara yazar (açılışta), `ApplyTo(session)` kutulardaki değeri oturuma yazar (Başlat/Sıfırla'da, `MainForm.ApplyControlsToSession`). Kutuların değeri doğrulanmaz; doğrulama `LabSession.Validate`'tedir.
4. **Kol parametreleri tablosu neden kod yazmadan çalışıyor?** — `PropertyGrid.SelectedObject = session.Parameters`. PropertyGrid nesnenin özelliklerini yansıma (reflection) ile okur; `[Category]`, `[DisplayName]`, `[Description]` öznitelikleri Türkçe ad ve açıklamayı verir (`ArmParameters.cs`). Kazanç tablosu da aynı yolla çalışır.
5. **Derece–radyan ve mm–m çevirisi nerede?** — Yalnız arayüzde: `InputGridBinder` (q0, q̇0), `TaskPanel` (`Deg` sabiti, x, y, z mm), `CirclePickPanel` (yarıçap mm), `PlotPanel.Values` (grafikte derece). Çekirdek her yerde SI (rad, m) kullanır.
6. **`NumericBox.Set` neden var?** — `NumericUpDown.Value`'ya kutunun en küçük–en büyük aralığı dışında bir değer yazılırsa istisna atılır. Oturumda (ör. testten) sınır dışı bir değer olsa bile form çökmesin diye değer önce kırpılır.
7. **Grafikler nasıl güncelleniyor?** — Her ScottPlot çizgisi `time` ve kendi seri listesine bağlıdır (kopya yok). `Append` yeni örnekleri 5 ms aralıkla listelere ekler; `RefreshVisible` yalnız açık sekmeyi eksenleri uydurup yeniden çizer. Bütün seriler tek dizide tutulur; her grubun başladığı numara `AngleSeries`, `TorqueSeries` gibi adlı sabitlerdir.
8. **3B görünüm nasıl forma gömülü? Açılmazsa ne olur?** — HelixToolkit bir WPF kütüphanesidir; `ElementHost` WPF denetimini Windows Forms'a gömer. Bunun için `[STAThread]` gerekir (`Program.cs`). Kurulum `MainForm.CreateRuntimeViews` içinde `try` ile yapılır; başarısız olursa yalnız 2B görünüm kalır ve neden durum çubuğunda yazılır.
9. **2B görünüm kinematik hesaplıyor mu?** — Hayır. Çekirdeğin verdiği eklem noktalarını q1 düzlemine izdüşürür: r = x cos q1 + y sin q1, z aynı (`ArmView2D.SetPose`). `ToScreen` metreyi piksele çevirir; ekranda y aşağı doğru olduğundan z ters çevrilir.
10. **Türkçe ondalık virgül nasıl ele alınıyor?** — Tablo hücreleri `ValueType = double` olduğundan bilgisayarın dil ayarıyla okunur ("0,5"). Tolerans kutuları hem Türkçe hem nokta yazımı kabul eder (`SolverPanel.ParseNumber`). CSV her zaman noktayla yazılır (`CsvExporter`).

Yedek soru: **Karşılaştır paneli diğer sekmeleri nasıl biliyor?** — Bilmez. `MainForm` panelin `PrepareSession` temsilcisine kendi `ApplyControlsToSession` metodunu verir (`MainForm.cs:48`); panel düğmeye basılınca bunu çağırıp oturumu alır.

### "Göster" görevleri

1. **"Hız listesine 8× ekle."** — İki yer: `MainForm.Designer.cs:89` `cmbSpeed.Items.AddRange(... "4×", "8×")` ve `MainForm.cs:20` `Speeds = { 0.25, 0.5, 1.0, 2.0, 4.0, 8.0 }`. İki dizi aynı sırada olmalıdır, çünkü `cmbSpeed_SelectedIndexChanged` seçili sırayı `Speeds` dizisinde arar. Üst sınır 100× (`SettingsValidator.MaxSpeedFactor`) olduğundan denetim geçer.
2. **"Grafiklere 5 ms yerine 1 ms'de bir nokta ekle."** — `PlotPanel.cs:19` `PlotIntervalS = 0.001`. `Append` (satır 90) bu aralıktan sık örnekleri atlar. Etkisi: eğriler daha ayrıntılı, uzun koşularda çizim yavaşlar. Simülasyon sonucu değişmez; yalnız gösterilen nokta sayısı değişir.
3. **"Yan görünümde uzuvları kırmızı ve daha kalın çiz."** — `ArmView2D.OnPaint`, satır 64: `new Pen(Color.SteelBlue, 6f)` → `new Pen(Color.Red, 10f)`. `SetPose` her tıkta `Invalidate()` çağırdığından Windows `OnPaint`'i yeniden çalıştırır.
