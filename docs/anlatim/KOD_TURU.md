# Kod turu: bir zamanlayıcı tıkı ve bir kontrolcü örneği

Bu belge `lab/RobotArmLab.sln` içindeki kodu çağrı sırasıyla izler. Her satırda dosya, metot ve satır numarası vardır; Visual Studio'da dosyayı açıp **Ctrl+G** ile satıra gidilebilir. Satır numaraları W4 commit'indeki koda göredir. Kod değişirse numaralar kayabilir; metot adları değişmez.

Kısaltmalar: `Forms/` = `lab/RobotArmLab.Forms/`, `Core/` = `lab/RobotArmLab.Core/`.

---

## 1. Başlat düğmesinden çözücüye ve grafiklere: bir zamanlayıcı tıkı

Varsayılan ayarlarla izlenir: açık çevrim, RK4, h = 0,5 ms, oynatma hızı 1×.

### 1a. Başlat'a basılınca (bir kez)

| # | Dosya:satır | Metot | Ne olur |
|---|---|---|---|
| 1 | `Forms/MainForm.Designer.cs:82` | `InitializeComponent` | `btnStart.Click += btnStart_Click;` Düğme olaya burada bağlanır. |
| 2 | `Forms/MainForm.cs:109` | `btnStart_Click` | Düğmeye basılınca çalışır. |
| 3 | `Forms/MainForm.cs:111` | `btnStart_Click` | Koşu yeni başlıyorsa (`IsAtStart`) veya bitmişse (`!CanContinue`) ayarlar uygulanır. Duraklatılmış koşu kaldığı yerden sürer. |
| 4 | `Forms/MainForm.cs:87` | `ResetFromControls` | |
| 5 | `Forms/MainForm.cs:89` → `:76` | `ApplyControlsToSession` | Her sekme kendi kutularını oturuma yazar (`:78`–`:82`): `environmentPanel.ApplyTo`, `solverPanel.ApplyTo`, `inputPanel.ApplyTo`, `controlPanel.ApplyTo`, `taskPanel.ApplyTo`. Örnek: `Forms/SolverPanel.cs:39` `session.StepSizeS = numStepMs.Value / 1000`. |
| 6 | `Forms/MainForm.cs:90` → `Core/Session/LabSession.cs:151` | `LabSession.Reset` | Simülasyon baştan kurulur. |
| 7 | `Core/Session/LabSession.cs:153` → `:141` → `Core/Session/SettingsValidator.cs:27` | `Validate` | Bütün ayarlar denetlenir. Hata varsa Türkçe liste döner, hiçbir şey değişmez ve form `MessageBox` gösterir (`MainForm.cs:91`–`:96`). |
| 8 | `Core/Session/LabSession.cs:158`–`:159` | `Reset` | `new OpenLoopSimulator(...)`. Parametrelerin ve ortamın kopyası verilir. Çözücü `SolverFactory.Create` ile kurulur (`Core/Session/SolverKind.cs:40`; RK4 için `new Rk4Solver()`). |
| 9 | `Core/Session/LabSession.cs:163`–`:164` | `Reset` | `sim.StepSizeS = h` ve `sim.FrameTimeS = h`. Böylece her `sim.Step()` çağrısı **tek bir** çözücü adımı atar; kaç adım atılacağına `Advance` karar verir. |
| 10 | `Core/Session/LabSession.cs:178` → `Core/Simulation/OpenLoopSimulator.cs:87` | `OpenLoopSimulator.Reset` | `:89` yük uzuv 4'e katılır (`Payload.ApplyTo`), `:90` `new ArmDynamics(parametreler, g)`, `:92` başlangıç durumu. |
| 11 | `Core/Session/LabSession.cs:183` | `Reset` | t = 0 örneği geçmişe (`History`) eklenir. |
| 12 | `Forms/MainForm.cs:98`–`:104` | `ResetFromControls` | Grafikler temizlenir ve t = 0 örneği çizilir; 3B hedef ayarlanır; `ShowState()`. |
| 13 | `Forms/MainForm.cs:116`–`:117` | `btnStart_Click` | `clock.Restart()` (Stopwatch, gerçek süreyi ölçer) ve `timer.Start()`. |

### 1b. Bir tık (her ~30 ms'de bir)

`Forms/MainForm.Designer.cs:177`–`:178`: `timer.Interval = 30; timer.Tick += timer_Tick;`. Bu bir `System.Windows.Forms.Timer` zamanlayıcısıdır: olayı pencerenin kendi iş parçacığında çalıştırır, bu yüzden kod denetimlere doğrudan dokunabilir.

| # | Dosya:satır | Metot | Ne olur |
|---|---|---|---|
| 1 | `Forms/MainForm.cs:133` | `timer_Tick` | |
| 2 | `Forms/MainForm.cs:135`–`:136` | `timer_Tick` | Önceki tıktan beri geçen gerçek süre okunur (ör. 0,031 s), Stopwatch sıfırlanır. |
| 3 | `Forms/MainForm.cs:137` → `Core/Session/LabSession.cs:192` | `LabSession.Advance(wallSeconds)` | |
| 4 | `Core/Session/LabSession.cs:198` | `Advance` | Gerçek süre en çok 0,25 s alınır (`MaxWallClockPerCallS`); pencere sürüklenip takılırsa biriken süre atılır. |
| 5 | `Core/Session/LabSession.cs:199` | `Advance` | `targetTimeS += wall × SpeedFactor`. Simülasyonun bu tıkın sonunda varması gereken zaman. 1×'te 0,031 s ileri. |
| 6 | `Core/Session/LabSession.cs:202`–`:205` | `Advance` | `while` döngüsü: simülasyon zamanı hedefin yarım adım gerisinde kaldıkça adım atar. En çok 20 000 adım (`MaxStepsPerCall`). 0,031 s / 0,5 ms ≈ 62 adım. |
| 7 | `Core/Session/LabSession.cs:204` → `Core/Simulation/OpenLoopSimulator.cs:112` | `OpenLoopSimulator.Step` | Açık çevrimde `loop` null olduğundan doğrudan tesis ilerler. |
| 8 | `Core/Simulation/OpenLoopSimulator.cs:115` | `Step` | `stepsPerFrame = FrameTimeS / StepSizeS = 1`. |
| 9 | `Core/Simulation/OpenLoopSimulator.cs:121` → `Core/Solvers/Rk4Solver.cs:10` | `Rk4Solver.Step(Derivatives, t, x, h)` | x = [q1..q4, q̇1..q̇4]. Çözücüye sağ taraf fonksiyonu `Derivatives` temsilci (delegate) olarak verilir. |
| 10 | `Core/Solvers/Rk4Solver.cs:12`–`:15` | `Step` | Dört eğim: k1 = f(t, x), k2 ve k3 adımın ortasında, k4 sonunda. Ara durumlar `VectorOps.AddScaled` ile (`Core/Solvers/VectorOps.cs:7`). Her `f(...)` çağrısı aşağıdaki 11–16. satırları çalıştırır. |
| 11 | → `Core/Simulation/OpenLoopSimulator.cs:158` | `Derivatives(t, x)` | Durum denkleminin sağ tarafı. NaN/∞ varsa NaN döndürür (`:161`–`:169`). |
| 12 | `Core/Simulation/OpenLoopSimulator.cs:172` → `:186` | `AppliedTorque(t, q̇)` | Her eklem için `Inputs[i].Value(t)` (`Core/Simulation/JointInput.cs:36`: sabit, basamak veya sinüs). Tork modunda `:196` ±τmax'a kırpar; gerilim modunda `:194` `motor.Torque` (`Core/Motor/DcMotor.cs:62`). |
| 13 | `Core/Simulation/OpenLoopSimulator.cs:173` → `Core/Dynamics/ArmDynamics.cs:99` | `Acceleration(q, q̇, τ)` | İleri dinamik. |
| 14 | `Core/Dynamics/ArmDynamics.cs:101` → `:32` → `Core/Dynamics/MassMatrixFormulas.cs:18` | `MassMatrix` → `Compute` | M(q): `:21` m11 (eş. 14), `:22` bağlaşım terimleri (eş. 16), `:23` eğim bloğu (eş. 17). |
| 15 | `Core/Dynamics/ArmDynamics.cs:102`–`:104` | `CoriolisVector` (`:42`), `GravityVector` (`:66`), `Friction` (`:86`) | c(q, q̇), g(q), τf(q̇). |
| 16 | `Core/Dynamics/ArmDynamics.cs:106`–`:109` | `Acceleration` | b = τ − c − g − τf; `Small4x4.SolveSymmetricPositiveDefinite(M, b)` (`Core/Numerics/Small4x4.cs:46`) M q̈ = b denklemini Cholesky ile çözer. q̈ döner. |
| 17 | `Core/Simulation/OpenLoopSimulator.cs:176`–`:177` | `Derivatives` | dx = [q̇; q̈] çözücüye döner. |
| 18 | `Core/Solvers/Rk4Solver.cs:19` | `Step` | x(t+h) = x + h/6 · (k1 + 2k2 + 2k3 + k4). |
| 19 | `Core/Simulation/OpenLoopSimulator.cs:128`–`:129` | `Step` | `stepCount++` (zaman = stepCount · h), yeni durum. |
| 20 | `Core/Simulation/OpenLoopSimulator.cs:130` → `:202` | `CheckFaults` | NaN/∞, eklem sınırı, takım zeminin altında mı? Öyleyse `StopReason` yazılır, simülasyon durur. |
| 21 | `Core/Simulation/OpenLoopSimulator.cs:131` → `:137` | `CurrentSample` | Örnek: zaman, q, q̇, τ, takım konumu (`ForwardKinematics.ToolPosition`, `:144`), kinetik ve potansiyel enerji (`:148`–`:149`). |
| 22 | `Core/Session/LabSession.cs:210` | `Advance` | Yeni örnekler geçmişe eklenir ve forma döner. |
| 23 | `Forms/MainForm.cs:138` → `Forms/PlotPanel.cs:90` | `PlotPanel.Append(fresh)` | 5 ms'den sık örnekler atlanır; kalanlar `time` (`:98`) ve seri listelerine eklenir. Değerler `Values` (`:117`) içinde dereceye çevrilir. ScottPlot çizgileri bu listelere bağlıdır, kopya yoktur. |
| 24 | `Forms/MainForm.cs:139` → `:145` | `ShowState` | |
| 25 | `Forms/MainForm.cs:147` → `Core/Session/LabSession.cs:215` | `CurrentJointPositions` | `ForwardKinematics.JointPositions` (`Core/Kinematics/ForwardKinematics.cs:31`) beş çizim noktasını verir: taban, omuz, dirsek, bilek, takım. |
| 26 | `Forms/MainForm.cs:149` → `Forms/ArmView2D.cs:41` | `ArmView2D.SetPose` | Noktalar q1 düzlemine izdüşürülür (r = x cos q1 + y sin q1), `Invalidate()` (`:51`) yeniden çizim ister. Windows sonra `OnPaint`'i (`:55`) çağırır. |
| 27 | `Forms/MainForm.cs:151` → `Forms/ArmView3D.cs:77` | `ArmView3D.SetPose` | Borular ve küreler yeni noktalara taşınır. |
| 28 | `Forms/MainForm.cs:152` → `Forms/PlotPanel.cs:106` | `RefreshVisible` | Yalnız açık olan grafik sekmesi eksenleri uydurup (`:111`) yeniden çizilir. |
| 29 | `Forms/MainForm.cs:153` | `ShowState` | Araç çubuğuna "t = 1,234 s" ve varsa durma nedeni yazılır. |
| 30 | `Forms/MainForm.cs:140`–`:141` | `timer_Tick` | Süre dolduysa veya hata olduysa zamanlayıcı durur. |

**Neden `while` ile değil zamanlayıcıyla?** Düğmeye basınca bir `while` döngüsü çalıştırılsaydı pencere mesaj kuyruğunu işleyemezdi: Duraklat düğmesi ve çizimler donardı. Zamanlayıcı her tıkta kısa bir iş yapıp denetimi pencereye geri verir.

---

## 2. Bir kontrolcü örneği: algılayıcı → kontrolcü → motor → tesis → ölçütler

Kontrol sekmesinde "Kapalı çevrim" seçilmiş olsun: hesaplanmış tork (CT), Ts = 2 ms, kodlayıcı, motor eyleyicisi, h = 0,1 ms, görev "Eklem uzayında beşinci derece".

### 2a. Kurulum (Reset sırasında, bir kez)

| # | Dosya:satır | Metot | Ne olur |
|---|---|---|---|
| 1 | `Core/Session/LabSession.cs:173` → `Core/Session/ClosedLoopBuilder.cs:16` | `ClosedLoopBuilder.Build` | Kapalı çevrim ayarlarının kopyasıyla çağrılır. |
| 2 | `Core/Session/ClosedLoopBuilder.cs:19` → `:72` | `CreateTask` | Beşinci derece görev: `new QuinticTask(q0, hedef, t0, T)`. |
| 3 | `Core/Session/ClosedLoopBuilder.cs:22`–`:26` | `Build` | `JointSensor` kurulur: tür, sayım, gürültü, tohum. |
| 4 | `Core/Session/ClosedLoopBuilder.cs:27` → `:55`, `:57` | `CreateController` | Kontrolcünün **kendi** modeli tahmini yükle kurulur: `new ArmDynamics(new Payload(EstimatedPayloadKg).ApplyTo(parametreler), g)`. Gerçek yük yalnız tesistedir. |
| 5 | `Core/Session/ClosedLoopBuilder.cs:31` | `Build` | `new ClosedLoopSimulator(plant, controller, task, sensor, actuator, Ts)`. |
| 6 | `Core/Session/LabSession.cs:176` → `Core/Simulation/ClosedLoopSimulator.cs:75` | `ClosedLoopSimulator.Reset` | Motor eyleyicisinde tesis gerilim moduna alınır; girişler sabit 0 olur; `:80` `Plant.FrameTimeS = Ts` (bir tesis `Step()` çağrısı artık Ts/h = 20 çözücü adımıdır); kontrolcü ve algılayıcı sıfırlanır. |

Zamanlayıcı tıkı bölüm 1b'deki gibidir; tek fark `Core/Session/LabSession.cs:204`'te `loop.Step()` çağrılmasıdır. 0,031 s'lik bir tıkta yaklaşık 15 kontrolcü örneği işlenir.

### 2b. Bir örnek (her Ts = 2 ms'de bir)

| # | Dosya:satır | Metot | Ne olur |
|---|---|---|---|
| 1 | `Core/Simulation/ClosedLoopSimulator.cs:89` | `ClosedLoopSimulator.Step` | |
| 2 | `Core/Simulation/ClosedLoopSimulator.cs:92`–`:93` | `Step` | Al-bırak açıksa ve zamanı geldiyse tesisin yükü değişir (`OpenLoopSimulator.ChangePayload`). Kontrolcünün modeli değişmez. |
| 3 | **Algılayıcı** `:94` → `Core/Simulation/JointSensor.cs:58` | `JointSensor.Measure` | Kodlayıcıda: `:70` gürültü eklenir, `:71` açı r = 2π/4096 adımına yuvarlanır (nicemleme), `:72` hız geri farkla bulunur: (q_m − q_m,önceki)/Ts. İlk örnekte hız 0. |
| 4 | **Referans** `:95` → `Core/Tasks/QuinticTask.cs:28` | `QuinticTask.At(t)` | q_ref, q̇_ref, q̈_ref: 10s³ − 15s⁴ + 6s⁵ polinomu ve türevleri. |
| 5 | **Kontrolcü** `:96` → `Core/Controllers/ComputedTorqueController.cs:44` | `ComputedTorqueController.Compute` | Ölçülen q, q̇ ve referansla çağrılır. |
| 6 | `Core/Controllers/ComputedTorqueController.cs:46` | `Compute` | Önceki örnekten beri geçen süre Δt (yarım örnek öngörüsü açıksa). |
| 7 | `Core/Controllers/ComputedTorqueController.cs:52` | `Compute` | İstenen ivme a = q̈_ref + Kd·ė + Kp·e. |
| 8 | `:55` → `:65` | `HalfSample.Predict` | Model, torkun tutulacağı aralığın ortasındaki tahmini durumda hesaplanır: q̄ = q + q̇·Δt/2 + a·Δt²/8. |
| 9 | `:56` → `Core/Dynamics/ArmDynamics.cs:116` | `InverseDynamics(q̄, q̄̇, a)` | τ = M̂ a + ĉ + ĝ + f̂ (kontrolcünün kendi modeliyle). |
| 10 | `:57` → `:75` | `HalfSample.Clamp` | Tork ±τmax = ±4,1 N·m'ye kırpılır. |
| 11 | **Motor** `Core/Simulation/ClosedLoopSimulator.cs:99` → `Core/Motor/DcMotor.cs:75` | `DcMotor.VoltageFor` | Ters motor modeli: tork komutu gerilime çevrilir, V = R·τ/(N·Kt) + N·Ke·q̇_ölçülen. İdeal tork eyleyicisinde bu adım atlanır, tork doğrudan kullanılır. |
| 12 | `Core/Simulation/ClosedLoopSimulator.cs:100` | `Step` | `Plant.Inputs[i].Amplitude = u`. Giriş sabit (Constant) sinyal olduğundan değer Ts boyunca tutulur: sıfırıncı dereceden tutucu (ZOH). |
| 13 | `Core/Simulation/ClosedLoopSimulator.cs:102` | `Step` | Tork komutu grafik ve ölçütler için saklanır. |
| 14 | **Tesis** `:104` → `Core/Simulation/OpenLoopSimulator.cs:112` | `OpenLoopSimulator.Step` | Ts/h = 20 çözücü adımı (bölüm 1b, satır 8–21). Her `Derivatives` çağrısında `AppliedTorque` gerilim modunda `DcMotor.Torque`'u (`Core/Motor/DcMotor.cs:62`) **gerçek** eklem hızıyla çağırır: `:64` gerilim ±12 V'a kırpılır, `:65` akım I = (V − N·Ke·q̇)/R, `:66` τ = N·Kt·I, `:67` ±τmax doyumu. Zıt EMK ve doyum böylece tesiste kalır. |
| 15 | `Core/Simulation/ClosedLoopSimulator.cs:106` → `:118` | `Annotate` | 20 örneğin her birine q_ref, q̇_ref ve tork komutu eklenir. |
| 16 | `Core/Session/LabSession.cs:210` | `Advance` | Örnekler geçmişe eklenir; grafikler bölüm 1b'deki gibi çizilir. Kesikli çizgiler referans ve komuttur. Hata sekmesi takım hatasını `ControlMetrics.ToolErrorMm` (`Core/Session/ControlMetrics.cs:73`) ile hesaplar (`Forms/PlotPanel.cs:138`). |

### 2c. Ölçütler (Karşılaştır sekmesi)

| # | Dosya:satır | Metot | Ne olur |
|---|---|---|---|
| 1 | `Forms/ComparePanel.cs:97` | `btnRun_Click` | İşaretli kontrolcüler listelenir; `PrepareSession` formdaki değerleri oturuma yazar. |
| 2 | `Forms/ComparePanel.cs:111` → `Core/Session/LabSession.cs:232` → `Core/Session/ControllerComparison.cs:51` | `RunOne` | Her kontrolcü için oturumun kopyası kurulur ve koşu baştan sona işletilir (`:64` `s.Advance` döngüsü, en yüksek hızda). |
| 3 | `Core/Session/ControllerComparison.cs:67` → `Core/Session/ControlMetrics.cs:61` | `ControlMetrics.Compute` | Bütün örneklerden: `:66` → `:81` `ErrorMeasures` (RMS, en büyük ve son hata, takım hatası, oturma süresi), `:67` → `:108` `TorqueMeasures` (en büyük tork, doyum süresi, ∫τ² dt, titreşim), `:68` → `:132` `Overshoot` (yüzde aşma). |
| 4 | `Forms/ComparePanel.cs:117` | `ShowResults` | Takım hatası eğrileri üst üste çizilir, tablo doldurulur; "Tabloyu CSV kaydet" `ComparisonCsv.Write`'ı çağırır. |
