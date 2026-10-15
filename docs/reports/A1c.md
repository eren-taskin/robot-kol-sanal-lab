# Phase A1c Report — Plain CT Default Gains (`RobotArm.Core`)

## 1. What Was Changed

- **`src/RobotArm.Core/Control/ComputedTorque.cs`**:
  - Retuned `ComputedTorque.DefaultGains` from `Kp = [144.0, 144.0, 144.0, 144.0] 1/s²`, `Kd = [24.0, 24.0, 24.0, 24.0] 1/s` to `Kp = [1600.0, 1600.0, 1800.0, 1800.0] 1/s²`, `Kd = [350.0, 350.0, 350.0, 350.0] 1/s`.
- **`tests/RobotArm.Tests/LoopFrequencyResponseTests.cs`**:
  - Updated `DefaultGains_MarginsAtLibraryStartPoses_DoNotRegress` so that both `"CT"` and `"CT+I"` are checked against `GainMarginDb >= 6.0` dB and `PhaseMarginDeg >= 40.0`° on every joint across all five planets, payloads 0 and 0.5 kg, and all library scenario start poses. `PID` and `PDG` keep their existing regression floors.
- **`tests/RobotArm.Tests/ControllerTests.cs`**:
  - Updated `ComputedTorque_ComputesInverseDynamicsOfTheCommandedAccelerationPlusFriction` to construct `ComputedTorque` with explicit unclamped gains (`Kp = [144, 144, 144, 144]`, `Kd = [24, 24, 24, 24]`) so all four joints remain below $\pm 4.1\text{ N}\cdot\text{m}$ when testing the unclamped inverse-dynamics formula on the large synthetic test state.
  - Updated `Spec7_DefaultGains_SettleWithin1s_WithoutSaturation_OnEarthWithoutPayload` so that `PID` and `PDG` assert `!r.Clamped` while all three controllers (`PID`, `PDG`, `CT`) assert `!r.PlantSaturated` and `r.SettlingTime <= 1.0`.
- **`SPEC.md`**:
  - Updated §7 default gains table, step-response settling table, peak $|u|$ note, sampled-loop stability table (`ρ` at $T_s = 2\text{ ms}$ and `MaxStableTs`), §11 `V39` description, and §12.5 linear step response percentages.
- **`docs/DECISIONS.md`**:
  - Added `D11 – Plain CT default gains retuned for loop-at-a-time margins (A1c)` with the $P_{ii} K_{d,i}$ vs. $d_i$ analysis, worst-case poses/planets/payloads for plain CT and PDG, and the before/after worst-case margin table across all 4 controllers × 4 joints.
- **`docs/results/robustness.csv`**:
  - Regenerated the 160-run informed robustness grid with the retuned plain CT defaults.
- **`PROJECT.md`**:
  - Added row `A1c` in §7 with status `done`.

---

## 2. Measured Numbers & Analysis

### 2.1 Item 1 — Why Old Plain CT Defaults (`Kp = 144`, `Kd = 24`) Had 0.34 dB GM and 10.3° PM

The three distinct start poses across `ScenarioLibrary.All` are:
- **Pose 0 (`"Dünya – boş kol – noktaya git"`)**: $q_0 = [0, -0.4, -0.3, 1.1]\text{ rad}$
- **Pose 1 (`"Yüksek g – 0,5 kg – tutma (doyum)"`, `horizontal_reach_pose`)**: $q_0 = [0, 1.3438881150221629, -1.3002383005116098, -0.04364981451055305]\text{ rad}$
- **Pose 2 (`"Örnekleme – Ts sınırına yakın"`, Pose B)**: $q_0 = [0, -0.1, 0.2, 0.2]\text{ rad}$

With `Kp = [144, 144, 144, 144] 1/s²` and `Kd = [24, 24, 24, 24] 1/s`:
- **Wrist (Joint 4) worst gain margin = `0.3417 dB` (`0.34 dB`)** occurs at **Orbit ($g = 0$), 0 kg payload, Pose 2 (`"Örnekleme – Ts sınırına yakın"`, $q_0 = [0, -0.1, 0.2, 0.2]$ rad)**, with $\text{PM} = 15.9101^\circ$, $\omega_{pc} = 10.62\text{ rad/s}$, $\omega_{gc} = 0.84\text{ rad/s}$, and $\rho = 0.981827$ (and $0.3417\text{–}0.3419\text{ dB}$ across all five planets at 0 kg, Pose 2).
- **Wrist (Joint 4) worst phase margin = `11.3592°` (`11.36°`)** (with $\text{GM} = 0.3482\text{ dB}$, $\omega_{pc} = 10.64\text{ rad/s}$, $\omega_{gc} = 0.88\text{ rad/s}$, $\rho = 0.978842$) occurs at **HighG ($g = 24.79\text{ m/s}^2$), 0 kg payload, Pose 1 (`"Yüksek g – 0,5 kg – tutma (doyum)"`)**.
- **Overall worst phase margin across all joints = `10.3138°` (`10.3°`)** occurs on **Joint 3 at HighG ($g = 24.79\text{ m/s}^2$), 0.5 kg payload, Pose 0 (`"Dünya – boş kol – noktaya git"`, $q_0 = [0, -0.4, -0.3, 1.1]$ rad)**, with $\text{GM} = 1.8696\text{ dB}$, $\omega_{pc} = 12.14\text{ rad/s}$, $\omega_{gc} = 3.60\text{ rad/s}$, $\rho = 0.982309$. (On Joint 3 at 0 kg, Pose 2, $\text{GM} = 0.8007\text{ dB}$ and $\text{PM} = 14.3712^\circ$.)
- **$P_{ii} K_{d,i}$ versus friction slope $d_i$**:
  Linearised at rest (`SPEC.md` §7, §12.2), CT's velocity feedback block is $K_{\text{vel}} = P\operatorname{diag}(K_d) - \hat{G}\frac{T_s}{2} - \hat{D}$, where $P = \hat{M}(q_0) + \hat{G}\frac{T_s^2}{8} + \hat{D}\frac{T_s}{2}$ and $\hat{D} = \operatorname{diag}(d_i)$ comes from cancelling the smoothed friction at rest ($d_i = f_v + f_c/\omega_s = 0.01 + 0.02/0.1 = 0.21\text{ N}\cdot\text{m}\cdot\text{s/rad}$).
  On the unloaded wrist (Joint 4, 0 kg), $\hat{M}_{44} = 3.6747 \times 10^{-4}\text{ kg}\cdot\text{m}^2$ and $P_{44} \approx 5.7747 \times 10^{-4}\text{ kg}\cdot\text{m}^2$. With $K_{d,4} = 24\text{ s}^{-1}$, $P_{44} K_{d,4} = 0.01386\text{ N}\cdot\text{m}\cdot\text{s/rad} \ll d_4 = 0.21\text{ N}\cdot\text{m}\cdot\text{s/rad}$, making the net diagonal velocity feedback negative:
  $$K_{\text{vel},44} = P_{44} K_{d,4} - \hat{G}_{44}\frac{T_s}{2} - d_4 \approx -0.19614\text{ N}\cdot\text{m}\cdot\text{s/rad} < 0.$$
  Likewise on Joint 3 at 0 kg, Pose 2, $\hat{M}_{33} = 4.9041 \times 10^{-3}\text{ kg}\cdot\text{m}^2$, $P_{33} = 5.1141 \times 10^{-3}\text{ kg}\cdot\text{m}^2$, and $P_{33} K_{d,3} = 0.12274 < d_3 = 0.21\text{ N}\cdot\text{m}\cdot\text{s/rad}$ ($K_{\text{vel},33} = -0.08726 < 0$). When `LoopFrequencyResponse` scales row $i$ of $K_\xi$ by a factor $k > 1$, the negative velocity feedback over-cancels the plant's passive damping $d_i$ and destabilises the wrist at $k = 1.0401$ ($0.3417\text{ dB}$).
  Across the 30 cases, the minimum $P_{ii}$ per joint is $[2.685 \times 10^{-3}, 1.206 \times 10^{-2}, 5.114 \times 10^{-3}, 5.775 \times 10^{-4}]\text{ kg}\cdot\text{m}^2$, so $P_{44} K_{d,4} \approx d_4$ requires $K_{d,4} \approx 350\text{ s}^{-1}$, and strong inertial coupling $M_{ir}(M^{-1})_{ri}$ at `horizontal_reach_pose` with 0.5 kg requires $K_d$ on joints 2–3 to match joint 4 at $350\text{ s}^{-1}$.

### 2.2 Item 3 — PDG 2.69 dB Worst Case on Joint 3

- **PDG worst gain margin on Joint 3 = `2.6869 dB` (`2.69 dB`)** occurs at **Orbit ($g = 0$), 0.5 kg payload, Pose 1 (`"Yüksek g – 0,5 kg – tutma (doyum)"`, `horizontal_reach_pose`, $q_0 = [0, 1.3439, -1.3002, -0.0436]$ rad)**, with $\text{PM} = 118.6473^\circ$, $\omega_{pc} = 1570.80\text{ rad/s}$ ($\pi/T_s$), $\omega_{gc} = 13.51\text{ rad/s}$, and $\rho = 0.995192$ (and $2.6869\text{–}2.7289\text{ dB}$ across all five planets at 0.5 kg, Pose 1). PDG gains were not changed.

### 2.3 Item 5 — Before/After Worst-Case Margin Table (Controller × Joint)

Worst-case values across all 5 planets × payloads $\{0, 0.5\}\text{ kg}$ × 3 library start poses (30 cases):

| Controller | max ρ | Joint 1 GM / PM | Joint 2 GM / PM | Joint 3 GM / PM | Joint 4 GM / PM |
|---|---|---|---|---|---|
| PID (unchanged) | 0.999320 | 18.34 dB / 31.73° | −23.70 dB (\|GM\| 1.07 dB) / 10.91° | −4.04 dB (\|GM\| 2.69 dB) / 60.92° | 11.26 dB / 38.90° |
| PDG (unchanged) | 0.995192 | 18.34 dB / 37.12° | 6.08 dB / 65.13° | 2.69 dB / 82.20° | 11.26 dB / 82.54° |
| CT (before A1c: `144 / 24`) | 0.982309 | 3.16 dB / 44.28° | 4.16 dB / 34.74° | 0.80 dB / 10.31° | 0.34 dB / 11.36° |
| CT (after A1c: `1600,1600,1800,1800 / 350`) | 0.990779 | 9.15 dB / 69.24° | 9.36 dB / 71.83° | 10.75 dB / 89.68° | 7.19 dB / 62.51° |
| CT+I (unchanged: `3600 / 360 / 10000`) | 0.989816 | 8.91 dB / 67.81° | 9.10 dB / 70.07° | 10.43 dB / 87.69° | 7.43 dB / 46.57° |

### 2.4 Item 2 — Old vs. New Values of Every Number in `SPEC.md` §7, §11, and §12.5 That Changed

| Location / Metric | Old Value (`Kp=144, Kd=24`) | New Value (`Kp=[1600,1600,1800,1800], Kd=350`) |
|---|---|---|
| §7 Default CT `Kp` (Joints 1–4, $\text{s}^{-2}$) | `144, 144, 144, 144` ($\omega_n = 12\text{ rad/s}$) | `1600, 1600, 1800, 1800` ($\omega_n \approx 40\text{–}42.4\text{ rad/s}$) |
| §7 Default CT `Kd` (Joints 1–4, $\text{s}^{-1}$) | `24, 24, 24, 24` ($\zeta = 1$) | `350, 350, 350, 350` ($\zeta \approx 4.1\text{–}4.4$) |
| §7 Step settling Earth 0 kg (`A→B / B→A`, s) | `0.49 / 0.49` | `0.87 C / 0.85 C` (`0.866 s / 0.854 s`) |
| §7 Step settling Earth 0.5 kg (`A→B / B→A`, s) | `0.58 C / F limit j4` | `0.96 C / 1.07 C` |
| §7 Step settling Moon 0 kg (`A→B / B→A`, s) | `0.49 / 0.49` | `0.87 C / 0.85 C` |
| §7 Step settling Moon 0.5 kg (`A→B / B→A`, s) | `0.64 C / 0.63 C` | `0.96 C / 0.99 C` |
| §7 Step settling Mars 0 kg (`A→B / B→A`, s) | `0.49 / 0.49` | `0.87 C / 0.85 C` |
| §7 Step settling Mars 0.5 kg (`A→B / B→A`, s) | `0.62 C / 0.69 C` | `0.96 C / 1.00 C` |
| §7 Step settling HighG 0 kg (`A→B / B→A`, s) | `0.49 / 0.49` | `0.87 C / 0.86 C` |
| §7 Step settling HighG 0.5 kg (`A→B / B→A`, s) | `F limit j4 / F limit j4` | `F limit j3 / F limit j4` |
| §7 Step settling Orbit 0 kg (`A→B / B→A`, s) | `0.49 / 0.49` | `0.87 C / 0.85 C` |
| §7 Step settling Orbit 0.5 kg (`A→B / B→A`, s) | `0.65 C / 0.57 C` | `0.96 C / 0.98 C` |
| §7 Peak $\|u\|$ on Earth 0 kg step test | `1.92 N·m` (unclamped) | `4.10 N·m` (clamped at $t=0$ on step; `1.31 N·m` on smooth quintics) |
| §7 Sampled-loop $\rho$ at $T_s = 2\text{ ms}$ (poses A, B, C) | `0.9785–0.9818` | `0.9908–0.9908` |
| §7 Sampled-loop `MaxStableTs` (poses A, B, C) | `80.7–86.2 ms` | `5.7–5.7 ms` (`5.716 ms` at Pose B, Earth, 0 kg) |
| §11 V18 (mid-hold, `predict=true`) `(a)` exact 0/0 max error | `4.29e-5 rad` | `1.73e-5 rad` (`1.727e-5 rad`, $< 10^{-3}\text{ rad}$) |
| §11 V18 (mid-hold, `predict=true`) `(b)` 0.3/0 max error | `1.12e+0 rad` (`26093×`) | `5.60e-1 rad` (`0.5598 rad`, `32407×`, fault-free) |
| §11 V18 (mid-hold, `predict=true`) `(c)` 0/0.3 max error | `5.73e-1 rad` (`13358×`) | `1.42e+0 rad` (`1.423 rad`, `82371×`, fault-free) |
| §11 V18 (textbook, `predict=false`) `(a)` / `(b)` / `(c)` | `3.4e-3` / `1.13` / `0.67 rad` | `2.71e-4` / `6.29e-1` (`2323×`) / `1.34e+0 rad` (`4940×`) |
| §11 V24 Encoder step final error (CT, $\tau = 5\text{ ms}$ / $\tau = 0$) | `0.10 counts` / `15 counts` | `0.17 counts` ($\le 2$) / `2.21 counts` ($> 2$) |
| §11 V29 (`V18(b)` 1.5 s) ordinary CT final / max joint error | `0.7287002454` / `1.119293392 rad` | `0.5542987672` / `0.559830798 rad` |
| §11 V39 Worst-case CT GM / PM across 30 cases | `0.34 dB / 10.31°` | `7.19 dB / 62.51°` ($\ge 6\text{ dB}$, $\ge 40^\circ$ asserted) |
| §11 V40 / §12.5 Linear step diff on CT (0.01 rad / 0.5 rad) | `0.30 % / 1.87 %` | `0.42 %` (`0.418 %`) / `5.39 %` (`5.394 %`) |

---

## 3. Anything Not Done

None.

---

## 4. Verbatim Command Outputs

### Last 5 lines of `dotnet build`

```text
Oluşturma başarılı oldu.
    0 Uyarı
    0 Hata

Geçen Süre 00:00:13.49
```

### Last 5 lines of `dotnet test --filter "Category!=Report"`

```text
  RobotArm.Tests -> /Users/erentaskin/Documents/GitHub/robot-kol-sanal-lab/tests/RobotArm.Tests/bin/Debug/net10.0/RobotArm.Tests.dll
/Users/erentaskin/Documents/GitHub/robot-kol-sanal-lab/tests/RobotArm.Tests/bin/Debug/net10.0/RobotArm.Tests.dll (.NETCoreApp,Version=v10.0) için test çalıştırması
Toplam 1 test dosyası belirtilen desenle eşleşti.

Başarılı!  - Başarısız:     0, Başarılı:   382, Atlanan:     0, Toplam:   382, Süre: 34 s - RobotArm.Tests.dll (net10.0)
```

### Output of `git status -s`

```text
 M PROJECT.md
 M SPEC.md
 M docs/DECISIONS.md
 M docs/results/robustness.csv
 M src/RobotArm.Core/Control/ComputedTorque.cs
 M tests/RobotArm.Tests/ControllerTests.cs
 M tests/RobotArm.Tests/LoopFrequencyResponseTests.cs
?? docs/reports/
```
