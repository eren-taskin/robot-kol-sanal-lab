# TASKS — prompts for the AI tools

Her görevin istemi aşağıda. İstemi olduğu gibi kopyalayıp ilgili araçta, depo klasöründe açılmış **yeni** bir oturuma yapıştırın. İstemler İngilizce, çünkü SPEC ve kod İngilizce. Araçlar Türkçe de anlar, ama karışıklık olmasın diye tek dil kullanıyoruz.

Each prompt is self-contained. Paste it into a **new** session opened in the repository folder.

---

## T01 — Solution skeleton and CI  ·  Claude Code

```
Read PROJECT.md and SPEC.md completely before doing anything. Do only task T01.

Goal: create the empty solution exactly as described in PROJECT.md §2, so that later tasks only add code.

Do:
1. global.json pinning SDK 10.0.300 with rollForward latestFeature. If that exact SDK is not installed, tell me which one is, and pin to the installed 10.0.x feature band instead.
2. RobotArm.sln with src/RobotArm.Core (classlib, net10.0), src/RobotArm.Web (Blazor WebAssembly standalone, net10.0), tests/RobotArm.Tests (xunit, net10.0).
3. References: Web -> Core, Tests -> Core. Nothing else. Core has no PackageReference.
4. Core: Nullable enable, TreatWarningsAsErrors true, GenerateDocumentationFile true.
5. Create the empty folders of PROJECT.md §2 inside Core, each with a placeholder file only if the build needs it.
6. Tests: copy-to-output of ../../reference/test_vectors.json, and a helper class TestVectors that loads it with System.Text.Json into typed records. Add one test that loads the file and asserts there are 16 poses.
7. One test that proves the dependency rule: load the RobotArm.Core assembly and assert it references no assembly whose name starts with "RobotArm." other than itself.
8. .gitignore for .NET (bin, obj, .vs, *.user), README.md with the run commands of PROJECT.md §4.
9. .github/workflows/ci.yml: on push and pull request, setup-dotnet using global.json, dotnet build, dotnet test.
10. docs/DECISIONS.md with the first entry: "D1 – IPlant takes a vector input" explaining PROJECT.md §2 rule 2.

Do not write any physics yet.

Done when: dotnet build and dotnet test pass with 0 warnings, and `dotnet run --project src/RobotArm.Web` serves the template page.

Finish with a summary: files created, commands run and their output (last lines), anything that differs from PROJECT.md.
```

---

## T02 — Parameters, chain, planets, payload  ·  Claude Code

```
Read PROJECT.md and SPEC.md completely. Do only task T02. Work in src/RobotArm.Core/Model and tests/RobotArm.Tests.

Goal: one source of truth for every parameter of SPEC §3 and the chain of SPEC §4.1, plus payload combination (SPEC §5.5).

Do:
1. BodyParameters (mass, Ell, Eps, Ixx, Iyy, Izz, Ixz) as an immutable type with validation (mass > 0, inertia tensor positive definite).
2. JointDefinition (origin in parent frame as a 3-vector, axis Z or Y, lower and upper limit).
3. ArmModel: list of joints, list of bodies, tool point in last frame, actuator parameters (SPEC §3.3). A static factory ArmModel.OpenManipulatorX() builds the arm of SPEC §3–§4.1. L2 and beta are computed from 0.024 and 0.128, not typed.
4. Body inertia and centre of mass expressed as 3-vectors and 3x3 matrices. Write a tiny Vector3d / Matrix3d struct pair yourself (no package). Readonly structs, no allocation.
5. Joint limits: download is not possible, so I will paste the limits from open_manipulator_x.urdf if you ask. Until then use SPEC §3.4 defaults and leave a TODO with the SPEC reference.
6. Planet as a record (Key, DisplayName, G) with the five entries of SPEC §3.5 and a factory for custom g with validation 0..30.
7. ArmModel.WithPayload(double massKg) returning a new ArmModel whose last body is the combined body of SPEC §5.5. Massless payload returns an equal model.
8. Tests: V1 and V14 of SPEC §11. V14 must compare against hand-computed values for one case AND against the 0.5 kg poses in test_vectors.json once T04 exists; for now test mass, centre of mass and Iyy by hand.

Do not write kinematics or dynamics.

Done when: build and all tests pass.
Finish with a summary and list every number you typed into code with its SPEC row.
```

---

## T03 — Kinematics  ·  Claude Code

```
Read PROJECT.md and SPEC.md completely. Do only task T03. Work in src/RobotArm.Core/Kinematics and tests.

Goal: SPEC §4.2–§4.4.

Do:
1. ForwardKinematics: (a) closed form of SPEC §4.2 returning tool position and pitch; (b) general frame chain over ArmModel joints (SPEC §4.1) returning every frame's rotation and origin, plus tool position. (b) is what the drawing and the dynamics will use. Both must agree.
2. InverseKinematics.Solve(position, pitch, branch) returning a result type with Success, Q[4], and a FailureReason (Unreachable, JointLimit). Never NaN, never throws for an unreachable target.
3. PlanarJacobian(q) and its determinant per SPEC §4.4, and a method that returns false near singularity (threshold from SPEC).
4. Tests: V2, V3, V4 of SPEC §11, using test_vectors.json. Add random round-trip tests (1000 seeded random poses inside the limits, both branches, IK(FK(q)) == q to 1e-9 when the branch matches).

Done when: build and all tests pass.
Finish with a summary.
```

---

## T04 — Dynamics  ·  Claude Code

```
Read PROJECT.md and SPEC.md completely. Do only task T04. Work in src/RobotArm.Core/Dynamics and tests.

Goal: SPEC §5.1–§5.3 with the general algorithm required by §5.2.

Do:
1. RecursiveNewtonEuler over an ArmModel: Inverse(q, qd, qdd, gravityVector, tauOut). No allocation per call; buffers sized from the model in the constructor.
2. InertiaMatrix(q, Mout) by columns of RNEA with zero gravity (or CRBA, your choice; write which in docs/DECISIONS.md). Add N^2*Jm to the diagonal.
3. BiasTorque(q, qd, g, hOut) = RNEA(q, qd, 0).
4. A Cholesky solver for symmetric positive definite matrices, in place, no allocation; throws a clear exception if the matrix is not positive definite.
5. In the TEST project (not Core): ClosedFormArm implementing SPEC §5.3 exactly (m11, pitch block, gravity, Christoffel C with dM/dq by central differences). It is the independent check.
6. Tests: V5, V6, V7, V8 of SPEC §11, plus the 0.5 kg payload poses from test_vectors.json (completes V14).
7. Document in XML comments which SPEC section each method implements.

Do not write the solver or the simulation loop.

Done when: build and all tests pass. Report the largest relative difference you measured between the general algorithm and the closed form.
```

---

## T05 — Plant, solvers, simulation  ·  Codex

```
Read PROJECT.md and SPEC.md completely. Do only task T05. Work in src/RobotArm.Core/Simulation and tests.

Goal: SPEC §5.4, §6.1, §6.2, §6.4 on top of the existing Model, Kinematics and Dynamics code. Do not modify Dynamics except to fix a bug, and report it if you do.

Do:
1. IPlant and ISolver exactly as in PROJECT.md §2.
2. ArmPlant : IPlant. State 8, input 4. Holds ArmModel, Planet, current payload model, friction parameters. Derivatives applies saturation and friction (SPEC §5.4), then solves M qdd = tau - tauf - h by Cholesky. Exposes the saturation flags of the last evaluation. No allocation.
3. EulerSolver and Rk4Solver with preallocated buffers (the course version allocates; ours must not, see Lecture Note 2 caution).
4. Simulation: owns time and a copy of the state, Advance(u, h), fault detection of SPEC §6.4 (joint limit, floor, NaN). Payload attach/detach at a step boundary (SPEC §5.5) with parameters switched and state unchanged.
5. Tests: V9, V10, V11, V12, V13, V15 of SPEC §11. V9–V12 use friction off and Jm = 0.
6. Look up in the DYNAMIXEL XM430-W350 e-manual whether encoder resolution 4096 and no-load speed 46 rpm at 12 V are right, if you can access the web. Otherwise say you could not.

Done when: build and all tests pass. Report the measured energy drift (V9), p1 drift (V10) and error ratios (V11).
```

---

## T06 — Independent review  ·  Antigravity (Gemini)

```
Read PROJECT.md and SPEC.md completely. Do only task T06. This is a REVIEW. Do not change any code.

Goal: an independent second reading of everything written in tasks T02–T05.

Do:
1. Read every file in src/RobotArm.Core and tests/RobotArm.Tests.
2. For each SPEC section §3 to §6, check the code line by line against the equations: signs, indices (zero-based arrays versus 1-based joints), units, the pitch sign convention, the order of rotations, the Christoffel indices.
3. Check the PROJECT.md rules: dependency rule, no allocation in per-step code, InvariantCulture, sealed classes, cloned arrays.
4. Check that each test actually tests what its name says, and that no tolerance is so loose that a sign error would pass. For each of V5, V7, V9 say what kind of bug it would catch and what it would miss.
5. Write docs/reviews/T06-review.md with: a table of findings (file, line, severity: bug / risk / style, explanation, suggested fix), and a short list of the 3 things you are least sure are correct.

Done when: the review file exists. Do not commit code changes.
```

---

## T07 — Controllers and sampled loop  ·  Codex

```
Read PROJECT.md and SPEC.md completely, and docs/reviews/T06-review.md if it exists. Do only task T07. Work in src/RobotArm.Core/Control and tests.

Goal: SPEC §6.3 and §7.

Do:
1. IController: Reset(), Compute(t, qMeasured, qdMeasured, reference, uOut). Reference holds qd, qdDot, qdDDot. No allocation in Compute.
2. Pid (per-joint gains, anti-windup driven by the plant's saturation flags passed in through the loop), PdGravity, ComputedTorque. Model-based controllers take their own ArmModel and Planet (the "belief", SPEC §5.6).
3. Sensors: PerfectSensor and EncoderSensor (quantisation 2*pi/4096, backward-difference speed, optional first-order filter, optional seeded Gaussian noise).
4. ControlLoop: runs the plant with step h and the controller every Ts (integer multiple of h, validated), holding u between samples.
5. Tune default gains so that SPEC §7's criterion holds; write the gains into SPEC §7 (this task is allowed to edit SPEC §7 only).
6. Tests: V16, V17, V18 of SPEC §11, plus a test that Ts not a multiple of h throws.

Done when: build and all tests pass. Report the gains and the measured settling times.
```

---

## T08 — Trajectories, tasks, scenario, metrics, logging  ·  Codex

```
Read PROJECT.md and SPEC.md completely. Do only task T08. Work in src/RobotArm.Core/Tasks, src/RobotArm.Core/Logging and tests.

Goal: SPEC §8, §9, §10.

Do:
1. QuinticJointTrajectory and CartesianLineTrajectory (IK per sample, clean failure if unreachable).
2. Tasks Reach, PickPlace, Track (circle and line), Hold (disturbance pulse), each producing references and evaluating its success criterion.
3. Scenario as a record serialisable to JSON with System.Text.Json (environment, payload, task, controller and gains, controller belief, h, Ts, sensor options, seed, duration).
4. ExperimentRunner: Run(scenario) -> RunResult (log rows, metrics, success, fault). Also a stepwise API (Start, AdvanceBy(simSeconds), Current) that the web page will call once per animation frame.
5. Metrics of SPEC §9 computed from the log.
6. CsvLogWriter writing to a TextWriter with InvariantCulture (SPEC §10).
7. Tests: one success test per task on Earth with CT, a PickPlace test whose metrics show the payload jump, V19, and a CSV round-trip test.
8. Provide 6 ready scenarios in a static ScenarioLibrary, in Turkish display names: "Dünya – boş kol – nokta", "Ay – 0,3 kg – al-bırak", "Mars – çember izleme", "Yüksek g – 0,5 kg – tutma (doyum)", "Yörünge – nokta", "Yanlış model – CT 0,3 kg hata".

Done when: build and all tests pass. Report the metrics of the 6 library scenarios in a table.
```

---

## T09 — Web: layout, run loop, arm drawing, design helper  ·  Antigravity (Gemini)

```
Read PROJECT.md, SPEC.md and docs/DECISIONS.md completely. Do only task T09. Work in src/RobotArm.Web only. Do not change Core; if Core lacks something, stop and report it.

Goal: the first usable page of the online laboratory. UI language: Turkish. Numbers shown with a decimal comma are fine on screen, but every number parsed or exported uses InvariantCulture.

Do:
1. Layout: left panel for the experiment setup, centre for the arm view, right or bottom for live numbers. Usable on a laptop and on a tablet.
   Setup panel: ScenarioLibrary dropdown (loads all fields), planet (or custom g), payload kg, task and its targets, controller type (PID, PDG, CT, CT+I), the CT "örneklemeye duyarlı / ders kitabı" option, belief (informed / unaware, and a belief payload override for model-mismatch runs), gains per joint, h, Ts, sensor (perfect / encoder, filter time constant, noise), seed, duration.
   Do not offer the elbow-down IK branch in the UI (it is almost entirely outside the joint limits, see T03).
2. Arm view: two SVG views drawn from Core's ForwardKinematics frames: side view (r–z plane) and top view (x–y). Show target, object, floor, payload when attached, and the joint-limit state (a joint near its limit turns amber). Scales in mm.
3. Run loop per PROJECT.md §2 rule 4: Start / Pause / Step / Reset. Each animation frame advances simulated time by a fixed slice through ExperimentRunner's stepwise API, then redraws. Show simulated time and real-time factor. A "hızlı" mode runs the whole scenario first and then plays it back.
4. Stability panel, always visible, recomputed whenever a relevant field changes:
   - RK4 step check: StabilityEstimate.MaxStableStep at the start pose; warning when h exceeds it.
   - Sampled loop: SampledLoopStability ρ at the chosen Ts and MaxStableTs at the start pose; red when ρ ≥ 1.
   - Torque budget: gravity torque at the start pose versus τmax per joint; warning when above 80 %.
   Write each warning as one plain Turkish sentence that says what will happen and what to change.
5. Design helper (Tasarım yardımcısı): per joint, the student enters ωn (rad/s) and ζ. The page computes gains and fills the gain fields:
   - CT / CT+I: Kp = ωn², Kd = 2ζωn (and Ki = ωn³/10 for CT+I as a starting value).
   - PD / PDG / PID: Kp = M̂ii ωn², Kd = 2ζωn M̂ii, with M̂ii the diagonal of the belief's inertia matrix at the start pose (state this approximation on screen).
   The stability panel updates immediately, so the student sees the effect before running.
6. Input validation in the UI (ranges from SPEC), plus Core's own validation; show Core's error messages in Turkish next to the field.
7. A fault (SPEC §6.4) stops the run and shows the reason.

No charting yet. No new packages without asking.

Done when: dotnet build passes, dotnet test still passes, and every library scenario runs in the page. Describe how you tested it in the browser. Set T09 to "done" in PROJECT.md §7, and add a row "T10b | Kendi kontrolcün (student-written controller) | — | not started" between T11 and T12.
```

---

## T10 — Web: plots, metrics, comparison, export  ·  Antigravity (Gemini)

```
Read PROJECT.md, SPEC.md and docs/DECISIONS.md completely. Do only task T10. Work in src/RobotArm.Web only.

Goal: the analysis half of the laboratory.

Do:
1. Plots against time: joint angles with references, joint errors, torques with the ±4.1 N·m limits drawn and clamped intervals shaded, tool position error. Mark payload attach/release events with a vertical line. Either a hand-written SVG/canvas plot component or ONE charting library; if a library, name it, its version and why, before installing.
2. Metrics panel (SPEC §9) with success / failure and reason.
3. Run comparison: keep up to 4 finished runs in memory, overlay a chosen signal from each, and show their metrics side by side.
4. Export: the run's CSV log and scenario JSON as downloads. Import: load a scenario JSON to repeat a run exactly.
5. Show each library scenario's Turkish "what it shows" text, and add a short "Ne gözlemlemelisin?" hint (2–3 sentences) under the plots for each.
6. Robustness map page: load docs/results/robustness.csv (copy it into wwwroot at build time) and show it as a heat map: rows controller, columns planet × payload, colour by success and settling time.

Done when: build and tests pass and the comparison works for "Yanlış model – CT" against "Yanlış model düzeltildi – CT+I". Set T10 to "done" in PROJECT.md §7.
```

---

## T11 — Publish to GitHub Pages  ·  Claude Code

```
Read PROJECT.md completely. Do only task T11.

Goal: the laboratory is reachable at https://<github-user>.github.io/<repo>/ from any browser.

Do:
1. A workflow that on push to main runs the tests, publishes RobotArm.Web in Release, fixes the base href for the repository sub-path, adds .nojekyll and a 404.html fallback, and deploys to GitHub Pages.
2. Tell me exactly which repository settings I must change by hand (Settings → Pages → Source). Note that GitHub Pages on a private repository needs a paid plan or GitHub Education benefits; say what the options are.

Done when: the workflow file exists and you have explained the manual steps. Set T11 to "done" in PROJECT.md §7.
```

---

## T10b — "Kendi kontrolcün" (student-written controller)  ·  any tool

Run only after T11, so a working laboratory is already published.

```
Read PROJECT.md, SPEC.md and docs/DECISIONS.md completely. Do only task T10b. Core may get one small addition (item 3); everything else is in src/RobotArm.Web.

Goal: a student can write their own control law in the browser and compare it with the built-in controllers using the same metrics.

Do:
1. A "Kendi kontrolcün" controller type. Selecting it shows a code editor (a plain textarea with a monospace font is acceptable; a small editor library only if you name it first) prefilled with a working PD + gravity example:
     function kontrol(t, q, qd, ref, model) { ... return u; }
   ref has q, qd, qdd (arrays of 4). model has M(q) (4x4), gravity(q) (4), bias(q, qd) (4), all computed by Core from the controller's BELIEF, never from the true plant.
2. A JsController in Web implementing Core's IController. Each Ts it calls the student's function through synchronous JS interop (IJSInProcessRuntime) and copies the result into u. model.* calls go back to .NET synchronously.
3. Safety:
   a. Before every run, a dry run in a Web Worker: 2000 calls with representative inputs; if it exceeds 500 ms, throws, or returns anything other than 4 finite numbers, refuse to start and show why in Turkish. This catches infinite loops before they can freeze the page.
   b. During the run: non-finite output stops the run with a fault "Kontrolcün sayı olmayan bir değer döndürdü"; outputs are clamped to ±τmax by the existing rules and shown as clamping; a call slower than 5 ms is reported.
   c. The student's code runs only in their own browser; nothing is sent anywhere.
4. Equivalence test: a C#-side test cannot run JS, so add a small JS test page or a Playwright-free check in the browser: the prefilled PD + gravity example and the built-in PDG with the same gains give the same log to 1e-9 for the "Dünya – boş kol – noktaya git" scenario. Describe how you ran it.
5. The student's code is saved in the scenario JSON (field customController), so export/import repeats the run exactly.
6. Three example laws selectable from a menu: PD + gravity, computed torque written by hand, and a simple sliding-mode law with a boundary layer. Each with one Turkish comment line per step.

Done when: build and tests pass, the equivalence check passes, and an infinite loop in the editor is refused without freezing the page. Add D-entry in DECISIONS.md. Set T10b to "done" in PROJECT.md §7.
```

---

## T12 and later

T12 (experiments for the report), T13 (MuJoCo cross-check) and T14 (optional extensions: 3D view, adaptive controller, 6-DOF data) are written by the conductor when T11 is done, using what the earlier tasks reported.
