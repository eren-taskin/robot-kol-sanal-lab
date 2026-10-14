# PROJECT — how this repository is built

Read this file and `SPEC.md` before doing anything in this repository. This applies to people and to AI coding tools alike.

## 1. What we are building

An online virtual laboratory for the 4-DOF OpenMANIPULATOR-X arm, for KON 445E (ITU, Fall 2026). A student chooses a planet, a payload, a task and a controller, runs the experiment in the browser, and studies the plots and metrics. The physics is specified in `SPEC.md`.

## 2. Architecture

The six parts of a control application (Lecture Note 1): plant model, solver, controller form the core; hardware I/O, storage and user interface are the edges.

```
RobotArm.sln
├── global.json                 SDK pinned to 10.0.300, rollForward latestFeature
├── src/
│   ├── RobotArm.Core/          net10.0 class library. References NOTHING.
│   │   ├── Model/              parameters, chain, planets, payload
│   │   ├── Kinematics/         FK, IK, Jacobian
│   │   ├── Dynamics/           RNEA, inertia matrix, Cholesky
│   │   ├── Simulation/         IPlant, ISolver, Rk4Solver, EulerSolver, ArmPlant, Simulation
│   │   ├── Control/            IController, Pid, PdGravity, ComputedTorque, sensors, sampled loop
│   │   ├── Tasks/              trajectories, tasks, scenario, metrics
│   │   └── Logging/            log record, CSV writer (to a TextWriter, never to a file path)
│   └── RobotArm.Web/           Blazor WebAssembly (standalone). References Core.
├── tests/
│   └── RobotArm.Tests/         xunit. References Core. Reads reference/test_vectors.json
├── reference/                  Python reference model and test vectors (not part of the build)
├── docs/                       report, roadmap, decisions
└── .github/workflows/          build + test on every push; publish Web to GitHub Pages
```

### Rules

1. **Dependency rule.** Core references nothing. Nothing in Core mentions a screen, a file path, a timer, the browser or the clock. The test: Core must still compile if Web and Tests are deleted.
2. **Interfaces from the course, extended for four inputs.** Note 2 defines `IPlant.Derivatives(double t, double[] x, double u, double[] dx)` with a scalar input. The arm has four inputs, so ours is:

   ```csharp
   public interface IPlant
   {
       int StateCount { get; }
       int InputCount { get; }
       void Derivatives(double t, ReadOnlySpan<double> x, ReadOnlySpan<double> u, Span<double> dx);
   }
   public interface ISolver
   {
       void Step(IPlant plant, double t, Span<double> x, ReadOnlySpan<double> u, double h);
   }
   ```

   This is a deliberate design decision. It is recorded in `docs/DECISIONS.md` and in the report.
3. **No physics in the user interface.** The UI builds a `Scenario`, asks Core to run it, and draws what Core returns.
4. **The UI never blocks.** The browser runs the simulation in batches: each animation frame advances simulated time by a fixed slice (for example 16 ms of simulated time = 16 steps), then draws. A "run as fast as possible" mode computes the whole run first and then shows it. This is the web version of Lecture Note 4.
5. **One source of truth for numbers.** All parameters live in `RobotArm.Core/Model`. The tests compare them against `reference/test_vectors.json`. Nobody types a parameter twice.

## 3. Coding conventions (from the lecture notes)

- C# 14, `net10.0`, nullable reference types on, warnings treated as errors in Core.
- Classes `sealed` unless there is a reason. Parameters are `init` properties; validation in constructors or setters throws `ArgumentOutOfRangeException` with a clear message.
- Store a copy of any array passed in (`Clone()`), never the caller's array.
- No allocation inside anything called once per simulation step (`Step`, `Derivatives`, controller `Compute`). Work buffers are fields created in the constructor.
- `CultureInfo.InvariantCulture` for every number written or parsed.
- Names: `PascalCase` for types and members, `_camelCase` for private fields, units in names when not SI (`AngleDeg`, `LengthMm`).
- Every public type and member has an XML doc comment that names the SPEC section it implements, for example `/// SPEC §5.3`.
- Tests: one test class per SPEC section, test names say what is checked (`InertiaMatrix_MatchesClosedForm_InRandomPoses`).

## 4. Running

```
dotnet build
dotnet test
dotnet run --project src/RobotArm.Web      # then open the address it prints
```

The Web project runs on macOS and Windows. No Windows Forms in this project.

## 5. How we work with AI tools

Three tools are used: Claude Code, Codex and Google Antigravity (Gemini). The conductor (Claude, in the planning chat) writes one task at a time in `TASKS.md`.

For every task:

1. Start a **new** session in the repository folder.
2. Paste the task prompt from `TASKS.md`.
3. The tool reads `PROJECT.md` and `SPEC.md` first, then does **only** that task.
4. The tool runs `dotnet build` and `dotnet test`. A task is finished only when everything builds and every test passes.
5. The tool ends with a summary: files changed, tests added, anything it could not do, any place where it deviated from SPEC and why.
6. A person reviews the change in GitHub Desktop, then commits and pushes. The commit message starts with the task id, for example `T03: inverse kinematics`.
7. The person pastes the tool's summary back to the conductor, who updates the status table below and writes the next task.

Rules for the tools:

- Do not change `SPEC.md`, `PROJECT.md` or `reference/` unless the task says so. If SPEC looks wrong, stop and report it.
- Do not weaken or delete a failing test to make it pass. Report it.
- Do not add a NuGet or npm package unless the task allows it. If one is needed, say which and why.
- Only one tool works on the repository at a time.

Course rule (Lecture Note 1, §8): every member must be able to explain and defend every line in their area. Owners below are responsible for that.

## 6. Owners

| Area | Owner | Second reader |
|---|---|---|
| Model, kinematics, dynamics (Core/Model, Kinematics, Dynamics) | TBD | TBD |
| Simulation, solver, sensors (Core/Simulation) | TBD | TBD |
| Controllers (Core/Control) | TBD | TBD |
| Tasks, metrics, logging (Core/Tasks, Logging) | TBD | TBD |
| Web interface (Web) | TBD | TBD |

## 7. Status

| Task | Title | Tool | Status |
|---|---|---|---|
| T01 | Solution skeleton and CI | Claude Code | done |
| T02 | Parameters, chain, planets, payload | Claude Code | done |
| T03 | Kinematics | Claude Code | done |
| T04 | Dynamics | Claude Code | not started |
| T05 | Plant, solvers, simulation | Codex | not started |
| T06 | Independent review of T02–T05 | Antigravity | not started |
| T07 | Controllers and sampled loop | Codex | not started |
| T08 | Trajectories, tasks, scenario, metrics, logging | Codex | not started |
| T09 | Web: layout, run loop, arm drawing | Antigravity | not started |
| T10 | Web: plots, metrics, comparison, export | Antigravity | not started |
| T11 | Publish to GitHub Pages | Claude Code | not started |
| T12 | Experiments for the report | conductor + team | not started |
| T13 | Optional: MuJoCo cross-check | conductor + team | not started |
| T14 | Optional: 3D view, adaptive controller, 6-DOF data | — | not started |
