# Robot Kol Sanal Laboratuvarı — KON 445E

Online virtual laboratory for the 4-DOF OpenMANIPULATOR-X arm (KON 445E, ITU, Fall 2026).

Read `PROJECT.md` (how the repository is built) and `SPEC.md` (the physics) before changing anything.

## Running

Requires the .NET 10 SDK (version pinned in `global.json`).

```
dotnet build
dotnet test
dotnet run --project src/RobotArm.Web      # then open the address it prints
```

The Web project runs on macOS and Windows.

## Layout

- `src/RobotArm.Core` — plant model, solver, controllers, tasks, logging. References nothing.
- `src/RobotArm.Web` — Blazor WebAssembly user interface. References Core.
- `tests/RobotArm.Tests` — xunit tests against `reference/test_vectors.json`.
- `reference/` — independent Python reference model and test vectors.
- `docs/` — report, roadmap, design decisions.
