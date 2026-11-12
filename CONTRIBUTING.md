# Contributing to FlightDynamics 6-DOF

Thank you for your interest in contributing to **FlightDynamics 6-DOF**! We welcome bug reports, aerodynamic improvements, GNC algorithms, documentation enhancements, and feature contributions from aerospace engineers, game developers, and roboticists worldwide.

---

## 1. Code of Conduct

All contributors and maintainers are expected to adhere to our [Code of Conduct](CODE_OF_CONDUCT.md). Please ensure respectful, constructive, and inclusive interactions across all discussions, issues, and pull requests.

---

## 2. Git Branching & Workflow Strategy

We adhere to a streamlined **Trunk-Based Development** (or GitHub Flow) workflow:

1. **`main` Branch**: Production-ready, stable, and protected. Every commit on `main` must pass all 190+ automated engineering assertions and compile cleanly in Unity 6.
2. **Feature / Fix Branches**: Branch off `main` using descriptive naming:
   - `feat/<feature-name>` (e.g., `feat/vtol-tilt-rotor-mixing`)
   - `fix/<issue-description>` (e.g., `fix/runway-pitch-rotation`)
   - `docs/<topic>` (e.g., `docs/architecture-vlm`)
   - `refactor/<module>` (e.g., `refactor/mavlink-codec`)

---

## 3. Commit Message Convention

We strictly follow the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) specification. This allows automated changelog generation and maintains a clean git history.

### Format
```text
<type>(<scope>): <short description in present tense>

[optional body explaining motivation and technical details]

[optional footer(s), e.g., Fixes #123]
```

### Allowed Types
| Type | Description |
| :--- | :--- |
| `feat` | Introduces a new feature or capability (e.g., new aerodynamic solver, sensor model) |
| `fix` | Resolves a bug or numerical instability |
| `docs` | Documentation-only updates (README, docs folder, code comments) |
| `refactor` | Code restructuring without changing observable physics or API behavior |
| `perf` | Optimizations improving simulation frame rate or sub-step computation |
| `test` | Adding or updating unit/integration test assertions |
| `build` | Changes to build scripts (`build-and-package.sh`, csproj, UPM packaging) |
| `ci` | Changes to container scripts, Dockerfiles, or CI workflows |
| `chore` | Routine maintenance tasks (dependency updates, metadata cleanup) |

### Examples
- `feat(aero): add ground boundary layer gust attenuation for Dryden turbulence`
- `fix(ground): decouple nose-gear pitch constraint to allow takeoff rotation`
- `docs(px4): add troubleshooting guide for Colima containerd socket forwarding`

---

## 4. Pull Request & Merge Rules

### Merge Policy
- Pull Requests are merged using **Squash and Merge** (or **Rebase and Merge**) to keep linear git history on `main`.
- Each PR must have a clear, descriptive title matching the Conventional Commits format.

### PR Requirements & Review Checklist
Before requesting a review, verify the following:
1. **Automated Verification**:
   Run the full pipeline script locally:
   ```bash
   ./scripts/build-and-package.sh
   ```
   **All 190+ engineering unit tests must pass** (`ALL 190 ENGINEERING ASSERTIONS PASSED SUCCESSFULLY`), and the Unity 6 C# compiler must complete with **0 warnings and 0 errors**.
2. **MAVLink & SITL Compatibility**:
   If changing sensor suites, actuator channels, or flight dynamics, verify that both PX4 SITL lockstep (`TCP :4560`) and QGroundControl (`UDP :14550`) operate without regressions.
3. **No GPL Contamination**:
   This project is licensed under Apache 2.0. Clean-room implementations only. Do **not** copy, paste, or link GPL/LGPL licensed code (e.g., original Fortran/C from XFOIL, AVL, QPROP).

---

## 5. Coding Standards

### C# / .NET Standard 2.1
- **Target Framework**: Core physics algorithms must stay pure C# targeting `.NET Standard 2.1` with **zero external dependencies** (`System.Math`, `System.Buffers.Binary`, `System.Numerics`).
- **Coordinate Conventions**:
  - Aerospace dynamics must strictly use **NED** (North-East-Down) for world frames and **FRD** (Forward-Right-Down) for body frames.
  - Conversions to/from Unity Left-Handed (`EUN/RUF`) must strictly pass through `FrameConverter` to maintain mathematical invariance.
- **Allocation & GC Zero-Pressure**:
  - Real-time physics sub-steps (200~400 Hz) must avoid heap allocations (`new byte[]`, `new List<T>`). Prefer `Span<T>`, `stackalloc`, and pre-allocated buffers.

---

## 6. Developing & Packaging

To build the protected `.NET Standard 2.1` assembly, obfuscate internal symbols, and update the UPM package:

```bash
# 1. Run full test suite, stage symbols, build managed DLL, and verify Unity assemblies
./scripts/build-and-package.sh

# 2. Control containerized PX4 SITL + ROS 2 stack
./scripts/px4-sitl.sh up [iris|plane|standard_vtol]
./scripts/px4-sitl.sh down
./scripts/px4-sitl.sh vm-stop
```
