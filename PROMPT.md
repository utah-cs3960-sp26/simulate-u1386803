# Autonomous task: 2D ball-in-box physics simulator (SDL3 + C++)

You are building this repository into a **working 2D physics demo**. The human is **not** implementing code in this loop—you are. Read this file fully before acting.

## Repository

- **Name pattern:** `simulate-uXXXXXXX` (already created; keep paths relative to repo root).
- **Primary deliverable:** A native C++ app using **SDL3** for window, rendering, and input.
- **Optional docs:** Update `README.md` with build/run instructions only when you have a working build; keep it accurate.

## Product requirements

### Simulation

1. **Bodies:** Many **circular** balls (dynamic) and **fixed, immovable axis-aligned wall segments** (or a closed polygon made of segments). Walls do not move.
2. **Forces:** **Gravity** downward (constant acceleration). Tune magnitude so ~1000 balls in a container visibly bounce then **settle** into a pile within a reasonable time.
3. **Collisions:** Ball–wall and ball–ball with **configurable coefficient of restitution** `e` in **(0, 1]** (perfectly inelastic would be `e = 0`; implement something like `0.2` default). Collisions are **non-elastic** in the sense that energy is lost when `e < 1`.
4. **Initial scene:** Approximately **1000 balls** spawned inside a **container** built from multiple wall pieces (not a single huge rectangle hack unless you also add internal dividers—prefer a visible “tank” with floor and sides). Arrange radii/masses reasonably (uniform or slight variation). Initial positions should avoid huge overlaps; a grid or layered pour is fine.
5. **Correctness goals (the hard part):**
   - Balls must **not** end up **stuck overlapping** other balls or walls after settling.
   - Balls must **not** **tunnel through** walls at the chosen timestep.
   - Eliminate runaway **vibration / energy explosion** (balls jittering faster and faster, then flying to infinity). This usually means fixing the integrator + **constraint solving** (impulses, position correction, substeps), not just cranking damping blindly.
6. **Restitution semantics:** Expose `e` via **command-line flag**, **environment variable**, or a small config—your choice, document it. With **lower** `e`, motion should **damp faster**; the **final resting pile** should occupy **similar area / height** (same packing geometry), not collapse into a bogus compressed state or expand due to bugs. If friction is zero or minimal, identical rigid packing is expected; if you add friction, state that in README.

### Performance / UX

- Target **smooth** animation on a typical laptop: aim for **60+ FPS** with ~1000 circles. Profile if needed; prefer **algorithmic** fixes (broadphase, solver iterations, substeps budget) over micro-optimizing hot loops first.
- Show **FPS** on screen or in the window title for quick feedback.
- Use **fixed timestep** simulation with accumulator pattern; decouple physics Hz from render Hz. Use **substepping** as required for stability (document chosen dt and max substeps).

## Implementation guidance (not optional—use engineering judgment)

- **Integrator:** Semi-implicit Euler or similar; avoid naive explicit Euler on stiff contacts without correction.
- **Collision detection:** Broadphase (uniform grid or spatial hash) for ball–ball; ball–wall can test against segment list. Keep complexity sane for N≈1000.
- **Response:** Sequential impulses or a small number of **position + velocity** correction iterations per substep. Apply **positional separation** along contact normals with **slop** caps to kill sinking without adding energy.
- **Resting contacts:** Detect low relative velocity and **zero or clamp** restitution for “micro-bounces” to stop jitter; optional **sleeping** for bodies at rest.
- **Numerical safety:** Clamp absurd velocities; assert or log in debug builds if overlaps exceed a threshold after solve.

## Build system and SDL3

- Prefer **CMake** (minimum C++17).
- **Install SDL3** in a portable way and document it in README:
  - **macOS:** Homebrew `sdl3` or build from source; CMake `find_package(SDL3)` if available, or `FetchContent`—pick one approach and stick to it.
  - **Linux:** distro packages or source; same CMake story.
- Provide clear **configure / build / run** commands. A `CMakeLists.txt` at repo root is expected.

## Testing and verification (you must define how *you* verify)

1. **Automated:** Add at least one **headless or short-run** test mode (e.g. `--headless --frames 600 --seed 1`) that runs physics with no window, exits 0, and **fails** if max penetration, max speed, or ball escape-from-bounds exceeds thresholds. This gives Amp a tight feedback loop without staring at pixels.
2. **Manual:** Human runs the graphical app; pile should settle; walls hold; no explosions.
3. **Restitution sweep:** Script or documented commands to run 2–3 `e` values and confirm faster settling with lower `e` without changing container size.

## Git workflow

- **Commit early and often** with **clear messages** (what changed, why).
- When you complete a **milestone** (builds, physics stable, perf OK), commit and briefly note in README or a `PROGRESS.md` what is done and what is next.
- Do **not** commit build artifacts, IDE files, or huge binaries.

## Tools and permissions

- Use the **terminal** to configure, build, and run; fix compile errors yourself.
- If tests fail, **reproduce locally**, add logging if needed, fix root cause, re-run.
- Prefer **small, reviewable commits** over one giant dump.

## Definition of done

- From a clean clone, following README, a developer can **build and run** on macOS and Linux.
- App shows ~1000 balls in a walled container, gravity, collisions, **configurable restitution**, stable settling, **no** tunneling/overlap explosions in normal runs, and **60 FPS** class performance on a typical machine.
- Headless test mode exists and is wired into build (e.g. `ctest` or a test target).

## First actions when this prompt is loaded

1. Inspect the repo; if empty aside from this file, scaffold **CMake + `src/main.cpp`** and get a window clearing to a color with SDL3.
2. Add the physics core incrementally: walls → one ball → many balls → broadphase → stability fixes → headless test.
3. Commit after each stable step.

Begin now from the current repository state.
