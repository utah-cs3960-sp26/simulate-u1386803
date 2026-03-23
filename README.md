# simulate-u1386803

2D circle-tank physics demo: many balls, gravity, ball–ball and ball–wall collisions (SDL3 + C++).

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

On macOS you can install SDL3 with Homebrew (`brew install sdl3`) so CMake can use `find_package(SDL3)`. Otherwise CMake fetches SDL3 via `FetchContent` (set `-DSIMULATE_FETCH_SDL3=ON` to force fetch).

## Run

```bash
./build/simulate
```

- **Restitution** `e`: `-e 0.4` or `--restitution 0.4`, or environment variable `RESTITUTION` (flag wins if both are set).
- **Ball count**: `-n 700` or `--balls 700` (default **700**).
- **Headless check** (no window): `simulate --headless --frames 600 --seed 1 -e 0.25 -n 700`

## Physics

- World **960×960**; circular tank; default **700** balls (radii about **6.4–9.2** px).
- Fixed timestep `1/480` s with accumulator and up to **40** substeps per rendered frame.
- **Friction** is not modeled (velocity-only impulses along contact normals).
- Default restitution is **high (~0.86)** for hard, sustained bouncing; spawn velocities are strong (tangential + radial + upward bias).
- **Linear drag** (`linear_drag_k` ≈ `0.088` 1/s) applies each substep so energy bleeds over time and the pile **settles after long chaotic motion** (on the order of many seconds of visible bouncing, then damp out).
- Contact tuning uses a **lower** `bounce_threshold` so only gentle impacts lose restitution early; furious hits stay elastic longer.

## Visuals

- All balls are drawn **solid green** in the graphical build.

## Tests

```bash
cd build && ctest
```

## Restitution sweep (manual)

Compare settling time with the same seed and ball count:

```bash
./build/simulate --headless --frames 400 --seed 1 -e 0.15 -n 700
./build/simulate --headless --frames 400 --seed 1 -e 0.45 -n 700
./build/simulate --headless --frames 400 --seed 1 -e 0.75 -n 700
```

Lower `e` should damp motion faster; the circular container size is unchanged.
