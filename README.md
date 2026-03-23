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
- **Ball count**: `-n 200` or `--balls 200` (default 200).
- **Headless check** (no window): `simulate --headless --frames 600 --seed 1 -e 0.25 -n 200`

## Physics

- Fixed timestep `1/480` s with accumulator and up to **32** substeps per rendered frame.
- **Friction** is not modeled (velocity-only impulses along contact normals).

## Tests

```bash
cd build && ctest
```

## Restitution sweep (manual)

Compare settling time with the same seed and ball count:

```bash
./build/simulate --headless --frames 400 --seed 1 -e 0.15 -n 200
./build/simulate --headless --frames 400 --seed 1 -e 0.45 -n 200
./build/simulate --headless --frames 400 --seed 1 -e 0.75 -n 200
```

Lower `e` should damp motion faster; the circular container size is unchanged.
