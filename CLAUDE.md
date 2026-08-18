# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release   # first run clones JSBSim (~100MB, 2-4 min build)
cmake --build build -j$(nproc)              # after the first build, only main.cpp recompiles
./build/sim                                 # batch: runs flat out, finishes in ~0.02s
./build/sim --fg                            # real-time (25s wall), streams to FlightGear
doxygen Doxyfile                            # -> docs/html/index.html, with Graphviz call graphs
```

There are no tests and no linter configured.

## Architecture

Two source files: `CMakeLists.txt` and `main.cpp`. Everything (FDM setup, controller, telemetry) is deliberately inline in `main.cpp` — the project constraint is "stupid simple, no unnecessary abstraction."

**JSBSim comes via FetchContent, and this is not incidental.** JSBSim ships no CMake config package and no pkg-config file, so `find_package(JSBSim)` does not work. Its `libJSBSim` target exports includes via `$<BUILD_INTERFACE:...>` *only*, meaning an installed copy gives consumers no usable include path. Consuming it as a subproject is what makes `#include <FGFDMExec.h>` resolve. Do not "clean this up" into a `find_package` call.

`${jsbsim_SOURCE_DIR}` is passed to the program as the `JSBSIM_ROOT` compile definition, because JSBSim's runtime data (`aircraft/`, `engine/`, `systems/`, `data_output/`) lives in the source tree, not in the library.

**Control flow.** `schedule(t)` returns target attitude as a pure function of time (open loop); `aileronGain()` and `elevatorGain()` are one-per-axis PD laws taking (target, actual, body rate) and returning a surface delta (closed loop). All three are free functions above `main`. The loop reads state via `GetPropertyValue`, writes commands via `SetPropertyValue`, then calls `fdm.Run()`.

Both gain functions take only doubles and have no JSBSim dependency, so they are tunable in isolation. Keep it that way. Each holds its own `Kp`/`Kd` locally rather than sharing a gain table.

**JSBSim is fully synchronous and single-threaded** — no threads anywhere in its `src/`. `FGFDMExec::Run()` advances exactly one frame of `DT` and returns. The caller owns the loop, so real-time pacing is opt-in (only under `--fg`) and the sim is deterministic. Pacing sleeps to an *absolute* deadline derived from sim time so jitter does not accumulate.

## JSBSim gotchas

These were all found the hard way. Each one fails silently or misleadingly.

- **Engine start is `propulsion/set-running` = `-1`** (all engines). `propulsion/engine[0]/set-running` is *not* a bound property — setting it does nothing, the engine stays off, throttle has no effect on thrust, and trim then fails with `TrimFailureException`.
- **Positive elevator is nose DOWN.** A positive elevator command pitches down, not up.
- **Capture every trim baseline you intend to overwrite.** After trim, `fcs/elevator-cmd-norm`, `fcs/aileron-cmd-norm` and `fcs/throttle-cmd-norm` hold the values that sustain level flight. Writing absolute values into any of them destroys the trim. Controller output is a *delta* added to the captured baseline. Forgetting the aileron one specifically causes a slow unexplained roll from t=0, since a nonzero aileron trim counters engine torque.
- **Set initial latitude via `ic/lat-geod-deg`, not `ic/lat-gc-deg`.** Map/chart latitudes are
  geodetic; `ic/lat-gc-deg` is geocentric. Feeding a geodetic value into the geocentric property
  silently misplaces the aircraft by roughly 20 km at mid-latitudes. Longitude has no such split.
- **`Setdt()` must precede `SetOutputDirectives()`.** Output rate decimation is computed as `0.5 + 1.0/(GetDeltaT()*rateHz)` at load time, so loading directives before `dt` is set yields the wrong output rate.
- **Do not clamp surface commands.** The c172p FCS already clips them to [-1,1] internally via `<clipto>`.
- **Use PD, not P.** Elevator and aileron command a *rate*, not an angle, so proportional-only feedback on angle closes an integrator loop and rings. Rate damping comes from `velocities/p-rad_sec` and `velocities/q-rad_sec` (both rad/s).
- **`BUILD_PYTHON_MODULE` and `BUILD_DOCS` default ON upstream** and pull in Cython/Doxygen. `CMakeLists.txt` forces both OFF; removing that breaks configure.
- **`Doxyfile` must keep `INPUT = main.cpp`.** Pointing Doxygen at the tree makes it crawl the ~119MB JSBSim checkout under `build/`.

## Verification

The controller is the thing worth checking after any change to `schedule()` or either gain function. Run `./build/sim` and confirm against the telemetry columns:

- `t < 5` — roll and pitch hold ~0. If altitude diverges here, trim failed or a trim baseline was not captured.
- `t = 5-11` — pitch tracks to +8 then -8 deg, settling without overshoot or ringing. Sustained oscillation means Kd too low or Kp too high; sluggish approach means Kp too low.
- `t = 15-18` — roll tracks to +30 deg and holds.
- `t > 18` — roll returns to ~0. This is the headline closed-loop behavior; the earlier open-loop version stayed banked indefinitely.

For `--fg` without FlightGear installed, verify the wire format directly: the stream should be 408-byte `FG_NET_FDM` version 24 packets at 60 Hz, and `--fg` should take ~25s wall clock while the default run stays instant.
