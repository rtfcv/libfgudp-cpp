# libfgudp-cpp

A minimal C++ program that drives the [JSBSim](https://github.com/JSBSim-Team/jsbsim) flight
dynamics model directly through its C++ API, flies a Cessna 172 through a predetermined attitude
schedule using a PD attitude-hold controller, prints telemetry, and optionally streams the state
live to [FlightGear](https://www.flightgear.org/) over UDP.

Two source files, no abstraction layers, no framework.

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

CMake fetches and builds JSBSim v1.3.1 as a subproject — no system install, no sudo. The first
configure clones ~100 MB and the first build takes 2–4 minutes; after that only `main.cpp`
recompiles.

Requirements: CMake ≥ 3.18, a C++17 compiler, and `git` on `PATH`. Builds on Linux and on Windows
with MSVC (JSBSim links `ws2_32` and propagates `JSBSIM_STATIC_LINK` itself).

## Run

```bash
./build/sim          # batch: runs as fast as the CPU allows, finishes instantly
./build/sim --fg     # real-time: 25 s wall clock, streams to FlightGear
```

Telemetry prints as aligned columns at 10 Hz, showing actual vs commanded attitude:

```
  time     alt    kias   roll  roll*  pitch pitch*   vs_fpm    elev     ail
 15.52   948.7   101.2  15.19   30.0   0.21    0.0    -65.3   0.207   0.000
 16.52   945.6   100.9  25.77   30.0  -0.31    0.0   -316.5   0.196   0.000
 17.52   939.4   101.3  29.07   30.0  -1.35    0.0   -388.1   0.137   0.000
```

## Watching it in FlightGear

Start FlightGear as an external-FDM viewer first, then run the sim:

```bash
fgfs --aircraft=c172p --native-fdm=socket,in,60,,5550,udp --fdm=external
./build/sim --fg
```

The sim emits 408-byte `FG_NET_FDM` (version 24) packets at 60 Hz on UDP port 5550, using JSBSim's
built-in FlightGear output rather than a hand-rolled protocol. Output is configured by
`data_output/flightgear.xml` from the JSBSim source tree.

If FlightGear is not listening, JSBSim logs `Connection refused` on every send and the simulation
carries on — the socket is non-blocking UDP and never stalls the loop.

The aircraft starts on a **3 nm final to KSFO runway 28R**, on the extended centerline at 968 ft MSL
(standard 3° glidepath height), heading 298° — so the runway is dead ahead in the view at `t=0`.
KSFO is FlightGear's default location, so its scenery ships in the base package.

Change `IC_LAT_DEG` / `IC_LON_DEG` / `IC_HDG_DEG` / `IC_ALT_FT` at the top of `main.cpp` to start
elsewhere. Note the latitude property is `ic/lat-geod-deg` (geodetic — the latitude a map gives you),
not `ic/lat-gc-deg`, which is geocentric and would misplace the aircraft by ~20 km at this latitude.

## How it works

The flight profile is split into an open-loop half and a closed-loop half:

- `schedule(t)` — a pure function of time returning **target** roll and pitch. Level, climb to +8°,
  descend to −8°, level, bank to 30°, then roll out.
- `aileronGain()` / `elevatorGain()` — one PD control law per axis, each taking the target attitude,
  the actual attitude and the body rate, and returning a control surface delta. Both take only
  `double`s, with no JSBSim dependency, so the gains can be tuned in isolation.

The loop reads state, computes commands, and steps the FDM. JSBSim is fully synchronous —
`FGFDMExec::Run()` advances exactly one frame and returns — so the program owns its own loop and
real-time pacing is opt-in.

Control deltas are applied on top of the values captured after trim, so the trimmed flight condition
is preserved.

## Documentation

```bash
doxygen Doxyfile     # -> docs/html/index.html
```

Generates API docs with Graphviz call and caller graphs (requires `doxygen` and `graphviz`).

## License

JSBSim is licensed under the **LGPL 2.1** and is linked **statically** here
(`BUILD_SHARED_LIBS=OFF`). Static linking under the LGPL carries relinking obligations that dynamic
linking does not; if that matters for your use, set `BUILD_SHARED_LIBS=ON` in `CMakeLists.txt`.
