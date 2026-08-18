/**
 * @file main.cpp
 * @brief Minimal JSBSim driver: attitude-hold autopilot on a Cessna 172.
 *
 * Flies a @c c172p through a predetermined attitude schedule, tracked by a
 * PD feedback controller, printing telemetry as aligned columns.
 *
 * JSBSim is fully synchronous: FGFDMExec::Run() advances exactly one frame of
 * @ref DT and returns, so this program owns the loop. Real-time pacing is
 * therefore opt-in and only enabled by @c --fg, which also streams the state
 * to FlightGear over UDP.
 *
 * @code
 *   ./sim          # batch, runs as fast as the CPU allows
 *   ./sim --fg     # real-time, streams to FlightGear on UDP 5550
 * @endcode
 */

#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>
#include <FGFDMExec.h>

const double DT    = 1.0 / 60.0;  ///< Sim step, sec (60 Hz).
const double T_END = 25.0;        ///< Run length, sec.
const int DECIM    = 6;           ///< Print every 6th frame -> 10 Hz.

const double RAD2DEG = 57.29577951308232;  ///< Radians to degrees.

// Start on a 3 nm final to KSFO runway 28R, on the extended centerline at
// standard 3-degree glidepath height, so the runway is dead ahead in the
// FlightGear view at t=0. KSFO is FlightGear's default location, so its
// scenery ships in the base package.
//
// Derived from the 28R threshold (37.6135336, -122.3571411, elev 12.9 ft,
// true heading 298): 3 nm back along bearing 118, and 3 nm * tan(3 deg).
// Latitude is geodetic (map) latitude, not geocentric.
const double IC_LAT_DEG =   37.5901;  ///< Geodetic latitude, deg.
const double IC_LON_DEG = -122.3015;  ///< Longitude, deg.
const double IC_HDG_DEG =  298.0;     ///< True heading, deg — runway 28R heading.
const double IC_ALT_FT  =  968.0;     ///< Altitude, ft MSL (955 ft AGL).

/// Attitude targets commanded by the schedule.
struct Target { double roll_deg, pitch_deg; };

/**
 * @brief Predetermined attitude schedule — a pure function of time.
 *
 * This is the open-loop half of the system: it says where the aircraft should
 * be pointing, and says nothing about how to get there. The closed-loop half
 * (@ref aileronGain and @ref elevatorGain) does the tracking.
 *
 * @param t Sim time, sec.
 * @return Commanded roll and pitch, deg.
 */
Target schedule(double t)
{
    if (t <  5.0) return { 0.0,  0.0};  // settle at trim
    if (t <  8.0) return { 0.0,  8.0};  // climb
    if (t < 11.0) return { 0.0, -8.0};  // descend
    if (t < 15.0) return { 0.0,  0.0};  // back to level
    if (t < 18.0) return {30.0,  0.0};  // bank right
    return { 0.0, 0.0 };                // roll out
}

/**
 * @brief Roll channel PD law: roll target and state -> aileron delta.
 *
 * Deliberately free of any JSBSim dependency — plain arithmetic on doubles, so
 * the gains can be tuned and reasoned about in isolation.
 *
 * The aileron commands a roll *rate*, not a bank angle, so proportional
 * feedback alone would close an integrator loop and ring; the rate term damps
 * it. Sign is direct: positive aileron rolls right.
 *
 * @param roll_target_deg Commanded bank angle, deg.
 * @param roll_deg        Actual bank angle, deg.
 * @param roll_rate_dps   Body roll rate p, deg/sec.
 * @return Aileron delta to add to the trimmed baseline.
 *
 * @note No clamping: the c172p FCS already clips this to [-1,1] internally.
 */
double aileronGain(double roll_target_deg, double roll_deg, double roll_rate_dps)
{
    const double Kp = 0.045, Kd = 0.014;

    return Kp * (roll_target_deg - roll_deg) - Kd * roll_rate_dps;
}

/**
 * @brief Pitch channel PD law: pitch target and state -> elevator delta.
 *
 * Same shape as @ref aileronGain, but negated overall: in JSBSim positive
 * elevator is nose *down*, so a positive pitch error (need more nose up) wants
 * a negative command.
 *
 * @param pitch_target_deg Commanded pitch angle, deg.
 * @param pitch_deg        Actual pitch angle, deg.
 * @param pitch_rate_dps   Body pitch rate q, deg/sec.
 * @return Elevator delta to add to the trimmed baseline.
 *
 * @note No clamping: the c172p FCS already clips this to [-1,1] internally.
 */
double elevatorGain(double pitch_target_deg, double pitch_deg, double pitch_rate_dps)
{
    const double Kp = 0.100, Kd = 0.025;

    return -(Kp * (pitch_target_deg - pitch_deg) - Kd * pitch_rate_dps);
}

/**
 * @brief Sets up the FDM, trims, then runs the schedule to completion.
 *
 * @param argc Argument count.
 * @param argv Pass @c --fg to enable FlightGear output and real-time pacing.
 * @return 0 on completion.
 */
int main(int argc, char** argv)
{
    bool fg = (argc > 1 && std::strcmp(argv[1], "--fg") == 0);

    JSBSim::FGFDMExec fdm;
    fdm.SetRootDir(SGPath(JSBSIM_ROOT));
    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));
    fdm.Setdt(DT);  // must precede SetOutputDirectives: the output rate is
                    // computed from dt at load time
    fdm.LoadModel("c172p");

    if (fg) fdm.SetOutputDirectives(SGPath("data_output/flightgear.xml"));

    fdm.SetPropertyValue("ic/lat-geod-deg", IC_LAT_DEG);
    fdm.SetPropertyValue("ic/long-gc-deg", IC_LON_DEG);
    fdm.SetPropertyValue("ic/psi-true-deg", IC_HDG_DEG);
    fdm.SetPropertyValue("ic/h-sl-ft", IC_ALT_FT);
    fdm.SetPropertyValue("ic/vc-kts", 100.0);
    fdm.SetPropertyValue("ic/gamma-deg", 0.0);
    fdm.RunIC();

    fdm.SetPropertyValue("propulsion/set-running", -1);  // -1 = all engines

    fdm.SetPropertyValue("simulation/do_simple_trim", 1);  // tFull

    const double elev_trim = fdm.GetPropertyValue("fcs/elevator-cmd-norm");
    const double aile_trim = fdm.GetPropertyValue("fcs/aileron-cmd-norm");
    const double thr_trim  = fdm.GetPropertyValue("fcs/throttle-cmd-norm");

    auto t0 = std::chrono::steady_clock::now();

    printf("  time     alt    kias   roll  roll*  pitch pitch*   vs_fpm    elev     ail\n");

    for (int frame = 0; fdm.GetSimTime() < T_END; ++frame) {
        Target tgt = schedule(fdm.GetSimTime());
        double roll  = fdm.GetPropertyValue("attitude/phi-deg");
        double pitch = fdm.GetPropertyValue("attitude/theta-deg");
        double elev = elevatorGain(tgt.pitch_deg, pitch,
                          fdm.GetPropertyValue("velocities/q-rad_sec") * RAD2DEG);
        double ail  = aileronGain(tgt.roll_deg, roll,
                          fdm.GetPropertyValue("velocities/p-rad_sec") * RAD2DEG);

        fdm.SetPropertyValue("fcs/elevator-cmd-norm", elev_trim + elev);
        fdm.SetPropertyValue("fcs/aileron-cmd-norm", aile_trim + ail);
        fdm.SetPropertyValue("fcs/throttle-cmd-norm", thr_trim);

        fdm.Run();

        if (frame % DECIM == 0)
            printf("%6.2f %7.1f %7.1f %6.2f %6.1f %6.2f %6.1f %8.1f %7.3f %7.3f\n",
                   fdm.GetSimTime(),
                   fdm.GetPropertyValue("position/h-sl-ft"),
                   fdm.GetPropertyValue("velocities/vc-kts"),
                   roll, tgt.roll_deg,
                   pitch, tgt.pitch_deg,
                   fdm.GetPropertyValue("velocities/h-dot-fps") * 60.0,
                   fdm.GetPropertyValue("fcs/elevator-pos-norm"),
                   fdm.GetPropertyValue("fcs/aileron-pos-norm"));

        // Absolute deadline, so pacing jitter does not accumulate.
        if (fg)
            std::this_thread::sleep_until(t0 +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(fdm.GetSimTime())));
    }

    return 0;
}
