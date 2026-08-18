#include <cstdio>
#include <FGFDMExec.h>

const double DT    = 1.0 / 120.0;  // sim step, sec
const double T_END = 25.0;         // run length, sec
const int DECIM    = 12;           // print every 12th frame -> 10 Hz

// Predetermined control input as a function of time: elevator doublet
// then an aileron roll input. Values are deltas applied on top of trim.
struct Cmd { double elevator, aileron; };

Cmd control(double t)
{
    if (t <  5.0) return { 0.0,  0.0};  // settle at trim
    if (t <  8.0) return {-0.2,  0.0};  // elevator doublet: pitch up (JSBSim: +elevator = nose down)
    if (t < 11.0) return { 0.2,  0.0};  //                   pitch down
    if (t < 15.0) return { 0.0,  0.0};  // recover
    if (t < 18.0) return { 0.0,  0.3};  // aileron roll input
    return { 0.0, 0.0 };                // release
}

int main()
{
    JSBSim::FGFDMExec fdm;
    fdm.SetRootDir(SGPath(JSBSIM_ROOT));
    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));
    fdm.Setdt(DT);
    fdm.LoadModel("c172p");

    fdm.SetPropertyValue("ic/h-sl-ft", 4500.0);
    fdm.SetPropertyValue("ic/vc-kts", 100.0);
    fdm.SetPropertyValue("ic/gamma-deg", 0.0);
    fdm.RunIC();

    fdm.SetPropertyValue("propulsion/set-running", -1);  // -1 = all engines

    fdm.SetPropertyValue("simulation/do_simple_trim", 1);  // tFull

    const double elev_trim = fdm.GetPropertyValue("fcs/elevator-cmd-norm");
    const double aile_trim = fdm.GetPropertyValue("fcs/aileron-cmd-norm");
    const double thr_trim  = fdm.GetPropertyValue("fcs/throttle-cmd-norm");

    printf("  time     alt    kias   pitch    roll     aoa   vs_fpm    elev     ail\n");

    for (int frame = 0; fdm.GetSimTime() < T_END; ++frame) {
        Cmd c = control(fdm.GetSimTime());
        fdm.SetPropertyValue("fcs/elevator-cmd-norm", elev_trim + c.elevator);
        fdm.SetPropertyValue("fcs/aileron-cmd-norm", aile_trim + c.aileron);
        fdm.SetPropertyValue("fcs/throttle-cmd-norm", thr_trim);

        fdm.Run();

        if (frame % DECIM == 0)
            printf("%6.2f %7.1f %7.1f %7.2f %7.2f %7.2f %8.1f %7.3f %7.3f\n",
                   fdm.GetSimTime(),
                   fdm.GetPropertyValue("position/h-sl-ft"),
                   fdm.GetPropertyValue("velocities/vc-kts"),
                   fdm.GetPropertyValue("attitude/theta-deg"),
                   fdm.GetPropertyValue("attitude/phi-deg"),
                   fdm.GetPropertyValue("aero/alpha-deg"),
                   fdm.GetPropertyValue("velocities/h-dot-fps") * 60.0,
                   fdm.GetPropertyValue("fcs/elevator-pos-norm"),
                   fdm.GetPropertyValue("fcs/aileron-pos-norm"));
    }

    return 0;
}
