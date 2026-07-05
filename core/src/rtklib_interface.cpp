
#include "rtklib_interface.hpp"

namespace rtklib_localization {

RTKEngine::RTKEngine() {
    rtkinit(&rtk_, &opt_);
    memset(&nav_, 0, sizeof(nav_t));

    // Default RTK Options for vehicular robotics
    opt_.mode = PMODE_KINEMA;  // Kinematic RTK
    opt_.nf = 2;               // Dual frequency (L1+L2/L5)
    opt_.elmin = 15.0 * D2R;   // Aggressive elevation mask (15 deg)
    opt_.modear = ARMODE_CONT; // Continuous Ambiguity Resolution
    opt_.thresar[0] = 3.0;     // Standard validation threshold
    opt_.ionoopt = IONOOPT_BRDC; // Broadcast ionospheric model
    opt_.tropopt = TROPOPT_SAAS; // Saastamoinen tropospheric model
}

RTKEngine::~RTKEngine() {
    rtkfree(&rtk_);
    freenav(&nav_, 0xFF);
}

bool RTKEngine::initialize() {
    return true;
}

bool RTKEngine::process(obs_t* obs, nav_t* nav) {
    if (obs->n <= 0) return false;
    return rtkpos(&rtk_, obs->data, obs->n, nav) != 0;
}

bool RTKEngine::processRaw(double timestamp, const std::vector<std::vector<double>>& raw_data) {
    if (raw_data.empty()) return false;

    obs_t obs = {0};
    obs.n = (int)raw_data.size();
    obs.data = (obsd_t*)malloc(sizeof(obsd_t) * obs.n);

    gtime_t time = epoch2time(nullptr);
    time.time = (time_t)timestamp;
    time.sec = timestamp - time.time;

    for (int i = 0; i < obs.n; ++i) {
        obs.data[i].time = time;
        obs.data[i].sat = (unsigned char)raw_data[i][0];
        obs.data[i].P[0] = raw_data[i][2];
        obs.data[i].L[0] = raw_data[i][3];
        obs.data[i].D[0] = (float)raw_data[i][4];
        obs.data[i].SNR[0] = (unsigned char)(raw_data[i][5] / 0.25);
    }

    int stat = rtkpos(&rtk_, obs.data, obs.n, &nav_);
    free(obs.data);

    return (stat != 0 && rtk_.sol.stat == SOLQ_FIX);
}

} // namespace rtklib_localization
