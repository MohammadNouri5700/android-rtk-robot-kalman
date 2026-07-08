
#pragma once
#include "rtklib.h"

// RTKLIB defines a macro 'lock' that conflicts with std::lock in <mutex>
#ifdef lock
#undef lock
#endif

#include <vector>

namespace rtklib_localization {

class RTKEngine {
public:
    RTKEngine();
    ~RTKEngine();

    bool initialize();

    /**
     * @brief Process raw observations to get a solution.
     * @param obs Rover observations
     * @param nav Navigation data
     * @return true if a solution was found
     */
    bool process(obs_t* obs, nav_t* nav);
    bool processRaw(double timestamp, const std::vector<std::vector<double>>& raw_data);

    sol_t getSolution() const { return rtk_.sol; }

private:
    rtk_t rtk_;
    prcopt_t opt_;
    solopt_t solopt_;
    nav_t nav_;
};


} // namespace rtklib_localization
