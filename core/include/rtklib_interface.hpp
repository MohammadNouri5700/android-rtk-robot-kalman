
#pragma once

namespace rtklib_localization {

class RTKEngine {
public:
    RTKEngine() = default;
    ~RTKEngine() = default;

    bool initialize();
};

} // namespace rtklib_localization
