#ifndef ATG_ENGINE_SIM_GEARBOX_CAPABILITIES_H
#define ATG_ENGINE_SIM_GEARBOX_CAPABILITIES_H

namespace powertrain {

    struct GearboxCapabilities {
        int gearCount = 0;
        const double *gearRatios = nullptr;

        double finalDrive = 0.0;
        double tireRadius = 0.0;

        bool supportsPreselect = false;
        bool requiresTorqueInterrupt = true;
        bool hasLaunchDevice = false;
        bool supportsRange = true;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_GEARBOX_CAPABILITIES_H */
