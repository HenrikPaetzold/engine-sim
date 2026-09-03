#ifndef ATG_ENGINE_SIM_ACTUATOR_COMMANDS_H
#define ATG_ENGINE_SIM_ACTUATOR_COMMANDS_H

#include "powertrain_state.h"

namespace powertrain {

    struct ActuatorCommands {
        double throttlePlate = 0.0;
        double ignitionCutFraction = 0.0;
        double fuelCutFraction = 0.0;
        double timingOffset = 0.0;

        int targetGear = -1;
        int preselectGear = -1;

        double clutchPressure[MaxClutches] = { 0.0, 0.0 };
        double lockupPressure = 0.0;

        bool starterEnabled = false;
        bool ignitionEnabled = true;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_ACTUATOR_COMMANDS_H */
