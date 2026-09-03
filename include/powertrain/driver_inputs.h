#ifndef ATG_ENGINE_SIM_DRIVER_INPUTS_H
#define ATG_ENGINE_SIM_DRIVER_INPUTS_H

namespace powertrain {

    struct DriverInputs {
        double accelerator = 0.0;
        double brake = 0.0;
        double clutchPedal = 1.0;

        int driveMode = 0;
        int selectedGear = -1;

        bool shiftUpRequest = false;
        bool shiftDownRequest = false;
        bool manualMode = false;

        bool ignitionKey = true;
        bool starterRequest = false;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_DRIVER_INPUTS_H */
