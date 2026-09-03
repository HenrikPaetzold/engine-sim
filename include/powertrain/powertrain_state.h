#ifndef ATG_ENGINE_SIM_POWERTRAIN_STATE_H
#define ATG_ENGINE_SIM_POWERTRAIN_STATE_H

namespace powertrain {

    static constexpr int MaxClutches = 2;

    struct PowertrainState {
        double dt = 0.0;
        double time = 0.0;

        double engineSpeed = 0.0;
        double engineRpm = 0.0;
        double throttlePlate = 0.0;
        double manifoldPressure = 0.0;
        double intakeAfr = 0.0;
        double exhaustO2 = 0.0;
        double indicatedTorque = 0.0;
        double timingAdvance = 0.0;

        double coolantTemperature = 0.0;
        double oilTemperature = 0.0;

        int gear = -1;
        int preselectedGear = -1;
        int gearCount = 0;

        double clutchPressure[MaxClutches] = { 0.0, 0.0 };
        double clutchSlipSpeed[MaxClutches] = { 0.0, 0.0 };
        double turbineSpeed = 0.0;
        double converterSlip = 0.0;
        double lockupPressure = 0.0;

        double vehicleSpeed = 0.0;
        double wheelSpeed = 0.0;
        double roadGrade = 0.0;

        bool engineRunning = false;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_POWERTRAIN_STATE_H */
