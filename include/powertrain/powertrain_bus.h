#ifndef ATG_ENGINE_SIM_POWERTRAIN_BUS_H
#define ATG_ENGINE_SIM_POWERTRAIN_BUS_H

namespace powertrain {

    enum class TorqueIntervention {
        Spark,
        Fuel,
        Throttle
    };

    enum class EngineState {
        Off,
        Cranking,
        ColdStart,
        Idle,
        Running,
        Limiting
    };

    struct PowertrainBus {
        double torqueReductionRequest = 0.0;
        TorqueIntervention interventionType = TorqueIntervention::Spark;
        double speedRequest = 0.0;
        bool speedRequestActive = false;
        bool shiftInProgress = false;

        double indicatedTorque = 0.0;
        double maxTorqueAtCurrentSpeed = 0.0;
        double engineSpeed = 0.0;
        bool torqueReductionAvailable = false;
        EngineState engineState = EngineState::Off;

        inline void resetTransmissionRequests() {
            torqueReductionRequest = 0.0;
            interventionType = TorqueIntervention::Spark;
            speedRequest = 0.0;
            speedRequestActive = false;
            shiftInProgress = false;
        }
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_POWERTRAIN_BUS_H */
