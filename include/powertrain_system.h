#ifndef ATG_ENGINE_SIM_POWERTRAIN_SYSTEM_H
#define ATG_ENGINE_SIM_POWERTRAIN_SYSTEM_H

#include "powertrain/powertrain_controller.h"
#include "external_throttle.h"

class Engine;
class Transmission;
class Vehicle;
class Simulator;

class PowertrainSystem {
    public:
        struct Parameters {
            double controlFrequency = 1000.0;
        };

    public:
        PowertrainSystem();
        ~PowertrainSystem();

        void initialize(const Parameters &params);
        void attach(Simulator *simulator);
        void detach();

        void setController(powertrain::PowertrainController *controller);
        inline powertrain::PowertrainController *getController() const { return m_controller; }

        inline bool isActive() const { return m_controller != nullptr && m_simulator != nullptr; }

        void reset();
        void update(double dt);

        inline powertrain::DriverInputs &getDriverInputs() { return m_inputs; }
        inline const powertrain::DriverInputs &getDriverInputs() const { return m_inputs; }
        inline const powertrain::PowertrainState &getState() const { return m_state; }
        inline const powertrain::ActuatorCommands &getCommands() const { return m_commands; }

        void sampleState(double dt);

    protected:
        void applyCommands();

        powertrain::PowertrainController *m_controller;
        Simulator *m_simulator;

        ExternalThrottle m_throttle;
        Throttle *m_previousThrottle;

        powertrain::PowertrainState m_state;
        powertrain::DriverInputs m_inputs;
        powertrain::ActuatorCommands m_commands;

        Parameters m_params;

        double m_accumulator;
        double m_time;
};

#endif /* ATG_ENGINE_SIM_POWERTRAIN_SYSTEM_H */
