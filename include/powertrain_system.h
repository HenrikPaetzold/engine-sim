#ifndef ATG_ENGINE_SIM_POWERTRAIN_SYSTEM_H
#define ATG_ENGINE_SIM_POWERTRAIN_SYSTEM_H

#include "powertrain/powertrain_controller.h"
#include "external_throttle.h"
#include "config/parameter_registry.h"
#include "config/config_server.h"
#include "config/shift_recorder.h"
#include "config/channel_recorder.h"
#include "control/signal_table.h"
#include "adaptation/adaptation_manager.h"
#include "config/drive_mode.h"

#include <string>
#include <vector>

class Engine;
class Transmission;
class Vehicle;
class Simulator;

class PowertrainSystem {
    public:
        struct Parameters {
            double controlFrequency = 1000.0;
            double telemetryFrequency = 20.0;
        };

    public:
        PowertrainSystem();
        ~PowertrainSystem();

        void initialize(const Parameters &params);
        void attach(Simulator *simulator);
        void detach();

        void setController(powertrain::PowertrainController *controller);
        void setOverlayController(powertrain::PowertrainController *overlay);

        inline config::ChannelTable &getChannels() { return m_channels; }
        inline const config::ChannelTable &getChannels() const { return m_channels; }
        inline config::ChannelRecorder &getScope() { return m_scope; }
        inline const config::ChannelRecorder &getScope() const { return m_scope; }
        void setAdaptationManager(adaptation::AdaptationManager *manager);
        void setConfigServer(config::ConfigServer *server);
        void setDriveModes(config::DriveModeSet *modes, config::ParameterRegistry *registry);
        void registerParameters(config::ParameterRegistry *registry);
        bool selectDriveMode(
            const std::string &name,
            config::DriveModeSet *modes,
            config::ParameterRegistry *registry);
        inline powertrain::PowertrainController *getController() const { return m_controller; }

        inline bool isActive() const { return m_controller != nullptr && m_simulator != nullptr; }

        void reset();
        void update(double dt);

        inline powertrain::DriverInputs &getDriverInputs() { return m_inputs; }
        inline const powertrain::DriverInputs &getDriverInputs() const { return m_inputs; }
        inline const powertrain::PowertrainState &getState() const { return m_state; }
        inline const powertrain::ActuatorCommands &getCommands() const { return m_commands; }
        inline const config::ShiftRecorder &getShiftRecorder() const { return m_shiftRecorder; }

        void sampleState(double dt);
        void syncGearbox();
        void applyGateMode();
        void recordShift(double dt);
        void fillTelemetry(config::TelemetrySample *sample) const;

    protected:
        void applyCommands();

        powertrain::PowertrainController *m_controller;
        powertrain::PowertrainController *m_overlay;

        config::ChannelTable m_channels;
        config::ChannelRecorder m_scope;
        control::SignalTable m_signalScratch;
        control::SignalTable m_actuatorScratch;
        std::vector<int> m_signalChannel;
        std::vector<int> m_actuatorChannel;
        bool m_channelsDefined = false;

        void fillChannels(double dt);
        adaptation::AdaptationManager *m_adaptation;
        config::ConfigServer *m_server;
        config::ShiftRecorder m_shiftRecorder;
        config::DriveModeSet *m_modes;
        config::ParameterRegistry *m_registry;
        std::string m_activeMode;
        Simulator *m_simulator;

        ExternalThrottle m_throttle;
        Throttle *m_previousThrottle;

        powertrain::PowertrainState m_state;
        powertrain::DriverInputs m_inputs;
        powertrain::ActuatorCommands m_commands;

        Parameters m_params;

        void publishTelemetry();

        double m_accumulator;
        double m_telemetryAccumulator;
        double m_time;
};

#endif /* ATG_ENGINE_SIM_POWERTRAIN_SYSTEM_H */
