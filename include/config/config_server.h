#ifndef ATG_ENGINE_SIM_CONFIG_SERVER_H
#define ATG_ENGINE_SIM_CONFIG_SERVER_H

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

class PowertrainSystem;

namespace config {

    class ParameterRegistry;
    class DriveModeSet;

    struct TelemetrySample {
        double time = 0.0;
        double engineRpm = 0.0;
        double throttlePlate = 0.0;
        double pedal = 0.0;
        double revLimitSoft = 0.0;
        double revLimitHard = 0.0;
        double indicatedTorque = 0.0;
        double outputTorque = 0.0;
        double lastShiftDuration = 0.0;
        double torqueRequest = 0.0;
        double coolantTemperature = 0.0;
        double oilTemperature = 0.0;
        double oilViscosity = 0.0;
        double frictionPower = 0.0;
        double vehicleSpeed = 0.0;
        double roadGrade = 0.0;
        int gear = -1;
        std::string range = "N";
        bool parkLock = false;
        double clutchPressure = 0.0;
        double ignitionCut = 0.0;
        double fuelCut = 0.0;

        double idleIntegrator = 0.0;
        double torqueCorrection = 0.0;
        double fuelTrim = 1.0;
        int shiftIterations = 0;
        double shiftErrorNorm = 0.0;
        bool adaptionEnabled = false;

        int selectedMode = -1;
        std::string engineState;
        std::string shiftState;
        std::string shiftBlock;
    };

    struct ParameterCommand {
        enum class Kind {
            SetParameter,
            SelectMode,
            ResetDefaults,
            SetAdaptive,
            SelectChannels,
            ScopeWindow,
            ScopeMode,
            ScopeArm,
            StartManoeuvre,
            StopManoeuvre,
            StartRecording,
            StopRecording
        };

        Kind kind = Kind::SetParameter;
        std::string path;
        double value = 0.0;
        std::vector<std::string> names;
    };

    class ShiftRecorder;
    class ChannelRecorder;
    class ChannelTable;

    class ConfigServer {
        public:
            struct Parameters {
                std::string host = "127.0.0.1";
                int port = 8420;
                std::string uiPath = "../assets/config_ui/index.html";
            };

        public:
            ConfigServer();
            ~ConfigServer();

            void initialize(
                const Parameters &params,
                ParameterRegistry *registry,
                DriveModeSet *modes);

            bool start();
            void stop();
            bool isRunning() const;
            int getBoundPort() const;

            void publish(const TelemetrySample &sample);
            void publishShifts(const ShiftRecorder &recorder);
            void publishScope(const ChannelRecorder &recorder, const ChannelTable &table);
            void setScope(ChannelRecorder *recorder);
            void setPowertrain(PowertrainSystem *system);
            void publishRecording(bool recording, int samples, int dropped, double elapsed, const std::string &script);
            int applyPendingCommands();

            std::string schemaJson() const;
            std::string stateJson() const;
            std::string exportScript() const;
            std::string exportOverrides() const;

            bool queueCommand(const ParameterCommand &command);

        protected:
            void buildSchema();
            void registerReadRoutes(void *handle);
            void registerSessionRoutes(void *handle);
            void registerWriteRoutes(void *handle);
            void writeValues(std::ostream &out) const;
            void writeAdaptiveFlags(std::ostream &out) const;
            void writeMaps(std::ostream &out) const;
            void writeTelemetry(std::ostream &out, const TelemetrySample &sample) const;
            void refreshExport();
            void refreshState(const TelemetrySample &sample);

            Parameters m_params;
            ParameterRegistry *m_registry;
            DriveModeSet *m_modes;

            mutable std::mutex m_mutex;
            std::string m_schema;
            std::string m_state;
            std::string m_export;
            std::string m_overrides;
            std::string m_shifts;
            std::string m_scope;
            std::string m_channelNames;
            ChannelRecorder *m_scopeRecorder = nullptr;
            PowertrainSystem *m_powertrain = nullptr;
            std::string m_recording = "{\"recording\":false,\"samples\":0,\"dropped\":0,\"elapsed\":0,\"script\":\"\"}";

            std::vector<ParameterCommand> m_commands;

            std::thread m_thread;
            std::atomic<bool> m_running;
            std::atomic<int> m_boundPort;

            void *m_server;
    };

} /* namespace config */

#endif /* ATG_ENGINE_SIM_CONFIG_SERVER_H */
