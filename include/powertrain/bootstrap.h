#ifndef ATG_ENGINE_SIM_POWERTRAIN_BOOTSTRAP_H
#define ATG_ENGINE_SIM_POWERTRAIN_BOOTSTRAP_H

#include "powertrain_unit.h"
#include "scripted_control_unit.h"
#include "../adaptation/adaptation_manager.h"
#include "../thermal_model.h"
#include "../config/drive_mode.h"
#include "../config/parameter_registry.h"

#include <string>
#include <utility>
#include <vector>

class Simulator;
class PowertrainSystem;

namespace config {
    class ConfigServer;
}

namespace powertrain {

    struct AdaptiveOverride {
        std::string path;
        bool adaptive = true;
        double adaptMin = 0.0;
        double adaptMax = 0.0;
    };

    struct BootstrapInputs {
        PowertrainUnit *unit = nullptr;
        ScriptedControlUnit *program = nullptr;
        adaptation::AdaptationManager::Parameters adaptation;
        ThermalModel::Parameters thermal;
        bool thermalAuthored = false;
        double driverPedalTimeConstant = -1.0;
        double driverClutchTimeConstant = -1.0;
        double driverClutchPedalRate = -1.0;
        std::string defaultMode;
        std::vector<std::pair<std::string, double>> parameterOverrides;
        std::vector<AdaptiveOverride> adaptiveOverrides;
    };

    struct BootstrapContext {
        PowertrainSystem *system = nullptr;
        Simulator *simulator = nullptr;
        config::ParameterRegistry *registry = nullptr;
        config::DriveModeSet *modes = nullptr;
        adaptation::AdaptationManager *adaptation = nullptr;
        config::ConfigServer *server = nullptr;
        std::string uiPath;
    };

    enum class ControlMode {
        ScriptOnly,
        ControlUnits,
        ScriptOverlay
    };

    struct ControllerSelection {
        ControlMode mode = ControlMode::ControlUnits;
        PowertrainController *primary = nullptr;
        PowertrainController *overlay = nullptr;
    };

    ControllerSelection selectControllers(
        PowertrainUnit *unit,
        ScriptedControlUnit *program);

    struct BootstrapResult {
        PowertrainController *controller = nullptr;
        PowertrainController *overlay = nullptr;
        bool adaptationAttached = false;
        bool serverStarted = false;
        int defaultModeIndex = -1;
    };

    BootstrapResult installPowertrain(
        const BootstrapInputs &inputs,
        const BootstrapContext &context);

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_POWERTRAIN_BOOTSTRAP_H */
