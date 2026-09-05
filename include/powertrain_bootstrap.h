#ifndef ATG_ENGINE_SIM_POWERTRAIN_BOOTSTRAP_H
#define ATG_ENGINE_SIM_POWERTRAIN_BOOTSTRAP_H

#include "powertrain/powertrain_unit.h"
#include "powertrain/scripted_control_unit.h"
#include "adaptation/adaptation_manager.h"
#include "config/drive_mode.h"
#include "config/parameter_registry.h"

#include <string>
#include <utility>
#include <vector>

class Simulator;
class PowertrainSystem;

namespace config {
    class ConfigServer;
}

namespace powertrain {

    struct BootstrapInputs {
        PowertrainUnit *unit = nullptr;
        ScriptedControlUnit *program = nullptr;
        adaptation::AdaptationManager::Parameters adaptation;
        std::string defaultMode;
        std::vector<std::pair<std::string, double>> parameterOverrides;
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
