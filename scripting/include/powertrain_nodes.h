#ifndef ATG_ENGINE_SIM_POWERTRAIN_NODES_H
#define ATG_ENGINE_SIM_POWERTRAIN_NODES_H

#include "object_reference_node.h"
#include "control_nodes.h"

#include "engine_sim.h"

#include <vector>

namespace es_script {

    class EngineControlUnitNode : public ObjectReferenceNode<EngineControlUnitNode> {
    public:
        EngineControlUnitNode() { /* void */ }
        virtual ~EngineControlUnitNode() { /* void */ }

        void generate(powertrain::EngineControlUnit *ecu) const {
            powertrain::EngineControlUnit::Parameters parameters = m_parameters;

            if (m_idleController != nullptr) {
                parameters.idleController = m_idleController->getParameters();
            }
            if (m_torqueController != nullptr) {
                parameters.torqueController = m_torqueController->getParameters();
            }

            ecu->initialize(parameters);

            if (m_throttleMap != nullptr && !m_throttleMap->isEmpty()) {
                m_throttleMap->generate(&ecu->getThrottleMap());
            }
            if (m_maxTorqueMap != nullptr && !m_maxTorqueMap->isEmpty()) {
                m_maxTorqueMap->generate(&ecu->getMaxTorqueMap());
            }
            if (m_pedalMap != nullptr && !m_pedalMap->isEmpty()) {
                m_pedalMap->generate(&ecu->getPedalMap());
            }
        }

    protected:
        virtual void registerInputs() override {
            addInput("reference_torque", &m_parameters.referenceTorque);
            addInput("torque_rise_rate", &m_parameters.torqueRiseRate);
            addInput("torque_fall_rate", &m_parameters.torqueFallRate);
            addInput("idle_speed_cold", &m_parameters.idleSpeedCold);
            addInput("idle_speed_warm", &m_parameters.idleSpeedWarm);
            addInput("cold_temperature", &m_parameters.coldTemperature);
            addInput("warm_temperature", &m_parameters.warmTemperature);
            addInput("cold_start_enrichment", &m_parameters.coldStartEnrichment);
            addInput("cold_start_timing_retard", &m_parameters.coldStartTimingRetard);
            addInput("cold_start_torque_cap", &m_parameters.coldStartTorqueCap);
            addInput("rev_limit", &m_parameters.revLimit);
            addInput("rev_limit_cold", &m_parameters.revLimitCold);
            addInput("soft_limit_band", &m_parameters.softLimitBand);
            addInput("hard_limit_offset", &m_parameters.hardLimitOffset);
            addInput("overrun_cut_speed", &m_parameters.overrunCutSpeed);
            addInput("overrun_resume_speed", &m_parameters.overrunResumeSpeed);
            addInput("cranking_speed", &m_parameters.crankingSpeed);

            addInput("idle_controller", &m_idleController, InputTarget::Type::Object);
            addInput("torque_controller", &m_torqueController, InputTarget::Type::Object);
            addInput("throttle_map", &m_throttleMap, InputTarget::Type::Object);
            addInput("max_torque_map", &m_maxTorqueMap, InputTarget::Type::Object);
            addInput("pedal_map", &m_pedalMap, InputTarget::Type::Object);

            ObjectReferenceNode<EngineControlUnitNode>::registerInputs();
        }

        virtual void _evaluate() override {
            setOutput(this);
            readAllInputs();
        }

        powertrain::EngineControlUnit::Parameters m_parameters;

        PidControllerNode *m_idleController = nullptr;
        PidControllerNode *m_torqueController = nullptr;
        Map2dNode *m_throttleMap = nullptr;
        Map2dNode *m_maxTorqueMap = nullptr;
        Map2dNode *m_pedalMap = nullptr;
    };

    class TransmissionControlUnitNode
        : public ObjectReferenceNode<TransmissionControlUnitNode>
    {
    public:
        TransmissionControlUnitNode() { /* void */ }
        virtual ~TransmissionControlUnitNode() { /* void */ }

        void addGatePosition(const powertrain::GatePosition &position) {
            m_gate.add(position);
        }

        void addGear(double ratio) {
            if (m_gears.size() < powertrain::MaxGears) m_gears.push_back(ratio);
        }

        void generate(powertrain::TransmissionControlUnit *tcu) const {
            powertrain::TransmissionControlUnit::Parameters parameters = m_parameters;

            if (!m_gears.empty()) {
                parameters.gearCount = static_cast<int>(m_gears.size());
                for (size_t i = 0; i < m_gears.size(); ++i) {
                    parameters.gearRatios[i] = m_gears[i];
                }
            }

            if (m_slipController != nullptr) {
                parameters.slipController = m_slipController->getParameters();
            }
            if (m_lockupController != nullptr) {
                parameters.lockupController = m_lockupController->getParameters();
            }

            tcu->initialize(parameters);

            if (!m_gate.isEmpty()) {
                tcu->getGate() = m_gate;
                tcu->reset();
            }

            const bool upshift = m_upshiftMap != nullptr && !m_upshiftMap->isEmpty();
            const bool downshift = m_downshiftMap != nullptr && !m_downshiftMap->isEmpty();
            const bool lockup = m_lockupMap != nullptr && !m_lockupMap->isEmpty();

            if (upshift) m_upshiftMap->generate(&tcu->getUpshiftMap());
            if (downshift) m_downshiftMap->generate(&tcu->getDownshiftMap());
            if (lockup) m_lockupMap->generate(&tcu->getLockupMap());

            tcu->markAuthoredMaps(upshift, downshift, lockup);
        }

    protected:
        virtual void registerInputs() override {
            addInput("final_drive", &m_parameters.finalDrive);
            addInput("tire_radius", &m_parameters.tireRadius);
            addInput("torque_interrupt", &m_parameters.requiresTorqueInterrupt);
            addInput("preselect", &m_parameters.supportsPreselect);
            addInput("launch_device", &m_parameters.hasLaunchDevice);
            addInput("driver_clutch", &m_parameters.driverClutchAuthority);
            addInput("torque_reduction", &m_parameters.shiftTorqueReduction);
            addInput("torque_cut", &m_parameters.shiftTorqueCut);
            addInput("overlap_hold", &m_parameters.overlapHold);
            addInput("speed_match_tolerance", &m_parameters.speedMatchTolerance);
            addInput("lockup_slip_target", &m_parameters.lockupSlipTarget);
            addInput("lockup_lock_slip", &m_parameters.lockupLockSlip);
            addInput("lockup_apply_rate", &m_parameters.lockupApplyRate);
            addInput("torque_reduction_time", &m_parameters.torqueReductionTime);
            addInput("clutch_release_time", &m_parameters.clutchReleaseTime);
            addInput("gear_change_time", &m_parameters.gearChangeTime);
            addInput("speed_match_time", &m_parameters.speedMatchTime);
            addInput("clutch_overlap_time", &m_parameters.clutchOverlapTime);
            addInput("clutch_engage_time", &m_parameters.clutchEngageTime);
            addInput("min_gear_time", &m_parameters.minGearTime);
            addInput("kickdown_threshold", &m_parameters.kickdownThreshold);
            addInput("launch_slip_target", &m_parameters.launchSlipTarget);
            addInput("launch_lock_slip", &m_parameters.launchLockSlip);
            addInput("stall_protect_speed", &m_parameters.stallProtectSpeed);
            addInput("brake_interlock", &m_parameters.brakeInterlock);
            addInput("default_position", &m_parameters.defaultPosition);

            addInput("slip_controller", &m_slipController, InputTarget::Type::Object);
            addInput("upshift_map", &m_upshiftMap, InputTarget::Type::Object);
            addInput("downshift_map", &m_downshiftMap, InputTarget::Type::Object);
            addInput("lockup_map", &m_lockupMap, InputTarget::Type::Object);
            addInput("lockup_controller", &m_lockupController, InputTarget::Type::Object);

            ObjectReferenceNode<TransmissionControlUnitNode>::registerInputs();
        }

        virtual void _evaluate() override {
            setOutput(this);
            readAllInputs();
        }

        powertrain::TransmissionControlUnit::Parameters m_parameters;
        std::vector<double> m_gears;
        powertrain::SelectorGate m_gate;

        PidControllerNode *m_slipController = nullptr;
        Map2dNode *m_upshiftMap = nullptr;
        Map2dNode *m_downshiftMap = nullptr;
        Map2dNode *m_lockupMap = nullptr;
        PidControllerNode *m_lockupController = nullptr;
    };

    class GatePositionNode : public ObjectReferenceNode<GatePositionNode> {
    public:
        GatePositionNode() { /* void */ }
        virtual ~GatePositionNode() { /* void */ }

        powertrain::GatePosition generate() const {
            powertrain::GatePosition position;
            position.name = m_name;
            position.engagement =
                powertrain::engagementFromName(m_engagement.c_str());
            position.maxEntrySpeed = m_maxEntrySpeed;
            position.maxExitSpeed = m_maxExitSpeed;
            position.requiresBrake = m_requiresBrake;
            position.mode = m_mode;

            return position;
        }

    protected:
        virtual void registerInputs() override {
            addInput("name", &m_name);
            addInput("engagement", &m_engagement);
            addInput("max_entry_speed", &m_maxEntrySpeed);
            addInput("max_exit_speed", &m_maxExitSpeed);
            addInput("requires_brake", &m_requiresBrake);
            addInput("mode", &m_mode);

            ObjectReferenceNode<GatePositionNode>::registerInputs();
        }

        virtual void _evaluate() override {
            setOutput(this);
            readAllInputs();
        }

        std::string m_name;
        std::string m_engagement = "neutral";
        std::string m_mode;
        double m_maxEntrySpeed = -1.0;
        double m_maxExitSpeed = -1.0;
        bool m_requiresBrake = false;
    };

    class AdaptationNode : public ObjectReferenceNode<AdaptationNode> {
    public:
        AdaptationNode() { /* void */ }
        virtual ~AdaptationNode() { /* void */ }

        const adaptation::AdaptationManager::Parameters &getParameters() const {
            return m_parameters;
        }

    protected:
        virtual void registerInputs() override {
            addInput("throttle_map", &m_parameters.throttleMapEnabled);
            addInput("idle", &m_parameters.idleEnabled);
            addInput("lambda", &m_parameters.lambdaEnabled);
            addInput("shift", &m_parameters.shiftEnabled);
            addInput("throttle_rate", &m_parameters.throttleLearningRate);
            addInput("throttle_deadband", &m_parameters.throttleDeadband);
            addInput("idle_drain_rate", &m_parameters.idleDrainRate);
            addInput("idle_limit", &m_parameters.idleTrimLimit);
            addInput("lambda_gain", &m_parameters.lambdaShortTermGain);
            addInput("lambda_limit", &m_parameters.lambdaTrimLimit);
            addInput("lambda_target", &m_parameters.lambdaTarget);
            addInput("warm_temperature", &m_parameters.conditions.warmTemperature);
            addInput("speed_window", &m_parameters.conditions.speedStabilityWindow);

            ObjectReferenceNode<AdaptationNode>::registerInputs();
        }

        virtual void _evaluate() override {
            setOutput(this);
            readAllInputs();
        }

        adaptation::AdaptationManager::Parameters m_parameters;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_POWERTRAIN_NODES_H */
