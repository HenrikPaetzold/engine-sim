#ifndef ATG_ENGINE_SIM_TRANSMISSION_CONTROL_UNIT_H
#define ATG_ENGINE_SIM_TRANSMISSION_CONTROL_UNIT_H

#include "powertrain_controller.h"
#include "selector_gate.h"

#include "../control/pid_controller.h"
#include "../control/map_2d.h"
#include "../control/hysteresis.h"
#include "../control/iterative_learning.h"
#include "../control/rate_limiter.h"
#include "../units.h"

namespace powertrain {

    static constexpr int MaxGears = 10;

    enum class ShiftState {
        Idle,
        TorqueReduction,
        ClutchRelease,
        GearChange,
        SpeedMatch,
        ClutchOverlap,
        ClutchEngage
    };

    class TransmissionControlUnit : public PowertrainController {
        public:
            struct Parameters {
                int gearCount = 6;
                double gearRatios[MaxGears] = {
                    3.60, 2.19, 1.41, 1.00, 0.83, 0.69, 0.0, 0.0, 0.0, 0.0 };
                double finalDrive = 3.42;
                double tireRadius = units::distance(12.0, units::inch);

                bool requiresTorqueInterrupt = true;
                bool supportsPreselect = false;
                bool hasLaunchDevice = false;
                bool driverClutchAuthority = false;

                double torqueReductionTime = 0.08;
                double clutchReleaseTime = 0.10;
                double gearChangeTime = 0.06;
                double speedMatchTime = 0.25;
                double clutchOverlapTime = 0.18;
                double clutchEngageTime = 0.30;
                double minGearTime = 0.80;

                double shiftTorqueReduction = 0.70;
                double shiftTorqueCut = 1.00;
                double overlapHold = 0.15;
                double kickdownThreshold = 0.85;
                double speedMatchTolerance = units::rpm(120.0);

                double launchSlipTarget = units::rpm(1000.0);
                double launchLockSlip = units::rpm(60.0);
                double launchSpeed = units::distance(2.0, units::m);
                double lockupSlipTarget = units::rpm(120.0);
                double lockupLockSlip = units::rpm(25.0);
                double lockupApplyRate = 1.5;
                control::PidController::Parameters lockupController =
                    defaultLockupController();
                double stallProtectSpeed = units::rpm(650.0);

                bool brakeInterlock = true;
                bool supportsEngagement = true;
                std::string defaultPosition;

                control::PidController::Parameters slipController = defaultSlipController();
                control::IterativeLearningControl::Parameters engageProfile;
            };

            static control::PidController::Parameters defaultSlipController();
            static control::PidController::Parameters defaultLockupController();

        public:
            TransmissionControlUnit();
            virtual ~TransmissionControlUnit();

            void initialize(const Parameters &params);

            virtual void registerParameters(config::ParameterRegistry *registry, const char *prefix);
            virtual void reset();
            virtual void configureGearbox(const GearboxCapabilities &capabilities);

            virtual void update(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs,
                ActuatorCommands *commands);

            inline control::Map2d &getUpshiftMap() { return m_upshiftMap; }
            inline control::IterativeLearningControl &getEngageProfile() { return m_engageProfile; }
            inline const control::IterativeLearningControl &getEngageProfile() const { return m_engageProfile; }
            inline double getEngagePhase() const { return m_engagePhase; }
            inline int getCompletedShiftCount() const { return m_completedShifts; }
            inline control::Map2d &getDownshiftMap() { return m_downshiftMap; }
            inline control::Map2d &getLockupMap() { return m_lockupMap; }

            inline ShiftState getShiftState() const { return m_shiftState; }
            inline SelectorGate &getGate() { return m_gate; }
            inline const SelectorGate &getGate() const { return m_gate; }
            inline int getGatePosition() const { return m_gateIndex; }
            const GatePosition &getPosition() const;
            inline GateEngagement getEngagement() const { return getPosition().engagement; }
            inline bool wasPositionRefused() const { return m_positionRefused; }
            inline const std::string &getRequestedMode() const { return m_requestedMode; }
            bool positionAllowed(
                int from,
                int to,
                const PowertrainState &state,
                const DriverInputs &inputs) const;
            inline int getTargetGear() const { return m_targetGear; }
            inline int getActiveClutch() const { return m_activeClutch; }
            int getClutchGear(int clutch) const;
            int clutchForGear(int gear) const;
            void beginShiftForTest(int gear) { beginShift(gear); }
            int preselectedNeighbour(
                const PowertrainState &state,
                const DriverInputs &inputs) const;
            inline bool isShifting() const { return m_shiftState != ShiftState::Idle; }
            inline const Parameters &getParameters() const { return m_params; }

            double engineSpeedForGear(int gear, double vehicleSpeed) const;
            int scheduleGear(
                int currentGear,
                double pedal,
                double vehicleSpeed) const;

        protected:
            void buildDefaultMaps();
            void beginShift(int gear);
            void advanceShift(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs,
                ActuatorCommands *commands);
            void resolvePosition(
                const PowertrainState &state,
                const DriverInputs &inputs);
            void updateClutchAssignment(
                const PowertrainState &state,
                const DriverInputs &inputs);
            double lockupPressure(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs);
            double launchPressure(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs);

            Parameters m_params;

            control::Map2d m_upshiftMap;
            control::Map2d m_downshiftMap;
            control::Map2d m_lockupMap;
            control::PidController m_slipController;
            control::PidController m_lockupController;
            control::RateLimiter m_lockupLimiter;
            double m_lockupPressure;
            control::IterativeLearningControl m_engageProfile;

            ShiftState m_shiftState;
            SelectorGate m_gate;
            int m_gateIndex;
            bool m_positionRefused;
            std::string m_requestedMode;
            control::StateTimer m_shiftTimer;
            control::StateTimer m_gearTimer;

            int m_currentGear;
            int m_targetGear;
            int m_previousGear;
            int m_clutchGear[MaxClutches];
            int m_activeClutch;

            double m_engagePhase;
            int m_completedShifts;

            double m_clutchPressure;
            double m_secondaryPressure;

            bool m_previousShiftUp;
            bool m_previousShiftDown;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_TRANSMISSION_CONTROL_UNIT_H */
