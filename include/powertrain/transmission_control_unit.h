#ifndef ATG_ENGINE_SIM_TRANSMISSION_CONTROL_UNIT_H
#define ATG_ENGINE_SIM_TRANSMISSION_CONTROL_UNIT_H

#include "powertrain_controller.h"

#include "../control/pid_controller.h"
#include "../control/map_2d.h"
#include "../control/hysteresis.h"
#include "../control/iterative_learning.h"
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
                double kickdownThreshold = 0.85;
                double speedMatchTolerance = units::rpm(120.0);

                double launchSlipTarget = units::rpm(1000.0);
                double launchLockSlip = units::rpm(60.0);
                double launchSpeed = units::distance(2.0, units::m);
                double stallProtectSpeed = units::rpm(650.0);

                control::PidController::Parameters slipController = defaultSlipController();
                control::IterativeLearningControl::Parameters engageProfile;
            };

            static control::PidController::Parameters defaultSlipController();

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

            inline ShiftState getShiftState() const { return m_shiftState; }
            inline int getTargetGear() const { return m_targetGear; }
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
            double launchPressure(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs);

            Parameters m_params;

            control::Map2d m_upshiftMap;
            control::Map2d m_downshiftMap;
            control::PidController m_slipController;
            control::IterativeLearningControl m_engageProfile;

            ShiftState m_shiftState;
            control::StateTimer m_shiftTimer;
            control::StateTimer m_gearTimer;

            int m_currentGear;
            int m_targetGear;
            int m_previousGear;

            double m_engagePhase;
            int m_completedShifts;

            double m_clutchPressure;
            double m_secondaryPressure;

            bool m_previousShiftUp;
            bool m_previousShiftDown;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_TRANSMISSION_CONTROL_UNIT_H */
