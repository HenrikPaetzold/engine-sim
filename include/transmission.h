#ifndef ATG_ENGINE_SIM_TRANSMISSION_H
#define ATG_ENGINE_SIM_TRANSMISSION_H

#include "vehicle.h"
#include "engine.h"
#include "ratio_clutch_constraint.h"
#include "torque_converter_constraint.h"
#include "scs.h"

namespace config {
    class ParameterRegistry;
}

class Transmission {
    public:
        enum class Type {
            Legacy,
            Manual,
            DualClutch,
            Converter
        };

        struct Parameters {
            int GearCount;
            const double *GearRatios;
            double MaxClutchTorque;

            Type GearboxType = Type::Legacy;
            double TurbineInertia = 0.08;
            double StallTorqueRatio = 2.0;
            double CouplingPoint = 0.85;
            double CapacityFactor = 4.04e-3;
        };

        static constexpr int ClutchCount = 2;

    public:
        Transmission();
        ~Transmission();

        void initialize(const Parameters &params);
        void registerParameters(config::ParameterRegistry *registry, const char *prefix);
        void update(double dt);
        void bind(
            atg_scs::RigidBody *rotatingMass,
            Vehicle *vehicle,
            Engine *engine);
        void addToSystem(
            atg_scs::RigidBodySystem *system,
            atg_scs::RigidBody *rotatingMass,
            Vehicle *vehicle,
            Engine *engine);
        void changeGear(int newGear);
        inline int getGear() const { return m_gear; }
        inline int getGearCount() const { return m_gearCount; }
        double getGearRatio(int gear) const;
        inline const double *getGearRatios() const { return m_gearRatios; }

        inline void setClutchPressure(double pressure) { m_clutchPressure[0] = pressure; }
        inline double getClutchPressure() const { return m_clutchPressure[0]; }
        void setClutchPressure(int index, double pressure);
        double getClutchPressure(int index) const;

        void setPreselectedGear(int gear);
        inline int getPreselectedGear() const { return m_preselectedGear; }
        inline void setLockupPressure(double pressure) { m_lockupPressure = pressure; }
        inline double getLockupPressure() const { return m_lockupPressure; }

        inline Type getType() const { return m_type; }
        inline bool supportsPreselect() const { return m_type == Type::DualClutch; }
        inline bool hasLaunchDevice() const { return m_type == Type::Converter; }
        bool requiresTorqueInterrupt() const;

        double getInputSpeed() const;
        double getOutputSpeed() const;
        double getClutchSlipSpeed() const;
        double getClutchSlipSpeed(int index) const;
        double getTurbineSpeed() const;
        double getConverterSlip() const;
        double getDrivelineInertia() const;
        double getClutchRatio(int index) const;
        double getClutchCapacity(int index) const;
        double getClutchTorque(int index) const;
        inline atg_scs::RigidBody *getRotatingMass() const { return m_rotatingMass; }

    protected:
        double referenceRatio() const;
        void updateDrivelineInertia();
        void updateRatioClutches();

        atg_scs::ClutchConstraint m_clutchConstraint;
        RatioClutchConstraint m_ratioClutch[ClutchCount];
        RatioClutchConstraint m_lockupClutch;
        TorqueConverterConstraint m_converter;
        atg_scs::RigidBody m_turbine;

        atg_scs::RigidBody *m_rotatingMass;
        Vehicle *m_vehicle;
        Engine *m_engine;

        Type m_type;
        int m_gear;
        int m_preselectedGear;
        int m_gearCount;
        double *m_gearRatios;
        double m_maxClutchTorque;
        double m_clutchPressure[ClutchCount];
        double m_lockupPressure;
        double m_turbineInertia;
};

#endif /* ATG_ENGINE_SIM_TRANSMISSION_H */
