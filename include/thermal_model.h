#ifndef ATG_ENGINE_SIM_THERMAL_MODEL_H
#define ATG_ENGINE_SIM_THERMAL_MODEL_H

#include "units.h"

namespace config {
    class ParameterRegistry;
}

class ThermalModel {
    public:
        struct Parameters {
            double blockThermalMass = 120000.0;
            double oilThermalMass = 40000.0;
            double blockToOilConductance = 60.0;
            double radiatorConductance = 900.0;
            double oilToAmbientConductance = 30.0;
            double speedCoolingCoefficient = 25.0;
            double thermostatOpenTemperature = units::celcius(85.0);
            double thermostatFullTemperature = units::celcius(100.0);
            double ambientTemperature = units::celcius(20.0);
            double combustionHeatFraction = 1.0;
        };

    public:
        ThermalModel();
        ~ThermalModel();

        void initialize(const Parameters &params);
        void registerParameters(config::ParameterRegistry *registry, const char *prefix);
        void reset();

        void addHeat(double energy);
        void update(double dt, double vehicleSpeed);

        double thermostatOpening() const;

        inline double getBlockTemperature() const { return m_blockTemperature; }
        inline double getOilTemperature() const { return m_oilTemperature; }
        inline void setBlockTemperature(double t) { m_blockTemperature = t; }
        inline void setOilTemperature(double t) { m_oilTemperature = t; }
        inline const Parameters &getParameters() const { return m_params; }
        inline Parameters &getParameters() { return m_params; }

    protected:
        Parameters m_params;

        double m_blockTemperature;
        double m_oilTemperature;
        double m_pendingHeat;
};

#endif /* ATG_ENGINE_SIM_THERMAL_MODEL_H */
