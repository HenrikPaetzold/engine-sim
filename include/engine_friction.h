#ifndef ATG_ENGINE_SIM_ENGINE_FRICTION_H
#define ATG_ENGINE_SIM_ENGINE_FRICTION_H

#include "units.h"

namespace config {
    class ParameterRegistry;
}

class EngineFriction {
    public:
        struct Parameters {
            double constantFmep = 0.0;
            double peakPressureFactor = 0.0;
            double speedFactor = 0.0;
            double speedSquaredFactor = 0.0;

            double vogelK = 0.123;
            double vogelB = 0.0;
            double vogelTheta = 160.0;
            double referenceTemperature = units::celcius(100.0);

            double peakPressureDecay = 0.5;
            double frictionHeatToOil = 1.0;
        };

    public:
        EngineFriction();
        ~EngineFriction();

        void initialize(const Parameters &params);
        void registerParameters(config::ParameterRegistry *registry);
        void reset();

        double viscosity(double oilTemperature) const;
        double viscosityRatio(double oilTemperature) const;

        void updatePeakPressure(double dt, double cylinderPressure);
        double meanPistonSpeed(double crankshaftSpeed, double stroke) const;
        double fmep(double crankshaftSpeed, double stroke, double viscosityRatio) const;
        double crankTorque(
            double crankshaftSpeed,
            double stroke,
            double displacement,
            double viscosityRatio) const;

        inline double getPeakPressure() const { return m_peakPressure; }
        inline const Parameters &getParameters() const { return m_params; }
        inline Parameters &getParameters() { return m_params; }

    protected:
        Parameters m_params;

        double m_peakPressure;
};

#endif /* ATG_ENGINE_SIM_ENGINE_FRICTION_H */
