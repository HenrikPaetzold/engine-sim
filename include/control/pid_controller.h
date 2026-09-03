#ifndef ATG_ENGINE_SIM_PID_CONTROLLER_H
#define ATG_ENGINE_SIM_PID_CONTROLLER_H

#include <cfloat>

namespace control {

    class PidController {
        public:
            struct Parameters {
                double kp = 0.0;
                double ki = 0.0;
                double kd = 0.0;
                double outputMin = 0.0;
                double outputMax = 1.0;
                double derivativeCutoff = 0.0;
                double trackingGain = 1.0;
                double integratorLimit = DBL_MAX;
            };

        public:
            PidController();
            ~PidController();

            void initialize(const Parameters &params);
            void reset();

            double update(
                double dt,
                double setpoint,
                double measurement,
                double feedforward = 0.0);

            void setIntegrator(double value);

            inline const Parameters &getParameters() const { return m_params; }
            inline Parameters &getParametersMutable() { return m_params; }
            inline void setParameters(const Parameters &params) { m_params = params; }

            inline double getIntegrator() const { return m_integrator; }
            inline double getProportionalTerm() const { return m_proportional; }
            inline double getDerivativeTerm() const { return m_derivative; }
            inline double getError() const { return m_error; }
            inline double getOutput() const { return m_output; }
            inline bool isSaturated() const { return m_saturated; }

        protected:
            Parameters m_params;

            double m_integrator;
            double m_filteredDerivative;
            double m_previousMeasurement;
            bool m_primed;

            double m_proportional;
            double m_derivative;
            double m_error;
            double m_output;
            bool m_saturated;
    };

} /* namespace control */

#endif /* ATG_ENGINE_SIM_PID_CONTROLLER_H */
