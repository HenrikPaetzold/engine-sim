#ifndef ATG_ENGINE_SIM_RLS_ESTIMATOR_H
#define ATG_ENGINE_SIM_RLS_ESTIMATOR_H

namespace adaptation {

    class RlsEstimator {
        public:
            struct Parameters {
                double initialEstimate = 1.0;
                double initialCovariance = 100.0;
                double forgettingFactor = 0.999;
                double covarianceLimit = 1e6;
                double minimumRegressor = 1e-6;
                double estimateMin = 0.0;
                double estimateMax = 10.0;
            };

        public:
            RlsEstimator();
            ~RlsEstimator();

            void initialize(const Parameters &params);
            void reset();

            double update(double regressor, double observation);

            inline double getEstimate() const { return m_estimate; }
            inline double getCovariance() const { return m_covariance; }
            inline double getResidual() const { return m_residual; }

        protected:
            Parameters m_params;

            double m_estimate;
            double m_covariance;
            double m_residual;
    };

} /* namespace adaptation */

#endif /* ATG_ENGINE_SIM_RLS_ESTIMATOR_H */
