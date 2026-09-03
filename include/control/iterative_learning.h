#ifndef ATG_ENGINE_SIM_ITERATIVE_LEARNING_H
#define ATG_ENGINE_SIM_ITERATIVE_LEARNING_H

namespace control {

    class IterativeLearningControl {
        public:
            struct Parameters {
                int binCount = 8;
                double learningRate = 0.4;
                double smoothing = 0.25;
                double outputMin = -0.4;
                double outputMax = 0.4;
            };

        public:
            IterativeLearningControl();
            ~IterativeLearningControl();

            void initialize(const Parameters &params);
            void destroy();
            void reset();

            void beginIteration();
            void sample(double phase, double error);
            void endIteration();
            void discardIteration();

            double correction(double phase) const;

            inline int getBinCount() const { return m_params.binCount; }
            double getBin(int i) const;
            double getLastErrorNorm() const { return m_lastErrorNorm; }
            inline int getIterationCount() const { return m_iterationCount; }
            inline bool isRecording() const { return m_recording; }

        protected:
            int binOf(double phase) const;

            Parameters m_params;

            double *m_profile;
            double *m_errorSum;
            int *m_errorCount;

            double m_lastErrorNorm;
            int m_iterationCount;
            bool m_recording;
    };

} /* namespace control */

#endif /* ATG_ENGINE_SIM_ITERATIVE_LEARNING_H */
