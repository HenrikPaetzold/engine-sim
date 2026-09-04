#ifndef ATG_ENGINE_SIM_CONTROL_BLOCK_H
#define ATG_ENGINE_SIM_CONTROL_BLOCK_H

#include "pid_controller.h"
#include "rate_limiter.h"
#include "hysteresis.h"
#include "map_2d.h"

#include <string>
#include <vector>

class Function;

namespace config {
    class ParameterRegistry;
}

namespace control {

    class SignalTable;

    struct BlockContext {
        const double *values = nullptr;
        const SignalTable *inputs = nullptr;
        SignalTable *outputs = nullptr;
        config::ParameterRegistry *registry = nullptr;
        double dt = 0.0;
    };

    class ControlBlock {
        public:
            ControlBlock();
            virtual ~ControlBlock();

            virtual void reset();
            virtual double evaluate(const BlockContext &context) = 0;
            virtual void latch(const BlockContext &context);
            virtual bool breaksCycles() const;

            void addOperand(int index);
            inline int getOperandCount() const { return static_cast<int>(m_operands.size()); }
            inline int getOperand(int i) const { return m_operands[i]; }

            inline double operandValue(const BlockContext &context, int i, double fallback = 0.0) const {
                if (i >= static_cast<int>(m_operands.size())) return fallback;
                const int index = m_operands[i];
                if (index < 0) return fallback;

                return context.values[index];
            }

            std::string m_name;
            int m_index;
            double m_output;

            bool m_adaptive = false;
            double m_adaptMin = 0.0;
            double m_adaptMax = 0.0;

        protected:
            std::vector<int> m_operands;
    };

    class ConstantBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
            double m_value = 0.0;
    };

    class SignalBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
            int m_signal = -1;
            double m_scale = 1.0;
    };

    class SumBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
    };

    class ProductBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
    };

    class MinBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
    };

    class MaxBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
    };

    class GainBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
            double m_gain = 1.0;
            double m_offset = 0.0;
    };

    class ClampBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
            double m_min = 0.0;
            double m_max = 1.0;
    };

    class CurveBlock : public ControlBlock {
        public:
            virtual ~CurveBlock();
            virtual double evaluate(const BlockContext &context) override;

            void setCurve(Function *curve, bool owned);

            Function *m_curve = nullptr;
            bool m_ownsCurve = false;
    };

    class MapBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
            Map2d m_map;
    };

    class PidBlock : public ControlBlock {
        public:
            virtual void reset() override;
            virtual double evaluate(const BlockContext &context) override;

            PidController m_controller;
    };

    class RateLimitBlock : public ControlBlock {
        public:
            virtual void reset() override;
            virtual double evaluate(const BlockContext &context) override;

            RateLimiter m_limiter;
            double m_riseRate = 0.0;
            double m_fallRate = 0.0;
    };

    class LowPassBlock : public ControlBlock {
        public:
            virtual void reset() override;
            virtual double evaluate(const BlockContext &context) override;

            double m_timeConstant = 0.1;

        protected:
            double m_state = 0.0;
            bool m_primed = false;
    };

    class SelectBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
            double m_threshold = 0.5;
    };

    class CompareBlock : public ControlBlock {
        public:
            virtual void reset() override;
            virtual double evaluate(const BlockContext &context) override;

            Hysteresis m_hysteresis;
            double m_band = 0.0;
    };

    class LatchBlock : public ControlBlock {
        public:
            virtual void reset() override;
            virtual double evaluate(const BlockContext &context) override;

            double m_threshold = 0.5;

        protected:
            bool m_state = false;
    };

    class IntegratorBlock : public ControlBlock {
        public:
            virtual void reset() override;
            virtual double evaluate(const BlockContext &context) override;

            double m_min = -1e9;
            double m_max = 1e9;
            double m_initial = 0.0;

        protected:
            double m_value = 0.0;
    };

    class TimerBlock : public ControlBlock {
        public:
            virtual void reset() override;
            virtual double evaluate(const BlockContext &context) override;

            double m_threshold = 0.5;

        protected:
            double m_elapsed = 0.0;
    };

    class DelayBlock : public ControlBlock {
        public:
            virtual void reset() override;
            virtual double evaluate(const BlockContext &context) override;
            virtual void latch(const BlockContext &context) override;
            virtual bool breaksCycles() const override;

            double m_initial = 0.0;

        protected:
            double m_previous = 0.0;
    };

    class LearnerBlock : public ControlBlock {
        public:
            virtual void reset() override;
            virtual double evaluate(const BlockContext &context) override;

            std::string m_target;
            double m_rate = 0.0;
            double m_threshold = 0.5;

        protected:
            double m_value = 0.0;
    };

    class ActuatorBlock : public ControlBlock {
        public:
            virtual double evaluate(const BlockContext &context) override;
            int m_actuator = -1;
    };

} /* namespace control */

#endif /* ATG_ENGINE_SIM_CONTROL_BLOCK_H */
