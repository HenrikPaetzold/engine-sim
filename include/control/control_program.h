#ifndef ATG_ENGINE_SIM_CONTROL_PROGRAM_H
#define ATG_ENGINE_SIM_CONTROL_PROGRAM_H

#include "control_block.h"
#include "signal_table.h"

#include <string>
#include <vector>

namespace control {

    class ControlProgram {
        public:
            ControlProgram();
            ~ControlProgram();

            void destroy();

            int addBlock(ControlBlock *block);
            inline int getBlockCount() const { return static_cast<int>(m_blocks.size()); }
            ControlBlock *getBlock(int index) const;
            ControlBlock *findBlock(const std::string &name) const;

            bool compile();
            inline bool isCompiled() const { return m_compiled; }
            inline const std::string &getError() const { return m_error; }

            void reset();
            void update(double dt);

            inline SignalTable &getInputs() { return m_inputs; }
            inline const SignalTable &getInputs() const { return m_inputs; }
            inline SignalTable &getOutputs() { return m_outputs; }
            inline const SignalTable &getOutputs() const { return m_outputs; }

            double getValue(int index) const;
            inline const std::vector<int> &getOrder() const { return m_order; }

        protected:
            bool sort();

            std::vector<ControlBlock *> m_blocks;
            std::vector<int> m_order;
            std::vector<double> m_values;

            SignalTable m_inputs;
            SignalTable m_outputs;

            std::string m_error;
            bool m_compiled;
    };

} /* namespace control */

#endif /* ATG_ENGINE_SIM_CONTROL_PROGRAM_H */
