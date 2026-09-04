#ifndef ATG_ENGINE_SIM_CONTROL_PROGRAM_NODES_H
#define ATG_ENGINE_SIM_CONTROL_PROGRAM_NODES_H

#include "object_reference_node.h"
#include "control_nodes.h"
#include "function_node.h"

#include "engine_sim.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace es_script {

    class ControlBlockNode : public ObjectReferenceNode<ControlBlockNode> {
    public:
        using EmitCache = std::unordered_map<const ControlBlockNode *, int>;

    public:
        ControlBlockNode() { /* void */ }
        virtual ~ControlBlockNode() { /* void */ }

        void addOperandNode(ControlBlockNode *node) {
            m_operandNodes.push_back(node);
        }

        int emit(powertrain::ScriptedControlUnit *unit, EmitCache *cache) const {
            if (m_kind == "none") return -1;

            EmitCache::const_iterator known = cache->find(this);
            if (known != cache->end()) return known->second;

            control::ControlProgram &program = unit->getProgram();

            control::ControlBlock *block = create(unit);
            if (block == nullptr) return -1;

            block->m_name = m_blockName;
            block->m_adaptive = m_adaptive;
            block->m_adaptMin = m_adaptMin;
            block->m_adaptMax = m_adaptMax;

            const int index = program.addBlock(block);
            (*cache)[this] = index;

            for (ControlBlockNode *node : m_operandNodes) {
                block->addOperand((node == nullptr) ? -1 : node->emit(unit, cache));
            }

            return index;
        }

    protected:
        control::ControlBlock *create(powertrain::ScriptedControlUnit *unit) const {
            if (m_kind == "constant") {
                control::ConstantBlock *block = new control::ConstantBlock;
                block->m_value = m_value;
                return block;
            }
            else if (m_kind == "signal") {
                control::SignalBlock *block = new control::SignalBlock;
                block->m_signal = unit->getProgram().getInputs().find(m_channel);
                block->m_scale = m_scale;
                return block;
            }
            else if (m_kind == "sum") {
                return new control::SumBlock;
            }
            else if (m_kind == "product") {
                return new control::ProductBlock;
            }
            else if (m_kind == "min") {
                return new control::MinBlock;
            }
            else if (m_kind == "max") {
                return new control::MaxBlock;
            }
            else if (m_kind == "gain") {
                control::GainBlock *block = new control::GainBlock;
                block->m_gain = m_gain;
                block->m_offset = m_offset;
                return block;
            }
            else if (m_kind == "clamp") {
                control::ClampBlock *block = new control::ClampBlock;
                block->m_min = m_min;
                block->m_max = m_max;
                return block;
            }
            else if (m_kind == "curve") {
                control::CurveBlock *block = new control::CurveBlock;
                if (m_curve != nullptr) block->setCurve(m_curve->createFunction(), true);
                return block;
            }
            else if (m_kind == "map") {
                control::MapBlock *block = new control::MapBlock;
                if (m_map != nullptr && !m_map->isEmpty()) m_map->generate(&block->m_map);
                return block;
            }
            else if (m_kind == "pid") {
                control::PidBlock *block = new control::PidBlock;
                if (m_controller != nullptr) {
                    block->m_controller.initialize(m_controller->getParameters());
                }
                return block;
            }
            else if (m_kind == "rate_limit") {
                control::RateLimitBlock *block = new control::RateLimitBlock;
                block->m_riseRate = m_riseRate;
                block->m_fallRate = m_fallRate;
                return block;
            }
            else if (m_kind == "low_pass") {
                control::LowPassBlock *block = new control::LowPassBlock;
                block->m_timeConstant = m_timeConstant;
                return block;
            }
            else if (m_kind == "select") {
                control::SelectBlock *block = new control::SelectBlock;
                block->m_threshold = m_threshold;
                return block;
            }
            else if (m_kind == "compare") {
                control::CompareBlock *block = new control::CompareBlock;
                block->m_band = m_band;
                return block;
            }
            else if (m_kind == "latch") {
                control::LatchBlock *block = new control::LatchBlock;
                block->m_threshold = m_threshold;
                return block;
            }
            else if (m_kind == "integrator") {
                control::IntegratorBlock *block = new control::IntegratorBlock;
                block->m_min = m_min;
                block->m_max = m_max;
                block->m_initial = m_initial;
                return block;
            }
            else if (m_kind == "timer") {
                control::TimerBlock *block = new control::TimerBlock;
                block->m_threshold = m_threshold;
                return block;
            }
            else if (m_kind == "delay") {
                control::DelayBlock *block = new control::DelayBlock;
                block->m_initial = m_initial;
                return block;
            }
            else if (m_kind == "learner") {
                control::LearnerBlock *block = new control::LearnerBlock;
                block->m_target = m_target;
                block->m_rate = m_rate;
                block->m_threshold = m_threshold;
                return block;
            }
            else if (m_kind == "actuator") {
                control::ActuatorBlock *block = new control::ActuatorBlock;
                block->m_actuator = unit->getProgram().getOutputs().find(m_channel);
                return block;
            }

            return nullptr;
        }

        virtual void registerInputs() override {
            addInput("kind", &m_kind);
            addInput("name", &m_blockName);
            addInput("channel", &m_channel);

            addInput("value", &m_value);
            addInput("scale", &m_scale);
            addInput("gain", &m_gain);
            addInput("offset", &m_offset);
            addInput("min", &m_min);
            addInput("max", &m_max);
            addInput("initial", &m_initial);
            addInput("threshold", &m_threshold);
            addInput("band", &m_band);
            addInput("rise", &m_riseRate);
            addInput("fall", &m_fallRate);
            addInput("tau", &m_timeConstant);
            addInput("adaptive", &m_adaptive);
            addInput("adapt_min", &m_adaptMin);
            addInput("adapt_max", &m_adaptMax);
            addInput("target", &m_target);
            addInput("rate", &m_rate);

            addInput("curve", &m_curve, InputTarget::Type::Object);
            addInput("map", &m_map, InputTarget::Type::Object);
            addInput("controller", &m_controller, InputTarget::Type::Object);

            ObjectReferenceNode<ControlBlockNode>::registerInputs();
        }

        virtual void _evaluate() override {
            setOutput(this);
            readAllInputs();
        }

        std::string m_kind;
        std::string m_blockName;
        std::string m_channel;
        std::string m_target;

        std::vector<ControlBlockNode *> m_operandNodes;

        double m_value = 0.0;
        double m_scale = 1.0;
        double m_gain = 1.0;
        double m_offset = 0.0;
        double m_min = 0.0;
        double m_max = 1.0;
        double m_initial = 0.0;
        double m_threshold = 0.5;
        double m_band = 0.0;
        double m_riseRate = 0.0;
        double m_fallRate = 0.0;
        double m_timeConstant = 0.1;
        double m_rate = 0.0;
        double m_adaptMin = 0.0;
        double m_adaptMax = 0.0;
        bool m_adaptive = false;

        FunctionNode *m_curve = nullptr;
        Map2dNode *m_map = nullptr;
        PidControllerNode *m_controller = nullptr;
    };

    class ControlProgramNode : public ObjectReferenceNode<ControlProgramNode> {
    public:
        ControlProgramNode() { /* void */ }
        virtual ~ControlProgramNode() { /* void */ }

        void addOutput(ControlBlockNode *node) {
            if (node != nullptr) m_outputs.push_back(node);
        }

        bool generate(powertrain::ScriptedControlUnit *unit) const {
            ControlBlockNode::EmitCache cache;
            for (ControlBlockNode *node : m_outputs) {
                node->emit(unit, &cache);
            }

            return unit->getProgram().compile();
        }

    protected:
        virtual void registerInputs() override {
            ObjectReferenceNode<ControlProgramNode>::registerInputs();
        }

        virtual void _evaluate() override {
            setOutput(this);
            readAllInputs();
        }

        std::vector<ControlBlockNode *> m_outputs;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_CONTROL_PROGRAM_NODES_H */
