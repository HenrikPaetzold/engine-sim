#ifndef ATG_ENGINE_SIM_POWERTRAIN_ACTIONS_H
#define ATG_ENGINE_SIM_POWERTRAIN_ACTIONS_H

#include "node.h"
#include "compiler.h"
#include "powertrain_nodes.h"
#include "control_nodes.h"
#include "control_program_nodes.h"

namespace es_script {

    class SetPowertrainNode : public Node {
    public:
        SetPowertrainNode() { /* void */ }
        virtual ~SetPowertrainNode() { /* void */ }

    protected:
        virtual void registerInputs() override {
            addInput("ecu", &m_ecu, InputTarget::Type::Object);
            addInput("tcu", &m_tcu, InputTarget::Type::Object);
            addInput("adaptation", &m_adaptation, InputTarget::Type::Object);
            addInput("default_mode", &m_defaultMode);

            Node::registerInputs();
        }

        virtual void _evaluate() override {
            readAllInputs();

            powertrain::PowertrainUnit *unit = new powertrain::PowertrainUnit;

            if (m_ecu != nullptr) {
                m_ecu->generate(&unit->getEngineControlUnit());
            }
            else {
                unit->getEngineControlUnit().initialize(
                    powertrain::EngineControlUnit::Parameters());
            }

            if (m_tcu != nullptr) {
                m_tcu->generate(&unit->getTransmissionControlUnit());
            }
            else {
                unit->getTransmissionControlUnit().initialize(
                    powertrain::TransmissionControlUnit::Parameters());
            }

            if (m_adaptation != nullptr) {
                Compiler::output()->adaptation = m_adaptation->getParameters();
            }

            delete Compiler::output()->powertrain;
            Compiler::output()->powertrain = unit;
            Compiler::output()->defaultMode = m_defaultMode;
        }

        EngineControlUnitNode *m_ecu = nullptr;
        TransmissionControlUnitNode *m_tcu = nullptr;
        AdaptationNode *m_adaptation = nullptr;
        std::string m_defaultMode;
    };

    class AddGearRatioNode : public Node {
    public:
        AddGearRatioNode() { /* void */ }
        virtual ~AddGearRatioNode() { /* void */ }

    protected:
        virtual void registerInputs() override {
            addInput("tcu", &m_tcu, InputTarget::Type::Object);
            addInput("ratio", &m_ratio);

            Node::registerInputs();
        }

        virtual void _evaluate() override {
            readAllInputs();

            if (m_tcu != nullptr) m_tcu->addGear(m_ratio);
        }

        TransmissionControlUnitNode *m_tcu = nullptr;
        double m_ratio = 1.0;
    };

    class AddGatePositionNode : public Node {
    public:
        AddGatePositionNode() { /* void */ }
        virtual ~AddGatePositionNode() { /* void */ }

    protected:
        virtual void registerInputs() override {
            addInput("tcu", &m_tcu, InputTarget::Type::Object);
            addInput("position", &m_position, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() override {
            readAllInputs();

            if (m_tcu != nullptr && m_position != nullptr) {
                m_tcu->addGatePosition(m_position->generate());
            }
        }

        TransmissionControlUnitNode *m_tcu = nullptr;
        GatePositionNode *m_position = nullptr;
    };

    class AddMapSampleNode : public Node {
    public:
        AddMapSampleNode() { /* void */ }
        virtual ~AddMapSampleNode() { /* void */ }

    protected:
        virtual void registerInputs() override {
            addInput("map", &m_map, InputTarget::Type::Object);
            addInput("x", &m_x);
            addInput("y", &m_y);
            addInput("value", &m_value);

            Node::registerInputs();
        }

        virtual void _evaluate() override {
            readAllInputs();

            if (m_map != nullptr) m_map->addSample(m_x, m_y, m_value);
        }

        Map2dNode *m_map = nullptr;
        double m_x = 0.0;
        double m_y = 0.0;
        double m_value = 0.0;
    };

    class SetDriveModeValueNode : public Node {
    public:
        SetDriveModeValueNode() { /* void */ }
        virtual ~SetDriveModeValueNode() { /* void */ }

    protected:
        virtual void registerInputs() override {
            addInput("mode", &m_mode, InputTarget::Type::Object);
            addInput("path", &m_path);
            addInput("value", &m_value);

            Node::registerInputs();
        }

        virtual void _evaluate() override {
            readAllInputs();

            if (m_mode != nullptr) m_mode->set(m_path, m_value);
        }

        DriveModeNode *m_mode = nullptr;
        std::string m_path;
        double m_value = 0.0;
    };

    class AddDriveModeNode : public Node {
    public:
        AddDriveModeNode() { /* void */ }
        virtual ~AddDriveModeNode() { /* void */ }

    protected:
        virtual void registerInputs() override {
            addInput("mode", &m_mode, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() override {
            readAllInputs();

            if (m_mode != nullptr) {
                Compiler::output()->driveModes.add(m_mode->generate());
            }
        }

        DriveModeNode *m_mode = nullptr;
    };

    class AddControlOperandNode : public Node {
    public:
        AddControlOperandNode() { /* void */ }
        virtual ~AddControlOperandNode() { /* void */ }

    protected:
        virtual void registerInputs() override {
            addInput("block", &m_block, InputTarget::Type::Object);
            addInput("source", &m_input, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() override {
            readAllInputs();

            if (m_block != nullptr) m_block->addOperandNode(m_input);
        }

        ControlBlockNode *m_block = nullptr;
        ControlBlockNode *m_input = nullptr;
    };

    class AddControlOutputNode : public Node {
    public:
        AddControlOutputNode() { /* void */ }
        virtual ~AddControlOutputNode() { /* void */ }

    protected:
        virtual void registerInputs() override {
            addInput("program", &m_program, InputTarget::Type::Object);
            addInput("block", &m_block, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() override {
            readAllInputs();

            if (m_program != nullptr) m_program->addOutput(m_block);
        }

        ControlProgramNode *m_program = nullptr;
        ControlBlockNode *m_block = nullptr;
    };

    class SetControlProgramNode : public Node {
    public:
        SetControlProgramNode() { /* void */ }
        virtual ~SetControlProgramNode() { /* void */ }

    protected:
        virtual void registerInputs() override {
            addInput("program", &m_program, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() override {
            readAllInputs();

            if (m_program == nullptr) return;

            powertrain::ScriptedControlUnit *unit =
                new powertrain::ScriptedControlUnit;

            if (!m_program->generate(unit)) {
                delete unit;
                return;
            }

            delete Compiler::output()->controlProgram;
            Compiler::output()->controlProgram = unit;
        }

        ControlProgramNode *m_program = nullptr;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_POWERTRAIN_ACTIONS_H */
