#ifndef ATG_ENGINE_SIM_TRANSMISSION_NODE_H
#define ATG_ENGINE_SIM_TRANSMISSION_NODE_H

#include "object_reference_node.h"

#include "engine_sim.h"

#include <string>
#include <vector>

namespace es_script {

    class TransmissionNode : public ObjectReferenceNode<TransmissionNode> {
    public:
        TransmissionNode() { /* void */ }
        virtual ~TransmissionNode() { /* void */ }

        void generate(Transmission *transmission) const {
            Transmission::Parameters parameters = m_parameters;
            parameters.GearCount = static_cast<int>(m_gears.size());
            parameters.GearRatios = m_gears.data();
            parameters.GearboxType = resolveType(m_type);

            transmission->initialize(parameters);
        }

        static Transmission::Type resolveType(const std::string &name) {
            if (name == "manual" || name == "amt") return Transmission::Type::Manual;
            else if (name == "dct") return Transmission::Type::DualClutch;
            else if (name == "converter") return Transmission::Type::Converter;
            else return Transmission::Type::Legacy;
        }

        void addGear(double ratio) {
            m_gears.push_back(ratio);
        }

    protected:
        virtual void registerInputs() {
            addInput("max_clutch_torque", &m_parameters.MaxClutchTorque);
            addInput("type", &m_type);
            addInput("turbine_inertia", &m_parameters.TurbineInertia);
            addInput("stall_torque_ratio", &m_parameters.StallTorqueRatio);
            addInput("coupling_point", &m_parameters.CouplingPoint);
            addInput("capacity_factor", &m_parameters.CapacityFactor);
            addInput("reverse_ratio", &m_parameters.ReverseRatio);
            addInput("park_lock_torque", &m_parameters.ParkLockTorque);

            ObjectReferenceNode<TransmissionNode>::registerInputs();
        }

        virtual void _evaluate() {
            setOutput(this);

            // Read inputs
            readAllInputs();
        }

        Transmission::Parameters m_parameters;
        std::vector<double> m_gears;
        std::string m_type = "legacy";
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_TRANSMISSION_NODE_H */
