#ifndef ATG_ENGINE_SIM_FRICTION_NODE_H
#define ATG_ENGINE_SIM_FRICTION_NODE_H

#include "object_reference_node.h"

#include "../../include/engine_friction.h"
#include "../../include/combustion_chamber.h"

namespace es_script {

    class FrictionNode : public ObjectReferenceNode<FrictionNode> {
    public:
        FrictionNode() { /* void */ }
        virtual ~FrictionNode() { /* void */ }

        const EngineFriction::Parameters &getParameters() const {
            return m_parameters;
        }

        const CombustionChamber::FrictionModelParams &getCylinderParameters() const {
            return m_cylinder;
        }

    protected:
        virtual void registerInputs() override {
            addInput("constant_fmep", &m_parameters.constantFmep);
            addInput("peak_pressure_factor", &m_parameters.peakPressureFactor);
            addInput("speed_factor", &m_parameters.speedFactor);
            addInput("speed_squared_factor", &m_parameters.speedSquaredFactor);

            addInput("vogel_k", &m_parameters.vogelK);
            addInput("vogel_b", &m_parameters.vogelB);
            addInput("vogel_theta", &m_parameters.vogelTheta);
            addInput("reference_temperature", &m_parameters.referenceTemperature);

            addInput("peak_pressure_decay", &m_parameters.peakPressureDecay);
            addInput("heat_to_oil", &m_parameters.frictionHeatToOil);

            addInput("cylinder_friction", &m_cylinder.frictionCoeff);
            addInput("breakaway_friction", &m_cylinder.breakawayFriction);
            addInput("breakaway_velocity", &m_cylinder.breakawayFrictionVelocity);
            addInput("viscous_friction", &m_cylinder.viscousFrictionCoefficient);
            addInput("boundary_exponent", &m_cylinder.boundaryExponent);

            ObjectReferenceNode<FrictionNode>::registerInputs();
        }

        virtual void _evaluate() override {
            setOutput(this);
            readAllInputs();
        }

        EngineFriction::Parameters m_parameters;
        CombustionChamber::FrictionModelParams m_cylinder;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_FRICTION_NODE_H */
